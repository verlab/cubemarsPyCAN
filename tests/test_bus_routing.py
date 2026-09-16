"""Frame routing: containment, isolation between motors, and strict filtering."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field

import pytest

from cubemarspycan.bus import MotorBus
from cubemarspycan.codec.servo_can import ServoFunction, arbitration_id
from cubemarspycan.frame import Frame
from cubemarspycan.transport.can_bus import CanTransport


@dataclass
class Recorder:
    """A minimal endpoint that accepts standard 8-byte frames whose byte 0 is its id."""

    motor_id: int
    frames: list[Frame] = field(default_factory=list)

    def accepts(self, frame: Frame) -> bool:
        return not frame.is_extended_id and frame.dlc == 8 and frame.data[0] == self.motor_id

    def on_frame(self, frame: Frame, rx_monotonic: float) -> None:
        self.frames.append(frame)


@dataclass
class Exploding:
    """Filters like a real endpoint, then fails while handling."""

    motor_id: int = 1
    calls: int = 0

    def accepts(self, frame: Frame) -> bool:
        return not frame.is_extended_id and frame.dlc == 8 and frame.data[0] == self.motor_id

    def on_frame(self, frame: Frame, rx_monotonic: float) -> None:
        self.calls += 1
        raise RuntimeError("endpoint bug")


@pytest.fixture
def bus() -> Iterator[MotorBus]:
    transport = CanTransport.virtual("routing")
    motor_bus = MotorBus(transport)
    yield motor_bus
    motor_bus.close()


def mit_reply(motor_id: int) -> Frame:
    return Frame(0x00, bytes([motor_id, 0x80, 0, 0x80, 0, 0, 60, 0]))


def deliver(bus: MotorBus, frame: Frame) -> None:
    """Call the sink directly, the way the notifier thread would."""
    bus._on_frame(frame, 1.0, 1.0)


# --- registration -------------------------------------------------------------------


def test_register_and_unregister(bus: MotorBus) -> None:
    ep = Recorder(1)
    bus.register(ep)
    assert bus.endpoints == (ep,)
    bus.unregister(ep)
    assert len(bus.endpoints) == 0


def test_registering_twice_is_a_no_op(bus: MotorBus) -> None:
    ep = Recorder(1)
    bus.register(ep)
    bus.register(ep)
    assert len(bus.endpoints) == 1


def test_endpoints_are_published_as_an_immutable_tuple(bus: MotorBus) -> None:
    """The receive thread snapshots this with a single attribute read, so it must never
    be mutated in place."""
    bus.register(Recorder(1))
    assert isinstance(bus.endpoints, tuple)
    before = bus.endpoints
    bus.register(Recorder(2))
    assert bus.endpoints is not before, "a new tuple is published, not an append"


# --- isolation ----------------------------------------------------------------------


def test_two_motors_on_one_bus_never_cross_talk(bus: MotorBus) -> None:
    one, two = Recorder(1), Recorder(2)
    bus.register(one)
    bus.register(two)
    deliver(bus, mit_reply(1))
    deliver(bus, mit_reply(2))
    deliver(bus, mit_reply(1))
    assert len(one.frames) == 2
    assert len(two.frames) == 1


def test_an_endpoint_that_raises_is_contained_and_counted(bus: MotorBus) -> None:
    """One motor's bug must not silence the others."""
    boom = Exploding()
    good = Recorder(1)
    bus.register(boom)
    bus.register(good)
    deliver(bus, mit_reply(1))
    assert good.frames, "the healthy endpoint still received"
    assert bus.stats.endpoint_errors == 1
    assert "RuntimeError" in bus.stats.errors[0]


def test_an_endpoint_whose_accepts_raises_is_also_contained(bus: MotorBus) -> None:
    class BadPredicate:
        def accepts(self, frame: Frame) -> bool:
            raise ValueError("predicate bug")

        def on_frame(self, frame: Frame, rx_monotonic: float) -> None:
            pass

    good = Recorder(1)
    bus.register(BadPredicate())
    bus.register(good)
    deliver(bus, mit_reply(1))
    assert good.frames
    assert bus.stats.endpoint_errors == 1


def test_containment_survives_repeated_failures(bus: MotorBus) -> None:
    boom = Exploding()
    good = Recorder(1)
    bus.register(boom)
    bus.register(good)
    for _ in range(50):
        deliver(bus, mit_reply(1))
    assert len(good.frames) == 50
    assert bus.stats.endpoint_errors == 50


# --- filtering ----------------------------------------------------------------------


def test_foreign_frames_are_ignored_and_counted(bus: MotorBus) -> None:
    ep = Recorder(1)
    bus.register(ep)
    deliver(bus, mit_reply(9))  # another motor's reply
    deliver(bus, Frame(0x123, b"\x01\x02"))  # wrong length
    deliver(bus, Frame(0x2901, bytes(8), True))  # extended, not MIT
    assert not ep.frames
    assert bus.stats.rx_unmatched == 3
    assert bus.stats.rx_matched == 0


def test_unmatched_frames_are_sampled_for_diagnosis(bus: MotorBus) -> None:
    """This is what lets `cubemars scan` distinguish an empty bus from a wrong id."""
    bus.register(Recorder(1))
    deliver(bus, mit_reply(9))
    assert bus.stats.unmatched_samples
    assert "09" in bus.stats.unmatched_samples[0]


def test_unmatched_sample_buffer_is_bounded(bus: MotorBus) -> None:
    bus.register(Recorder(1))
    for _ in range(100):
        deliver(bus, mit_reply(9))
    assert len(bus.stats.unmatched_samples) == 16
    assert bus.stats.rx_unmatched == 100


def test_a_dlc_mismatch_is_rejected(bus: MotorBus) -> None:
    """TMotorCANControl checks only data[0], so any frame whose first byte collides is
    decoded as motor state."""
    ep = Recorder(1)
    bus.register(ep)
    deliver(bus, Frame(0x00, bytes([1, 2, 3])))
    assert not ep.frames


def test_extended_and_standard_frames_are_distinguished(bus: MotorBus) -> None:
    ep = Recorder(1)
    bus.register(ep)
    payload = bytes([1, 0x80, 0, 0x80, 0, 0, 60, 0])
    deliver(bus, Frame(0x00, payload))  # standard: MIT
    deliver(bus, Frame(arbitration_id(ServoFunction.STATUS, 1), payload, True))
    assert len(ep.frames) == 1, "the extended servo frame must not reach a MIT endpoint"


def test_matched_frames_are_counted_once_even_with_two_recipients(bus: MotorBus) -> None:
    bus.register(Recorder(1))
    bus.register(Recorder(1))
    deliver(bus, mit_reply(1))
    assert bus.stats.rx_matched == 1


# --- plumbing -----------------------------------------------------------------------


def test_send_goes_through_the_transport() -> None:
    import can

    listener = can.interface.Bus(channel="bus-send", interface="virtual")
    transport = CanTransport.open("virtual:bus-send")
    with MotorBus(transport) as motor_bus:
        motor_bus.send(Frame(0x321, b"\xaa"))
        msg = listener.recv(timeout=1.0)
    assert msg is not None and msg.arbitration_id == 0x321
    listener.shutdown()


def test_describe_reports_what_went_wrong(bus: MotorBus) -> None:
    bus.register(Exploding())
    bus.register(Recorder(1))
    deliver(bus, mit_reply(1))
    deliver(bus, mit_reply(9))
    text = bus.describe()
    assert "endpoints      : 2" in text
    assert "endpoint errors" in text
    assert "recent unmatched frames" in text


def test_context_manager_starts_and_closes() -> None:
    transport = CanTransport.virtual("ctx")
    with MotorBus(transport) as motor_bus:
        assert motor_bus.transport is transport
