"""Tests that need a real socketcan interface.

python-can's ``virtual`` backend is an in-process queue. It never touches a kernel socket,
so it exercises none of the code that only exists for socketcan: interface opening,
bitrate and controller-state read-back, kernel-level receive filters, and the
Frame <-> can.Message conversion as the kernel actually sees it.

These are skipped unless an interface is present. Create one with::

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0

Point them somewhere else with ``CUBEMARS_TEST_CHANNEL=socketcan:can0``.

A note on what vcan proves and what it does not: a virtual CAN interface has no bit
timing and no error state, so ``read_socketcan_bitrate`` and ``read_socketcan_state``
return ``None`` there. That is the *interesting* case - it is exactly the shape of the
gs_usb adapter whose missing ``/sys/class/net/can0/can_bittiming`` made ``doctor`` report
"no bitrate set" for a perfectly healthy 1 Mbit/s link.
"""

from __future__ import annotations

import os
import time
from collections.abc import Iterator

import pytest

from cubemarspycan import CanTransport, MitMotor, MotorBus, get_spec
from cubemarspycan.codec import mit
from cubemarspycan.frame import Frame
from cubemarspycan.transport import can_bus
from cubemarspycan.transport.can_bus import socketcan_is_up

SPEC = get_spec("AK40-10")
CHANNEL_URL = os.environ.get("CUBEMARS_TEST_CHANNEL", "socketcan:vcan0")
INTERFACE = CHANNEL_URL.split(":", 1)[1].split("@")[0]


pytestmark = [
    pytest.mark.socketcan,
    pytest.mark.skipif(
        socketcan_is_up(INTERFACE) is not True,
        reason=f"no socketcan interface {INTERFACE!r} is up "
        f"(sudo ip link add dev {INTERFACE} type vcan && sudo ip link set up {INTERFACE})",
    ),
]


@pytest.fixture
def transport() -> Iterator[CanTransport]:
    tp = CanTransport.open(CHANNEL_URL)
    tp.start()
    yield tp
    tp.close()


@pytest.fixture
def peer() -> Iterator[CanTransport]:
    """A second socket on the same interface, standing in for the motor."""
    tp = CanTransport.open(CHANNEL_URL)
    tp.start()
    yield tp
    tp.close()


# --- opening ------------------------------------------------------------------------


def test_open_socketcan_by_url() -> None:
    """Exercises _open_socketcan, which the virtual backend never reaches."""
    with CanTransport.open(CHANNEL_URL) as tp:
        assert INTERFACE in tp.channel_info


def test_opening_a_missing_interface_explains_the_fix() -> None:
    from cubemarspycan import TransportError

    with pytest.raises(TransportError, match="ip link set"):
        CanTransport.open("socketcan:definitely-not-here")


# --- link introspection -------------------------------------------------------------


def test_link_info_returns_something_for_a_real_interface() -> None:
    info = can_bus._link_info(INTERFACE)
    assert isinstance(info, dict)


def test_bitrate_readback_is_none_or_plausible() -> None:
    """A vcan interface has no bit timing; a real one reports its configured rate.

    Both must work without raising. Reporting ``None`` as "no bitrate" rather than
    crashing is the behaviour that was missing when a gs_usb adapter turned up with no
    ``can_bittiming`` directory in sysfs.
    """
    bitrate = can_bus.read_socketcan_bitrate(INTERFACE)
    assert bitrate is None or 10_000 <= bitrate <= 5_000_000


def test_controller_state_is_none_or_a_known_state() -> None:
    state = can_bus.read_socketcan_state(INTERFACE)
    assert state is None or state in {
        "ERROR-ACTIVE",
        "ERROR-WARNING",
        "ERROR-PASSIVE",
        "BUS-OFF",
        "STOPPED",
        "SLEEPING",
    }


# --- frames across a real kernel socket ---------------------------------------------


def test_a_standard_frame_survives_the_kernel(transport: CanTransport, peer: CanTransport) -> None:
    received: list[Frame] = []
    peer.add_sink(lambda f, rx, ts: received.append(f))
    transport.send(Frame(0x123, b"\x01\x02\x03\x04"))
    _wait_for(received)
    assert received[0].arbitration_id == 0x123
    assert received[0].data == b"\x01\x02\x03\x04"
    assert not received[0].is_extended_id


def test_an_extended_frame_keeps_its_flag(transport: CanTransport, peer: CanTransport) -> None:
    """Servo frames are extended; MIT frames are standard. Confusing them is a real bug."""
    received: list[Frame] = []
    peer.add_sink(lambda f, rx, ts: received.append(f))
    transport.send(Frame(0x2901, bytes(8), is_extended_id=True))
    _wait_for(received)
    assert received[0].is_extended_id
    assert received[0].arbitration_id == 0x2901


@pytest.mark.parametrize("length", [0, 1, 4, 8])
def test_every_payload_length_round_trips(
    transport: CanTransport, peer: CanTransport, length: int
) -> None:
    """DLC varies per servo packet: 4 for scalars, 1 for origin, 8 for position-velocity."""
    received: list[Frame] = []
    peer.add_sink(lambda f, rx, ts: received.append(f))
    transport.send(Frame(0x100, bytes(range(length))))
    _wait_for(received)
    assert received[0].dlc == length
    assert received[0].data == bytes(range(length))


def test_a_full_mit_command_round_trips(transport: CanTransport, peer: CanTransport) -> None:
    received: list[Frame] = []
    peer.add_sink(lambda f, rx, ts: received.append(f))
    command = mit.command_frame(
        SPEC.mit, 1, position_rad=1.0, velocity_radps=-2.0, kp=100.0, kd=1.0, torque_nm=0.5
    )
    transport.send(command)
    _wait_for(received)
    assert received[0].data == command.data
    assert received[0].data.hex(" ").upper() == "8A 3D 7A 63 33 33 38 CC"


# --- kernel filters -----------------------------------------------------------------


def test_kernel_filters_drop_unwanted_traffic(transport: CanTransport, peer: CanTransport) -> None:
    """socketcan applies these in the kernel; the virtual backend ignores them entirely.

    Opt-in in this library precisely because a wrong filter drops frames silently, which
    is a worse failure than a few wasted wakeups.
    """
    received: list[Frame] = []
    peer.add_sink(lambda f, rx, ts: received.append(f))
    peer.set_filters([{"can_id": 0x200, "can_mask": 0x7FF, "extended": False}])
    time.sleep(0.05)

    transport.send(Frame(0x111, b"\x00"))  # filtered out by the kernel
    transport.send(Frame(0x200, b"\x01"))  # allowed through
    time.sleep(0.2)

    assert [f.arbitration_id for f in received] == [0x200]
    peer.set_filters(None)


# --- the whole stack over a real socket ---------------------------------------------


def test_a_motor_loop_runs_over_a_real_socketcan_socket(
    transport: CanTransport, peer: CanTransport
) -> None:
    """The simulator on one kernel socket, the library on another."""
    from cubemarspycan.sim import SimMitDriver

    driver = SimMitDriver(SPEC, motor_id=1)
    replies: list[Frame] = []

    def respond(frame: Frame, rx: float, ts: float) -> None:
        reply = driver.handle(frame)
        if reply is not None:
            replies.append(reply)
            peer.send(reply)

    peer.add_sink(respond)
    bus = MotorBus(transport)
    motor = MitMotor(bus, motor_id=1, spec=SPEC, supply_voltage=24.0)

    with motor.control(wait_s=0.0):
        for _ in range(120):
            motor.update(position=0.3, velocity=0.0, kp=20.0, kd=0.5, torque=0.0)
            driver.step(0.005)
            time.sleep(0.002)
        time.sleep(0.1)
        state = motor.update()

    assert replies, "the simulated driver answered over the kernel socket"
    assert state.seq > 0, "feedback reached the latch through a real socket"
    assert state.fault_code == 0
    assert bus.stats.endpoint_errors == 0
    assert transport.stats.tx_failed == 0
    assert motor.decode_errors == 0


def _wait_for(items: list[Frame], timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if items:
            return
        time.sleep(0.005)
    raise AssertionError("no frame arrived over the socketcan interface within 1 s")
