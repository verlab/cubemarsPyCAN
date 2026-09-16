"""End-to-end through the real receive path, with no hardware.

Every test here drives frames across two python-can virtual buses, so the whole chain
runs for real: notifier thread, sink dispatch, routing, accepts, codec. Nothing is
monkeypatched. This is the seam that makes the rest of the library testable.
"""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass, field

import pytest

from cubemarspycan import get_spec
from cubemarspycan.codec import mit, servo_can
from cubemarspycan.codec.servo_can import OriginMode, ServoPacket
from cubemarspycan.frame import Frame
from cubemarspycan.sim import ScalingVariant, SimMitDriver, SimServoDriver, sim_bus
from cubemarspycan.unwrap import TurnCounter, WrapMode

SPEC = get_spec("AK40-10")
SETTLE = 0.05
"""The notifier thread is real, so give it a moment to drain before asserting."""


@dataclass
class MitSink:
    """Endpoint that decodes MIT replies for one motor."""

    motor_id: int = 1
    states: list[mit.MitFeedback] = field(default_factory=list)

    def accepts(self, frame: Frame) -> bool:
        return not frame.is_extended_id and frame.dlc == 8 and frame.data[0] == self.motor_id

    def on_frame(self, frame: Frame, rx_monotonic: float) -> None:
        self.states.append(mit.unpack_feedback(SPEC.mit, frame.data))


@dataclass
class ServoSink:
    motor_id: int = 1
    status: list[servo_can.ServoFeedback] = field(default_factory=list)
    events: list[servo_can.ServoEventFrame] = field(default_factory=list)

    def accepts(self, frame: Frame) -> bool:
        if not frame.is_extended_id:
            return False
        fn, mid = servo_can.split_arbitration_id(frame.arbitration_id)
        return mid == self.motor_id and fn in {0x09, 0x29, 0x2C}

    def on_frame(self, frame: Frame, rx_monotonic: float) -> None:
        out = servo_can.decode(frame, SPEC.servo)
        if isinstance(out, servo_can.ServoFeedback):
            self.status.append(out)
        else:
            self.events.append(out)


def command(bus, **kw: float) -> None:  # type: ignore[no-untyped-def]
    base = {"position_rad": 0.0, "velocity_radps": 0.0, "kp": 0.0, "kd": 0.0, "torque_nm": 0.0}
    bus.send(mit.command_frame(SPEC.mit, 1, **{**base, **kw}))


# --- MIT lifecycle ------------------------------------------------------------------


def test_commands_are_ignored_until_mit_mode_is_entered() -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        command(bus, position_rad=1.0, kp=20.0)
        sim.advance(0.01)
        assert driver.commands_ignored == 1
        assert driver.commands_seen == 0

        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        assert driver.in_mit_mode

        command(bus, position_rad=1.0, kp=20.0)
        sim.advance(0.01)
        assert driver.commands_seen == 1
    finally:
        bus.close()
        sim.close()


def test_exit_stops_the_motor_producing_torque() -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        command(bus, position_rad=1.0, kp=20.0, kd=0.5)
        for _ in range(20):
            sim.advance(0.005)
        moving = abs(driver.plant.velocity)
        assert moving > 0.0

        bus.send(mit.exit_mit_frame(1))
        sim.advance(0.0)
        assert not driver.in_mit_mode
        for _ in range(200):
            sim.advance(0.005)
        assert abs(driver.plant.velocity) < moving, "it coasts to a stop with no torque"
    finally:
        bus.close()
        sim.close()


def test_zero_frame_zeroes_the_reported_position() -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    driver.plant.position = 3.0
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.send(mit.zero_position_frame(1))
        sim.advance(0.0)
        assert driver.zero_count == 1
        assert driver.plant.position == 0.0
    finally:
        bus.close()
        sim.close()


# --- a closed control loop ----------------------------------------------------------


def test_position_step_converges_through_the_whole_stack() -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    sink = MitSink()
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.register(sink)
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        for _ in range(400):
            command(bus, position_rad=0.5, kp=20.0, kd=0.5)
            sim.advance(0.005)
        time.sleep(SETTLE)

        assert len(sink.states) > 100, "feedback arrived over the real notifier thread"
        final = sink.states[-1]
        assert final.position_rad == pytest.approx(0.5, abs=3 * SPEC.mit.position.lsb)
        assert abs(final.velocity_radps) < 0.1, "settled, not still moving"
        assert final.temperature_c == 30
        assert final.fault_code == 0
        assert bus.stats.endpoint_errors == 0
    finally:
        bus.close()
        sim.close()


def test_a_negative_setpoint_converges_too() -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    sink = MitSink()
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.register(sink)
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        for _ in range(400):
            command(bus, position_rad=-1.25, kp=20.0, kd=0.5)
            sim.advance(0.005)
        time.sleep(SETTLE)
        assert sink.states[-1].position_rad == pytest.approx(-1.25, abs=3 * SPEC.mit.position.lsb)
    finally:
        bus.close()
        sim.close()


@pytest.mark.parametrize("variant", list(ScalingVariant))
def test_commands_survive_either_firmware_scaling_convention(
    variant: ScalingVariant,
) -> None:
    """The manual's pack and unpack formulas are not exact inverses, so we cannot know
    which the firmware uses. Our encoder must be right under either reading."""
    driver = SimMitDriver(SPEC, motor_id=1, scaling=variant)
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        for target in (-12.0, -1.0, 0.0, 1.0, 12.0):
            command(bus, position_rad=target, kp=1.0)
            sim.advance(0.001)
            assert driver.last_command is not None
            assert driver.last_command[0] == pytest.approx(target, abs=2 * SPEC.mit.position.lsb)
    finally:
        bus.close()
        sim.close()


# --- wrap versus saturate -----------------------------------------------------------


def test_unwrapping_recovers_position_past_the_field_limit() -> None:
    driver = SimMitDriver(SPEC, motor_id=1, wrap_mode=WrapMode.WRAP)
    sink = MitSink()
    bus, sim = sim_bus(mit_drivers=[driver])
    counter = TurnCounter(SPEC.mit.position, WrapMode.WRAP)
    try:
        bus.register(sink)
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        driver.plant.position = 11.0
        for _ in range(60):
            command(bus, velocity_radps=10.0, kd=1.0)
            sim.advance(0.01)
        time.sleep(SETTLE)
        unwrapped = [counter.update(s.position_rad) for s in sink.states]
        assert counter.turns >= 1, "the sim wrapped at least once"
        assert unwrapped[-1] > 12.5, "and unwrapping carried past the field limit"
        assert all(b - a > -1.0 for a, b in itertools.pairwise(unwrapped)), (
            "the unwrapped series is monotonic, with no wrap discontinuity"
        )
    finally:
        bus.close()
        sim.close()


def test_saturating_firmware_pins_at_the_limit() -> None:
    driver = SimMitDriver(SPEC, motor_id=1, wrap_mode=WrapMode.SATURATE)
    sink = MitSink()
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.register(sink)
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        driver.plant.position = 11.0
        for _ in range(60):
            command(bus, velocity_radps=10.0, kd=1.0)
            sim.advance(0.01)
        time.sleep(SETTLE)
        assert driver.plant.position > 12.5, "the plant really did travel past the field"
        assert sink.states[-1].position_rad == pytest.approx(12.5, abs=SPEC.mit.position.lsb)
    finally:
        bus.close()
        sim.close()


# --- adversarial --------------------------------------------------------------------


def test_foreign_traffic_never_reaches_a_motor() -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    sink = MitSink(motor_id=1)
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.register(sink)
        sim.inject(Frame(0x00, bytes([9, 1, 2, 3, 4, 5, 6, 7])))  # another motor
        sim.inject(Frame(0x123, bytes([1, 2, 3])))  # short frame
        sim.inject(Frame(0x2801, bytes(8), True))  # unknown function
        sim.inject(Frame(0x00, bytes([1, 2, 3])))  # right id, wrong dlc
        time.sleep(SETTLE)
        assert not sink.states
        assert bus.stats.rx_unmatched == 4
    finally:
        bus.close()
        sim.close()


@pytest.mark.parametrize("code", [1, 6, 7, 8, 200])
def test_any_fault_byte_survives_the_receive_path(code: int) -> None:
    """Code 7 is where TMotorCANControl raises KeyError inside the notifier thread."""
    driver = SimMitDriver(SPEC, motor_id=1, fault_code=code)
    sink = MitSink()
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.register(sink)
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        time.sleep(SETTLE)
        assert sink.states[-1].fault_code == code
        assert bus.stats.endpoint_errors == 0
    finally:
        bus.close()
        sim.close()


@pytest.mark.parametrize("reply_id", [0x00, None])
def test_both_reply_arbitration_conventions_are_observable(reply_id: int | None) -> None:
    """The manual says "0x00 + Drive ID", which is ambiguous; the sim can do either."""
    driver = SimMitDriver(SPEC, motor_id=1, reply_arbitration_id=reply_id)
    seen: list[int] = []

    class AnyStandard:
        def accepts(self, frame: Frame) -> bool:
            return not frame.is_extended_id and frame.dlc == 8 and frame.data[0] == 1

        def on_frame(self, frame: Frame, rx_monotonic: float) -> None:
            seen.append(frame.arbitration_id)

    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.register(AnyStandard())
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        time.sleep(SETTLE)
        assert seen == [0x00 if reply_id == 0x00 else 1]
    finally:
        bus.close()
        sim.close()


def test_a_frozen_motor_simply_stops_replying() -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    sink = MitSink()
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.register(sink)
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        time.sleep(SETTLE)
        before = len(sink.states)
        assert before > 0

        sim.freeze()
        for _ in range(20):
            command(bus, position_rad=1.0, kp=10.0)
            sim.advance(0.005)
        time.sleep(SETTLE)
        assert len(sink.states) == before, "no further feedback while frozen"
    finally:
        bus.close()
        sim.close()


# --- servo mode ---------------------------------------------------------------------


def test_servo_status_arrives_at_the_configured_rate() -> None:
    driver = SimServoDriver(SPEC, motor_id=1, status_rate_hz=100.0)
    sink = ServoSink()
    bus, sim = sim_bus(servo_drivers=[driver])
    try:
        bus.register(sink)
        for _ in range(50):
            sim.advance(0.01)  # 0.5 s at 100 Hz
        time.sleep(SETTLE)
        assert 45 <= len(sink.status) <= 55
    finally:
        bus.close()
        sim.close()


def test_a_status_rate_of_zero_yields_nothing_at_all() -> None:
    """The real trap: CubeMarsTool can leave the CAN status rate at 0, and then nothing
    is wrong with the wiring but no frames ever arrive."""
    driver = SimServoDriver(SPEC, motor_id=1, status_rate_hz=0.0)
    sink = ServoSink()
    bus, sim = sim_bus(servo_drivers=[driver])
    try:
        bus.register(sink)
        for _ in range(100):
            sim.advance(0.01)
        time.sleep(SETTLE)
        assert not sink.status
        assert not sink.events
    finally:
        bus.close()
        sim.close()


def test_the_servo_handshake_is_an_event_not_a_position() -> None:
    driver = SimServoDriver(SPEC, motor_id=1, status_rate_hz=0.0)
    sink = ServoSink()
    bus, sim = sim_bus(servo_drivers=[driver])
    try:
        bus.register(sink)
        bus.send(servo_can.encode_duty(1, 0.05))
        sim.advance(0.0)
        time.sleep(SETTLE)
        assert [e.kind for e in sink.events] == ["servo_mode_ack"]
        assert not sink.status, "the handshake must never be decoded as state"
    finally:
        bus.close()
        sim.close()


def test_a_bootloader_frame_is_an_event() -> None:
    driver = SimServoDriver(SPEC, motor_id=1, status_rate_hz=0.0)
    sink = ServoSink()
    bus, sim = sim_bus(servo_drivers=[driver])
    try:
        bus.register(sink)
        sim.inject(driver.bootloader_frame())
        time.sleep(SETTLE)
        assert [e.kind for e in sink.events] == ["bootloader_jump"]
        assert not sink.status
    finally:
        bus.close()
        sim.close()


def test_servo_commands_arrive_with_the_values_we_sent() -> None:
    driver = SimServoDriver(SPEC, motor_id=1, status_rate_hz=0.0)
    bus, sim = sim_bus(servo_drivers=[driver])
    try:
        bus.send(servo_can.encode_position(1, 90.0))
        sim.advance(0.0)
        # Read the attribute into a fresh local each time: the driver mutates it out of
        # band, so narrowing it across statements would be wrong.
        seen: ServoPacket | None = driver.last_packet
        assert seen is ServoPacket.SET_POS
        assert driver.last_values[0] == pytest.approx(90.0)

        bus.send(servo_can.encode_position_speed(1, 45.0, 5000, 30000))
        sim.advance(0.0)
        seen = driver.last_packet
        assert seen is ServoPacket.SET_POS_SPD
        assert driver.last_values == pytest.approx((45.0, 5000.0, 30000.0))

        bus.send(servo_can.encode_current(1, 2.5))
        sim.advance(0.0)
        assert driver.last_values[0] == pytest.approx(2.5)
    finally:
        bus.close()
        sim.close()


def test_a_single_encoder_motor_rejects_permanent_zero_on_the_wire() -> None:
    """Validated end to end, not just at the library's own boundary."""
    driver = SimServoDriver(SPEC, motor_id=1, status_rate_hz=0.0)
    assert driver.reject_permanent, "AK40-10 has one encoder"
    bus, sim = sim_bus(servo_drivers=[driver])
    try:
        bus.send(servo_can.encode_origin(1, OriginMode.PERMANENT))
        sim.advance(0.0)
        assert driver.rejected_origin_calls == 1
        assert driver.origin_calls == []

        bus.send(servo_can.encode_origin(1, OriginMode.TEMPORARY))
        sim.advance(0.0)
        assert driver.origin_calls == [0]
    finally:
        bus.close()
        sim.close()


def test_a_dual_encoder_motor_accepts_permanent_zero() -> None:
    dual = get_spec("AK80-8-KV60")
    driver = SimServoDriver(dual, motor_id=1, status_rate_hz=0.0)
    assert not driver.reject_permanent
    bus, sim = sim_bus(servo_drivers=[driver])
    try:
        bus.send(servo_can.encode_origin(1, OriginMode.PERMANENT))
        sim.advance(0.0)
        assert driver.origin_calls == [1]
    finally:
        bus.close()
        sim.close()


def test_servo_status_reports_motion() -> None:
    driver = SimServoDriver(SPEC, motor_id=1, status_rate_hz=200.0)
    sink = ServoSink()
    bus, sim = sim_bus(servo_drivers=[driver])
    try:
        bus.register(sink)
        bus.send(servo_can.encode_current(1, 3.0))
        for _ in range(100):
            sim.advance(0.005)
        time.sleep(SETTLE)
        assert sink.status
        assert sink.status[-1].position_deg > 0.0, "current produced motion"
        assert sink.status[-1].velocity_erpm > 0.0
        assert sink.status[-1].current_a == pytest.approx(3.0, abs=0.05)
    finally:
        bus.close()
        sim.close()


def test_servo_and_mit_motors_coexist_on_one_bus() -> None:
    mit_driver = SimMitDriver(SPEC, motor_id=1)
    servo_driver = SimServoDriver(SPEC, motor_id=2, status_rate_hz=100.0)
    mit_sink, servo_sink = MitSink(motor_id=1), ServoSink(motor_id=2)
    bus, sim = sim_bus(mit_drivers=[mit_driver], servo_drivers=[servo_driver])
    try:
        bus.register(mit_sink)
        bus.register(servo_sink)
        bus.send(mit.enter_mit_frame(1))
        for _ in range(50):
            command(bus, position_rad=0.2, kp=20.0, kd=0.5)
            sim.advance(0.01)
        time.sleep(SETTLE)
        assert mit_sink.states and servo_sink.status
        assert all(s.motor_id == 1 for s in mit_sink.states)
        assert all(s.motor_id == 2 for s in servo_sink.status)
        assert bus.stats.endpoint_errors == 0
    finally:
        bus.close()
        sim.close()


def test_dropped_replies_do_not_corrupt_anything() -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    sink = MitSink()
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.register(sink)
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        sim.drop_every(3)
        for _ in range(60):
            command(bus, position_rad=0.3, kp=20.0, kd=0.5)
            sim.advance(0.005)
        time.sleep(SETTLE)
        assert sim.dropped > 0
        assert len(sink.states) < 61
        assert bus.stats.endpoint_errors == 0
        assert sink.states[-1].position_rad == pytest.approx(0.3, abs=0.05)
    finally:
        bus.close()
        sim.close()


def test_a_plant_with_hard_stops_pins_at_them() -> None:
    """Homing routines look for torque at the ceiling with velocity at zero."""
    driver = SimMitDriver(SPEC, motor_id=1)
    driver.plant.limit_hi = 0.5
    sink = MitSink()
    bus, sim = sim_bus(mit_drivers=[driver])
    try:
        bus.register(sink)
        bus.send(mit.enter_mit_frame(1))
        sim.advance(0.0)
        for _ in range(200):
            command(bus, velocity_radps=2.0, kd=1.0)
            sim.advance(0.005)
        time.sleep(SETTLE)
        assert driver.plant.position == pytest.approx(0.5, abs=1e-6)
        assert driver.plant.velocity <= 0.0, "the stop absorbs the motion"
        assert driver.plant.at_limit
        assert abs(sink.states[-1].torque_nm) > 0.1, "still pushing against it"
    finally:
        bus.close()
        sim.close()


def test_zeroing_carries_travel_limits_with_the_origin() -> None:
    from cubemarspycan.sim import Plant

    plant = Plant(position=1.0, limit_hi=1.5, limit_lo=-0.5)
    plant.zero_here()
    assert plant.position == 0.0
    assert plant.limit_hi == pytest.approx(0.5)
    assert plant.limit_lo == pytest.approx(-1.5)
