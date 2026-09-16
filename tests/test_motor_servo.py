"""ServoMotor: setpoint exclusivity, capability refusal, and mode diagnostics."""

from __future__ import annotations

import time
from collections.abc import Iterator
from dataclasses import replace

import pytest

from cubemarspycan import (
    MotorError,
    OriginMode,
    SafetyPolicy,
    ServoModeNotConfirmed,
    ServoMotor,
    Source,
    Sourced,
    SpecIncompleteError,
    UnresolvedFrameError,
    get_spec,
    servo,
)
from cubemarspycan.codec import servo_can as codec
from cubemarspycan.errors import CapabilityError
from cubemarspycan.frame import Frame
from cubemarspycan.sim import SimServoDriver, SteppedSim, sim_bus

SPEC = get_spec("AK40-10")
DUAL = get_spec("AK80-8-KV60")
SETTLE = 0.05


class Rig:
    def __init__(self, motor: ServoMotor, sim: SteppedSim, driver: SimServoDriver) -> None:
        self.motor = motor
        self.sim = sim
        self.driver = driver

    def tx_count(self) -> int:
        self.sim.pump()
        return len(self.sim.received)


def make_rig(spec=SPEC, **kwargs: object) -> Iterator[Rig]:  # type: ignore[no-untyped-def]
    driver_kwargs = {k[7:]: v for k, v in kwargs.items() if k.startswith("driver_")}
    motor_kwargs = {k: v for k, v in kwargs.items() if not k.startswith("driver_")}
    driver_kwargs.setdefault("status_rate_hz", 200.0)
    driver = SimServoDriver(spec, motor_id=1, **driver_kwargs)  # type: ignore[arg-type]
    bus, sim = sim_bus(servo_drivers=[driver])
    motor = ServoMotor(bus, motor_id=1, spec=spec, **motor_kwargs)  # type: ignore[arg-type]
    try:
        yield Rig(motor, sim, driver)
    finally:
        bus.close()
        sim.close()


@pytest.fixture
def rig() -> Iterator[Rig]:
    yield from make_rig()


# --- setpoints ----------------------------------------------------------------------


@pytest.mark.parametrize(
    ("setpoint", "packet"),
    [
        (servo.Duty(0.05), codec.ServoPacket.SET_DUTY),
        (servo.Current(1.5), codec.ServoPacket.SET_CURRENT),
        (servo.CurrentBrake(1.0), codec.ServoPacket.SET_CURRENT_BRAKE),
        (servo.Rpm(5000), codec.ServoPacket.SET_RPM),
        (servo.Position(90.0), codec.ServoPacket.SET_POS),
        (servo.PositionSpeed(90.0, 5000, 30000), codec.ServoPacket.SET_POS_SPD),
    ],
)
def test_each_setpoint_maps_to_its_own_packet(
    rig: Rig, setpoint: servo.Setpoint, packet: codec.ServoPacket
) -> None:
    with rig.motor.control(wait_s=0.0):
        rig.motor.update(setpoint)
        rig.sim.advance(0.0)
    assert rig.driver.last_packet == packet


def test_setpoints_carry_their_values_to_the_wire(rig: Rig) -> None:
    with rig.motor.control(wait_s=0.0):
        rig.motor.update(servo.Position(90.0))
        rig.sim.advance(0.0)
        assert rig.driver.last_values[0] == pytest.approx(90.0)

        rig.motor.update(servo.PositionSpeed(45.0, 5000, 30000))
        rig.sim.advance(0.0)
        assert rig.driver.last_values == pytest.approx((45.0, 5000.0, 30000.0))


def test_a_staged_setpoint_is_resent_when_update_is_called_bare(rig: Rig) -> None:
    with rig.motor.control(wait_s=0.0):
        rig.motor.update(servo.Position(45.0))
        rig.sim.advance(0.0)
        rig.motor.update()
        rig.sim.advance(0.0)
    assert rig.driver.last_packet == codec.ServoPacket.SET_POS
    assert rig.driver.last_values[0] == pytest.approx(45.0)


def test_only_one_setpoint_is_in_flight(rig: Rig) -> None:
    """Exclusivity is in the type system: there is no way to hold two at once."""
    # command() mutates the staged setpoint, so read it into a local each time rather
    # than letting a type checker narrow the property across the call. That mypy narrows
    # it at all is the point: Position and Duty are disjoint types, so holding both is
    # not merely untested but unrepresentable.
    with rig.motor.control(wait_s=0.0):
        rig.motor.command(servo.Position(90.0))
        first: servo.Setpoint = rig.motor.staged_setpoint
        rig.motor.command(servo.Duty(0.1))
        second: servo.Setpoint = rig.motor.staged_setpoint
    assert isinstance(first, servo.Position)
    assert isinstance(second, servo.Duty)


def test_setpoints_are_frozen_value_objects() -> None:
    point = servo.Position(90.0)
    with pytest.raises(AttributeError):
        point.degrees = 1.0  # type: ignore[misc]
    assert servo.Position(90.0) == servo.Position(90.0)
    assert servo.Position(90.0) != servo.Position(45.0)


def test_setpoints_summarise_themselves() -> None:
    assert "90.00 deg" in servo.Position(90.0).summary
    assert "A" in servo.Current(1.5).summary
    assert "brake" in servo.CurrentBrake(1.0).summary
    assert "ERPM" in servo.Rpm(5000).summary
    assert "duty" in servo.Duty(0.05).summary
    assert "accel" in servo.PositionSpeed(1.0, 2.0, 3.0).summary


def test_stop_stages_zero_duty(rig: Rig) -> None:
    rig.motor.command(servo.Position(90.0))
    rig.motor.stop()
    assert rig.motor.staged_setpoint == servo.Duty(0.0)


# --- current guarding ---------------------------------------------------------------


def test_a_current_above_the_datasheet_peak_is_refused(rig: Rig) -> None:
    """The wire accepts +/-60 A; this motor peaks at 7.3."""
    with pytest.raises(MotorError, match=r"7\.30 A limit"):
        rig.motor.command(servo.Current(50.0))


def test_a_negative_current_is_guarded_by_magnitude(rig: Rig) -> None:
    with pytest.raises(MotorError, match="limit"):
        rig.motor.command(servo.Current(-50.0))


def test_a_brake_current_is_guarded_too(rig: Rig) -> None:
    with pytest.raises(MotorError, match="limit"):
        rig.motor.command(servo.CurrentBrake(30.0))


def test_a_current_within_the_rating_is_accepted(rig: Rig) -> None:
    rig.motor.command(servo.Current(5.0))
    assert rig.motor.staged_setpoint == servo.Current(5.0)


def test_a_refused_current_never_reaches_the_wire(rig: Rig) -> None:
    with rig.motor.control(wait_s=0.0):
        before = rig.tx_count()
        with pytest.raises(MotorError):
            rig.motor.update(servo.Current(50.0))
        assert rig.tx_count() == before


def test_a_spec_without_a_peak_current_demands_an_explicit_ceiling() -> None:
    """Refusing to guess. The alternative is a silent 8x-over-rating command."""
    bare = SPEC.evolve(limits=replace(SPEC.limits, peak_current_a=Sourced(None, Source.UNKNOWN)))
    gen = make_rig(spec=bare)
    rig = next(gen)
    try:
        with pytest.raises(MotorError, match="current_ceiling_a"):
            rig.motor.command(servo.Current(1.0))
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_an_explicit_ceiling_unblocks_an_unknown_spec() -> None:
    bare = SPEC.evolve(limits=replace(SPEC.limits, peak_current_a=Sourced(None, Source.UNKNOWN)))
    gen = make_rig(spec=bare, policy=SafetyPolicy(current_ceiling_a=3.0))
    rig = next(gen)
    try:
        rig.motor.command(servo.Current(2.0))
        with pytest.raises(MotorError, match=r"3\.00 A"):
            rig.motor.command(servo.Current(4.0))
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_the_policy_ceiling_tightens_the_datasheet_value() -> None:
    gen = make_rig(policy=SafetyPolicy(current_ceiling_a=2.0))
    rig = next(gen)
    try:
        with pytest.raises(MotorError, match=r"2\.00 A"):
            rig.motor.command(servo.Current(5.0))  # within 7.3 A, outside the policy
    finally:
        with pytest.raises(StopIteration):
            next(gen)


# --- origin -------------------------------------------------------------------------


def test_permanent_zero_is_refused_on_a_single_encoder_motor_and_sends_nothing(
    rig: Rig,
) -> None:
    """It writes flash, so refusing must happen before a frame is built."""
    before = rig.tx_count()
    with pytest.raises(CapabilityError, match="dual-encoder"):
        rig.motor.set_origin(OriginMode.PERMANENT)
    assert rig.tx_count() == before, "no frame may reach the wire"
    assert rig.driver.origin_calls == []
    assert rig.driver.rejected_origin_calls == 0, "the driver never even saw it"


def test_temporary_zero_is_the_default_and_is_sent(rig: Rig) -> None:
    rig.motor.set_origin()
    rig.sim.pump()
    assert rig.driver.origin_calls == [OriginMode.TEMPORARY]


def test_permanent_zero_is_allowed_on_a_dual_encoder_motor() -> None:
    gen = make_rig(spec=DUAL)
    rig = next(gen)
    try:
        rig.motor.set_origin(OriginMode.PERMANENT)
        rig.sim.pump()
        assert rig.driver.origin_calls == [OriginMode.PERMANENT]
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_policy_can_forbid_permanent_zero_even_where_the_hardware_allows_it() -> None:
    gen = make_rig(spec=DUAL, policy=SafetyPolicy(forbidden=frozenset({"permanent_zero"})))
    rig = next(gen)
    try:
        with pytest.raises(MotorError, match="forbidden"):
            rig.motor.set_origin(OriginMode.PERMANENT)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


# --- mode detection -----------------------------------------------------------------


def test_a_status_rate_of_zero_names_cubemarstool() -> None:
    """The commonest bench trap: wiring is fine, the driver simply never uploads."""
    gen = make_rig(driver_status_rate_hz=0.0, driver_ack_on_first_command=False)
    rig = next(gen)
    try:
        with (
            pytest.raises(ServoModeNotConfirmed, match="CubeMarsTool") as info,
            rig.motor.control(wait_s=0.05),
        ):
            pass
        assert "status rate" in str(info.value)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_frames_on_another_id_are_reported_as_such() -> None:
    gen = make_rig(driver_status_rate_hz=0.0, driver_ack_on_first_command=False)
    rig = next(gen)
    try:
        rig.sim.inject(Frame(codec.arbitration_id(codec.ServoFunction.STATUS, 9), bytes(8), True))
        time.sleep(SETTLE)
        with (
            pytest.raises(ServoModeNotConfirmed, match="not answering to id 1"),
            rig.motor.control(wait_s=0.05),
        ):
            pass
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_an_unknown_function_on_our_id_suggests_mit_mode() -> None:
    gen = make_rig(driver_status_rate_hz=0.0, driver_ack_on_first_command=False)
    rig = next(gen)
    try:
        rig.sim.inject(Frame(codec.arbitration_id(0x28, 1), bytes(8), True))
        time.sleep(SETTLE)
        with (
            pytest.raises(ServoModeNotConfirmed, match="MIT mode") as info,
            rig.motor.control(wait_s=0.05),
        ):
            pass
        assert "assume_mit" in str(info.value)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_a_status_frame_confirms_servo_mode(rig: Rig) -> None:
    """A driver in servo mode uploads unprompted, so status is already arriving."""
    rig.sim.inject(rig.driver.status_frame())
    time.sleep(SETTLE)
    with rig.motor.control(wait_s=0.5):
        pass  # must not raise


def test_assume_mit_sends_the_mit_exit_frame_on_entry() -> None:
    from cubemarspycan.codec import mit as mit_codec

    gen = make_rig(assume_mit=True)
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            pass
        rig.sim.pump()
        assert any(f.data == mit_codec.EXIT_MIT for f in rig.sim.received)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_by_default_nothing_is_sent_to_try_to_enter_servo_mode(rig: Rig) -> None:
    """The manual documents no such frame, so we do not invent one.

    TMotorCANControl sends the MIT FF..FC payload as an extended servo frame, where the
    control-mode bits make it a malformed duty-cycle command.
    """
    with rig.motor.control(wait_s=0.0):
        pass
    rig.sim.pump()
    assert all(f.data != bytes([0xFF] * 7 + [0xFC]) for f in rig.sim.received)


# --- replies ------------------------------------------------------------------------


def test_the_handshake_is_recorded_as_an_event_not_a_status(rig: Rig) -> None:
    with rig.motor.control(wait_s=0.0):
        rig.motor.update(servo.Duty(0.0))
        rig.sim.advance(0.0)
        time.sleep(SETTLE)
    assert rig.motor.servo_mode_acknowledged
    assert [e.kind for e in rig.motor.events] == ["servo_mode_ack"]


def test_a_bootloader_jump_is_flagged(rig: Rig) -> None:
    rig.sim.inject(rig.driver.bootloader_frame())
    time.sleep(SETTLE)
    assert rig.motor.bootloader_seen
    assert any(e.kind == "bootloader_jump" for e in rig.motor.events)


def test_status_reports_wire_units(rig: Rig) -> None:
    with rig.motor.control(wait_s=0.0):
        rig.motor.update(servo.Current(1.0))
        for _ in range(40):
            rig.sim.advance(0.005)
        time.sleep(SETTLE)
        status = rig.motor.update()
    assert status.seq > 0
    assert status.position_deg != 0.0
    assert status.current_a == pytest.approx(1.0, abs=0.05)
    assert status.temperature_c == 30


def test_a_malformed_reply_is_counted_not_raised(rig: Rig) -> None:
    rig.motor.on_frame(
        Frame(codec.arbitration_id(codec.ServoFunction.STATUS, 1), bytes(4), True), 1.0
    )
    assert rig.motor.decode_errors == 1
    assert rig.motor.status is None


# --- refusing to guess about the gearbox --------------------------------------------


def test_raw_degrees_are_always_available_but_output_radians_refuse(rig: Rig) -> None:
    """The manual never says which side of the gearbox servo position refers to."""
    with rig.motor.control(wait_s=0.0):
        rig.motor.update(servo.Current(1.0))
        for _ in range(20):
            rig.sim.advance(0.005)
        time.sleep(SETTLE)
        status = rig.motor.update()
    assert status.position_deg != 0.0
    with pytest.raises(UnresolvedFrameError, match="B7"):
        _ = status.output_rad


def test_output_radians_work_once_the_side_is_measured() -> None:
    from cubemarspycan.spec import Side

    measured = SPEC.evolve(
        servo=replace(SPEC.servo, position_side=Sourced(Side.OUTPUT, Source.MEASURED, "bench B7"))
    )
    gen = make_rig(spec=measured)
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            rig.motor.update(servo.Current(1.0))
            for _ in range(20):
                rig.sim.advance(0.005)
            time.sleep(SETTLE)
            status = rig.motor.update()
        assert status.output_rad == pytest.approx(status.position_deg * 3.141592653589793 / 180.0)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_erpm_conversion_refuses_on_an_incomplete_spec() -> None:
    gen = make_rig(spec=get_spec("AK70-10"), policy=SafetyPolicy(current_ceiling_a=1.0))
    rig = next(gen)
    try:
        with pytest.raises(SpecIncompleteError, match="pole pairs"):
            rig.motor.erpm_for(10.0)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_erpm_conversion_works_for_a_known_spec(rig: Rig) -> None:
    assert rig.motor.erpm_for(45.5) == pytest.approx(60832.0, rel=1e-3)


# --- diagnostics --------------------------------------------------------------------


def test_describe_flags_the_refusal_and_the_current_limit(rig: Rig) -> None:
    text = rig.motor.describe()
    assert "permanent zero REFUSED" in text
    assert "1 encoder(s)" in text
    assert "7.3 A (datasheet)" in text
    assert "command x10000" in text, "the 1e4 position scale, not the reference's 1e6"


def test_describe_says_when_no_current_limit_is_set() -> None:
    bare = SPEC.evolve(limits=replace(SPEC.limits, peak_current_a=Sourced(None, Source.UNKNOWN)))
    gen = make_rig(spec=bare)
    rig = next(gen)
    try:
        assert "UNSET" in rig.motor.describe()
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_repr(rig: Rig) -> None:
    assert "ServoMotor" in repr(rig.motor)
