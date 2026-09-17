"""MitMotor: lifecycle, fault handling, staleness, clamping, unwrapping.

The most important assertion in this file is that a driver fault raises on the **calling**
thread, after a safe-stop frame is already on the wire. TMotorCANControl raises inside the
python-can notifier thread, where the exception is swallowed, your loop never learns, and
the motor keeps being commanded while faulted.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Iterator

import pytest

from cubemarspycan import (
    ClampMode,
    FaultAction,
    MitMotor,
    MitReplyMode,
    MotorFault,
    NotInControlMode,
    SafetyPolicy,
    StaleFeedbackError,
    WrapMode,
    get_spec,
)
from cubemarspycan.codec import mit as codec
from cubemarspycan.errors import MotorError
from cubemarspycan.frame import Frame
from cubemarspycan.sim import SimMitDriver, SteppedSim, sim_bus

SPEC = get_spec("AK40-10")
SETTLE = 0.05


class Rig:
    """A motor wired to a stepped simulator."""

    def __init__(self, motor: MitMotor, sim: SteppedSim, driver: SimMitDriver) -> None:
        self.motor = motor
        self.sim = sim
        self.driver = driver

    def run(self, iterations: int, dt: float = 0.005, **command: float) -> None:
        for _ in range(iterations):
            self.motor.update(**command)
            self.sim.advance(dt)

    def payloads(self) -> list[bytes]:
        return [f.data for f in self.sim.received]

    def tx_count(self) -> int:
        self.sim.pump()
        return len(self.sim.received)


def make_rig(**kwargs: object) -> Iterator[Rig]:
    driver_kwargs = {k[7:]: v for k, v in kwargs.items() if k.startswith("driver_")}
    motor_kwargs = {k: v for k, v in kwargs.items() if not k.startswith("driver_")}
    driver = SimMitDriver(SPEC, motor_id=1, **driver_kwargs)  # type: ignore[arg-type]
    bus, sim = sim_bus(mit_drivers=[driver])
    motor = MitMotor(bus, motor_id=1, spec=SPEC, **motor_kwargs)  # type: ignore[arg-type]
    try:
        yield Rig(motor, sim, driver)
    finally:
        bus.close()
        sim.close()


@pytest.fixture
def rig() -> Iterator[Rig]:
    yield from make_rig()


# --- lifecycle ----------------------------------------------------------------------


def test_commanding_outside_control_is_refused(rig: Rig) -> None:
    with pytest.raises(NotInControlMode, match="control"):
        rig.motor.update(position=1.0)


def test_control_sends_enter_then_safe_stop_then_exit(rig: Rig) -> None:
    with rig.motor.control(wait_s=0.0):
        rig.run(3, kp=20.0, position=0.5)
    rig.sim.pump()
    payloads = rig.payloads()
    assert payloads[0] == codec.ENTER_MIT
    assert payloads[-1] == codec.EXIT_MIT
    assert payloads[-2] == codec.pack_command(
        SPEC.mit, position_rad=0.0, velocity_radps=0.0, kp=0.0, kd=0.0, torque_nm=0.0
    ), "a zero-gain, zero-torque frame must precede the exit"


def test_the_exit_sequence_runs_even_when_the_body_raises(rig: Rig) -> None:
    """The failure mode that matters: an exception must not leave a motor driven."""
    with pytest.raises(ZeroDivisionError), rig.motor.control(wait_s=0.0):
        rig.run(2, kp=20.0, position=0.5)
        raise ZeroDivisionError
    rig.sim.pump()
    payloads = rig.payloads()
    assert payloads[-1] == codec.EXIT_MIT
    assert payloads[-2] == codec.pack_command(
        SPEC.mit, position_rad=0.0, velocity_radps=0.0, kp=0.0, kd=0.0, torque_nm=0.0
    )
    assert not rig.motor.in_control


def test_in_control_tracks_the_block(rig: Rig) -> None:
    states: list[bool] = [rig.motor.in_control]
    with rig.motor.control(wait_s=0.0):
        states.append(rig.motor.in_control)
    states.append(rig.motor.in_control)
    assert states == [False, True, False]


def test_waiting_for_feedback_reports_a_dead_bus() -> None:
    from cubemarspycan import CanTransport, MotorBus

    transport = CanTransport.virtual("dead-bus")
    bus = MotorBus(transport)
    motor = MitMotor(bus, motor_id=1, spec=SPEC)
    try:
        with (
            pytest.raises(StaleFeedbackError, match="Nothing at all was received"),
            motor.control(wait_s=0.05),
        ):
            pass
    finally:
        bus.close()


def test_waiting_for_feedback_distinguishes_a_wrong_id() -> None:
    driver = SimMitDriver(SPEC, motor_id=7)  # the motor answers on a different id
    bus, sim = sim_bus(mit_drivers=[driver])
    motor = MitMotor(bus, motor_id=1, spec=SPEC)
    try:
        sim.inject(Frame(0x00, bytes([7, 0x80, 0, 0x80, 0, 0, 60, 0])))
        time.sleep(SETTLE)
        with (
            pytest.raises(StaleFeedbackError, match="not answering to id 1"),
            motor.control(wait_s=0.05),
        ):
            pass
    finally:
        bus.close()
        sim.close()


# --- the update contract ------------------------------------------------------------


def test_update_returns_the_snapshot_taken_before_it_sent(rig: Rig) -> None:
    """One cycle of causality, stated rather than accidental."""
    with rig.motor.control(wait_s=0.0):
        rig.run(20, kp=20.0, position=0.5)
        time.sleep(SETTLE)
        before = rig.motor.state
        returned = rig.motor.update()
        assert before is not None
        assert returned.seq == before.seq
        assert returned.position_rad == before.position_rad


def test_update_with_no_arguments_resends_the_staged_command(rig: Rig) -> None:
    with rig.motor.control(wait_s=0.0):
        rig.motor.update(position=0.25, kp=10.0, kd=0.5)
        rig.sim.advance(0.005)
        staged = rig.motor.staged_command
        rig.motor.update()
        rig.sim.advance(0.005)
        assert rig.motor.staged_command == staged
        assert rig.payloads()[-1] == rig.payloads()[-2]


def test_unspecified_fields_hold_their_previous_value(rig: Rig) -> None:
    with rig.motor.control(wait_s=0.0):
        rig.motor.command(position=1.0, velocity=2.0, kp=30.0, kd=1.0, torque=0.5)
        rig.motor.command(position=2.0)
        assert rig.motor.staged_command == (2.0, 2.0, 30.0, 1.0, 0.5)


def test_hold_and_brake(rig: Rig) -> None:
    rig.motor.command(position=1.0, kp=50.0, torque=1.0)
    rig.motor.hold()
    assert rig.motor.staged_command == (0.0, 0.0, 0.0, 0.0, 0.0)
    rig.motor.brake(kd=2.0)
    assert rig.motor.staged_command == (0.0, 0.0, 0.0, 2.0, 0.0)


def test_a_position_step_converges(rig: Rig) -> None:
    with rig.motor.control(wait_s=0.0):
        rig.run(400, kp=20.0, kd=0.5, position=0.5)
        time.sleep(SETTLE)
        final = rig.motor.update()
    assert final.position_rad == pytest.approx(0.5, abs=3 * SPEC.mit.position.lsb)
    assert abs(final.velocity_radps) < 0.1


def test_zero_here_zeroes_the_motor(rig: Rig) -> None:
    with rig.motor.control(wait_s=0.0):
        rig.run(100, kp=20.0, kd=0.5, position=1.0)
        assert abs(rig.driver.plant.position) > 0.5
        rig.motor.zero_here()
        rig.sim.advance(0.0)
        assert rig.driver.zero_count == 1
        assert rig.driver.plant.position == 0.0


def test_zero_here_drops_the_gains_on_the_wire_before_moving_the_origin(rig: Rig) -> None:
    """Staging zero gains is not enough: hold() only stages, it does not transmit.

    If the origin moves while the last frame on the wire still carries kp=20 and a
    setpoint expressed in the old frame, the driver's inner loop - which runs far faster
    than CAN - drives back toward a target that has just been redefined underneath it.
    """
    with rig.motor.control(wait_s=0.0):
        rig.run(20, kp=20.0, kd=0.5, position=1.0)
        rig.motor.zero_here()
        rig.sim.advance(0.0)

    payloads = rig.payloads()
    zero_at = payloads.index(codec.ZERO_POSITION)
    assert payloads[zero_at - 1] == codec.pack_command(
        SPEC.mit, position_rad=0.0, velocity_radps=0.0, kp=0.0, kd=0.0, torque_nm=0.0
    ), "the zeroed command must be on the wire before the origin moves"


def test_zero_here_tolerates_the_driver_going_quiet(rig: Rig) -> None:
    """The driver may stop replying for about a second while it zeroes.

    Staleness is measured against the last frame *received*, so transmitting through the
    gap does not help - which is why a settle() that merely keeps sending still raised.
    zero_here() opens an explicit grace window instead.
    """
    with rig.motor.control(wait_s=0.0):
        rig.run(20, kp=20.0, kd=0.5, position=0.2)
        # Let the last reply land before the window opens. rx_monotonic is stamped
        # by the notifier thread, and advance() yields only ~300 us, so a busy
        # runner can stamp it after - which correctly declines the grace, and would
        # make this test flake rather than fail honestly.
        time.sleep(SETTLE)
        rig.motor.zero_here(grace_s=5.0)
        rig.sim.freeze()  # the driver answers nothing at all from here on
        time.sleep(rig.motor.policy.stale_fatal_s + 0.05)
        rig.motor.update()  # would raise StaleFeedbackError without the grace window


def test_the_grace_window_expires_rather_than_masking_a_dead_link(rig: Rig) -> None:
    """A grace window that never ended would be worse than the bug it fixes."""
    with rig.motor.control(wait_s=0.0):
        rig.run(20, kp=20.0, kd=0.5, position=0.2)
        # Let the last reply land before the window opens. rx_monotonic is stamped
        # by the notifier thread, and advance() yields only ~300 us, so a busy
        # runner can stamp it after - which correctly declines the grace, and would
        # make this test flake rather than fail honestly.
        time.sleep(SETTLE)
        rig.motor.zero_here(grace_s=0.05)
        rig.sim.freeze()
        time.sleep(rig.motor.policy.stale_fatal_s + 0.1)
        with pytest.raises(StaleFeedbackError):
            rig.motor.update()


def test_zero_here_outside_control_is_refused_and_sends_nothing(rig: Rig) -> None:
    """zero_here() transmits, so it takes the same guard as update().

    The rule: a public method that puts a frame on the wire requires control mode; one
    that only stages (command, hold, brake) does not. A caller who forgot the `with` gets
    NotInControlMode rather than two frames sent to a driver that was never put into MIT
    mode, and a zero that silently did not happen.
    """
    rig.motor.command(kp=20.0, position=0.5)
    before = rig.tx_count()

    with pytest.raises(NotInControlMode, match="control"):
        rig.motor.zero_here()

    assert rig.tx_count() == before, "no frame may reach the wire"
    assert rig.driver.zero_count == 0
    assert rig.motor.staged_command == (0.5, 0.0, 20.0, 0.0, 0.0), (
        "a refused call must not have staged a hold() either"
    )


def test_a_frame_after_zero_here_ends_the_grace_window_early(rig: Rig) -> None:
    """The window covers the driver's silence, not the whole of ``grace_s``.

    Once the driver has answered *after* the window opened, the link is demonstrably
    alive, so a gap from there on is real and must still be fatal on schedule. Without
    this, a motor that loses power one tick after zeroing goes unnoticed for the full
    grace_s - thirty seconds here.

    The trap this pins is the other way of writing the fix: clearing the window whenever
    feedback merely looks fresh closes it immediately, because at zero_here() the last
    frame is only ~10 ms old.
    """
    with rig.motor.control(wait_s=0.0):
        rig.run(20, kp=20.0, kd=0.5, position=0.2)
        time.sleep(SETTLE)
        rig.motor.zero_here(grace_s=30.0)  # far longer than stale_fatal_s

        rig.run(3)  # the driver answers again, well inside the window
        time.sleep(SETTLE)
        rig.motor.update()  # fresh feedback: must not raise

        rig.sim.freeze()  # now the link really does die
        time.sleep(rig.motor.policy.stale_fatal_s + 0.05)
        with pytest.raises(StaleFeedbackError):
            rig.motor.update()


# --- faults -------------------------------------------------------------------------


def test_a_fault_raises_on_the_calling_thread_after_a_safe_stop() -> None:
    """Both halves matter: the thread, and the ordering."""
    gen = make_rig(driver_fault_code=2)  # over-current
    rig = next(gen)
    caller = threading.get_ident()
    raised_on: list[int] = []
    try:
        with (
            pytest.raises(MotorFault, match="over-current") as info,
            rig.motor.control(wait_s=0.0),
        ):
            for _ in range(10):
                try:
                    rig.motor.update(kp=20.0, position=0.5)
                except MotorFault:
                    raised_on.append(threading.get_ident())
                    raise
                rig.sim.advance(0.005)
                time.sleep(0.005)
        assert raised_on == [caller], "the fault must surface on the control thread"
        assert "safe-stop" in str(info.value)
        rig.sim.pump()
        safe_stop = codec.pack_command(
            SPEC.mit, position_rad=0.0, velocity_radps=0.0, kp=0.0, kd=0.0, torque_nm=0.0
        )
        assert safe_stop in rig.payloads(), "a safe stop went out before the raise"
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_a_fault_is_latched_and_readable() -> None:
    gen = make_rig(driver_fault_code=7)  # motor stall, new in manual v1.0.18
    rig = next(gen)
    try:
        with pytest.raises(MotorFault, match="stall"), rig.motor.control(wait_s=0.0):
            for _ in range(10):
                rig.motor.update(kp=1.0)
                rig.sim.advance(0.005)
                time.sleep(0.005)
        latched = rig.motor.fault
        assert latched is not None and latched.code == 7
        assert rig.motor.faulted
        rig.motor.clear_fault()
        assert not rig.motor.faulted
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_a_fault_raises_once_not_on_every_call() -> None:
    gen = make_rig(driver_fault_code=3, policy=SafetyPolicy(on_fault=FaultAction.RAISE))
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            rig.sim.advance(0.0)
            time.sleep(SETTLE)
            with pytest.raises(MotorFault):
                rig.motor.update(kp=1.0)
            rig.motor.update(kp=1.0)  # must not raise again for the same fault
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_fault_action_warn_keeps_going() -> None:
    gen = make_rig(driver_fault_code=1, policy=SafetyPolicy(on_fault=FaultAction.WARN))
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            rig.sim.advance(0.0)
            time.sleep(SETTLE)
            with pytest.warns(UserWarning, match="over-temperature"):
                rig.motor.update(kp=1.0)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_fault_action_ignore_is_silent_but_still_latches() -> None:
    gen = make_rig(driver_fault_code=4, policy=SafetyPolicy(on_fault=FaultAction.IGNORE))
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            rig.sim.advance(0.0)
            time.sleep(SETTLE)
            rig.motor.update(kp=1.0)
        assert rig.motor.faulted
    finally:
        with pytest.raises(StopIteration):
            next(gen)


@pytest.mark.parametrize("code", [8, 200, 255])
def test_an_unknown_fault_code_still_raises_rather_than_crashing(code: int) -> None:
    gen = make_rig(driver_fault_code=code)
    rig = next(gen)
    try:
        with pytest.raises(MotorFault, match=str(code)), rig.motor.control(wait_s=0.0):
            for _ in range(10):
                rig.motor.update(kp=1.0)
                rig.sim.advance(0.005)
                time.sleep(0.005)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


# --- temperature and staleness ------------------------------------------------------


def test_over_temperature_stops_the_motor() -> None:
    gen = make_rig(driver_temperature_c=90, policy=SafetyPolicy(max_temp_c=75.0))
    rig = next(gen)
    try:
        with pytest.raises(MotorError, match="90 C"), rig.motor.control(wait_s=0.0):
            for _ in range(10):
                rig.motor.update(kp=1.0)
                rig.sim.advance(0.005)
                time.sleep(0.005)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_stale_feedback_warns_then_raises() -> None:
    """Thresholds are well clear of the harness's own settle time, so this is not a race."""
    gen = make_rig(policy=SafetyPolicy(stale_warn_s=0.15, stale_fatal_s=0.40))
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            rig.motor.update(kp=1.0)
            rig.sim.advance(0.0)
            rig.motor.update(kp=1.0)  # fresh feedback, no complaint

            rig.sim.freeze()
            time.sleep(0.20)
            with pytest.warns(UserWarning, match="old"):
                rig.motor.update(kp=1.0)

            time.sleep(0.25)
            with pytest.raises(StaleFeedbackError, match="safe-stop"):
                rig.motor.update(kp=1.0)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_fresh_feedback_never_warns() -> None:
    import warnings as w

    gen = make_rig(policy=SafetyPolicy(stale_warn_s=0.15, stale_fatal_s=0.40))
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            rig.motor.update(kp=1.0)
            rig.sim.advance(0.005)
            with w.catch_warnings():
                w.simplefilter("error")
                rig.motor.update(kp=1.0)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


# --- clamping -----------------------------------------------------------------------


def test_clamping_to_the_physical_limit_is_reported(rig: Rig) -> None:
    """The AK40-10's torque field reaches 5.0 Nm but the motor peaks at 4.1."""
    reports = rig.motor.command(torque=5.0)
    assert len(reports) == 1
    assert reports[0].field_name == "torque"
    assert reports[0].applied == pytest.approx(4.1)
    assert "datasheet" in reports[0].reason
    assert rig.motor.clamp_events == 1


def test_clamping_to_the_wire_field_is_reported(rig: Rig) -> None:
    reports = rig.motor.command(position=99.0)
    assert reports[0].applied == pytest.approx(12.5)
    assert "wire field" in reports[0].reason


def test_an_in_range_command_reports_nothing(rig: Rig) -> None:
    assert rig.motor.command(position=1.0, torque=1.0, kp=20.0) == []
    assert rig.motor.clamp_events == 0


def test_clamp_mode_field_allows_exceeding_the_motor_rating() -> None:
    gen = make_rig(policy=SafetyPolicy(clamp=ClampMode.FIELD))
    rig = next(gen)
    try:
        assert rig.motor.command(torque=5.0) == []
        assert rig.motor.staged_command[4] == pytest.approx(5.0)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_clamp_mode_raise_refuses_instead_of_trimming() -> None:
    gen = make_rig(policy=SafetyPolicy(clamp=ClampMode.RAISE))
    rig = next(gen)
    try:
        with pytest.raises(ValueError, match="outside the usable range"):
            rig.motor.command(torque=5.0)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_negative_commands_clamp_symmetrically(rig: Rig) -> None:
    reports = rig.motor.command(torque=-5.0)
    assert reports[0].applied == pytest.approx(-4.1)


# --- reply arbitration id -----------------------------------------------------------


@pytest.mark.parametrize(("sim_reply", "expected"), [(0x00, 0x00), (None, 1)])
def test_the_reply_arbitration_id_is_learned(sim_reply: int | None, expected: int) -> None:
    gen = make_rig(driver_reply_arbitration_id=sim_reply)
    rig = next(gen)
    try:
        assert rig.motor.reply_arbitration_id is None, "nothing assumed before a reply"
        with rig.motor.control(wait_s=0.0):
            rig.sim.advance(0.0)
            time.sleep(SETTLE)
        assert rig.motor.reply_arbitration_id == expected
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_pinning_the_reply_mode_rejects_the_other_convention() -> None:
    gen = make_rig(
        driver_reply_arbitration_id=None,  # the sim answers on the motor id
        reply_mode=MitReplyMode.ARB_ZERO,  # but we insist on 0x00
    )
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            rig.sim.advance(0.0)
            time.sleep(SETTLE)
        assert rig.motor.state is None, "frames on the wrong id must not be accepted"
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_a_colliding_first_byte_on_a_foreign_id_is_rejected(rig: Rig) -> None:
    """The exact hole in TMotorCANControl's filter."""
    rig.sim.inject(Frame(0x321, bytes([1, 0xFF, 0xFF, 0, 0, 0, 60, 0])))
    time.sleep(SETTLE)
    assert rig.motor.state is None
    assert rig.motor.reply_arbitration_id is None


def test_a_malformed_frame_is_counted_not_raised(rig: Rig) -> None:
    rig.motor._pinned_arb = 0x00
    rig.motor.on_frame(Frame(0x00, bytes(4)), 1.0)
    assert rig.motor.decode_errors == 1
    assert rig.motor.state is None


# --- unwrapping ---------------------------------------------------------------------


def test_unwrapping_is_off_for_a_spec_whose_behaviour_is_unmeasured() -> None:
    """Refusing to guess. Only a bench measurement turns unwrapping on."""
    driver = SimMitDriver(get_spec("AK70-10"), motor_id=1, wrap_mode=WrapMode.WRAP)
    bus, sim = sim_bus(mit_drivers=[driver])
    motor = MitMotor(bus, motor_id=1, spec=get_spec("AK70-10"))
    try:
        with motor.control(wait_s=0.0):
            driver.plant.position = 11.0
            for _ in range(60):
                motor.update(position=0.0, velocity=10.0, kp=0.0, kd=1.0, torque=0.0)
                sim.advance(0.01)
            time.sleep(SETTLE)
            state = motor.update()
        assert abs(state.position_rad) <= 12.5, "raw field reading, not unwrapped"
    finally:
        bus.close()
        sim.close()


def test_unwrapping_recovers_position_when_the_firmware_wraps() -> None:
    gen = make_rig(wrap_mode=WrapMode.WRAP, driver_wrap_mode=WrapMode.WRAP)
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            rig.driver.plant.position = 11.0
            rig.run(80, dt=0.01, velocity=10.0, kd=1.0)
            time.sleep(SETTLE)
            state = rig.motor.update()
        assert state.position_rad > 12.5
        assert state.position_rad == pytest.approx(rig.driver.plant.position, abs=0.5)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_saturating_firmware_reports_the_rail() -> None:
    gen = make_rig(wrap_mode=WrapMode.SATURATE, driver_wrap_mode=WrapMode.SATURATE)
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            rig.driver.plant.position = 11.0
            rig.run(80, dt=0.01, velocity=10.0, kd=1.0)
            time.sleep(SETTLE)
            state = rig.motor.update()
        assert rig.driver.plant.position > 12.5
        assert state.position_rad == pytest.approx(12.5, abs=SPEC.mit.position.lsb)
    finally:
        with pytest.raises(StopIteration):
            next(gen)


def test_zero_here_resets_the_turn_counter() -> None:
    gen = make_rig(wrap_mode=WrapMode.WRAP, driver_wrap_mode=WrapMode.WRAP)
    rig = next(gen)
    try:
        with rig.motor.control(wait_s=0.0):
            rig.driver.plant.position = 11.0
            rig.run(80, dt=0.01, velocity=10.0, kd=1.0)
            assert rig.motor._turns.turns >= 1
            rig.motor.zero_here()
            assert rig.motor._turns.turns == 0
    finally:
        with pytest.raises(StopIteration):
            next(gen)


# --- diagnostics --------------------------------------------------------------------


def test_describe_names_the_effective_limits(rig: Rig) -> None:
    text = rig.motor.describe()
    assert "AK40-10-KV170 id 1" in text
    assert "effective +/-4.1 Nm" in text, "the datasheet peak, not the 5.0 field"
    assert "datasheet" in text
    assert "Notes:" in text


def test_a_supply_above_the_saturation_voltage_warns() -> None:
    from cubemarspycan import CanTransport, MotorBus

    transport = CanTransport.virtual("supply-warn")
    bus = MotorBus(transport)
    try:
        with pytest.warns(UserWarning, match="faster than its velocity field"):
            MitMotor(bus, motor_id=1, spec=SPEC, supply_voltage=48.0)
    finally:
        bus.close()


def test_the_rated_supply_does_not_warn() -> None:
    import warnings as w

    from cubemarspycan import CanTransport, MotorBus

    transport = CanTransport.virtual("supply-quiet")
    bus = MotorBus(transport)
    try:
        with w.catch_warnings():
            w.simplefilter("error")
            MitMotor(bus, motor_id=1, spec=SPEC, supply_voltage=24.0)
    finally:
        bus.close()


def test_repr(rig: Rig) -> None:
    assert "MitMotor" in repr(rig.motor)
    assert "id 1" in repr(rig.motor)


def test_unwrapping_defaults_to_what_the_spec_measured(rig: Rig) -> None:
    """The AK40-10's wrap behaviour is a bench measurement, so unwrapping is on by
    default for it and no caller has to repeat the flag."""
    assert rig.motor._turns.mode is WrapMode.WRAP
    assert rig.motor._unwrap


def test_an_unmeasured_spec_still_refuses_to_unwrap() -> None:
    from cubemarspycan import CanTransport, MotorBus

    transport = CanTransport.virtual("unwrap-default")
    bus = MotorBus(transport)
    try:
        motor = MitMotor(bus, motor_id=1, spec=get_spec("AK70-10"))
        assert motor._turns.mode is WrapMode.UNKNOWN
        assert not motor._unwrap
    finally:
        bus.close()


def test_an_explicit_wrap_mode_overrides_the_spec() -> None:
    from cubemarspycan import CanTransport, MotorBus

    transport = CanTransport.virtual("unwrap-override")
    bus = MotorBus(transport)
    try:
        motor = MitMotor(bus, motor_id=1, spec=SPEC, wrap_mode=WrapMode.SATURATE)
        assert motor._turns.mode is WrapMode.SATURATE
    finally:
        bus.close()
