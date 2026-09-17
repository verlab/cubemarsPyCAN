"""The user-facing MIT-mode motor."""

from __future__ import annotations

import warnings
from collections.abc import Iterator
from contextlib import contextmanager
from enum import Enum

from ..bus import MotorBus
from ..codec import mit as codec
from ..errors import MalformedFrame
from ..frame import Frame
from ..policy import DEFAULT_POLICY, ClampReport, SafetyPolicy
from ..spec import MotorSpec
from ..state import FaultEvent, MitState
from ..unwrap import TurnCounter, WrapMode
from .base import MotorEndpoint


class MitReplyMode(Enum):
    """Which arbitration id the driver answers on.

    The manual says "0X00+Drive ID", which is ambiguous, and firmware builds differ. So we
    learn it from the first plausible reply and then filter strictly on it, rather than
    matching on the payload's first byte alone the way TMotorCANControl does - which lets
    any 8-byte frame whose first byte collides be decoded as motor state.
    """

    AUTO = "auto"
    ARB_ZERO = "zero"
    ARB_MOTOR_ID = "motor_id"


class MitMotor(MotorEndpoint[MitState]):
    """An AK actuator in MIT (impedance) mode.

    Commands go out as a single frame carrying five coupled fields, so they are set
    together through :meth:`command` rather than one attribute at a time. Reading state
    and writing a setpoint are deliberately different operations: ``m.position = x``
    would write a setpoint while ``m.position`` read feedback, and the value you read
    back is never the value you wrote.
    """

    def __init__(
        self,
        bus: MotorBus,
        motor_id: int,
        spec: MotorSpec,
        *,
        policy: SafetyPolicy = DEFAULT_POLICY,
        supply_voltage: float | None = None,
        reply_mode: MitReplyMode = MitReplyMode.AUTO,
        wrap_mode: WrapMode | None = None,
    ) -> None:
        super().__init__(bus, motor_id, spec, policy=policy, supply_voltage=supply_voltage)
        self.reply_mode = reply_mode
        self._pinned_arb: int | None = {
            MitReplyMode.ARB_ZERO: 0x00,
            MitReplyMode.ARB_MOTOR_ID: motor_id,
        }.get(reply_mode)
        # Default to whatever the spec records, so a bench measurement enables
        # unwrapping everywhere without a caller repeating it.
        if wrap_mode is None:
            recorded = spec.mit_wrap_mode
            wrap_mode = recorded.value if recorded.known else WrapMode.UNKNOWN
        assert wrap_mode is not None
        self._turns = TurnCounter(spec.mit.position, wrap_mode)
        self._unwrap = wrap_mode is not WrapMode.UNKNOWN
        self._command = (0.0, 0.0, 0.0, 0.0, 0.0)
        self._decode_errors = 0
        self._warned_no_feedback = False

    # --- Endpoint -------------------------------------------------------------------

    def accepts(self, frame: Frame) -> bool:
        """Four conditions, not one. Learns the reply arbitration id on the first match."""
        if frame.is_extended_id or frame.dlc != codec.FEEDBACK_DLC:
            return False
        if frame.data[0] != self.motor_id:
            return False  # necessary, never sufficient
        if self._pinned_arb is not None:
            return frame.arbitration_id == self._pinned_arb
        if frame.arbitration_id in (0x00, self.motor_id):
            self._pinned_arb = frame.arbitration_id
            return True
        return False

    def on_frame(self, frame: Frame, rx_monotonic: float) -> None:
        """Runs on the receive thread. Must never raise."""
        try:
            feedback = codec.unpack_feedback(self.spec.mit, frame.data)
        except MalformedFrame:
            self._decode_errors += 1
            return
        _, _, seq = self._states.read()
        state = MitState(
            spec=self.spec,
            position_rad=feedback.position_rad,
            velocity_radps=feedback.velocity_radps,
            torque_nm=feedback.torque_nm,
            temperature_c=feedback.temperature_c,
            fault_code=feedback.fault_code,
            rx_monotonic=rx_monotonic,
            seq=seq + 1,
        )
        self._states.publish(state, rx_monotonic)
        if feedback.fault_code != 0:
            self._faults.set(
                FaultEvent.from_code(feedback.fault_code, "mit", rx_monotonic, seq + 1)
            )

    # --- lifecycle ------------------------------------------------------------------

    def _enter_frames(self) -> list[Frame]:
        return [codec.enter_mit_frame(self.motor_id)]

    def _exit_frames(self) -> list[Frame]:
        return [codec.exit_mit_frame(self.motor_id)]

    def _safe_stop_frame(self) -> Frame:
        """Zero gains and zero feed-forward, so the motor free-spins.

        "Zero" torque is not exactly zero on the wire: a symmetric range over an
        even-sized field has no exact centre, so 0.0 N*m encodes to half an LSB, which the
        firmware decodes as +1.22 mN*m on an AK40-10. That is 2% of the motor's 60 mN*m
        break-away torque, so it cannot move the output - but it is a floor, not a null,
        and a frictionless model will drift under it.
        """
        return codec.command_frame(
            self.spec.mit,
            self.motor_id,
            position_rad=0.0,
            velocity_radps=0.0,
            kp=0.0,
            kd=0.0,
            torque_nm=0.0,
        )

    @contextmanager
    def control(self, wait_s: float = 1.0) -> Iterator[MitMotor]:
        with super().control(wait_s):
            yield self

    # --- commands -------------------------------------------------------------------

    def command(
        self,
        *,
        position: float | None = None,
        velocity: float | None = None,
        kp: float | None = None,
        kd: float | None = None,
        torque: float | None = None,
    ) -> list[ClampReport]:
        """Stage the next command. Unspecified fields hold their previous value.

        Returns whatever had to be clamped, so a saturating controller is visible rather
        than silently trimmed.
        """
        prev = self._command
        wanted = (
            prev[0] if position is None else position,
            prev[1] if velocity is None else velocity,
            prev[2] if kp is None else kp,
            prev[3] if kd is None else kd,
            prev[4] if torque is None else torque,
        )
        fields = self.spec.mit
        limits = self.spec.limits
        applied: list[float] = []
        reports: list[ClampReport] = []
        for name, value, field, physical in (
            ("position", wanted[0], fields.position, None),
            ("velocity", wanted[1], fields.velocity, limits.no_load_speed_radps),
            ("kp", wanted[2], fields.kp, None),
            ("kd", wanted[3], fields.kd, None),
            ("torque", wanted[4], fields.torque, limits.peak_torque_nm),
        ):
            value_out, report = self._apply(name, value, field, physical)
            applied.append(value_out)
            if report is not None:
                reports.append(report)
        self._command = (applied[0], applied[1], applied[2], applied[3], applied[4])
        return reports

    def hold(self) -> None:
        """Free-spin: zero gains, zero feed-forward torque."""
        self._command = (0.0, 0.0, 0.0, 0.0, 0.0)

    def brake(self, kd: float = 1.0) -> None:
        """Damping only. Resists motion but does not seek a position."""
        self.command(position=0.0, velocity=0.0, kp=0.0, kd=kd, torque=0.0)

    # --- the loop -------------------------------------------------------------------

    def update(
        self,
        *,
        position: float | None = None,
        velocity: float | None = None,
        kp: float | None = None,
        kd: float | None = None,
        torque: float | None = None,
    ) -> MitState:
        """Snapshot the latest state, send the staged command, return the snapshot.

        The returned state is taken at the **top** of the call, so it predates the frame
        this call puts on the wire. One cycle of causality, stated rather than accidental.
        Calling with no arguments re-sends the staged command, which is what a
        fault-recovery path wants.
        """
        self._require_control()
        state, rx_monotonic, seq = self._snapshot()

        self._check_fault()
        self._check_staleness(rx_monotonic, seq)
        if state is not None:
            self._check_temperature(state)
            state = self._with_unwrapped_position(state)

        if any(v is not None for v in (position, velocity, kp, kd, torque)):
            self.command(position=position, velocity=velocity, kp=kp, kd=kd, torque=torque)
        self._send_command()

        if state is None:
            if not self._warned_no_feedback:
                self._warned_no_feedback = True
                warnings.warn(
                    f"{self.device_info()}: no feedback received yet, so update() is "
                    f"returning a placeholder whose seq is 0. Check `state.seq != 0` "
                    f"before acting on it, or enter control() with wait_s > 0.",
                    stacklevel=2,
                )
            return self._placeholder(rx_monotonic)
        return state

    def _send_command(self) -> None:
        p, v, kp, kd, t = self._command
        self.bus.send(
            codec.command_frame(
                self.spec.mit,
                self.motor_id,
                position_rad=p,
                velocity_radps=v,
                kp=kp,
                kd=kd,
                torque_nm=t,
            )
        )

    def _placeholder(self, rx_monotonic: float) -> MitState:
        return MitState(
            spec=self.spec,
            position_rad=0.0,
            velocity_radps=0.0,
            torque_nm=0.0,
            temperature_c=0,
            fault_code=0,
            rx_monotonic=rx_monotonic,
            seq=0,
        )

    def _check_temperature(self, state: MitState) -> None:
        if state.temperature_c > self.policy.max_temp_c:
            self._emergency_stop()
            from ..errors import MotorError

            raise MotorError(
                f"{self.device_info()} is at {state.temperature_c} C, above the "
                f"{self.policy.max_temp_c:g} C policy limit. A safe-stop frame has been "
                f"sent."
            )

    def _with_unwrapped_position(self, state: MitState) -> MitState:
        if not self._unwrap:
            return state
        from dataclasses import replace

        return replace(state, position_rad=self._turns.update(state.position_rad))

    # --- utilities ------------------------------------------------------------------

    def zero_here(self, *, grace_s: float = 1.5) -> None:
        """Set the current position as zero.

        The manual does not say whether this survives a power cycle, so do not rely on
        either behaviour.

        The driver needs about a second afterwards before position is trustworthy, and it
        may stop replying during that time. Because staleness is measured against the last
        frame *received*, transmitting through that gap does not keep it at bay - so this
        calls :meth:`~cubemarspycan.motor.base.MotorEndpoint.expect_silence` for
        ``grace_s`` and the wait becomes routine::

            m.zero_here()
            m.settle(1.5)

        ``grace_s=0`` restores the strict behaviour if you would rather see the gap.

        **The gains are dropped for you.** Zeroing moves the coordinate system, so a
        setpoint staged in the old frame would become an instruction to drive back to
        where the motor just came from. Staging zero gains is not enough on its own -
        :meth:`hold` only stages, it does not transmit - so the zeroed command is put on
        the wire *before* the origin moves.

        This **transmits** - a zeroed command frame, then the zero-position frame - so
        like :meth:`update` it requires control mode. Methods that only stage
        (:meth:`command`, :meth:`hold`, :meth:`brake`) do not: that is the line. Outside
        the ``with`` block the driver is not in MIT mode, so both frames would go to a
        device that is not listening and the caller would never learn.

        (:meth:`~cubemarspycan.motor.servo.ServoMotor.set_origin` is a deliberate
        exception: servo mode has no documented entry handshake, so "control mode" there
        is a library-side assertion rather than a device state, and the origin packet
        stands alone.)
        """
        self._require_control()
        self.hold()
        self._send_command()
        self.bus.send(codec.zero_position_frame(self.motor_id))
        self.expect_silence(grace_s)
        self._turns.reset()

    @property
    def state(self) -> MitState | None:
        """The most recent state, without sending anything."""
        return self._states.read()[0]

    @property
    def reply_arbitration_id(self) -> int | None:
        """Which id the driver actually answers on, once learned. Worth writing down."""
        return self._pinned_arb

    @property
    def staged_command(self) -> tuple[float, float, float, float, float]:
        return self._command

    @property
    def decode_errors(self) -> int:
        return self._decode_errors

    def describe(self) -> str:
        fields = self.spec.mit
        lines = [
            f"{self.device_info()}  (MIT mode, manual v{self.spec.manual_version})",
            f"  position : {fields.position}  effective +/-"
            f"{self._effective(fields.position, None):g} rad",
            f"  velocity : {fields.velocity}  effective +/-"
            f"{self._effective(fields.velocity, self.spec.limits.no_load_speed_radps):g} rad/s",
            f"  torque   : {fields.torque}  effective +/-"
            f"{self._effective(fields.torque, self.spec.limits.peak_torque_nm):g} Nm",
            f"  kp       : {fields.kp}",
            f"  kd       : {fields.kd}",
            f"  policy   : max {self.policy.max_temp_c:g} C, clamp "
            f"{self.policy.clamp.value}, on fault {self.policy.on_fault.value}",
            f"  unwrap   : {self._turns.mode.value}",
            "",
            self.spec.provenance_report(),
        ]
        if self.spec.notes:
            lines += ["", "Notes:"] + [f"  {ln}" for ln in self.spec.notes.splitlines()]
        return "\n".join(lines)

    def __repr__(self) -> str:
        return f"<MitMotor {self.device_info()}>"
