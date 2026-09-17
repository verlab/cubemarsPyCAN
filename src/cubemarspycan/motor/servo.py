"""The user-facing servo-mode motor."""

from __future__ import annotations

import time
from collections.abc import Iterator
from contextlib import contextmanager

from .. import servo as setpoints
from ..bus import MotorBus
from ..codec import mit as mit_codec
from ..codec import servo_can as codec
from ..codec.servo_can import OriginMode, ServoFunction
from ..errors import MalformedFrame, MotorError, ServoModeNotConfirmed
from ..frame import Frame
from ..policy import DEFAULT_POLICY, SafetyPolicy
from ..spec import MotorSpec
from ..state import FaultEvent, ServoEvent, ServoStatus
from .base import MotorEndpoint


class ServoMotor(MotorEndpoint[ServoStatus]):
    """An AK actuator in servo mode over CAN.

    Servo mode is configured in CubeMarsTool, not commanded over CAN: the manual documents
    ``0x2C`` as an *"entered servo mode"* **reply** but no frame that causes entry. So this
    class **detects** rather than asserts, and says what it observed when detection fails.
    TMotorCANControl guesses, sending the MIT ``FF..FC`` payload as an extended frame,
    where it lands as a malformed duty-cycle command.
    """

    def __init__(
        self,
        bus: MotorBus,
        motor_id: int,
        spec: MotorSpec,
        *,
        policy: SafetyPolicy = DEFAULT_POLICY,
        supply_voltage: float | None = None,
        assume_mit: bool = False,
    ) -> None:
        super().__init__(bus, motor_id, spec, policy=policy, supply_voltage=supply_voltage)
        self.assume_mit = assume_mit
        """Send the MIT exit frame on entry, for a driver left in MIT mode."""
        self._setpoint: setpoints.Setpoint = setpoints.Duty(0.0)
        self._servo_mode_ack = False
        self._bootloader_seen = False
        self._decode_errors = 0
        self._wrong_function_frames = 0
        self._events: list[ServoEvent] = []

    # --- Endpoint -------------------------------------------------------------------

    def accepts(self, frame: Frame) -> bool:
        if not frame.is_extended_id:
            return False
        function_id, motor_id = codec.split_arbitration_id(frame.arbitration_id)
        if motor_id != self.motor_id:
            return False
        if function_id in (
            ServoFunction.STATUS,
            ServoFunction.SERVO_MODE_ACK,
            ServoFunction.BOOTLOADER_JUMP,
        ):
            return True
        self._wrong_function_frames += 1
        return False

    def on_frame(self, frame: Frame, rx_monotonic: float) -> None:
        """Runs on the receive thread. Must never raise."""
        try:
            decoded = codec.decode(frame, self.spec.servo)
        except MalformedFrame:
            self._decode_errors += 1
            return

        if isinstance(decoded, codec.ServoEventFrame):
            self._events.append(
                ServoEvent(decoded.kind, decoded.function_id, decoded.payload, rx_monotonic)
            )
            if decoded.kind == "servo_mode_ack":
                self._servo_mode_ack = True
            else:
                self._bootloader_seen = True
            return

        _, _, seq = self._states.read()
        status = ServoStatus(
            spec=self.spec,
            position_deg=decoded.position_deg,
            velocity_erpm=decoded.velocity_erpm,
            current_a=decoded.current_a,
            temperature_c=decoded.temperature_c,
            fault_code=decoded.fault_code,
            rx_monotonic=rx_monotonic,
            seq=seq + 1,
        )
        self._states.publish(status, rx_monotonic)
        if decoded.fault_code != 0:
            self._faults.set(
                FaultEvent.from_code(decoded.fault_code, "servo", rx_monotonic, seq + 1)
            )

    # --- lifecycle ------------------------------------------------------------------

    def _enter_frames(self) -> list[Frame]:
        # There is no documented "enter servo mode" frame. The only thing we can usefully
        # send is the MIT exit, and only when the caller says the driver may be in MIT.
        return [mit_codec.exit_mit_frame(self.motor_id)] if self.assume_mit else []

    def _exit_frames(self) -> list[Frame]:
        return []

    def _safe_stop_frame(self) -> Frame:
        """Zero duty: no drive at all. The motor free-spins."""
        return codec.encode_duty(self.motor_id, 0.0, self.spec.servo)

    @contextmanager
    def control(self, wait_s: float = 1.0) -> Iterator[ServoMotor]:
        """Confirm the driver really is in servo mode, then hand control over."""
        with super().control(wait_s=0.0):
            if wait_s > 0.0:
                self._confirm_servo_mode(wait_s)
            yield self

    def _confirm_servo_mode(self, wait_s: float) -> None:
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if self._states.seq > 0 or self._servo_mode_ack:
                return
            time.sleep(0.001)
        raise ServoModeNotConfirmed(self._servo_mode_diagnosis(wait_s))

    def _servo_mode_diagnosis(self, wait_s: float) -> str:
        """Say which of the three usual causes fits what was actually observed."""
        head = (
            f"{self.device_info()}: no servo status (0x29) or mode acknowledgement "
            f"(0x2C) within {wait_s:g} s. "
        )
        if self.bus.stats.rx_matched == 0 and self.bus.stats.rx_unmatched == 0:
            return head + (
                "Nothing at all arrived on the bus. Either the CAN status rate is set to "
                "0 in CubeMarsTool - in which case the driver never uploads anything and "
                "the wiring is fine - or the motor is unpowered or off the bus. Check the "
                "status rate first; it is the more common cause."
            )
        if self._wrong_function_frames > 0:
            return head + (
                f"{self._wrong_function_frames} frame(s) arrived addressed to id "
                f"{self.motor_id} but with an unrecognised function id, which usually "
                f"means the driver is in MIT mode rather than servo mode. Switch it in "
                f"CubeMarsTool, or construct with assume_mit=True to send the MIT exit "
                f"frame on entry."
            )
        return head + (
            f"{self.bus.stats.rx_unmatched} frame(s) arrived but matched no motor, so "
            f"something is on the bus and not answering to id {self.motor_id}. Recent "
            f"frames: " + ", ".join(self.bus.stats.unmatched_samples)
        )

    # --- commands -------------------------------------------------------------------

    def command(self, setpoint: setpoints.Setpoint) -> None:
        """Stage a setpoint. Only one command mode is in flight at a time."""
        self._check_setpoint(setpoint)
        self._setpoint = setpoint

    def _check_setpoint(self, setpoint: setpoints.Setpoint) -> None:
        if isinstance(setpoint, setpoints.Current | setpoints.CurrentBrake):
            self._check_current(abs(setpoint.amps))

    def _check_current(self, amps: float) -> None:
        """Guard the current field, which is eight times wider than this motor.

        The wire accepts +/-60 A while an AK40-10 peaks at 7.3, so a misplaced decimal
        point asks for far more than the motor can survive.
        """
        peak = self.spec.limits.peak_current_a
        ceiling = self.policy.current_ceiling_a
        if ceiling is None and not peak.known:
            raise MotorError(
                f"{self.device_info()}: the servo current field spans +/-60 A but this "
                f"spec has no peak current, so there is nothing to check a command "
                f"against. Set SafetyPolicy(current_ceiling_a=...) explicitly, or record "
                f"the datasheet value on the spec."
            )
        limit = min(x for x in (ceiling, peak.value if peak.known else None) if x is not None)
        if amps > limit:
            raise MotorError(
                f"{self.device_info()}: {amps:.2f} A exceeds the {limit:.2f} A limit "
                f"for this motor. Raise SafetyPolicy.current_ceiling_a if you mean it."
            )

    def update(self, setpoint: setpoints.Setpoint | None = None) -> ServoStatus:
        """Snapshot the latest status, send the staged setpoint, return the snapshot.

        As in MIT mode, the returned status is taken at the top of the call and so
        predates the frame this call sends.
        """
        self._require_control()
        status, rx_monotonic, seq = self._snapshot()

        self._check_fault()
        self._check_staleness(rx_monotonic, seq)
        if status is not None and status.temperature_c > self.policy.max_temp_c:
            self._emergency_stop()
            raise MotorError(
                f"{self.device_info()} is at {status.temperature_c} C, above the "
                f"{self.policy.max_temp_c:g} C policy limit. A safe-stop frame has "
                f"been sent."
            )

        if setpoint is not None:
            self.command(setpoint)
        self.bus.send(self._setpoint.to_frame(self.motor_id, self.spec))

        if status is None:
            return self._placeholder(rx_monotonic)
        return status

    def _placeholder(self, rx_monotonic: float) -> ServoStatus:
        return ServoStatus(
            spec=self.spec,
            position_deg=0.0,
            velocity_erpm=0.0,
            current_a=0.0,
            temperature_c=0,
            fault_code=0,
            rx_monotonic=rx_monotonic,
            seq=0,
        )

    def stop(self) -> None:
        """Stage zero duty. The motor free-spins."""
        self._setpoint = setpoints.Duty(0.0)

    # --- origin ---------------------------------------------------------------------

    def set_origin(self, mode: OriginMode = OriginMode.TEMPORARY) -> None:
        """Set the current position as origin.

        :attr:`OriginMode.PERMANENT` writes flash and the manual restricts it to
        dual-encoder models. On a single-encoder motor such as the AK40-10 this raises
        :class:`~cubemarspycan.errors.CapabilityError` and **no frame is sent**.
        TMotorCANControl sends mode 1 unconditionally, and by default.
        """
        if mode is OriginMode.PERMANENT:
            self.spec.require_permanent_zero()
            if "permanent_zero" in self.policy.forbidden:
                raise MotorError(
                    f"{self.device_info()}: permanent zero is listed in SafetyPolicy.forbidden"
                )
        self.bus.send(codec.encode_origin(self.motor_id, mode))

    # --- conversions ----------------------------------------------------------------

    def erpm_for(self, output_radps: float) -> float:
        """Output rad/s to electrical RPM. Refuses if pole pairs or gear ratio are unknown."""
        return self.spec.radps_output_to_erpm(output_radps)

    # --- observation ----------------------------------------------------------------

    @property
    def status(self) -> ServoStatus | None:
        return self._states.read()[0]

    @property
    def servo_mode_acknowledged(self) -> bool:
        """True once a 0x2C handshake has been seen."""
        return self._servo_mode_ack

    @property
    def bootloader_seen(self) -> bool:
        """True if the driver announced a jump to its bootloader (function id 0x09)."""
        return self._bootloader_seen

    @property
    def events(self) -> list[ServoEvent]:
        """Non-status replies, in arrival order."""
        return list(self._events)

    @property
    def staged_setpoint(self) -> setpoints.Setpoint:
        return self._setpoint

    @property
    def decode_errors(self) -> int:
        return self._decode_errors

    def describe(self) -> str:
        scaling = self.spec.servo
        peak = self.spec.limits.peak_current_a
        current_limit = (
            f"{peak.value:g} A (datasheet)"
            if peak.known
            else f"{self.policy.current_ceiling_a} A (policy)"
            if self.policy.current_ceiling_a is not None
            else "UNSET - current commands will be refused"
        )
        lines = [
            f"{self.device_info()}  (servo mode, manual v{self.spec.manual_version})",
            f"  position : {scaling.feedback_deg_per_lsb} deg/LSB, command x"
            f"{scaling.position_scale:g}",
            f"  velocity : {scaling.feedback_erpm_per_lsb} ERPM/LSB (electrical)",
            f"  current  : {scaling.feedback_amps_per_lsb} A/LSB, limit {current_limit}",
            f"  origin   : permanent zero "
            f"{'supported' if self.spec.capabilities.permanent_zero else 'REFUSED'} "
            f"({self.spec.capabilities.encoders} encoder(s))",
            f"  policy   : max {self.policy.max_temp_c:g} C, on fault {self.policy.on_fault.value}",
            "",
            self.spec.provenance_report(),
        ]
        return "\n".join(lines)

    def __repr__(self) -> str:
        return f"<ServoMotor {self.device_info()}>"
