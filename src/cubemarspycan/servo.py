"""Servo-mode setpoints, as value objects.

Servo mode has six mutually exclusive commands: a duty cycle, a current, a braking
current, a speed, a position, and a position with a speed and acceleration profile. Only
one can be in flight, and there is no meaningful way to combine them.

Making each a distinct type puts that exclusivity in the type system rather than in a
mode enum the caller has to keep in step. ``m.update(servo.Position(90.0))`` says exactly
one thing; ``m.position = 90; m.current = 2.0`` - which is how TMotorCANControl models
it - says two contradictory things and silently resolves them by whichever mode flag was
set last. For example::

    m.update(servo.Duty(0.05))
    m.update(servo.Current(1.5))
    m.update(servo.Position(90.0))
    m.update(servo.PositionSpeed(90.0, speed_erpm=5000, accel_erpm_s2=30000))
"""

from __future__ import annotations

from dataclasses import dataclass

from .codec import servo_can as codec
from .frame import Frame
from .spec import MotorSpec


@dataclass(frozen=True, slots=True)
class Setpoint:
    """Base for the six servo commands."""

    def to_frame(self, motor_id: int, spec: MotorSpec) -> Frame:  # pragma: no cover
        """Encode this setpoint as one extended CAN frame for ``motor_id``.

        The shared contract, which every subclass keeps: pure, allocates exactly one
        :class:`~cubemarspycan.frame.Frame`, and performs **no** safety clamping - the
        wire fields are far wider than any motor (the current field is +/-60 A against the
        AK40-10's 7.3 A peak), so limits belong to
        :class:`~cubemarspycan.policy.SafetyPolicy`, not here. Raises
        :class:`~cubemarspycan.errors.SpecIncompleteError` if the scaling needed for this
        command is unknown for ``spec``.

        Each subclass documents only what differs: its packet id and its scale factor.
        """
        raise NotImplementedError

    @property
    def summary(self) -> str:  # pragma: no cover
        """A short human-readable form, for logs and CLI output.

        Value and unit only - never the motor id, and never enough to reconstruct the
        frame. Each subclass states its own format.
        """
        raise NotImplementedError


@dataclass(frozen=True, slots=True)
class Duty(Setpoint):
    """Open-loop duty cycle, -1.0 to 1.0. Square-wave-like drive; no closed loop at all."""

    value: float

    def to_frame(self, motor_id: int, spec: MotorSpec) -> Frame:
        """Packet ``0``, ``SET_DUTY``. Duty times the spec's duty scale, as int32."""
        return codec.encode_duty(motor_id, self.value, spec.servo)

    @property
    def summary(self) -> str:
        """``duty +0.050``, three decimals and always signed."""
        return f"duty {self.value:+.3f}"


@dataclass(frozen=True, slots=True)
class Current(Setpoint):
    """q-axis current in amps. Output torque is roughly ``amps * Kt * gear_ratio``."""

    amps: float

    def to_frame(self, motor_id: int, spec: MotorSpec) -> Frame:
        """Packet ``1``, ``SET_CURRENT``. Amps times the spec's current scale, int32."""
        return codec.encode_current(motor_id, self.amps, spec.servo)

    @property
    def summary(self) -> str:
        """``+1.50 A``, two decimals and always signed."""
        return f"{self.amps:+.2f} A"


@dataclass(frozen=True, slots=True)
class CurrentBrake(Setpoint):
    """Braking current in amps, never negative. Holds position; watch the temperature."""

    amps: float

    def to_frame(self, motor_id: int, spec: MotorSpec) -> Frame:
        """Packet ``2``, ``SET_CURRENT_BRAKE``. Same scaling as :class:`Current`.

        A negative value has no meaning here - braking current is a magnitude.
        """
        return codec.encode_current_brake(motor_id, self.amps, spec.servo)

    @property
    def summary(self) -> str:
        """``brake 0.50 A``. Unsigned, because the value is a magnitude."""
        return f"brake {self.amps:.2f} A"


@dataclass(frozen=True, slots=True)
class Rpm(Setpoint):
    """Speed in **electrical** RPM, not mechanical.

    Use :meth:`~cubemarspycan.motor.servo.ServoMotor.erpm_for` to convert from output
    rad/s, which needs the pole pair count and gear ratio and refuses if either is
    unknown.
    """

    erpm: float

    def to_frame(self, motor_id: int, spec: MotorSpec) -> Frame:
        """Packet ``3``, ``SET_RPM``. **Electrical** RPM times the spec's rpm scale."""
        return codec.encode_rpm(motor_id, self.erpm, spec.servo)

    @property
    def summary(self) -> str:
        """``+3000 ERPM``, no decimals. Electrical, not mechanical."""
        return f"{self.erpm:+.0f} ERPM"


@dataclass(frozen=True, slots=True)
class Position(Setpoint):
    """Target angle in degrees. The driver moves there at its configured maximum speed."""

    degrees: float

    def to_frame(self, motor_id: int, spec: MotorSpec) -> Frame:
        """Packet ``4``, ``SET_POS``. Degrees scaled by **1e4**, as int32.

        1e4, not 1e6. TMotorCANControl uses 1e6, which is a 100x error: a commanded 90
        degrees arrives as 0.9.
        """
        return codec.encode_position(motor_id, self.degrees, spec.servo)

    @property
    def summary(self) -> str:
        """``+90.00 deg``, two decimals and always signed."""
        return f"{self.degrees:+.2f} deg"


@dataclass(frozen=True, slots=True)
class PositionSpeed(Setpoint):
    """Trapezoidal move: a target angle with speed and acceleration limits.

    ``speed_erpm`` and ``accel_erpm_s2`` are packed as int16 after dividing by 10, so
    their resolution is 10 ERPM and 10 ERPM/s^2 respectively.
    """

    degrees: float
    speed_erpm: float
    accel_erpm_s2: float

    def to_frame(self, motor_id: int, spec: MotorSpec) -> Frame:
        """Packet ``6``, ``SET_POS_SPD``. Degrees by 1e4; speed and accel each **/10**.

        The divide-by-ten applies to *both* limits before they are packed as int16, so the
        effective resolution is 10 ERPM and 10 ERPM/s^2. Missing it on either field is a
        10x error in the move profile.
        """
        return codec.encode_position_speed(
            motor_id, self.degrees, self.speed_erpm, self.accel_erpm_s2, spec.servo
        )

    @property
    def summary(self) -> str:
        """``+90.00 deg at 5000 ERPM, accel 30000`` - the target and both limits."""
        return (
            f"{self.degrees:+.2f} deg at {self.speed_erpm:.0f} ERPM, accel {self.accel_erpm_s2:.0f}"
        )


__all__ = [
    "Current",
    "CurrentBrake",
    "Duty",
    "Position",
    "PositionSpeed",
    "Rpm",
    "Setpoint",
]
