"""Servo-mode CAN codec. Pure: bytes in, bytes out.

Manual v1.0.18 pp.35-45. Servo frames are **extended**, and the arbitration id carries
the packet id above the motor id::

    arbitration_id = (packet_id << 8) | motor_id

Three details here are each a bug in TMotorCANControl:

* ``SET_POS`` scales degrees by **1e4**, not 1e6 (a 100x error).
* ``SET_POS_SPD`` divides speed *and* acceleration by 10 before packing them as int16.
* Replies come in three flavours and only ``0x29`` is state. ``0x2C`` is the
  "entered servo mode" handshake with a fixed ``FA FB FC FD`` payload and ``0x09`` is a
  bootloader jump; decoding either as position yields a plausible-looking lie.

Payload lengths differ per packet - 4 bytes for the scalar setpoints, 1 for origin,
8 for position-velocity - so DLC is not a constant.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

from ..errors import MalformedFrame
from ..frame import Frame
from ..spec import SERVO_CAN_COMMON, ServoScaling
from ..units import to_signed

MOTOR_ID_MASK = 0xFF
STATUS_DLC = 8
SERVO_MODE_ACK_PAYLOAD = bytes((0xFA, 0xFB, 0xFC, 0xFD))

_INT32_MIN, _INT32_MAX = -(2**31), 2**31 - 1
_INT16_MIN, _INT16_MAX = -(2**15), 2**15 - 1

# Wire limits from the manual, in raw counts.
_DUTY_LIMIT = 100_000  # +/-1.0 duty
_CURRENT_LIMIT = 60_000  # +/-60 A
_ERPM_LIMIT = 100_000
_POSITION_LIMIT = 360_000_000  # +/-36000 deg
_ACCEL_MAX = 32_767  # 1 unit = 10 electrical RPM/s^2


class ServoPacket(IntEnum):
    """Command packet ids, from the manual's ``CAN_PACKET_ID`` enum."""

    SET_DUTY = 0
    SET_CURRENT = 1
    SET_CURRENT_BRAKE = 2
    SET_RPM = 3
    SET_POS = 4
    SET_ORIGIN = 5
    SET_POS_SPD = 6
    SET_MIT = 8
    """Listed in the manual's enum with no payload or example. Deliberately has no
    encoder: a guessed payload on a packet id that exists is worse than none."""


class ServoFunction(IntEnum):
    """Function ids the driver replies with."""

    BOOTLOADER_JUMP = 0x09
    STATUS = 0x29
    SERVO_MODE_ACK = 0x2C


class OriginMode(IntEnum):
    """Argument to ``SET_ORIGIN``."""

    TEMPORARY = 0
    """Cleared on power loss. The safe default."""
    PERMANENT = 1
    """Writes flash. The manual restricts this to dual-encoder models."""


@dataclass(frozen=True, slots=True)
class ServoFeedback:
    """A decoded 0x29 status frame, in the wire's own units."""

    motor_id: int
    position_deg: float
    velocity_erpm: float
    current_a: float
    temperature_c: int
    fault_code: int


@dataclass(frozen=True, slots=True)
class ServoEventFrame:
    """A reply that is not state. The motor layer timestamps it."""

    kind: str
    function_id: int
    payload: bytes


# --- arbitration id -----------------------------------------------------------------


def arbitration_id(packet: ServoPacket | int, motor_id: int) -> int:
    """The extended arbitration id for a servo packet: ``(packet_id << 8) | motor_id``.

    Servo mode puts the command above the motor id in one 29-bit extended id, which is why
    servo framing is unambiguous where MIT's is not.
    """
    if not 0 <= motor_id <= MOTOR_ID_MASK:
        raise ValueError(f"servo motor id must be 0..255, got {motor_id}")
    return (int(packet) << 8) | motor_id


def split_arbitration_id(arb: int) -> tuple[int, int]:
    """Return ``(function_or_packet_id, motor_id)``."""
    return (arb >> 8) & 0xFF, arb & MOTOR_ID_MASK


def _clamp(value: int, lo: int, hi: int) -> int:
    return min(max(value, lo), hi)


def _i32(value: float, limit: int) -> bytes:
    return struct.pack(">i", _clamp(round(value), max(-limit, _INT32_MIN), min(limit, _INT32_MAX)))


def _frame(packet: ServoPacket, motor_id: int, payload: bytes) -> Frame:
    return Frame(arbitration_id(packet, motor_id), payload, is_extended_id=True)


# --- commands -----------------------------------------------------------------------


def encode_duty(motor_id: int, duty: float, scaling: ServoScaling = SERVO_CAN_COMMON) -> Frame:
    """Duty-cycle mode. ``duty`` is -1.0..1.0."""
    return _frame(ServoPacket.SET_DUTY, motor_id, _i32(duty * scaling.duty_scale, _DUTY_LIMIT))


def encode_current(motor_id: int, amps: float, scaling: ServoScaling = SERVO_CAN_COMMON) -> Frame:
    """Current-loop mode, i.e. torque control. ``amps`` is -60..60."""
    return _frame(
        ServoPacket.SET_CURRENT, motor_id, _i32(amps * scaling.current_scale, _CURRENT_LIMIT)
    )


def encode_current_brake(
    motor_id: int, amps: float, scaling: ServoScaling = SERVO_CAN_COMMON
) -> Frame:
    """Current-brake mode. Holds position with a braking current; 0..60 A, never negative."""
    raw = _clamp(round(amps * scaling.current_scale), 0, _CURRENT_LIMIT)
    return _frame(ServoPacket.SET_CURRENT_BRAKE, motor_id, struct.pack(">i", raw))


def encode_rpm(motor_id: int, erpm: float, scaling: ServoScaling = SERVO_CAN_COMMON) -> Frame:
    """Velocity mode. ``erpm`` is *electrical* RPM, -100000..100000."""
    return _frame(ServoPacket.SET_RPM, motor_id, _i32(erpm * scaling.rpm_scale, _ERPM_LIMIT))


def encode_position(
    motor_id: int, degrees: float, scaling: ServoScaling = SERVO_CAN_COMMON
) -> Frame:
    """Position mode. Degrees are scaled by 1e4 - the reference library uses 1e6."""
    return _frame(
        ServoPacket.SET_POS, motor_id, _i32(degrees * scaling.position_scale, _POSITION_LIMIT)
    )


def encode_origin(motor_id: int, mode: OriginMode = OriginMode.TEMPORARY) -> Frame:
    """Set the current position as origin. One payload byte.

    This codec does not police :attr:`OriginMode.PERMANENT`; the capability check lives
    on the spec, where it can name the motor and refuse before a frame is built.
    """
    return _frame(ServoPacket.SET_ORIGIN, motor_id, bytes((int(mode) & 0xFF,)))


def encode_position_speed(
    motor_id: int,
    degrees: float,
    speed_erpm: float,
    accel_erpm_s2: float,
    scaling: ServoScaling = SERVO_CAN_COMMON,
) -> Frame:
    """Position-velocity mode: a trapezoidal move to ``degrees``.

    Speed and acceleration are packed as int16 **after dividing by 10**, so one speed
    count is 10 ERPM and one acceleration count is 10 ERPM/s^2. The reference library
    omits both divisors. Acceleration is unsigned per the manual.
    """
    pos = _clamp(round(degrees * scaling.pos_spd_position_scale), -_POSITION_LIMIT, _POSITION_LIMIT)
    spd = _clamp(round(speed_erpm / scaling.pos_spd_speed_divisor), _INT16_MIN, _INT16_MAX)
    acc = _clamp(round(accel_erpm_s2 / scaling.pos_spd_accel_divisor), 0, _ACCEL_MAX)
    return _frame(ServoPacket.SET_POS_SPD, motor_id, struct.pack(">ihh", pos, spd, acc))


# --- replies ------------------------------------------------------------------------


def classify(frame: Frame) -> ServoFunction | None:
    """Identify a reply, or ``None`` if it is not one we recognise."""
    if not frame.is_extended_id:
        return None
    function_id, _ = split_arbitration_id(frame.arbitration_id)
    try:
        return ServoFunction(function_id)
    except ValueError:
        return None


def decode_status(data: bytes, scaling: ServoScaling = SERVO_CAN_COMMON) -> ServoFeedback:
    """Decode a 0x29 status payload.

    Uses explicit two's-complement arithmetic rather than NumPy. ``np.int16`` raises
    ``OverflowError`` on NumPy 2 for any value with the high bit set, which is what makes
    the reference library's receive path die on every negative position.
    """
    if len(data) != STATUS_DLC:
        raise MalformedFrame(f"servo status must be {STATUS_DLC} bytes, got {len(data)}")
    pos = to_signed((data[0] << 8) | data[1], 16)
    spd = to_signed((data[2] << 8) | data[3], 16)
    cur = to_signed((data[4] << 8) | data[5], 16)
    return ServoFeedback(
        motor_id=0,  # the status frame carries the id in the arbitration field, not the payload
        position_deg=pos * scaling.feedback_deg_per_lsb,
        velocity_erpm=spd * scaling.feedback_erpm_per_lsb,
        current_a=cur * scaling.feedback_amps_per_lsb,
        temperature_c=to_signed(data[6], 8),
        fault_code=data[7],
    )


def decode(
    frame: Frame, scaling: ServoScaling = SERVO_CAN_COMMON
) -> ServoFeedback | ServoEventFrame:
    """Decode any recognised servo reply.

    Returns :class:`ServoFeedback` only for ``0x29``. The handshake and bootloader frames
    come back as :class:`ServoEventFrame` so they can never be mistaken for a position.
    """
    function = classify(frame)
    if function is None:
        raise MalformedFrame(
            f"not a servo reply: {frame} "
            f"(expected an extended frame with function id 0x09, 0x29 or 0x2C)"
        )
    if function is ServoFunction.STATUS:
        feedback = decode_status(frame.data, scaling)
        _, motor_id = split_arbitration_id(frame.arbitration_id)
        return ServoFeedback(
            motor_id=motor_id,
            position_deg=feedback.position_deg,
            velocity_erpm=feedback.velocity_erpm,
            current_a=feedback.current_a,
            temperature_c=feedback.temperature_c,
            fault_code=feedback.fault_code,
        )
    kind = "servo_mode_ack" if function is ServoFunction.SERVO_MODE_ACK else "bootloader_jump"
    return ServoEventFrame(kind=kind, function_id=int(function), payload=frame.data)
