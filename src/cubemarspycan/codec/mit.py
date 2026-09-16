"""MIT-mode codec. Pure: bytes in, bytes out.

Manual v1.0.18 pp.60-68. Command and reply are both **standard** frames with DLC 8.

Command bit layout::

    D0 = p >> 8                      D1 = p & 0xFF
    D2 = v >> 4                      D3 = (v & 0xF) << 4 | kp >> 8
    D4 = kp & 0xFF                   D5 = kd >> 4
    D6 = (kd & 0xF) << 4 | t >> 8    D7 = t & 0xFF

Reply::

    D0        = driver id
    D1..D2    = position, 16 bit
    D3, D4hi  = velocity, 12 bit
    D4lo, D5  = torque,   12 bit
    D6        = temperature + 40
    D7        = fault code

No field can encode an exact zero: each is a symmetric range over an even-sized field, so
the midpoint sits half an LSB above zero. On an AK40-10 a commanded 0.0 N*m arrives as
+1.22 mN*m. Far below the 60 mN*m needed to break the output away, but worth knowing
before treating a zero command as an exact null.

Note that the special payloads below share the command encoding space: a saturated
command can land exactly on one of them. :func:`pack_command` detects that and steps one
torque LSB away, because the alternative is a silent mid-motion re-zero.

The third reply quantity is **torque**, not current - the manual's own ``unpack_reply``
names it so and scales it by the torque field. TMotorCANControl converts it to a
"q-axis current" through Kt, the gear ratio and an undocumented 0.59 factor, then
presents that as the primary reading. We return the torque the wire carries; deriving a
current from it is an explicitly-modelled step elsewhere.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..errors import MalformedFrame
from ..frame import Frame
from ..spec import MitFields

FEEDBACK_DLC = 8
COMMAND_DLC = 8

TEMPERATURE_OFFSET = 40
"""Reply byte 6 carries ``temperature + 40``, giving a -40..215 C range."""

ENTER_MIT = bytes((0xFF,) * 7 + (0xFC,))
EXIT_MIT = bytes((0xFF,) * 7 + (0xFD,))
ZERO_POSITION = bytes((0xFF,) * 7 + (0xFE,))

_SPECIAL_PAYLOADS = frozenset({ENTER_MIT, EXIT_MIT, ZERO_POSITION})


@dataclass(frozen=True, slots=True)
class MitFeedback:
    """One decoded MIT reply, in the wire's own terms."""

    motor_id: int
    position_rad: float
    velocity_radps: float
    torque_nm: float
    temperature_c: int
    fault_code: int


def pack_command(
    fields: MitFields,
    *,
    position_rad: float,
    velocity_radps: float,
    kp: float,
    kd: float,
    torque_nm: float,
) -> bytes:
    """Quantise and pack the five command fields into 8 bytes.

    Values outside a field's range are clamped: the wire physically cannot express them.
    Clamping against the motor's *physical* limits is a policy decision and happens a
    layer up, where it can be reported to the caller.
    """
    p = fields.position.to_uint(position_rad)
    v = fields.velocity.to_uint(velocity_radps)
    kp_i = fields.kp.to_uint(kp)
    kd_i = fields.kd.to_uint(kd)
    t = fields.torque.to_uint(torque_nm)

    packed = _lay_out(p, v, kp_i, kd_i, t)

    # A fully saturated command is byte-identical to a mode-control frame: with position,
    # velocity, Kp and Kd all at maximum, a torque of 4.99756 N*m on an AK40-10 packs to
    # FF FF FF FF FF FF FF FE, which the driver reads as "set current position to zero"
    # rather than as a torque. The neighbouring codes are ENTER and EXIT. A controller
    # that winds up against its limits can reach this band, and the failure is silent and
    # severe -- a mid-motion re-zero moves the position reference under the loop.
    # One LSB of torque (2.4 mN*m here) is far below anything the motor can resolve, so
    # stepping off the collision costs nothing and is always safe.
    # The three special codes are consecutive (0xFC, 0xFD, 0xFE), so a single step can
    # land on a neighbour; walk until clear. At most three iterations, and the band only
    # occurs when every other field is saturated.
    while packed in _SPECIAL_PAYLOADS:
        t = t - 1 if t > 0 else t + 1
        packed = _lay_out(p, v, kp_i, kd_i, t)
    return packed


def _lay_out(p: int, v: int, kp_i: int, kd_i: int, t: int) -> bytes:
    """The bit layout itself.

    The 0x0F masks on the high nibbles are defensive. With ((1<<bits)-1)/span scaling a
    12-bit value provably fits, but a mis-specified field must corrupt its own byte
    rather than silently bleeding into its neighbour's.
    """
    return bytes(
        (
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((kp_i >> 8) & 0x0F),
            kp_i & 0xFF,
            (kd_i >> 4) & 0xFF,
            ((kd_i & 0x0F) << 4) | ((t >> 8) & 0x0F),
            t & 0xFF,
        )
    )


def unpack_feedback(fields: MitFields, data: bytes) -> MitFeedback:
    """Decode an 8-byte MIT reply.

    Raises :class:`~cubemarspycan.errors.MalformedFrame` and nothing else, for any input.
    """
    if len(data) != FEEDBACK_DLC:
        raise MalformedFrame(f"MIT reply must be {FEEDBACK_DLC} bytes, got {len(data)}")
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0x0F) << 8) | data[5]
    return MitFeedback(
        motor_id=data[0],
        position_rad=fields.position.from_uint(p),
        velocity_radps=fields.velocity.from_uint(v),
        torque_nm=fields.torque.from_uint(t),
        temperature_c=data[6] - TEMPERATURE_OFFSET,
        fault_code=data[7],
    )


# --- frames -------------------------------------------------------------------------


def command_frame(
    fields: MitFields,
    motor_id: int,
    *,
    position_rad: float,
    velocity_radps: float,
    kp: float,
    kd: float,
    torque_nm: float,
) -> Frame:
    """A MIT command as a standard frame addressed to ``motor_id``."""
    return Frame(
        motor_id,
        pack_command(
            fields,
            position_rad=position_rad,
            velocity_radps=velocity_radps,
            kp=kp,
            kd=kd,
            torque_nm=torque_nm,
        ),
    )


def enter_mit_frame(motor_id: int) -> Frame:
    """Enter MIT control mode. Must be sent before any command is honoured."""
    return Frame(motor_id, ENTER_MIT)


def exit_mit_frame(motor_id: int) -> Frame:
    """Leave MIT control mode."""
    return Frame(motor_id, EXIT_MIT)


def zero_position_frame(motor_id: int) -> Frame:
    """Set the current position as zero.

    The manual does not say whether this persists across a power cycle, so callers should
    not assume either way.
    """
    return Frame(motor_id, ZERO_POSITION)


def is_special(data: bytes) -> bool:
    """True for the enter/exit/zero payloads, which are not commands."""
    return data in (ENTER_MIT, EXIT_MIT, ZERO_POSITION)
