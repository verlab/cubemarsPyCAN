"""A fake AK driver speaking MIT mode.

Decodes commands with its own hand-written bit arithmetic rather than calling the library
codec, so a shared bug cannot cancel itself out.

The knobs exist to test things the manual leaves open, and each one corresponds to a
question the bench sequence answers:

* :class:`ScalingVariant` - the manual's pack and unpack formulas are not exact inverses,
  so we cannot know which the firmware uses. Our encoder must be within 1 LSB of either.
* :class:`~cubemarspycan.spec.WrapMode` - whether position wraps or saturates past the field limit.
* ``reply_arbitration_id`` - the manual says "0x00 + Drive ID", which is ambiguous.
"""

from __future__ import annotations

from enum import Enum

from ..codec.mit import ENTER_MIT, EXIT_MIT, TEMPERATURE_OFFSET, ZERO_POSITION
from ..frame import Frame
from ..spec import FieldRange, MotorSpec
from ..unwrap import WrapMode
from .plant import FIRMWARE_LOOP_DT, Plant


class ScalingVariant(Enum):
    """Which float<->uint convention the fake firmware uses."""

    EXACT = "exact"
    """``span / ((1<<bits) - 1)``, as the manual's ``uint_to_float`` documents."""
    TRUNCATED = "truncated"
    """``span / (1<<bits)``, the inverse of the manual's ``float_to_uint``."""


def _decode(u: int, field: FieldRange, variant: ScalingVariant) -> float:
    divisor = field.max_uint if variant is ScalingVariant.EXACT else (1 << field.bits)
    return u * field.span / divisor + field.lo


def _encode(x: float, field: FieldRange, variant: ScalingVariant) -> int:
    divisor = field.max_uint if variant is ScalingVariant.EXACT else (1 << field.bits)
    x = min(max(x, field.lo), field.hi)
    return min(max(round((x - field.lo) * divisor / field.span), 0), field.max_uint)


class SimMitDriver:
    """A driver that ignores everything until it is told to enter MIT mode."""

    def __init__(
        self,
        spec: MotorSpec,
        motor_id: int = 1,
        plant: Plant | None = None,
        *,
        reply_arbitration_id: int | None = 0x00,
        scaling: ScalingVariant = ScalingVariant.EXACT,
        wrap_mode: WrapMode = WrapMode.WRAP,
        velocity_wraps: bool = False,
        temperature_c: int = 30,
        fault_code: int = 0,
    ) -> None:
        self.spec = spec
        self.motor_id = motor_id
        self.plant = plant if plant is not None else Plant()
        self.reply_arbitration_id = (
            motor_id if reply_arbitration_id is None else reply_arbitration_id
        )
        self.scaling = scaling
        self.wrap_mode = wrap_mode
        self.velocity_wraps = velocity_wraps
        self.temperature_c = temperature_c
        self.fault_code = fault_code

        self.in_mit_mode = False
        self.enter_count = 0
        self.exit_count = 0
        self.zero_count = 0
        self.commands_seen = 0
        self.commands_ignored = 0
        self.last_command: tuple[float, float, float, float, float] | None = None
        self._torque = 0.0

    # --- protocol -------------------------------------------------------------------

    def handle(self, frame: Frame) -> Frame | None:
        """Process one frame; return a reply, or ``None`` if it was not for us."""
        if frame.is_extended_id or frame.arbitration_id != self.motor_id:
            return None
        data = frame.data
        if len(data) != 8:
            return None

        if data == ENTER_MIT:
            self.in_mit_mode = True
            self.enter_count += 1
            return self._reply()
        if data == EXIT_MIT:
            self.in_mit_mode = False
            self.exit_count += 1
            self._torque = 0.0
            return self._reply()
        if data == ZERO_POSITION:
            self.zero_count += 1
            self.plant.zero_here()
            return self._reply()

        if not self.in_mit_mode:
            self.commands_ignored += 1
            return None

        self.commands_seen += 1
        f = self.spec.mit
        p_int = (data[0] << 8) | data[1]
        v_int = (data[2] << 4) | (data[3] >> 4)
        kp_int = ((data[3] & 0x0F) << 8) | data[4]
        kd_int = (data[5] << 4) | (data[6] >> 4)
        t_int = ((data[6] & 0x0F) << 8) | data[7]

        p_des = _decode(p_int, f.position, self.scaling)
        v_des = _decode(v_int, f.velocity, self.scaling)
        kp = _decode(kp_int, f.kp, self.scaling)
        kd = _decode(kd_int, f.kd, self.scaling)
        t_ff = _decode(t_int, f.torque, self.scaling)
        self.last_command = (p_des, v_des, kp, kd, t_ff)

        return self._reply()

    def _inner_loop_torque(self) -> float:
        """The firmware's impedance law, saturated at the torque field."""
        if self.last_command is None:
            return 0.0
        p_des, v_des, kp, kd, t_ff = self.last_command
        raw = kp * (p_des - self.plant.position) + kd * (v_des - self.plant.velocity) + t_ff
        return self.spec.mit.torque.clamp(raw)

    def step(self, dt: float) -> None:
        """Advance by ``dt``, running the inner loop at the firmware's own rate."""
        if dt <= 0.0:
            return
        if not self.in_mit_mode:
            self._torque = 0.0
            self.plant.step(dt, 0.0)
            return
        steps = max(1, int(dt / FIRMWARE_LOOP_DT + 0.5))
        h = dt / steps
        for _ in range(steps):
            self._torque = self._inner_loop_torque()
            self.plant.step(h, self._torque)

    # --- feedback -------------------------------------------------------------------

    def _fold(self, value: float, field: FieldRange, wraps: bool) -> float:
        if field.contains(value):
            return value
        if not wraps:
            return field.clamp(value)
        span = field.span
        return ((value - field.lo) % span) + field.lo

    def _reply(self) -> Frame:
        f = self.spec.mit
        position = self._fold(self.plant.position, f.position, self.wrap_mode is WrapMode.WRAP)
        velocity = self._fold(self.plant.velocity, f.velocity, self.velocity_wraps)
        torque = f.torque.clamp(self._torque)

        p = _encode(position, f.position, self.scaling)
        v = _encode(velocity, f.velocity, self.scaling)
        t = _encode(torque, f.torque, self.scaling)
        return Frame(
            self.reply_arbitration_id,
            bytes(
                (
                    self.motor_id,
                    (p >> 8) & 0xFF,
                    p & 0xFF,
                    (v >> 4) & 0xFF,
                    ((v & 0x0F) << 4) | ((t >> 8) & 0x0F),
                    t & 0xFF,
                    (self.temperature_c + TEMPERATURE_OFFSET) & 0xFF,
                    self.fault_code & 0xFF,
                )
            ),
        )
