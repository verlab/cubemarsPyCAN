"""The motor data model.

Two ideas carry this module, and between them they retire most of the defect classes
catalogued in PLAN.md section 2.

**1. A wire field and a physical limit are different types.**
:class:`FieldRange` carries ``bits`` and exists to be handed to the quantiser.
:class:`PhysicalLimits` has no ``bits`` and can never reach a codec. Keeping them apart
matters because across the AK line they differ in *both* directions: the AK40-10's MIT
torque field is +/-5.0 N*m against a 4.1 N*m peak (the field over-promises), while the
AK80-9's is +/-18 N*m against a 22 N*m peak (the field is what binds). Code that treats
"the limit" as one number is wrong for one of those motors whichever it picks.

**2. Unknown is representable, and it refuses.**
Every constant that is not on a wire is wrapped in :class:`Sourced`, which records where
the number came from. A conversion that needs an unknown constant raises
:class:`~cubemarspycan.errors.SpecIncompleteError` rather than guessing. TMotorCANControl
guessed - it hard-codes ``radps_per_ERPM = 5.82e-4`` for every motor, which is 22% wrong
for the AK40-10 - and shipped a table of constants literally commented
``UNTESTED CONSTANT!``. Refusing is more useful than a plausible wrong answer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Any, Generic, TypeVar

from .errors import CapabilityError, SpecIncompleteError
from .units import erpm_to_radps, float_to_uint, lsb, max_uint, radps_to_erpm, uint_to_float

T = TypeVar("T")


class Source(Enum):
    """Where a number came from, loosely ordered by how much it should be trusted."""

    MEASURED = "measured"
    """Fitted on a bench by the user of this library."""
    MANUAL = "manual"
    """AK Series Module Driver Manual v1.0.18."""
    DATASHEET = "datasheet"
    """CubeMars published product specification."""
    TOOL = "tool"
    """Read out of CubeMarsTool from this particular driver."""
    NAMEPLATE = "nameplate"
    """Inferred from the model name, e.g. the "-10" in AK40-10 meaning 10:1."""
    ESTIMATED = "estimated"
    """Derived from another constant, e.g. Kt from Kv."""
    ASSUMED = "assumed"
    """Set by the user via evolve() without verification."""
    UNKNOWN = "unknown"
    """No value. Conversions that need it must refuse."""


_TRUSTED = frozenset({Source.MEASURED, Source.MANUAL, Source.DATASHEET, Source.TOOL})


@dataclass(frozen=True, slots=True)
class Sourced(Generic[T]):
    """A constant together with its provenance."""

    value: T | None
    source: Source
    ref: str = ""
    """Manual page, datasheet URL, or bench-run identifier."""
    note: str = ""

    @property
    def known(self) -> bool:
        return self.value is not None and self.source is not Source.UNKNOWN

    @property
    def trusted(self) -> bool:
        return self.known and self.source in _TRUSTED

    def require(self, what: str) -> T:
        """Return the value, or explain what needs measuring and how to record it."""
        if not self.known:
            detail = f" ({self.note})" if self.note else ""
            raise SpecIncompleteError(
                f"{what} is not known for this motor{detail}. Measure it, then record it "
                f"with spec.evolve(...) using Source.MEASURED. Raw wire values remain "
                f"available without it."
            )
        assert self.value is not None  # narrowed by .known
        return self.value

    def __str__(self) -> str:
        if not self.known:
            return f"unknown ({self.note})" if self.note else "unknown"
        return f"{self.value} [{self.source.value}{': ' + self.ref if self.ref else ''}]"


def unknown(note: str = "") -> Sourced[T]:
    """A constant that has not been established. Conversions needing it will refuse."""
    return Sourced(None, Source.UNKNOWN, note=note)


class WrapMode(Enum):
    """What the driver does when position leaves the field's range."""

    WRAP = "wrap"
    """Reported value rolls over to the far end. Unwrapping can recover true position."""
    SATURATE = "saturate"
    """Reported value sticks at the limit. True position is unrecoverable past it."""
    UNKNOWN = "unknown"
    """Not yet established on this firmware. Refuses rather than guessing."""


class Side(Enum):
    """Which side of the gearbox a mechanical quantity refers to."""

    ROTOR = "rotor"
    OUTPUT = "output"


# --- wire fields --------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class FieldRange:
    """The scaling of one CAN bit-field. Handed to the quantiser; never a safety limit."""

    lo: float
    hi: float
    bits: int

    def __post_init__(self) -> None:
        if self.hi <= self.lo:
            raise ValueError(f"empty field range: lo={self.lo!r} hi={self.hi!r}")
        if not 1 <= self.bits <= 32:
            raise ValueError(f"field bits out of range: {self.bits!r}")

    @property
    def span(self) -> float:
        return self.hi - self.lo

    @property
    def lsb(self) -> float:
        """Resolution of this field in its own units."""
        return lsb(self.lo, self.hi, self.bits)

    @property
    def max_uint(self) -> int:
        return max_uint(self.bits)

    def clamp(self, x: float) -> float:
        return min(max(x, self.lo), self.hi)

    def contains(self, x: float) -> bool:
        return self.lo <= x <= self.hi

    def to_uint(self, x: float) -> int:
        return float_to_uint(x, self.lo, self.hi, self.bits)

    def from_uint(self, u: int) -> float:
        return uint_to_float(u, self.lo, self.hi, self.bits)

    def __str__(self) -> str:
        return f"[{self.lo:g}, {self.hi:g}] / {self.bits}b (lsb {self.lsb:.3e})"


@dataclass(frozen=True, slots=True)
class MitFields:
    """The five MIT command fields. Manual v1.0.18 p.63.

    Position and velocity are output-side. For the AK40-10 this is confirmed rather than
    assumed: the velocity field's 45.5 rad/s is 434.5 rpm, which is the datasheet's
    435 rpm no-load speed.
    """

    position: FieldRange
    velocity: FieldRange
    torque: FieldRange
    kp: FieldRange
    kd: FieldRange


@dataclass(frozen=True, slots=True)
class ServoScaling:
    """Servo-mode wire scaling. Identical across AK models; manual v1.0.18 pp.38-45.

    Note the command and feedback scalings for position are different numbers
    (``deg * 1e4`` going out, ``0.1 deg`` per LSB coming back) and the position-velocity
    packet divides speed and acceleration by 10 while the plain velocity packet does not.
    TMotorCANControl gets both of these wrong.
    """

    # feedback, function id 0x29
    feedback_deg_per_lsb: float = 0.1
    feedback_erpm_per_lsb: float = 10.0
    feedback_amps_per_lsb: float = 0.01
    # commands
    duty_scale: float = 100_000.0
    current_scale: float = 1_000.0
    rpm_scale: float = 1.0
    position_scale: float = 10_000.0
    pos_spd_position_scale: float = 10_000.0
    pos_spd_speed_divisor: float = 10.0
    pos_spd_accel_divisor: float = 10.0
    # field limits (wire, not physical)
    current_field: FieldRange = field(default_factory=lambda: FieldRange(-60.0, 60.0, 32))
    erpm_field: FieldRange = field(default_factory=lambda: FieldRange(-100_000.0, 100_000.0, 32))
    position_side: Sourced[Side] = field(
        default_factory=lambda: unknown(
            "the manual states degrees and a +/-3200 deg range but never says whether "
            "servo position is rotor- or output-side; settle it with bench step B7"
        )
    )


SERVO_CAN_COMMON = ServoScaling()
"""Shared servo scaling. Every AK model uses these numbers."""


# --- physical -----------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Drivetrain:
    """Constants that relate the rotor to the output shaft and current to torque."""

    gear_ratio: Sourced[float]
    pole_pairs: Sourced[int]
    kt_nm_per_a: Sourced[float]
    """Torque constant, rotor-side, N*m per amp of q-axis current."""
    kv_rpm_per_v: Sourced[float] = field(default_factory=unknown)
    ke_v_per_krpm: Sourced[float] = field(default_factory=unknown)


@dataclass(frozen=True, slots=True)
class PhysicalLimits:
    """What the motor can actually do. Never handed to a codec: there are no ``bits`` here."""

    peak_torque_nm: Sourced[float] = field(default_factory=unknown)
    rated_torque_nm: Sourced[float] = field(default_factory=unknown)
    peak_current_a: Sourced[float] = field(default_factory=unknown)
    rated_current_a: Sourced[float] = field(default_factory=unknown)
    no_load_speed_radps: Sourced[float] = field(default_factory=unknown)
    rated_speed_radps: Sourced[float] = field(default_factory=unknown)
    rated_voltage_v: Sourced[float] = field(default_factory=unknown)
    max_board_temp_c: Sourced[float] = field(default_factory=unknown)


@dataclass(frozen=True, slots=True)
class Capabilities:
    """What this variant's hardware supports."""

    encoders: int = 1
    inner_encoder_bits: Sourced[int] = field(default_factory=unknown)
    outer_encoder_bits: Sourced[int] = field(default_factory=unknown)

    @property
    def permanent_zero(self) -> bool:
        """Whether origin mode 1 is legal.

        The manual restricts permanent zero to dual-encoder models. It writes flash, so
        sending it to a single-encoder motor is both meaningless and wearing. The
        AK40-10 has one encoder; the AK10-9 and AK80-8 have two.
        """
        return self.encoders >= 2


# --- the spec -----------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class MotorSpec:
    """Everything known about one motor variant.

    Keyed by *variant*, not model: KV and hardware revision change Kt, pole pairs and even
    encoder count, while the MIT field ranges are shared by every variant of a model.
    """

    name: str
    """Variant key, e.g. "AK40-10-KV170"."""
    model: str
    """Model as the manual's field-range table names it, e.g. "AK40-10"."""
    mit: MitFields
    drivetrain: Drivetrain
    limits: PhysicalLimits = field(default_factory=PhysicalLimits)
    capabilities: Capabilities = field(default_factory=Capabilities)
    servo: ServoScaling = SERVO_CAN_COMMON
    mit_wrap_mode: Sourced[WrapMode] = field(
        default_factory=lambda: Sourced(
            WrapMode.UNKNOWN,
            Source.UNKNOWN,
            note="the manual does not say whether position wraps or saturates past the "
            "field limit; drive past it on a bench to find out",
        )
    )
    mit_position_side: Sourced[Side] = field(
        default_factory=lambda: Sourced(
            Side.OUTPUT,
            Source.ESTIMATED,
            note="inferred from the velocity field matching the datasheet no-load speed",
        )
    )
    manual_version: str = "1.0.18"
    notes: str = ""

    # --- conversions that refuse rather than guess ---------------------------------

    def erpm_to_radps_output(self, erpm: float) -> float:
        return erpm_to_radps(
            erpm,
            self.drivetrain.pole_pairs.require("ERPM to rad/s conversion (pole pairs)"),
            self.drivetrain.gear_ratio.require("ERPM to rad/s conversion (gear ratio)"),
        )

    def radps_output_to_erpm(self, radps: float) -> float:
        return radps_to_erpm(
            radps,
            self.drivetrain.pole_pairs.require("rad/s to ERPM conversion (pole pairs)"),
            self.drivetrain.gear_ratio.require("rad/s to ERPM conversion (gear ratio)"),
        )

    def output_torque_from_current(self, amps: float) -> float:
        """Output-shaft torque for a q-axis current, ignoring gearbox losses.

        For the AK40-10 this reproduces the datasheet: 7.3 A * 0.056 * 10 = 4.09 N*m
        against a published 4.1 N*m peak.
        """
        kt = self.drivetrain.kt_nm_per_a.require("torque from current (Kt)")
        gr = self.drivetrain.gear_ratio.require("torque from current (gear ratio)")
        return amps * kt * gr

    def current_for_output_torque(self, torque_nm: float) -> float:
        kt = self.drivetrain.kt_nm_per_a.require("current from torque (Kt)")
        gr = self.drivetrain.gear_ratio.require("current from torque (gear ratio)")
        return torque_nm / (kt * gr)

    # --- field vs physical ----------------------------------------------------------

    def effective_torque_limit_nm(self) -> float:
        """The smaller of what the wire can express and what the motor can produce."""
        peak = self.limits.peak_torque_nm
        if peak.known:
            return min(self.mit.torque.hi, peak.require("effective torque limit"))
        return self.mit.torque.hi

    def effective_velocity_limit_radps(self) -> float:
        no_load = self.limits.no_load_speed_radps
        if no_load.known:
            return min(self.mit.velocity.hi, no_load.require("effective velocity limit"))
        return self.mit.velocity.hi

    def no_load_speed_radps_at(self, supply_v: float) -> float:
        """Predicted no-load output speed at a given supply voltage."""
        ke = self.drivetrain.ke_v_per_krpm
        gr = self.drivetrain.gear_ratio.require("no-load speed (gear ratio)")
        if ke.known:
            rotor_rpm = supply_v / ke.require("no-load speed (Ke)") * 1000.0
        else:
            kv = self.drivetrain.kv_rpm_per_v.require("no-load speed (Kv or Ke)")
            rotor_rpm = supply_v * kv
        return rotor_rpm / gr * (2.0 * math.pi) / 60.0

    def velocity_field_saturation_voltage(self) -> float:
        """Supply voltage above which the motor can outrun the MIT velocity field.

        For the AK40-10 this is 25.6 V. At a fixed 24 V supply there is 6% headroom and
        the field is adequate under power; only back-driving can exceed it.
        """
        gr = self.drivetrain.gear_ratio.require("saturation voltage (gear ratio)")
        rotor_rpm = self.mit.velocity.hi * 60.0 / (2.0 * math.pi) * gr
        ke = self.drivetrain.ke_v_per_krpm
        if ke.known:
            return rotor_rpm / 1000.0 * ke.require("saturation voltage (Ke)")
        kv = self.drivetrain.kv_rpm_per_v.require("saturation voltage (Kv or Ke)")
        return rotor_rpm / kv

    # --- capability gates -----------------------------------------------------------

    def require_permanent_zero(self) -> None:
        """Raise unless this variant may be sent origin mode 1. Emits no frame."""
        if not self.capabilities.permanent_zero:
            raise CapabilityError(
                f"{self.name} has {self.capabilities.encoders} encoder(s); the manual "
                f"restricts permanent zero (origin mode 1) to dual-encoder models. It "
                f"writes flash. Use OriginMode.TEMPORARY instead."
            )

    # --- ergonomics -----------------------------------------------------------------

    def evolve(self, **changes: object) -> MotorSpec:
        """Return a new spec with fields replaced, e.g. after a bench measurement."""
        return replace(self, **changes)  # type: ignore[arg-type]

    def provenance_report(self) -> str:
        """Human-readable audit of where every non-wire constant came from."""
        rows: list[tuple[str, Sourced[Any]]] = [
            ("gear_ratio", self.drivetrain.gear_ratio),
            ("pole_pairs", self.drivetrain.pole_pairs),
            ("kt_nm_per_a", self.drivetrain.kt_nm_per_a),
            ("kv_rpm_per_v", self.drivetrain.kv_rpm_per_v),
            ("ke_v_per_krpm", self.drivetrain.ke_v_per_krpm),
            ("peak_torque_nm", self.limits.peak_torque_nm),
            ("rated_torque_nm", self.limits.rated_torque_nm),
            ("peak_current_a", self.limits.peak_current_a),
            ("rated_current_a", self.limits.rated_current_a),
            ("no_load_speed_radps", self.limits.no_load_speed_radps),
            ("rated_voltage_v", self.limits.rated_voltage_v),
            ("mit.position_side", self.mit_position_side),
            ("servo.position_side", self.servo.position_side),
        ]
        width = max(len(n) for n, _ in rows)
        lines = [f"{self.name}  (model {self.model}, manual v{self.manual_version})"]
        lines += [f"  {n:<{width}}  {s}" for n, s in rows]
        unknowns = [n for n, s in rows if not s.known]
        if unknowns:
            lines.append(f"  -> {len(unknowns)} unknown: {', '.join(unknowns)}")
        return "\n".join(lines)
