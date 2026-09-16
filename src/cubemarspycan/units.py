"""Pure unit conversions.

Nothing in this module performs I/O, imports ``can``, or holds mutable state. It is the
bottom of the dependency graph and is covered to 100%.

The float/uint pair is the single most important thing here. The manual's own
``float_to_uint`` uses ``(1 << bits) / span``, which **overflows the field at exactly
x_max** (12.5 rad in a 16-bit field maps to 65536, which does not fit; 5.0 N*m in a
12-bit field maps to 4096, which does not fit). We use ``((1 << bits) - 1) / span``,
which is the exact inverse of the firmware's documented ``uint_to_float`` and can never
overflow. The two formulas disagree by at most 1 LSB (0.00038 rad on AK40-10 position).
"""

from __future__ import annotations

import math

TAU = 2.0 * math.pi
"""One full turn in radians."""

_RADPS_PER_RPM = TAU / 60.0


def max_uint(bits: int) -> int:
    """Largest value representable in an unsigned field of ``bits`` bits."""
    return (1 << bits) - 1


def float_to_uint(x: float, lo: float, hi: float, bits: int) -> int:
    """Quantise ``x`` in ``[lo, hi]`` onto an unsigned ``bits``-bit field.

    ``x`` is clamped into range first, so this never raises for out-of-range input and
    the result is always in ``[0, max_uint(bits)]``. ``lo`` maps to 0 and ``hi`` maps to
    ``max_uint(bits)`` exactly.
    """
    if hi <= lo:
        raise ValueError(f"empty range: lo={lo!r} hi={hi!r}")
    if not 1 <= bits <= 32:
        raise ValueError(f"bits out of range: {bits!r}")
    if math.isnan(x):
        raise ValueError("cannot encode NaN")
    x = min(max(x, lo), hi)
    u = round((x - lo) * (max_uint(bits) / (hi - lo)))
    # round() on a clamped value cannot escape the field, but belt and braces: a value
    # that escapes here would corrupt neighbouring bit-fields in the packed frame.
    return min(max(u, 0), max_uint(bits))


def uint_to_float(u: int, lo: float, hi: float, bits: int) -> float:
    """Inverse of :func:`float_to_uint`. This matches the firmware's documented formula."""
    if hi <= lo:
        raise ValueError(f"empty range: lo={lo!r} hi={hi!r}")
    if not 1 <= bits <= 32:
        raise ValueError(f"bits out of range: {bits!r}")
    return u * (hi - lo) / max_uint(bits) + lo


def lsb(lo: float, hi: float, bits: int) -> float:
    """Size of one least-significant bit, in the field's own units."""
    return (hi - lo) / max_uint(bits)


# --- rotational -------------------------------------------------------------------


def rpm_to_radps(rpm: float) -> float:
    return rpm * _RADPS_PER_RPM


def radps_to_rpm(radps: float) -> float:
    return radps / _RADPS_PER_RPM


def erpm_to_radps(erpm: float, pole_pairs: int, gear_ratio: float) -> float:
    """Electrical RPM -> mechanical rad/s **at the output shaft**.

    ERPM counts electrical revolutions of the rotor, so both the pole-pair count and the
    gearbox divide out. For the AK40-10 (14 pole pairs, 10:1) one ERPM is 7.480e-4 rad/s.
    """
    if pole_pairs <= 0:
        raise ValueError(f"pole_pairs must be positive, got {pole_pairs!r}")
    if gear_ratio <= 0:
        raise ValueError(f"gear_ratio must be positive, got {gear_ratio!r}")
    return erpm * TAU / (60.0 * pole_pairs * gear_ratio)


def radps_to_erpm(radps: float, pole_pairs: int, gear_ratio: float) -> float:
    """Mechanical rad/s at the output shaft -> electrical RPM."""
    if pole_pairs <= 0:
        raise ValueError(f"pole_pairs must be positive, got {pole_pairs!r}")
    if gear_ratio <= 0:
        raise ValueError(f"gear_ratio must be positive, got {gear_ratio!r}")
    return radps * 60.0 * pole_pairs * gear_ratio / TAU


deg_to_rad = math.radians
rad_to_deg = math.degrees


# --- two's complement helpers (servo-mode wire fields) ------------------------------


def to_signed(value: int, bits: int) -> int:
    """Reinterpret the low ``bits`` of ``value`` as a two's-complement signed integer."""
    value &= max_uint(bits)
    sign = 1 << (bits - 1)
    return value - (1 << bits) if value & sign else value


def to_unsigned(value: int, bits: int) -> int:
    """Two's-complement encode ``value`` into ``bits`` bits, wrapping like C would."""
    return value & max_uint(bits)
