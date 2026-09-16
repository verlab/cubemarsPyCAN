"""Unit conversions. The float/uint pair is the foundation everything else stands on."""

from __future__ import annotations

import math

import pytest
from hypothesis import given
from hypothesis import strategies as st

from cubemarspycan.units import (
    TAU,
    erpm_to_radps,
    float_to_uint,
    lsb,
    max_uint,
    radps_to_erpm,
    radps_to_rpm,
    rpm_to_radps,
    to_signed,
    to_unsigned,
    uint_to_float,
)

# Every MIT field on every motor in the manual's table, as (lo, hi, bits).
AK40_10_FIELDS = [
    (-12.5, 12.5, 16),  # position
    (-45.5, 45.5, 12),  # velocity
    (-5.0, 5.0, 12),  # torque
    (0.0, 500.0, 12),  # kp
    (0.0, 5.0, 12),  # kd
]


def test_max_uint() -> None:
    assert max_uint(12) == 4095
    assert max_uint(16) == 65535


@pytest.mark.parametrize(("lo", "hi", "bits"), AK40_10_FIELDS)
def test_endpoints_and_midpoint_are_exact(lo: float, hi: float, bits: int) -> None:
    assert float_to_uint(lo, lo, hi, bits) == 0
    assert float_to_uint(hi, lo, hi, bits) == max_uint(bits)
    assert uint_to_float(0, lo, hi, bits) == pytest.approx(lo)
    assert uint_to_float(max_uint(bits), lo, hi, bits) == pytest.approx(hi)


@pytest.mark.parametrize(("lo", "hi", "bits"), AK40_10_FIELDS)
def test_never_exceeds_the_field(lo: float, hi: float, bits: int) -> None:
    """The defect that motivated this formula.

    The manual's own float_to_uint uses (1<<bits)/span and maps x_max to 1<<bits, which
    does not fit: 12.5 rad -> 65536 in a 16-bit field, 5.0 Nm -> 4096 in a 12-bit field.
    Overflowing here corrupts the neighbouring bit-field in the packed frame.
    """
    span = hi - lo
    for i in range(0, 2001):
        x = lo + span * i / 2000.0
        assert 0 <= float_to_uint(x, lo, hi, bits) <= max_uint(bits)
    # including well outside the range, where clamping must engage
    for x in (lo - 1e6, hi + 1e6, lo - 0.001, hi + 0.001):
        assert 0 <= float_to_uint(x, lo, hi, bits) <= max_uint(bits)


@pytest.mark.parametrize(("lo", "hi", "bits"), AK40_10_FIELDS)
def test_manual_formula_would_overflow(lo: float, hi: float, bits: int) -> None:
    """Pin the bug we are avoiding, so nobody 'simplifies' back to the manual's version."""
    manual = int((hi - lo) * ((1 << bits) / (hi - lo)))
    assert manual > max_uint(bits)
    assert float_to_uint(hi, lo, hi, bits) == max_uint(bits)


@pytest.mark.parametrize(("lo", "hi", "bits"), AK40_10_FIELDS)
def test_round_trip_within_one_lsb(lo: float, hi: float, bits: int) -> None:
    step = lsb(lo, hi, bits)
    span = hi - lo
    for i in range(0, 1001):
        x = lo + span * i / 1000.0
        back = uint_to_float(float_to_uint(x, lo, hi, bits), lo, hi, bits)
        assert abs(back - x) <= step / 2 + 1e-12


@pytest.mark.parametrize(("lo", "hi", "bits"), AK40_10_FIELDS)
def test_monotone(lo: float, hi: float, bits: int) -> None:
    span = hi - lo
    prev = -1
    for i in range(0, 501):
        u = float_to_uint(lo + span * i / 500.0, lo, hi, bits)
        assert u >= prev
        prev = u


def test_clamps_out_of_range() -> None:
    assert float_to_uint(99.0, -12.5, 12.5, 16) == 65535
    assert float_to_uint(-99.0, -12.5, 12.5, 16) == 0


def test_rejects_nonsense() -> None:
    with pytest.raises(ValueError, match="empty range"):
        float_to_uint(0.0, 1.0, 1.0, 12)
    with pytest.raises(ValueError, match="bits out of range"):
        float_to_uint(0.0, -1.0, 1.0, 0)
    with pytest.raises(ValueError, match="NaN"):
        float_to_uint(math.nan, -1.0, 1.0, 12)
    with pytest.raises(ValueError, match="empty range"):
        uint_to_float(0, 1.0, 1.0, 12)
    with pytest.raises(ValueError, match="bits out of range"):
        uint_to_float(0, -1.0, 1.0, 33)


@given(
    x=st.floats(min_value=-12.5, max_value=12.5, allow_nan=False),
    bits=st.integers(min_value=8, max_value=16),
)
def test_round_trip_property(x: float, bits: int) -> None:
    lo, hi = -12.5, 12.5
    u = float_to_uint(x, lo, hi, bits)
    assert 0 <= u <= max_uint(bits)
    assert abs(uint_to_float(u, lo, hi, bits) - x) <= lsb(lo, hi, bits) / 2 + 1e-9


def test_lsb_values_for_ak40_10() -> None:
    """Wire resolution, for comparison against the 14-bit encoder (3.83e-5 rad output)."""
    assert lsb(-12.5, 12.5, 16) == pytest.approx(3.8149e-4, rel=1e-3)
    assert lsb(-45.5, 45.5, 12) == pytest.approx(2.2222e-2, rel=1e-3)
    assert lsb(-5.0, 5.0, 12) == pytest.approx(2.4420e-3, rel=1e-3)


# --- rotational ---------------------------------------------------------------------


def test_erpm_for_ak40_10() -> None:
    """14 pole pairs, 10:1. TMotorCANControl hard-codes 5.82e-4 for every motor."""
    assert erpm_to_radps(1.0, 14, 10.0) == pytest.approx(7.479983e-4, rel=1e-6)
    assert erpm_to_radps(1.0, 14, 10.0) != pytest.approx(5.82e-4, rel=1e-3)


def test_erpm_for_ak80_9() -> None:
    """21 pole pairs, 9:1 - even the motor the 5.82e-4 constant was tuned on is 5% off."""
    assert erpm_to_radps(1.0, 21, 9.0) == pytest.approx(5.5411e-4, rel=1e-4)


def test_erpm_round_trip() -> None:
    for w in (-50.0, -1.0, 0.0, 1.0, 45.5):
        assert radps_to_erpm(erpm_to_radps(w, 14, 10.0), 14, 10.0) == pytest.approx(w)


def test_servo_erpm_field_cannot_saturate_on_ak40_10() -> None:
    """+/-320000 ERPM feedback field vs a 435 rpm no-load speed: 5x headroom."""
    field_radps = erpm_to_radps(320_000.0, 14, 10.0)
    assert radps_to_rpm(field_radps) == pytest.approx(2285.7, rel=1e-3)
    assert field_radps > rpm_to_radps(435.0) * 5


@pytest.mark.parametrize("bad", [0, -1])
def test_erpm_rejects_nonsense(bad: int) -> None:
    with pytest.raises(ValueError, match="pole_pairs"):
        erpm_to_radps(1.0, bad, 10.0)
    with pytest.raises(ValueError, match="gear_ratio"):
        erpm_to_radps(1.0, 14, bad)
    with pytest.raises(ValueError, match="pole_pairs"):
        radps_to_erpm(1.0, bad, 10.0)
    with pytest.raises(ValueError, match="gear_ratio"):
        radps_to_erpm(1.0, 14, bad)


def test_rpm_radps() -> None:
    assert rpm_to_radps(60.0) == pytest.approx(TAU)
    assert radps_to_rpm(TAU) == pytest.approx(60.0)
    assert rpm_to_radps(435.0) == pytest.approx(45.553, rel=1e-4)


# --- two's complement ---------------------------------------------------------------


@pytest.mark.parametrize(
    ("raw", "bits", "expected"),
    [
        (0x4E20, 16, 20000),
        (0xB1E0, 16, -20000),
        (0x7FFF, 16, 32767),
        (0x8000, 16, -32768),
        (0xFFFF, 16, -1),
        (0x0000, 16, 0),
    ],
)
def test_to_signed(raw: int, bits: int, expected: int) -> None:
    """The decode path TMotorCANControl crashes on: np.int16(0xB1E0) raises on NumPy 2."""
    assert to_signed(raw, bits) == expected


def test_signed_round_trip() -> None:
    for v in (-32768, -20000, -1, 0, 1, 20000, 32767):
        assert to_signed(to_unsigned(v, 16), 16) == v
