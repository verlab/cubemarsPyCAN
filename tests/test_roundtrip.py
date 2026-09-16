"""Round-trip fidelity across every motor and every field.

These are the property tests that back the claim "within 1 LSB". They also pin the
comparison against the manual's own scaling formula, so the deviation stays a known
quantity rather than a surprise.
"""

from __future__ import annotations

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from cubemarspycan.codec import mit, servo_can
from cubemarspycan.registry import MODEL_MIT_FIELDS, SPECS
from cubemarspycan.spec import FieldRange

ALL_FIELDS = [
    pytest.param(fields.__getattribute__(name), id=f"{model}.{name}")
    for model, fields in MODEL_MIT_FIELDS.items()
    for name in ("position", "velocity", "torque", "kp", "kd")
]


@pytest.mark.parametrize("field", ALL_FIELDS)
def test_round_trip_within_one_lsb(field: FieldRange) -> None:
    for i in range(0, 257):
        x = field.lo + field.span * i / 256.0
        back = field.from_uint(field.to_uint(x))
        assert abs(back - x) <= field.lsb / 2 + 1e-9


@pytest.mark.parametrize("field", ALL_FIELDS)
def test_encoded_value_always_fits_the_field(field: FieldRange) -> None:
    for i in range(-10, 267):
        x = field.lo + field.span * i / 256.0
        assert 0 <= field.to_uint(x) <= field.max_uint


@pytest.mark.parametrize("field", ALL_FIELDS)
def test_every_code_decodes_inside_the_declared_range(field: FieldRange) -> None:
    for u in (0, 1, field.max_uint // 2, field.max_uint - 1, field.max_uint):
        assert field.lo <= field.from_uint(u) <= field.hi


@pytest.mark.parametrize("field", ALL_FIELDS)
def test_differs_from_the_manual_formula_by_at_most_one_lsb(field: FieldRange) -> None:
    """The manual uses (1<<bits)/span, which overflows at x_max. Ours never does, and
    the two agree everywhere else to within a single count."""
    ratio = (1 << field.bits) / field.span
    for i in range(0, 513):
        x = field.lo + field.span * i / 512.0
        manual = int((x - field.lo) * ratio)
        assert abs(field.to_uint(x) - manual) <= 1


# --- whole-frame round trips --------------------------------------------------------


@pytest.mark.parametrize("model", sorted(MODEL_MIT_FIELDS))
def test_mit_command_survives_a_pack_unpack_cycle(model: str) -> None:
    """Re-lay a command as a reply frame; the position field is bit-identical."""
    f = MODEL_MIT_FIELDS[model]
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        pos = f.position.lo + f.position.span * frac
        cmd = mit.pack_command(
            f, position_rad=pos, velocity_radps=0.0, kp=0.0, kd=0.0, torque_nm=0.0
        )
        reply = bytes([1, cmd[0], cmd[1], 0, 0, 0, 60, 0])
        assert mit.unpack_feedback(f, reply).position_rad == pytest.approx(pos, abs=f.position.lsb)


@given(
    position=st.floats(min_value=-12.5, max_value=12.5, allow_nan=False),
    velocity=st.floats(min_value=-45.5, max_value=45.5, allow_nan=False),
    torque=st.floats(min_value=-5.0, max_value=5.0, allow_nan=False),
)
@settings(max_examples=300)
def test_mit_full_frame_round_trip(position: float, velocity: float, torque: float) -> None:
    f = SPECS["AK40-10"].mit
    cmd = mit.pack_command(
        f, position_rad=position, velocity_radps=velocity, kp=0.0, kd=0.0, torque_nm=torque
    )
    # The reply layout shifts everything one byte right to make room for the id.
    reply = bytes([1, cmd[0], cmd[1], cmd[2], (cmd[3] & 0xF0) | (cmd[6] & 0x0F), cmd[7], 60, 0])
    fb = mit.unpack_feedback(f, reply)
    assert fb.position_rad == pytest.approx(position, abs=f.position.lsb)
    assert fb.velocity_radps == pytest.approx(velocity, abs=f.velocity.lsb)
    assert fb.torque_nm == pytest.approx(torque, abs=f.torque.lsb)


@given(
    degrees=st.floats(min_value=-3200.0, max_value=3200.0, allow_nan=False),
    erpm=st.integers(min_value=-320_000, max_value=320_000),
    amps=st.floats(min_value=-60.0, max_value=60.0, allow_nan=False),
)
@settings(max_examples=300)
def test_servo_status_round_trip(degrees: float, erpm: int, amps: float) -> None:
    """Build a status payload the way the driver would, then decode it."""
    scaling = SPECS["AK40-10"].servo
    pos_raw = round(degrees / scaling.feedback_deg_per_lsb)
    spd_raw = round(erpm / scaling.feedback_erpm_per_lsb)
    cur_raw = round(amps / scaling.feedback_amps_per_lsb)
    data = bytes(
        [
            (pos_raw >> 8) & 0xFF,
            pos_raw & 0xFF,
            (spd_raw >> 8) & 0xFF,
            spd_raw & 0xFF,
            (cur_raw >> 8) & 0xFF,
            cur_raw & 0xFF,
            30,
            0,
        ]
    )
    fb = servo_can.decode_status(data, scaling)
    assert fb.position_deg == pytest.approx(degrees, abs=scaling.feedback_deg_per_lsb)
    assert fb.velocity_erpm == pytest.approx(erpm, abs=scaling.feedback_erpm_per_lsb)
    assert fb.current_a == pytest.approx(amps, abs=scaling.feedback_amps_per_lsb)


@given(degrees=st.floats(min_value=-36_000.0, max_value=36_000.0, allow_nan=False))
@settings(max_examples=300)
def test_servo_position_command_round_trip(degrees: float) -> None:
    """Decode our own SET_POS payload the way the firmware documents it."""
    import struct

    scaling = SPECS["AK40-10"].servo
    frame = servo_can.encode_position(1, degrees, scaling)
    (raw,) = struct.unpack(">i", frame.data)
    assert raw / scaling.position_scale == pytest.approx(degrees, abs=1e-4)


@given(
    degrees=st.floats(min_value=-36_000.0, max_value=36_000.0, allow_nan=False),
    speed=st.integers(min_value=-320_000, max_value=320_000),
    accel=st.integers(min_value=0, max_value=320_000),
)
@settings(max_examples=300)
def test_servo_position_speed_command_round_trip(degrees: float, speed: int, accel: int) -> None:
    import struct

    scaling = SPECS["AK40-10"].servo
    frame = servo_can.encode_position_speed(1, degrees, speed, accel, scaling)
    pos, spd, acc = struct.unpack(">ihh", frame.data)
    assert pos / scaling.pos_spd_position_scale == pytest.approx(degrees, abs=1e-4)
    assert spd * scaling.pos_spd_speed_divisor == pytest.approx(speed, abs=10)
    assert acc * scaling.pos_spd_accel_divisor == pytest.approx(accel, abs=10)
