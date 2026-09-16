"""Decoders must never raise anything but MalformedFrame, for any input at all.

A decoder runs on the receive thread. An unexpected exception there is invisible: python-can
swallows it and the motor simply stops updating. The reference library dies with
OverflowError on any negative servo position and KeyError on fault code 7, both silently.
"""

from __future__ import annotations

import contextlib

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from cubemarspycan import MalformedFrame, get_spec
from cubemarspycan.codec import mit, servo_can
from cubemarspycan.codec.servo_can import ServoFunction
from cubemarspycan.frame import Frame

FIELDS = get_spec("AK40-10").mit
SCALING = get_spec("AK40-10").servo

payloads = st.binary(min_size=0, max_size=8)


@given(data=payloads)
def test_mit_unpack_only_ever_raises_malformed_frame(data: bytes) -> None:
    with contextlib.suppress(MalformedFrame):
        mit.unpack_feedback(FIELDS, data)


@given(data=payloads)
def test_servo_decode_status_only_ever_raises_malformed_frame(data: bytes) -> None:
    with contextlib.suppress(MalformedFrame):
        servo_can.decode_status(data, SCALING)


@given(
    arb=st.integers(min_value=0, max_value=0x1FFFFFFF),
    data=payloads,
    extended=st.booleans(),
)
@settings(max_examples=500)
def test_servo_decode_only_ever_raises_malformed_frame(
    arb: int, data: bytes, extended: bool
) -> None:
    if not extended and arb > 0x7FF:
        return
    with contextlib.suppress(MalformedFrame):
        servo_can.decode(Frame(arb, data, extended), SCALING)


@given(arb=st.integers(min_value=0, max_value=0x1FFFFFFF), data=payloads)
def test_classify_never_raises(arb: int, data: bytes) -> None:
    result = servo_can.classify(Frame(arb, data, True))
    assert result is None or isinstance(result, ServoFunction)


@pytest.mark.parametrize("code", range(256))
def test_every_possible_fault_byte_decodes_without_raising(code: int) -> None:
    """Fault 7 (motor stall) is where the reference raises KeyError."""
    assert mit.unpack_feedback(FIELDS, bytes([1, 0, 0, 0, 0, 0, 60, code])).fault_code == code
    assert servo_can.decode_status(bytes([0, 0, 0, 0, 0, 0, 30, code]), SCALING).fault_code == code


@pytest.mark.parametrize("byte6", range(256))
def test_every_possible_temperature_byte_decodes(byte6: int) -> None:
    t = mit.unpack_feedback(FIELDS, bytes([1, 0, 0, 0, 0, 0, byte6, 0])).temperature_c
    assert -40 <= t <= 215


@given(
    position=st.floats(allow_nan=False, allow_infinity=False, width=32),
    velocity=st.floats(allow_nan=False, allow_infinity=False, width=32),
    kp=st.floats(allow_nan=False, allow_infinity=False, width=32),
    kd=st.floats(allow_nan=False, allow_infinity=False, width=32),
    torque=st.floats(allow_nan=False, allow_infinity=False, width=32),
)
@settings(max_examples=500)
def test_mit_pack_accepts_any_finite_input_and_emits_eight_bytes(
    position: float, velocity: float, kp: float, kd: float, torque: float
) -> None:
    out = mit.pack_command(
        FIELDS,
        position_rad=position,
        velocity_radps=velocity,
        kp=kp,
        kd=kd,
        torque_nm=torque,
    )
    assert len(out) == 8
    assert all(0 <= b <= 255 for b in out)
    assert out not in (mit.ENTER_MIT, mit.EXIT_MIT, mit.ZERO_POSITION)


@given(value=st.floats(allow_nan=False, allow_infinity=False, width=32))
@settings(max_examples=300)
def test_servo_encoders_accept_any_finite_input(value: float) -> None:
    for build in (
        servo_can.encode_duty,
        servo_can.encode_current,
        servo_can.encode_current_brake,
        servo_can.encode_rpm,
        servo_can.encode_position,
    ):
        frame = build(1, value)
        assert frame.dlc == 4
        assert frame.is_extended_id
    frame = servo_can.encode_position_speed(1, value, value, value)
    assert frame.dlc == 8
