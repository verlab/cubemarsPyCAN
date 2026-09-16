"""Servo-CAN codec: goldens per packet id, and the reply classifier.

Three of these pin bugs in TMotorCANControl directly: the 1e4 position scale (it uses
1e6), the /10 divisors on position-velocity speed and acceleration (it omits both), and
the refusal to decode 0x2C or 0x09 as state (it decodes all three the same way).
"""

from __future__ import annotations

import pytest

from cubemarspycan import MalformedFrame, get_spec
from cubemarspycan.codec import servo_can as sv
from cubemarspycan.codec.servo_can import (
    SERVO_MODE_ACK_PAYLOAD,
    OriginMode,
    ServoEventFrame,
    ServoFeedback,
    ServoFunction,
    ServoPacket,
)
from cubemarspycan.frame import Frame

SCALING = get_spec("AK40-10").servo
MID = 1


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


# --- arbitration id -----------------------------------------------------------------


def test_arbitration_id_packs_the_packet_above_the_motor() -> None:
    assert sv.arbitration_id(ServoPacket.SET_POS, 1) == 0x0401
    assert sv.arbitration_id(ServoPacket.SET_DUTY, 0) == 0x0000
    assert sv.arbitration_id(ServoPacket.SET_POS_SPD, 255) == 0x06FF


def test_split_arbitration_id_is_the_inverse() -> None:
    assert sv.split_arbitration_id(0x0401) == (0x04, 0x01)
    assert sv.split_arbitration_id(0x2903) == (0x29, 0x03)


def test_motor_id_is_bounds_checked() -> None:
    for bad in (-1, 256):
        with pytest.raises(ValueError, match=r"0\.\.255"):
            sv.arbitration_id(ServoPacket.SET_POS, bad)


# --- command goldens ----------------------------------------------------------------


def test_duty_scales_by_1e5() -> None:
    f = sv.encode_duty(MID, 0.20)
    assert hexs(f.data) == "00 00 4E 20"  # 20000
    assert hexs(sv.encode_duty(MID, -0.20).data) == "FF FF B1 E0"


def test_current_scales_by_1e3() -> None:
    assert hexs(sv.encode_current(MID, 5.0).data) == "00 00 13 88"  # 5000
    assert hexs(sv.encode_current(MID, -5.0).data) == "FF FF EC 78"


def test_current_brake_is_unsigned() -> None:
    assert hexs(sv.encode_current_brake(MID, 5.0).data) == "00 00 13 88"
    assert hexs(sv.encode_current_brake(MID, -5.0).data) == "00 00 00 00"


def test_rpm_is_raw_erpm() -> None:
    assert hexs(sv.encode_rpm(MID, 1000).data) == "00 00 03 E8"
    assert hexs(sv.encode_rpm(MID, -1000).data) == "FF FF FC 18"


def test_position_scales_by_1e4_not_1e6() -> None:
    """TMotorCANControl uses 1e6 here, which is 100x too large."""
    assert hexs(sv.encode_position(MID, 90.0).data) == "00 0D BB A0"  # 900000
    assert hexs(sv.encode_position(MID, 180.0).data) == "00 1B 77 40"  # 1800000
    assert hexs(sv.encode_position(MID, -90.0).data) == "FF F2 44 60"


def test_position_speed_divides_speed_and_accel_by_ten() -> None:
    """The reference library omits both divisors, so it commands 10x speed and accel."""
    f = sv.encode_position_speed(MID, 90.0, speed_erpm=5000, accel_erpm_s2=30000)
    assert hexs(f.data) == "00 0D BB A0 01 F4 0B B8"
    #                      \--pos--/ \spd/ \acc/
    assert f.data[4:6] == (500).to_bytes(2, "big")  # 5000 / 10
    assert f.data[6:8] == (3000).to_bytes(2, "big")  # 30000 / 10


def test_position_speed_accepts_negative_speed_but_not_negative_accel() -> None:
    f = sv.encode_position_speed(MID, 0.0, speed_erpm=-5000, accel_erpm_s2=-100)
    assert f.data[4:6] == (-500 & 0xFFFF).to_bytes(2, "big")
    assert f.data[6:8] == b"\x00\x00"


def test_origin_defaults_to_temporary() -> None:
    assert sv.encode_origin(MID).data == b"\x00"
    assert sv.encode_origin(MID, OriginMode.TEMPORARY).data == b"\x00"
    assert sv.encode_origin(MID, OriginMode.PERMANENT).data == b"\x01"


# --- framing ------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("build", "packet", "dlc"),
    [
        (lambda: sv.encode_duty(MID, 0.1), ServoPacket.SET_DUTY, 4),
        (lambda: sv.encode_current(MID, 1.0), ServoPacket.SET_CURRENT, 4),
        (lambda: sv.encode_current_brake(MID, 1.0), ServoPacket.SET_CURRENT_BRAKE, 4),
        (lambda: sv.encode_rpm(MID, 100), ServoPacket.SET_RPM, 4),
        (lambda: sv.encode_position(MID, 10.0), ServoPacket.SET_POS, 4),
        (lambda: sv.encode_origin(MID), ServoPacket.SET_ORIGIN, 1),
        (lambda: sv.encode_position_speed(MID, 1.0, 10, 10), ServoPacket.SET_POS_SPD, 8),
    ],
)
def test_frames_are_extended_with_the_right_id_and_dlc(build, packet, dlc) -> None:  # type: ignore[no-untyped-def]
    """DLC varies per packet: 4 for scalars, 1 for origin, 8 for position-velocity."""
    f = build()
    assert f.is_extended_id, "servo frames are extended; MIT frames are standard"
    assert f.arbitration_id == sv.arbitration_id(packet, MID)
    assert f.dlc == dlc


def test_set_mit_has_no_encoder() -> None:
    """Listed in the manual's enum with no documented payload, so we do not guess one."""
    assert ServoPacket.SET_MIT.value == 8
    assert not any(name.endswith("mit") for name in dir(sv) if name.startswith("encode_"))


# --- clamping -----------------------------------------------------------------------


def test_commands_clamp_to_the_wire_limits() -> None:
    assert sv.encode_duty(MID, 99.0).data == sv.encode_duty(MID, 1.0).data
    assert sv.encode_current(MID, 1e6).data == sv.encode_current(MID, 60.0).data
    assert sv.encode_current(MID, -1e6).data == sv.encode_current(MID, -60.0).data
    assert sv.encode_rpm(MID, 1e9).data == sv.encode_rpm(MID, 100_000).data
    assert sv.encode_position(MID, 1e9).data == sv.encode_position(MID, 36_000.0).data


def test_position_speed_clamps_into_int16() -> None:
    f = sv.encode_position_speed(MID, 0.0, speed_erpm=1e9, accel_erpm_s2=1e9)
    assert f.data[4:6] == b"\x7f\xff"
    assert f.data[6:8] == b"\x7f\xff"


# --- reply classification -----------------------------------------------------------


def status_frame(motor_id: int = MID, payload: bytes = bytes(8)) -> Frame:
    return Frame(sv.arbitration_id(ServoFunction.STATUS, motor_id), payload, True)


def test_classify_recognises_the_three_reply_types() -> None:
    assert sv.classify(status_frame()) is ServoFunction.STATUS
    ack = Frame(sv.arbitration_id(ServoFunction.SERVO_MODE_ACK, MID), SERVO_MODE_ACK_PAYLOAD, True)
    assert sv.classify(ack) is ServoFunction.SERVO_MODE_ACK
    boot = Frame(sv.arbitration_id(ServoFunction.BOOTLOADER_JUMP, MID), bytes(8), True)
    assert sv.classify(boot) is ServoFunction.BOOTLOADER_JUMP


def test_classify_rejects_standard_frames_and_unknown_functions() -> None:
    assert sv.classify(Frame(0x401, bytes(8))) is None, "must be extended"
    assert sv.classify(Frame(0x2801, bytes(8), True)) is None
    assert sv.classify(Frame(0x2A01, bytes(8), True)) is None


def test_the_handshake_frame_is_an_event_not_a_position() -> None:
    """TMotorCANControl decodes FA FB FC FD as position, yielding a bogus -128.5 deg."""
    ack = Frame(sv.arbitration_id(ServoFunction.SERVO_MODE_ACK, MID), SERVO_MODE_ACK_PAYLOAD, True)
    out = sv.decode(ack, SCALING)
    assert isinstance(out, ServoEventFrame)
    assert out.kind == "servo_mode_ack"
    assert out.payload == SERVO_MODE_ACK_PAYLOAD
    # mypy proves ServoEventFrame and ServoFeedback are disjoint, so a handshake frame
    # cannot be mistaken for state even in principle - a stronger guarantee than a check.


def test_the_bootloader_frame_is_an_event() -> None:
    boot = Frame(sv.arbitration_id(ServoFunction.BOOTLOADER_JUMP, MID), bytes(8), True)
    out = sv.decode(boot, SCALING)
    assert isinstance(out, ServoEventFrame)
    assert out.kind == "bootloader_jump"


def test_decode_rejects_anything_it_does_not_recognise() -> None:
    with pytest.raises(MalformedFrame, match="not a servo reply"):
        sv.decode(Frame(0x2801, bytes(8), True), SCALING)


# --- status decoding ----------------------------------------------------------------


def test_status_scaling_matches_the_manual() -> None:
    # pos +20000 -> 2000.0 deg, spd +32000 -> 320000 ERPM, cur +500 -> 5.00 A
    data = bytes([0x4E, 0x20, 0x7D, 0x00, 0x01, 0xF4, 25, 0])
    fb = sv.decode(status_frame(payload=data), SCALING)
    assert isinstance(fb, ServoFeedback)
    assert fb.position_deg == pytest.approx(2000.0)
    assert fb.velocity_erpm == pytest.approx(320_000.0)
    assert fb.current_a == pytest.approx(5.0)
    assert fb.temperature_c == 25
    assert fb.fault_code == 0
    assert fb.motor_id == MID


def test_negative_status_values_decode() -> None:
    """NumPy 2 raises OverflowError here, which is what kills the reference receive path."""
    data = bytes([0xB1, 0xE0, 0x83, 0x00, 0xFE, 0x0C, 0xFB, 0])
    fb = sv.decode(status_frame(payload=data), SCALING)
    assert isinstance(fb, ServoFeedback)
    assert fb.position_deg == pytest.approx(-2000.0)
    assert fb.velocity_erpm == pytest.approx(-320_000.0)
    assert fb.current_a == pytest.approx(-5.0)
    assert fb.temperature_c == -5, "temperature is int8, so it can go below zero"


def test_status_extremes() -> None:
    lo = sv.decode_status(bytes([0x83, 0x00, 0x83, 0x00, 0xE8, 0x90, 0xEC, 0]), SCALING)
    assert lo.position_deg == pytest.approx(-3200.0)
    assert lo.velocity_erpm == pytest.approx(-320_000.0)
    assert lo.current_a == pytest.approx(-60.0)
    assert lo.temperature_c == -20


def test_status_motor_id_comes_from_the_arbitration_field() -> None:
    """Unlike MIT, the servo status payload has no id byte - all 8 are data."""
    fb = sv.decode(status_frame(motor_id=42), SCALING)
    assert isinstance(fb, ServoFeedback)
    assert fb.motor_id == 42


@pytest.mark.parametrize("n", [0, 1, 4, 7, 9])
def test_status_wrong_length_is_rejected(n: int) -> None:
    with pytest.raises(MalformedFrame, match="8 bytes"):
        sv.decode_status(bytes(n), SCALING)


@pytest.mark.parametrize("code", [0, 1, 6, 7, 8, 255])
def test_every_fault_byte_decodes(code: int) -> None:
    fb = sv.decode_status(bytes([0, 0, 0, 0, 0, 0, 30, code]), SCALING)
    assert fb.fault_code == code
