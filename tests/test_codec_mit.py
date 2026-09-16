"""MIT codec: byte-exact goldens and per-byte bit-layout assertions.

Round-trip tests alone cannot catch a swapped nibble, because packing and unpacking would
swap it back. Everything here asserts on the actual bytes.
"""

from __future__ import annotations

import pytest

from cubemarspycan import MalformedFrame, get_spec
from cubemarspycan.codec import mit
from cubemarspycan.codec.mit import (
    ENTER_MIT,
    EXIT_MIT,
    TEMPERATURE_OFFSET,
    ZERO_POSITION,
    MitFeedback,
    pack_command,
    unpack_feedback,
)

F = get_spec("AK40-10").mit


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def pack(**kw: float) -> bytes:
    base = {
        "position_rad": 0.0,
        "velocity_radps": 0.0,
        "kp": 0.0,
        "kd": 0.0,
        "torque_nm": 0.0,
    }
    return pack_command(F, **{**base, **kw})


# --- golden vectors, AK40-10 (P +/-12.5, V +/-45.5, T +/-5.0, Kp 0-500, Kd 0-5) ------

GOLDEN = [
    (
        "all minimum",
        {"position_rad": -12.5, "velocity_radps": -45.5, "kp": 0.0, "kd": 0.0, "torque_nm": -5.0},
        "00 00 00 00 00 00 00 00",
    ),
    (
        "all maximum",
        {"position_rad": 12.5, "velocity_radps": 45.5, "kp": 500.0, "kd": 5.0, "torque_nm": 5.0},
        "FF FF FF FF FF FF FF FF",
    ),
    (
        "all centre",
        {"position_rad": 0.0, "velocity_radps": 0.0, "kp": 0.0, "kd": 0.0, "torque_nm": 0.0},
        "80 00 80 00 00 00 08 00",
    ),
    (
        "asymmetric",
        {"position_rad": 1.0, "velocity_radps": -2.0, "kp": 100.0, "kd": 1.0, "torque_nm": 0.5},
        "8A 3D 7A 63 33 33 38 CC",
    ),
    # These two isolate one field each, so a nibble swap between neighbours fails loudly.
    (
        "kp alone at max",
        {"position_rad": -12.5, "velocity_radps": -45.5, "kp": 500.0, "kd": 0.0, "torque_nm": -5.0},
        "00 00 00 0F FF 00 00 00",
    ),
    (
        "kd alone at max",
        {"position_rad": -12.5, "velocity_radps": -45.5, "kp": 0.0, "kd": 5.0, "torque_nm": -5.0},
        "00 00 00 00 00 FF F0 00",
    ),
]


@pytest.mark.parametrize(("name", "kw", "expected"), GOLDEN, ids=[g[0] for g in GOLDEN])
def test_golden_command_bytes(name: str, kw: dict[str, float], expected: str) -> None:
    assert hexs(pack_command(F, **kw)) == expected


def test_position_occupies_bytes_0_and_1_only() -> None:
    b = pack(position_rad=12.5)
    assert b[0] == 0xFF and b[1] == 0xFF
    assert b[2:] == pack(position_rad=-12.5)[2:]


def test_velocity_occupies_byte_2_and_the_high_nibble_of_byte_3() -> None:
    b = pack(velocity_radps=45.5)
    assert b[2] == 0xFF
    assert b[3] & 0xF0 == 0xF0
    assert b[3] & 0x0F == 0x00, "velocity must not reach into Kp's nibble"
    assert b[0:2] == pack()[0:2] and b[4:] == pack()[4:]


def test_kp_occupies_the_low_nibble_of_byte_3_and_byte_4() -> None:
    b = pack(kp=500.0)
    assert b[3] & 0x0F == 0x0F
    assert b[3] & 0xF0 == pack()[3] & 0xF0, "Kp must not disturb velocity's nibble"
    assert b[4] == 0xFF


def test_kd_occupies_byte_5_and_the_high_nibble_of_byte_6() -> None:
    b = pack(kd=5.0)
    assert b[5] == 0xFF
    assert b[6] & 0xF0 == 0xF0
    assert b[6] & 0x0F == pack()[6] & 0x0F, "Kd must not disturb torque's nibble"


def test_torque_occupies_the_low_nibble_of_byte_6_and_byte_7() -> None:
    b = pack(torque_nm=5.0)
    assert b[6] & 0x0F == 0x0F
    assert b[7] == 0xFF
    assert b[6] & 0xF0 == pack()[6] & 0xF0, "torque must not disturb Kd's nibble"


def test_command_is_always_eight_bytes() -> None:
    assert len(pack()) == 8
    assert len(pack(position_rad=12.5, velocity_radps=45.5, kp=500, kd=5, torque_nm=5)) == 8


def test_out_of_range_is_clamped_not_rejected() -> None:
    """The wire cannot express it; refusing here would be a policy decision."""
    assert pack(position_rad=1e6) == pack(position_rad=12.5)
    assert pack(position_rad=-1e6) == pack(position_rad=-12.5)
    assert pack(kp=1e6) == pack(kp=500.0)
    assert pack(kd=-1.0) == pack(kd=0.0)


def test_zero_is_not_exactly_representable() -> None:
    """A symmetric range over an even-sized field has no exact centre. Off by half an LSB."""
    b = pack(position_rad=0.0)
    p = (b[0] << 8) | b[1]
    assert F.position.from_uint(p) == pytest.approx(0.0, abs=F.position.lsb)
    assert F.position.from_uint(p) != 0.0


# --- special-frame collision --------------------------------------------------------


@pytest.mark.parametrize(
    ("special", "name"),
    [(ENTER_MIT, "enter"), (EXIT_MIT, "exit"), (ZERO_POSITION, "zero")],
)
def test_saturated_command_never_collides_with_a_special_frame(special: bytes, name: str) -> None:
    """A saturated command can land exactly on a mode-control payload.

    With p, v, Kp and Kd at maximum, a torque of ~4.9976 Nm packs to FF..FE, which the
    driver reads as "zero the position here" rather than as a torque. A windup against
    the limits can reach that band, and the consequence is a silent mid-motion re-zero.
    """
    torque = F.torque.from_uint(((special[6] & 0x0F) << 8) | special[7])
    packed = pack_command(
        F, position_rad=12.5, velocity_radps=45.5, kp=500.0, kd=5.0, torque_nm=torque
    )
    assert packed != special


def test_collision_avoidance_costs_one_torque_lsb() -> None:
    torque = F.torque.from_uint(((ZERO_POSITION[6] & 0x0F) << 8) | ZERO_POSITION[7])
    packed = pack_command(
        F, position_rad=12.5, velocity_radps=45.5, kp=500.0, kd=5.0, torque_nm=torque
    )
    got = F.torque.from_uint(((packed[6] & 0x0F) << 8) | packed[7])
    # The three special codes are consecutive, so avoidance may need up to three steps.
    assert abs(got - torque) <= 3 * F.torque.lsb + 1e-12
    assert abs(got - torque) < 0.008, "under 8 mNm on an AK40-10"


def test_no_command_on_any_motor_can_produce_a_special_payload() -> None:
    """Sweep the collision-prone corner of the space for every motor in the registry."""
    from cubemarspycan.codec.mit import _SPECIAL_PAYLOADS
    from cubemarspycan.registry import MODEL_MIT_FIELDS

    for fields in MODEL_MIT_FIELDS.values():
        for t_uint in range(4090, 4096):
            torque = fields.torque.from_uint(t_uint)
            packed = pack_command(
                fields,
                position_rad=fields.position.hi,
                velocity_radps=fields.velocity.hi,
                kp=fields.kp.hi,
                kd=fields.kd.hi,
                torque_nm=torque,
            )
            assert packed not in _SPECIAL_PAYLOADS


# --- special frames -----------------------------------------------------------------


def test_special_payloads_match_the_manual() -> None:
    assert hexs(ENTER_MIT) == "FF FF FF FF FF FF FF FC"
    assert hexs(EXIT_MIT) == "FF FF FF FF FF FF FF FD"
    assert hexs(ZERO_POSITION) == "FF FF FF FF FF FF FF FE"


def test_special_frames_are_standard_and_addressed_to_the_motor() -> None:
    for build, payload in (
        (mit.enter_mit_frame, ENTER_MIT),
        (mit.exit_mit_frame, EXIT_MIT),
        (mit.zero_position_frame, ZERO_POSITION),
    ):
        f = build(3)
        assert f.arbitration_id == 3
        assert not f.is_extended_id
        assert f.data == payload
        assert f.dlc == 8


def test_is_special() -> None:
    assert mit.is_special(ENTER_MIT)
    assert mit.is_special(ZERO_POSITION)
    assert not mit.is_special(pack())


def test_command_frame_is_standard() -> None:
    f = mit.command_frame(F, 7, position_rad=0.0, velocity_radps=0.0, kp=0.0, kd=0.0, torque_nm=0.0)
    assert f.arbitration_id == 7
    assert not f.is_extended_id
    assert f.dlc == 8


# --- decode -------------------------------------------------------------------------


def test_decode_hand_built_reply() -> None:
    # id 1, position mid, velocity mid, torque mid, 25 C, no fault
    data = bytes([0x01, 0x80, 0x00, 0x80, 0x00, 0x00, 25 + TEMPERATURE_OFFSET, 0x00])
    fb = unpack_feedback(F, data)
    assert isinstance(fb, MitFeedback)
    assert fb.motor_id == 1
    assert fb.position_rad == pytest.approx(0.0, abs=F.position.lsb)
    assert fb.velocity_radps == pytest.approx(0.0, abs=F.velocity.lsb)
    assert fb.temperature_c == 25
    assert fb.fault_code == 0


def test_decode_extremes() -> None:
    lo = unpack_feedback(F, bytes([0x01, 0, 0, 0, 0, 0, 0, 0]))
    assert lo.position_rad == pytest.approx(-12.5)
    assert lo.velocity_radps == pytest.approx(-45.5)
    assert lo.torque_nm == pytest.approx(-5.0)
    hi = unpack_feedback(F, bytes([0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00]))
    assert hi.position_rad == pytest.approx(12.5)
    assert hi.velocity_radps == pytest.approx(45.5)
    assert hi.torque_nm == pytest.approx(5.0)


@pytest.mark.parametrize(("raw", "celsius"), [(0, -40), (40, 0), (255, 215), (65, 25)])
def test_temperature_offset(raw: int, celsius: int) -> None:
    """Byte 6 carries temperature + 40, giving -40..215 C."""
    fb = unpack_feedback(F, bytes([1, 0, 0, 0, 0, 0, raw, 0]))
    assert fb.temperature_c == celsius


@pytest.mark.parametrize("code", [0, 1, 6, 7, 8, 255])
def test_every_fault_byte_decodes(code: int) -> None:
    assert unpack_feedback(F, bytes([1, 0, 0, 0, 0, 0, 60, code])).fault_code == code


def test_velocity_and_torque_share_byte_4_correctly() -> None:
    """Byte 4 is velocity's low nibble over torque's high nibble."""
    fb = unpack_feedback(F, bytes([1, 0, 0, 0x00, 0xF0, 0x00, 60, 0]))
    assert fb.velocity_radps == pytest.approx(F.velocity.from_uint(0x00F))
    assert fb.torque_nm == pytest.approx(F.torque.from_uint(0x000))
    fb2 = unpack_feedback(F, bytes([1, 0, 0, 0x00, 0x0F, 0xFF, 60, 0]))
    assert fb2.velocity_radps == pytest.approx(F.velocity.from_uint(0x000))
    assert fb2.torque_nm == pytest.approx(F.torque.from_uint(0xFFF))


@pytest.mark.parametrize("n", [0, 1, 5, 6, 7, 9, 16])
def test_wrong_length_is_rejected(n: int) -> None:
    with pytest.raises(MalformedFrame, match="8 bytes"):
        unpack_feedback(F, bytes(n))


def test_command_decode_round_trip() -> None:
    """Pack then decode the same bit layout; values survive to within one LSB."""
    cmd = pack_command(F, position_rad=3.0, velocity_radps=-10.0, kp=50.0, kd=2.0, torque_nm=1.5)
    # A reply frame has the id in byte 0, so re-lay the command bytes to match.
    fb = unpack_feedback(F, bytes([1, cmd[0], cmd[1], cmd[2], cmd[3] & 0xF0, 0x00, 60, 0]))
    assert fb.position_rad == pytest.approx(3.0, abs=F.position.lsb)


def test_no_field_can_encode_an_exact_zero() -> None:
    """Each MIT field is a symmetric range over an even-sized field, so its midpoint sits
    half an LSB above zero. Worth pinning: a "zero torque" safe stop is a floor, not a null.

    On an AK40-10 that floor is +1.22 mNm, against a 60 mNm break-away torque - 2%, so the
    output cannot move, but a frictionless model will drift under it.
    """
    for field in (F.position, F.velocity, F.torque):
        decoded = field.from_uint(field.to_uint(0.0))
        assert decoded != 0.0
        assert abs(decoded) == pytest.approx(field.lsb / 2, rel=1e-6)
    assert F.torque.from_uint(F.torque.to_uint(0.0)) == pytest.approx(1.221e-3, rel=1e-3)
