"""The spec data model and the registry.

Two things are being pinned here: that the numbers match the sources they claim, and that
an unknown constant *refuses* rather than producing a plausible wrong answer.
"""

from __future__ import annotations

import pytest

from cubemarspycan import (
    CapabilityError,
    Source,
    Sourced,
    SpecIncompleteError,
    UnresolvedFrameError,
    get_spec,
    models,
    variants,
)
from cubemarspycan.registry import MODEL_GEAR_RATIO, MODEL_MIT_FIELDS, SPECS
from cubemarspycan.spec import Capabilities, FieldRange, Side, unknown
from cubemarspycan.units import radps_to_rpm

# Manual v1.0.18 p.63, all ten models. Position is +/-12.5 rad for every one.
MANUAL_P63 = {
    "AK10-9": (50.0, 65.0),
    "AK60-6": (45.0, 15.0),
    "AK70-10": (50.0, 25.0),
    "AK80-6": (76.0, 12.0),
    "AK80-9": (50.0, 18.0),
    "AK80-64": (8.0, 144.0),
    "AK80-8": (37.5, 32.0),
    "AK45-36": (6.0, 34.0),
    "AK45-10": (20.0, 8.0),
    "AK40-10": (45.5, 5.0),
}


def test_every_manual_model_is_registered() -> None:
    assert set(models()) == set(MANUAL_P63)


@pytest.mark.parametrize(("model", "vt"), MANUAL_P63.items())
def test_mit_fields_match_the_manual(model: str, vt: tuple[float, float]) -> None:
    v_max, t_max = vt
    f = MODEL_MIT_FIELDS[model]
    assert (f.velocity.lo, f.velocity.hi, f.velocity.bits) == (-v_max, v_max, 12)
    assert (f.torque.lo, f.torque.hi, f.torque.bits) == (-t_max, t_max, 12)
    assert (f.position.lo, f.position.hi, f.position.bits) == (-12.5, 12.5, 16)
    assert (f.kp.lo, f.kp.hi, f.kp.bits) == (0.0, 500.0, 12)
    assert (f.kd.lo, f.kd.hi, f.kd.bits) == (0.0, 5.0, 12)


def test_corrections_to_tmotorcancontrol() -> None:
    """Two constants the reference library gets wrong."""
    assert MODEL_MIT_FIELDS["AK60-6"].velocity.hi == 45.0  # reference says 50.0
    assert MODEL_GEAR_RATIO["AK80-64"] == 64.0  # reference says 80.0


@pytest.mark.parametrize("name", sorted(SPECS))
def test_every_spec_is_structurally_valid(name: str) -> None:
    s = SPECS[name]
    assert s.model in MODEL_MIT_FIELDS
    for fld in (s.mit.position, s.mit.velocity, s.mit.torque, s.mit.kp, s.mit.kd):
        assert fld.hi > fld.lo
        assert 1 <= fld.bits <= 32
        assert fld.lsb > 0
    assert s.drivetrain.gear_ratio.known, "gear ratio is always at least nameplate-known"
    assert s.manual_version == "1.0.18"


def test_field_range_rejects_nonsense() -> None:
    with pytest.raises(ValueError, match="empty field range"):
        FieldRange(1.0, 1.0, 12)
    with pytest.raises(ValueError, match="field bits"):
        FieldRange(-1.0, 1.0, 99)


def test_field_range_helpers() -> None:
    f = FieldRange(-12.5, 12.5, 16)
    assert f.span == 25.0
    assert f.max_uint == 65535
    assert f.clamp(99.0) == 12.5
    assert f.clamp(-99.0) == -12.5
    assert f.contains(0.0) and not f.contains(99.0)
    assert f.from_uint(f.to_uint(1.0)) == pytest.approx(1.0, abs=f.lsb)
    assert "16b" in str(f)


# --- AK40-10, the target motor ------------------------------------------------------


def test_ak40_10_datasheet_values() -> None:
    s = get_spec("AK40-10")
    assert s.name == "AK40-10-KV170"
    assert s.drivetrain.gear_ratio.value == 10.0
    assert s.drivetrain.pole_pairs.value == 14
    assert s.drivetrain.kt_nm_per_a.value == 0.056
    assert s.drivetrain.kv_rpm_per_v.value == 170.0
    assert s.limits.peak_torque_nm.value == 4.1
    assert s.limits.peak_current_a.value == 7.3
    assert s.capabilities.encoders == 1


def test_ak40_10_torque_chain_reproduces_the_datasheet() -> None:
    """7.3 A * 0.056 Nm/A * 10 = 4.09, against a published 4.1 Nm peak."""
    s = get_spec("AK40-10")
    assert s.output_torque_from_current(7.3) == pytest.approx(4.1, abs=0.02)
    assert s.current_for_output_torque(4.1) == pytest.approx(7.32, abs=0.02)


def test_ak40_10_velocity_field_equals_datasheet_no_load_speed() -> None:
    """This equality is what proves MIT position/velocity are output-side, not rotor."""
    s = get_spec("AK40-10")
    assert radps_to_rpm(s.mit.velocity.hi) == pytest.approx(435.0, rel=2e-3)
    assert s.mit_position_side.value is Side.OUTPUT


def test_ak40_10_saturation_voltage() -> None:
    s = get_spec("AK40-10")
    assert s.velocity_field_saturation_voltage() == pytest.approx(25.55, abs=0.1)


def test_ak40_10_datasheet_is_internally_inconsistent() -> None:
    """Recorded so it is a known quantity rather than a surprise on the bench.

    Kv=170 and Ke=5.88 both give 408 rpm output at the rated 24 V, but the same datasheet
    states a 435 rpm no-load speed - which implies Kv=181, or a 25.6 V supply. The MIT
    velocity field was evidently sized to the 435 figure, so at 24 V the real headroom
    depends on which number you believe: ~6% on Kv/Ke, or none at all on 435 rpm.
    """
    s = get_spec("AK40-10")
    from_constants = s.no_load_speed_radps_at(24.0)
    stated = s.limits.no_load_speed_radps.require("no-load speed")
    assert radps_to_rpm(from_constants) == pytest.approx(408.0, rel=1e-2)
    assert radps_to_rpm(stated) == pytest.approx(435.0, rel=1e-3)
    assert stated > s.mit.velocity.hi  # the stated figure exceeds the field
    assert from_constants < s.mit.velocity.hi  # the derived one does not
    assert "inconsistent" in s.limits.no_load_speed_radps.note


def test_ak40_10_no_load_at_48v_would_outrun_the_field() -> None:
    s = get_spec("AK40-10")
    assert s.no_load_speed_radps_at(48.0) == pytest.approx(85.45, abs=0.1)
    assert s.no_load_speed_radps_at(48.0) > s.mit.velocity.hi


# --- field range vs physical limit --------------------------------------------------


def test_effective_limits_differ_in_both_directions_across_the_line() -> None:
    """The concrete reason these are separate types."""
    ak40 = get_spec("AK40-10")
    assert ak40.mit.torque.hi == 5.0  # field over-promises
    assert ak40.limits.peak_torque_nm.value == 4.1
    assert ak40.effective_torque_limit_nm() == 4.1  # physical binds

    ak80 = get_spec("AK80-9")
    assert ak80.mit.torque.hi == 18.0  # field under-promises
    assert ak80.limits.peak_torque_nm.value == 22.0
    assert ak80.effective_torque_limit_nm() == 18.0  # field binds


def test_effective_limits_fall_back_to_the_field_when_physical_is_unknown() -> None:
    s = get_spec("AK70-10")  # skeleton, no datasheet consulted
    assert not s.limits.peak_torque_nm.known
    assert s.effective_torque_limit_nm() == s.mit.torque.hi
    assert s.effective_velocity_limit_radps() == s.mit.velocity.hi


def test_effective_velocity_limit_for_ak40_10() -> None:
    s = get_spec("AK40-10")
    assert s.effective_velocity_limit_radps() == 45.5


# --- provenance refuses rather than guessing ----------------------------------------


def test_unknown_constant_refuses_the_conversion() -> None:
    s = get_spec("AK70-10")
    with pytest.raises(SpecIncompleteError, match="pole pairs"):
        s.erpm_to_radps_output(1000.0)
    with pytest.raises(SpecIncompleteError, match="Kt"):
        s.output_torque_from_current(1.0)
    with pytest.raises(SpecIncompleteError, match="Kt"):
        s.current_for_output_torque(1.0)
    with pytest.raises(SpecIncompleteError, match="Kv or Ke"):
        s.no_load_speed_radps_at(24.0)


def test_refusal_says_how_to_fix_it() -> None:
    with pytest.raises(SpecIncompleteError, match="evolve"):
        get_spec("AK70-10").output_torque_from_current(1.0)


def test_sourced_semantics() -> None:
    known = Sourced(1.0, Source.DATASHEET, "ref")
    assert known.known and known.trusted
    assert known.require("x") == 1.0
    assert "datasheet" in str(known)

    guess = Sourced(1.0, Source.ESTIMATED)
    assert guess.known and not guess.trusted

    missing: Sourced[float] = unknown("go measure it")
    assert not missing.known and not missing.trusted
    assert "go measure it" in str(missing)
    with pytest.raises(SpecIncompleteError, match="go measure it"):
        missing.require("x")


def test_evolve_records_a_measurement_and_stays_frozen() -> None:
    s = get_spec("AK70-10")
    from dataclasses import replace

    better = s.evolve(
        drivetrain=replace(
            s.drivetrain, pole_pairs=Sourced(21, Source.MEASURED, "bench 2026-09-20")
        )
    )
    assert better.drivetrain.pole_pairs.value == 21
    assert not s.drivetrain.pole_pairs.known, "the original spec is untouched"
    with pytest.raises(AttributeError):
        better.name = "nope"  # type: ignore[misc]


# --- capabilities -------------------------------------------------------------------


def test_permanent_zero_is_refused_on_single_encoder_motors() -> None:
    ak40 = get_spec("AK40-10")
    assert ak40.capabilities.encoders == 1
    assert not ak40.capabilities.permanent_zero
    with pytest.raises(CapabilityError, match="dual-encoder"):
        ak40.require_permanent_zero()


def test_permanent_zero_is_allowed_on_dual_encoder_motors() -> None:
    for name in ("AK10-9-V2.0-KV60", "AK80-8-KV60"):
        s = get_spec(name)
        assert s.capabilities.encoders == 2
        assert s.capabilities.permanent_zero
        s.require_permanent_zero()  # must not raise


def test_capabilities_default_to_single_encoder() -> None:
    assert not Capabilities().permanent_zero


# --- servo position side stays honest -----------------------------------------------


def test_servo_position_side_is_unknown_everywhere() -> None:
    for name in SPECS:
        assert not SPECS[name].servo.position_side.known, name


def test_servo_scaling_matches_the_manual() -> None:
    sv = get_spec("AK40-10").servo
    assert sv.feedback_deg_per_lsb == 0.1
    assert sv.feedback_erpm_per_lsb == 10.0
    assert sv.feedback_amps_per_lsb == 0.01
    assert sv.position_scale == 10_000.0  # reference library uses 1e6 - 100x wrong
    assert sv.pos_spd_speed_divisor == 10.0  # reference library omits this entirely
    assert sv.pos_spd_accel_divisor == 10.0
    assert sv.current_scale == 1_000.0
    assert sv.duty_scale == 100_000.0


# --- registry plumbing --------------------------------------------------------------


def test_model_alias_resolves_to_the_verified_variant() -> None:
    assert get_spec("AK40-10") is get_spec("AK40-10-KV170")
    assert get_spec("AK80-9") is get_spec("AK80-9-V3.0-KV100")


def test_variants_listed() -> None:
    assert set(variants()) == {
        "AK40-10-KV170",
        "AK10-9-V2.0-KV60",
        "AK80-9-V3.0-KV100",
        "AK80-8-KV60",
    }


def test_unknown_motor_names_the_alternatives() -> None:
    with pytest.raises(KeyError, match="AK40-10"):
        get_spec("AK99-1")


def test_provenance_report_flags_unknowns() -> None:
    report = get_spec("AK70-10").provenance_report()
    assert "unknown" in report
    assert "pole_pairs" in report
    assert get_spec("AK40-10").provenance_report().count("datasheet") >= 10


def test_no_guessed_constants_were_imported_from_the_reference_library() -> None:
    """TMotorCANControl's Kt values are measurably wrong; none may appear here."""
    forbidden = {0.091, 0.115, 0.16, 0.206, 0.068, 0.087, 0.095 + 1e-9, 0.119, 0.153}
    for name, s in SPECS.items():
        kt = s.drivetrain.kt_nm_per_a
        if kt.known:
            assert kt.value not in forbidden, f"{name} carries a reference-library constant"
            assert kt.source is Source.DATASHEET, name


def test_ak80_9_kt_is_the_datasheet_value_not_the_reference_value() -> None:
    kt = get_spec("AK80-9").drivetrain.kt_nm_per_a
    assert kt.value == 0.095  # datasheet
    assert kt.value not in (0.091, 0.115)  # what TMotorCANControl claims


# --- the Kv fallback paths (motors whose datasheet omits Ke) -------------------------


def test_radps_to_erpm_round_trip() -> None:
    s = get_spec("AK40-10")
    assert s.radps_output_to_erpm(s.erpm_to_radps_output(1000.0)) == pytest.approx(1000.0)
    assert s.radps_output_to_erpm(45.5) == pytest.approx(60_832.0, rel=1e-3)


def test_no_load_and_saturation_fall_back_to_kv_when_ke_is_absent() -> None:
    """AK80-8's datasheet lists Kv but no Ke, so both helpers take the Kv branch."""
    s = get_spec("AK80-8-KV60")
    assert not s.drivetrain.ke_v_per_krpm.known
    assert s.drivetrain.kv_rpm_per_v.value == 60.0
    # 60 rpm/V * 48 V = 2880 rotor rpm / 8 = 360 output rpm, matching the datasheet
    assert radps_to_rpm(s.no_load_speed_radps_at(48.0)) == pytest.approx(360.0, rel=1e-3)
    # field 37.5 rad/s = 358.1 rpm output = 2865 rotor rpm -> 47.75 V
    assert s.velocity_field_saturation_voltage() == pytest.approx(47.75, abs=0.1)


def test_saturation_voltage_refuses_without_kv_or_ke() -> None:
    with pytest.raises(SpecIncompleteError, match="Kv or Ke"):
        get_spec("AK70-10").velocity_field_saturation_voltage()


def test_unknown_helper_carries_its_note() -> None:
    u: Sourced[float] = unknown("measure me")
    assert not u.known
    assert u.source is Source.UNKNOWN
    assert u.note == "measure me"


def test_side_enum_values() -> None:
    assert {s.value for s in Side} == {"rotor", "output"}


def test_unresolved_frame_error_is_exported() -> None:
    assert issubclass(UnresolvedFrameError, Exception)


def test_mit_position_side_is_measured_not_inferred_for_the_ak40_10() -> None:
    """Bench 2026-09-16: one hand-turn of the output shaft read 6.3867 rad.

    Output-side predicts 6.2832 (1.6% off, i.e. hand accuracy); rotor-side would have
    been 62.83. The datasheet inference was right, and it is now a measurement.
    """
    side = get_spec("AK40-10").mit_position_side
    assert side.value is Side.OUTPUT
    assert side.source is Source.MEASURED
    assert "6.3867" in side.note


def test_other_models_only_infer_the_mit_position_side() -> None:
    side = get_spec("AK70-10").mit_position_side
    assert side.value is Side.OUTPUT
    assert side.source is Source.ESTIMATED
    assert not side.trusted


def test_wrap_mode_is_measured_for_the_ak40_10() -> None:
    """Bench 2026-09-16: driven past the limit, the reading jumped +12.4985 -> -12.4863.

    That is a wrap, not a saturation, so multi-turn unwrapping is usable on this firmware.
    """
    from cubemarspycan.spec import WrapMode

    mode = get_spec("AK40-10").mit_wrap_mode
    assert mode.value is WrapMode.WRAP
    assert mode.source is Source.MEASURED
    assert "12.4985" in mode.note


def test_wrap_mode_is_unknown_for_unmeasured_models() -> None:
    from cubemarspycan.spec import WrapMode

    mode = get_spec("AK70-10").mit_wrap_mode
    assert not mode.known
    assert mode.value is WrapMode.UNKNOWN
