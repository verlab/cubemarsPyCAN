"""The motor registry.

Two tiers, because they have different provenance and change for different reasons:

* **Per model** - the MIT field ranges, straight from manual v1.0.18 p.63. Authoritative
  for all ten models and shared by every variant of a model.
* **Per variant** - drivetrain, physical limits and capabilities, from the CubeMars
  product datasheets. KV and hardware revision change Kt, pole pairs and even encoder
  count, so "AK80-9" alone does not identify a set of constants.

Where a datasheet has not been consulted, the constant is ``unknown()`` and the library
refuses the conversion that needs it. Do not fill these in from TMotorCANControl: its
table is measurably wrong (it lists AK80-9 Kt as 0.091/0.115 against a datasheet 0.095,
and AK10-9 as 0.16/0.206 against 0.198) and several entries are commented
``UNTESTED CONSTANT!``.
"""

from __future__ import annotations

from .spec import (
    SERVO_CAN_COMMON,
    Capabilities,
    Drivetrain,
    FieldRange,
    MitFields,
    MotorSpec,
    PhysicalLimits,
    Side,
    Source,
    Sourced,
    WrapMode,
    unknown,
)
from .units import rpm_to_radps

_P = FieldRange(-12.5, 12.5, 16)
_KP = FieldRange(0.0, 500.0, 12)
_KD = FieldRange(0.0, 5.0, 12)
_MANUAL_P63 = "manual v1.0.18 p.63"


def _mit(v_max: float, t_max: float) -> MitFields:
    """Build the five MIT fields. Position, Kp and Kd are identical for every AK model."""
    return MitFields(
        position=_P,
        velocity=FieldRange(-v_max, v_max, 12),
        torque=FieldRange(-t_max, t_max, 12),
        kp=_KP,
        kd=_KD,
    )


MODEL_MIT_FIELDS: dict[str, MitFields] = {
    "AK10-9": _mit(50.0, 65.0),
    "AK60-6": _mit(45.0, 15.0),
    "AK70-10": _mit(50.0, 25.0),
    "AK80-6": _mit(76.0, 12.0),
    "AK80-9": _mit(50.0, 18.0),
    "AK80-64": _mit(8.0, 144.0),
    "AK80-8": _mit(37.5, 32.0),
    "AK45-36": _mit(6.0, 34.0),
    "AK45-10": _mit(20.0, 8.0),
    "AK40-10": _mit(45.5, 5.0),
}
"""MIT command/feedback field scaling for every model in manual v1.0.18 p.63."""

MODEL_GEAR_RATIO: dict[str, float] = {
    "AK10-9": 9.0,
    "AK60-6": 6.0,
    "AK70-10": 10.0,
    "AK80-6": 6.0,
    "AK80-9": 9.0,
    "AK80-64": 64.0,
    "AK80-8": 8.0,
    "AK45-36": 36.0,
    "AK45-10": 10.0,
    "AK40-10": 10.0,
}
"""Gear ratio read off the model name. TMotorCANControl has AK80-64 as 80:1; it is 64:1."""

_NO_DATASHEET = "no datasheet consulted yet; fetch the spec table from cubemars.com"


def _nameplate_gear(model: str) -> Sourced[float]:
    return Sourced(MODEL_GEAR_RATIO[model], Source.NAMEPLATE, ref=f"model name {model}")


def _skeleton(model: str) -> MotorSpec:
    """A spec with authoritative wire scaling and everything physical still unknown."""
    return MotorSpec(
        name=model,
        model=model,
        mit=MODEL_MIT_FIELDS[model],
        drivetrain=Drivetrain(
            gear_ratio=_nameplate_gear(model),
            pole_pairs=unknown(_NO_DATASHEET),
            kt_nm_per_a=unknown(_NO_DATASHEET),
        ),
        servo=SERVO_CAN_COMMON,
        notes="Wire scaling is authoritative; physical constants are not yet populated.",
    )


# --- datasheet-verified variants ----------------------------------------------------

_DS_AK40_10 = "cubemars.com AK40-10 KV170 spec table"

AK40_10_KV170 = MotorSpec(
    name="AK40-10-KV170",
    model="AK40-10",
    mit=MODEL_MIT_FIELDS["AK40-10"],
    drivetrain=Drivetrain(
        gear_ratio=Sourced(10.0, Source.DATASHEET, _DS_AK40_10, "reduction ratio 10:1"),
        pole_pairs=Sourced(14, Source.DATASHEET, _DS_AK40_10),
        kt_nm_per_a=Sourced(0.056, Source.DATASHEET, _DS_AK40_10, "rotor-side"),
        kv_rpm_per_v=Sourced(170.0, Source.DATASHEET, _DS_AK40_10),
        ke_v_per_krpm=Sourced(5.88, Source.DATASHEET, _DS_AK40_10),
    ),
    limits=PhysicalLimits(
        peak_torque_nm=Sourced(4.1, Source.DATASHEET, _DS_AK40_10),
        rated_torque_nm=Sourced(1.3, Source.DATASHEET, _DS_AK40_10),
        peak_current_a=Sourced(7.3, Source.DATASHEET, _DS_AK40_10),
        rated_current_a=Sourced(2.7, Source.DATASHEET, _DS_AK40_10),
        no_load_speed_radps=Sourced(
            rpm_to_radps(435.0),
            Source.DATASHEET,
            _DS_AK40_10,
            "435 rpm is internally inconsistent with the same datasheet's Kv=170 and "
            "Ke=5.88, which both give 408 rpm at the rated 24 V. 435 rpm implies "
            "Kv=181 or a 25.6 V supply. Treat 435 as the optimistic bound",
        ),
        rated_speed_radps=Sourced(rpm_to_radps(370.0), Source.DATASHEET, _DS_AK40_10),
        rated_voltage_v=Sourced(24.0, Source.DATASHEET, _DS_AK40_10),
        max_board_temp_c=Sourced(100.0, Source.MANUAL, "manual v1.0.18 p.9"),
    ),
    capabilities=Capabilities(
        encoders=1,
        inner_encoder_bits=Sourced(14, Source.DATASHEET, _DS_AK40_10),
    ),
    mit_wrap_mode=Sourced(
        WrapMode.WRAP,
        Source.MEASURED,
        "bench 2026-09-16, gs_usb + socketcan can0",
        "driven past the limit at 3 rad/s, the reading jumped +12.4985 -> -12.4863 rad",
    ),
    mit_position_side=Sourced(
        Side.OUTPUT,
        Source.MEASURED,
        "bench 2026-09-16, gs_usb + socketcan can0",
        "one hand-turn of the output shaft read 6.3867 rad against 6.2832 expected "
        "for output-side and 62.83 for rotor-side",
    ),
    notes=(
        "Single encoder: permanent zero (origin mode 1) is refused.\n"
        "The MIT velocity field (45.5 rad/s = 434.5 rpm) matches the datasheet no-load "
        "speed of 435 rpm to within 0.1%, which is what confirms MIT position and "
        "velocity are output-side rather than rotor-side.\n"
        "That same match means the field was sized to the no-load speed and there is no "
        "designed-in headroom: at 24 V the Kv/Ke-derived no-load of 408 rpm leaves ~6%, "
        "but the datasheet's own 435 rpm figure sits 0.1% ABOVE the field maximum. "
        "Under power at 24 V feedback should not saturate; back-driving will exceed it."
    ),
)

_DS_AK10_9 = "cubemars.com AK10-9 V2.0 KV60 spec table"

AK10_9_V2_KV60 = MotorSpec(
    name="AK10-9-V2.0-KV60",
    model="AK10-9",
    mit=MODEL_MIT_FIELDS["AK10-9"],
    drivetrain=Drivetrain(
        gear_ratio=Sourced(9.0, Source.DATASHEET, _DS_AK10_9),
        pole_pairs=Sourced(21, Source.DATASHEET, _DS_AK10_9),
        kt_nm_per_a=Sourced(0.198, Source.DATASHEET, _DS_AK10_9, "rotor-side"),
        kv_rpm_per_v=Sourced(60.0, Source.DATASHEET, _DS_AK10_9),
        ke_v_per_krpm=Sourced(17.2, Source.DATASHEET, _DS_AK10_9),
    ),
    limits=PhysicalLimits(
        peak_torque_nm=Sourced(48.0, Source.DATASHEET, _DS_AK10_9),
        rated_torque_nm=Sourced(18.0, Source.DATASHEET, _DS_AK10_9),
        peak_current_a=Sourced(29.8, Source.DATASHEET, _DS_AK10_9),
        rated_current_a=Sourced(10.6, Source.DATASHEET, _DS_AK10_9),
        no_load_speed_radps=Sourced(
            rpm_to_radps(320.0), Source.DATASHEET, _DS_AK10_9, "at 48 V; 160 rpm at 24 V"
        ),
        rated_speed_radps=Sourced(
            rpm_to_radps(228.0), Source.DATASHEET, _DS_AK10_9, "at 48 V; 109 rpm at 24 V"
        ),
        rated_voltage_v=Sourced(48.0, Source.DATASHEET, _DS_AK10_9, "datasheet lists 24/48 V"),
    ),
    capabilities=Capabilities(
        encoders=2,
        inner_encoder_bits=Sourced(14, Source.DATASHEET, _DS_AK10_9),
        outer_encoder_bits=Sourced(15, Source.DATASHEET, _DS_AK10_9, "magnetic"),
    ),
    notes="Dual encoder: permanent zero (origin mode 1) is supported.",
)

_DS_AK80_9 = "cubemars.com AK80-9 V3.0 KV100 spec table"

AK80_9_V3_KV100 = MotorSpec(
    name="AK80-9-V3.0-KV100",
    model="AK80-9",
    mit=MODEL_MIT_FIELDS["AK80-9"],
    drivetrain=Drivetrain(
        gear_ratio=Sourced(9.0, Source.DATASHEET, _DS_AK80_9),
        pole_pairs=Sourced(21, Source.DATASHEET, _DS_AK80_9),
        kt_nm_per_a=Sourced(0.095, Source.DATASHEET, _DS_AK80_9, "rotor-side"),
        kv_rpm_per_v=Sourced(100.0, Source.DATASHEET, _DS_AK80_9),
        ke_v_per_krpm=Sourced(10.0, Source.DATASHEET, _DS_AK80_9),
    ),
    limits=PhysicalLimits(
        peak_torque_nm=Sourced(22.0, Source.DATASHEET, _DS_AK80_9),
        rated_torque_nm=Sourced(9.0, Source.DATASHEET, _DS_AK80_9),
        peak_current_a=Sourced(28.0, Source.DATASHEET, _DS_AK80_9),
        rated_current_a=Sourced(12.0, Source.DATASHEET, _DS_AK80_9),
        no_load_speed_radps=Sourced(rpm_to_radps(570.0), Source.DATASHEET, _DS_AK80_9),
        rated_speed_radps=Sourced(rpm_to_radps(390.0), Source.DATASHEET, _DS_AK80_9),
        rated_voltage_v=Sourced(48.0, Source.DATASHEET, _DS_AK80_9),
    ),
    capabilities=Capabilities(
        encoders=1,
        inner_encoder_bits=Sourced(16, Source.DATASHEET, _DS_AK80_9),
    ),
    notes=(
        "The MIT torque field (+/-18 N*m) is NARROWER than the 22 N*m peak, the opposite "
        "sense to the AK40-10. This is why field range and physical limit are separate "
        "types rather than one number."
    ),
)

_DS_AK80_8 = "cubemars.com AK80-8 KV60 spec table"

AK80_8_KV60 = MotorSpec(
    name="AK80-8-KV60",
    model="AK80-8",
    mit=MODEL_MIT_FIELDS["AK80-8"],
    drivetrain=Drivetrain(
        gear_ratio=Sourced(8.0, Source.DATASHEET, _DS_AK80_8),
        pole_pairs=Sourced(21, Source.DATASHEET, _DS_AK80_8),
        kt_nm_per_a=Sourced(0.199, Source.DATASHEET, _DS_AK80_8, "rotor-side"),
        kv_rpm_per_v=Sourced(60.0, Source.DATASHEET, _DS_AK80_8),
    ),
    limits=PhysicalLimits(
        peak_torque_nm=Sourced(25.0, Source.DATASHEET, _DS_AK80_8),
        rated_torque_nm=Sourced(10.0, Source.DATASHEET, _DS_AK80_8),
        peak_current_a=Sourced(21.0, Source.DATASHEET, _DS_AK80_8),
        rated_current_a=Sourced(6.9, Source.DATASHEET, _DS_AK80_8),
        no_load_speed_radps=Sourced(rpm_to_radps(360.0), Source.DATASHEET, _DS_AK80_8),
        rated_voltage_v=Sourced(48.0, Source.DATASHEET, _DS_AK80_8),
    ),
    capabilities=Capabilities(
        encoders=2,
        outer_encoder_bits=Sourced(15, Source.DATASHEET, _DS_AK80_8, "magnetic"),
    ),
    notes="Dual encoder: permanent zero (origin mode 1) is supported.",
)


_VARIANTS: tuple[MotorSpec, ...] = (
    AK40_10_KV170,
    AK10_9_V2_KV60,
    AK80_9_V3_KV100,
    AK80_8_KV60,
)

SPECS: dict[str, MotorSpec] = {v.name: v for v in _VARIANTS}
"""Every known spec, by variant key and by model alias."""

# Model-name aliases. A model with a verified variant resolves to it; the rest resolve to
# a skeleton whose wire scaling is authoritative and whose physical constants refuse.
# setdefault so that a variant named exactly after its model is never shadowed.
for _model in MODEL_MIT_FIELDS:
    _matching = [v for v in _VARIANTS if v.model == _model]
    SPECS.setdefault(_model, _matching[0] if _matching else _skeleton(_model))
del _model


def get(name: str) -> MotorSpec:
    """Look up a spec by variant key or model name, with a helpful error."""
    try:
        return SPECS[name]
    except KeyError:
        raise KeyError(f"unknown motor {name!r}. Known: {', '.join(sorted(SPECS))}") from None


def models() -> list[str]:
    """Model names as the manual's field-range table spells them."""
    return sorted(MODEL_MIT_FIELDS)


def variants() -> list[str]:
    """Variant keys that carry datasheet-verified physical constants."""
    return sorted(v.name for v in _VARIANTS)
