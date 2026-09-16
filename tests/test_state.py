"""Immutable state snapshots."""

from __future__ import annotations

import dataclasses
from dataclasses import replace
from typing import Any

import pytest

from cubemarspycan import (
    FaultEvent,
    MitState,
    ServoEvent,
    ServoStatus,
    SpecIncompleteError,
    UnresolvedFrameError,
    get_spec,
)
from cubemarspycan.faults import CanFault
from cubemarspycan.spec import Side, Source, Sourced

AK40 = get_spec("AK40-10")
AK70 = get_spec("AK70-10")  # skeleton: no datasheet constants


def mit(**kw: Any) -> MitState:
    base = {
        "spec": AK40,
        "position_rad": 1.0,
        "velocity_radps": 2.0,
        "torque_nm": 0.5,
        "temperature_c": 30,
        "fault_code": 0,
        "rx_monotonic": 123.0,
        "seq": 1,
    }
    return MitState(**{**base, **kw})


def servo(**kw: Any) -> ServoStatus:
    base = {
        "spec": AK40,
        "position_deg": 90.0,
        "velocity_erpm": 1000.0,
        "current_a": 1.5,
        "temperature_c": 30,
        "fault_code": 0,
        "rx_monotonic": 123.0,
        "seq": 1,
    }
    return ServoStatus(**{**base, **kw})


# --- the frozen invariant the latch depends on --------------------------------------


@pytest.mark.parametrize("cls", [MitState, ServoStatus, ServoEvent, FaultEvent])
def test_state_classes_are_frozen_with_slots(cls: Any) -> None:
    """Load-bearing: this is what makes publishing a reference equivalent to a copy."""
    params = cls.__dataclass_params__
    assert params.frozen, f"{cls.__name__} must be frozen"
    assert getattr(cls, "__slots__", None) is not None, f"{cls.__name__} must use slots"


def test_states_cannot_be_mutated() -> None:
    s = mit()
    with pytest.raises(AttributeError):
        s.position_rad = 9.0  # type: ignore[misc]


def test_replace_makes_a_new_object() -> None:
    a = mit()
    b = replace(a, position_rad=2.0)
    assert a.position_rad == 1.0 and b.position_rad == 2.0
    assert a is not b


# --- faults -------------------------------------------------------------------------


def test_clean_state_reports_no_fault() -> None:
    s = mit()
    assert not s.is_faulted
    assert s.fault is CanFault.NONE
    assert s.fault_text == "no fault"


def test_fault_code_7_does_not_explode() -> None:
    s = mit(fault_code=7)
    assert s.is_faulted
    assert s.fault is CanFault.MOTOR_STALL


def test_unknown_fault_code_is_reported_not_raised() -> None:
    s = mit(fault_code=200)
    assert s.is_faulted
    assert s.fault is None
    assert "200" in s.fault_text


def test_fault_event_from_code() -> None:
    ev = FaultEvent.from_code(6, "mit", 1.0, 3)
    assert ev.fault is CanFault.MOSFET_OVER_TEMPERATURE
    assert "MOSFET" in str(ev)
    assert ev.source == "mit"


def test_fault_event_tolerates_unknown_codes() -> None:
    ev = FaultEvent.from_code(99, "servo", 1.0, 1)
    assert ev.fault is None
    assert "99" in ev.text


# --- gearbox sides ------------------------------------------------------------------


def test_mit_rotor_side_applies_the_gear_ratio() -> None:
    s = mit(position_rad=1.0, velocity_radps=2.0)
    assert s.position_rotor_rad == pytest.approx(10.0)
    assert s.velocity_rotor_radps == pytest.approx(20.0)


def test_mit_position_in_degrees() -> None:
    assert mit(position_rad=3.141592653589793).position_deg == pytest.approx(180.0)


def test_rotor_side_refuses_without_a_gear_ratio() -> None:
    bare = AK40.evolve(
        drivetrain=replace(AK40.drivetrain, gear_ratio=Sourced(None, Source.UNKNOWN))
    )
    with pytest.raises(SpecIncompleteError):
        _ = mit(spec=bare).position_rotor_rad
    with pytest.raises(SpecIncompleteError):
        _ = mit(spec=bare).velocity_rotor_radps


# --- derived values are honest ------------------------------------------------------


def test_estimated_current_warns_while_kt_is_not_measured() -> None:
    from cubemarspycan import state as state_mod

    state_mod._kt_warned.discard(AK40.name)
    with pytest.warns(UserWarning, match="estimate"):
        i = mit(torque_nm=4.1).estimated_current_a
    assert i == pytest.approx(7.32, abs=0.02)


def test_estimated_current_warns_only_once_per_spec() -> None:
    from cubemarspycan import state as state_mod

    state_mod._kt_warned.discard(AK40.name)
    with pytest.warns(UserWarning):
        _ = mit().estimated_current_a
    import warnings

    with warnings.catch_warnings():
        warnings.simplefilter("error")
        _ = mit().estimated_current_a  # must not warn again


def test_estimated_current_is_silent_once_kt_is_measured() -> None:
    import warnings

    measured = AK40.evolve(
        drivetrain=replace(AK40.drivetrain, kt_nm_per_a=Sourced(0.056, Source.MEASURED, "bench"))
    )
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert mit(spec=measured, torque_nm=1.0).estimated_current_a > 0


# --- servo --------------------------------------------------------------------------


def test_servo_raw_wire_values_always_work() -> None:
    s = servo()
    assert s.position_deg == 90.0
    assert s.velocity_erpm == 1000.0
    assert s.current_a == 1.5
    assert not s.is_faulted


def test_servo_velocity_converts_with_known_constants() -> None:
    assert servo(velocity_erpm=1000.0).velocity_radps == pytest.approx(0.74800, abs=1e-5)


def test_servo_velocity_refuses_on_a_skeleton_spec() -> None:
    with pytest.raises(SpecIncompleteError, match="pole pairs"):
        _ = servo(spec=AK70).velocity_radps


def test_servo_output_angle_refuses_while_the_side_is_unknown() -> None:
    """position_deg stays available; only the gearbox-referred value refuses."""
    s = servo()
    assert s.position_deg == 90.0
    with pytest.raises(UnresolvedFrameError, match="B7"):
        _ = s.output_rad


def test_servo_output_angle_once_measured_output_side() -> None:
    spec = AK40.evolve(
        servo=replace(
            AK40.servo,
            position_side=Sourced(Side.OUTPUT, Source.MEASURED, "bench B7"),
        )
    )
    assert servo(spec=spec).output_rad == pytest.approx(1.5707963, abs=1e-6)


def test_servo_output_angle_once_measured_rotor_side() -> None:
    spec = AK40.evolve(
        servo=replace(
            AK40.servo,
            position_side=Sourced(Side.ROTOR, Source.MEASURED, "bench B7"),
        )
    )
    assert servo(spec=spec).output_rad == pytest.approx(0.15707963, abs=1e-7)


def test_servo_event_is_not_state() -> None:
    ev = ServoEvent("servo_mode_ack", 0x2C, b"\xfa\xfb\xfc\xfd", 1.0)
    assert not hasattr(ev, "position_deg")
    assert "servo_mode_ack" in str(ev)
    assert "2C" in str(ev)


# --- rendering ----------------------------------------------------------------------


def test_str_renders_without_touching_unknown_constants() -> None:
    assert "rad" in str(mit())
    assert "ERPM" in str(servo())
    assert "no fault" in str(mit())


def test_state_field_names_are_unit_bearing() -> None:
    """Every float field says its unit, so a caller cannot mistake rad for deg."""
    for cls in (MitState, ServoStatus):
        for f in dataclasses.fields(cls):
            if f.type in ("float", float) and f.name not in {"rx_monotonic"}:
                assert any(
                    f.name.endswith(u)
                    for u in ("_rad", "_radps", "_nm", "_deg", "_erpm", "_a", "_c")
                ), f"{cls.__name__}.{f.name} does not name its unit"


def test_servo_fault_enum_and_text() -> None:
    assert servo(fault_code=0).fault is CanFault.NONE
    assert servo(fault_code=2).fault is CanFault.OVER_CURRENT
    assert servo(fault_code=2).fault_text == "over-current"
    assert servo(fault_code=250).fault is None
    assert servo(fault_code=250).is_faulted
