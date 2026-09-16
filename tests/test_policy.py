"""Caller-chosen safety policy, kept out of the motor spec on purpose."""

from __future__ import annotations

from cubemarspycan import ClampMode, ClampReport, FaultAction, SafetyPolicy
from cubemarspycan.policy import DEFAULT_POLICY


def test_defaults_are_conservative() -> None:
    p = SafetyPolicy()
    assert p.max_temp_c == 75.0  # manual allows 100; we stop earlier
    assert p.clamp is ClampMode.EFFECTIVE
    assert p.on_fault is FaultAction.RAISE
    assert p.stale_fatal_s > p.stale_warn_s
    assert p.current_ceiling_a is None
    assert p.forbidden == frozenset()


def test_policy_is_frozen() -> None:
    import pytest

    with pytest.raises(AttributeError):
        DEFAULT_POLICY.max_temp_c = 200.0  # type: ignore[misc]


def test_clamp_report_describes_what_changed() -> None:
    r = ClampReport("torque_nm", 5.0, 4.1, 4.1, "peak torque")
    assert r.clamped
    assert "requested 5" in str(r)
    assert "peak torque 4.1" in str(r)


def test_clamp_report_knows_when_nothing_changed() -> None:
    assert not ClampReport("torque_nm", 1.0, 1.0, 4.1, "peak torque").clamped


def test_policy_carries_no_motor_facts() -> None:
    """Policy holds decisions; spec holds sourced facts. Mixing them is how the
    reference library ended up with one AK80-9's 0.59 fudge factor on every motor."""
    names = set(SafetyPolicy.__dataclass_fields__)
    assert not names & {"gear_ratio", "pole_pairs", "kt_nm_per_a", "peak_torque_nm"}
