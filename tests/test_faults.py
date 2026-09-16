"""Fault tables. The CAN and serial enums are different and must stay that way."""

from __future__ import annotations

import pytest

from cubemarspycan.faults import CanFault, SerialFault, describe_can_fault


@pytest.mark.parametrize(
    ("code", "expected"),
    [
        (0, "no fault"),
        (1, "motor over-temperature"),
        (2, "over-current"),
        (3, "over-voltage"),
        (4, "under-voltage"),
        (5, "encoder fault"),
        (6, "MOSFET over-temperature"),
        (7, "motor stall"),
    ],
)
def test_can_fault_table_matches_manual_p45(code: int, expected: str) -> None:
    fault, text = describe_can_fault(code)
    assert fault == CanFault(code)
    assert text == expected


def test_code_7_exists() -> None:
    """TMotorCANControl dies with KeyError: 7 here; v1.0.18 added motor stall."""
    fault, text = describe_can_fault(7)
    assert fault is CanFault.MOTOR_STALL
    assert "stall" in text


def test_code_6_is_mosfet_not_phase_imbalance() -> None:
    """The reference library labels 6 'phase current unbalance'. v1.0.18 says otherwise."""
    assert describe_can_fault(6)[1] == "MOSFET over-temperature"


@pytest.mark.parametrize("code", [8, 9, 255, -1, 1000])
def test_unknown_codes_are_reported_not_raised(code: int) -> None:
    fault, text = describe_can_fault(code)
    assert fault is None
    assert str(code) in text


def test_serial_table_is_genuinely_different() -> None:
    """Mixing the two tables silently mislabels every serial fault."""
    assert SerialFault.OVER_VOLTAGE.value == 1
    assert CanFault.MOTOR_OVER_TEMPERATURE.value == 1
    assert SerialFault(1).name != CanFault(1).name
    assert SerialFault.UNBALANCED_CURRENTS.value == 18
