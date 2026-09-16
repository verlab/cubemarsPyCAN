"""Multi-turn unwrapping, tested against both firmware behaviours.

The manual does not say whether position wraps or saturates past the field limit, so the
unwrapper refuses by default and is verified against a simulator configured each way.
"""

from __future__ import annotations

import pytest

from cubemarspycan import SpecIncompleteError, get_spec
from cubemarspycan.unwrap import TurnCounter, WrapMode

FIELD = get_spec("AK40-10").mit.position  # +/-12.5 rad


def test_refuses_until_told_which_behaviour_to_expect() -> None:
    counter = TurnCounter(FIELD)
    assert counter.mode is WrapMode.UNKNOWN
    with pytest.raises(SpecIncompleteError, match="wraps or saturates"):
        counter.update(0.0)


def test_refusal_explains_how_to_settle_it() -> None:
    with pytest.raises(SpecIncompleteError, match="bench"):
        TurnCounter(FIELD).update(0.0)


def test_passes_through_inside_the_field() -> None:
    # Steps stay under half a span (12.5 rad), which is the detector's precondition.
    counter = TurnCounter(FIELD, WrapMode.WRAP)
    for value in (0.0, 1.0, -1.0, 8.0, 12.0, 4.0, 0.0, -8.0, -12.0, 0.0):
        assert counter.update(value) == pytest.approx(value)
    assert counter.turns == 0


def test_counts_a_forward_wrap() -> None:
    counter = TurnCounter(FIELD, WrapMode.WRAP)
    counter.update(12.4)
    assert counter.update(-12.4) == pytest.approx(12.6)
    assert counter.turns == 1
    assert counter.update(-11.5) == pytest.approx(13.5)


def test_counts_a_backward_wrap() -> None:
    counter = TurnCounter(FIELD, WrapMode.WRAP)
    counter.update(-12.4)
    assert counter.update(12.4) == pytest.approx(-12.6)
    assert counter.turns == -1


def test_many_turns_in_both_directions() -> None:
    counter = TurnCounter(FIELD, WrapMode.WRAP)
    span = FIELD.span
    continuous = 0.0
    counter.update(0.0)
    for step in [2.0] * 40 + [-2.0] * 80 + [2.0] * 40:
        continuous += step
        folded = ((continuous - FIELD.lo) % span) + FIELD.lo
        assert counter.update(folded) == pytest.approx(continuous, abs=1e-9)
    assert counter.turns == 0


def test_saturate_mode_passes_through_and_flags_the_rail() -> None:
    """When the driver clamps, true position is simply not on the wire."""
    # update() mutates saturation_seen, so read it into a local each time rather than
    # letting a type checker narrow the property across the call.
    counter = TurnCounter(FIELD, WrapMode.SATURATE)
    assert counter.update(1.0) == pytest.approx(1.0)
    before: bool = counter.saturation_seen
    assert not before
    assert counter.update(12.5) == pytest.approx(12.5)
    after: bool = counter.saturation_seen
    assert after
    assert counter.turns == 0, "saturating firmware gives nothing to count"


def test_saturate_mode_flags_the_negative_rail_too() -> None:
    counter = TurnCounter(FIELD, WrapMode.SATURATE)
    counter.update(-12.5)
    assert counter.saturation_seen


def test_reset_clears_turns_and_history() -> None:
    counter = TurnCounter(FIELD, WrapMode.WRAP)
    counter.update(12.4)
    counter.update(-12.4)
    assert counter.turns == 1
    counter.reset()
    assert counter.turns == 0
    assert counter.update(-12.4) == pytest.approx(-12.4)


def test_first_sample_never_counts_a_wrap() -> None:
    counter = TurnCounter(FIELD, WrapMode.WRAP)
    assert counter.update(12.4) == pytest.approx(12.4)
    assert counter.turns == 0


def test_sampling_faster_than_half_a_span_is_the_precondition() -> None:
    """Undersample and the direction is genuinely ambiguous; this documents the bound.

    Half a span is 12.5 rad and the motor tops out at 45.5 rad/s, so any loop above
    3.6 Hz is safe. Below that, a real forward wrap is indistinguishable from a jump
    backwards - and the unwrapper guesses wrong, as it must.
    """
    counter = TurnCounter(FIELD, WrapMode.WRAP)
    counter.update(0.0)
    assert counter.update(12.4) == pytest.approx(12.4)  # under half a span: fine
    counter2 = TurnCounter(FIELD, WrapMode.WRAP)
    counter2.update(-12.0)
    counter2.update(12.4)  # a 24.4 rad jump: read as a backward wrap
    assert counter2.turns == -1
