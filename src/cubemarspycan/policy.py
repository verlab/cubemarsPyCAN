"""Caller-chosen safety behaviour.

Policy is deliberately *not* part of :class:`~cubemarspycan.spec.MotorSpec`. A spec holds
sourced facts about a motor; a policy holds decisions about how your application wants to
treat them. Mixing the two is how TMotorCANControl ended up with an empirical 0.59
current fudge factor - measured once for one AK80-9 - baked into the constants of every
motor it supports.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class ClampMode(Enum):
    """What to do with a command outside the usable range."""

    EFFECTIVE = "effective"
    """Clamp to ``min(wire field, physical limit)`` per field. The default."""
    FIELD = "field"
    """Clamp only to what the wire can express. Lets you exceed the motor's rating."""
    RAISE = "raise"
    """Refuse out-of-range commands instead of clamping."""


class FaultAction(Enum):
    """What ``update()`` does when the driver reports a non-zero fault code."""

    RAISE = "raise"
    """Send a safe-stop frame, then raise MotorFault on the control thread. Default."""
    WARN = "warn"
    """Emit a warning and keep going. For diagnostics only."""
    IGNORE = "ignore"
    """Latch it for inspection and say nothing."""


@dataclass(frozen=True, slots=True)
class ClampReport:
    """What a command had to be changed to before it could be sent."""

    field_name: str
    requested: float
    applied: float
    limit: float
    reason: str

    @property
    def clamped(self) -> bool:
        return self.requested != self.applied

    def __str__(self) -> str:
        return (
            f"{self.field_name}: requested {self.requested:g}, sent {self.applied:g} "
            f"({self.reason} {self.limit:g})"
        )


@dataclass(frozen=True, slots=True)
class SafetyPolicy:
    """Limits and reactions chosen by the caller, not read off the motor."""

    max_temp_c: float = 75.0
    """Board temperature above which update() stops. The manual allows 100 C."""
    clamp: ClampMode = ClampMode.EFFECTIVE
    on_fault: FaultAction = FaultAction.RAISE
    stale_warn_s: float = 0.1
    """No feedback for this long while commanding -> warn."""
    stale_fatal_s: float = 0.5
    """No feedback for this long -> safe-stop and raise StaleFeedbackError."""
    current_ceiling_a: float | None = None
    """Hard ceiling for servo current commands.

    The servo current field spans +/-60 A while an AK40-10 peaks at 7.3 A, so a typo can
    ask for eight times the motor's rating. Mandatory for any variant whose
    ``limits.peak_current_a`` is still unknown; otherwise it tightens that value.
    """
    supply_voltage_v: float | None = None
    """Declared supply. Used only to warn when the motor can outrun the velocity field."""
    warn_on_estimates: bool = True

    forbidden: frozenset[str] = field(default_factory=frozenset)
    """Named operations to refuse outright, e.g. {"permanent_zero"}."""


DEFAULT_POLICY = SafetyPolicy()
