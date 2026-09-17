"""Multi-turn position tracking.

A CAN position field is finite. The AK40-10's MIT field covers +/-12.5 rad, so a little
under two output turns; servo mode covers +/-3200 degrees. Travel past the end and the
reported value either wraps or saturates, and **the manual does not say which**. So this
is opt-in: you tell it which behaviour you observed on the bench, and until you have, the
motor layer does not unwrap at all.

Sampling requirement: wrap detection assumes that between two samples the motor moved less
than half a field span. For the AK40-10 in MIT mode that is 12.5 rad at up to 45.5 rad/s,
so any loop faster than 3.6 Hz is safe - a wide margin, but it is a real precondition and
it is why this class refuses to guess when a sample is missed.
"""

from __future__ import annotations

from .errors import SpecIncompleteError
from .spec import FieldRange, WrapMode

__all__ = ["TurnCounter", "WrapMode"]


class TurnCounter:
    """Turns a wrapping field reading into a continuous value.

    Not thread-safe by design: it belongs to one motor and is driven from ``update()`` on
    the control thread, never from the receive thread. Keeping the derivation off the
    receive path means it runs at a known rate rather than at whatever rate frames happen
    to arrive.
    """

    __slots__ = ("_field", "_last_raw", "_mode", "_saturation_seen", "_turns")

    def __init__(self, field: FieldRange, mode: WrapMode = WrapMode.UNKNOWN) -> None:
        self._field = field
        self._mode = mode
        self._turns = 0
        self._last_raw: float | None = None
        self._saturation_seen = False

    @property
    def turns(self) -> int:
        """Net field spans traversed since the last reset."""
        return self._turns

    @property
    def mode(self) -> WrapMode:
        """The wrap behaviour this counter was built for.

        :attr:`~cubemarspycan.spec.WrapMode.UNKNOWN` means unwrapping is disabled and
        multi-turn reads refuse: guessing whether a field wraps or saturates produces a
        position that is wrong by a whole field span.
        """
        return self._mode

    @property
    def saturation_seen(self) -> bool:
        """True once a reading has sat at a field limit, where position is unrecoverable."""
        return self._saturation_seen

    def reset(self, raw: float | None = None) -> None:
        """Forget the accumulated turns, optionally re-anchoring on ``raw``.

        Called by :meth:`~cubemarspycan.motor.mit.MitMotor.zero_here`, since the origin
        has moved and the old turn count no longer means anything. Without ``raw`` the
        next reading establishes the new anchor.
        """
        self._turns = 0
        self._last_raw = raw
        self._saturation_seen = False

    def update(self, raw: float) -> float:
        """Feed a raw field reading, get a continuous position back."""
        if self._mode is WrapMode.UNKNOWN:
            raise SpecIncompleteError(
                "multi-turn unwrapping needs to know whether this firmware wraps or "
                "saturates at the position field limit; the manual does not say. Settle "
                "it on the bench (drive past the limit and watch the reported value), "
                "then pass wrap_mode=WrapMode.WRAP or WrapMode.SATURATE. Until then the "
                "raw field reading is available unmodified."
            )

        if self._mode is WrapMode.SATURATE:
            # Nothing to recover: once the reading sticks at the rail, the true position
            # is simply not on the wire. Flag it so the caller can distinguish "at the
            # limit" from "moving normally near the limit".
            if raw >= self._field.hi - self._field.lsb or raw <= self._field.lo + self._field.lsb:
                self._saturation_seen = True
            self._last_raw = raw
            return raw

        if self._last_raw is not None:
            delta = raw - self._last_raw
            half = self._field.span / 2.0
            if delta > half:
                self._turns -= 1
            elif delta < -half:
                self._turns += 1
        self._last_raw = raw
        return raw + self._turns * self._field.span
