"""Shared motor plumbing: latch wiring, the update path, and the control lifecycle.

The one rule that shapes this module: **nothing in the receive path raises**. A fault
arrives as a frame, becomes a :class:`~cubemarspycan.state.FaultEvent` in a latch, and
only turns into control flow inside :meth:`MotorEndpoint.update`, on the caller's thread,
*after* a safe-stop frame has gone out. TMotorCANControl raises ``RuntimeError`` straight
from the python-can notifier thread, where the exception is swallowed and the motor keeps
being commanded while faulted.
"""

from __future__ import annotations

import logging
import time
import warnings
from collections.abc import Iterator
from contextlib import contextmanager
from typing import Generic, TypeVar

from ..bus import MotorBus
from ..errors import (
    MotorFault,
    NotInControlMode,
    StaleFeedbackError,
)
from ..frame import Frame
from ..latch import FaultLatch, StateLatch
from ..policy import DEFAULT_POLICY, ClampMode, ClampReport, FaultAction, SafetyPolicy
from ..spec import FieldRange, MotorSpec, Sourced
from ..state import FaultEvent

log = logging.getLogger(__name__)

StateT = TypeVar("StateT")


class MotorEndpoint(Generic[StateT]):
    """One motor on one bus."""

    def __init__(
        self,
        bus: MotorBus,
        motor_id: int,
        spec: MotorSpec,
        *,
        policy: SafetyPolicy = DEFAULT_POLICY,
        supply_voltage: float | None = None,
    ) -> None:
        self.bus = bus
        self.motor_id = motor_id
        self.spec = spec
        self.policy = policy
        self.supply_voltage = (
            supply_voltage if supply_voltage is not None else policy.supply_voltage_v
        )

        self._states: StateLatch[StateT] = StateLatch()
        self._faults = FaultLatch()
        self._entered = False
        self._last_command_sent: float = 0.0
        self._clamp_events = 0
        self._stale_grace_until: float = 0.0
        self._stale_grace_opened: float = 0.0

        bus.register(self)
        self._warn_about_the_supply_voltage()

    # --- Endpoint protocol (subclasses implement) -----------------------------------

    def accepts(self, frame: Frame) -> bool:  # pragma: no cover - abstract
        raise NotImplementedError

    def on_frame(self, frame: Frame, rx_monotonic: float) -> None:  # pragma: no cover
        raise NotImplementedError

    # --- lifecycle ------------------------------------------------------------------

    def _enter_frames(self) -> list[Frame]:  # pragma: no cover - abstract
        raise NotImplementedError

    def _exit_frames(self) -> list[Frame]:  # pragma: no cover - abstract
        raise NotImplementedError

    def _safe_stop_frame(self) -> Frame:  # pragma: no cover - abstract
        raise NotImplementedError

    @property
    def in_control(self) -> bool:
        return self._entered

    @contextmanager
    def control(self, wait_s: float = 1.0) -> Iterator[MotorEndpoint[StateT]]:
        """Enter control mode, and guarantee leaving it.

        The exit path runs even if the body raises, and it sends a safe stop *before* the
        exit frame so the motor is never left producing torque.

        ``wait_s`` blocks until the first feedback frame arrives, which catches a wrong id
        or a dead bus at the top of the ``with`` rather than a hundred silent iterations
        later. Pass ``0.0`` to skip it, which is what the stepped simulator needs because
        nothing advances until the test says so.
        """
        for frame in self._enter_frames():
            self.bus.send(frame)
        self._entered = True
        try:
            if wait_s > 0.0:
                self._await_first_feedback(wait_s)
            yield self
        finally:
            self._entered = False
            try:
                self.bus.send(self._safe_stop_frame())
            finally:
                for frame in self._exit_frames():
                    self.bus.send(frame)

    def _await_first_feedback(self, wait_s: float) -> None:
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if self._states.seq > 0:
                return
            time.sleep(0.001)
        raise StaleFeedbackError(self._no_feedback_diagnosis(wait_s))

    def _no_feedback_diagnosis(self, wait_s: float) -> str:
        unmatched = self.bus.stats.rx_unmatched
        lines = [f"no feedback from {self.spec.name} id {self.motor_id} within {wait_s:g} s."]
        if unmatched == 0 and self.bus.stats.rx_matched == 0:
            lines.append(
                "Nothing at all was received: check power, wiring, termination, and that "
                "the bus bitrate is 1 Mbit/s."
            )
        else:
            lines.append(
                f"{unmatched} frame(s) arrived but matched no motor, so something is on "
                f"the bus but not answering to id {self.motor_id}. Recent frames: "
                + ", ".join(self.bus.stats.unmatched_samples)
                or ""
            )
        return " ".join(lines)

    # --- the update path ------------------------------------------------------------

    def _snapshot(self) -> tuple[StateT | None, float, int]:
        return self._states.read()

    def _require_control(self) -> None:
        if not self._entered:
            raise NotInControlMode(
                f"{self.spec.name} id {self.motor_id} is not in control mode; do this "
                f"inside a `with motor.control():` block"
            )

    def _check_fault(self) -> None:
        """Turn a latched fault into control flow, on this thread, after a safe stop."""
        event = self._faults.take_new()
        if event is None:
            return
        if self.policy.on_fault is FaultAction.IGNORE:
            return
        if self.policy.on_fault is FaultAction.WARN:
            warnings.warn(f"{self.spec.name} id {self.motor_id}: {event}", stacklevel=3)
            return
        self._emergency_stop()
        raise MotorFault(
            f"{self.spec.name} id {self.motor_id} reported {event.text} "
            f"(code {event.code}). A safe-stop frame has been sent. Clear the condition, "
            f"then call motor.clear_fault() before continuing."
        )

    def expect_silence(self, seconds: float) -> None:
        """Tolerate missing feedback for ``seconds``, starting now.

        Some operations stop the driver replying for a while - :meth:`MitMotor.zero_here`
        is the one that bites. Staleness is measured against the last frame *received*, so
        transmitting through the gap does not help: without this, the first ``update()``
        after such an operation raises :class:`~cubemarspycan.errors.StaleFeedbackError`
        even though nothing is wrong.

        This suppresses only the *fatal* limit. The warning still fires, so a gap that
        turns out to be permanent is still visible, and a frame received **after the
        window opened** ends it early - see :meth:`_in_stale_grace` for why "after"
        rather than "fresh".
        """
        now = time.monotonic()
        self._stale_grace_opened = now
        self._stale_grace_until = now + max(0.0, seconds)

    def _in_stale_grace(self, rx_monotonic: float, now: float) -> bool:
        """Whether a deliberate silence is still being tolerated.

        Two conditions, and the second is the one that is easy to get wrong. The obvious
        test - "stop tolerating once feedback looks fresh" - defeats the window entirely:
        at the instant :meth:`MitMotor.zero_here` opens it, the last received frame is
        normally ~10 ms old, so the very next ``update()`` would close the window before
        the driver has even gone quiet, which is the whole case it exists for.

        What ends it early is a frame received *after* it opened. That is proof the link
        came back, so any gap from there on is a real one and must still be fatal on
        schedule.
        """
        return now < self._stale_grace_until and rx_monotonic <= self._stale_grace_opened

    def _check_staleness(self, rx_monotonic: float, seq: int) -> None:
        if seq == 0:
            return
        # One `now` for both the age and the grace comparison: sampling the clock twice
        # measured them against subtly different instants.
        now = time.monotonic()
        age = now - rx_monotonic
        if age > self.policy.stale_fatal_s and not self._in_stale_grace(rx_monotonic, now):
            self._emergency_stop()
            raise StaleFeedbackError(
                f"{self.spec.name} id {self.motor_id}: no feedback for {age:.3f} s "
                f"(limit {self.policy.stale_fatal_s:g} s). A safe-stop frame has been "
                f"sent. The motor may have lost power or left the bus."
            )
        if age > self.policy.stale_warn_s:
            warnings.warn(
                f"{self.spec.name} id {self.motor_id}: feedback is {age * 1e3:.0f} ms "
                f"old. Lower the loop rate, or check the link.",
                stacklevel=3,
            )

    def _emergency_stop(self) -> None:
        """Best-effort safe stop. Never raises: it runs while another error is unwinding."""
        try:
            self.bus.send(self._safe_stop_frame())
        except Exception as exc:
            log.warning("safe stop for id %d could not be sent: %s", self.motor_id, exc)

    # --- faults ---------------------------------------------------------------------

    @property
    def fault(self) -> FaultEvent | None:
        """The latched fault, if any. Reading it does not consume it."""
        return self._faults.peek()

    @property
    def faulted(self) -> bool:
        return self._faults.faulted

    def clear_fault(self) -> None:
        """Forget the latched fault. Call only once the cause is addressed."""
        self._faults.clear()

    # --- limits ---------------------------------------------------------------------

    def _effective(self, field: FieldRange, physical: Sourced[float] | None) -> float:
        """The binding limit for one field.

        ``min(wire field, physical limit)``, never "physical" alone. Across the AK line
        the two differ in both directions: the AK40-10's torque field over-promises
        against a 4.1 N*m peak, while the AK80-9's under-promises against 22 N*m.
        """
        if self.policy.clamp is ClampMode.FIELD or physical is None or not physical.known:
            return field.hi
        return min(field.hi, physical.require("effective limit"))

    def _apply(
        self,
        name: str,
        value: float,
        field: FieldRange,
        physical: Sourced[float] | None = None,
    ) -> tuple[float, ClampReport | None]:
        """Clamp one command field, reporting what had to change."""
        limit = self._effective(field, physical)
        low = max(field.lo, -limit) if field.lo < 0 else field.lo
        applied = min(max(value, low), limit)
        if applied == value:
            return applied, None
        reason = "wire field limit" if limit >= field.hi else "motor limit from the datasheet"
        if self.policy.clamp is ClampMode.RAISE:
            raise ValueError(
                f"{name}={value:g} is outside the usable range [{low:g}, {limit:g}] "
                f"for {self.spec.name} ({reason}); policy clamp mode is RAISE"
            )
        self._clamp_events += 1
        return applied, ClampReport(name, value, applied, limit, reason)

    @property
    def clamp_events(self) -> int:
        """How many command fields have been clamped since construction."""
        return self._clamp_events

    # --- diagnostics ----------------------------------------------------------------

    def _warn_about_the_supply_voltage(self) -> None:
        if self.supply_voltage is None:
            return
        try:
            threshold = self.spec.velocity_field_saturation_voltage()
        except Exception:
            return
        if self.supply_voltage > threshold:
            warnings.warn(
                f"{self.spec.name} at {self.supply_voltage:g} V can spin faster than its "
                f"velocity field can report (saturates above {threshold:.1f} V). Velocity "
                f"feedback may clip or wrap. A command-side clamp cannot prevent this.",
                stacklevel=3,
            )

    def device_info(self) -> str:
        return f"{self.spec.name} id {self.motor_id}"
