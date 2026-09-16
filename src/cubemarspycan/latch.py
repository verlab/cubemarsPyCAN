"""Publication boxes between the receive thread and the control thread.

This is the only concurrency-critical module in the library, which is why it is a module
rather than a few lines inside the motor class. It is about sixty lines and carries a
thread-stress test.

The invariant that makes it correct is a *type* property enforced elsewhere: every state
class is ``frozen=True, slots=True``, so the writer must construct a new value per frame
and can never mutate one it has already published. Handing a reference back to the reader
is therefore semantically a copy, and tearing is impossible rather than merely avoided.

For contrast, TMotorCANControl's ``mit_can.py:774`` copies field-by-field out of an object
the receive thread is concurrently mutating, so a caller can observe position from frame N
beside velocity from frame N+1; and ``servo_serial.py:783`` rebinds the name instead of
copying, collapsing its double buffer entirely after the first update.
"""

from __future__ import annotations

import threading
from typing import Generic, TypeVar

from .state import FaultEvent

S = TypeVar("S")


class StateLatch(Generic[S]):
    """Single-writer, single-reader publication of an immutable value.

    The lock is held only for three attribute assignments: no allocation, no logging and
    no I/O happen inside it, so the receive thread never blocks the control thread for
    longer than a few hundred nanoseconds.
    """

    __slots__ = ("_lock", "_rx_t", "_seq", "_value")

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: S | None = None
        self._rx_t: float = 0.0
        self._seq: int = 0

    def publish(self, value: S, rx_monotonic: float) -> None:
        """Called from the receive thread. Must never raise."""
        with self._lock:
            self._value = value
            self._rx_t = rx_monotonic
            self._seq += 1

    def read(self) -> tuple[S | None, float, int]:
        """Called from the control thread. Returns ``(value, rx_monotonic, seq)``.

        The three move together, so a caller can always tell whether the value it is
        holding is the one whose timestamp it just checked.
        """
        with self._lock:
            return self._value, self._rx_t, self._seq

    @property
    def seq(self) -> int:
        """Number of values published so far. Monotonic."""
        with self._lock:
            return self._seq

    def reset(self) -> None:
        with self._lock:
            self._value = None
            self._rx_t = 0.0
            self._seq = 0


class FaultLatch:
    """Sticky fault storage. Set from the receive thread, consumed from the control thread.

    Nothing here raises. A fault stays *data* until ``update()`` decides, on the caller's
    thread, whether it becomes control flow. That is the fix for the most safety-relevant
    defect in the reference library, where a fault raised inside the python-can notifier
    thread never reached the control loop and the motor kept being commanded.
    """

    __slots__ = ("_count", "_event", "_lock", "_unseen")

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._event: FaultEvent | None = None
        self._count: int = 0
        self._unseen: bool = False

    def set(self, event: FaultEvent) -> None:
        """Latch a fault. The first one wins; later ones only bump the counter.

        Keeping the first is deliberate: a fault often cascades (an over-current trips,
        which stalls the motor, which trips over-temperature), and the first code is the
        one that tells you what actually happened.
        """
        with self._lock:
            self._count += 1
            if self._event is None:
                self._event = event
                self._unseen = True

    def peek(self) -> FaultEvent | None:
        """The latched fault, without consuming it."""
        with self._lock:
            return self._event

    def take_new(self) -> FaultEvent | None:
        """The latched fault if it has not been reported yet, else ``None``.

        Lets ``update()`` raise once per fault instead of on every call.
        """
        with self._lock:
            if self._event is not None and self._unseen:
                self._unseen = False
                return self._event
            return None

    def clear(self) -> None:
        """Forget the fault. Explicit, and only from the control thread."""
        with self._lock:
            self._event = None
            self._count = 0
            self._unseen = False

    @property
    def count(self) -> int:
        """How many faulted frames have arrived since the last clear."""
        with self._lock:
            return self._count

    @property
    def faulted(self) -> bool:
        with self._lock:
            return self._event is not None
