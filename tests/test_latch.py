"""The publication boxes, including a thread-stress test for tearing."""

from __future__ import annotations

import threading
from dataclasses import dataclass

from cubemarspycan.faults import CanFault
from cubemarspycan.latch import FaultLatch, StateLatch
from cubemarspycan.state import FaultEvent


@dataclass(frozen=True, slots=True)
class Sample:
    """A value whose fields must agree. ``total`` is redundant on purpose."""

    a: int
    b: int
    total: int

    @classmethod
    def of(cls, n: int) -> Sample:
        return cls(n, n * 2, n * 3)

    @property
    def consistent(self) -> bool:
        return self.total == self.a + self.b


# --- StateLatch ---------------------------------------------------------------------


def test_starts_empty() -> None:
    latch: StateLatch[Sample] = StateLatch()
    value, rx_t, seq = latch.read()
    assert value is None and rx_t == 0.0 and seq == 0


def test_publish_then_read() -> None:
    latch: StateLatch[Sample] = StateLatch()
    latch.publish(Sample.of(1), 123.5)
    value, rx_t, seq = latch.read()
    assert value == Sample.of(1)
    assert rx_t == 123.5
    assert seq == 1


def test_seq_is_monotonic() -> None:
    latch: StateLatch[Sample] = StateLatch()
    for i in range(1, 51):
        latch.publish(Sample.of(i), float(i))
        assert latch.seq == i
    assert latch.read()[2] == 50


def test_reading_twice_returns_the_same_value() -> None:
    latch: StateLatch[Sample] = StateLatch()
    latch.publish(Sample.of(7), 1.0)
    assert latch.read() == latch.read()


def test_reset() -> None:
    latch: StateLatch[Sample] = StateLatch()
    latch.publish(Sample.of(1), 1.0)
    latch.reset()
    assert latch.read() == (None, 0.0, 0)


def test_no_tearing_under_concurrent_publication() -> None:
    """The invariant the whole design leans on.

    Because published values are frozen, a reader can never observe half of one update
    beside half of another. TMotorCANControl copies field-by-field out of an object the
    receive thread is mutating, which is exactly this failure.
    """
    latch: StateLatch[Sample] = StateLatch()
    latch.publish(Sample.of(0), 0.0)
    stop = threading.Event()
    torn: list[Sample] = []
    out_of_order: list[tuple[int, int]] = []

    def writer(base: int) -> None:
        for i in range(20_000):
            latch.publish(Sample.of(base * 100_000 + i), float(i))

    def reader() -> None:
        last_seq = 0
        while not stop.is_set():
            value, _, seq = latch.read()
            if value is not None and not value.consistent:
                torn.append(value)
            if seq < last_seq:
                out_of_order.append((last_seq, seq))
            last_seq = seq

    readers = [threading.Thread(target=reader) for _ in range(2)]
    writers = [threading.Thread(target=writer, args=(n,)) for n in range(4)]
    for t in readers + writers:
        t.start()
    for t in writers:
        t.join()
    stop.set()
    for t in readers:
        t.join()

    assert not torn, f"observed {len(torn)} torn reads"
    assert not out_of_order, f"sequence went backwards: {out_of_order[:3]}"
    assert latch.seq == 4 * 20_000 + 1


# --- FaultLatch ---------------------------------------------------------------------


def event(code: int, seq: int = 1) -> FaultEvent:
    return FaultEvent.from_code(code, "mit", 1.0, seq)


def test_fault_latch_starts_clear() -> None:
    latch = FaultLatch()
    assert not latch.faulted
    assert latch.peek() is None
    assert latch.take_new() is None
    assert latch.count == 0


def test_first_fault_wins_later_ones_only_count() -> None:
    """A fault cascades - over-current stalls the motor, which trips over-temperature.
    The first code is the one that says what actually happened."""
    latch = FaultLatch()
    latch.set(event(2))  # over-current
    latch.set(event(7))  # motor stall, a consequence
    latch.set(event(1))  # over-temperature, a further consequence
    latched = latch.peek()
    assert latched is not None and latched.fault is CanFault.OVER_CURRENT
    assert latch.count == 3


def test_take_new_fires_once_per_fault() -> None:
    """So update() raises once, not on every subsequent call."""
    latch = FaultLatch()
    latch.set(event(3))
    first = latch.take_new()
    assert first is not None and first.fault is CanFault.OVER_VOLTAGE
    assert latch.take_new() is None
    assert latch.peek() is not None, "peek stays available after take_new"


def test_clear_resets_everything() -> None:
    latch = FaultLatch()
    latch.set(event(4))
    latch.set(event(4))
    latch.clear()
    assert not latch.faulted
    assert latch.count == 0
    assert latch.take_new() is None


def test_a_new_fault_after_clear_is_reported_again() -> None:
    latch = FaultLatch()
    latch.set(event(5))
    latch.take_new()
    latch.clear()
    latch.set(event(5))
    assert latch.take_new() is not None


def test_unknown_fault_codes_latch_without_raising() -> None:
    latch = FaultLatch()
    latch.set(event(200))
    latched = latch.peek()
    assert latched is not None
    assert latched.fault is None
    assert "200" in latched.text


def test_fault_latch_is_thread_safe() -> None:
    latch = FaultLatch()

    def setter() -> None:
        for _ in range(5_000):
            latch.set(event(2))

    threads = [threading.Thread(target=setter) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert latch.count == 20_000
    latched = latch.peek()
    assert latched is not None and latched.fault is CanFault.OVER_CURRENT
