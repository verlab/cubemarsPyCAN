"""Frame routing.

One :class:`MotorBus` owns one transport and fans received frames out to registered
endpoints. Two properties matter:

* **Containment.** An endpoint that raises is caught and counted; the other motors on the
  bus keep receiving. One motor's bug must not silence the rest.
* **Lock-free reads.** The endpoint list is an immutable tuple, replaced wholesale on
  register. The receive thread does a single attribute read to snapshot it, so registering
  a motor never blocks reception.

Unmatched frames are counted and sampled, which is what makes ``cubemars scan`` able to
tell "nothing on the bus" from "something is there but not answering to that id".
"""

from __future__ import annotations

import logging
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from .frame import Frame
from .transport.base import FrameTransport

log = logging.getLogger(__name__)


@runtime_checkable
class Endpoint(Protocol):
    """Something that consumes frames addressed to it."""

    def accepts(self, frame: Frame) -> bool:
        """Cheap predicate. Must not raise."""
        ...

    def on_frame(self, frame: Frame, rx_monotonic: float) -> None:
        """Handle an accepted frame. Runs on the receive thread; must not raise."""
        ...


@dataclass
class BusStats:
    """Routing counters. Diagnostics, not shared state."""

    rx_matched: int = 0
    rx_unmatched: int = 0
    endpoint_errors: int = 0
    errors: deque[str] = field(default_factory=lambda: deque(maxlen=16))
    unmatched_samples: deque[str] = field(default_factory=lambda: deque(maxlen=16))

    def record_endpoint_error(self, endpoint: object, exc: BaseException) -> None:
        """Count and sample an exception raised by an endpoint.

        Called on the receive thread, from inside the ``except`` that keeps one motor's bug
        from silencing the others, so it must not raise. The sample list is bounded.
        """
        self.endpoint_errors += 1
        self.errors.append(f"{type(endpoint).__name__}: {type(exc).__name__}: {exc}")


class MotorBus:
    """Routes frames between a transport and a set of motors."""

    def __init__(self, transport: FrameTransport) -> None:
        self._transport = transport
        self._endpoints: tuple[Endpoint, ...] = ()
        self._register_lock = threading.Lock()
        self._started = False
        self.stats = BusStats()
        transport.add_sink(self._on_frame)

    # --- lifecycle ------------------------------------------------------------------

    def start(self) -> None:
        """Begin receiving. Idempotent - a second call is a no-op.

        Registering a motor afterwards is safe and needs no restart, since routing reads
        an immutable tuple that :meth:`register` replaces wholesale.
        """
        if not self._started:
            self._transport.start()
            self._started = True

    def close(self) -> None:
        """Close the transport and stop receiving.

        Delegates to the transport, whose ``close`` is required to be idempotent and
        non-raising, so this is safe from an error path. Endpoints stay registered: closing
        the bus ends reception, it does not dismantle the object.
        """
        self._transport.close()
        self._started = False

    def __enter__(self) -> MotorBus:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def transport(self) -> FrameTransport:
        """The injected transport. Exposed for diagnostics, not for sending.

        Use :meth:`send` instead, so frames are counted.
        """
        return self._transport

    @property
    def endpoints(self) -> tuple[Endpoint, ...]:
        """The registered endpoints, as an immutable snapshot.

        Safe to read from any thread: :meth:`register` and :meth:`unregister` publish a
        new tuple rather than mutating this one, which is what lets the receive thread take
        its snapshot with a single attribute read and no lock.
        """
        return self._endpoints

    # --- registration ---------------------------------------------------------------

    def register(self, endpoint: Endpoint) -> None:
        """Add an endpoint. Publishes a new tuple so the receive thread never locks."""
        with self._register_lock:
            if endpoint in self._endpoints:
                return
            self._endpoints = (*self._endpoints, endpoint)

    def unregister(self, endpoint: Endpoint) -> None:
        """Remove an endpoint. Unknown endpoints are ignored.

        Compares by identity, not equality. A frame already being dispatched may still
        reach the endpoint: the receive thread snapshots the tuple before iterating, so
        removal takes effect from the next frame, not the current one.
        """
        with self._register_lock:
            self._endpoints = tuple(e for e in self._endpoints if e is not endpoint)

    # --- transmit -------------------------------------------------------------------

    def send(self, frame: Frame, timeout: float | None = 0.05) -> None:
        """Put one frame on the bus, blocking up to ``timeout`` seconds.

        Called on the caller's thread. Propagates
        :class:`~cubemarspycan.errors.SendFailed` unchanged - a command that did not reach
        the motor must not look like one that did.
        """
        self._transport.send(frame, timeout)

    # --- receive: runs on the notifier thread and MUST NOT RAISE --------------------

    def _on_frame(self, frame: Frame, rx_monotonic: float, bus_timestamp: float) -> None:
        endpoints = self._endpoints  # one attribute read: an atomic snapshot of the tuple
        matched = False
        for endpoint in endpoints:
            try:
                if not endpoint.accepts(frame):
                    continue
                matched = True
                endpoint.on_frame(frame, rx_monotonic)
            except BaseException as exc:
                self.stats.record_endpoint_error(endpoint, exc)
        if matched:
            self.stats.rx_matched += 1
        else:
            self.stats.rx_unmatched += 1
            self.stats.unmatched_samples.append(str(frame))

    # --- diagnostics ----------------------------------------------------------------

    def describe(self) -> str:
        """A multi-line diagnostic summary: channel, endpoint count, and receive stats.

        Includes the last few frames that matched no endpoint, which is what distinguishes
        "nothing is on the bus" from "something is there but not answering to that id".
        """
        lines = [f"MotorBus on {getattr(self._transport, 'channel_info', self._transport)}"]
        lines.append(f"  endpoints      : {len(self._endpoints)}")
        lines.append(f"  rx matched     : {self.stats.rx_matched}")
        lines.append(f"  rx unmatched   : {self.stats.rx_unmatched}")
        if self.stats.endpoint_errors:
            lines.append(f"  endpoint errors: {self.stats.endpoint_errors}")
            lines += [f"    {e}" for e in self.stats.errors]
        if self.stats.unmatched_samples:
            lines.append("  recent unmatched frames:")
            lines += [f"    {s}" for s in self.stats.unmatched_samples]
        return "\n".join(lines)
