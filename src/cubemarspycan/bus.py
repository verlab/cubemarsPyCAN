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
        if not self._started:
            self._transport.start()
            self._started = True

    def close(self) -> None:
        self._transport.close()
        self._started = False

    def __enter__(self) -> MotorBus:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def transport(self) -> FrameTransport:
        return self._transport

    @property
    def endpoints(self) -> tuple[Endpoint, ...]:
        return self._endpoints

    # --- registration ---------------------------------------------------------------

    def register(self, endpoint: Endpoint) -> None:
        """Add an endpoint. Publishes a new tuple so the receive thread never locks."""
        with self._register_lock:
            if endpoint in self._endpoints:
                return
            self._endpoints = (*self._endpoints, endpoint)

    def unregister(self, endpoint: Endpoint) -> None:
        with self._register_lock:
            self._endpoints = tuple(e for e in self._endpoints if e is not endpoint)

    # --- transmit -------------------------------------------------------------------

    def send(self, frame: Frame, timeout: float | None = 0.05) -> None:
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
