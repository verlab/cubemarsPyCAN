"""Transport protocols.

Deliberately CAN-shaped. A serial link is a byte stream with its own framing and CRC, and
forcing both under one abstraction now would produce something that fits neither. The seam
that keeps servo-over-serial possible for v1.1 is one level up: the motor classes never
touch a :class:`~cubemarspycan.frame.Frame` transport directly, they talk to a link object.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from ..frame import Frame


class FrameSink(Protocol):
    """Called on the receive thread for every frame. Must never raise."""

    # Positional-only: a sink is a callback, so its parameter *names* are not part of
    # the contract and a plain function must satisfy it.
    def __call__(self, frame: Frame, rx_monotonic: float, bus_timestamp: float, /) -> None: ...


@runtime_checkable
class FrameTransport(Protocol):
    """A bidirectional CAN frame link."""

    def send(self, frame: Frame, timeout: float | None = ...) -> None: ...

    def add_sink(self, sink: FrameSink) -> None: ...

    def start(self) -> None: ...

    def close(self) -> None: ...
