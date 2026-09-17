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

    def send(self, frame: Frame, timeout: float | None = ...) -> None:
        """Put one frame on the wire, blocking up to ``timeout`` seconds.

        Called on the **caller's** thread, from inside the control loop, so the blocking
        window is the loop's jitter budget: a socketcan send is microseconds, an slcan
        send over USB serial is 0.5-2 ms.

        Raises :class:`~cubemarspycan.errors.SendFailed` rather than returning a status or
        swallowing the error. A command that did not reach the motor is not a detail the
        caller can be left to infer.
        """
        ...

    def add_sink(self, sink: FrameSink) -> None:
        """Register a receiver. Sinks must be added before :meth:`start`.

        Every registered sink is called on the **receive** thread for every frame, so a
        sink must never raise and must not block - see :class:`FrameSink`. An
        implementation is expected to contain and count an exception rather than let it
        kill reception for the other sinks.
        """
        ...

    def start(self) -> None:
        """Begin receiving. Idempotent: calling it twice must not start a second reader.

        Until this is called, frames may be dropped by the underlying driver. Sinks
        registered afterwards are not guaranteed to see earlier frames.
        """
        ...

    def close(self) -> None:
        """Stop receiving and release the link. Idempotent, and safe after a failed
        :meth:`start`.

        Must not raise: it runs from ``__exit__`` and from error-recovery paths, where
        another exception is usually already unwinding.
        """
        ...
