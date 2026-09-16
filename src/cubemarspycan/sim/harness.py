"""Wiring for tests and demos.

Two independent ``can.Bus(interface="virtual")`` instances on one channel: one owned by
the library's transport, one by the simulator. python-can delivers between instances on
the same channel, so the **real** receive path runs - notifier thread, sink dispatch,
routing, ``accepts``, codec, latch - rather than a monkeypatched stand-in. That is what
makes a green CI run on a laptop mean something.

:class:`SteppedSim` is driven by the test (``pump``/``step``), so nothing depends on wall
clock and there are no flaky sleeps.
"""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass, field

import can

from ..bus import MotorBus
from ..frame import Frame
from ..transport.can_bus import CanTransport
from .mit import SimMitDriver
from .servo import SimServoDriver

_channels = itertools.count()


@dataclass
class SteppedSim:
    """A fake bus segment the test advances by hand."""

    bus: can.BusABC
    mit_drivers: list[SimMitDriver] = field(default_factory=list)
    servo_drivers: list[SimServoDriver] = field(default_factory=list)
    dropped: int = 0
    received: list[Frame] = field(default_factory=list)
    """Every command frame the sim saw, in order. Lets tests assert on what was sent."""
    _drop_every: int = 0
    _seen: int = 0
    frozen: bool = False
    settle_s: float = 3.0e-4
    """Yield to the notifier thread after each step.

    Replies go out over a real python-can virtual bus and are delivered by a real
    notifier thread. A test loop that only sends and steps never releases the GIL, so
    that thread is starved and no feedback ever lands. A short sleep hands it the
    interpreter, which is also what a real control loop does while it waits for its
    period."""

    # --- knobs ----------------------------------------------------------------------

    def freeze(self) -> None:
        """Stop replying entirely, as a motor that has lost power would."""
        self.frozen = True

    def thaw(self) -> None:
        self.frozen = False

    def drop_every(self, n: int) -> None:
        """Drop one reply in ``n``. 0 disables."""
        self._drop_every = n

    # --- driving --------------------------------------------------------------------

    def pump(self) -> int:
        """Consume every pending command and emit replies. Returns frames handled."""
        handled = 0
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                return handled
            handled += 1
            frame = Frame(msg.arbitration_id, bytes(msg.data), msg.is_extended_id)
            self.received.append(frame)
            if self.frozen:
                continue
            replies: list[Frame] = []
            for driver in self.mit_drivers:
                reply = driver.handle(frame)
                if reply is not None:
                    replies.append(reply)
            for servo in self.servo_drivers:
                replies.extend(servo.handle(frame))
            for reply in replies:
                self._send(reply)

    def step(self, dt: float) -> None:
        """Advance every plant, and emit any periodic servo status frames."""
        for driver in self.mit_drivers:
            driver.step(dt)
        for servo in self.servo_drivers:
            for frame in servo.step(dt):
                if not self.frozen:
                    self._send(frame)

    def advance(self, dt: float) -> None:
        """One control period: handle what arrived, advance time, let the reader run."""
        self.pump()
        self.step(dt)
        self.settle()

    def settle(self) -> None:
        """Give the notifier thread a chance to deliver what was just sent."""
        if self.settle_s > 0.0:
            time.sleep(self.settle_s)

    def inject(self, frame: Frame) -> None:
        """Put an arbitrary frame on the bus, for adversarial tests."""
        self._send(frame)

    def _send(self, frame: Frame) -> None:
        self._seen += 1
        if self._drop_every and self._seen % self._drop_every == 0:
            self.dropped += 1
            return
        self.bus.send(
            can.Message(
                arbitration_id=frame.arbitration_id,
                data=frame.data,
                is_extended_id=frame.is_extended_id,
            )
        )

    def close(self) -> None:
        self.bus.shutdown()


def sim_bus(
    mit_drivers: list[SimMitDriver] | None = None,
    servo_drivers: list[SimServoDriver] | None = None,
    channel: str | None = None,
) -> tuple[MotorBus, SteppedSim]:
    """Build a started :class:`MotorBus` wired to a :class:`SteppedSim`.

    Caller closes both; :meth:`SteppedSim.close` and ``MotorBus.close`` are independent.
    """
    name = channel or f"cubemars-sim-{next(_channels)}"
    library_bus = can.interface.Bus(channel=name, interface="virtual", receive_own_messages=False)
    sim_side = can.interface.Bus(channel=name, interface="virtual", receive_own_messages=False)
    transport = CanTransport(library_bus, owns_bus=True)
    motor_bus = MotorBus(transport)
    motor_bus.start()
    return motor_bus, SteppedSim(
        bus=sim_side,
        mit_drivers=list(mit_drivers or []),
        servo_drivers=list(servo_drivers or []),
    )
