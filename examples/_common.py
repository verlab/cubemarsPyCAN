"""Shared plumbing for the examples.

Every example takes ``--sim`` and runs against a protocol-accurate simulator, so you can
read, run and modify them with no hardware attached. That is also how CI keeps them from
going stale.

The control logic in each example stays inline and obvious; only the bus wiring and the
loop timing live here.
"""

from __future__ import annotations

import argparse
import math
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass

from cubemarspycan import CanTransport, MotorBus, get_spec
from cubemarspycan.spec import MotorSpec


def base_parser(description: str) -> argparse.ArgumentParser:
    """An argument parser with the options every example shares."""
    parser = argparse.ArgumentParser(
        description=description, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--url", default="socketcan:can0", help="CAN url")
    parser.add_argument("--id", type=int, default=1, help="motor CAN id")
    parser.add_argument("--motor", default="AK40-10", help="spec name, e.g. AK80-9")
    parser.add_argument("--supply", type=float, default=24.0, help="supply voltage")
    parser.add_argument("--period", type=float, default=0.005, help="control period, s")
    parser.add_argument("--duration", type=float, default=8.0, help="run time, s")
    parser.add_argument("--sim", action="store_true", help="run against a simulator")
    return parser


@dataclass
class Rig:
    """A bus, plus the simulator behind it when running with ``--sim``."""

    bus: MotorBus
    sim: object | None
    spec: MotorSpec

    @property
    def simulated(self) -> bool:
        return self.sim is not None

    def ticker(self, period: float) -> Ticker:
        return Ticker(period, self.sim)


@contextmanager
def open_rig(
    args: argparse.Namespace,
    motor_ids: tuple[int, ...] = (),
    servo_ids: tuple[int, ...] = (),
) -> Iterator[Rig]:
    """Open a real bus, or a simulated one carrying the given MIT and servo drivers."""
    spec = get_spec(args.motor)
    if not args.sim:
        with CanTransport.open(args.url) as transport, MotorBus(transport) as bus:
            yield Rig(bus, None, spec)
        return

    from cubemarspycan.sim import SimMitDriver, SimServoDriver, sim_bus

    mit_ids = motor_ids or ((args.id,) if not servo_ids else ())
    bus, sim = sim_bus(
        mit_drivers=[SimMitDriver(spec, motor_id=i) for i in mit_ids],
        servo_drivers=[SimServoDriver(spec, motor_id=i, status_rate_hz=200.0) for i in servo_ids],
    )
    try:
        yield Rig(bus, sim, spec)
    finally:
        bus.close()
        sim.close()


class Ticker:
    """Paces a control loop, against either the wall clock or a stepped simulator.

    The simulator only advances when told to, so a loop that just slept would never see
    the motor move. Calling :meth:`tick` does the right thing either way.
    """

    def __init__(self, period: float, sim: object | None = None) -> None:
        self.period = period
        self._sim = sim
        self._t = 0.0
        self._start = time.monotonic()
        self._deadline = self._start
        self._prev_t = -period
        self._skipped = 0

    @property
    def t(self) -> float:
        """Seconds since the loop started."""
        return self._t

    @property
    def skipped(self) -> int:
        """Ticks dropped because the loop fell more than a whole period behind.

        Non-zero means the loop did not keep up. It is *reported* rather than repaid -
        see :meth:`tick`.
        """
        return self._skipped

    def tick(self) -> None:
        self._prev_t = self._t
        if self._sim is not None:
            self._sim.advance(self.period)  # type: ignore[attr-defined]
            self._t += self.period
            return

        # Sleep to the next deadline, not for a fixed period. Sleeping `period` after a
        # body that itself took `body` gives a real rate of 1/(period + body) - a 200 Hz
        # loop with a 2 ms CAN send runs at ~99 Hz.
        self._deadline += self.period
        now = time.monotonic()
        if now < self._deadline:
            time.sleep(self._deadline - now)
        elif now - self._deadline >= self.period:
            # More than a whole period lost - a GC pause, a page fault, a laptop lid.
            # Drop the missed ticks; do not repay them. Repaying means every late
            # iteration runs back to back with no sleep at all: measured, one 0.5 s stall
            # left 34 of 34 following iterations sleeping ~0 ms, which on hardware is a
            # hundred command frames burst onto a bus paced for 200 Hz. A control loop
            # that falls behind should resume at its nominal rate, not sprint to catch up.
            self._skipped += int((now - self._deadline) // self.period)
            self._deadline = now
        # Behind by less than a period: no sleep, but the deadline stays on the original
        # grid so ordinary jitter is absorbed rather than costing a whole dropped tick.
        self._t = time.monotonic() - self._start

    def running(self, duration: float) -> bool:
        return self._t < duration

    def every(self, seconds: float) -> bool:
        """True once per ``seconds`` of loop time. For throttling prints.

        Uses the previous tick's ``t`` rather than ``t - period``: on hardware a tick can
        overrun its period, and assuming it did not makes a throttle skip or double-fire.
        """
        if seconds <= 0.0:
            return True
        return math.floor(self._t / seconds) != math.floor(self._prev_t / seconds)


def wait_for_control(rig: Rig) -> float:
    """How long ``control()`` should wait for first feedback.

    Zero under the stepped simulator, which cannot produce a frame until the test loop
    advances it; a real motor gets a real timeout.
    """
    return 0.0 if rig.simulated else 1.5


_HARDWARE_SETTLE_S = 1.5
"""Wall-clock hold after ``zero_here()`` on hardware.

Matched to ``MitMotor.zero_here``'s default ``grace_s=1.5`` on purpose, so the hold ends
*with* the grace window rather than after it.
"""

_SIM_SETTLE_TICKS = 2
_SIM_SETTLE_PERIOD = 0.005


def settle(rig: Rig, first: object, /, *rest: object) -> None:
    """Hold after ``zero_here()``, doing the right thing for hardware and for the sim.

    Pass **every** motor you have zeroed. A MIT driver only answers when it is commanded,
    so a motor left out here goes unspoken-to for the whole wait and its next ``update()``
    trips staleness. At least one is required, positionally: ``settle(rig)`` with none
    used to degenerate into a wall-clock spin that transmitted nothing, which is the exact
    failure this function exists to prevent.

    On hardware the driver may stop replying for about a second while it zeroes.
    ``zero_here()`` opens a grace window for exactly that - staleness is measured on
    *received* frames, so transmitting through the gap does not help by itself.

    Under the stepped simulator the opposite problem applies: ``motor.settle()`` sleeps,
    and sleeping never advances a simulator that only moves when told to. The sim zeroes
    instantly, so two ticks is enough to get a fresh frame - and deliberately no more,
    because each tick integrates the MIT torque field's half-LSB zero offset and walks the
    shaft back off the zero we just set.

    There is deliberately no ``seconds`` parameter. It was silently ignored under
    ``--sim``, and it could not have been honoured there: ``seconds / period`` ticks would
    walk the zero off by millimetres of shaft. The two branches are different operations
    that happen to share a name.
    """
    motors = (first, *rest)
    if rig.simulated:
        ticker = rig.ticker(_SIM_SETTLE_PERIOD)
        for _ in range(_SIM_SETTLE_TICKS):
            # Tick first: zero_here() has already put a zeroed command on the wire, and
            # updating before the sim has answered it warns about absent feedback.
            ticker.tick()
            for motor in motors:
                motor.update()  # type: ignore[attr-defined]
        return
    deadline = time.monotonic() + _HARDWARE_SETTLE_S
    while time.monotonic() < deadline:
        for motor in motors:
            motor.update()  # type: ignore[attr-defined]
        time.sleep(0.01)


__all__ = [
    "base_parser",
    "open_rig",
    "settle",
    "wait_for_control",
]
