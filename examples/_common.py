"""Shared plumbing for the examples.

Every example takes ``--sim`` and runs against a protocol-accurate simulator, so you can
read, run and modify them with no hardware attached. That is also how CI keeps them from
going stale.

The control logic in each example stays inline and obvious; only the bus wiring and the
loop timing live here.
"""

from __future__ import annotations

import argparse
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass

from cubemarspycan import SPECS, CanTransport, MotorBus, get_spec
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
def open_rig(args: argparse.Namespace, motor_ids: tuple[int, ...] = ()) -> Iterator[Rig]:
    """Open a real bus, or a simulated one carrying ``motor_ids`` MIT drivers."""
    spec = get_spec(args.motor)
    if not args.sim:
        with CanTransport.open(args.url) as transport, MotorBus(transport) as bus:
            yield Rig(bus, None, spec)
        return

    from cubemarspycan.sim import SimMitDriver, sim_bus

    ids = motor_ids or (args.id,)
    drivers = [SimMitDriver(spec, motor_id=i) for i in ids]
    bus, sim = sim_bus(mit_drivers=drivers)
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

    @property
    def t(self) -> float:
        """Seconds since the loop started."""
        return self._t

    def tick(self) -> None:
        if self._sim is not None:
            self._sim.advance(self.period)  # type: ignore[attr-defined]
            self._t += self.period
        else:
            time.sleep(self.period)
            self._t = time.monotonic() - self._start

    def running(self, duration: float) -> bool:
        return self._t < duration


def wait_for_control(rig: Rig) -> float:
    """How long ``control()`` should wait for first feedback.

    Zero under the stepped simulator, which cannot produce a frame until the test loop
    advances it; a real motor gets a real timeout.
    """
    return 0.0 if rig.simulated else 1.5


def settle_time(rig: Rig) -> float:
    """Seconds to hold after ``zero_here()``. The simulator zeroes instantly."""
    return 0.0 if rig.simulated else 1.5


__all__ = [
    "SPECS",
    "Rig",
    "Ticker",
    "base_parser",
    "open_rig",
    "settle_time",
    "wait_for_control",
]
