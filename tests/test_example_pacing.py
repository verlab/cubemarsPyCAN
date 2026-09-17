"""The examples' shared loop pacing, which nothing else covers.

Every other test runs the examples as subprocesses with ``--sim``, which takes the
simulator branch of ``Ticker.tick()``. The wall-clock branch - the one that runs on real
hardware - had no coverage at all, which is how it shipped with a catch-up burst.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
from types import ModuleType

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
_CACHED: list[ModuleType] = []


def load_common() -> ModuleType:
    """Import ``examples/_common.py`` without putting ``examples/`` on ``sys.path``.

    It is deliberately not part of the installed package: a loop pacer is not a CAN
    concern, and ``tests/test_public_api.py`` forces every re-exported name into
    ``__all__``, so promoting it would commit a demo utility to the semver surface. Not
    public is not the same as not tested.
    """
    if _CACHED:
        return _CACHED[0]
    path = ROOT / "examples" / "_common.py"
    spec = importlib.util.spec_from_file_location("examples_common", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Register before exec: `_common` uses `from __future__ import annotations`, so
    # @dataclass resolves its string annotations through sys.modules[cls.__module__].
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    _CACHED.append(module)
    return module


class FakeTime:
    """A clock that only moves when something sleeps.

    Substituted for the ``time`` module *inside* ``_common``, never for the real one: the
    sim harness and the python-can notifier thread both call ``time.monotonic``, and
    freezing that globally would hang unrelated tests in the same process.
    """

    def __init__(self) -> None:
        self.now = 1000.0
        self.slept: list[float] = []

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.slept.append(seconds)
        self.now += seconds


class FakeSim:
    def __init__(self) -> None:
        self.advances: list[float] = []

    def advance(self, dt: float) -> None:
        self.advances.append(dt)


class FakeMotor:
    def __init__(self) -> None:
        self.updates = 0

    def update(self) -> None:
        self.updates += 1


# --- pacing ---------------------------------------------------------------------------


def test_a_stall_drops_the_missed_ticks_instead_of_bursting_can_frames(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Measured before the fix: after one 0.5 s stall, 34 of 34 following iterations
    slept ~0 ms.

    Sleeping to ``start + n * period`` repays the whole debt, so the loop free-runs at the
    body's own rate until it catches up - a hundred command frames back to back on a bus
    paced for 200 Hz.
    """
    common = load_common()
    fake = FakeTime()
    monkeypatch.setattr(common, "time", fake)

    ticker = common.Ticker(0.005)  # no sim: the wall-clock branch
    for _ in range(3):
        ticker.tick()
    assert fake.slept == [pytest.approx(0.005)] * 3

    fake.now += 0.5  # the loop body stalls for half a second
    before = len(fake.slept)
    ticker.tick()
    assert len(fake.slept) == before, "an already-late tick must not sleep"
    assert ticker.skipped == 99, "the missed ticks are dropped, and counted"

    for _ in range(5):
        ticker.tick()
    assert fake.slept[-5:] == [pytest.approx(0.005)] * 5, (
        "after a stall the loop resumes at its period; it does not free-run to catch up"
    )
    assert ticker.skipped == 99, "the rebase happens once, not on every later tick"


def test_ordinary_jitter_is_absorbed_rather_than_dropped(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The clamp must not fire on a body that merely overruns.

    Rebasing whenever ``now`` is past the deadline would push every deadline a full extra
    period on a loop that is 10% slow, halving its real rate. Only the loss of more than
    one whole period rebases - the threshold is ``>= period``, not ``> 0``.
    """
    common = load_common()
    fake = FakeTime()
    monkeypatch.setattr(common, "time", fake)

    ticker = common.Ticker(0.005)
    ticker.tick()

    fake.now += 0.007  # overran the period by 2 ms
    before = len(fake.slept)
    ticker.tick()
    assert len(fake.slept) == before, "already late: no sleep"

    ticker.tick()
    assert fake.slept[-1] == pytest.approx(0.003), "back on the original grid"
    assert ticker.skipped == 0


def test_the_simulator_branch_steps_rather_than_sleeps() -> None:
    """The sim only moves when told to; a tick that slept would never see the motor."""
    common = load_common()
    sim = FakeSim()
    ticker = common.Ticker(0.005, sim)

    ticker.tick()
    ticker.tick()

    assert sim.advances == [0.005, 0.005]
    assert ticker.t == pytest.approx(0.010)


def test_every_fires_once_per_window_including_at_zero() -> None:
    """`int()` truncates toward zero, so `int(0.0) != int(-period)` was False and the
    t=0 edge never fired - fault_handling.py printed nothing at all under CI's own args.
    """
    common = load_common()
    ticker = common.Ticker(0.005, FakeSim())

    assert ticker.every(0.5), "must fire on the first pass, at t=0"
    fired = 0
    for _ in range(200):  # 1.0 s of simulated time
        ticker.tick()
        if ticker.every(0.5):
            fired += 1
    assert fired == 2, f"expected t=0.5 and t=1.0, got {fired}"


# --- settle ---------------------------------------------------------------------------


def test_settle_refuses_to_run_with_no_motors() -> None:
    """`settle(rig)` was a 1.5 s wall-clock spin that transmitted nothing - the exact
    failure settle() exists to prevent, wearing settle()'s name. Make it unwritable."""
    common = load_common()
    with pytest.raises(TypeError):
        common.settle(object())


def test_settle_under_the_simulator_ticks_exactly_twice_for_every_motor() -> None:
    """The 2 is load-bearing in both directions: fewer risks StaleFeedbackError on the
    next update(), and more integrates the MIT torque field's half-LSB zero offset,
    walking the shaft ~6 mrad off the zero just set against a 20 mrad test budget."""
    common = load_common()
    sim = FakeSim()
    rig = common.Rig(bus=None, sim=sim, spec=None)
    leader, follower = FakeMotor(), FakeMotor()

    common.settle(rig, leader, follower)

    assert len(sim.advances) == 2
    assert (leader.updates, follower.updates) == (2, 2)


def test_settle_takes_no_seconds_parameter() -> None:
    """It was ignored under --sim and could not have been honoured there, so a caller
    passing it would have got silence rather than an error."""
    import inspect

    assert "seconds" not in inspect.signature(load_common().settle).parameters
