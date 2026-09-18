"""The examples run, and the documentation does not point at files that do not exist."""

from __future__ import annotations

import pathlib
import re
import subprocess
import sys
from collections.abc import Iterable

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent


def _real(paths: Iterable[pathlib.Path]) -> list[pathlib.Path]:
    """Skip dotfiles. macOS tar emits AppleDouble `._name` siblings, and a glob that
    picks them up turns a file transfer into a test failure."""
    return sorted(p for p in paths if not p.name.startswith("."))


# `_`-prefixed modules are shared helpers, not examples.
EXAMPLES = _real(p for p in (ROOT / "examples").glob("*.py") if not p.name.startswith("_"))
DOCS = _real((ROOT / "docs").glob("*.md"))


# Every example must be registered here with arguments that make it finish quickly.
# A new example that nobody registers fails the suite rather than silently going untested.
EXAMPLE_ARGS: dict[str, list[str]] = {
    "mit_position_step.py": ["--dwell", "0.3"],
    "trajectory_tracking.py": ["--duration", "1.5"],
    "impedance_control.py": ["--stage-seconds", "0.3"],
    "torque_control.py": ["--duration", "1"],
    "velocity_control.py": ["--hold", "0.6"],
    "two_motors.py": ["--duration", "1"],
    "fault_handling.py": ["--duration", "1"],
    "homing.py": ["--duration", "3"],
    "log_to_csv.py": ["--duration", "1"],
    "servo_position.py": ["--duration", "1"],
    "servo_modes.py": ["--hold", "0.4"],
}


def test_every_example_is_registered_for_testing() -> None:
    names = {p.name for p in EXAMPLES}
    assert names == set(EXAMPLE_ARGS), (
        f"unregistered: {names - set(EXAMPLE_ARGS)}, stale entries: {set(EXAMPLE_ARGS) - names}"
    )


def test_docs_are_all_present() -> None:
    assert {p.name for p in DOCS} == {
        "adding-a-motor.md",
        "ak-2-0.md",
        "bench.md",
        "can-setup.md",
        "cli.md",
        "migration.md",
        "troubleshooting.md",
        "units.md",
    }


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.name)
def test_every_example_runs_against_the_simulator(
    path: pathlib.Path, tmp_path: pathlib.Path
) -> None:
    """The --sim path exists so an example is never stale: CI runs it every commit."""
    args = list(EXAMPLE_ARGS[path.name])
    if path.name == "log_to_csv.py":
        args += ["--out", str(tmp_path / "run.csv")]
    result = subprocess.run(
        [sys.executable, str(path), "--sim", *args],
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
        cwd=str(ROOT),
    )
    assert result.returncode == 0, f"{path.name} failed:\n{result.stderr[-2000:]}"
    # returncode alone is not enough: fault_handling.py and servo_modes.py catch
    # MotorError and exit 0, so a run that latched a spurious fault would still look
    # green. None of these are invoked in a way that should fault.
    assert "FAULT:" not in result.stdout, f"{path.name} latched a fault:\n{result.stdout[-2000:]}"
    assert "STALE:" not in result.stdout, f"{path.name} went stale:\n{result.stdout[-2000:]}"


def test_the_position_step_example_converges() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "examples" / "mit_position_step.py"),
            "--sim",
            "--dwell",
            "1.0",
            "--amplitude",
            "0.25",
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=True,
        cwd=str(ROOT),
    )
    # Parse the measured error, not the echoed setpoint. Asserting "+0.250" appears in
    # stdout passes even with kp=kd=0 (no tracking at all), because the example prints
    # `target {target:+.3f}` straight from --amplitude.
    errors = [float(m) for m in re.findall(r"error\s+([-+][\d.]+) mrad", result.stdout)]
    assert errors, f"no error lines parsed:\n{result.stdout[-800:]}"
    worst = max(abs(e) for e in errors)
    assert worst < 20.0, f"worst settle error {worst:.1f} mrad:\n{result.stdout[-800:]}"


def test_the_servo_example_reaches_its_target() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "examples" / "servo_position.py"),
            "--sim",
            "--duration",
            "3",
            "--degrees",
            "90",
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=True,
        cwd=str(ROOT),
    )
    assert "+90.00 deg" in result.stdout, result.stdout[-800:]


def test_the_feedforward_example_shows_an_improvement() -> None:
    """The measured hardware finding, reproduced in simulation on every commit."""
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "examples" / "trajectory_tracking.py"),
            "--sim",
            "--duration",
            "4",
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=True,
        cwd=str(ROOT),
    )
    assert "feedforward reduced median following error" in result.stdout
    factor = float(result.stdout.split("following error ")[1].split("x")[0])
    assert factor > 2.0, f"expected a clear improvement, got {factor}x"


def test_the_homing_example_finds_the_simulated_stop() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "examples" / "homing.py"),
            "--sim",
            "--duration",
            "4",
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=True,
        cwd=str(ROOT),
    )
    assert "hard stop at" in result.stdout
    assert "zeroed; now reads" in result.stdout


LINK = re.compile(r"\[[^\]]+\]\((?!https?://)([^)#]+)")


@pytest.mark.parametrize("path", [*DOCS, ROOT / "README.md"], ids=lambda p: p.name)
def test_relative_links_resolve(path: pathlib.Path) -> None:
    for target in LINK.findall(path.read_text()):
        resolved = (path.parent / target).resolve()
        assert resolved.exists(), f"{path.name} links to missing {target}"


def test_readme_documents_every_cli_command() -> None:
    """A new subcommand that nobody documented should fail the build."""
    import argparse

    from cubemarspycan.cli import build_parser

    readme = (ROOT / "README.md").read_text()
    commands: set[str] = set()
    for action in build_parser()._actions:
        if isinstance(action, argparse._SubParsersAction):
            commands |= set(action.choices)
    assert commands, "no subcommands found"
    for command in sorted(commands):
        assert f"cubemars {command}" in readme, f"{command} is undocumented"


def test_readme_is_explicit_about_what_is_hardware_verified() -> None:
    """The status claim must distinguish tested-on-a-motor from tested-in-simulation.

    MIT mode was verified on an AK40-10 on 2026-09-16; servo-over-CAN was not, because it
    needs the driver switched to servo mode in CubeMarsTool. Claiming both would be a
    lie, and claiming neither would be false modesty that hides a real result.
    """
    readme = (ROOT / "README.md").read_text().lower()
    assert "hardware-verified" in readme
    assert "servo" in readme
    assert "not yet been run" in readme or "simulator-only" in readme


# --- licensing ----------------------------------------------------------------------


def test_the_licence_file_exists_and_is_mit() -> None:
    text = (ROOT / "LICENSE").read_text()
    assert text.startswith("MIT License")
    assert "Permission is hereby granted, free of charge" in text
    assert "WITHOUT WARRANTY OF ANY KIND" in text
    assert "Copyright (c)" in text


def test_the_package_declares_the_same_licence() -> None:
    """A LICENSE file nobody declares does not reach anyone who pip-installs it."""
    pyproject = (ROOT / "pyproject.toml").read_text()
    assert 'license = "MIT"' in pyproject
    assert 'license-files = ["LICENSE"]' in pyproject


def test_nothing_still_calls_the_licence_undecided() -> None:
    """It was undecided for most of this project's life; stale text would mislead."""
    stale = []
    for path in [
        ROOT / "README.md",
        ROOT / ".github" / "workflows" / "ci.yml",
        ROOT / "tools" / "check_cleanroom.py",
        *DOCS,
    ]:
        body = path.read_text().lower()
        if "licence is undecided" in body or "licence for cubemarspycan is undecided" in body:
            stale.append(path.name)
    assert not stale, f"stale 'undecided' licence text in {stale}"


def test_the_readme_explains_why_mit_is_defensible() -> None:
    """MIT next to a GPLv3 reference only holds up because of the clean-room work.

    Saying so is the difference between a licence choice and a licence claim.
    """
    readme = (ROOT / "README.md").read_text()
    assert "[MIT](LICENSE)" in readme


def test_the_gpl_reference_library_is_not_publishable() -> None:
    """The audited library is GPLv3 and not ours to redistribute."""
    gitignore = (ROOT / ".gitignore").read_text()
    assert "TMotorCANControl-master/" in gitignore
    assert "*.pdf" in gitignore, "CubeMars manuals are their copyright"


def _run_example(name: str, args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(ROOT / "examples" / name), "--sim", *args],
        capture_output=True,
        text=True,
        timeout=180,
        check=True,
        cwd=str(ROOT),
    )


def test_the_csv_example_writes_rows_that_read_back(tmp_path: pathlib.Path) -> None:
    """Nothing read the file back, so a run that wrote a header and nothing else looked
    exactly like a good one: returncode 0 either way."""
    import csv

    out = tmp_path / "run.csv"
    _run_example("log_to_csv.py", ["--duration", "1", "--out", str(out)])
    with out.open(newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        rows = list(reader)

    assert header == [
        "t",
        "rx_monotonic",
        "seq",
        "target_rad",
        "position_rad",
        "velocity_radps",
        "torque_nm",
        "temperature_c",
        "fault_code",
    ]
    assert len(rows) > 100, f"only {len(rows)} rows for a 1 s run at 200 Hz"
    assert all(len(r) == len(header) for r in rows)
    assert float(rows[-1][0]) > 0.5, "the t column must advance across the run"
    assert len({r[2] for r in rows}) > 1, "seq must advance: feedback actually arrived"


def test_an_empty_run_does_not_truncate_the_previous_csv(
    tmp_path: pathlib.Path,
) -> None:
    """The write sits in a `finally` and opened "w" unconditionally, so a 401-row run
    followed by a 0-row run left the file with 0 rows - the successful run's data
    destroyed by the failed one after it."""
    out = tmp_path / "run.csv"
    _run_example("log_to_csv.py", ["--duration", "1", "--out", str(out)])
    good = out.read_text()
    assert good.count("\n") > 100

    # --duration 0 makes ticker.running(0.0) false on the first check, so the loop body
    # never runs and `rows` stays empty - exactly the measured case.
    result = _run_example("log_to_csv.py", ["--duration", "0", "--out", str(out)])
    assert out.read_text() == good, "an empty run must leave the previous file alone"
    assert "left as it was" in result.stderr
