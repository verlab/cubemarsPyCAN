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


EXAMPLES = _real((ROOT / "examples").glob("*.py"))
DOCS = _real((ROOT / "docs").glob("*.md"))


def test_there_are_examples_and_docs() -> None:
    assert {p.name for p in EXAMPLES} == {"mit_position_step.py", "servo_position.py"}
    assert {p.name for p in DOCS} == {
        "ak-2-0.md",
        "bench.md",
        "can-setup.md",
        "migration.md",
        "troubleshooting.md",
        "units.md",
    }


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.name)
def test_every_example_runs_against_the_simulator(path: pathlib.Path) -> None:
    """The --sim path exists so an example is never stale: CI runs it every commit."""
    args = ["--sim"]
    args += ["--duration", "0.3"] if "mit" in path.name else ["--steps", "30"]
    result = subprocess.run(
        [sys.executable, str(path), *args],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "no fault" in result.stdout


def test_the_mit_example_converges() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "examples" / "mit_position_step.py"),
            "--sim",
            "--duration",
            "1.0",
            "--amplitude",
            "0.25",
        ],
        capture_output=True,
        text=True,
        timeout=120,
        check=True,
    )
    last = result.stdout.replace("\r", "\n").strip().splitlines()[-2]
    assert "+0.25" in last, f"did not reach the setpoint: {last}"


def test_the_servo_example_converges() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "examples" / "servo_position.py"),
            "--sim",
            "--steps",
            "150",
            "--degrees",
            "90",
        ],
        capture_output=True,
        text=True,
        timeout=120,
        check=True,
    )
    last = result.stdout.replace("\r", "\n").strip().splitlines()[-2]
    assert "+90.0 deg" in last, f"did not reach the setpoint: {last}"


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


def test_bench_doc_records_what_was_measured() -> None:
    """Bench results belong in the docs, with the numbers, not just in a chat log."""
    bench = (ROOT / "docs" / "bench.md").read_text()
    for measurement in ("6.3867", "12.4985", "0.989", "5602"):
        assert measurement in bench, f"bench.md lost the {measurement} measurement"
    assert "Measured:" in bench
