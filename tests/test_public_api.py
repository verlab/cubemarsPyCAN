"""Invariants of the package surface itself."""

from __future__ import annotations

import subprocess
import sys

import cubemarspycan


def test_importing_the_package_does_not_pull_in_pyserial() -> None:
    """pyserial lives behind the [slcan] extra; a plain socketcan install must not need it."""
    code = (
        "import sys, cubemarspycan; "
        "assert 'serial' not in sys.modules, sorted(m for m in sys.modules if 'serial' in m)"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_importing_the_package_does_not_pull_in_numpy() -> None:
    """NumPy is not a dependency at all: its 2.0 integer behaviour broke the reference."""
    code = "import sys, cubemarspycan; assert 'numpy' not in sys.modules"
    subprocess.run([sys.executable, "-c", code], check=True)


def test_all_is_complete_and_unique() -> None:
    """Ordering is ruff's job (RUF022); this pins that every name actually resolves."""
    assert len(cubemarspycan.__all__) == len(set(cubemarspycan.__all__))
    for name in cubemarspycan.__all__:
        assert hasattr(cubemarspycan, name), name


def test_all_covers_every_public_name() -> None:
    """Catches a re-export added to the module but forgotten in __all__."""
    import types

    exported = set(cubemarspycan.__all__)
    # `annotations` is the __future__ feature object that `from __future__ import
    # annotations` binds into every module; it is not part of our surface.
    for name, value in vars(cubemarspycan).items():
        if name.startswith("_") or isinstance(value, types.ModuleType):
            continue
        if name == "annotations":
            continue
        assert name in exported, f"{name} is public but missing from __all__"


def test_version_is_exported() -> None:
    assert cubemarspycan.__version__.startswith("0.")
