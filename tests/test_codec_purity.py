"""The codec layer is pure, enforced structurally rather than by convention.

This is the seam the whole design rests on. If a codec could open a socket, read a clock
or hold mutable state, it would stop being testable without hardware - which is exactly
how TMotorCANControl accumulated thirty defects nobody could write a test for.
"""

from __future__ import annotations

import ast
import pathlib

import pytest

CODEC_DIR = pathlib.Path("src/cubemarspycan/codec")
# Skip dotfiles too: macOS tar emits AppleDouble `._name` siblings on transfer.
CODEC_FILES = sorted(
    p for p in CODEC_DIR.glob("*.py") if p.name != "__init__.py" and not p.name.startswith(".")
)

FORBIDDEN_IMPORTS = {
    "can": "would drag python-can into the codec and make it untestable in isolation",
    "serial": "serial belongs behind the [slcan] extra, never in a codec",
    "time": "a codec must not read a clock; timestamps come from the transport",
    "threading": "a codec must not be concurrency-aware",
    "asyncio": "a codec must not be concurrency-aware",
    "numpy": "not a dependency; its 2.0 integer behaviour broke the reference decoder",
    "socket": "a codec must not do I/O",
    "os": "a codec must not touch the environment",
    "logging": "a codec returns values; it does not narrate",
}


def top_level_imports(path: pathlib.Path) -> set[str]:
    tree = ast.parse(path.read_text())
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            found.add(node.module.split(".")[0])
    return found


def test_there_are_codecs_to_check() -> None:
    assert {p.name for p in CODEC_FILES} == {"mit.py", "servo_can.py"}


@pytest.mark.parametrize("path", CODEC_FILES, ids=lambda p: p.name)
def test_codec_imports_nothing_impure(path: pathlib.Path) -> None:
    offending = top_level_imports(path) & FORBIDDEN_IMPORTS.keys()
    assert not offending, "; ".join(f"{m}: {FORBIDDEN_IMPORTS[m]}" for m in sorted(offending))


@pytest.mark.parametrize("path", CODEC_FILES, ids=lambda p: p.name)
def test_codec_has_no_module_level_mutable_state(path: pathlib.Path) -> None:
    """Module-level lists/dicts/sets are shared across every motor on every bus."""
    tree = ast.parse(path.read_text())
    for node in tree.body:
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        value = node.value
        if isinstance(value, (ast.List, ast.Dict, ast.Set, ast.ListComp, ast.DictComp)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            names = [t.id for t in targets if isinstance(t, ast.Name)]
            pytest.fail(f"{path.name}: mutable module-level state {names}")


@pytest.mark.parametrize("path", CODEC_FILES, ids=lambda p: p.name)
def test_codec_defines_no_globals_statements(path: pathlib.Path) -> None:
    tree = ast.parse(path.read_text())
    assert not [n for n in ast.walk(tree) if isinstance(n, ast.Global)]


@pytest.mark.parametrize("path", CODEC_FILES, ids=lambda p: p.name)
def test_codec_functions_are_annotated(path: pathlib.Path) -> None:
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
            assert node.returns is not None, f"{path.name}:{node.name} lacks a return type"


@pytest.mark.parametrize("path", CODEC_FILES, ids=lambda p: p.name)
def test_codec_module_namespace_holds_no_can_objects(path: pathlib.Path) -> None:
    """The structural guarantee, checked at runtime rather than only in the AST.

    The package root does import python-can - it is the one runtime dependency - so
    "importing the codec pulls in can" is not the invariant. The invariant is that no
    python-can object is reachable from a codec's own namespace, which is what would let
    I/O creep in.
    """
    import importlib
    import types

    module = importlib.import_module(f"cubemarspycan.codec.{path.stem}")
    for name, value in vars(module).items():
        if name.startswith("__"):
            continue
        origin = getattr(value, "__module__", None)
        if isinstance(value, types.ModuleType):
            origin = value.__name__
        assert origin is None or not origin.startswith("can"), (
            f"{path.name} exposes {name} from {origin}"
        )
