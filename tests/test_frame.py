"""The transport-neutral frame."""

from __future__ import annotations

import ast
import pathlib

import pytest

from cubemarspycan.frame import EXTENDED_ID_MAX, STANDARD_ID_MAX, Frame, frame


def test_accepts_any_byte_iterable_but_stores_bytes() -> None:
    for payload in (b"\x01\x02", bytearray(b"\x01\x02"), [1, 2], (1, 2)):
        f = Frame(1, payload)  # type: ignore[arg-type]
        assert isinstance(f.data, bytes)
        assert f.data == b"\x01\x02"


def test_dlc_and_hex() -> None:
    f = Frame(0x123, bytes(range(8)))
    assert f.dlc == 8
    assert f.hex() == "00 01 02 03 04 05 06 07"
    assert "123" in str(f)


def test_extended_frames_render_differently() -> None:
    assert str(Frame(0x2901, b"\x00", is_extended_id=True)).startswith("00002901x")
    assert str(Frame(0x123, b"\x00")).startswith("123 ")


def test_standard_id_bounds() -> None:
    Frame(STANDARD_ID_MAX, b"")
    with pytest.raises(ValueError, match="standard frame"):
        Frame(STANDARD_ID_MAX + 1, b"")


def test_extended_id_bounds() -> None:
    Frame(EXTENDED_ID_MAX, b"", is_extended_id=True)
    with pytest.raises(ValueError, match="extended frame"):
        Frame(EXTENDED_ID_MAX + 1, b"", is_extended_id=True)


def test_negative_id_rejected() -> None:
    with pytest.raises(ValueError, match="out of range"):
        Frame(-1, b"")


def test_payload_length_capped_at_classic_can() -> None:
    Frame(1, bytes(8))
    with pytest.raises(ValueError, match="at most 8 bytes"):
        Frame(1, bytes(9))


def test_frames_are_frozen_and_hashable() -> None:
    f = Frame(1, b"\x01")
    with pytest.raises(AttributeError):
        f.arbitration_id = 2  # type: ignore[misc]
    assert {f, Frame(1, b"\x01")} == {f}


def test_helper_constructor() -> None:
    assert frame(1, [0xFF] * 8) == Frame(1, b"\xff" * 8)
    assert frame(0x2901, [0], extended=True).is_extended_id


def test_frame_module_does_not_import_can() -> None:
    """Load-bearing: if frame.py depended on python-can, the codec layer would too."""
    tree = ast.parse(pathlib.Path("src/cubemarspycan/frame.py").read_text())
    imported: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            imported.add(node.module.split(".")[0])
    assert "can" not in imported
    assert imported <= {"__future__", "collections", "dataclasses", "typing"}


def test_int_payload_is_rejected_rather_than_silently_zero_filled() -> None:
    """bytes(4) is four zero bytes, which is never what a caller meant."""
    with pytest.raises(TypeError, match="not an int"):
        Frame(1, 4)  # type: ignore[arg-type]
