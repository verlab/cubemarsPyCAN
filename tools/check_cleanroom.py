#!/usr/bin/env python3
"""Check that no source here overlaps textually with the GPLv3 reference library.

The licence for cubemarsPyCAN is undecided, so the implementation is written clean-room
from the manual and the CubeMars datasheets. Some resemblance is unavoidable and harmless:
the MIT bit layout is dictated by the protocol and there is only one way to write
``D3 = (v & 0xF) << 4 | kp >> 8``. The real risk is copied *structure* - class shapes,
dict layouts, docstrings - so this compares token shingles rather than lines.

    python tools/check_cleanroom.py [--reference DIR] [--threshold N]

Run it before any release, while the reference is still in the tree. Exits non-zero on a
run of matching tokens longer than the threshold.
"""

from __future__ import annotations

import argparse
import io
import pathlib
import tokenize

DEFAULT_REFERENCE = "TMotorCANControl-master"
DEFAULT_THRESHOLD = 12


def significant_tokens(path: pathlib.Path) -> list[str]:
    """Token text, minus comments, docstrings, whitespace and formatting."""
    skip = {
        tokenize.COMMENT,
        tokenize.NL,
        tokenize.NEWLINE,
        tokenize.INDENT,
        tokenize.DEDENT,
        tokenize.ENCODING,
        tokenize.ENDMARKER,
    }
    out: list[str] = []
    try:
        source = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return out
    try:
        for token in tokenize.generate_tokens(io.StringIO(source).readline):
            if token.type in skip or token.type == tokenize.STRING:
                continue
            out.append(token.string)
    except (tokenize.TokenError, IndentationError, SyntaxError):
        return out
    return out


MIN_DISTINCT_NAMES = 4


def _interesting(shingle: tuple[str, ...]) -> bool:
    """Ignore runs that carry no structure.

    A dozen commas from an ``__all__`` list, or a tuple of zeros, matches everything and
    means nothing. Require a few distinct identifiers before a run counts as evidence.
    """
    names = {t for t in shingle if t.isidentifier()}
    return len(names) >= MIN_DISTINCT_NAMES


def shingles(tokens: list[str], n: int) -> dict[tuple[str, ...], int]:
    out: dict[tuple[str, ...], int] = {}
    for i in range(len(tokens) - n + 1):
        shingle = tuple(tokens[i : i + n])
        if _interesting(shingle):
            out[shingle] = i
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="src")
    parser.add_argument("--reference", default=DEFAULT_REFERENCE)
    parser.add_argument(
        "--threshold",
        type=int,
        default=DEFAULT_THRESHOLD,
        help="longest run of identical tokens to tolerate",
    )
    args = parser.parse_args(argv)

    reference_root = pathlib.Path(args.reference)
    if not reference_root.exists():
        print(f"reference {args.reference} not present; nothing to compare against")
        return 0

    reference: dict[tuple[str, ...], str] = {}
    for path in reference_root.rglob("*.py"):
        if "__pycache__" in path.parts:
            continue
        for shingle in shingles(significant_tokens(path), args.threshold):
            reference.setdefault(shingle, str(path))

    hits: list[tuple[str, str]] = []
    for path in pathlib.Path(args.source).rglob("*.py"):
        for shingle in shingles(significant_tokens(path), args.threshold):
            if shingle in reference:
                hits.append((str(path), " ".join(shingle)))

    if not hits:
        print(
            f"clean: no run of {args.threshold} consecutive tokens in {args.source}/ "
            f"matches {args.reference}/"
        )
        return 0

    print(f"{len(hits)} overlapping run(s) of >= {args.threshold} tokens:\n")
    for source_path, text in hits[:20]:
        print(f"  {source_path}\n    {text[:120]}")
    print("\nReview each. Protocol-dictated expressions are fine; copied structure is not.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
