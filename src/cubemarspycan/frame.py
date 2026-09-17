"""A transport-neutral CAN frame.

This module deliberately does **not** import ``can``. It is the shared vocabulary between
the pure codec layer and the transport layer; if it depended on python-can, the codec
would too, and the codec would stop being testable in isolation.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

STANDARD_ID_MAX = 0x7FF
EXTENDED_ID_MAX = 0x1FFFFFFF


@dataclass(frozen=True, slots=True)
class Frame:
    """One classic-CAN data frame: an arbitration id, up to 8 payload bytes, and a flag."""

    arbitration_id: int
    data: bytes
    is_extended_id: bool = False

    def __post_init__(self) -> None:
        # Accept bytearray/list/tuple for ergonomics, but store immutable bytes so the
        # frozen dataclass is genuinely frozen all the way down. Typed as Any so the
        # runtime coercion is not narrowed away by the declared annotation.
        raw: Any = self.data
        if isinstance(raw, int):
            # bytes(4) would silently produce four zero bytes, which is never intended.
            raise TypeError("Frame.data must be a byte sequence, not an int")
        object.__setattr__(self, "data", bytes(raw))
        limit = EXTENDED_ID_MAX if self.is_extended_id else STANDARD_ID_MAX
        if not 0 <= self.arbitration_id <= limit:
            kind = "extended" if self.is_extended_id else "standard"
            raise ValueError(
                f"arbitration id 0x{self.arbitration_id:X} out of range for a "
                f"{kind} frame (max 0x{limit:X})"
            )
        if len(self.data) > 8:
            raise ValueError(f"classic CAN payload is at most 8 bytes, got {len(self.data)}")

    @property
    def dlc(self) -> int:
        """Data length code: the payload length in bytes, 0-8.

        AK frames are DLC 8 in both modes; anything else is a foreign frame.
        """
        return len(self.data)

    def hex(self) -> str:
        """The payload as space-separated uppercase hex, e.g. ``"FF FF FF FF FF FF FF FC"``.

        For logs and for comparing against the manual's byte tables.
        """
        return " ".join(f"{b:02X}" for b in self.data)

    def __str__(self) -> str:
        width = 8 if self.is_extended_id else 3
        tag = "x" if self.is_extended_id else " "
        return f"{self.arbitration_id:0{width}X}{tag} [{self.dlc}] {self.hex()}"


def frame(arbitration_id: int, data: Iterable[int], *, extended: bool = False) -> Frame:
    """Convenience constructor that accepts any iterable of byte values."""
    return Frame(arbitration_id, bytes(data), extended)
