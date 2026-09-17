"""python-can transport.

The bus is **injected**. ``CanTransport(bus)`` takes any ``can.BusABC`` you have already
constructed, so socketcan, slcan, gs_usb, PCAN, Kvaser and the virtual backend all work
without this module knowing about them. :meth:`CanTransport.open` is sugar over the common
cases and is never the only path.

Nothing here shells out. TMotorCANControl runs ``os.system('sudo /sbin/ip link set can0
up ...')`` from a singleton's ``__new__``, which hard-codes the interface and the bitrate,
requires root, only works on Linux, and leaves no seam to inject a bus - which is why none
of its thirty defects had a test.
"""

from __future__ import annotations

import json
import logging
import platform
import re
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import can

from ..errors import SendFailed, TransportError, UnsupportedPlatform
from ..frame import Frame
from .base import FrameSink

log = logging.getLogger(__name__)

DEFAULT_BITRATE = 1_000_000
"""The AK drivers use 1 Mbit/s. The manual says changing it is not recommended."""

_URL = re.compile(r"^(?P<scheme>[a-z0-9_]+):(?P<channel>[^@]*)(?:@(?P<bitrate>[0-9]+[KMkm]?))?$")


def parse_bitrate(text: str) -> int:
    """``"1M"`` -> 1000000, ``"500K"`` -> 500000, ``"1000000"`` -> 1000000."""
    match = re.fullmatch(r"(\d+)([KMkm]?)", text)
    if not match:
        raise ValueError(f"cannot parse bitrate {text!r}")
    value = int(match.group(1))
    return value * {"": 1, "k": 1_000, "K": 1_000, "m": 1_000_000, "M": 1_000_000}[match.group(2)]


@dataclass
class TransportStats:
    """Counters worth looking at when something is not working.

    Mutable on purpose: this is diagnostics, not state that crosses a thread boundary as
    a snapshot.
    """

    rx: int = 0
    tx: int = 0
    rx_ignored: int = 0
    """Error and remote frames, which carry no payload for us."""
    tx_failed: int = 0
    errors: deque[tuple[str, str]] = field(default_factory=lambda: deque(maxlen=16))
    tx_durations: deque[float] = field(default_factory=lambda: deque(maxlen=1024))

    def record_error(self, where: str, exc: BaseException) -> None:
        self.errors.append((where, f"{type(exc).__name__}: {exc}"))

    def tx_percentiles(self) -> dict[str, float]:
        """p50/p95/max of send() wall time, in milliseconds.

        Worth watching on slcan: a send is an ASCII line over USB CDC, typically 0.5-2 ms
        with scheduling spikes into the tens of milliseconds. That is what decides whether
        a given loop rate is realistic on a given adapter.
        """
        if not self.tx_durations:
            return {"p50": 0.0, "p95": 0.0, "max": 0.0}
        ordered = sorted(self.tx_durations)
        n = len(ordered)
        return {
            "p50": ordered[n // 2] * 1e3,
            "p95": ordered[min(n - 1, int(n * 0.95))] * 1e3,
            "max": ordered[-1] * 1e3,
        }


class CanTransport(can.Listener):
    """Wraps an injected ``can.BusABC`` and fans received frames out to sinks."""

    def __init__(self, bus: can.BusABC, *, owns_bus: bool = False) -> None:
        self._bus = bus
        self._owns_bus = owns_bus
        self._sinks: list[FrameSink] = []
        self._notifier: can.Notifier | None = None
        self._lock = threading.Lock()
        self._closed = False
        self.stats = TransportStats()

    # --- construction sugar ---------------------------------------------------------

    @classmethod
    def open(cls, url: str, **kwargs: Any) -> CanTransport:
        """Build a transport from ``scheme:channel[@bitrate]``.

        ``socketcan:can0`` (Linux, production), ``slcan:/dev/tty.usbmodem1101@1M``,
        ``gs_usb:0@1M``, ``virtual:test``.
        """
        match = _URL.match(url)
        if not match:
            raise ValueError(
                f"cannot parse {url!r}; expected scheme:channel[@bitrate], e.g. "
                f"'socketcan:can0' or 'slcan:/dev/tty.usbmodem1101@1M'"
            )
        scheme = match.group("scheme")
        channel = match.group("channel")
        bitrate = (
            parse_bitrate(match.group("bitrate")) if match.group("bitrate") else DEFAULT_BITRATE
        )

        if scheme == "socketcan":
            return cls(_open_socketcan(channel, bitrate), owns_bus=True)
        if scheme == "virtual":
            return cls(
                can.interface.Bus(channel=channel or "cubemars", interface="virtual"),
                owns_bus=True,
            )
        if scheme == "slcan":
            _require("serial", "slcan", "pyserial")
        if scheme == "gs_usb":
            _require("gs_usb", "gs-usb", "gs-usb")
        try:
            bus = can.interface.Bus(channel=channel, interface=scheme, bitrate=bitrate, **kwargs)
        except Exception as exc:
            raise TransportError(f"could not open {url!r}: {exc}") from exc
        return cls(bus, owns_bus=True)

    @classmethod
    def virtual(cls, channel: str = "cubemars") -> CanTransport:
        """An in-process bus. Used by the simulator and by CI on every platform."""
        return cls(can.interface.Bus(channel=channel, interface="virtual"), owns_bus=True)

    # --- lifecycle ------------------------------------------------------------------

    def add_sink(self, sink: FrameSink) -> None:
        """Register a receiver. Sinks must be added before :meth:`start`."""
        self._sinks.append(sink)

    def start(self) -> None:
        if self._notifier is None:
            self._notifier = can.Notifier(bus=self._bus, listeners=[self])

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._notifier is not None:
            self._notifier.stop()
            self._notifier = None
        if self._owns_bus:
            self._bus.shutdown()

    def __enter__(self) -> CanTransport:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def bus(self) -> can.BusABC:
        return self._bus

    @property
    def channel_info(self) -> str:
        return str(getattr(self._bus, "channel_info", self._bus))

    # --- transmit -------------------------------------------------------------------

    def send(self, frame: Frame, timeout: float | None = 0.05) -> None:
        """Put one frame on the bus. Raises :class:`SendFailed` rather than swallowing."""
        message = can.Message(
            arbitration_id=frame.arbitration_id,
            data=frame.data,
            is_extended_id=frame.is_extended_id,
        )
        started = time.monotonic()
        try:
            with self._lock:
                self._bus.send(message, timeout=timeout)
        except can.CanError as exc:
            self.stats.tx_failed += 1
            self.stats.record_error("send", exc)
            raise SendFailed(
                f"could not send {frame} on {self.channel_info}: {exc}. On socketcan a "
                f"full transmit queue (ENOBUFS) usually means the bus is not connected "
                f"or no other node is acknowledging."
            ) from exc
        finally:
            self.stats.tx_durations.append(time.monotonic() - started)
        self.stats.tx += 1

    def set_filters(self, filters: Any | None) -> None:
        """Install hardware/kernel receive filters.

        On socketcan these are applied in the kernel, so the receive thread is not woken
        for traffic belonging to other nodes. Opt-in: a wrong filter drops frames silently,
        which is a worse failure than a few wasted wakeups.
        """
        self._bus.set_filters(filters)

    # --- receive: runs on the notifier thread and MUST NOT RAISE --------------------

    def on_message_received(self, msg: can.Message) -> None:
        rx_monotonic = time.monotonic()
        try:
            if msg.is_error_frame or msg.is_remote_frame:
                self.stats.rx_ignored += 1
                return
            frame = Frame(msg.arbitration_id, bytes(msg.data), msg.is_extended_id)
            self.stats.rx += 1
        except BaseException as exc:
            self.stats.record_error("frame-build", exc)
            return
        for sink in tuple(self._sinks):
            try:
                sink(frame, rx_monotonic, msg.timestamp)
            except BaseException as exc:
                # BaseException, not Exception: a sink that raises KeyboardInterrupt must
                # not take the receive thread down with it and silence every motor.
                self.stats.record_error("sink", exc)

    def on_error(self, exc: Exception) -> None:
        """python-can's own notifier error hook."""
        self.stats.record_error("notifier", exc)


# --- backend helpers -----------------------------------------------------------------


def _require(module: str, extra: str, package: str) -> None:
    try:
        __import__(module)
    except ImportError as exc:
        raise TransportError(
            f"the {extra} backend needs {package}; install it with "
            f'`pip install "cubemarspycan[{extra}]"`'
        ) from exc


def _open_socketcan(channel: str, bitrate: int) -> can.BusABC:
    if platform.system() != "Linux":
        raise UnsupportedPlatform(
            f"socketcan is a Linux kernel facility and does not exist on "
            f"{platform.system()}. For development here use a USB-CAN adapter: "
            f"'slcan:/dev/tty.usbmodem...@1M' or 'gs_usb:0@1M'."
        )
    if not channel:
        raise ValueError("socketcan needs an interface name, e.g. 'socketcan:can0'")

    state = read_socketcan_state(channel)
    if state in ("ERROR-PASSIVE", "BUS-OFF"):
        log.warning(
            "%s is in CAN state %s. That usually means no other powered node is "
            "acknowledging frames - check motor power and bus termination. Clear it with: "
            "sudo ip link set %s down && sudo ip link set %s up type can bitrate %d",
            channel,
            state,
            channel,
            channel,
            bitrate,
        )

    configured = read_socketcan_bitrate(channel)
    if configured is not None and configured != bitrate:
        log.warning(
            "%s is configured for %d bit/s but %d was requested. socketcan bitrate is set "
            "by the kernel, not by python-can, so the interface wins. Reconfigure with: "
            "sudo ip link set %s down && sudo ip link set %s up type can bitrate %d",
            channel,
            configured,
            bitrate,
            channel,
            channel,
            bitrate,
        )
    try:
        return can.interface.Bus(channel=channel, interface="socketcan")
    except OSError as exc:
        raise TransportError(
            f"could not open socketcan interface {channel!r}: {exc}. Bring it up first: "
            f"sudo ip link set {channel} up type can bitrate {bitrate}"
        ) from exc


@dataclass(frozen=True, slots=True)
class _LinkProbe:
    """What one ``ip -details -json link show`` could tell us.

    Three outcomes, not two. ``answered`` is True whenever ``ip`` itself gave a verdict we
    can trust - **including** "no such device", which is a negative answer rather than a
    failure to ask. Collapsing that into the same empty result as "iproute2 is not
    installed" is why a definitively absent interface used to report "cannot tell".
    """

    entry: dict[str, Any] = field(default_factory=dict)
    answered: bool = False


def _link_probe(interface: str) -> _LinkProbe:
    """Run ``ip -details -json link show <interface>`` once, keeping the outcomes apart.

    Read-only, unprivileged, never raises.
    """
    try:
        result = subprocess.run(
            ["ip", "-details", "-json", "link", "show", interface],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return _LinkProbe()  # no iproute2, or it hung: we could not ask
    if result.returncode != 0:
        return _LinkProbe(answered=True)  # `ip` ran and said: no such interface
    try:
        entries = json.loads(result.stdout)
    except json.JSONDecodeError:
        return _LinkProbe()  # unparseable: do not guess
    if not isinstance(entries, list):
        return _LinkProbe()
    if not entries:
        return _LinkProbe(answered=True)  # ran, matched nothing
    if not isinstance(entries[0], dict):
        return _LinkProbe()
    return _LinkProbe(entries[0], answered=True)


def _link_entry(interface: str) -> dict[str, Any]:
    """The whole ``ip -details -json link show`` entry for ``interface``, or ``{}``.

    Always a dict, never ``None``: :func:`_link_info` and :func:`socketcan_link_flags`
    call ``.get`` on the result, and an ``AttributeError`` out of the diagnostics path is
    exactly the bug this shape exists to make unrepresentable.
    """
    return _link_probe(interface).entry


def _flags_of(entry: dict[str, Any]) -> list[str]:
    flags = entry.get("flags", [])
    return [str(f) for f in flags] if isinstance(flags, list) else []


def _info_data_of(entry: dict[str, Any]) -> dict[str, Any]:
    """The CAN ``info_data`` block of a link entry, or ``{}``.

    Every level is isinstance-checked. ``ip`` emits ``null`` for an absent sub-object, and
    ``{}.get("linkinfo", {}).get(...)`` raises ``AttributeError`` on that - on the
    diagnostics path, where an exception is worth less than a shrug.
    """
    linkinfo = entry.get("linkinfo")
    if not isinstance(linkinfo, dict):
        return {}
    info_data = linkinfo.get("info_data")
    return info_data if isinstance(info_data, dict) else {}


def _bitrate_of(info_data: dict[str, Any]) -> int | None:
    bittiming = info_data.get("bittiming")
    if not isinstance(bittiming, dict):
        return None
    bitrate = bittiming.get("bitrate")
    return int(bitrate) if isinstance(bitrate, int) and bitrate else None


def _sysfs_bitrate(interface: str) -> int | None:
    try:
        value = int(Path(f"/sys/class/net/{interface}/can_bittiming/bitrate").read_text())
    except (OSError, ValueError):
        return None
    return value or None


def _sysfs_up(interface: str) -> bool | None:
    """IFF_UP from sysfs, or ``None`` when sysfs cannot answer. Needs no external binary."""
    try:
        raw = Path(f"/sys/class/net/{interface}/flags").read_text().strip()
        return bool(int(raw, 16) & 0x1)  # IFF_UP
    except (OSError, ValueError):
        return None


@dataclass(frozen=True, slots=True)
class LinkStatus:
    """Everything ``doctor`` reports about one interface, from a single ``ip`` call."""

    up: bool | None
    bitrate: int | None
    state: str | None


def read_link_status(interface: str) -> LinkStatus:
    """All three link facts from **one** ``ip`` invocation.

    The three single-fact readers below each run their own probe, which is right for
    standalone use and wasteful in a loop: ``doctor`` was forking ``ip`` up to three times
    per interface for three keys of one JSON object, multiplying the 2 s timeout by three.
    """
    probe = _link_probe(interface)
    info_data = _info_data_of(probe.entry)
    up = _sysfs_up(interface)
    if up is None:
        up = ("UP" in _flags_of(probe.entry)) if probe.answered else None
    state = info_data.get("state")
    return LinkStatus(
        up=up,
        bitrate=_sysfs_bitrate(interface) or _bitrate_of(info_data),
        state=state if isinstance(state, str) else None,
    )


def _link_info(interface: str) -> dict[str, Any]:
    """CAN-specific link details from ``ip -details -json link show``.

    sysfs exposes ``can_bittiming`` on some kernel and driver combinations but not all - a
    gs_usb adapter on Linux 6.x/7.x has no such directory - so sysfs alone silently
    reports "unknown" for a perfectly healthy interface. ``ip`` reports it everywhere.

    Read-only, no privileges. Any failure means "unknown", never an exception.
    """
    return _info_data_of(_link_entry(interface))


def read_socketcan_bitrate(interface: str) -> int | None:
    """The kernel's configured bitrate, or ``None`` if it cannot be determined.

    python-can cannot set a socketcan bitrate - ``ip link`` does - so the only honest
    check is to read back what the interface is actually running at. A virtual interface
    has no bit timing at all, and returns ``None``.
    """
    return _sysfs_bitrate(interface) or _bitrate_of(_link_info(interface))


def socketcan_link_flags(interface: str) -> list[str]:
    """Link flags for ``interface``, e.g. ``["NOARP", "UP", "LOWER_UP"]``.

    Empty if the interface does not exist or cannot be read. Never raises.
    """
    return _flags_of(_link_entry(interface))


def socketcan_is_up(interface: str) -> bool | None:
    """Whether ``interface`` is administratively up. ``None`` means "cannot tell".

    Reads the ``UP`` **flag**, not ``operstate``. A virtual CAN interface has no carrier,
    so it reports ``state UNKNOWN`` however healthy it is, while real CAN hardware reports
    ``state UP``. Anything that keys off operstate will call a working vcan interface
    down.

    ``False`` also covers "there is no such interface". A definitive absence is an answer;
    reporting it as "cannot tell" hides a typo'd interface name behind a shrug.

    ``None`` is returned only when neither sysfs nor ``ip`` could be consulted at all - no
    ``/sys``, no iproute2, a slim container, a BusyBox rootfs, macOS. Reporting *that* as
    "down" is how a healthy interface gets blamed, which is the failure this replaced.

    sysfs is tried first so a host without iproute2 still gets a real answer.
    """
    return read_link_status(interface).up


def read_socketcan_state(interface: str) -> str | None:
    """The CAN controller's error state, e.g. ``ERROR-ACTIVE`` or ``BUS-OFF``.

    Worth surfacing: a controller in ERROR-PASSIVE or BUS-OFF usually means nothing is
    acknowledging its frames - no other powered node, or missing termination - which
    looks identical to a software fault from the application's side.
    """
    state = _link_info(interface).get("state")
    return state if isinstance(state, str) else None
