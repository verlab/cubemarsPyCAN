"""The python-can transport: URL handling, platform gates, and receive-thread survival."""

from __future__ import annotations

import shutil
import sys

import can
import pytest

from cubemarspycan import SendFailed, TransportError, UnsupportedPlatform
from cubemarspycan.frame import Frame
from cubemarspycan.transport.can_bus import (
    DEFAULT_BITRATE,
    CanTransport,
    parse_bitrate,
    read_socketcan_bitrate,
)

# --- URL parsing --------------------------------------------------------------------


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("1M", 1_000_000),
        ("1m", 1_000_000),
        ("500K", 500_000),
        ("500k", 500_000),
        ("1000000", 1_000_000),
        ("125K", 125_000),
    ],
)
def test_parse_bitrate(text: str, expected: int) -> None:
    assert parse_bitrate(text) == expected


@pytest.mark.parametrize("text", ["", "abc", "1G", "1.5M", "-1"])
def test_parse_bitrate_rejects_nonsense(text: str) -> None:
    with pytest.raises(ValueError, match="bitrate"):
        parse_bitrate(text)


def test_default_bitrate_is_1mbit() -> None:
    """The manual says the AK drivers use 1 Mbit/s and changing it is not recommended."""
    assert DEFAULT_BITRATE == 1_000_000


@pytest.mark.parametrize("url", ["nonsense", "no-scheme", "scheme:chan@bad@bits"])
def test_bad_urls_explain_the_format(url: str) -> None:
    with pytest.raises(ValueError, match="scheme:channel"):
        CanTransport.open(url)


def test_virtual_url_round_trips() -> None:
    with CanTransport.open("virtual:unit-test") as tp:
        assert "unit-test" in tp.channel_info


# --- platform gates -----------------------------------------------------------------


@pytest.mark.skipif(sys.platform == "linux", reason="socketcan exists on Linux")
def test_socketcan_refuses_clearly_off_linux() -> None:
    """A confusing OSError out of python-can helps nobody."""
    with pytest.raises(UnsupportedPlatform, match="Linux kernel facility"):
        CanTransport.open("socketcan:can0")


@pytest.mark.skipif(sys.platform == "linux", reason="socketcan exists on Linux")
def test_socketcan_refusal_names_the_alternative() -> None:
    with pytest.raises(UnsupportedPlatform) as info:
        CanTransport.open("socketcan:can0")
    message = str(info.value)
    assert "slcan" in message and "gs_usb" in message


@pytest.mark.skipif(sys.platform != "linux", reason="Linux only")
def test_socketcan_needs_an_interface_name() -> None:
    with pytest.raises(ValueError, match="interface name"):
        CanTransport.open("socketcan:")


def test_reading_a_missing_socketcan_bitrate_returns_none() -> None:
    assert read_socketcan_bitrate("definitely-not-an-interface") is None


# --- send ---------------------------------------------------------------------------


def test_send_puts_the_frame_on_the_bus() -> None:
    listener = can.interface.Bus(channel="tx-test", interface="virtual")
    with CanTransport.open("virtual:tx-test") as tp:
        tp.send(Frame(0x123, b"\x01\x02\x03"))
        msg = listener.recv(timeout=1.0)
    assert msg is not None
    assert msg.arbitration_id == 0x123
    assert bytes(msg.data) == b"\x01\x02\x03"
    assert not msg.is_extended_id
    listener.shutdown()


def test_send_preserves_the_extended_flag() -> None:
    listener = can.interface.Bus(channel="tx-ext", interface="virtual")
    with CanTransport.open("virtual:tx-ext") as tp:
        tp.send(Frame(0x2901, bytes(8), is_extended_id=True))
        msg = listener.recv(timeout=1.0)
    assert msg is not None and msg.is_extended_id
    listener.shutdown()


def test_send_counts_and_times() -> None:
    with CanTransport.open("virtual:tx-stats") as tp:
        for _ in range(5):
            tp.send(Frame(1, b"\x00"))
        assert tp.stats.tx == 5
        pct = tp.stats.tx_percentiles()
        assert set(pct) == {"p50", "p95", "max"}
        assert pct["max"] >= 0.0


def test_send_failure_is_raised_not_swallowed() -> None:
    """TMotorCANControl prints to stdout when debug is on and otherwise says nothing."""

    class Broken(can.BusABC):
        def __init__(self) -> None:
            self.channel_info = "broken"

        def send(self, msg: can.Message, timeout: float | None = None) -> None:
            raise can.CanError("no")

        def _recv_internal(self, timeout: float | None):  # type: ignore[no-untyped-def]
            return None, False

        def shutdown(self) -> None:
            pass

    tp = CanTransport(Broken())
    with pytest.raises(SendFailed, match="could not send"):
        tp.send(Frame(1, b"\x00"))
    assert tp.stats.tx_failed == 1
    assert tp.stats.errors


def test_send_failure_mentions_the_usual_socketcan_cause() -> None:
    class Broken(can.BusABC):
        def __init__(self) -> None:
            self.channel_info = "broken"

        def send(self, msg: can.Message, timeout: float | None = None) -> None:
            raise can.CanError("ENOBUFS")

        def _recv_internal(self, timeout: float | None):  # type: ignore[no-untyped-def]
            return None, False

        def shutdown(self) -> None:
            pass

    with pytest.raises(SendFailed, match="ENOBUFS"):
        CanTransport(Broken()).send(Frame(1, b"\x00"))


# --- receive thread survival --------------------------------------------------------


def test_a_sink_that_raises_does_not_kill_reception() -> None:
    """The receive thread must survive anything a sink does to it."""
    tp = CanTransport.virtual("sink-boom")
    received: list[Frame] = []

    def bad(frame: Frame, rx: float, ts: float) -> None:
        raise RuntimeError("sink bug")

    def good(frame: Frame, rx: float, ts: float) -> None:
        received.append(frame)

    tp.add_sink(bad)
    tp.add_sink(good)
    tp.on_message_received(can.Message(arbitration_id=1, data=b"\x01", is_extended_id=False))
    assert received, "the healthy sink still ran"
    assert tp.stats.errors
    tp.close()


def test_a_sink_raising_baseexception_is_still_contained() -> None:
    tp = CanTransport.virtual("sink-base")

    def nasty(frame: Frame, rx: float, ts: float) -> None:
        raise KeyboardInterrupt

    tp.add_sink(nasty)
    tp.on_message_received(can.Message(arbitration_id=1, data=b"\x01"))
    assert tp.stats.errors
    tp.close()


def test_error_and_remote_frames_are_ignored_not_decoded() -> None:
    tp = CanTransport.virtual("ignore")
    seen: list[Frame] = []
    tp.add_sink(lambda f, r, t: seen.append(f))
    tp.on_message_received(can.Message(arbitration_id=1, data=b"", is_error_frame=True))
    tp.on_message_received(can.Message(arbitration_id=1, data=b"", is_remote_frame=True))
    assert not seen
    assert tp.stats.rx_ignored == 2
    assert tp.stats.rx == 0
    tp.close()


def test_receive_uses_a_monotonic_clock_not_the_bus_timestamp() -> None:
    """Staleness must be measured against our own clock: bus timestamps are epoch-based
    and on some backends come from the driver with an unrelated origin."""
    import time

    tp = CanTransport.virtual("clock")
    captured: list[tuple[float, float]] = []
    tp.add_sink(lambda f, rx, ts: captured.append((rx, ts)))
    before = time.monotonic()
    tp.on_message_received(can.Message(arbitration_id=1, data=b"\x00", timestamp=0.0))
    after = time.monotonic()
    rx, bus_ts = captured[0]
    assert before <= rx <= after
    assert bus_ts == 0.0, "the bus timestamp is carried through for logs, unchanged"
    tp.close()


def test_on_error_records_rather_than_raising() -> None:
    tp = CanTransport.virtual("on-error")
    tp.on_error(RuntimeError("notifier blew up"))
    assert tp.stats.errors
    tp.close()


# --- lifecycle ----------------------------------------------------------------------


def test_close_is_idempotent() -> None:
    tp = CanTransport.virtual("close-twice")
    tp.start()
    tp.close()
    tp.close()


def test_an_injected_bus_is_not_shut_down_by_default() -> None:
    """You own what you construct; the transport only closes buses it opened."""
    bus = can.interface.Bus(channel="borrowed", interface="virtual")
    tp = CanTransport(bus)
    tp.start()
    tp.close()
    bus.send(can.Message(arbitration_id=1, data=b"\x00"))  # still usable
    bus.shutdown()


def test_set_filters_reaches_the_bus() -> None:
    bus = can.interface.Bus(channel="filters", interface="virtual")
    tp = CanTransport(bus)
    tp.set_filters([{"can_id": 0x29, "can_mask": 0xFF, "extended": True}])
    tp.close()
    bus.shutdown()


def test_a_missing_backend_extra_is_named_in_the_error() -> None:
    """A bare ImportError from python-can does not tell you what to install."""
    from cubemarspycan.transport import can_bus

    with pytest.raises(TransportError, match=r"cubemarspycan\[slcan\]"):
        can_bus._require("definitely_not_installed_xyz", "slcan", "pyserial")


def test_a_present_backend_passes_the_check() -> None:
    from cubemarspycan.transport import can_bus

    can_bus._require("json", "slcan", "pyserial")  # must not raise


# --- bitrate and state discovery (found on real hardware) ---------------------------


def test_bitrate_falls_back_from_sysfs_to_ip(monkeypatch: pytest.MonkeyPatch) -> None:
    """sysfs has no can_bittiming on some driver/kernel combinations.

    A gs_usb adapter on Linux 7.x has no such directory, so a sysfs-only reader reports
    "unknown" for a perfectly healthy 1 Mbit/s interface. Found on real hardware.
    """
    from cubemarspycan.transport import can_bus

    monkeypatch.setattr(can_bus, "_link_info", lambda _: {"bittiming": {"bitrate": 1_000_000}})
    assert can_bus.read_socketcan_bitrate("no-such-iface") == 1_000_000


def test_bitrate_is_none_when_neither_source_knows(monkeypatch: pytest.MonkeyPatch) -> None:
    from cubemarspycan.transport import can_bus

    monkeypatch.setattr(can_bus, "_link_info", lambda _: {})
    assert can_bus.read_socketcan_bitrate("no-such-iface") is None


def test_can_state_is_read(monkeypatch: pytest.MonkeyPatch) -> None:
    from cubemarspycan.transport import can_bus

    monkeypatch.setattr(can_bus, "_link_info", lambda _: {"state": "ERROR-PASSIVE"})
    assert can_bus.read_socketcan_state("x") == "ERROR-PASSIVE"
    monkeypatch.setattr(can_bus, "_link_info", lambda _: {})
    assert can_bus.read_socketcan_state("x") is None


def test_link_info_never_raises_on_a_missing_interface() -> None:
    from cubemarspycan.transport import can_bus

    assert can_bus._link_info("definitely-not-an-interface") == {}


def test_link_info_survives_a_missing_ip_command(monkeypatch: pytest.MonkeyPatch) -> None:
    import subprocess

    from cubemarspycan.transport import can_bus

    def no_ip(*args: object, **kwargs: object) -> None:
        raise FileNotFoundError("ip")

    monkeypatch.setattr(subprocess, "run", no_ip)
    assert can_bus._link_info("can0") == {}


def test_link_info_survives_garbage_output(monkeypatch: pytest.MonkeyPatch) -> None:
    import subprocess
    from types import SimpleNamespace

    from cubemarspycan.transport import can_bus

    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *a, **k: SimpleNamespace(returncode=0, stdout="not json", stderr=""),
    )
    assert can_bus._link_info("can0") == {}


# --- link flags: the operstate trap --------------------------------------------------


@pytest.mark.parametrize(
    ("entry", "answered", "expected"),
    [
        # `ip` answered and the UP flag is set: up, whatever operstate says.
        ({"flags": ["NOARP", "UP", "LOWER_UP"]}, True, True),
        ({"flags": ["NOARP", "UP", "LOWER_UP", "ECHO"]}, True, True),
        # `ip` answered and the flag is absent: created, never brought up.
        ({"flags": ["NOARP"]}, True, False),
        # `ip` answered "no such device". An absence is an answer, so False - not None.
        ({}, True, False),
        # `ip` could not be asked at all. The third outcome, and the only None.
        ({}, False, None),
    ],
)
def test_up_is_decided_by_the_flag_not_the_operstate(
    entry: dict[str, object],
    answered: bool,
    expected: bool | None,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A virtual CAN interface has no carrier, so it reports `state UNKNOWN` however
    healthy it is; real CAN hardware reports `state UP`.

    Keying off operstate therefore calls a working vcan interface down. It silently
    skipped the whole socketcan suite on CI even after vcan0 came up correctly, and only
    showed up because that job fails when it selects nothing that passes.

    The interface named here cannot exist, deliberately. `socketcan_is_up` reads sysfs
    first, so naming a real one ("vcan0", as this test used to) makes the monkeypatch
    inert on any host where that interface happens to be up - the False and None rows
    then fail, and only the accident of CI job configuration hid it.
    """
    from cubemarspycan.transport import can_bus

    monkeypatch.setattr(can_bus, "_link_probe", lambda _: can_bus._LinkProbe(entry, answered))
    assert can_bus.socketcan_is_up("definitely-no-such-iface") is expected


def test_link_flags_cannot_tell_absence_from_unreadability() -> None:
    """The flag list is the lossy view, and stays that way: use socketcan_is_up when the
    difference matters."""
    from cubemarspycan.transport import can_bus

    assert can_bus.socketcan_link_flags("definitely-not-an-interface") == []


@pytest.mark.skipif(shutil.which("ip") is None, reason="no iproute2 to answer with")
def test_a_definitively_missing_interface_reads_as_down() -> None:
    """`ip` runs, returns non-zero, and that is a real answer - not a shrug."""
    from cubemarspycan.transport import can_bus

    assert can_bus.socketcan_is_up("definitely-not-an-interface") is False


@pytest.mark.skipif(shutil.which("ip") is not None, reason="this host can ask `ip`")
def test_without_iproute2_a_missing_interface_is_honestly_unknown() -> None:
    """macOS has neither /sys nor `ip`. "Cannot tell" is the *true* answer here, and
    asserting False would be asserting something this host cannot know."""
    from cubemarspycan.transport import can_bus

    assert can_bus.socketcan_is_up("definitely-not-an-interface") is None


@pytest.mark.parametrize(
    ("returncode", "stdout", "entry", "answered"),
    [
        (1, "", {}, True),  # `ip`: no such device
        (0, "[]", {}, True),  # ran, matched nothing
        (0, "not json", {}, False),  # unparseable
        (0, '["not a dict"]', {}, False),  # wrong shape
        (0, "null", {}, False),  # not even a list
        (0, '[{"flags": ["UP"]}]', {"flags": ["UP"]}, True),  # the happy path
    ],
)
def test_the_probe_separates_a_negative_answer_from_no_answer(
    returncode: int,
    stdout: str,
    entry: dict[str, object],
    answered: bool,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The whole tri-state lives here; everything above is a consequence."""
    import subprocess
    from types import SimpleNamespace

    from cubemarspycan.transport import can_bus

    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *a, **k: SimpleNamespace(returncode=returncode, stdout=stdout, stderr=""),
    )
    probe = can_bus._link_probe("can0")
    assert (probe.entry, probe.answered) == (entry, answered)
    assert can_bus._link_entry("can0") == entry, "the dict view never changes shape"


def test_a_missing_ip_command_is_not_an_answer(monkeypatch: pytest.MonkeyPatch) -> None:
    import subprocess

    from cubemarspycan.transport import can_bus

    def no_ip(*args: object, **kwargs: object) -> None:
        raise FileNotFoundError("ip")

    monkeypatch.setattr(subprocess, "run", no_ip)
    assert can_bus._link_probe("can0").answered is False
    assert can_bus._link_entry("can0") == {}, "still a dict, never None"


def test_link_entry_survives_garbage(monkeypatch: pytest.MonkeyPatch) -> None:
    import subprocess
    from types import SimpleNamespace

    from cubemarspycan.transport import can_bus

    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *a, **k: SimpleNamespace(returncode=0, stdout="[]", stderr=""),
    )
    assert can_bus._link_entry("can0") == {}
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *a, **k: SimpleNamespace(returncode=0, stdout='["not a dict"]', stderr=""),
    )
    assert can_bus._link_entry("can0") == {}


def test_link_status_asks_ip_once_for_all_three_facts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """`doctor` asked three helpers for three keys of one JSON object, forking `ip` three
    times per interface and multiplying its 2 s timeout with it."""
    import subprocess
    from types import SimpleNamespace

    from cubemarspycan.transport import can_bus

    calls: list[object] = []
    payload = (
        '[{"flags": ["UP"], "linkinfo": {"info_data": '
        '{"state": "ERROR-ACTIVE", "bittiming": {"bitrate": 1000000}}}}]'
    )

    def record(*args: object, **kwargs: object) -> SimpleNamespace:
        calls.append(args[0])
        return SimpleNamespace(returncode=0, stdout=payload, stderr="")

    monkeypatch.setattr(subprocess, "run", record)
    status = can_bus.read_link_status("can0")

    assert len(calls) == 1, f"one probe, not {len(calls)}"
    assert (status.up, status.bitrate, status.state) == (True, 1000000, "ERROR-ACTIVE")


def test_a_null_bittiming_does_not_raise(monkeypatch: pytest.MonkeyPatch) -> None:
    """`info_data.get("bittiming", {}).get(...)` raised AttributeError on `null` - the
    same trap as `linkinfo`, one level down."""
    import subprocess
    from types import SimpleNamespace

    from cubemarspycan.transport import can_bus

    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *a, **k: SimpleNamespace(
            returncode=0,
            stdout='[{"linkinfo": {"info_data": {"bittiming": null}}}]',
            stderr="",
        ),
    )
    assert can_bus.read_socketcan_bitrate("can0") is None
    assert can_bus.read_link_status("can0").bitrate is None
