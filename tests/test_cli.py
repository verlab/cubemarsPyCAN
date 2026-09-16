"""The command-line tools.

`scan` is the one that matters on a bench: it answers "is anything there, what id, which
mode, and which arbitration id does MIT reply on" - the last of which the manual leaves
ambiguous and which every other library hard-codes.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Iterator

import can
import pytest

from cubemarspycan import get_spec
from cubemarspycan.cli import Scanner, build_parser, main
from cubemarspycan.codec import mit as mit_codec
from cubemarspycan.codec import servo_can as servo_codec
from cubemarspycan.frame import Frame
from cubemarspycan.sim import SimMitDriver, SimServoDriver

SPEC = get_spec("AK40-10")


class Broadcaster:
    """Pushes simulated replies onto a virtual bus from a background thread."""

    def __init__(self, channel: str, frames: list[Frame], period: float = 0.01) -> None:
        self.bus = can.interface.Bus(
            channel=channel, interface="virtual", receive_own_messages=False
        )
        self.frames = frames
        self.period = period
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._stop.is_set():
            for frame in self.frames:
                self.bus.send(
                    can.Message(
                        arbitration_id=frame.arbitration_id,
                        data=frame.data,
                        is_extended_id=frame.is_extended_id,
                    )
                )
            time.sleep(self.period)

    def __enter__(self) -> Broadcaster:
        self._thread.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.bus.shutdown()


@pytest.fixture
def channel(request: pytest.FixtureRequest) -> Iterator[str]:
    yield f"cli-{request.node.name}"


# --- Scanner classification ---------------------------------------------------------


def mit_reply(motor_id: int, arbitration_id: int = 0x00) -> Frame:
    return Frame(arbitration_id, bytes([motor_id, 0x80, 0, 0x80, 0, 0, 60, 0]))


def servo_status(motor_id: int) -> Frame:
    return Frame(
        servo_codec.arbitration_id(servo_codec.ServoFunction.STATUS, motor_id),
        bytes(8),
        is_extended_id=True,
    )


def test_scanner_classifies_mit_replies() -> None:
    scanner = Scanner(SPEC)
    scanner.on_frame(mit_reply(1), 0.0)
    assert scanner.sightings[1].mode == "MIT"
    assert scanner.sightings[1].mit_replies == 1
    assert scanner.sightings[1].last_mit is not None


def test_scanner_records_which_arbitration_id_mit_answered_on() -> None:
    """The ambiguity in the manual, resolved by observation."""
    scanner = Scanner(SPEC)
    scanner.on_frame(mit_reply(1, arbitration_id=0x00), 0.0)
    scanner.on_frame(mit_reply(1, arbitration_id=0x00), 0.0)
    assert dict(scanner.sightings[1].mit_arbitration_ids) == {0x00: 2}

    other = Scanner(SPEC)
    other.on_frame(mit_reply(2, arbitration_id=2), 0.0)
    assert dict(other.sightings[2].mit_arbitration_ids) == {2: 1}


def test_scanner_classifies_servo_frames() -> None:
    scanner = Scanner(SPEC)
    scanner.on_frame(servo_status(3), 0.0)
    assert scanner.sightings[3].mode == "servo"
    assert scanner.sightings[3].servo_status == 1


def test_scanner_separates_acks_and_bootloader_jumps() -> None:
    scanner = Scanner(SPEC)
    scanner.on_frame(
        Frame(
            servo_codec.arbitration_id(servo_codec.ServoFunction.SERVO_MODE_ACK, 1),
            servo_codec.SERVO_MODE_ACK_PAYLOAD,
            True,
        ),
        0.0,
    )
    scanner.on_frame(
        Frame(
            servo_codec.arbitration_id(servo_codec.ServoFunction.BOOTLOADER_JUMP, 1),
            bytes(8),
            True,
        ),
        0.0,
    )
    assert scanner.sightings[1].servo_acks == 1
    assert scanner.sightings[1].bootloader == 1
    assert scanner.sightings[1].servo_status == 0


def test_scanner_ignores_mode_control_frames() -> None:
    """Our own enter/exit/zero frames are not evidence of a motor."""
    scanner = Scanner(SPEC)
    for payload in (mit_codec.ENTER_MIT, mit_codec.EXIT_MIT, mit_codec.ZERO_POSITION):
        scanner.on_frame(Frame(1, payload), 0.0)
    assert scanner.sightings == {}
    assert scanner.total == 3


def test_scanner_collects_what_it_cannot_classify() -> None:
    scanner = Scanner(SPEC)
    scanner.on_frame(Frame(0x123, b"\x01\x02\x03"), 0.0)
    scanner.on_frame(Frame(servo_codec.arbitration_id(0x28, 1), bytes(8), True), 0.0)
    assert scanner.sightings == {}
    assert len(scanner.unclassified) == 2


# --- scan end to end ----------------------------------------------------------------


def test_scan_finds_both_modes_on_one_bus(channel: str, capsys: pytest.CaptureFixture[str]) -> None:
    mit_driver = SimMitDriver(SPEC, motor_id=1)
    mit_driver.in_mit_mode = True
    servo_driver = SimServoDriver(SPEC, motor_id=3, status_rate_hz=100.0)
    frames = [mit_driver._reply(), servo_driver.status_frame()]
    with Broadcaster(channel, frames):
        code = main(["scan", "--url", f"virtual:{channel}", "--timeout", "0.4"])
    out = capsys.readouterr().out
    assert code == 0
    assert "id 1  mode MIT" in out
    assert "id 3  mode servo" in out
    assert "arbitration id 0x000" in out


def test_scan_on_a_silent_bus_explains_the_likely_causes(
    channel: str, capsys: pytest.CaptureFixture[str]
) -> None:
    code = main(["scan", "--url", f"virtual:{channel}", "--timeout", "0.2"])
    out = capsys.readouterr().out
    assert code == 1
    assert "No motors found" in out
    assert "CubeMarsTool" in out, "the status-rate-0 trap is named first"
    assert "--poke" in out
    assert "1 Mbit/s" in out


def test_scan_reports_traffic_that_is_not_an_ak_reply(
    channel: str, capsys: pytest.CaptureFixture[str]
) -> None:
    with Broadcaster(channel, [Frame(0x123, b"\xde\xad")]):
        code = main(["scan", "--url", f"virtual:{channel}", "--timeout", "0.3"])
    out = capsys.readouterr().out
    assert code == 1
    assert "none looked like an AK reply" in out


def test_scan_flags_a_bootloader_jump(channel: str, capsys: pytest.CaptureFixture[str]) -> None:
    servo_driver = SimServoDriver(SPEC, motor_id=2, status_rate_hz=0.0)
    with Broadcaster(channel, [servo_driver.bootloader_frame()]):
        main(["scan", "--url", f"virtual:{channel}", "--timeout", "0.3"])
    out = capsys.readouterr().out
    assert "BOOTLOADER JUMPS" in out
    assert "driver is not running" in out


def test_scan_poke_sends_mit_enter_frames(channel: str) -> None:
    """A MIT driver only replies when commanded, so scanning it needs a prod."""
    listener = can.interface.Bus(channel=channel, interface="virtual", receive_own_messages=False)
    try:
        main(["scan", "--url", f"virtual:{channel}", "--timeout", "0.15", "--poke", "2"])
        seen = []
        while (msg := listener.recv(timeout=0.0)) is not None:
            seen.append(msg)
        assert seen, "poking must put frames on the wire"
        assert all(bytes(m.data) == mit_codec.ENTER_MIT for m in seen)
        assert {m.arbitration_id for m in seen} == {1, 2}
    finally:
        listener.shutdown()


# --- dump-spec ----------------------------------------------------------------------


def test_dump_spec_lists_every_model(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["dump-spec"]) == 0
    out = capsys.readouterr().out
    for model in ("AK40-10", "AK80-9", "AK80-64", "AK45-36"):
        assert model in out
    assert "gear 64:1" in out, "the reference library has this as 80:1"


def test_dump_spec_shows_derived_values_and_provenance(
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert main(["dump-spec", "AK40-10"]) == 0
    out = capsys.readouterr().out
    assert "7.479983e-04" in out
    assert "saturates above  25.55 V" in out
    assert "effective torque limit          4.1 Nm" in out
    assert "REFUSED (single encoder)" in out
    assert "datasheet" in out
    assert "Notes:" in out


def test_dump_spec_says_what_it_cannot_derive(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["dump-spec", "AK70-10"]) == 0
    out = capsys.readouterr().out
    assert "refused" in out
    assert "unknown (needs Kv or Ke)" in out


def test_dump_spec_on_an_unknown_motor_lists_the_alternatives(
    capsys: pytest.CaptureFixture[str],
) -> None:
    with pytest.raises(KeyError, match="AK40-10"):
        main(["dump-spec", "AK99-1"])


# --- doctor -------------------------------------------------------------------------


def test_doctor_reports_the_environment(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["doctor"]) == 0
    out = capsys.readouterr().out
    assert "cubemarspycan" in out
    assert "python-can" in out
    assert "slcan" in out


def test_doctor_never_runs_a_privileged_command(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """It prints bring-up lines for you to run; it must not run them itself.

    Reading link state with `ip ... show` is fine - read-only and unprivileged. What must
    never happen is sudo, `ip link set`, or modprobe. TMotorCANControl shells out to
    `sudo ip link set can0 up` from a constructor.
    """
    import os
    import subprocess

    executed: list[list[str]] = []
    real_run = subprocess.run

    def record(cmd: object, *args: object, **kwargs: object) -> object:
        executed.append(list(cmd) if isinstance(cmd, list) else [str(cmd)])
        return real_run(cmd, *args, **kwargs)  # type: ignore[call-overload]

    def explode(*args: object, **kwargs: object) -> None:
        raise AssertionError(f"doctor must not call this: {args!r}")

    # subprocess.run legitimately uses Popen internally, so only the shell-style
    # entry points are banned outright; `run` is recorded and inspected instead.
    monkeypatch.setattr(subprocess, "run", record)
    monkeypatch.setattr(subprocess, "call", explode)
    monkeypatch.setattr(os, "system", explode)

    assert main(["doctor"]) == 0

    for cmd in executed:
        assert cmd[0] != "sudo", f"doctor invoked sudo: {cmd}"
        assert "modprobe" not in cmd[0], f"doctor invoked modprobe: {cmd}"
        if cmd[0] == "ip":
            assert "set" not in cmd, f"doctor mutated a link: {cmd}"
            assert "add" not in cmd, f"doctor created a link: {cmd}"


def test_doctor_gives_platform_appropriate_advice(
    capsys: pytest.CaptureFixture[str],
) -> None:
    import platform

    main(["doctor"])
    out = capsys.readouterr().out
    if platform.system() == "Linux":
        assert "ip link set can0 up type can bitrate 1000000" in out
        assert "vcan" in out
    else:
        assert "socketcan is Linux-only" in out
        assert "slcan:" in out


# --- jog ----------------------------------------------------------------------------


def test_jog_refuses_a_target_beyond_its_own_guard(
    capsys: pytest.CaptureFixture[str],
) -> None:
    code = main(["jog", "--position", "5.0", "--url", "virtual:jog-guard"])
    assert code == 2
    assert "max-position" in capsys.readouterr().err


def test_jog_aborts_without_confirmation(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr("builtins.input", lambda _: "n")
    code = main(["jog", "--position", "0.1", "--url", "virtual:jog-no"])
    assert code == 1
    assert "aborted" in capsys.readouterr().out


def test_jog_prints_what_it_will_do_before_asking(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr("builtins.input", lambda _: "n")
    main(["jog", "--position", "0.2", "--kp", "3", "--url", "virtual:jog-print"])
    out = capsys.readouterr().out
    assert "+0.2 rad" in out
    assert "kp 3" in out
    assert "Clamp the motor" in out


def test_jog_without_a_tty_says_how_to_proceed(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    def no_tty(_: str) -> str:
        raise EOFError

    monkeypatch.setattr("builtins.input", no_tty)
    assert main(["jog", "--url", "virtual:jog-eof"]) == 1
    assert "--yes" in capsys.readouterr().out


# --- parser -------------------------------------------------------------------------


def test_every_subcommand_is_wired_up() -> None:
    parser = build_parser()
    for command in ("scan", "monitor", "jog", "dump-spec", "doctor"):
        args = parser.parse_args([command])
        assert hasattr(args, "func")


def test_a_missing_subcommand_is_an_error() -> None:
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_jog_defaults_are_conservative() -> None:
    args = build_parser().parse_args(["jog"])
    assert args.position == 0.1
    assert args.kp == 5.0
    assert args.max_position == 0.5
    assert not args.yes


@pytest.mark.skipif(__import__("platform").system() == "Linux", reason="socketcan exists on Linux")
def test_a_library_error_becomes_exit_code_1(capsys: pytest.CaptureFixture[str]) -> None:
    """Asking for socketcan off Linux is the natural path here, and a real user mistake."""
    assert main(["scan", "--url", "socketcan:can0"]) == 1
    err = capsys.readouterr().err
    assert "error:" in err
    assert "Linux kernel facility" in err


def test_an_unparseable_url_is_reported(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(ValueError, match="scheme:channel"):
        main(["scan", "--url", "not-a-url"])


# --- monitor and jog against a live simulator ---------------------------------------


class LiveSim:
    """Runs a simulated driver on its own thread, so commands get real replies.

    The stepped simulator cannot serve `monitor` or `jog`, because those own the loop and
    never hand control back to a test to advance time.
    """

    def __init__(self, channel: str, driver: SimMitDriver, dt: float = 0.002) -> None:
        self.bus = can.interface.Bus(
            channel=channel, interface="virtual", receive_own_messages=False
        )
        self.driver = driver
        self.dt = dt
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._stop.is_set():
            msg = self.bus.recv(timeout=self.dt)
            if msg is not None:
                frame = Frame(msg.arbitration_id, bytes(msg.data), msg.is_extended_id)
                reply = self.driver.handle(frame)
                if reply is not None:
                    self.bus.send(
                        can.Message(
                            arbitration_id=reply.arbitration_id,
                            data=reply.data,
                            is_extended_id=reply.is_extended_id,
                        )
                    )
            self.driver.step(self.dt)

    def __enter__(self) -> LiveSim:
        self._thread.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.bus.shutdown()


def test_jog_drives_the_motor_and_leaves_it_safe(
    channel: str, capsys: pytest.CaptureFixture[str]
) -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    with LiveSim(channel, driver):
        code = main(
            [
                "jog",
                "--url",
                f"virtual:{channel}",
                "--id",
                "1",
                "--position",
                "0.2",
                "--kp",
                "20",
                "--kd",
                "0.5",
                "--duration",
                "0.5",
                "--period",
                "0.005",
                "--yes",
            ]
        )
    out = capsys.readouterr().out
    assert code == 0
    assert "safe stop sent, MIT mode exited" in out
    assert driver.commands_seen > 10
    assert driver.plant.position == pytest.approx(0.2, abs=0.05)
    assert not driver.in_mit_mode, "the control block exited MIT mode"


def test_jog_can_zero_first(channel: str) -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    driver.plant.position = 3.0
    with LiveSim(channel, driver):
        main(
            [
                "jog",
                "--url",
                f"virtual:{channel}",
                "--position",
                "0.0",
                "--duration",
                "0.1",
                "--zero",
                "--yes",
            ]
        )
    assert driver.zero_count == 1


def test_jog_leaves_the_motor_safe_when_interrupted(
    channel: str, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """Ctrl-C mid-move must still exit MIT mode."""
    driver = SimMitDriver(SPEC, motor_id=1)
    calls = {"n": 0}
    real_sleep = time.sleep

    def interrupting_sleep(seconds: float) -> None:
        calls["n"] += 1
        if calls["n"] > 5:
            raise KeyboardInterrupt
        real_sleep(seconds)

    with LiveSim(channel, driver):
        monkeypatch.setattr("cubemarspycan.cli.time.sleep", interrupting_sleep)
        code = main(
            [
                "jog",
                "--url",
                f"virtual:{channel}",
                "--position",
                "0.2",
                "--duration",
                "10",
                "--period",
                "0.005",
                "--yes",
            ]
        )
    assert code == 0
    assert "interrupted" in capsys.readouterr().out
    assert not driver.in_mit_mode


def test_monitor_mit_prints_state_and_does_not_drive(
    channel: str, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    driver = SimMitDriver(SPEC, motor_id=1)
    calls = {"n": 0}
    real_sleep = time.sleep

    def interrupting_sleep(seconds: float) -> None:
        calls["n"] += 1
        if calls["n"] > 8:
            raise KeyboardInterrupt
        real_sleep(seconds)

    with LiveSim(channel, driver):
        monkeypatch.setattr("cubemarspycan.cli.time.sleep", interrupting_sleep)
        code = main(["monitor", "--url", f"virtual:{channel}", "--id", "1", "--period", "0.01"])
    out = capsys.readouterr().out
    assert code == 0
    assert "will not be driven" in out
    assert "rad" in out
    # "Zero" torque encodes to half an LSB (+1.22 mNm on an AK40-10), which is 2% of the
    # motor's 60 mNm break-away torque. A real motor cannot move on it; this frictionless
    # model drifts slowly, which is exactly the right distinction to pin.
    assert abs(driver.plant.position) < 0.05, "monitor must not actually drive the motor"
    assert abs(driver.last_command[4]) <= SPEC.mit.torque.lsb  # type: ignore[index]


def test_monitor_servo_is_passive(
    channel: str, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    servo_driver = SimServoDriver(SPEC, motor_id=2, status_rate_hz=100.0)
    calls = {"n": 0}
    real_sleep = time.sleep

    def interrupting_sleep(seconds: float) -> None:
        calls["n"] += 1
        if calls["n"] > 8:
            raise KeyboardInterrupt
        real_sleep(seconds)

    with Broadcaster(channel, [servo_driver.status_frame()], period=0.005):
        monkeypatch.setattr("cubemarspycan.cli.time.sleep", interrupting_sleep)
        code = main(
            [
                "monitor",
                "--url",
                f"virtual:{channel}",
                "--id",
                "2",
                "--mode",
                "servo",
                "--period",
                "0.01",
                "--wait",
                "0.2",
            ]
        )
    out = capsys.readouterr().out
    assert code == 0
    assert "servo mode (passive)" in out
    assert "deg" in out
