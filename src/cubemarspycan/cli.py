"""Command-line tools.

``scan`` is the one that earns its place: it tells you whether anything is on the bus,
which ids answer, which mode they are in, and which arbitration id a MIT driver replies
on - the question the manual leaves ambiguous. Run it before anything else.

Nothing here runs a privileged command. ``doctor`` *prints* the interface bring-up lines
for you to run yourself.
"""

from __future__ import annotations

import argparse
import contextlib
import platform
import sys
import time
from collections import Counter
from dataclasses import dataclass, field

from . import __version__
from .bus import MotorBus
from .codec import mit as mit_codec
from .codec import servo_can as servo_codec
from .codec.servo_can import ServoFunction
from .errors import CubemarsError
from .frame import Frame
from .motor import MitMotor, ServoMotor
from .policy import SafetyPolicy
from .registry import SPECS, get, models, variants
from .spec import MotorSpec
from .transport.can_bus import (
    DEFAULT_BITRATE,
    CanTransport,
    read_link_status,
)

DEFAULT_URL = "socketcan:can0" if platform.system() == "Linux" else "slcan:/dev/ttyUSB0@1M"


# --- scan ---------------------------------------------------------------------------


@dataclass
class Sighting:
    """What one motor id looked like on the bus."""

    motor_id: int
    mit_replies: int = 0
    servo_status: int = 0
    servo_acks: int = 0
    bootloader: int = 0
    mit_arbitration_ids: Counter[int] = field(default_factory=Counter)
    last_mit: mit_codec.MitFeedback | None = None
    last_servo: servo_codec.ServoFeedback | None = None

    @property
    def mode(self) -> str:
        if self.servo_status or self.servo_acks:
            return "servo"
        if self.mit_replies:
            return "MIT"
        return "?"


class Scanner:
    """A bus endpoint that accepts everything and classifies it."""

    def __init__(self, spec: MotorSpec) -> None:
        self.spec = spec
        self.sightings: dict[int, Sighting] = {}
        self.total = 0
        self.unclassified: list[str] = []

    def accepts(self, frame: Frame) -> bool:
        return True

    def on_frame(self, frame: Frame, rx_monotonic: float) -> None:
        self.total += 1
        if frame.is_extended_id:
            self._servo(frame)
        else:
            self._mit(frame)

    def _for(self, motor_id: int) -> Sighting:
        return self.sightings.setdefault(motor_id, Sighting(motor_id))

    def _mit(self, frame: Frame) -> None:
        if frame.dlc != mit_codec.FEEDBACK_DLC:
            self.unclassified.append(str(frame))
            return
        if mit_codec.is_special(frame.data):
            return  # our own, or another host's, mode-control frame
        sighting = self._for(frame.data[0])
        sighting.mit_replies += 1
        sighting.mit_arbitration_ids[frame.arbitration_id] += 1
        with contextlib.suppress(CubemarsError):
            sighting.last_mit = mit_codec.unpack_feedback(self.spec.mit, frame.data)

    def _servo(self, frame: Frame) -> None:
        function = servo_codec.classify(frame)
        _, motor_id = servo_codec.split_arbitration_id(frame.arbitration_id)
        if function is None:
            self.unclassified.append(str(frame))
            return
        sighting = self._for(motor_id)
        if function is ServoFunction.STATUS:
            sighting.servo_status += 1
            try:
                decoded = servo_codec.decode(frame, self.spec.servo)
            except CubemarsError:
                return
            if isinstance(decoded, servo_codec.ServoFeedback):
                sighting.last_servo = decoded
        elif function is ServoFunction.SERVO_MODE_ACK:
            sighting.servo_acks += 1
        else:
            sighting.bootloader += 1


def cmd_scan(args: argparse.Namespace) -> int:
    spec = get(args.motor)
    transport = CanTransport.open(args.url)
    scanner = Scanner(spec)
    bus = MotorBus(transport)
    bus.register(scanner)
    print(
        f"Listening on {transport.channel_info} for {args.timeout:g} s"
        f"{' (poking MIT ids)' if args.poke else ''} ..."
    )
    try:
        bus.start()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if args.poke:
                for motor_id in range(1, args.poke + 1):
                    # Entering MIT mode elicits a reply and produces no motion. It does
                    # leave the driver in MIT mode, which is why it is opt-in.
                    transport.send(mit_codec.enter_mit_frame(motor_id))
                    time.sleep(0.01)
            time.sleep(0.02)
    finally:
        bus.close()

    print()
    if not scanner.sightings:
        print(f"No motors found. {scanner.total} frame(s) seen in total.")
        if scanner.total == 0:
            print(
                "\nNothing at all arrived. In order of likelihood:\n"
                "  1. A servo-mode driver with its CAN status rate set to 0 in "
                "CubeMarsTool never uploads anything.\n"
                "  2. A MIT-mode driver only replies when commanded - try --poke 4.\n"
                "  3. Power, wiring, termination, or a bitrate other than 1 Mbit/s."
            )
        else:
            print("\nFrames arrived but none looked like an AK reply:")
            for line in scanner.unclassified[:8]:
                print(f"  {line}")
        return 1

    print(f"{len(scanner.sightings)} motor(s), {scanner.total} frame(s):\n")
    for motor_id in sorted(scanner.sightings):
        s = scanner.sightings[motor_id]
        print(f"  id {motor_id}  mode {s.mode}")
        if s.mit_replies:
            ids = ", ".join(f"0x{a:03X} ({n})" for a, n in s.mit_arbitration_ids.items())
            print(f"    MIT replies      : {s.mit_replies}  on arbitration id {ids}")
            print(
                "    -> the manual is ambiguous here; record this id and pass "
                "reply_mode= to pin it."
            )
            if s.last_mit is not None:
                m = s.last_mit
                print(
                    f"    last             : {m.position_rad:+.4f} rad  "
                    f"{m.velocity_radps:+.3f} rad/s  {m.torque_nm:+.3f} Nm  "
                    f"{m.temperature_c} C  fault {m.fault_code}"
                )
        if s.servo_status:
            print(f"    servo status     : {s.servo_status}")
            if s.last_servo is not None:
                v = s.last_servo
                print(
                    f"    last             : {v.position_deg:+.1f} deg  "
                    f"{v.velocity_erpm:+.0f} ERPM  {v.current_a:+.2f} A  "
                    f"{v.temperature_c} C  fault {v.fault_code}"
                )
        if s.servo_acks:
            print(f"    servo mode acks  : {s.servo_acks}")
        if s.bootloader:
            print(f"    BOOTLOADER JUMPS : {s.bootloader}  <- the driver is not running")
        print()
    if scanner.unclassified:
        print(f"{len(scanner.unclassified)} unclassified frame(s), first few:")
        for line in scanner.unclassified[:5]:
            print(f"  {line}")
    return 0


# --- monitor ------------------------------------------------------------------------


def cmd_monitor(args: argparse.Namespace) -> int:
    spec = get(args.motor)
    transport = CanTransport.open(args.url)
    bus = MotorBus(transport)
    try:
        bus.start()
        if args.mode == "servo":
            motor = ServoMotor(bus, args.id, spec)
            print(f"Monitoring {motor.device_info()} in servo mode (passive). Ctrl-C to stop.\n")
            _monitor_servo(motor, args)
        else:
            motor = MitMotor(bus, args.id, spec)  # type: ignore[assignment]
            print(
                f"Monitoring {motor.device_info()} in MIT mode. Ctrl-C to stop.\n"
                "A MIT driver only replies when commanded, so this sends zero-gain, "
                "zero-torque frames. The motor will not be driven.\n"
            )
            _monitor_mit(motor, args)  # type: ignore[arg-type]
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        bus.close()
    return 0


def _monitor_mit(motor: MitMotor, args: argparse.Namespace) -> None:
    policy = SafetyPolicy(max_temp_c=args.max_temp)
    motor.policy = policy
    with motor.control(wait_s=args.wait):
        while True:
            state = motor.update()
            print(f"\r  {state}", end="", flush=True)
            time.sleep(args.period)


def _monitor_servo(motor: ServoMotor, args: argparse.Namespace) -> None:
    motor.policy = SafetyPolicy(max_temp_c=args.max_temp)
    deadline = time.monotonic() + args.wait
    while time.monotonic() < deadline and motor.status is None:
        time.sleep(0.01)
    while True:
        status = motor.status
        print(
            f"\r  {status if status is not None else 'waiting for status frames...'}",
            end="",
            flush=True,
        )
        time.sleep(args.period)


# --- jog ----------------------------------------------------------------------------


def cmd_jog(args: argparse.Namespace) -> int:
    """Drive the motor to a position, gently, with everything printed first."""
    spec = get(args.motor)
    limit = spec.mit.position.hi
    if abs(args.position) > args.max_position:
        print(
            f"refusing: {args.position:g} rad exceeds --max-position "
            f"{args.max_position:g}. Raise it deliberately if you mean it.",
            file=sys.stderr,
        )
        return 2
    print(
        f"About to command {spec.name} id {args.id}:\n"
        f"  position {args.position:+g} rad   (field limit +/-{limit:g})\n"
        f"  kp {args.kp:g}, kd {args.kd:g}\n"
        f"  for {args.duration:g} s at {1 / args.period:.0f} Hz\n"
        f"Clamp the motor before continuing."
    )
    if not args.yes:
        try:
            if input("Proceed? [y/N] ").strip().lower() not in ("y", "yes"):
                print("aborted")
                return 1
        except EOFError:
            print("aborted (no tty; pass --yes to skip this prompt)")
            return 1

    transport = CanTransport.open(args.url)
    bus = MotorBus(transport)
    try:
        bus.start()
        motor = MitMotor(bus, args.id, spec, policy=SafetyPolicy(max_temp_c=args.max_temp))
        with motor.control(wait_s=args.wait):
            if args.zero:
                # Not a bare sleep: that sends nothing, and the driver may go quiet while
                # it zeroes, so the next update() would trip staleness. zero_here() opens
                # a grace window; settle() keeps the loop running through it.
                motor.zero_here()
                motor.settle(1.0)
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                state = motor.update(
                    position=args.position, velocity=0.0, kp=args.kp, kd=args.kd, torque=0.0
                )
                print(f"\r  {state}", end="", flush=True)
                time.sleep(args.period)
        print("\nsafe stop sent, MIT mode exited")
    except KeyboardInterrupt:
        print("\ninterrupted; the control block still sent a safe stop and exited MIT mode")
    finally:
        bus.close()
    return 0


# --- dump-spec ----------------------------------------------------------------------


def cmd_dump_spec(args: argparse.Namespace) -> int:
    if args.motor is None:
        print("Models (MIT field ranges, manual v1.0.18 p.63):")
        for name in models():
            spec = SPECS[name]
            print(
                f"  {name:<10} V +/-{spec.mit.velocity.hi:<6g} rad/s   "
                f"T +/-{spec.mit.torque.hi:<6g} Nm   gear "
                f"{spec.drivetrain.gear_ratio.value:g}:1"
            )
        print("\nVariants with datasheet-verified physical constants:")
        for name in variants():
            print(f"  {name}")
        return 0

    spec = get(args.motor)
    f = spec.mit
    print(f"{spec.name}  (model {spec.model}, manual v{spec.manual_version})\n")
    print("MIT fields (wire scaling):")
    for label, fld in (
        ("position", f.position),
        ("velocity", f.velocity),
        ("torque", f.torque),
        ("kp", f.kp),
        ("kd", f.kd),
    ):
        print(f"  {label:<9} {fld}")
    print("\nDerived:")
    try:
        print(f"  ERPM -> rad/s (output)          {spec.erpm_to_radps_output(1.0):.6e}")
    except CubemarsError as exc:
        print(f"  ERPM -> rad/s (output)          refused: {exc}")
    try:
        print(f"  velocity field saturates above  {spec.velocity_field_saturation_voltage():.2f} V")
    except CubemarsError:
        print("  velocity field saturates above  unknown (needs Kv or Ke)")
    print(f"  effective torque limit          {spec.effective_torque_limit_nm():g} Nm")
    print(f"  effective velocity limit        {spec.effective_velocity_limit_radps():g} rad/s")
    print(
        f"  permanent zero (origin mode 1)  "
        f"{'supported' if spec.capabilities.permanent_zero else 'REFUSED (single encoder)'}"
    )
    print()
    print(spec.provenance_report())
    if spec.notes:
        print("\nNotes:")
        for line in spec.notes.splitlines():
            print(f"  {line}")
    return 0


# --- doctor -------------------------------------------------------------------------


def cmd_doctor(args: argparse.Namespace) -> int:
    import can

    print(f"cubemarspycan {__version__}")
    print(f"  python      {sys.version.split()[0]} on {platform.system()} {platform.machine()}")
    print(f"  python-can  {can.__version__}")
    for module, extra in (("serial", "slcan"), ("gs_usb", "gs-usb")):
        try:
            __import__(module)
            print(f"  {module:<11} installed  ({extra} backend available)")
        except ImportError:
            print(f'  {module:<11} missing    (pip install "cubemarspycan[{extra}]")')

    print()
    if platform.system() == "Linux":
        print("socketcan interfaces:")
        found = False
        from pathlib import Path

        for iface in sorted(Path("/sys/class/net").glob("can*")) + sorted(
            Path("/sys/class/net").glob("vcan*")
        ):
            found = True
            # One probe for all three facts: asking separately forked `ip` up to three
            # times per interface, and multiplied its 2 s timeout by three with it.
            status = read_link_status(iface.name)
            bitrate = status.bitrate
            # Not operstate: a vcan interface reports "unknown" however healthy it is.
            # None means neither sysfs nor `ip` could be consulted at all - say so rather
            # than "down", which is how a healthy interface gets blamed.
            link = "?" if status.up is None else ("up" if status.up else "down")
            note = f"{bitrate} bit/s" if bitrate else "no bitrate (vcan, or never configured)"
            if bitrate not in (None, DEFAULT_BITRATE):
                note += "   <- AK drivers expect 1 Mbit/s"
            print(f"  {iface.name:<8} link {link:<6} {note}")
            can_state = status.state
            if can_state:
                warn = (
                    "   <- nothing is ACKing; check motor power and termination"
                    if can_state in ("ERROR-PASSIVE", "BUS-OFF")
                    else ""
                )
                print(f"  {'':<8} can state {can_state}{warn}")
        if not found:
            print("  none found")
        print(
            "\nBring one up (run these yourself; this tool never uses sudo):\n"
            "  sudo ip link set can0 up type can bitrate 1000000\n"
            "  sudo ip link set can0 txqueuelen 1000\n"
            "\nFor a virtual interface to test against with no hardware:\n"
            "  sudo modprobe vcan\n"
            "  sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0"
        )
    else:
        print(
            f"socketcan is Linux-only and is not available on {platform.system()}.\n"
            "Use a USB-CAN adapter here:\n"
            "  cubemars scan --url slcan:/dev/tty.usbmodem1101@1M\n"
            "  cubemars scan --url gs_usb:0@1M\n"
            "Deploy targets on Linux can use 'socketcan:can0'."
        )
    print(f"\nDefault url on this machine: {DEFAULT_URL}")
    return 0


# --- argument parsing ---------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="cubemars",
        description="CAN tools for CubeMars AK-series actuators.",
    )
    parser.add_argument("--version", action="version", version=f"cubemarspycan {__version__}")
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_link(sub: argparse.ArgumentParser) -> None:
        sub.add_argument("--url", default=DEFAULT_URL, help=f"CAN url (default {DEFAULT_URL})")
        sub.add_argument("--motor", default="AK40-10", help="motor spec name")

    scan = subparsers.add_parser("scan", help="find motors on the bus and say what they are")
    add_link(scan)
    scan.add_argument("--timeout", type=float, default=3.0, help="seconds to listen")
    scan.add_argument(
        "--poke",
        type=int,
        default=0,
        metavar="N",
        help="also send MIT enter frames to ids 1..N to elicit replies "
        "(leaves those drivers in MIT mode)",
    )
    scan.set_defaults(func=cmd_scan)

    monitor = subparsers.add_parser("monitor", help="print one motor's state continuously")
    add_link(monitor)
    monitor.add_argument("--id", type=int, default=1)
    monitor.add_argument("--mode", choices=("mit", "servo"), default="mit")
    monitor.add_argument("--period", type=float, default=0.05)
    monitor.add_argument("--wait", type=float, default=1.0)
    monitor.add_argument("--max-temp", type=float, default=75.0)
    monitor.set_defaults(func=cmd_monitor)

    jog = subparsers.add_parser("jog", help="move one motor to a position, gently")
    add_link(jog)
    jog.add_argument("--id", type=int, default=1)
    jog.add_argument("--position", type=float, default=0.1, help="target in rad")
    jog.add_argument("--kp", type=float, default=5.0)
    jog.add_argument("--kd", type=float, default=0.3)
    jog.add_argument("--duration", type=float, default=3.0)
    jog.add_argument("--period", type=float, default=0.01)
    jog.add_argument("--wait", type=float, default=1.0)
    jog.add_argument("--max-temp", type=float, default=75.0)
    jog.add_argument(
        "--max-position",
        type=float,
        default=0.5,
        help="refuse targets beyond this magnitude (default 0.5 rad)",
    )
    jog.add_argument("--zero", action="store_true", help="zero the position first")
    jog.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    jog.set_defaults(func=cmd_jog)

    dump = subparsers.add_parser("dump-spec", help="print a motor spec and its provenance")
    dump.add_argument("motor", nargs="?", help="omit to list every known motor")
    dump.set_defaults(func=cmd_dump_spec)

    doctor = subparsers.add_parser("doctor", help="check the environment and print CAN setup")
    doctor.set_defaults(func=cmd_doctor)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result: int = args.func(args)
        return result
    except CubemarsError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
