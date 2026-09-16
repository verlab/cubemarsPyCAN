"""The six servo command types, and why they are separate Python types.

Servo mode offers duty cycle, current, brake current, speed, position, and a trapezoidal
position-with-profile. They are mutually exclusive: only one is in flight at a time, and
combining them has no defined meaning.

Here they are value objects rather than attributes, so that exclusivity lives in the type
system. `m.update(servo.Position(90))` says exactly one thing. The library this replaces
models them as attributes - `dev.position = 90; dev.current = 2.0` - which says two
contradictory things and resolves them by whichever mode flag was set last.

    python examples/servo_modes.py --sim
    python examples/servo_modes.py --url socketcan:can0 --id 1
"""

from __future__ import annotations

from _common import base_parser, open_rig, wait_for_control

from cubemarspycan import MotorError, SafetyPolicy, ServoMotor, servo


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument("--hold", type=float, default=2.0, help="seconds per mode")
    args = parser.parse_args()

    with open_rig(args, servo_ids=(args.id,)) as rig:
        spec = rig.spec
        motor = ServoMotor(
            rig.bus,
            args.id,
            spec,
            supply_voltage=args.supply,
            policy=SafetyPolicy(max_temp_c=70.0, current_ceiling_a=2.0),
        )

        stages: list[tuple[str, servo.Setpoint]] = [
            ("duty cycle, open loop      ", servo.Duty(0.04)),
            ("current, i.e. torque       ", servo.Current(0.8)),
            ("speed, electrical RPM      ", servo.Rpm(3000)),
            ("position, degrees          ", servo.Position(45.0)),
            ("position with a profile    ", servo.PositionSpeed(0.0, 4000, 25000)),
            ("brake current, holds still ", servo.CurrentBrake(0.5)),
        ]

        with motor.control(wait_s=wait_for_control(rig)):
            for label, setpoint in stages:
                ticker = rig.ticker(args.period)
                status = None
                while ticker.running(args.hold):
                    status = motor.update(setpoint)
                    ticker.tick()
                print(f"  {label} {setpoint.summary:<34} -> {status}")
            motor.stop()

        print("\n--- what the library refuses, and why ---")
        try:
            motor.command(servo.Current(50.0))
        except MotorError as exc:
            print(f"\n  servo.Current(50.0):\n    {exc}")
        print(
            f"\n  The wire accepts +/-60 A. This motor peaks at "
            f"{spec.limits.peak_current_a.value} A, so a misplaced decimal point would"
        )
        print("  otherwise ask for eight times its rating.")

        from cubemarspycan import CapabilityError, OriginMode

        try:
            motor.set_origin(OriginMode.PERMANENT)
        except CapabilityError as exc:
            print(f"\n  set_origin(PERMANENT):\n    {exc}")
        print("\n  No frame was sent. Origin mode 1 writes flash, so the refusal has to")
        print("  happen before a frame is built, not after the driver rejects it.")


if __name__ == "__main__":
    main()
