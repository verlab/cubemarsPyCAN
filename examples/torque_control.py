"""Direct torque control, and holding a load against gravity.

With kp=0 and kd=0 the motor applies exactly the feed-forward torque you ask for and does
nothing else. That is the mode for force control, admittance control, and holding a mass.

Two caveats the wire imposes, both real:

* The torque field cannot encode an exact zero. A commanded 0.0 N*m arrives as +1.22 mN*m
  on an AK40-10 - half an LSB. That is 2% of its 60 mN*m break-away torque, so it cannot
  move the output, but it is a floor rather than a null.
* The field spans +/-5.0 N*m while the motor peaks at 4.1. The library clamps to the
  smaller of the two and tells you it did.

    python examples/torque_control.py --sim
    python examples/torque_control.py --url socketcan:can0 --id 1 --torque 0.2
"""

from __future__ import annotations

from _common import base_parser, open_rig, settle, wait_for_control

from cubemarspycan import MitMotor, SafetyPolicy


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument("--torque", type=float, default=0.15, help="N*m, output side")
    parser.add_argument(
        "--damping", type=float, default=0.5, help="kd, to stop a free shaft running away"
    )
    args = parser.parse_args()

    with open_rig(args) as rig:
        spec = rig.spec
        motor = MitMotor(
            rig.bus,
            args.id,
            spec,
            supply_voltage=args.supply,
            policy=SafetyPolicy(max_temp_c=70.0),
        )

        peak = spec.limits.peak_torque_nm
        print(f"torque field  +/-{spec.mit.torque.hi:g} Nm")
        if peak.known:
            print(f"motor peak    +/-{peak.value:g} Nm  (the binding limit)")
        print(f"one LSB       {spec.mit.torque.lsb * 1000:.2f} mNm")
        if peak.known and spec.drivetrain.kt_nm_per_a.known:
            amps = spec.current_for_output_torque(args.torque)
            print(
                f"\n{args.torque:+.3f} Nm is about {amps:.2f} A of q-axis current "
                f"(Kt {spec.drivetrain.kt_nm_per_a.value} x gear "
                f"{spec.drivetrain.gear_ratio.value:g})"
            )

        reports = motor.command(torque=args.torque, kp=0.0, kd=args.damping)
        for report in reports:
            print(f"\nclamped: {report}")

        print(
            f"\napplying {motor.staged_command[4]:+.3f} Nm with kd={args.damping:g} "
            f"for {args.duration:g} s\n"
        )

        with motor.control(wait_s=wait_for_control(rig)):
            motor.zero_here()
            settle(rig, motor)

            ticker = rig.ticker(args.period)
            while ticker.running(args.duration):
                state = motor.update(
                    position=0.0,
                    velocity=0.0,
                    kp=0.0,
                    kd=args.damping,
                    torque=args.torque,
                )
                if ticker.every(0.25):
                    print(f"\r  t={ticker.t:5.2f}s  {state}", end="", flush=True)
                ticker.tick()

            # Wind the torque down before releasing, so nothing lurches.
            for scale in (0.66, 0.33, 0.0):
                down = rig.ticker(args.period)
                while down.running(0.3):
                    motor.update(torque=args.torque * scale, kp=0.0, kd=args.damping)
                    down.tick()
            motor.hold()

    print("\n\nWith kp=0 the motor never seeks a position; it only pushes. On a free shaft")
    print("the kd term is what stops it accelerating away.")


if __name__ == "__main__":
    main()
