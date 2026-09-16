"""Servo-mode position and trapezoidal moves.

Servo mode is a different protocol on the same wire: extended frames, degrees instead of
radians, electrical RPM instead of rad/s, and six mutually exclusive command types. The
driver runs its own position loop, so you send a target rather than gains.

Two prerequisites, both set in CubeMarsTool and neither commandable over CAN:

* The driver must be in servo mode. The manual documents a reply that *acknowledges* the
  mode but no frame that causes it, so this library detects rather than guesses.
* The CAN status message rate must be non-zero. At 0 the driver never uploads anything,
  the wiring is fine, and nothing arrives - the single most common bench confusion.

    python examples/servo_position.py --sim
    python examples/servo_position.py --url socketcan:can0 --id 1 --degrees 90
"""

from __future__ import annotations

from _common import base_parser, open_rig, wait_for_control

from cubemarspycan import OriginMode, SafetyPolicy, ServoMotor, servo


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument("--degrees", type=float, default=90.0, help="target, degrees")
    parser.add_argument("--speed-erpm", type=float, default=5000.0)
    parser.add_argument("--accel", type=float, default=30000.0, help="ERPM/s^2")
    parser.add_argument(
        "--simple", action="store_true", help="plain SET_POS instead of a trapezoidal profile"
    )
    args = parser.parse_args()

    with open_rig(args, servo_ids=(args.id,)) as rig:
        motor = ServoMotor(
            rig.bus,
            args.id,
            rig.spec,
            supply_voltage=args.supply,
            policy=SafetyPolicy(max_temp_c=70.0, current_ceiling_a=3.0),
        )
        print(motor.describe().split("\n\n")[0])
        print()

        with motor.control(wait_s=wait_for_control(rig)):
            # Temporary, not permanent: mode 1 writes flash and the manual restricts it
            # to dual-encoder models. On an AK40-10 the library refuses it outright.
            motor.set_origin(OriginMode.TEMPORARY)

            setpoint: servo.Setpoint = (
                servo.Position(args.degrees)
                if args.simple
                else servo.PositionSpeed(
                    args.degrees,
                    speed_erpm=args.speed_erpm,
                    accel_erpm_s2=args.accel,
                )
            )
            print(f"commanding {setpoint.summary}\n")

            ticker = rig.ticker(args.period)
            status = None
            while ticker.running(args.duration):
                status = motor.update(setpoint)
                if int(ticker.t * 4) != int((ticker.t - args.period) * 4):
                    print(f"\r  {status}", end="", flush=True)
                ticker.tick()
            motor.stop()

        print(f"\n\nfinal {status.position_deg:+.2f} deg, commanded {args.degrees:+.2f}")
        if rig.spec.drivetrain.pole_pairs.known and status is not None:
            print(
                f"  {status.velocity_erpm:+.0f} ERPM is "
                f"{status.velocity_radps:+.3f} rad/s at the output"
            )
        print("\n  status.position_deg is always available. status.output_rad refuses")
        print("  until the gearbox side is measured - the manual never says which it is.")


if __name__ == "__main__":
    main()
