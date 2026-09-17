"""Speed control in MIT mode, and what the steady-state error tells you.

MIT mode has no dedicated velocity loop. You get one by setting kp=0 and commanding a
velocity with some kd: the firmware then applies `tau = kd * (v_des - v)`, which is a
proportional speed controller whose gain is kd.

That has a consequence worth understanding: at steady state the motor settles where kd
times the speed error equals the friction torque, so the error is `friction / kd`. Raise
kd to reduce it. On a real AK40-10 the measured ratio was 0.989 across +/-2 and +/-5 rad/s
- the 1.1% shortfall is friction, not a scaling error.

    python examples/velocity_control.py --sim
    python examples/velocity_control.py --url socketcan:can0 --id 1 --speeds 2 5 -2 -5
"""

from __future__ import annotations

import statistics

from _common import base_parser, open_rig, wait_for_control

from cubemarspycan import MitMotor, SafetyPolicy


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument(
        "--speeds",
        type=float,
        nargs="+",
        default=[2.0, 5.0, -2.0, -5.0],
        help="rad/s at the output shaft",
    )
    parser.add_argument("--kd", type=float, default=1.0, help="speed-loop gain")
    parser.add_argument("--hold", type=float, default=2.5, help="seconds per speed")
    args = parser.parse_args()

    if any(speed == 0.0 for speed in args.speeds):
        # ratio = mean / target. Zero is not a speed to hold, it is a stop.
        parser.error("--speeds must all be non-zero; use torque_control.py to hold still")

    with open_rig(args) as rig:
        spec = rig.spec
        motor = MitMotor(
            rig.bus,
            args.id,
            spec,
            supply_voltage=args.supply,
            policy=SafetyPolicy(max_temp_c=70.0),
        )
        print(
            f"velocity field +/-{spec.mit.velocity.hi:g} rad/s "
            f"({spec.mit.velocity.lsb * 1000:.1f} mrad/s per LSB)"
        )
        if spec.limits.no_load_speed_radps.known:
            print(f"no-load speed  {spec.limits.no_load_speed_radps.value:.1f} rad/s (datasheet)")
        print(f"\nkd={args.kd:g}; steady-state error is friction/kd, so higher kd tracks closer\n")

        ratios = []
        with motor.control(wait_s=wait_for_control(rig)):
            for target in args.speeds:
                ticker = rig.ticker(args.period)
                samples = []
                while ticker.running(args.hold):
                    state = motor.update(
                        position=0.0, velocity=target, kp=0.0, kd=args.kd, torque=0.0
                    )
                    if ticker.t > args.hold * 0.6:  # steady state only
                        samples.append(state.velocity_radps)
                    ticker.tick()
                if not samples:
                    # `ticker.t > args.hold * 0.6` never fired: the hold is shorter than
                    # a couple of periods. statistics.mean([]) raises StatisticsError.
                    print(f"  commanded {target:+6.2f} rad/s -> no steady-state samples")
                    continue
                mean = statistics.mean(samples)
                ratios.append(mean / target)
                print(
                    f"  commanded {target:+6.2f} rad/s -> measured {mean:+7.3f}   "
                    f"ratio {mean / target:5.3f}"
                )

            # Ramp down rather than dropping the command; a free shaft would coast.
            last = args.speeds[-1]
            for scale in (0.6, 0.3, 0.0):
                down = rig.ticker(args.period)
                while down.running(0.4):
                    motor.update(velocity=last * scale, kp=0.0, kd=args.kd)
                    down.tick()
            motor.hold()

    if not ratios:
        print("\nno speed held long enough to measure; raise --hold")
        return
    mean_ratio = statistics.mean(ratios)
    print(f"\nmean measured/commanded ratio {mean_ratio:.4f}")
    if 0.9 <= mean_ratio <= 1.1:
        print("Within 10% of 1:1, so the firmware's velocity field matches the manual.")
        print("The shortfall is load and friction; raise kd to close it.")
    else:
        print(
            f"Not 1:1. Suspect the firmware's V_max differs from the manual's "
            f"{rig.spec.mit.velocity.hi:g} rad/s."
        )


if __name__ == "__main__":
    main()
