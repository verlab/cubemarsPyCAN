"""Variable stiffness: make the motor feel soft, then stiff, then like a spring-damper.

This is what MIT mode is for. Rather than forcing a position, you set how *hard* the motor
argues with you when something pushes it off target. Push the shaft by hand as it runs and
feel the difference.

Kp and Kd are the firmware's own gains, not N*m/rad and N*m*s/rad. Nothing in this library
converts them, because the manual gives no conversion and inventing one would be a guess
dressed up as a unit. See docs/units.md.

    python examples/impedance_control.py --sim
    python examples/impedance_control.py --url socketcan:can0 --id 1
"""

from __future__ import annotations

from _common import base_parser, open_rig, settle, wait_for_control

from cubemarspycan import MitMotor, SafetyPolicy

# (label, kp, kd) - held at the same target throughout, so the only thing
# changing is how strongly the motor resists being moved off it.
STAGES = [
    ("free          ", 0.0, 0.0),
    ("damping only  ", 0.0, 1.0),
    ("soft spring   ", 5.0, 0.3),
    ("medium spring ", 20.0, 0.5),
    ("stiff spring  ", 60.0, 1.0),
]


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument("--target", type=float, default=0.0, help="hold position, rad")
    parser.add_argument(
        "--stage-seconds", type=float, default=4.0, help="seconds per stiffness setting"
    )
    args = parser.parse_args()

    with open_rig(args) as rig:
        motor = MitMotor(
            rig.bus,
            args.id,
            rig.spec,
            supply_voltage=args.supply,
            policy=SafetyPolicy(max_temp_c=70.0),
        )
        print(motor.describe().splitlines()[0])
        print(f"\nholding {args.target:+.2f} rad. Push the shaft by hand and feel each stage.\n")

        with motor.control(wait_s=wait_for_control(rig)):
            motor.zero_here()
            settle(rig, motor)

            for label, kp, kd in STAGES:
                seconds = args.stage_seconds
                ticker = rig.ticker(args.period)
                excursion = 0.0
                peak_torque = 0.0
                while ticker.running(seconds):
                    state = motor.update(
                        position=args.target, velocity=0.0, kp=kp, kd=kd, torque=0.0
                    )
                    excursion = max(excursion, abs(state.position_rad - args.target))
                    peak_torque = max(peak_torque, abs(state.torque_nm))
                    ticker.tick()
                print(
                    f"  {label} kp={kp:5.1f} kd={kd:4.1f}  "
                    f"max excursion {excursion * 1000:7.1f} mrad   "
                    f"peak torque {peak_torque:5.3f} Nm"
                )

            motor.hold()

    print("\nA larger excursion for the same push means a softer setting. With kp=0 the")
    print("motor does not seek the target at all, so excursion is just wherever you left it.")


if __name__ == "__main__":
    main()
