"""A square-wave position step, the simplest closed-loop thing you can do.

Start here. It zeroes, then alternates between two positions, printing the tracking error
in both radians and LSBs so you can see where the wire's resolution ends and the
mechanism's backlash begins.

    python examples/mit_position_step.py --sim
    python examples/mit_position_step.py --url socketcan:can0 --id 1

Clamp the motor before running this against real hardware.
"""

from __future__ import annotations

from _common import base_parser, open_rig, settle, wait_for_control

from cubemarspycan import MitMotor, SafetyPolicy


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument("--amplitude", type=float, default=0.25, help="rad")
    parser.add_argument("--dwell", type=float, default=1.5, help="seconds per side")
    parser.add_argument("--kp", type=float, default=20.0)
    parser.add_argument("--kd", type=float, default=0.5)
    args = parser.parse_args()

    with open_rig(args) as rig:
        motor = MitMotor(
            rig.bus,
            args.id,
            rig.spec,
            supply_voltage=args.supply,
            policy=SafetyPolicy(max_temp_c=70.0),
        )
        print(motor.describe())
        print()

        with motor.control(wait_s=wait_for_control(rig)):
            motor.zero_here()
            settle(rig, motor)

            lsb = rig.spec.mit.position.lsb
            for target in (args.amplitude, 0.0, -args.amplitude, 0.0):
                ticker = rig.ticker(args.period)
                state = None
                while ticker.running(args.dwell):
                    state = motor.update(
                        position=target,
                        velocity=0.0,
                        kp=args.kp,
                        kd=args.kd,
                        torque=0.0,
                    )
                    ticker.tick()
                if state is None:
                    # Not `assert`: python -O strips it, and the next line would then
                    # raise AttributeError instead of saying what went wrong.
                    print(f"  target {target:+.3f} -> no feedback; is --dwell too short?")
                    continue
                error = state.position_rad - target
                print(
                    f"  target {target:+.3f} -> settled {state.position_rad:+.4f} rad  "
                    f"error {error * 1000:+6.2f} mrad ({abs(error) / lsb:5.1f} LSB)  "
                    f"torque {state.torque_nm:+.3f} Nm"
                )
            motor.hold()

    print(f"\n1 LSB = {lsb * 1000:.3f} mrad. On an AK40-10 the datasheet allows 18 arcmin")
    print("(5.24 mrad) of gearbox backlash, so residuals of a few mrad are mechanical.")
    print("\nFor a moving target, see trajectory_tracking.py - a velocity feedforward")
    print("term cuts following error several-fold.")


if __name__ == "__main__":
    main()
