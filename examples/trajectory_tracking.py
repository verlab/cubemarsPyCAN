"""Track a moving target, with and without velocity feedforward.

The single most useful thing to know about MIT mode. The firmware computes:

    tau = kp * (p_des - p) + kd * (v_des - v) + tau_ff

Command a *moving* position while leaving `velocity` at 0 and the kd term fights the
motion you are asking for, giving a following error of roughly `kd * v / kp`. Feed the
target's derivative instead and it largely disappears.

Measured on an AK40-10 (kp=20, kd=0.5, +/-0.5 rad at 0.3 Hz, peak 0.94 rad/s):

    velocity = 0.0          median error 23.33 mrad   (predicted kd*v/kp = 23.6)
    velocity = derivative   median error  6.74 mrad   3.5x better

    python examples/trajectory_tracking.py --sim
    python examples/trajectory_tracking.py --url socketcan:can0 --id 1
"""

from __future__ import annotations

import math
import statistics

from _common import base_parser, open_rig, settle, wait_for_control

from cubemarspycan import MitMotor, SafetyPolicy


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument("--amplitude", type=float, default=0.5, help="rad")
    parser.add_argument("--freq", type=float, default=0.3, help="Hz")
    parser.add_argument("--kp", type=float, default=20.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--no-feedforward", action="store_true", help="run only the naive version")
    args = parser.parse_args()

    with open_rig(args) as rig:
        motor = MitMotor(
            rig.bus,
            args.id,
            rig.spec,
            supply_voltage=args.supply,
            policy=SafetyPolicy(max_temp_c=70.0),
        )
        with motor.control(wait_s=wait_for_control(rig)):
            motor.zero_here()
            settle(rig, motor)

            modes = [False] if args.no_feedforward else [False, True]
            results = {}
            for feedforward in modes:
                label = "derivative" if feedforward else "zero"
                print(f"tracking with velocity = {label} ...", flush=True)
                results[label] = _run(motor, args, rig, feedforward)

            motor.hold()

    peak_v = args.amplitude * 2 * math.pi * args.freq
    print(f"\npeak target velocity {peak_v:.3f} rad/s, kp={args.kp:g} kd={args.kd:g}\n")
    for label, errors in results.items():
        ordered = sorted(errors)
        print(
            f"  velocity = {label:<11} median {statistics.median(errors) * 1000:6.2f} mrad"
            f"   p95 {ordered[int(len(ordered) * 0.95)] * 1000:6.2f}"
        )
    if len(results) == 2:
        ratio = statistics.median(results["zero"]) / statistics.median(results["derivative"])
        print(f"\n  feedforward reduced median following error {ratio:.1f}x")
        print(
            f"  predicted error with velocity=0: kd*v/kp = "
            f"{args.kd * peak_v / args.kp * 1000:.1f} mrad at peak velocity"
        )


def _run(motor: MitMotor, args, rig, feedforward: bool) -> list[float]:  # type: ignore[no-untyped-def]
    w = 2 * math.pi * args.freq
    ticker = rig.ticker(args.period)
    errors: list[float] = []
    while ticker.running(args.duration):
        t = ticker.t
        target = args.amplitude * math.sin(w * t)
        # This is the whole point of the example: the derivative of the target, or zero.
        v_des = args.amplitude * w * math.cos(w * t) if feedforward else 0.0
        state = motor.update(position=target, velocity=v_des, kp=args.kp, kd=args.kd, torque=0.0)
        if t > args.duration * 0.3:  # skip the startup transient
            errors.append(abs(state.position_rad - target))
        ticker.tick()
    return errors


if __name__ == "__main__":
    main()
