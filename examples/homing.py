"""Find a hard stop by feel, then zero against it.

A repeatable zero needs a physical reference. This drives slowly in one direction with a
deliberately low torque ceiling and watches for the signature of contact: torque at the
ceiling while velocity has fallen to nothing. Then it backs off and zeros.

Safety here is the torque cap, not the software. Pick a ceiling the mechanism can absorb
indefinitely - this defaults to 0.3 N*m on a motor rated for 4.1 - and make sure the
travel between here and the stop is clear.

    python examples/homing.py --sim
    python examples/homing.py --url socketcan:can0 --id 1 --direction -1 --ceiling 0.3
"""

from __future__ import annotations

from _common import base_parser, open_rig, settle, wait_for_control

from cubemarspycan import MitMotor, SafetyPolicy


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument("--direction", type=int, choices=(1, -1), default=1)
    parser.add_argument("--speed", type=float, default=1.0, help="approach speed, rad/s")
    parser.add_argument("--ceiling", type=float, default=0.3, help="torque cap, N*m")
    parser.add_argument("--backoff", type=float, default=0.05, help="rad to retreat")
    parser.add_argument(
        "--stall-speed", type=float, default=0.15, help="below this counts as stopped, rad/s"
    )
    parser.add_argument(
        "--confirm", type=float, default=0.3, help="seconds of stall before believing it"
    )
    args = parser.parse_args()

    if args.ceiling > 1.0:
        parser.error(
            "refusing a torque ceiling above 1.0 Nm for homing; raise it "
            "deliberately in the source if your mechanism needs it"
        )

    with open_rig(args) as rig:
        if rig.simulated:
            # Give the fake mechanism a stop to find, so --sim shows the real behaviour.
            plant = rig.sim.mit_drivers[0].plant  # type: ignore[attr-defined]
            stop = 1.2 * args.direction
            if args.direction > 0:
                plant.limit_hi = stop
            else:
                plant.limit_lo = stop
            print(f"simulated hard stop at {stop:+.2f} rad")

        motor = MitMotor(
            rig.bus,
            args.id,
            rig.spec,
            supply_voltage=args.supply,
            policy=SafetyPolicy(max_temp_c=70.0),
        )
        # kd alone gives a speed loop; the torque ceiling comes from clamping the command
        # rather than from the field, so it is ours to choose and ours to keep small.
        kd = args.ceiling / max(args.speed, 1e-6)
        print(
            f"approaching at {args.speed:g} rad/s, direction {args.direction:+d}, "
            f"torque ceiling {args.ceiling:g} Nm (kd={kd:.2f})\n"
        )

        found = False
        with motor.control(wait_s=wait_for_control(rig)):
            ticker = rig.ticker(args.period)
            stalled_for = 0.0
            while ticker.running(args.duration):
                state = motor.update(
                    position=0.0,
                    velocity=args.speed * args.direction,
                    kp=0.0,
                    kd=kd,
                    torque=0.0,
                )
                pushing = abs(state.torque_nm) > args.ceiling * 0.7
                stopped = abs(state.velocity_radps) < args.stall_speed
                stalled_for = stalled_for + args.period if (pushing and stopped) else 0.0
                if stalled_for >= args.confirm:
                    print(
                        f"  hard stop at {state.position_rad:+.4f} rad "
                        f"(torque {state.torque_nm:+.3f} Nm, "
                        f"speed {state.velocity_radps:+.3f} rad/s)"
                    )
                    found = True
                    break
                ticker.tick()

            if not found:
                motor.hold()
                print(
                    f"  no stop found in {args.duration:g} s. Either the travel is "
                    f"longer than that, or the ceiling is too low to overcome friction."
                )
                return

            # Back off before zeroing, so the reference is not taken while loaded.
            back = rig.ticker(args.period)
            target = motor.state.position_rad - args.backoff * args.direction  # type: ignore[union-attr]
            while back.running(1.5):
                motor.update(position=target, velocity=0.0, kp=20.0, kd=0.5, torque=0.0)
                back.tick()
            print(f"  backed off to {motor.state.position_rad:+.4f} rad")  # type: ignore[union-attr]

            # Stop driving before moving the origin. zero_here() changes the coordinate
            # system, and a position command staged in the old frame becomes a command to
            # run back to where the motor just came from.
            motor.hold()
            motor.zero_here()
            settle(rig, motor)
            print(f"  zeroed; now reads {motor.update().position_rad:+.4f} rad")
            motor.hold()

    print("\nThe zero is now referenced to the stop, minus the backoff, so it survives a")
    print("restart as long as nothing has moved. CubeMars does not document whether the")
    print("driver's own zero persists across a power cycle, so re-home rather than assume.")


if __name__ == "__main__":
    main()
