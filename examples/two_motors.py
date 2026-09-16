"""Two motors on one bus: leader-follower.

One motor is left free (kp=kd=0) so you can move it by hand; the other mirrors it. This is
the classic bilateral-teleoperation starting point, and it exercises the parts of the bus
layer that a single motor never touches.

Two properties worth noticing in the output:

* The motors never cross-talk. Each endpoint filters on the extended/standard flag, the
  DLC, the arbitration id it learned, *and* the id byte - four conditions. The library
  this replaces checks only the payload's first byte, so any frame whose first byte
  collides is decoded as that motor's state.
* An exception in one motor's receive handler is contained and counted; the other keeps
  receiving. Check `bus.stats.endpoint_errors` after a run.

    python examples/two_motors.py --sim
    python examples/two_motors.py --url socketcan:can0 --leader 1 --follower 2
"""

from __future__ import annotations

from _common import base_parser, open_rig, settle, wait_for_control

from cubemarspycan import MitMotor, SafetyPolicy


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument("--leader", type=int, default=1, help="the one you move by hand")
    parser.add_argument("--follower", type=int, default=2, help="the one that mirrors it")
    parser.add_argument("--kp", type=float, default=20.0, help="follower stiffness")
    parser.add_argument("--kd", type=float, default=0.5, help="follower damping")
    parser.add_argument("--ratio", type=float, default=1.0, help="follower/leader scaling")
    args = parser.parse_args()

    with open_rig(args, motor_ids=(args.leader, args.follower)) as rig:
        policy = SafetyPolicy(max_temp_c=70.0)
        leader = MitMotor(rig.bus, args.leader, rig.spec, supply_voltage=args.supply, policy=policy)
        follower = MitMotor(
            rig.bus, args.follower, rig.spec, supply_voltage=args.supply, policy=policy
        )

        print(f"leader   id {args.leader}  free, move it by hand")
        print(f"follower id {args.follower}  kp={args.kp:g} kd={args.kd:g}, ratio {args.ratio:g}\n")

        with (
            leader.control(wait_s=wait_for_control(rig)),
            follower.control(wait_s=wait_for_control(rig)),
        ):
            for motor in (leader, follower):
                motor.zero_here()
            settle(rig, leader)

            ticker = rig.ticker(args.period)
            worst = 0.0
            while ticker.running(args.duration):
                # Leader free: zero gains, so it reports where you put it and nothing else.
                lead = leader.update(position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0)
                # Follower chases it. Passing the leader's measured velocity as the
                # follower's target velocity is the feedforward term - see
                # trajectory_tracking.py for why it matters.
                target = lead.position_rad * args.ratio
                follow = follower.update(
                    position=target,
                    velocity=lead.velocity_radps * args.ratio,
                    kp=args.kp,
                    kd=args.kd,
                    torque=0.0,
                )
                worst = max(worst, abs(follow.position_rad - target))
                if int(ticker.t * 2) != int((ticker.t - args.period) * 2):
                    print(
                        f"\r  leader {lead.position_rad:+7.4f}  "
                        f"follower {follow.position_rad:+7.4f}  "
                        f"lag {(follow.position_rad - target) * 1000:+6.1f} mrad",
                        end="",
                        flush=True,
                    )
                ticker.tick()

            for motor in (leader, follower):
                motor.hold()

        print(f"\n\nworst follower lag      {worst * 1000:.1f} mrad")
        print(f"leader reply arb id     0x{leader.reply_arbitration_id:03X}")
        print(f"follower reply arb id   0x{follower.reply_arbitration_id:03X}")
        print(f"frames matched          {rig.bus.stats.rx_matched}")
        print(f"frames matching nobody  {rig.bus.stats.rx_unmatched}")
        print(f"endpoint errors         {rig.bus.stats.endpoint_errors}")
        print(
            f"decode errors           leader {leader.decode_errors}, "
            f"follower {follower.decode_errors}"
        )


if __name__ == "__main__":
    main()
