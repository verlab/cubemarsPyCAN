"""Faults, staleness, and getting going again.

Three things go wrong in the field, and each surfaces differently:

* **A driver fault.** The motor reports a non-zero code. This library latches it and
  raises `MotorFault` on *your* thread, after a safe-stop frame is already on the wire.
  The library it replaces raises inside the python-can receive thread, where the exception
  is swallowed, your loop never learns, and the motor keeps being commanded while faulted.
* **Feedback stops.** Power loss, a pulled connector, a saturated link. `StaleFeedbackError`
  fires after `stale_fatal_s`, again after a safe stop.
* **Over-temperature.** Checked against your policy, not the driver's own limit.

Run with --sim to see all three without breaking anything.

    python examples/fault_handling.py --sim
    python examples/fault_handling.py --url socketcan:can0 --id 1
"""

from __future__ import annotations

from _common import base_parser, open_rig, wait_for_control

from cubemarspycan import (
    FaultAction,
    MitMotor,
    MotorError,
    MotorFault,
    SafetyPolicy,
    StaleFeedbackError,
)


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument(
        "--inject",
        action="store_true",
        help="with --sim, make the fake driver report an over-current fault",
    )
    args = parser.parse_args()

    policy = SafetyPolicy(
        max_temp_c=70.0,
        on_fault=FaultAction.RAISE,  # the default; WARN or IGNORE also exist
        stale_warn_s=0.1,
        stale_fatal_s=0.5,
    )

    with open_rig(args) as rig:
        motor = MitMotor(rig.bus, args.id, rig.spec, supply_voltage=args.supply, policy=policy)
        print(
            f"policy: stop above {policy.max_temp_c:g} C, "
            f"on fault {policy.on_fault.value}, "
            f"stale after {policy.stale_fatal_s:g} s\n"
        )

        if args.sim and args.inject:
            rig.require_sim().mit_drivers[0].fault_code = 2
            print("simulated driver will report fault 2 (over-current)\n")

        try:
            with motor.control(wait_s=wait_for_control(rig)):
                ticker = rig.ticker(args.period)
                while ticker.running(args.duration):
                    state = motor.update(position=0.0, velocity=0.0, kp=5.0, kd=0.3, torque=0.0)
                    if state.seq and ticker.every(0.5):
                        print(f"\r  t={ticker.t:4.1f}s  {state}", end="", flush=True)
                    ticker.tick()
            print("\n\ncompleted without incident")

        except MotorFault as exc:
            print(f"\n\nFAULT: {exc}")
            latched = motor.fault
            if latched is not None:
                print(f"  code {latched.code} ({latched.text}), source {latched.source}")
            print("\n  A safe stop went out before this was raised, and control() exited")
            print("  MIT mode on the way out. Address the cause, then clear_fault().")
            motor.clear_fault()
            print(f"  cleared; motor.faulted is now {motor.faulted}")

        except StaleFeedbackError as exc:
            print(f"\n\nSTALE: {exc}")
            print("\n  Usual causes: the motor lost power, the connector came out, or the")
            print("  loop is slower than stale_fatal_s. A safe stop has been sent.")

        except MotorError as exc:
            print(f"\n\nMOTOR ERROR: {exc}")

    print("\nWhichever path ran, control() sent a safe stop and left MIT mode.")
    print("That happens even when the body raises - it is a finally, not a happy path.")


if __name__ == "__main__":
    main()
