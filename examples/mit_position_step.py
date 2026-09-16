"""MIT position step on an AK40-10.

    python examples/mit_position_step.py --url socketcan:can0 --id 1
    python examples/mit_position_step.py --sim          # no hardware needed

Clamp the motor before running against real hardware.
"""

from __future__ import annotations

import argparse
import math
import time

from cubemarspycan import CanTransport, MitMotor, MotorBus, SafetyPolicy, get_spec


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="socketcan:can0")
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--motor", default="AK40-10")
    parser.add_argument("--amplitude", type=float, default=0.25, help="rad")
    parser.add_argument("--kp", type=float, default=20.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--period", type=float, default=0.005)
    parser.add_argument("--sim", action="store_true", help="run against a simulated motor")
    args = parser.parse_args()

    spec = get_spec(args.motor)
    policy = SafetyPolicy(max_temp_c=75.0)

    if args.sim:
        from cubemarspycan.sim import SimMitDriver, sim_bus

        driver = SimMitDriver(spec, motor_id=args.id)
        bus, sim = sim_bus(mit_drivers=[driver])
        run(bus, args, spec, policy, sim=sim)
        bus.close()
        sim.close()
        return

    with CanTransport.open(args.url) as transport, MotorBus(transport) as bus:
        run(bus, args, spec, policy)


def run(bus, args, spec, policy, sim=None) -> None:  # type: ignore[no-untyped-def]
    motor = MitMotor(bus, args.id, spec, policy=policy, supply_voltage=24.0)
    print(motor.describe())
    print()

    # wait_s=0 for the stepped simulator, which only advances when we tell it to.
    with motor.control(wait_s=0.0 if sim else 1.0):
        motor.zero_here()
        time.sleep(0.0 if sim else 1.0)

        start = time.monotonic()
        elapsed = 0.0
        while elapsed < args.duration:
            target = args.amplitude * math.copysign(1.0, math.sin(2 * math.pi * 0.5 * elapsed))
            state = motor.update(position=target, velocity=0.0, kp=args.kp, kd=args.kd, torque=0.0)
            print(
                f"\r t={elapsed:5.2f}s  target {target:+.3f}  {state}",
                end="",
                flush=True,
            )
            if sim is not None:
                sim.advance(args.period)
                elapsed += args.period
            else:
                time.sleep(args.period)
                elapsed = time.monotonic() - start
    print("\nsafe stop sent, MIT mode exited")


if __name__ == "__main__":
    main()
