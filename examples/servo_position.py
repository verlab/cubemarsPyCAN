"""Servo-mode position command on an AK40-10.

    python examples/servo_position.py --url socketcan:can0 --id 1
    python examples/servo_position.py --sim

The driver must already be in servo mode, with a non-zero CAN status rate, both set in
CubeMarsTool. This library detects servo mode; the manual documents no frame that causes
it to be entered.
"""

from __future__ import annotations

import argparse

from cubemarspycan import (
    CanTransport,
    MotorBus,
    OriginMode,
    SafetyPolicy,
    ServoMotor,
    get_spec,
    servo,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="socketcan:can0")
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--motor", default="AK40-10")
    parser.add_argument("--degrees", type=float, default=90.0)
    parser.add_argument("--speed-erpm", type=float, default=5000.0)
    parser.add_argument("--accel", type=float, default=30000.0)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--sim", action="store_true")
    args = parser.parse_args()

    spec = get_spec(args.motor)
    policy = SafetyPolicy(max_temp_c=75.0, current_ceiling_a=3.0)

    if args.sim:
        from cubemarspycan.sim import SimServoDriver, sim_bus

        driver = SimServoDriver(spec, motor_id=args.id, status_rate_hz=200.0)
        bus, sim = sim_bus(servo_drivers=[driver])
        run(bus, args, spec, policy, sim=sim)
        bus.close()
        sim.close()
        return

    with CanTransport.open(args.url) as transport, MotorBus(transport) as bus:
        run(bus, args, spec, policy)


def run(bus, args, spec, policy, sim=None) -> None:  # type: ignore[no-untyped-def]
    import time

    motor = ServoMotor(bus, args.id, spec, policy=policy)
    print(motor.describe())
    print()

    with motor.control(wait_s=0.0 if sim else 1.0):
        # Temporary, not permanent: mode 1 writes flash and the AK40-10 is single-encoder.
        motor.set_origin(OriginMode.TEMPORARY)

        setpoint = servo.PositionSpeed(
            args.degrees, speed_erpm=args.speed_erpm, accel_erpm_s2=args.accel
        )
        print(f"commanding {setpoint.summary}\n")
        for _ in range(args.steps):
            status = motor.update(setpoint)
            print(f"\r  {status}", end="", flush=True)
            if sim is not None:
                sim.advance(0.005)
            else:
                time.sleep(0.005)
    print("\ndone")


if __name__ == "__main__":
    main()
