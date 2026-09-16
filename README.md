# cubemarsPyCAN

CAN control for CubeMars AK-series actuators — MIT and servo modes — written against the
*AK Series Module Driver Manual* **v1.0.18** (AK 2.0 drivers).

Developed for the **AK40-10 KV170**. Production target is Linux + socketcan; macOS with a
USB-CAN adapter works for development.

> **Status: MIT mode hardware-verified; servo mode simulator-only.**
> Bench session 2026-09-16 on an AK40-10 KV170 over socketcan at 1 Mbit/s: position,
> velocity and torque control, multi-turn unwrapping, fault handling and a 3-minute
> closed-loop soak (0 faults, 0 dropped frames, 0 decode errors) all pass on real
> hardware. Servo-over-CAN is verified against the simulator and the manual but has not
> yet been run on a motor — it needs the driver switched to servo mode in CubeMarsTool.
> See [docs/bench.md](docs/bench.md) for what each step measured.

## Install

```bash
pip install cubemarspycan                # socketcan + virtual
pip install "cubemarspycan[slcan]"       # + USB-CAN over serial
```

Python 3.10+. The only runtime dependency is `python-can`.

## Use

```python
from cubemarspycan import CanTransport, MitMotor, MotorBus, SPECS

with CanTransport.open("socketcan:can0") as tp, MotorBus(tp) as bus:
    m = MitMotor(bus, motor_id=1, spec=SPECS["AK40-10"], supply_voltage=24.0)
    with m.control():
        m.zero_here()
        for t in loop:
            s = m.update(position=0.5, velocity=0.0, kp=20.0, kd=0.5, torque=0.0)
            print(s.position_rad, s.torque_nm, s.temperature_c)
```

Servo mode uses setpoint value objects, because its six commands are mutually exclusive:

```python
from cubemarspycan import ServoMotor, servo

m = ServoMotor(bus, motor_id=1, spec=SPECS["AK40-10"])
with m.control():
    s = m.update(servo.PositionSpeed(90.0, speed_erpm=5000, accel_erpm_s2=30000))
```

Runnable examples are in [`examples/`](examples); both take `--sim` and need no hardware.

## Command line

```bash
cubemars doctor                     # environment, and the CAN bring-up lines to run
cubemars scan                       # what is on the bus, and which mode it is in
cubemars monitor --id 1             # live state, zero-gain keepalives
cubemars jog --id 1 --position 0.1  # a gentle move, confirms first
cubemars dump-spec AK40-10          # fields, derived values, provenance
```

`scan` is the one to reach for when something is wrong. A MIT driver only replies when
commanded, so `--poke N` prods ids 1..N without moving anything:

```
$ cubemars scan --poke 8
2 motor(s), 85 frame(s):

  id 1  mode MIT
    MIT replies      : 43  on arbitration id 0x001 (43)
    -> the manual is ambiguous here; record this id and pass reply_mode= to pin it.
    last             : -1.6939 rad  +0.011 rad/s  +0.001 Nm  32 C  fault 0

  id 3  mode servo
    servo status     : 42
```

On a silent bus it ranks the causes, naming the one that wastes an afternoon first: a
servo driver whose CAN status rate is 0 in CubeMarsTool never uploads anything, so the
wiring is fine and nothing arrives.

Full recipes — first bring-up, finding an unknown id, diagnosing a dead bus, checking a
motor before a run — are in **[docs/cli.md](docs/cli.md)**.

## Examples

Every example takes `--sim` and runs against a protocol-accurate simulator, so you can try
them all with no hardware. CI runs each one on every commit, so they cannot go stale.

```bash
python examples/mit_position_step.py --sim
python examples/trajectory_tracking.py --url socketcan:can0 --id 1
```

| Example | What it shows |
|---|---|
| [`mit_position_step.py`](examples/mit_position_step.py) | Start here. Square-wave position steps, error in rad and LSBs. |
| [`trajectory_tracking.py`](examples/trajectory_tracking.py) | **Velocity feedforward.** Measured 3.5× less following error on hardware. |
| [`impedance_control.py`](examples/impedance_control.py) | Variable stiffness, from free to stiff. What MIT mode is actually for. |
| [`torque_control.py`](examples/torque_control.py) | Direct torque, and the ±1.22 mN·m quantisation floor. |
| [`velocity_control.py`](examples/velocity_control.py) | Speed control via `kd`, and why steady-state error is `friction/kd`. |
| [`two_motors.py`](examples/two_motors.py) | Leader–follower on one bus; proves the endpoints never cross-talk. |
| [`homing.py`](examples/homing.py) | Find a hard stop by torque, back off, zero against it. |
| [`fault_handling.py`](examples/fault_handling.py) | Faults, staleness, over-temperature, and recovery. |
| [`log_to_csv.py`](examples/log_to_csv.py) | Record a run, using the receive timestamp rather than `time.time()`. |
| [`servo_position.py`](examples/servo_position.py) | Servo mode: degrees, ERPM, trapezoidal moves. |
| [`servo_modes.py`](examples/servo_modes.py) | All six servo commands, and the two the library refuses. |

## Why another library

The existing Python libraries for these motors are wrong in ways that matter. An audit of
the most widely used one is in [PLAN.md](PLAN.md); every finding below was reproduced
against a virtual bus, not read off:

- Its **servo-over-CAN module does not execute** — 6 of 7 commands raise `NameError` or
  `TypeError` before a frame reaches the bus.
- **Motor-side and output-side torque are swapped**, so a 1 N·m request is off by 78×.
- **Servo position is scaled by `π/21` instead of `π/180`** — an 8.6× error — and
  `SET_POS` by 1e6 instead of 1e4, another 100×.
- **Driver faults are raised inside the receive thread**, where python-can swallows them.
  Your control loop never learns, and the motor keeps being commanded while faulted.
- On NumPy ≥ 2 its decoder raises `OverflowError` on **any negative position**.

## Design

- **The codec is pure.** `bytes` in, `bytes` out, no I/O — enforced by an AST-walk test
  that fails if `codec/` ever imports `can`, `time`, `threading` or `numpy`. Every
  protocol claim is a unit test that runs on a laptop.
- **The transport is injected.** Bring any `can.BusABC`. No `sudo`, no hard-coded `can0`,
  no singletons. `cubemars doctor` prints bring-up commands rather than running them.
- **A wire field is not a limit.** `FieldRange` carries `bits` and feeds the quantiser;
  `PhysicalLimits` has no `bits` and never reaches a codec. Across the AK line they differ
  in *both* directions — the AK40-10's torque field over-promises against a 4.1 N·m peak,
  the AK80-9's under-promises against 22 N·m.
- **Unknown constants refuse.** Every non-wire number carries its provenance
  (`MEASURED` / `DATASHEET` / `MANUAL` / `NAMEPLATE` / …), and a conversion that needs an
  unknown one raises instead of guessing. Guessing is what produced the 8.6× and 78×
  errors above.
- **Faults never raise off-thread.** They latch as data and surface on your thread, after
  a safe stop is already on the wire.
- **State is frozen.** The receive thread builds a new immutable snapshot per frame and
  never mutates a published one, so tearing is impossible rather than merely avoided.

## Things worth knowing before you plug in

- **A saturated MIT command collides with a mode-control frame.** With position, velocity,
  Kp and Kd at maximum, a torque near 4.9976 N·m packs to `FF FF FF FF FF FF FF FE` —
  which the driver reads as *set position to zero*. `pack_command` steps one torque LSB
  (2.4 mN·m) away. Other implementations do not.
- **No field encodes an exact zero.** A commanded 0.0 N·m arrives as +1.22 mN·m — half an
  LSB, and 2% of the AK40-10's 60 mN·m break-away torque. A floor, not a null.
- **A control loop with no sleep starves the receive thread.** Frames arrive on a
  python-can notifier thread; a busy-wait never releases the GIL and looks exactly like a
  dead motor.
- **Feed the target's derivative as `velocity`, not 0.** Otherwise `kd` fights the motion
  you are asking for. Measured 3.5× less following error on hardware.
- **The AK40-10 datasheet contradicts itself.** It states 435 rpm no-load, but its own
  Kv=170 and Ke=5.88 both give 408 rpm at the rated 24 V. The MIT velocity field was sized
  to the 435 figure, so there is no designed-in headroom.
- **Permanent zero is dual-encoder only.** Your AK40-10 has one encoder, so origin mode 1
  is refused and no frame is sent. It writes flash.

## Documentation

| | |
|---|---|
| [docs/cli.md](docs/cli.md) | command-line recipes for bring-up and diagnosis |
| [docs/units.md](docs/units.md) | the unit contract, and why Kp/Kd are not in SI units |
| [docs/can-setup.md](docs/can-setup.md) | socketcan on Ubuntu, slcan on macOS, vcan for CI |
| [docs/bench.md](docs/bench.md) | first bench session, step by step |
| [docs/troubleshooting.md](docs/troubleshooting.md) | what each error means and what causes it |
| [docs/ak-2-0.md](docs/ak-2-0.md) | the protocol, and what changed in manual v1.0.18 |
| [docs/migration.md](docs/migration.md) | moving from TMotorCANControl |

## Motors

MIT field ranges for all ten models in manual v1.0.18 are included. Four variants carry
datasheet-verified drivetrain constants:

| Variant | KV | Kt | poles | gear | peak τ | encoders |
|---|---|---|---|---|---|---|
| **AK40-10 KV170** | 170 | 0.056 | 14 | 10:1 | 4.1 N·m | 1 |
| AK10-9 V2.0 KV60 | 60 | 0.198 | 21 | 9:1 | 48 N·m | 2 |
| AK80-9 V3.0 KV100 | 100 | 0.095 | 21 | 9:1 | 22 N·m | 1 |
| AK80-8 KV60 | 60 | 0.199 | 21 | 8:1 | 25 N·m | 2 |

Other models work at the wire level; their physical constants are `UNKNOWN` until a
datasheet is transcribed, and conversions needing them refuse. `cubemars dump-spec` shows
the state of any of them.

## Development

```bash
pip install -e ".[dev]"
pytest -q                    # ~1200 tests, no hardware needed
ruff check . && mypy
```

CI runs macOS and Ubuntu on Python 3.10–3.13, plus a dedicated `vcan` job so the socketcan
paths are exercised for real.

## Licence

[MIT](LICENSE).

The implementation is clean-room from the *AK Series Module Driver Manual* v1.0.18 and the
CubeMars product datasheets. No code is taken from TMotorCANControl, which is GPLv3 —
`tools/check_cleanroom.py` compares token shingles against it and runs in CI, so the
permissive licence stays defensible rather than merely asserted.

The manual and datasheets remain CubeMars' copyright. This repository cites them by page
and does not redistribute them.
