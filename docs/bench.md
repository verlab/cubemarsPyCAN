# First bench session

> **Session of 2026-09-16 (AK40-10 KV170, gs_usb adapter, socketcan `can0` at 1 Mbit/s,
> 24 V).** B0-B5 and the wrap test are done, and their results are recorded in the spec
> as `Source.MEASURED`. What each one settled is noted inline below. B7 (servo) is
> outstanding: it needs the driver switched to servo mode in CubeMarsTool, which cannot
> be done over CAN.

Run these in order, motor **clamped to the bench**. Each step has a defined abort. Steps
B2, B5, B6 and B7 answer questions the manual leaves open; record what you find.

Before anything: `cubemars doctor`, then `cubemars dump-spec AK40-10`.

### B0 — adapter only, motor unpowered

```bash
cubemars scan --timeout 5
```

Expect zero frames, a clean exit, no traceback. *Proves the transport and the
"nothing there" path before the motor can confuse matters.*

### B1 — motor powered, first frame ever sent

```bash
cubemars monitor --id 1
```

`monitor` sends zero-gain, zero-torque frames, which produce no motion, so this is safe
first contact. Expect feedback, a plausible temperature, fault 0.

**Record the MIT reply arbitration id** that `cubemars scan` reports. The manual says
"0X00+Drive ID", which is ambiguous; pin it afterwards with
`MitMotor(..., reply_mode=MitReplyMode.ARB_MOTOR_ID)`.

> **Measured:** this driver replies on **arbitration id = motor id** (`0x001` for id 1),
> not `0x00`. Both conventions exist in the wild, which is why the library learns it from
> the first reply rather than assuming.

*Abort if the temperature is implausible or any fault appears.*

### B2 — sign and gearbox side, hand-rotated, still zero gains

With `monitor` running, rotate the output shaft **exactly one turn** by hand.

| Δposition | meaning |
|---|---|
| 2π | output-side (expected) |
| 2π × 10 | rotor-side |

> **Measured:** one hand-turn read **6.3867 rad** against 6.2832 expected for output-side
> and 62.83 for rotor-side. MIT position is **output-side**, confirming the inference from
> the datasheet no-load speed.

Then push past ±12.5 rad by hand and watch what the reading does at the limit:

| behaviour | pass to `MitMotor` |
|---|---|
| rolls over to the far end | `wrap_mode=WrapMode.WRAP` |
| sticks at the limit | `wrap_mode=WrapMode.SATURATE` |

Until you have done this, multi-turn unwrapping **refuses** rather than guessing.

> **Measured:** driven past the limit at 3 rad/s, the reading jumped
> **+12.4985 → −12.4863 rad**. The field **wraps**. The AK40-10 spec now records this, so
> unwrapping is on by default for it; 5.57 output turns tracked continuously afterwards,
> largest sample-to-sample step 17 mrad.

Easier than doing it by hand: drive past the limit under power at a few rad/s.

> **Note:** `zero_here()` stops the driver replying for about a second. It opens a grace
> window for exactly that, so the wait is routine — use `m.settle(1.5)` to keep the loop
> running through it. Note that keeping the link alive is not by itself what fixes this:
> staleness is measured on frames *received*, so transmitting through the gap does not
> reset it. The grace window does.

### B3 — first commanded torque, damping only

```python
m.update(position=0.0, velocity=0.0, kp=0.0, kd=0.3, torque=0.0)
```

The motor should feel like a viscous brake and must not move on its own. Check that the
reported torque *opposes* the direction you push. *First time the motor produces torque.*

> **Measured:** torque opposed motion in **5602 of 5603 samples (100%)** across a 40 s
> hand-push session, peak 0.372 N·m. Position drifted 0.33 rad without seeking a target,
> as it should with `kp=0`.

### B4 — 🚩 first commanded motion

```bash
cubemars jog --id 1 --position 0.1 --kp 5 --kd 0.3 --zero
```

> **Measured:** a ±0.1 rad step settled within **0.5–3.6 mrad** (1.4–9.5 LSB) at `kp=5`,
> peak torque 0.14 N·m. The residuals sit inside the datasheet's 18 arcmin (5.2 mrad) of
> gearbox backlash, so they are mechanical rather than a control problem.

Small gain, small step. Verify it moves the right way, settles, and that the reported
position matches the command within a few LSB (one LSB is 0.38 mrad).

**This is the checkpoint.**

### B5 — scaling agreement

```python
m.update(position=0.0, velocity=5.0, kp=0.0, kd=1.0, torque=0.0)
```

Check the reported velocity settles near 5 rad/s. A consistent factor off means your
firmware's field constants differ from the manual's table, which silently mis-scales
everything.

> **Measured:** commanded ±2 and ±5 rad/s came back at a mean ratio of **0.989**
> (spread 0.984–0.997). The firmware agrees with the manual's ±45.5 rad/s. The 1.1%
> shortfall is steady-state friction error at finite `kd`, not scaling. The simulator covers both plausible readings
(`ScalingVariant.EXACT` / `TRUNCATED`), so the library is correct either way — but you
want to know which you have.

### B6 — back-drive check

At 24 V the motor cannot reach the ±45.5 rad/s velocity field under its own power
(no-load is ~408 rpm = 42.7 rad/s from Kv/Ke). Spin the output **by hand** faster than
that and see whether the reading saturates or wraps. This is the only remaining
velocity-field concern at your supply voltage.

> The datasheet is internally inconsistent here: it states a 435 rpm no-load speed, but
> its own Kv=170 and Ke=5.88 both give 408 rpm at the rated 24 V. 435 rpm implies Kv=181,
> or a 25.6 V supply. The MIT velocity field was evidently sized to the 435 figure, so
> there is no designed-in headroom.

### B7 — servo mode

First confirm in CubeMarsTool that the driver is in servo mode **and that the CAN status
rate is not 0**. A rate of 0 means the driver never uploads anything; the wiring is fine
and nothing arrives.

```bash
cubemars scan            # expect "mode servo"
```

```python
m.update(servo.Duty(0.05))  # 1 s
m.update(servo.Position(90.0))
```

Repeat B2's one-turn test in servo mode:

| Δposition | meaning |
|---|---|
| 360° | output-side |
| 3600° | rotor-side |

That settles `servo.position_side`, the last unconfirmed constant.

## Recording what you found

Measurements belong back in the spec, with their provenance:

```python
from dataclasses import replace
from cubemarspycan import Source, Sourced, get_spec
from cubemarspycan.spec import Side

spec = get_spec("AK40-10").evolve(
    servo=replace(
        get_spec("AK40-10").servo,
        position_side=Sourced(Side.OUTPUT, Source.MEASURED, ref="bench 2026-09-20"),
    )
)
```
