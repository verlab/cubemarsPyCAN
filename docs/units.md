# Unit contract

Every value in this library says what it is in its own name. `position_rad` is radians;
`position_deg` is degrees; `velocity_erpm` is *electrical* RPM. There is no bare
`position`, because the one thing worse than an ambiguous unit is a plausible one.

## Wire units and where they come from

| Quantity | Wire | Library type | Conversion |
|---|---|---|---|
| MIT position | `uint16` over ±`P_max` | rad, **output side** | `uint_to_float` |
| MIT velocity | `uint12` over ±`V_max` | rad/s, output side | `uint_to_float` |
| MIT torque | `uint12` over ±`T_max` | N·m, output side | `uint_to_float` |
| MIT temperature | `uint8` | °C | `raw − 40` |
| Servo position | `int16` × 0.1 | **degrees**, mechanical † | `× π/180` |
| Servo velocity | `int16` × 10 | **ERPM** | see below |
| Servo current | `int16` × 0.01 | A (q-axis) | — |

† Which side of the gearbox servo position refers to is **not documented**. `position_deg`
always works; `output_rad` raises `UnresolvedFrameError` until you settle it on the bench
(step B7) and record it with `spec.evolve(...)`.

## ERPM ↔ rad/s

```
rad/s at output = ERPM · 2π / (60 · pole_pairs · gear_ratio)
```

For the AK40-10 (14 pole pairs, 10:1) one ERPM is **7.480e-4 rad/s**. This number is
per-motor and there is no universal constant: TMotorCANControl hard-codes `5.82e-4` for
every model, which is 22% wrong for the AK40-10 and 5% wrong even for the AK80-9 it was
derived on.

If `pole_pairs` or `gear_ratio` is unknown for your spec, the conversion **raises**
instead of guessing.

## Rotor versus output

`output = rotor / gear_ratio` for angles and speeds; torque is the inverse. Accessors are
explicit about which side they mean:

```python
state.position_rad  # output shaft
state.position_rotor_rad  # before the gearbox
```

MIT position and velocity are output-side. For the AK40-10 this is confirmed rather than
assumed: the velocity field's 45.5 rad/s is 434.5 rpm, which is the datasheet's 435 rpm
no-load speed.

## Torque and current

`Kt` is **rotor-side**, so output torque is `amps · Kt · gear_ratio`. The AK40-10's
datasheet is self-consistent here: 7.3 A × 0.056 × 10 = 4.09 N·m against a published
4.1 N·m peak.

The MIT reply's third field is **torque**, not current — the manual's own `unpack_reply`
names it so. `state.torque_nm` is what the wire carries. `state.estimated_current_a`
inverts an idealised lossless model and warns once while `Kt` is not `Source.MEASURED`.

## MIT Kp and Kd are not in SI units

They are the firmware's own gains, dimensionless to us, spanning 0–500 and 0–5. They are
*not* stiffness in N·m/rad or damping in N·m·s/rad, and nothing in this library converts
them. TMotorCANControl's `set_impedance_gains_real_unit(K, B)` documents them as N·m/rad
and N·m·s/rad and then passes them through unconverted — the name is the bug.

## Feed the derivative, not zero

The firmware computes:

```
tau = kp * (p_des - p) + kd * (v_des - v) + tau_ff
```

If you command a *moving* position while leaving `velocity` at 0, the `kd` term actively
fights the motion needed to follow it, and you get a following error of roughly
`kd * v / kp`. Measured on an AK40-10 tracking a ±0.5 rad sinusoid at 0.3 Hz
(peak 0.94 rad/s) with `kp=20, kd=0.5`:

| command | median following error |
|---|---|
| `velocity=0.0` | 23.33 mrad |
| predicted `kd·v/kp` | 23.6 mrad |
| `velocity=` target derivative | **6.74 mrad** |

Feeding the derivative cut the error 3.5×, and the prediction matched the measurement to
1%. The 6.74 mrad residual is close to the datasheet's 5.24 mrad of gearbox backlash, so
that is the mechanical floor rather than a tuning problem.

```python
w = 2 * math.pi * freq
m.update(
    position=amp * math.sin(w * t),
    velocity=amp * w * math.cos(w * t),  # not 0.0
    kp=20.0,
    kd=0.5,
)
```

## Quantisation

No field can encode an exact zero. Each is a symmetric range over an even-sized field, so
the midpoint sits half an LSB above zero:

| Field | LSB | value of a commanded `0.0` |
|---|---|---|
| position | 0.381 mrad | +0.191 mrad |
| velocity | 22.2 mrad/s | +11.1 mrad/s |
| torque | 2.44 mN·m | **+1.22 mN·m** |

The torque floor matters for a safe stop. On an AK40-10 it is 2% of the 60 mN·m
break-away torque, so the output genuinely cannot move — but it is a floor, not a null.

## Field range is not a limit

A `FieldRange` exists to be handed to the quantiser and carries `bits`. A
`PhysicalLimits` says what the motor can do and has no `bits`, so it can never reach a
codec. They are different types because across the AK line they differ in **both**
directions:

| Motor | MIT torque field | datasheet peak | binding |
|---|---|---|---|
| AK40-10 | ±5.0 N·m | 4.1 N·m | the motor |
| AK80-9 V3.0 | ±18 N·m | 22 N·m | the field |

`ClampMode.EFFECTIVE` (the default) clamps to `min(field, physical)` per field.
