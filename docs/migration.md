# Migrating from TMotorCANControl

The APIs are different so a couple of things are needed for a code migration between the two libraries.

## Setup

| TMotorCANControl | cubemarsPyCAN |
|---|---|
| `TMotorManager_mit_can(motor_type='AK80-9', motor_ID=3)` | `MitMotor(bus, motor_id=3, spec=SPECS["AK80-9"])` |
| implicit `sudo ip link set can0 up` in the constructor | you bring the interface up; `cubemars doctor` prints the lines |
| hard-coded `can0` at 1 Mbit/s | any `can.BusABC`, or `CanTransport.open("socketcan:can0")` |
| process-wide singleton | one `MotorBus` per bus, as many as you like |

## Control loop

```python
# before
with TMotorManager_mit_can(motor_type="AK80-9", motor_ID=3) as dev:
    dev.set_impedance_gains_real_unit(K=10, B=0.5)
    for t in loop:
        dev.update()
        dev.position = 1.0

# after
m = MitMotor(bus, motor_id=3, spec=SPECS["AK80-9"])
with m.control():
    for t in loop:
        state = m.update(position=1.0, kp=10.0, kd=0.5)
```

`update()` both sends and returns state, and the state it returns is the snapshot taken at
the **top** of the call — it predates the frame that call sends. The old library's own
demos were inconsistent about this ordering: its MIT demos called `update()` before
setting position, its servo demos after.

## Why setpoints are not attributes

`dev.position = x` is gone. MIT packs five coupled fields into one frame, so three
assignments produce two intermediate inconsistent commands; a property setter can only
clamp silently or raise from an assignment; and `m.position` reading *feedback* while
`m.position = x` writes a *setpoint* means the value you read is never the value you
wrote. Servo's six commands are mutually exclusive, which a kwargs setter cannot express —
they are separate types here.

## Things that changed because they were wrong

| Old behaviour | Now |
|---|---|
| `servo_can.py` raised `NameError` before sending (6 of 7 commands) | works |
| `set_motor_torque_newton_meters` off by 78× (used `Kt` where it needed the gear ratio) | `output_torque_from_current` / `current_for_output_torque` |
| servo position scaled by `π/21` — 8.6× wrong | degrees × π/180, and `output_rad` refuses until the gearbox side is measured |
| `comm_can_set_pos` scaled by 1e6 | 1e4, per the manual |
| `set_pos_spd` omitted the `/10` on speed and acceleration | applied |
| faults raised inside the receive thread, where they vanish | latched, raised on your thread after a safe stop |
| acceleration sign inverted (`dt = last - now`) | not derived at all; differentiate yourself if you need it |
| `AK60-6` velocity ±50 rad/s | ±45, per the manual |
| `AK80-64` gear ratio 80 | 64 |
| fault code 6 labelled "phase current unbalance" | MOSFET over-temperature; code 7 (stall) added |
| `set_zero_position()` sent origin mode 1 (permanent, writes flash) | `OriginMode.TEMPORARY` by default; permanent refused on single-encoder motors |
| `Kt` constants marked `UNTESTED CONSTANT!` copied to every motor | per-variant datasheet values, or `UNKNOWN` and the conversion refuses |
| a `0.59` current fudge factor from one AK80-9 applied everywhere | gone; `estimated_current_a` is explicit and warns |
| NumPy 2 raised `OverflowError` on any negative servo position | no NumPy dependency at all |