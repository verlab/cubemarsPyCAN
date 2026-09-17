# Command line

Five commands. `scan` is the one you will actually use.

```bash
cubemars doctor                     # environment, interfaces, bring-up lines
cubemars scan                       # what is on the bus, and what mode it is in
cubemars monitor --id 1             # live state
cubemars jog --id 1 --position 0.1  # a gentle move
cubemars dump-spec AK40-10          # fields, derived values, provenance
```

Every command takes `--url`, defaulting to `socketcan:can0` on Linux and
`slcan:/dev/ttyUSB0@1M` elsewhere. `--motor` selects the spec, defaulting to `AK40-10`.

---

## Recipes

### Bringing up a bus for the first time

```bash
cubemars doctor
```

Shows your Python, python-can, which optional backends are installed, every CAN interface
with its link state, bitrate and **CAN controller state**, and the exact `ip link` lines to
run. It never runs them — this library does not shell out to `sudo`.

A healthy interface looks like:

```
  can0     link up     1000000 bit/s
           can state ERROR-ACTIVE
```

`ERROR-PASSIVE` or `BUS-OFF` means nothing is acknowledging frames — no other powered
node, or missing termination:

```
  can0     link up     1000000 bit/s
           can state ERROR-PASSIVE   <- nothing is ACKing; check motor power and termination
```

`link ?` is a third state, and it means the tool could not tell — neither
`/sys/class/net/<iface>/flags` nor `ip` could be read. It is deliberately not reported as
`down`: on a slim container or a BusyBox rootfs with no iproute2, calling a healthy
interface down is how you get sent to fix something that was never broken.

```
  can0     link ?      1000000 bit/s
```

A bitrate read from sysfs beside a `?` link, as above, is exactly that case.

Clear it once the cause is fixed:

```bash
sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000
```

### Finding a motor whose id you don't know

```bash
cubemars scan --poke 8
```

A servo-mode driver uploads unprompted, so a plain `scan` finds it. A **MIT-mode driver
only replies when commanded**, so it looks like a dead bus until you prod it. `--poke N`
sends enter-MIT frames to ids 1..N, which produce no motion.

```
2 motor(s), 85 frame(s):

  id 1  mode MIT
    MIT replies      : 43  on arbitration id 0x001 (43)
    -> the manual is ambiguous here; record this id and pass reply_mode= to pin it.
    last             : -1.6939 rad  +0.011 rad/s  +0.001 Nm  32 C  fault 0

  id 3  mode servo
    servo status     : 42
```

`--poke` leaves those drivers in MIT mode, which is why it is opt-in.

### Nothing at all on the bus

```bash
cubemars scan --timeout 5
```

```
No motors found. 0 frame(s) seen in total.

Nothing at all arrived. In order of likelihood:
  1. A servo-mode driver with its CAN status rate set to 0 in CubeMarsTool
     never uploads anything.
  2. A MIT-mode driver only replies when commanded - try --poke 4.
  3. Power, wiring, termination, or a bitrate other than 1 Mbit/s.
```

Cause 1 is the one that wastes an afternoon: the wiring is fine and the driver is simply
configured never to speak. Check it before touching a cable.

If frames *are* arriving but none look like an AK reply, `scan` prints them so you can see
what else is on the bus.

### Checking a motor before a run

```bash
cubemars monitor --id 1
```

Prints state continuously. In MIT mode it sends zero-gain, zero-torque frames to elicit
replies — **the motor is not driven**. In servo mode it is fully passive:

```bash
cubemars monitor --id 1 --mode servo
```

Watch for a plausible temperature, `no fault`, and a position that holds still.

### A first, cautious move

```bash
cubemars jog --id 1 --position 0.1 --kp 5 --kd 0.3 --zero
```

Prints what it is about to do and asks before moving. Defaults are deliberately timid:
±0.5 rad maximum, `kp=5`. Ctrl-C still sends a safe stop and exits MIT mode.

```
About to command AK40-10-KV170 id 1:
  position +0.1 rad   (field limit +/-12.5)
  kp 5, kd 0.3
  for 3 s at 100 Hz
Clamp the motor before continuing.
Proceed? [y/N]
```

Add `--yes` for scripts. Raise `--max-position` deliberately if you mean to go further.

### Checking what the library believes about a motor

```bash
cubemars dump-spec AK40-10
```

Every wire field with its resolution, the derived values, and **where each constant came
from**:

```
  ERPM -> rad/s (output)          7.479983e-04
  velocity field saturates above  25.55 V
  effective torque limit          4.1 Nm
  permanent zero (origin mode 1)  REFUSED (single encoder)

  gear_ratio           10.0 [datasheet: cubemars.com AK40-10 KV170 spec table]
  pole_pairs           14 [datasheet: ...]
  mit.position_side    Side.OUTPUT [measured: bench 2026-09-16, ...]
  servo.position_side  unknown (...)
  -> 1 unknown: servo.position_side
```

Anything still `unknown` will make the conversions that need it **refuse** rather than
guess. With no argument, lists every known model and variant:

```bash
cubemars dump-spec
```

### A different adapter or motor

```bash
cubemars scan --url "slcan:/dev/tty.usbmodem1101@1M"     # USB-CAN on macOS
cubemars scan --url "gs_usb:0@1M"                        # CANable / candleLight
cubemars scan --url "socketcan:vcan0"                    # virtual, no hardware
cubemars monitor --url socketcan:can1 --id 2 --motor AK80-9
```

### Testing with no hardware at all

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
cubemars scan --url socketcan:vcan0
```

A `vcan` interface behaves like a real one, which is how this project's CI exercises the
socketcan path. For a motor that actually answers, use the simulator through the
[examples](https://github.com/verlab/cubemarsPyCAN/tree/main/examples) instead: every one takes `--sim`.

---

## Exit codes

| Code | Meaning |
|---|---|
| 0 | success; for `scan`, at least one motor found |
| 1 | `scan` found nothing, `jog` was declined, or a library error occurred |
| 2 | `jog` refused the target as beyond `--max-position` |

Library errors print as `error: <message>` on stderr.
