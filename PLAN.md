# cubemarsPyCAN — analysis and build plan

Target hardware: **CubeMars AK40-10 KV170**, AK-DRV-V2.1 driver ("AK 2.0").
Reference: *AK Series Module Driver Manual* **V1.0.18** (2026‑01‑19), the PDF in this repo.
Audited codebase: `TMotorCANControl-master` (v1.2.6, GPLv3, neurobionics/UMich).

---

## 1. Verdict: write `cubemarsPyCAN`, don't patch TMotorCANControl

**Recommendation: new library.** Four blockers, each of which independently forces a
rewrite of the layer it sits in:

1. **`servo_can.py` does not execute.** Six of its seven CAN commands raise before a
   frame reaches the bus (see §2.1). This is not "outdated", it is non-functional. Fixing
   it means rewriting the CAN layer.
2. **The unit contract is wrong in the public API.** `set_motor_torque_newton_meters`,
   `get_motor_torque_newton_meters`, `rad_per_Eang` and `set_impedance_gains_real_unit`
   are wrong in ways you cannot fix without breaking every caller (§2.3). A fix is a new
   major version with a different API — i.e. a new library.
3. **The transport is a `sudo`-shelling singleton.** `CAN_Manager.__new__` runs
   `os.system('sudo /sbin/ip link set can0 …')`, hard-codes `can0` and 1 Mbit/s, and is a
   process-wide singleton. There is no seam to inject a bus, so **nothing is testable
   without hardware**. Every other defect below survived for years because of this.
4. **It targets AK 1.0-era firmware.** Your manual is AK 2.0 / v1.0.18 and adds
   AK40-10 (not present in the library at all), a new CAN fault code 7, a corrected
   fault-6 meaning, and a corrected pos-velocity scaling. The motor table would have to
   be rebuilt regardless.

**Licensing note:** TMotorCANControl is **GPLv3**. If you copy or adapt its code,
`cubemarsPyCAN` must be GPLv3. If you want a permissive licence (MIT/BSD/Apache),
write the codec clean-room from the manual — which is what this plan does anyway,
because the manual's own hex examples give you the whole protocol (§6).

**What is worth keeping from TMotorCANControl** (as reference, reimplemented):
- The AK80-9 friction/torque regression model (`a_hat`) — genuinely useful research output.
- The idea of a multi-turn position unwrapper.
- The `with`-block power-on/power-off lifecycle.
- The serial CRC table and the `mit_can.py` MIT bit-packing, which are correct.

---

## 2. Evidence — what is actually broken

Everything below was **reproduced**, not read off. Method: stub `os.system`, swap
`can.interface.Bus` for python-can's `virtual` backend, import the real modules.

### 2.1 `servo_can.py`: every motion command is dead on arrival

```
set_duty     FAIL -> NameError: name 'send_index' is not defined
set_current  FAIL -> NameError: name 'send_index' is not defined
set_brake    FAIL -> NameError: name 'send_index' is not defined
set_rpm      FAIL -> NameError: name 'send_index' is not defined
set_pos      FAIL -> TypeError: buffer_append_int32() takes 2 positional arguments but 3 were given
set_pos_spd  FAIL -> TypeError: buffer_append_int32() takes 2 positional arguments but 3 were given
set_origin   OK
```

`pyflakes` flags the four `undefined name 'send_index'` statically
(`servo_can.py:414, 428, 442, 456`). Servo-over-CAN has never worked in this release.

### 2.2 Decode path crashes on modern NumPy, and on half the position range

`parse_servo_message` does `np.int16(data[0] << 8 | data[1])`. NumPy ≥ 2.0 raises on
out-of-range Python ints instead of wrapping:

```
pos raw 0x4E20 (+20000) -> pos=2000.0 deg          OK
pos raw 0xB1E0 (-20000) -> OverflowError: Python integer 45536 out of bounds for int16
```

So with NumPy 2.x, **any negative position crashes the receive path** — and it crashes
inside the python-can notifier thread, where the traceback is swallowed. Same pattern in
`np.int32(...)` on the transmit side.

### 2.3 Unit and gear-ratio errors in the public API

Reproduced on `AK80-9` (GR = 9, `Kt_actual` = 0.115):

```
set_motor_torque_newton_meters(1.0)  -> commands 0.1111 A
   -> which is 0.0128 Nm at the motor shaft.   Asked for 1.0 Nm. Off by ~78x.
get_motor_torque()  reports 1.0350 Nm
get_output_torque() reports 0.1150 Nm    <-- output must be 9x motor, not 1/9
```

Cause: `set_motor_torque_newton_meters` multiplies by `Kt_actual` where it should
multiply by `GEAR_RATIO`; `get_motor_torque_newton_meters` multiplies by `GEAR_RATIO`
where it should divide. The two sides of the gearbox are swapped.

`servo_can.py` compounds this:

- `rad_per_Eang = π / NUM_POLE_PAIRS` is applied to the servo position feedback — but
  that feedback is **mechanical degrees** (`int16 × 0.1`, range ±3200°), not electrical
  angle. The correct factor is `π/180`. As written it is wrong by 180/21 ≈ **8.6×**.
- `radps_per_ERPM = 5.82e-4` is a hard-coded magic number that ignores the
  `NUM_POLE_PAIRS` field sitting right next to it, and is absent for `AK10-9`
  (→ `KeyError` on construction).
- `get_motor_velocity_radians_per_second` returns `velocity * GEAR_RATIO` with **no**
  ERPM→rad/s conversion at all, while `get_output_velocity_*` does apply it.
- `set_output_angle_radians` guards `abs(pos) >= P_max` where `P_max = 32000` is in raw
  0.1° counts and `pos` is in radians — the guard can never fire.
- `comm_can_set_pos` scales by `1e6`; the manual says `1e4`. **100× wrong.**
- `comm_can_set_pos_spd` omits the `/10` that the manual applies to speed and
  acceleration, and passes floats into a bit-shift (`TypeError`).

`set_impedance_gains_real_unit(K, B)` documents K as "stiffness in Nm/rad" and B as
"damping in Nm/(rad·s⁻¹)", then passes them **unconverted** into the MIT Kp/Kd fields,
which the firmware interprets as its own internal gains at the rotor. There is no
"real unit" anywhere in the path; the name is the bug.

### 2.4 Faults and state

```
error code 7 (motor stall, new in v1.0.18) -> KeyError: 7
```

- Both CAN modules are missing code 7, and label code 6 "Phase current unbalance" when
  v1.0.18 says **MOSFET over-temperature**.
- `servo_serial.py` correctly uses the *other*, longer `mc_fault_code` enum. The manual
  really does define **two incompatible fault tables** — one for CAN feedback (0–7), one
  for the serial `GET_VALUES` reply (0–18). Any library must keep them separate.
  TMotorCANControl gets this right only by accident of having been written twice.

### 2.5 Acceleration sign is inverted

```
velocity rose 0 -> +1 rad/s, reported acceleration = -79.76 rad/s^2
```

`dt = self._last_update_time - now` — backwards. Present identically in `mit_can.py` and
`servo_can.py`. `servo_serial.py` has a different bug in the same place:
`acceleration = speed / dt` (velocity over dt, not Δvelocity over dt).

### 2.6 Errors raised in the RX thread

`_update_state_async` raises `RuntimeError` on a non-zero fault code. It runs in the
python-can `Notifier` thread, so the exception **never reaches your control loop** — the
motor keeps being commanded while faulted. This is the most safety-relevant defect in
the codebase.

### 2.7 Frame filtering is too loose

- MIT listener matches on `data[0] == motor_id` only. It ignores the arbitration ID,
  `is_extended_id` and the DLC, so any 8-byte frame from anything whose first byte
  collides is decoded as motor state.
- Servo listener matches `arbitration_id & 0xFF` but **ignores the function ID** in bits
  8–15. The manual defines three reply types: `0x29` (state), `0x2C` (entered-servo-mode
  handshake, payload fixed `FA FB FC FD`) and `0x09` (bootloader jump). The library
  decodes all three as state — the handshake frame becomes a bogus −128.5° reading.
- `check_can_connection()` attaches a bare `BufferedReader` to the shared notifier and
  counts *any* 10 messages, so on a multi-motor bus it reports success for a motor that
  is not there.

### 2.8 Smaller but real

| | |
|---|---|
| `mit_can.py:1079` | `__str__` uses `self.θ/θd/i/τ`, which don't exist → `print(dev)` raises `AttributeError` (reproduced) |
| `mit_can.py` | `Current_Factor = 0.59` (an AK80-9 empirical fudge) is copied to every motor, most marked `UNTESTED CONSTANT!` |
| `mit_can.py` | `AK60-6` has `V_max = 50.0`; manual says **45.0** |
| `mit_can.py` | `AK80-64` has `GEAR_RATIO = 80.0`; it is **64** |
| `servo_can.py` | `set_zero_position()` sends origin mode **1** = *permanent* zero, which v1.0.18 restricts to dual-encoder models and which writes flash. Default must be **0** (temporary). |
| `servo_can.py` | `power_on()` sends the **MIT** `FF…FC` frame as an extended-ID servo frame — with control-mode bits = 0 this lands as a malformed DUTY command |
| `servo_serial.py` | rotor-position feedback scaled `/1000` (manual: **/10000**) and gratuitously negated |
| `servo_serial.py` | default baud `961200`; README and the tool say **921600** |
| all | duplicate imports; `__init__.py` pulls `can` *and* `serial` on any import |
| packaging | `pyproject.toml` has only a build-system table; `setup.cfg` carries everything and pins `NeuroLocoMiddleware` as a hard runtime dep for what the demos need |
| repo | ships `__pycache__/`, `dist/`, `docs/build/`, `.DS_Store` |
| | no tests of any kind |

---

## 3. Protocol ground truth (manual v1.0.18)

This is the reference the new library encodes. Page numbers are from the PDF in this repo.

### 3.1 AK40-10 MIT ranges (p. 63) — **your motor**

| Field | Range | Bits |
|---|---|---|
| Position | −12.5 … +12.5 rad | 16 |
| Velocity | **−45.5 … +45.5 rad/s** | 12 |
| Torque | **−5.0 … +5.0 N·m** | 12 |
| Kp | 0 … 500 | 12 |
| Kd | 0 … 5 | 12 |

Full table, all ten models in v1.0.18 (P is ±12.5 rad for every model):

| Model | V (rad/s) | T (N·m) |
|---|---|---|
| AK10-9 | ±50.0 | ±65.0 |
| AK60-6 | ±45.0 | ±15.0 |
| AK70-10 | ±50.0 | ±25.0 |
| AK80-6 | ±76.0 | ±12.0 |
| AK80-9 | ±50.0 | ±18.0 |
| AK80-64 | ±8.0 | ±144.0 |
| AK80-8 | ±37.5 | ±32.0 |
| AK45-36 | ±6.0 | ±34.0 |
| AK45-10 | ±20.0 | ±8.0 |
| **AK40-10** | **±45.5** | **±5.0** |

> ⚠️ These are the **scaling ranges for the CAN fields**, not the motor's mechanical
> limits, and they must match the firmware's constants exactly or every command is
> silently mis-scaled. Verify against your actual firmware build before trusting torque
> numbers (§8).

### 3.2 MIT frame layout (pp. 60–68)

Command — **standard** frame, arbitration ID = motor ID, DLC 8:

```
D0 = p>>8   D1 = p&0xFF   D2 = v>>4   D3 = (v&0xF)<<4 | kp>>8
D4 = kp&0xFF   D5 = kd>>4   D6 = (kd&0xF)<<4 | t>>8   D7 = t&0xFF
```

Reply — standard frame, DLC 8:

```
D0 = driver ID
pos = D1<<8 | D2                    (16 bit, P_MIN..P_MAX)
vel = D3<<4 | D4>>4                 (12 bit, V_MIN..V_MAX)
tau = (D4 & 0xF)<<8 | D5            (12 bit, -T_MAX..T_MAX)
temp = D6 - 40                      (deg C, range -40..215)
err  = D7
```

Note the reply's third field is **torque**, not q-axis current — the manual's
`unpack_reply` names it `torque` and scales it by `±T_MAX`. TMotorCANControl converts it
to `0.59 · τ / (GEAR_RATIO · Kt)` and calls the result "q-axis current" — three modelling
assumptions baked into what looks like a raw reading. Keep the decoded torque as the
primary value; offer current as an explicitly-modelled derived quantity.

Special frames (8 bytes, same standard ID):

| Action | Payload |
|---|---|
| Enter MIT mode | `FF FF FF FF FF FF FF FC` |
| Exit MIT mode | `FF FF FF FF FF FF FF FD` |
| Set current position = 0 | `FF FF FF FF FF FF FF FE` |

### 3.3 Fix the float↔uint scaling

The manual's own `float_to_uint` uses `(1<<bits)/span`, which **overflows the field at
exactly x_max** — verified:

```
x = +12.5 rad, 16 bit : manual -> 65536  (does not fit in 16 bits)
tau = +5.0 N·m, 12 bit : manual -> 4096  (does not fit in 12 bits)
```

TMotorCANControl papers over this by clamping the input to `x_max − 2/bitratio`.
Use `((1<<bits)-1)/span` with rounding instead: it is the exact inverse of the firmware's
`uint_to_float`, never overflows, and differs from the manual's formula by **at most
1 LSB (0.00038 rad on position)**.

```python
def float_to_uint(x, lo, hi, bits):
    x = min(max(x, lo), hi)
    return int(round((x - lo) * (((1 << bits) - 1) / (hi - lo))))


def uint_to_float(u, lo, hi, bits):
    return u * (hi - lo) / ((1 << bits) - 1) + lo
```

### 3.4 Servo mode over CAN (pp. 35–45)

**Extended** frame. `arbitration_id = (packet_id << 8) | motor_id`.

| Packet | ID | Payload | Scale |
|---|---|---|---|
| SET_DUTY | 0 | int32 | `duty × 100 000` |
| SET_CURRENT | 1 | int32 | `A × 1000`, ±60000 |
| SET_CURRENT_BRAKE | 2 | int32 | `A × 1000`, 0…60000 |
| SET_RPM | 3 | int32 | ERPM, ±100 000 |
| SET_POS | 4 | int32 | **`deg × 10 000`**, ±360 000 000 |
| SET_ORIGIN_HERE | 5 | uint8 | 0 = temporary, 1 = permanent (dual-encoder only) |
| SET_POS_SPD | 6 | int32 + int16 + int16 | pos `deg × 10 000`; **spd `ERPM/10`**; **acc `(ERPM/s²)/10`** |
| SET_MIT | 8 | undocumented | present in the manual's `CAN_PACKET_ID` enum; no payload or example given — treat as unsupported until sniffed |

Feedback, function ID **`0x29`** only (`(arb_id >> 8) & 0xFF`), DLC 8:

```
pos  = int16(D0<<8|D1) * 0.1     deg     (±32000 -> ±3200 deg)
vel  = int16(D2<<8|D3) * 10.0    ERPM    (±32000 -> ±320000 ERPM)
cur  = int16(D4<<8|D5) * 0.01    A       (±6000  -> ±60 A)
temp = int8(D6)                  deg C   (-20..127)
err  = uint8(D7)
```

Also decode, and do **not** treat as state: `0x2C` (entered servo mode, payload
`FA FB FC FD`) and `0x09` (jump-to-bootloader).

CAN fault codes: `0` none · `1` motor over-temp · `2` over-current · `3` over-voltage ·
`4` under-voltage · `5` encoder · `6` **MOSFET over-temp** · `7` **motor stall**.

### 3.5 Servo mode over serial (pp. 46–59)

Frame: `0x02 | len | payload… | crc16_hi | crc16_lo | 0x03`, CRC-16/XMODEM over the
payload only. (For payloads > 255 the header becomes `0x03 | len_hi | len_lo`.)

The **position scale differs per command** — a genuine trap, both verified against the
manual's own hex:

| Command | ID | Scale |
|---|---|---|
| `COMM_SET_POS` | 9 | `deg × 1 000 000` |
| `COMM_SET_POS_SPD` | 91 | `deg × 1 000`, then int32 ERPM, int32 ERPM/s² |
| `COMM_ROTOR_POSITION` reply | 22 | `/ 10 000` |
| `GET_VALUES` outer-loop position | 4 | `/ 1 000` |
| `GET_VALUES_SETUP` motor position | 50 | `/ 1 000 000` |

Undocumented-in-the-enum but present in the examples: **`0x65` (101) = "shortest
distance to zero"**.

Serial `GET_VALUES` uses the long `mc_fault_code` enum (over-voltage = 1,
under-voltage = 2, DRV = 3, over-current = 4, FET over-temp = 5, …, unbalanced = 18) —
**not** the CAN table.

---

## 4. Architecture for `cubemarsPyCAN`

The organising principle: **a pure codec layer that never touches I/O.** That single seam
is what makes the ~30 defects in §2 impossible to ship, because every one of them
becomes a unit test that runs on a laptop.

```
cubemarspycan/
  units.py              # ERPM<->rad/s, deg<->rad, float<->uint. Pure. No I/O.
  specs.py              # frozen MotorSpec dataclass + registry, each field citing manual p.
  faults.py             # CanFault and SerialFault as separate IntEnums
  codec/
    mit.py              # encode_command/decode_feedback: bytes in, bytes out
    servo_can.py        # encode_*/decode_feedback + arbitration-ID helpers
    servo_serial.py     # framing, CRC, per-command scaling
  transport/
    base.py             # Transport protocol: send(frame), subscribe(cb), close()
    can_bus.py          # wraps an injected can.BusABC
    serial_bus.py       # wraps an injected serial.Serial
  bus.py                # MotorBus: routing, RX thread, fault latch, liveness
  motor.py              # MitMotor / ServoMotor  (the user-facing API)
  sim.py                # a fake motor that speaks the protocol, for tests + demos
  cli.py                # scan / monitor / jog / dump-spec
```

### Design rules

1. **Codec = pure functions.** `mit.encode_command(spec, p, v, kp, kd, tau) -> bytes`.
   No sockets, no threads, no globals. This is where the golden vectors bite (§6).
2. **Transport is injected.** `MotorBus(bus=can.Bus(channel="can0", interface="socketcan"))`.
   Accept any python-can backend: socketcan, slcan, USB adapters, `virtual` for tests.
   Offer `MotorBus.open("socketcan:can0@1M")` as sugar, never as the only path.
3. **No `sudo`, ever.** Interface bring-up is documented, plus an opt-in
   `cubemarspycan up can0` CLI that prints the command it would run before running it.
4. **No singleton.** Two buses in one process must work.
5. **Never raise in the RX thread.** Latch the fault into state; raise on the caller's
   thread at the next `update()`. Add an explicit `motor.fault` property and an
   `on_fault` callback.
6. **Explicit unit frames.** Two namespaces, no ambiguous bare names:
   `motor.rotor.position_rad` vs `motor.output.position_rad`. Never a method whose name
   says "real unit" while doing no conversion.
7. **MIT Kp/Kd are firmware gains, dimensionless to us.** Expose them as `kp`, `kd`.
   Provide `spec.kp_for_output_stiffness(nm_per_rad)` as a *separate, documented*
   helper that states its gear-ratio assumption, so the conversion is opt-in and visible.
8. **Torque is the primary MIT feedback quantity.** Current is derived, behind
   `motor.estimated_current_a`, and only when the spec carries a measured `kt`.
9. **`MotorSpec` is frozen and sourced.** Each spec records `manual_version="1.0.18"` and
   the page it came from, plus `verified: bool` for constants you have measured yourself.
10. **Multi-turn unwrapping is opt-in** and states its Nyquist condition: sampling must be
    faster than half a field-span of motion. For AK40-10 MIT at 45.5 rad/s and a 12.5 rad
    half-span that means dt < 0.27 s — trivially satisfied, but it belongs in the docstring.

### Public API sketch

```python
from cubemarspycan import MotorBus, MitMotor, SPECS

with MotorBus.open("socketcan:can0@1M") as bus:
    m = MitMotor(bus, motor_id=1, spec=SPECS["AK40-10"])
    with m.control():  # enter MIT mode, exit guaranteed
        m.set_zero()
        for t in loop:
            s = m.update()  # send command, return fresh state
            m.command(position=0.5, velocity=0.0, kp=20, kd=0.5, torque=0.0)
            print(s.output.position_rad, s.torque_nm, s.temperature_c)
```

---

## 5. Unit contract (write this down once, enforce in tests)

| Quantity | Wire | Library type | Conversion |
|---|---|---|---|
| MIT position | uint16 over ±P_max | rad, **output side** | `uint_to_float` |
| MIT velocity | uint12 over ±V_max | rad/s, output side | `uint_to_float` |
| MIT torque | uint12 over ±T_max | N·m, output side | `uint_to_float` |
| MIT temperature | uint8 | °C | `raw − 40` |
| Servo position | int16 ×0.1 | **degrees**, mechanical† | `× π/180` for rad |
| Servo velocity | int16 ×10 | **ERPM** | `rad/s = ERPM · 2π / (60 · pole_pairs · gear_ratio)` |
| Servo current | int16 ×0.01 | A (q-axis) | — |
| Rotor ↔ output | — | — | `output = rotor / gear_ratio`; torque is the inverse |

† The manual states the unit (degrees) and range (±3200°) but never says **which side of
the gearbox** the servo position refers to, and likewise does not state that MIT position
is output-side. MIT ±12.5 rad ≈ ±2 output turns is the near-universal reading and is what
the table above assumes, but both need one bench check (§8.7) before any controller
depends on them.

For AK40-10: `gear_ratio = 10`. **`pole_pairs` is not in the manual — read it from
CubeMarsTool and record it in the spec** (§8). Until then, refuse to convert ERPM→rad/s
rather than guessing; TMotorCANControl guessed (`5.82e-4`) and was wrong.

---

## 6. Test strategy — the manual hands you the test suite

The serial section prints literal byte strings for nearly every command. I ran these
through a from-scratch encoder: **19/19 reproduce exactly, CRC included.**

```
PASS  duty  +0.20          02 05 05 00 00 4E 20 29 F6 03
PASS  duty  -0.20          02 05 05 FF FF B1 E0 77 85 03
PASS  iq    +5 A           02 05 06 00 00 13 88 8B 25 03
PASS  iq    -5 A           02 05 06 FF FF EC 78 E3 05 03
PASS  brake  5 A           02 05 07 00 00 13 88 21 74 03
PASS  rpm +1000            02 05 08 00 00 03 E8 2B 58 03
PASS  pos  180 deg         02 05 09 0A BA 95 00 1E F7 03
PASS  pos   90 deg         02 05 09 05 5D 4A 80 7B 29 03
PASS  handbrake 5A         02 05 0A 00 00 13 88 00 0E 03
PASS  posspd 180/5000/30000 02 0D 5B 00 02 BF 20 00 00 13 88 00 00 75 30 A5 AC 03
PASS  multi-loop           02 05 5C 00 00 00 00 9E 19 03
PASS  single-loop          02 05 5D 00 00 00 00 34 48 03
PASS  set origin           02 02 5F 01 0E A0 03
PASS  shortest->0          02 05 65 00 00 00 00 3A 8B 03
PASS  get values           02 01 04 40 84 03
PASS  get position         02 02 0B 04 9C 7E 03
PASS  get MOS temp         02 05 32 00 00 00 01 58 4C 03
PASS  dbg 'encoder'        02 08 14 65 6E 63 6F 64 65 72 B0 4C 03
PASS  dbg 'exit'           02 05 14 65 78 69 74 96 C3 03
```

That is the whole serial protocol pinned by construction, including the two different
position scalings. Freeze them as `tests/golden/servo_serial.py`.

Four layers:

1. **Golden vectors** — the 19 above, plus the manual's servo-CAN and MIT worked examples.
2. **Property tests** (hypothesis) — `uint_to_float(float_to_uint(x)) ≈ x` within 1 LSB for
   every field of every motor in the registry; encode/decode round-trip for every packet.
3. **Simulated bus** — `sim.py` speaks the protocol over python-can `virtual`. Whole
   control loops run in CI, no hardware. This is what `check_can_connection` should have
   been tested against.
4. **Hardware smoke test** — a marked, opt-in `pytest -m hardware` suite you run on the
   bench: enter/exit MIT, zero, small position step, read back, confirm sign conventions.

---

## 7. Milestones

| # | Deliverable | Notes |
|---|---|---|
| 0 | Repo skeleton, `pyproject.toml` (PEP 621, hatch/setuptools), ruff + mypy strict, CI | Runtime deps: `python-can` only. `pyserial` as an extra. Drop `numpy` — it buys nothing here and its v2 change is what broke the old decoder. |
| 1 | `units.py` + `specs.py` with all 10 models from §3.1, `faults.py` with both enums | Pure, 100 % covered |
| 2 | `codec/mit.py` + golden/property tests | AK40-10 correct from day one |
| 3 | `transport/` + `bus.py` + `sim.py`; loop runs green in CI | The seam that makes everything else testable |
| 4 | `motor.py` MIT API, fault latching, opt-in unwrapping | **First bench test on the AK40-10** |
| 5 | `codec/servo_can.py` + `ServoMotor` | Correct `1e4` scaling and `/10` spd/acc |
| 6 | `codec/servo_serial.py` + serial transport | 19 golden vectors already written |
| 7 | `cli.py`: `scan`, `monitor`, `jog`, `dump-spec` | `scan` is how you'll debug wiring |
| 8 | Docs: unit contract, CAN bring-up, AK 2.0 vs 1.0 differences, migration from TMotorCANControl | |

Milestones 0–4 get you a correct, tested AK40-10 in MIT mode. That is the useful
stopping point if you only need MIT.

---

## 8. Open questions — need hardware or CubeMarsTool

1. **Pole pairs for AK40-10.** Not in the manual. Read from CubeMarsTool, or derive:
   command a known ERPM in servo mode and time the output shaft. Blocks ERPM↔rad/s.
2. **Torque constant Kt.** Not in the manual either. KV170 gives a first estimate of
   ~9.55/170 ≈ 0.056 N·m/A rotor-side, but TMotor's published Kt and the true measured Kt
   disagree by ~25 % on the AK80-9, so **measure it** (known load, sweep current, fit).
   Until then mark `kt_verified = False` and don't quote torque from current.
3. **Firmware MIT constants.** §3.1 is the manual's table; confirm your firmware build
   agrees — command a known velocity and check the reported value scales 1:1. A mismatch
   here silently mis-scales everything.
4. **Does AK 2.0 MIT position wrap or saturate at ±12.5 rad?** TMotorCANControl assumes
   wrap. Test it before relying on the unwrapper.
5. **MIT reply arbitration ID.** Manual says "0x00 + Drive ID", which is ambiguous.
   Sniff it with `candump` and filter on the real value rather than on `data[0]` alone.
6. **Gear ratio.** The "-10" in AK40-10 implies 10:1, but read the driver's
   reduction-ratio parameter in CubeMarsTool to confirm — it is the one number that scales
   every rotor↔output conversion in the library.
7. **Which side of the gearbox do the position fields report?** The manual never says.
   Check both at once: with the motor in MIT mode, rotate the output shaft exactly one
   turn by hand and read the reported delta — 2π means output-side, 2π·10 means rotor-side.
   Repeat in servo mode (expect 360° vs 3600°). Everything in §5 hangs on this.

---

## 9. Appendix — AK40-10 spec, ready to drop into `specs.py`

Every number traced to manual v1.0.18. The `None` fields are the ones §8 tells you to
measure; keeping them `None` rather than guessing is the whole point — it makes the
library refuse a conversion it cannot do correctly instead of silently returning the
kind of 8.6×-wrong answer catalogued in §2.3.

```python
AK40_10 = MotorSpec(
    name="AK40-10",
    # --- MIT field scaling (manual v1.0.18 p.63) ---
    p_min=-12.5,
    p_max=12.5,  # rad,    16-bit field
    v_min=-45.5,
    v_max=45.5,  # rad/s,  12-bit field
    t_min=-5.0,
    t_max=5.0,  # N*m,    12-bit field
    kp_min=0.0,
    kp_max=500.0,  # firmware gain, 12-bit field
    kd_min=0.0,
    kd_max=5.0,  # firmware gain, 12-bit field
    # --- servo-mode field scaling (manual p.44, identical for all models) ---
    servo_pos_deg_per_lsb=0.1,  # int16 +/-32000 -> +/-3200 deg
    servo_erpm_per_lsb=10.0,  # int16 +/-32000 -> +/-320000 ERPM
    servo_amps_per_lsb=0.01,  # int16 +/-6000  -> +/-60 A
    # --- drivetrain: NOT in the manual, see PLAN.md section 8 ---
    gear_ratio=10.0,  # nominal from the model name; confirm in CubeMarsTool
    pole_pairs=None,  # blocks ERPM<->rad/s until measured
    kt_nm_per_a=None,  # KV170 suggests ~0.056 rotor-side; measure it
    kv_rpm_per_v=170.0,
    # --- provenance ---
    manual_version="1.0.18",
    verified=False,  # flip to True per-field once bench-checked
)
```

---

## 10. Migration path

Keep `TMotorCANControl-master/` in the tree read-only as a reference oracle while you
build, then delete it. Do not import from it — the GPLv3 licence would infect
`cubemarsPyCAN`, and §2 is the list of reasons you don't want its behaviour anyway.
