# Troubleshooting

## Nothing arrives on the bus

In order of likelihood:

1. **Servo mode with the CAN status rate set to 0.** The driver never uploads anything.
   The wiring is fine. Check this first in CubeMarsTool; it is the commonest cause and it
   looks exactly like a dead bus.
2. **MIT mode, and nothing is commanding.** A MIT driver only replies when it receives a
   frame. `cubemars scan --poke 4` sends enter-MIT frames to ids 1–4 to elicit replies.
   (It leaves those drivers in MIT mode.)
3. **Bitrate.** AK drivers are 1 Mbit/s. On socketcan the kernel owns this; `cubemars
   doctor` shows what each interface is actually configured for.
4. Power, wiring, termination.

`cubemars scan` distinguishes these and says which it thinks it is seeing.

## Feedback stops after a while, or never starts

**A control loop with no sleep starves the receive thread.** Frames are delivered by a
python-can notifier thread; a tight loop that only sends and computes may never release
the GIL, so nothing is ever delivered and the state latch stays empty. Every real control
loop sleeps or blocks on a timer — but a busy-wait will look exactly like a dead motor.

This bit during development: a stepped-simulator test left the latch permanently empty
until the harness was given a yield.

## Position lags the setpoint by a constant-looking amount

Almost always a missing velocity feedforward. With `velocity=0` while commanding a moving
position, the `kd` term opposes the motion you are asking for, giving an error of about
`kd * v / kp`. Feed the target's derivative as `velocity`. Measured 3.5x improvement on
real hardware - see [units.md](units.md).

## `state.seq` jumps by more than one

Normal. The latch holds only the newest frame, so if several arrived between two
`update()` calls the sequence number advances by several. It means your loop is slower
than the motor's reply rate, not that frames were lost. Compare `seq` deltas against
`bus.transport.stats.rx` if you want the real reception count.

## The motor lurches right after `zero_here()`

Zeroing moves the coordinate system. A position setpoint that was in flight is then
expressed in the old frame, and the motor drives back toward where it just came from.

`zero_here()` now handles this for you: it stages zero gains **and puts them on the wire**
before moving the origin. Staging alone is not enough - `hold()` only assigns to the
staged command, it does not transmit - which is why calling `hold()` yourself was never
quite the fix it appeared to be.

Found while writing `examples/homing.py`: after homing to a stop and backing off 0.05 rad,
the zero appeared to read 0.0498 rad instead of 0 - the motor was being commanded back to
the pre-zero target.

## `StaleFeedbackError` right after `zero_here()`

The driver stops replying for about a second while it zeroes.

The reason a bare `time.sleep()` fails is not that it sends nothing. Staleness is measured
against the last frame **received**, so transmitting through the gap does not reset it
either - `settle()` alone would raise just the same. What fixes it is that `zero_here()`
opens an explicit grace window (`grace_s`, default 1.5 s) that tolerates the silence, and
only the *fatal* limit is suppressed, so the staleness warning still fires and a link that
never comes back is still visible. Use `motor.settle(1.5)` to keep the loop running
through the window.

Found on the bench, not in simulation - the simulator zeroes instantly.

## `StaleFeedbackError`

Feedback is older than `SafetyPolicy.stale_fatal_s`. A safe stop has already been sent.
Usual causes: the motor lost power, the loop is slower than the warn threshold, or the
link is saturated. `MotorBus.transport.stats.tx_percentiles()` shows whether sends are
taking longer than the loop period.

## `MotorFault`

The driver reported a non-zero fault code. A safe stop has already gone out, and the
exception is raised on **your** thread, not the receive thread. Address the cause, then
`motor.clear_fault()`.

Codes are 0–7 for CAN feedback (manual v1.0.18 p.45). Code 7, motor stall, is new in
v1.0.18; libraries written against older manuals raise `KeyError` on it.

> The serial `GET_VALUES` reply uses a **completely different** fault table where 1 means
> over-voltage rather than over-temperature. Serial is out of scope for v1, but
> `SerialFault` exists so nobody reaches for the wrong one later.

## `ServoModeNotConfirmed`

No `0x29` status and no `0x2C` acknowledgement arrived. The message triages what was
actually seen: nothing at all, frames on another id, or frames on your id with an
unrecognised function (usually a driver still in MIT mode — construct with
`assume_mit=True` to send the MIT exit frame on entry).

The manual documents no frame that *causes* servo-mode entry, so this library detects
rather than guesses. Servo mode is selected in CubeMarsTool.

## `SpecIncompleteError`

A conversion needs a constant this spec does not have — usually `pole_pairs` or `Kt` for a
model whose datasheet has not been transcribed. Measure it and record it with
`spec.evolve(...)` using `Source.MEASURED`. Raw wire values remain available meanwhile.

This is deliberate. Guessing is what produced an 8.6× position error and a 78× torque
error in the library this one replaces.

## `CapabilityError` on `set_origin`

`OriginMode.PERMANENT` writes flash and the manual restricts it to dual-encoder models.
The AK40-10 has one encoder, so it is refused and **no frame is sent**. Use
`OriginMode.TEMPORARY` (the default).

## `UnresolvedFrameError` on `status.output_rad`

The manual never says whether servo position is rotor- or output-side. `position_deg` is
always available. Settle it with bench step B7.

## The motor twitches at "zero" torque

It shouldn't, but the floor is real: no MIT field can encode an exact zero, so a commanded
`0.0 N·m` arrives as +1.22 mN·m on an AK40-10 — half an LSB. That is 2% of the motor's
60 mN·m break-away torque, so a healthy motor cannot move on it. If yours does, suspect
mechanical binding or a mis-set `Kt`.

## Commands near full scale behave strangely

They shouldn't, and this library guards it, but the protocol hazard is worth knowing. With
position, velocity, Kp and Kd all at maximum, a torque near 4.9976 N·m packs to
`FF FF FF FF FF FF FF FE` — which is byte-identical to the *set position to zero* frame.
The neighbouring codes are enter and exit MIT mode.

`pack_command` detects the collision and steps one torque LSB (2.4 mN·m) away. Other
implementations, including TMotorCANControl, do not.
