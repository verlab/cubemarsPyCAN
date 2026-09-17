"""Record a run to CSV for later analysis.

Logging is deliberately not built into the motor class. What you want logged, at what
rate, in what format, and whether writing to disk is allowed to stall your control loop
are all application decisions. A loop of your own is a dozen lines and stays honest about
its costs.

Note the timestamp: `state.rx_monotonic` is when the frame was *received*, on a monotonic
clock. Use it, not `time.time()` at the top of your loop - they differ by however long the
frame sat in the latch, and `time.time()` can jump.

    python examples/log_to_csv.py --sim --out /tmp/run.csv
    python examples/log_to_csv.py --url socketcan:can0 --id 1 --out run.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

from _common import base_parser, open_rig, settle, wait_for_control

from cubemarspycan import MitMotor, SafetyPolicy

COLUMNS = [
    "t",
    "rx_monotonic",
    "seq",
    "target_rad",
    "position_rad",
    "velocity_radps",
    "torque_nm",
    "temperature_c",
    "fault_code",
]


def main() -> None:
    parser = base_parser(__doc__ or "")
    parser.add_argument("--out", default="run.csv", help="output file")
    parser.add_argument("--amplitude", type=float, default=0.3)
    parser.add_argument("--freq", type=float, default=0.4)
    args = parser.parse_args()

    out = Path(args.out)
    rows: list[tuple[object, ...]] = []

    # try/finally, because the interesting runs are the ones that end badly. MotorFault,
    # StaleFeedbackError and an over-temperature MotorError all raise by design, and the
    # rows leading up to one are exactly the rows worth having.
    try:
        _run(args, rows)
    finally:
        _flush(out, rows)
    _summarise(out, rows)


def _run(args: argparse.Namespace, rows: list[tuple[object, ...]]) -> None:
    with open_rig(args) as rig:
        motor = MitMotor(
            rig.bus,
            args.id,
            rig.spec,
            supply_voltage=args.supply,
            policy=SafetyPolicy(max_temp_c=70.0),
        )
        with motor.control(wait_s=wait_for_control(rig)):
            motor.zero_here()
            settle(rig, motor)

            w = 2 * math.pi * args.freq
            ticker = rig.ticker(args.period)
            # Buffer in memory and write at the end: a csv.writer flush inside the loop
            # is a filesystem call on your control thread.
            while ticker.running(args.duration):
                t = ticker.t
                target = args.amplitude * math.sin(w * t)
                state = motor.update(
                    position=target,
                    velocity=args.amplitude * w * math.cos(w * t),
                    kp=20.0,
                    kd=0.5,
                    torque=0.0,
                )
                rows.append(
                    (
                        round(t, 6),
                        round(state.rx_monotonic, 6),
                        state.seq,
                        round(target, 6),
                        round(state.position_rad, 6),
                        round(state.velocity_radps, 4),
                        round(state.torque_nm, 5),
                        state.temperature_c,
                        state.fault_code,
                    )
                )
                ticker.tick()
            motor.hold()


def _flush(out: Path, rows: list[tuple[object, ...]]) -> None:
    """Write the rows, if there are any, without ever displacing the real error.

    Two rules, each of which cost a real file:

    * An empty run must not truncate. ``_write`` opens "w", so a run that captured
      nothing - a wrong ``--id``, a fault before the first ``update()`` - replaced a good
      401-row CSV with a bare header. The run *before* a bad one is often the one worth
      keeping.
    * This runs from a ``finally``. An exception raised here would replace the MotorFault
      or StaleFeedbackError that is the actual diagnosis, so a write failure is reported
      and swallowed: the error you came for wins.
    """
    if not rows:
        print(f"no rows captured; {out} left as it was", file=sys.stderr)
        return
    try:
        _write(out, rows)
    except OSError as exc:
        print(f"could not write {len(rows)} rows to {out}: {exc}", file=sys.stderr)


def _write(out: Path, rows: list[tuple[object, ...]]) -> None:
    with out.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(COLUMNS)
        writer.writerows(rows)


def _summarise(out: Path, rows: list[tuple[object, ...]]) -> None:
    if not rows:
        return  # _flush has already said that nothing was written
    fresh = len({r[2] for r in rows})
    errors = [abs(float(r[4]) - float(r[3])) for r in rows]  # type: ignore[arg-type]
    print(f"wrote {len(rows)} rows to {out}")
    print(f"  distinct feedback frames : {fresh}")
    print(f"  repeats (loop faster than the motor replies): {len(rows) - fresh}")
    print(f"  tracking |err| median    : {statistics.median(errors) * 1000:.2f} mrad")
    print("\nA seq that advances by more than 1 between rows means several frames arrived")
    print("while your loop was busy. The latch keeps the newest; nothing was lost.")


if __name__ == "__main__":
    main()
