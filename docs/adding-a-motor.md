# Adding a motor

The registry is deliberately **two-tier**, and which tier you touch depends on what you
are adding:

| Tier | What it holds | Provenance | Scope |
|---|---|---|---|
| **Per model** | the five MIT field ranges | manual v1.0.18 p.63 — authoritative | shared by every variant |
| **Per variant** | drivetrain, physical limits, capabilities | CubeMars product datasheets | one KV and hardware revision |

The split exists because KV and hardware revision change Kt, pole pairs and even encoder
count. `"AK80-9"` alone does not identify a set of constants — the V2.0 KV135 and the V3.0
KV100 are different motors that share a wire protocol.

There are three routes. Pick by what you actually have.

## 1. A new variant of a model already in the manual

The common case, and the least work: the wire scaling already exists, so you add only the
variant. In [`src/cubemarspycan/registry.py`](https://github.com/verlab/cubemarsPyCAN/blob/main/src/cubemarspycan/registry.py),
beside the existing four:

```python
_DS_AK80_9_V2 = "cubemars.com AK80-9 V2.0 KV135 spec table"

AK80_9_V2_KV135 = MotorSpec(
    name="AK80-9-V2.0-KV135",
    model="AK80-9",
    mit=MODEL_MIT_FIELDS["AK80-9"],  # already correct; never restate it
    drivetrain=Drivetrain(
        gear_ratio=Sourced(9.0, Source.DATASHEET, _DS_AK80_9_V2),
        pole_pairs=Sourced(21, Source.DATASHEET, _DS_AK80_9_V2),
        kt_nm_per_a=Sourced(0.070, Source.DATASHEET, _DS_AK80_9_V2, "rotor-side"),
    ),
    limits=PhysicalLimits(
        peak_torque_nm=Sourced(22.0, Source.DATASHEET, _DS_AK80_9_V2),
        peak_current_a=Sourced(28.0, Source.DATASHEET, _DS_AK80_9_V2),
        rated_voltage_v=Sourced(24.0, Source.DATASHEET, _DS_AK80_9_V2),
    ),
    capabilities=Capabilities(encoders=1),
)
```

Then add it to `_VARIANTS`. `SPECS` and the model-name alias are both derived from that
tuple, so nothing else needs editing.

Two things you will hit:

- **`tests/test_spec.py::test_variants_listed` asserts an exact set**, so add the new key
  there. It is a guard against a variant silently disappearing, so extend it rather than
  loosening it.
- **`unknown()` is not exported at the top level.** Import it from the spec module:
  `from cubemarspycan.spec import unknown`.

## 2. A model that is not in the manual's table

Then you need both tiers. Add a `MODEL_MIT_FIELDS` entry through the `_mit(v_max, t_max)`
helper — position, Kp and Kd are identical for every AK model, so the helper takes only
the two that differ — and a `MODEL_GEAR_RATIO` entry:

```python
MODEL_MIT_FIELDS = {
    ...
    "AK70-20": _mit(30.0, 40.0),   # velocity rad/s, torque N*m
}

MODEL_GEAR_RATIO = {
    ...
    "AK70-20": 20.0,
}
```

`tests/test_spec.py::test_every_manual_model_is_registered` compares `models()` against
the manual's table, so add the model there too.

With no variant, the model resolves to a `_skeleton()`: wire scaling authoritative, every
physical constant refusing. That is a **usable** state, not a broken one — you can command
position, velocity and torque in wire units immediately, and only the conversions that
need a datasheet constant will refuse.

## 3. Without modifying the library

`MotorSpec` is a plain frozen dataclass, so build one in your own code:

```python
from cubemarspycan import Capabilities, Drivetrain, MitMotor, MotorSpec, Source, Sourced
from cubemarspycan.registry import MODEL_MIT_FIELDS

my_spec = MotorSpec(
    name="AK80-9-mine",
    model="AK80-9",
    mit=MODEL_MIT_FIELDS["AK80-9"],
    drivetrain=Drivetrain(
        gear_ratio=Sourced(9.0, Source.DATASHEET, "my datasheet"),
        pole_pairs=Sourced(21, Source.DATASHEET, "my datasheet"),
        kt_nm_per_a=Sourced(0.095, Source.DATASHEET, "my datasheet"),
    ),
    capabilities=Capabilities(encoders=1),
)

m = MitMotor(bus, motor_id=1, spec=my_spec, supply_voltage=24.0)
```

Right choice when the motor is yours alone, or the datasheet is not public. You lose
`cubemars dump-spec` and the shared test coverage.

## Leave what you do not know unknown

This is the part that matters more than the mechanics.

Every constant that is not on a wire is wrapped in
{class}`~cubemarspycan.spec.Sourced`, which records where the number came from. A
conversion that needs an unknown constant raises
{class}`~cubemarspycan.errors.SpecIncompleteError` instead of guessing:

```python
from cubemarspycan.spec import unknown

kt_nm_per_a = unknown("no datasheet consulted yet")
```

## Recording a bench measurement

Do not hand-edit a constant after measuring it. Use
{meth}`~cubemarspycan.spec.MotorSpec.evolve`, which keeps the provenance attached:

```python
from cubemarspycan import Side, Source, Sourced, get_spec

spec = get_spec("AK80-9").evolve(
    mit_position_side=Sourced(
        Side.OUTPUT,
        Source.MEASURED,
        "bench 2026-09-17, socketcan can0",
        "one hand-turn of the output shaft read 6.2870 rad against 6.2832 expected",
    ),
)
```

Every bench step in [bench.md](bench.md) prints a paste-able block in exactly that shape,
so measurements flow back into the registry with their run date intact.

`AK40_10_KV170` is the worked example to copy. It carries two `Source.MEASURED` fields
from the 2026-09-16 bench, and its `no_load_speed_radps` note records that the datasheet's
own 435 rpm contradicts the Kv and Ke on the same page, which both give 408 rpm at 24 V.
Recording the contradiction is the point — silently picking one of the two numbers is how
a spec becomes untrustworthy.

## Verify

```bash
cubemars dump-spec AK80-9-V2.0-KV135
```

prints the fields, the derived values and the full provenance audit — the fastest way to
see that a spec is incomplete *before* a conversion refuses mid-run. Then:

```bash
pytest -q tests/test_spec.py tests/test_roundtrip.py
```

`test_roundtrip.py` is parametrised over `MODEL_MIT_FIELDS`, so a new model is
automatically checked for quantisation round-trips within one LSB and for never exceeding
its field's `max_uint`.
