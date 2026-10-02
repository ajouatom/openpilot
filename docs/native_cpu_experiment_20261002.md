# Native CPU experiment, 2026-10-02

The user explicitly requested a separate trial branch after the EV9 CPU review.
`carrot-native-cpu` starts at `a4d8ef647c` on `carrot-wip`. Production branch,
vehicle settings and production NAS deployment remain separate.

## Why this work

EV9 `000002df--25c01be8dd--0` on `abe1a232` saturates core5. During 30–54 s,
card uses about 62.31% of that core; radard gets 36.89% CPU and spends 62.91%
waiting runnable. Radar input and model production remain near 20 Hz, while
radarState falls to about 16 Hz. No sustained raw radar/CAN fault appears.
`sm.all_checks()` becomes false when radard cannot consume inputs fast enough.
The downstream radarFault is therefore consistent with CPU starvation, not
evidence of a failed radar sensor. At 20 Hz the measured card/radard work would
demand roughly 107% of their shared core.

Comma's [0.10 release notes](https://blog.comma.ai/010release/) describe its Python
parser rewrite under car-interface maintainability and porting simplification.
They do not claim it was a CPU optimization.

## Scope

- Radar: pairwise median slopes, motion metrics and nearest-path-segment loops
  compiled through Cython to C++. Track history, thresholds, lead policy, path
  geometry/cache and terminal tangent rules remain Python.
- CAN: batch raw-signal extraction and packing loops compiled through Cython.
  Preserve arbitrary-width Python integers, original rounding, signal iteration
  order, checksum callbacks, counter bookkeeping and parser validation.
- No changes to model, affinity, real-time priorities, message frequencies,
  checksums, fault suppression or driving thresholds. No formula simplification.
- Preserve Python implementations as differential references. Process-start
  `CARROT_NATIVE_CPU=0` selects them; absent extension imports fall back to Python.
  `runtimeTiming.motion_backend` (radard) and `can_backend` (card) show `cython`
  or `python`. A fallback is functional but does not validate native performance.
- C++ double arithmetic disables fast-math and floating-point contraction.
  Nonfinite/unsupported radar inputs use the reference. Stable sorting preserves
  ties. A successful compile alone is not evidence of behavioral equivalence.

## Building and replay image

Normal device SCons includes both extensions. For desktop/standalone testing:

```sh
pip install Cython==3.2.9 setuptools
python tools/build_native_cpu.py build_ext --inplace
PYTHONPATH=.:opendbc_repo python opendbc_repo/opendbc/dbc/generator/generator.py
PYTHONPATH=.:opendbc_repo python -m pytest -c /dev/null --rootdir=. \
  --confcutdir=tools/native_cpu tools/native_cpu/test_kernels.py
```

The experiment's workflow builds/tests Linux x86_64 and ARM64 kernels and builds
a replay Docker image without publishing/deploying it. The NAS bundle includes
the `.pyx` sources and standalone builder; a separate image build stage compiles
extensions. Replay cache identity includes native sources and the selected radar
backend. Promotion still needs the normal NAS production verification workflow.

## Desktop validation

Windows CPython 3.12 / Cython 3.2.9 / MSVC x64 builds both extensions successfully.
The differential tests require the compiled modules to load. They compare
double bit patterns (including signed zero), irregular/duplicate timestamps,
short histories, nonfinite fallback, path projections, both CAN endiannesses,
1–128-bit signals, truncated payloads, six brands/DBCs, counters and checksum
rejection state. Existing CAN tests also exercise many DBC/checksum combinations.

- 682 initial radar tests and 256 additional radar/integration tests pass.
- Native differential + existing CAN suite: 120 tests and 586 subtests pass.
  One pre-existing `test_parser_can_valid` expects initial invalidity, contrary
  to this fork's two-second first-seen grace; it fails identically in Python and
  native modes and is excluded explicitly. The experiment does not alter grace.
- Replay service tests: 36 pass, four platform-dependent skips on Windows.
- Linux/ARM and image CI results are recorded after the branch run completes.

### EV9 recorded-input comparison

Input SHA256: `5f5edddf477dfecc038cd6acb61b1cc7cbbdd9011a07e7566c27389631794b1c`.
Full cereal schema, original mode 3, corner setting 2. Compare Python and native
variants on exactly the same reconstructed input schedule.

| Computation | Python mean | Native mean | Reduction |
|---|---:|---:|---:|
| Radar controller, 990 model-paced frames | 3.737 ms | 2.540 ms | 32.0% |
| CarInterface update+apply, 4,782 steady iterations | 0.628 ms | 0.554 ms | 11.8% |

Each mean averages three full replay runs. Radar excludes the first 30 frames;
card excludes the first 12 s. Comparisons cover all 990 radar output dataclass
fields and all 5,983 carState dictionaries and generated CAN byte sequences.
No differences were observed. Raw logs/settings are private analysis inputs and
are not committed. Reproduction scripts/results live in the local analysis archive.

Radar replay uses the latest preceding inputs at each model publication, not
the delayed live subscriber schedule. Card uses recorded CAN batch boundaries,
but reconstructed registration adds a missing BLINDSPOTS_REAR_CORNERS state;
its result is variant-to-variant equivalence, not equality to live vehicle TX.
Timers exclude IPC, Params and OS scheduling. PC reductions must not be applied
as measured ARM CPU reductions or proof that the vehicle fault is fixed.

## Device trial acceptance

On the experimental branch, first verify a full target build and `cython` in both
runtime summaries. Then compare the same parked workload/settings against Python:
per-core deviceState CPU, thread CPU/frame, runnable wait, card 100 Hz, radard
20 Hz, input age, CAN validity and radar/communication faults. Include front and
corner tracks and the same UI/DM/model configuration. Preserve fault thresholds.
If timing or outputs differ unexpectedly, return to `carrot-wip`; an environment
override alone is not a persisted vehicle setting. Loaded driving and multiple
vehicle types require further validation before production promotion.
