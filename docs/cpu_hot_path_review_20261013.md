# CPU review and behavior-preserving optimizations, 2026-10-13

## Scope

The user requested auditing recent additions for CPU overhead, then clarified
that existing behavior must be preserved. Do not trade recognition/control
functionality or configured cadence for lower load. This change removes repeated
work; it does not change ONNX models, perception thresholds, stop/go policy,
driver monitoring, CPU affinity, scheduling priority or configured worker duty.

The inventory compares `4320337100` (before September 25) with `c54ca5ac25`:
871 changed paths, including 252 runtime Python candidates after excluding
test directories and tools. This includes imported/vendor and other authors'
changes, not just agent-written code. Inventory/static screening is not a
profile of every file. Detailed review focused on the running signal observer,
planner, coasting, impact detection, UI/cluster, diagnostics, control/startup
checks, radar adapters and model/DM scheduling paths. Inactive installation,
web and external-host code does not explain the recorded onroad hot spots.

## Recorded load

The morning drive has 1,184 deviceState CPU samples. Per-core mean utilization:

| Core | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Mean % | 77.2 | 75.1 | 86.6 | 82.4 | 73.3 | 78.1 | 91.2 | 68.8 |

Core6 median is 93%, p95 95%. In a two-minute procLog window, process CPU
(100% = one core) is UI 35.06%, selfdrived 27.02%, controlsd 21.26%,
plannerd 22.93%, signalcolord 44.00%. The first three are observed on core6,
plannerd on core4 and signalcolord on cores0-3, consistent with source affinity.
The sampled processor field is not complete per-thread attribution; these
numbers do not precisely decompose all core utilization or prove the cause of
reported functional failures. Duplicate-name processes were excluded from these
figures because the exploratory aggregator averages their individual intervals.

The quoted old `signal_assist` block is disabled in production by SignalAssistMode;
similar context handling exists in the active start/preparation paths. Dictionary
access alone is not the dominant load. A native minimal-call benchmark measured
3.96 us with repeated defaults/conversions, 2.51 us with direct known fields and
2.49 us with a refilled reusable slots object. These are wrapper costs, not full
planner execution; replacing arguments with an object is not an automatic win.

## Changes

| Area | Repeated work removed | Behavior retained |
|---|---|---|
| Signal tracker | Share one current-frame grayscale image across detection and tracking; compute the upper-scene darkness histogram once | Same pixels, regions, thresholds, proposal order, track identity and current-color checks; no old-frame image cache |
| Impact detector | Cache gravity and calibration transforms until their validated input values change | Every new sensor sample, freshness check, invalid-input handling and trigger/rearm state still runs |
| Signal transport/display | Decode byte-identical JSON only once; retain bounded last-packet storage | Re-read the endpoint at the same cadence; context/freshness/activation checks still run every use; missing, malformed and oversized input cannot grant permission |
| Runtime diagnostics | Update existing aggregate cells instead of constructing default infinities and replacement tuples per metric | Same finite-sample selection, sums, maxima, counts and emission schedule |
| Planner/UI/cluster | Initialize owned members in constructors; access required internal context fields directly; avoid eager `monotonic()` default evaluation and repeated scalar conversion | Optional helper presence remains explicit; external-data validation remains; UI drawing and longitudinal decisions are unchanged |
| Coasting | Reject cheap ineligible controller/driver states before subscriber scans, navigation reads and speed-cap arithmetic | All original vetoes, target stability history reset, finite-value rejection and re-entry delay remain |

Python `and` already short-circuits. The coasting improvement comes from moving
unneeded work after early vetoes and resetting immediately, not merely rewriting
an expression with `if`. The state/lead/driver/freshness conditions are not
cached across cycles. Required internal fields use direct access; dynamic library
capability checks and genuinely optional/untrusted input fields still need
validation. No blanket removal of `getattr` or `get` was performed.

The legacy assist logger also builds its dataclass payload only when its existing
emission condition passes. There is no log-rate reduction. UI diagnostics retain
first-render initialization so their first interval does not acquire offroad idle
time. That initialization check is deliberately retained rather than changing
diagnostic timing for a negligible member-lookup saving.

## Native isolated A/B measurements

Vehicle remained offroad at production `2ec25da3`. Both variants ran in a separate
SCHED_OTHER/nice19 process pinned to core0 with the existing Python/OpenCV 4.13
environment. No production files, settings, services or vehicle controls changed.
Thread CPU time excludes sleeping; old/new order alternates. These are isolated
compute measurements on an offroad little core, not new whole-core onroad load.

| Workload | Before mean | After mean | Reduction |
|---|---:|---:|---:|
| Signal tracker, 70 stored images x 5 passes | 48.273 ms | 41.794 ms | 13.4% |
| Impact detector, 5,000 samples, 100 Hz IMU / 20 Hz pose / 1 Hz calibration | 355.48 us | 117.79 us | 66.9% |
| Diagnostic aggregation, 20 metrics, 100,000 calls | 68.78 us | 32.74 us | 52.4% |
| JSON transport, 20 tracks, 3,000 reads, new packet every second read | 431.09 us | 299.88 us | 30.4% |

All paired signal/impact results, transport outputs and aggregate payloads match.
The sensor/update schedule and JSON packet sizes are benchmark inputs, not a
claim about every production cycle. Core6 has different frequency/performance;
do not translate the impact numbers directly into a core6 percentage reduction.
The unchanged signal duty limiter can spend savings on earlier processing when
camera scheduling permits, so per-frame savings alone do not prove lower total
signalcolord utilization. Coasting and member-access cleanup are not assigned a
whole-process speedup. Windows CPU-clock quantization made its per-call timing
unsuitable; the table uses native measurements only.

## Equivalence and checks

- Replayed 11,046 actual encoded camera frames at retained observation timestamps
  across 23 completed night/morning segments through old and new production-mode
  trackers. Complete result dictionaries match, including boxes, scores, identity,
  raw/confirmed color and histories. Encoded images are not live NV12 parity proof.
- Same recorded-input Carrot state-machine replay: 21,550 updates match in state,
  signal decisions, stop distance, cruise target, comfort braking and events.
  Parameter polling/driving-mode/navigation adapters retain the existing replay
  limitations; this is not native MPC or closed-loop vehicle-response validation.
- Coasting A/B: 33 eligible/veto scenarios, 2,970 updates, including stable entry,
  transient veto, recovery, both leads, cut-in risk, stale/unseen navigation and
  nonfinite target differences. Targets and all coasting state fields match.
- 712 focused desktop tests pass, one existing platform-dependent skip. These
  include transport expiry/re-enable/malformed cases, live mode/freshness tests,
  detector/guard/preparation, UI geometry, diagnostics, impact transform changes,
  coasting vetoes and cluster camera/performance tests. Windows substitutes Params
  storage and native MPC/graphics adapters where already used by the tests.
- The existing impact/UI/sound focused runner additionally passes 123 tests
  with one pre-existing deselection. This is not a physical display/audio test.
- Differential lint checks preserve pre-existing findings and require no newly
  introduced findings. No blanket formatting of unrelated files.

## Publication and remaining measurement limits

Publish the same changes to the maintained carrot-nav and carrot-wip branches
with single-parent commits. Preserve their different navigation/model choices
and unrelated working-tree edits. This task does not install the change in the
vehicle. No measured post-change whole-core onroad utilization or resolution of
the user's unspecified stopped functions is claimed.

Existing native radar/CAN paths and UI geometry batching were already present;
their checks/history cannot safely be removed just to shorten code. The remaining
large UI/selfdrived/controlsd load needs same-workload onroad process/thread timing
to establish total headroom and deadline recovery. Retain the same metrics when
comparing a later drive: CPU utilization, runqueue wait, model/DM rate, camera gaps
and control timing, not CPU percentage alone.

Private inventory, before sources, native benchmark scripts/results, frame and
planner parity results are under `.analysis/scratch/2026-10-13-cpu-review/`.
Raw recordings and device-specific evidence stay private.
