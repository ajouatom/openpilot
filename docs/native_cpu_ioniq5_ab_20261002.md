# Ioniq 5 PE native CPU before/after, 2026-10-02

## Scope and provenance

- Device: HYUNDAI_IONIQ_5_PE / 07b62e389ed26c81 / C4 (mici).
- Before: `00001005--f5fbe6ee48--3`, approximately 60 s. Raw initData is `carrot-wip / abe1a232d81fd5b0f8db4b7212587952514e0274`, clean.
- After: `00001006--475cef72a1--3`, approximately 60 s. Raw initData is `carrot-native-cpu / 64aa1b9b46158a15468d3c57931e8d2879bf828d`, clean.
- The upload presentation labels both as 64aa1b9b; the recorded initData distinguishes them. The reason for the presentation mismatch was not investigated.
- After: all 59 radard summaries have motion_backend=cython; all 60 card summaries have can_backend=cython. Before has no backend fields, consistent with its recorded pre-native source.
- Identical recorded carParams fingerprint/flags/extFlags, openpilot longitudinal enabled, 50 ms radar period. Same OS 19.8-carrot-bt1 and kernel.
- Both: EnableRadarTracks=1, EnableCornerRadar=2, RadarTrackFlip=0, DriverMonitoringEnabled=1, DriverMonitoringMode=1, CarrotVisionEnabled=1, SteerHandoverMode=3. Both DM models produce about 20 Hz.
- procLog observes card and radard on core5 (priorities -54/-52), radarcan/plannerd on core4, model/DM on core7 in both. This is processor sampling, not a complete affinity-mask trace.

## Whole-core CPU

DeviceState samples, 105 per segment; percentage of each core, including background/kernel work.

| Core | Before mean | After mean | Change (percentage points) | Before max | After max |
|---|---:|---:|---:|---:|---:|
| 0 | 70.51% | 69.47% | -1.05 | 91% | 86% |
| 1 | 68.38% | 67.50% | -0.88 | 91% | 91% |
| 2 | 68.36% | 66.17% | -2.19 | 93% | 88% |
| 3 | 81.20% | 81.02% | -0.18 | 91% | 88% |
| 4 | 55.72% | 53.50% | -2.23 | 100% | 100% |
| 5 | 77.66% | 70.86% | -6.80 | 88% | 84% |
| 6 | 85.18% | 88.26% | +3.08 | 93% | 93% |
| 7 | 51.45% | 51.36% | -0.09 | 60% | 59% |

Core5 mean headroom rises 22.34% -> 29.14% (6.80 percentage points). Neither segment has a core5 sample at or above 90%; this car was not saturating core5 before the change. Core6 remains busy and increases; do not attribute that increase to native kernels from unmatched drives.

## Per-process measurements

Use complete runtimeTiming intervals contained in the segment, weighting metric means by their own sample counts. Scheduler CPU/runqueue wait percentages use summed time divided by summed interval seconds. procLog cumulative CPU independently corroborates process CPU figures.

| Process | CPU per frame/loop, before -> after | Reduction | Scheduler CPU, before -> after | Work elapsed, before -> after |
|---|---:|---:|---:|---:|
| card | 6.019 -> 5.800 ms | 3.6% | 60.97% -> 58.78% | 5.943 -> 5.731 ms |
| radard | 7.698 -> 4.729 ms | 38.6% | 16.45% -> 10.61% | 18.251 -> 7.883 ms |
| radarcan | 1.168 -> 1.023 ms | 12.4% | 24.19% -> 21.40% | 1.176 -> 1.030 ms |
| plannerd | 3.834 -> 4.086 ms | -6.6% | 18.54% -> 19.50% | 3.860 -> 4.106 ms |

- radard elapsed work falls 56.8%, including preemption/wait within its update. This is not the CPU-time reduction (38.6%). Maximum work falls 38.473 -> 26.892 ms.
- radard runnable wait falls 41.61% -> 35.45% of wall time. That metric spans the entire scheduling interval, not only the update/publish section; do not derive one directly from the other.
- card update stage: 2.138 -> 2.019 ms (5.6% lower); apply stage: 1.891 -> 1.887 ms (essentially unchanged). Overall card CPU improves only 3.6% and still takes about 59% of core5. Earlier PC kernel/interface speedups are not measured whole-device reductions.
- radarcan uses the shared CAN parser, so it can also benefit from the native loops, but its 12.4% CPU/loop reduction includes workload differences.

## Rates and faults

- Both segments: radarState, liveTracks, modelV2 and all three camera streams have 1,200 messages, about 20 Hz. card produces 5,999 carState messages, about 100 Hz.
- All radarState/liveTracks/carState/modelV2/pose envelopes valid in both. No CAN-invalid, CAN timeout, steering faults, decoded radar faults, pose input/sensor failures or unexpected process-down entries.
- No radarFault/commIssue/commIssueAvgFreq onroad event in either segment. Model diagnostics report zero dropped frames.
- Timing tails do not improve uniformly: radarState publication maximum gap 74.481 -> 77.430 ms; p95 65.237 -> 63.338 ms. Do not claim all latency is fixed.
- Before contains cruiseMismatch events (32 logged event-list entries); after does not. They are not radar/CAN parser faults, and different engagement/driver behavior prevents attributing their disappearance to this optimization.

## Workload confounding and same-input replay

- Radar objects: 27.13 -> 20.01 average; front 20.06 -> 17.59, corner235 7.04 -> 2.26. Fewer corner targets can substantially reduce radar processing.
- Mean vehicle speed: 11.60 -> 28.04 km/h. Enabled-state share changes from approximately 25% to 74%, plus about 12% overriding after.
- Core5 temperature: 75.6 -> 67.1 C average; thermal status yellow -> green. Temperature/environment and CPU clocks are not controlled. Do not claim native code caused the approximately 8 C decrease; recorded data here does not establish clock/throttling equivalence.
- Therefore 38.6% lower radard CPU per frame is the observed drive-to-drive change, not an isolated causal effect estimate.
- Replayed each complete log separately through Python and native DPathRadarController on the PC, recorded mode 1 and corner preference enabled, same latest-preceding input schedule for each variant. Three runs per backend, discard first 30 timing samples.

| Input log | Python mean | Native mean | Reduction | Complete output equality |
|---|---:|---:|---:|---:|
| before | 1.112 ms | 0.764 ms | 31.3% | 1,200 / 1,200 frames |
| after | 0.953 ms | 0.662 ms | 30.5% | 1,200 / 1,200 frames |

All radar output dataclass fields compare exactly for both reconstructed variants, 2,400 frames total. This supports algorithmic equivalence and approximately 30-31% intrinsic radar compute improvement on this PC. It is not vehicle-response validation, an exact subscriber-timing replay or a measured isolated ARM speedup.

## Conclusion

Native execution is confirmed on the device. Radar compute and shared-core headroom improve substantially; whole-card improvement is modest. Both logs were already healthy at 20 Hz, so this is validation of activation and improved margin, not proof of fixing a reproduced radar fault. The original EV9 failure was mode 3 with saturated core5; this is an Ioniq 5 PE mode-1 comparison with no saturation. Keep the experiment separate until the originally overloaded case is measured. No code, settings, CPU placement or production deployment changed in this analysis.

## Reproduction evidence

Private scripts, metric summaries, reduced time series and replay results: `.analysis/archive/2026-10-02/native-cpu-ioniq5-ab/`. Full rlogs/settings are not committed. Full openpilot cereal schema was used.

- Before rlog SHA256: `d76183792b0dda1987537111f08dfa09145b0d32f63d8889de39e0f0f687b29b`
- After rlog SHA256: `1b0b604de2904f88dc9c0399b5812f5454e1ced5834ffae801f03fd3d482bd6e`


## Follow-up: why EV9 saturates core5 but Ioniq 5 does not

Compare the EV9 incident window 30?54 s to the complete pre-native Ioniq 5
segment. Both actually use clean abe1a232 on C4/mici and the same OS. This removes
native vs Python and source revision as explanations for their difference.

| Measurement | EV9 | Ioniq 5, before native |
|---|---:|---:|
| card main-thread scheduler CPU | 62.31% | about 61% |
| radard main-thread scheduler CPU | 36.89% | about 16.5% |
| card CPU/update | 6.153 ms | about 6.02 ms |
| radard CPU/update | 22.553 ms | about 7.70 ms |
| radard output rate | 16.11 Hz | 20 Hz |
| core5 usage | 100% | 77.66% |
| CAN receive frames/s, sources 0+1+2 | 5,440.7 | 5,418.5 |
| Raw radar objects/frame | 31.65 | 27.13 |
| Core5 temperature mean | 60.0 C | 75.6 C |

No additional heavy core5 process is found in the EV9 procLog samples. Process
CPU deltas independently show card 62.50% plus radard 36.91%, already explaining
almost all of the busy core. Both main threads are sampled on core5, with the
same -54/-52 priorities and 4/3 total process threads respectively. EV9 rsyslogd
adds about 0.09% process CPU; sampled core5 kernel helpers and a separate idle
pandad-named process have zero measured CPU delta. The active pandad is on core3,
radarcan/plannerd on core4, controlsd/selfdrived/UI on core6, model/DM on core7.

procLog processor is a sampled last CPU, not a complete per-thread/IRQ trace;
process CPU aggregates must not be assigned wholesale to one core for migrating
processes. On Ioniq 5, several background processes occasionally sample core5,
which has spare capacity. Their presence does not establish a new EV9 load.
The measured two real-time consumers suffice to explain EV9 saturation.

### Controlled replay to distinguish settings from work per target

Both recorded-input schedules were replayed with native kernels disabled in the
same experimental checkout. EV9 profiling uses model timestamps in 30?54 s
(480 frames); Ioniq 5 uses 0?60 s (1,198 frames, exposure-time boundary excludes
two publications). Inputs use the latest preceding radar/state/pose data. This
is deterministic computational analysis, not exact reconstruction of the EV9's
slow live subscriber schedule.

| Per model frame | EV9 | Ioniq 5 |
|---|---:|---:|
| Targets entering trajectory statistics | 22.82 | 3.48 |
| Median-slope calls | 71.91 | 11.00 |
| Historical pairs evaluated (dt >= 0.10 s) | 6,031 | 715 |
| Mean samples per slope call | 13.78 | 11.61 |

Target count is derived from the one global_path_s slope call per processed
trajectory point, not an estimate of unique physical vehicles. Median/history
counts are instrumented counters, not cProfile timings (instrumentation adds
its own runtime). Raw object count is only about 17% higher, but the expensive
trajectory workload has about 6.6 times as many targets and 8.4 times as many
historical pairs.

Production selection explains why raw points are not an adequate cost metric:
corner-preferred processing adds appropriate unmatched front targets with
abs(vRel) <= 5 m/s, then trajectory processing checks measurement/range and
path proximity (-5 to 80 m longitudinal and abs(dPath) <= 5.4 m). Eligible points
build histories and perform pairwise slope statistics. A history with n samples
can have up to n(n-1)/2 pairs, before the minimum time separation filter. This is
pairs of observations of one target, not all pairs of different radar objects.

Mode-only A/B, two unprofiled PC runs per variant, keeping each log's input fixed:
EV9 mode1 3.743 ms vs mode3 3.753 ms; Ioniq 5 mode1 1.098 vs mode3 1.099 ms.
These differences are within small timing variation. EnableRadarTracks=3 vs 1
is a configuration difference, but it is not supported as the main cause of
this roughly threefold per-update CPU cost. Do not recommend switching modes
as a demonstrated CPU remedy. No production code/settings were changed.

At its measured 22.55 ms/update, EV9 needs about 45% of one core to deliver
20 Hz, on top of card's 62%. Ioniq 5 needs about 15.4% for the same 20 Hz,
so its combined demand fits. EV9's higher-priority card remains near 100 Hz;
radard consumes the remainder and drops to about 16 Hz. No additional busy
process is required to explain this difference.

Evidence: `.analysis/archive/2026-10-02/native-cpu-core5-vehicles/`.


## Follow-up: EV9 trajectory prefilter opportunities

Instrumented original mode-3 EV9 replay, model timestamps 30?54 s (480 frames),
using the native trial checkout. No production code/settings changed.

| Stage or property | Mean points per frame |
|---|---:|
| Controller motion input | 25.97 |
| Corner input | 4.69 |
| Unmatched front input | 21.27 |
| Rejected by longitudinal bounds | 1.98 |
| Rejected by path proximity | 1.17 |
| Full trajectory analysis | 22.82 |
| Analysed front points | 18.12 |
| Analysed corner points | 4.69 |
| Above 45 m, among analysed points | 4.18 |
| Estimated vLead <= 0.5 m/s, among analysed points | 8.65 |
| Not satisfying 0.8 < dRel <= 45 and vLead > 0.5 | 12.62 |
| Satisfying those basic current cut-in conditions | 10.20 |

Property rows overlap; do not add 4.18 and 8.65 as independent exclusions.
Estimated abs(vLead)<=0.5 accounts for 8.48 points and vLead<-0.5 for 0.17;
these are estimated kinematics, not physical object-class ground truth.

The controller's corner-preferred mode deliberately includes unmatched front
points with abs(vRel)<=5 m/s as fallback. The detector then admits front points
with dRel>0.8 even without measured=true, uses a broad -5..80 m range and
abs(dPath)<=5.4 m corridor, and computes motion statistics before most of the
final eligibility checks. Low-speed driving lets near-stationary returns pass
abs(vRel)<=5, so that check is not a moving-object filter. The front fallback is
not restricted to only currently vision-matched points at this early stage.

All analysed front points in this window lack the detector's current vision
support, but that alone is not sufficient reason to discard them: the policy
also permits historical/corroborated support. Final output has no raw cut-in,
confirmed cut-in, control-eligible cut-in or predeceleration risk in the analysed
window, but about 3.51 current_path estimates per frame. These outputs still
serve downstream lead/display bookkeeping; lack of a cut-in event does not mean
all calculations are universally unnecessary.

The 12.62 points (55.3% of trajectory targets) failing simple instantaneous
cut-in criteria identify potential heavy-work reduction, not a proven equivalent
filter or a 55% total CPU saving. Existing current_path is not capped at 45 m;
latched cut-in/risk state and continuity/outside history can outlive current
eligibility. Blindly moving a continue before those updates changes semantics.

Recommended next design is two stages: preserve cheap observation/history,
identity/association, expiry and outside-state updates for the existing broad
scope; run full pairwise motion statistics for potentially relevant targets and
any state requiring continued control/hold/other output calculation. Reuse or
explicitly define cheap outputs only after auditing their consumers. Candidate
activation must see the retained preceding history. Check stopped-to-moving,
45 m boundary crossing, vision dropout/reacquisition, active latches, corner
handoffs and current_path/lead consumers in differential replay. Do not assume
bit-identical complete output from this proposal before implementation/testing.
Avoid an arbitrary nearest-N cap or discarding targets simply for missing vision.

Evidence: `.analysis/archive/2026-10-02/radar-prefilter-review/`.
