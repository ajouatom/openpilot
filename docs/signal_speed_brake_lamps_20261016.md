# Signal processing overhead and opt-in lead brake-lamp observation

The user requested faster signal recognition (20–50 Hz target) and an ON/OFF
setting for lead brake lamps. Current road-camera input is 20 fps. Repeating
the same image is not a new detection. Sustained 20 Hz signal recognition has
NOT been demonstrated by this change; 50 Hz new-image detection requires a
different camera pipeline.

## Signal changes

- Read padded NV12 planes directly with `cvtColorTwoPlane`, removing the packed
  intermediate allocation/copy. RGB remains independently owned. Preserve the
  existing frame-ID reuse check and all input/result freshness gates.
- CPU duty is measured over total elapsed iteration time. The extra delay is
  `max(0, 50ms-wall, 2*cpu-wall)`, instead of sleeping a full CPU duration even
  after preemption/I/O already made the thread wait. The mathematical budget
  remains at most 50% of one CPU over work plus delay; no catch-up or duplicate
  frame processing. Same function is shared with the eligible neural worker.
- Signal discovery, geometry, colors, identities, green/red confirmations,
  planner/control consumers, model artifacts, CPU affinity and DM/GPU policy
  are unchanged. Recognition failures in the previous drive remain unresolved.

## Measurements and rejected experiments

Nine recent retained segments have recorded rates 6.38–11.51 Hz and per-segment
median capture-to-result age 93.67–122.57 ms. These are BEFORE this change.
Current vehicle camera/model/DM read-only snapshots are approximately 20 Hz;
short samples around 19.99 Hz are not a strict >=20 Hz proof.

Desktop 300-iteration conversion medians: packed 1.0684 ms, two-plane 0.6848 ms;
random padded input produces identical pixels and output survives source mutation.
Native parked camera-frame A/B/A/B conversion CPU medians: old 5.2198/5.2460 ms,
new 4.6323/4.5946 ms (about 11–12% less conversion CPU). Native wall medians vary
6.14/6.82 ms old versus 7.04/6.00 ms new due to scheduling. This is a conversion
microbenchmark, NOT end-to-end signal FPS or proof of improved control.

A global color-integral prefilter was slower on desktop and was discarded.
Native low-priority recorded-image trials on cores 4/5 reduce computation but
show significant scheduling waits; no CPU affinity change was adopted. Those
trials used four alternating saved images and are not tracking-accuracy tests.
Only private /tmp diagnostic scripts/images ran on the parked vehicle; no new
observer code, setting, process restart or control change was installed.

## Brake lamps

`LeadBrakeLights` is a persistent boolean, default OFF. Signal mode 0 dominates.
OFF creates no additional subscriber or detector. ON shares the already-converted
RGB after signal publication, attempts at most 10 Hz, and admits crop work only
with at least 5 ms left in a 50 ms iteration. A result taking over 5 ms is
discarded and backed off for 500 ms. These are admission/post-work limits, not
a preemptive hard deadline: a single overrun can still consume time.

Require current valid radar/model/car context and bounded calibrated geometry,
radar/vision agreement, 3–35 m distance, and no detected turn. Project a bounded
central upper rear search area, then require one bright horizontal red candidate
and three increasing samples over at least 150 ms on the same lead. The camera
geometry is the 1344x760 C4 OS04C10 road input accepted by this worker. This is a
rule-based candidate experiment, not a learned or complete brake-state detector.

`BRAKE?` means candidate; `BRK ?` means unknown. Never infer OFF or grant go from
absence. Separate small display transport has capture expiry and camera matching;
no previous-color hold and no planner reads. It is connected to both observer
backend loops; a slow neural iteration will skip it under the same work budget.

The earlier three-encounter/36-frame development set had 10 visually ON and 16
OFF frames. A repeated-still spatial screen of the new candidate rules retains
8/10 ON, produces no candidates on those 16 OFF, rejects the turning ON frame
and misses the 17.22 m ON frame. The earlier two tail-lamp false proposals are
excluded. Repeating stills is explicitly NOT temporal/encounter-independent
accuracy validation. All vehicles are SUVs; other designs, LED aliasing, rain,
distance, real-time load and physical display behavior remain unvalidated.

## Checks and publication

693 focused desktop tests pass with a default-OFF Params test double because
Windows lacks the native Params extension. No detector/planner is substituted;
parameter/backend tests supply their own doubles. Existing NumPy deprecation
warnings remain. New checks cover exact NV12 pixels and ownership, elapsed-time
CPU budget, duplicate/gapped observations, lead replacement, turns/invalid
context, mode-0/OFF inactivity, busy-work rejection and stale/wrong-camera display.
Native conversion was also checked against an actual camera frame.

Keep both maintained branches single-parent. Publication is separate from
vehicle installation, a native Params rebuild, end-to-end 20 Hz/DM verification,
and road validation. Raw images/logs stay in the private dated analysis folders.
