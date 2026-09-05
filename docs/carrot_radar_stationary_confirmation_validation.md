# Stationary front-radar confirmation review — 2026-09-05

The production controller must not start a stationary front-radar L1 from
observation age, long range, or track quality alone. It requires corresponding
vision confirmation or a continuously matching measured central corner object.
An already confirmed object's physical continuity may retain that identity;
it cannot authorize an unrelated return after an ID or position discontinuity.

## Reproduction and cause

The two user-reviewed Carnival logs are `00000175--f97562d8d2--35` and
`00000175--f97562d8d2--42`. Their recorded code is `4314dff8e4`, with
`EnableRadarTracks=3` and `EnableCornerRadar=2`.

Both full `rlog.zst` files were decoded with the OpenPilot cereal schema.
`carState`, `carControl`, `radarState`, `modelV2`, `liveTracks`, and
`longitudinalPlan` were present and inspected. Time below is relative to the
replay frame clock, not the boot-time `initData` timestamp. Original qcamera
frames show an unobstructed ego lane at the five reported occurrences.

Commit `50765b3860` introduced an uncorroborated front-only stationary
acquisition path. A high-quality return observed for at least one second,
initially at least 60 m away, could complete a further 0.50-second dwell and
become L1 without corresponding vision or corner radar. Persistent stationary
reflections satisfied these conditions. Vision probability stayed below 0.02
throughout the false L1 runs.

| Segment | False L1 interval (s) | Front track IDs | Before frames | After frames |
|---|---|---|---:|---:|
| 175 / 35 | 32.992–34.692 | 37, 58 | 35 | 0 |
| 175 / 35 | 42.541–44.140 | 48 | 33 | 0 |
| 175 / 35 | 45.990–46.441 | 49 | 10 | 0 |
| 175 / 35 | 48.549–50.789 | 55 | 46 | 0 |
| 175 / 42 | 8.203–13.147 | 61 | 100 | 0 |

The first interval contains a short ID 37 selection immediately before ID 58.
The complete two-log replay removes all 224 false L1 frames with both
`EnableRadarTracks=2` and the recorded value `3`. A separate one-frame vision
lead at 36.851 s in segment 42 remains unchanged.

Recorded control impact must be distinguished from recalculated lead roles:

- Segment 35, 32.8–37.0 s: `carControl.actuators.accel` reaches -4.00 m/s²,
  and `carState.aEgo` reaches -3.35 m/s². No brake-pedal press is logged in
  that window.
- Segment 42, 8.0–15.0 s: the recorded acceleration request reaches
  -3.32 m/s² and measured acceleration reaches -1.56 m/s². The brake pedal
  is pressed during part of this longer window, so all of that measured
  deceleration cannot be attributed to controller output alone.

The original acceleration traces do not change when L1 is replayed. This
review verifies corrected lead selection, not a counterfactual MPC trajectory
or a new on-road drive.

## Fix and focused checks

The front-only acquisition exception and its observation-age/distance gates
are removed. A no-vision front candidate must match a measured central corner
object in range, raw lateral position, and speed throughout confirmation.
Corner detection elsewhere, a one-frame match, an unmeasured return, and
incomplete confirmation do not authorize it. Configured SCC behavior and
confirmed-object retention continue through their existing paths.

The maintained case file now explicitly forbids the five reported objects in
L1 and L2 and checks their pre-deceleration output. All five fail against the
original code and pass against the correction. CUT-IN-only clear labels would
not have detected these L1 failures; these two logs were absent from the old
maintained corpus.

One earlier Ioniq 5 expectation depended on the removed exception. In
`00000e7d--929c32ef9a--3`, front track 52 has its first corresponding vision
frame at 15.049 s and completes normal confirmation at 15.388 s. No matching
corner object corroborates that earlier acquisition. The former 15.15 s
deadline is replaced with an explicit no-admission window through 15.24 s and
a confirmed-detection deadline of 15.45 s. This intentional delay is recorded
instead of restoring the unsupported exception to satisfy the old deadline.

Subsequent review found that the 15.388 s acquisition still borrowed time
across interrupted vision support. It is not a completed fresh confirmation.
The follow-up [interrupted-vision analysis](carrot_radar_interrupted_vision_validation.md)
records the corrected 15.540 s acquisition, its 0.152 s delay, and the
replacement expectations. The figures below describe this earlier revision.

Focused tests cover observation age across the stationary speed band, all
three front-track settings, mismatched/absent/intermittent corner evidence,
confirmation after a delayed corner arrival, retention after confirmation,
and rejection of a reused identity. The radar, replay, CUT-IN, and focused
longitudinal test selection passes: **467 tests**.

## Full corpus comparison and limits

Both implementations were replayed against the same **83 full logs and 464
case/trajectory labels**, with no missing logs. The production controller,
not recorded lead roles, supplies the evaluated outputs.

| Result | Original `4314dff8e4` | Corrected |
|---|---:|---:|
| Failed expectations | 44 | 38 |
| Newly failed expectations | — | 0 |
| Newly added no-admission cases satisfied | 0 / 6 | 6 / 6 |
| Pre-deceleration expectation failures | 0 | 0 |

The remaining 38 failures are unchanged: 37 manual trajectory detection labels
and the maintained `gv80-218-2-adjacent-right-14` detection case. They have not
been removed, relabelled, or treated as passing. Consequently the full strict
validator still returns a failing exit status. The exact comparison and
remaining IDs are in
[the machine-readable report](carrot_radar_stationary_confirmation_validation.json).

Run the maintained checks with:

```powershell
.\.venv\Scripts\python.exe openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --case carnival-175 --shadow-only --strict-shadow --strict-predecel
.\.venv\Scripts\python.exe openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --case ioniq5-e7d --shadow-only --strict-shadow --strict-predecel
.\.venv\Scripts\python.exe openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --shadow-only --strict-shadow --strict-predecel --report replay-report.json
```

The code contains no route-ID or vehicle-specific detection exception.
