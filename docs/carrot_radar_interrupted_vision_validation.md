# Interrupted stationary confirmation, 2026-09-05

In Carnival segment `00000178--52f7f496c5--62`, a white vehicle briefly
appears while crossing the view at a roundabout. The full rlog and qcamera
were inspected. The log runs `f8ce05da2e`, radar mode 3 and corner mode 2.
Replay times below use the service/frame clock, not initData boot time.

Vision probability exceeds 0.40 in two separated runs:

| Interval (s) | Frames | Approximate span including one frame |
|---|---:|---:|
| 17.397–17.451 | 2 | 0.10 s |
| 17.801–18.150 | 8 | 0.40 s |

Probability exceeds 0.80 for only two frames, 18.002–18.054 s. At the
screenshot time, 18.097 s, vision estimates 12.38 m and 5.97 m/s, while
selected front 56 is at 16.00 m and 0.09 m/s. Video shows the moving white
vehicle passing sideways and becoming occluded. Its brief lead probability
does not establish that a nearby stationary radar reflection is that vehicle.
This review does not claim to identify the model's internal reason for every
probability change or artificially extend its output.

The confirmation bug is independent of model inference: when a front-only
pending stationary object lost vision beyond its permitted short hold, only
the supported-frame count reset. Its old confirmation start time remained.
Front 36 therefore retained a start time of 17.397 s through a long support
interruption. Fresh support at 17.851, 17.901, and 17.954 s supplied three
frames, and the old elapsed time immediately satisfied the nominal
0.25-second dwell. Front 56 subsequently inherited the selected role.

The correction resets the pending identity and the start time together when
that support gap expires. The existing measured, physically continuous
0.10-second hold remains available. Renewed evidence must complete a fresh
0.25-second dwell as well as three supported frames. Confirmation does not
borrow time from an earlier interrupted glimpse.

| Implementation | False L1 frames for 36/56, 17.3–19.4 s |
|---|---:|
| Recorded `f8ce05da2e` replay | 27 |
| Previous correction `5de19eabce` | 5 |
| This correction | 0 |

Both radar modes 2 and 3 give those counts. The prior correction already
stopped unrelated corner evidence from extending this false hold; the new
change removes the remaining premature acquisition. The original log's
planner requests as much as -2.48 m/s² and carControl requests -2.88 m/s²
around 19.15 s while fast lead ID 56 is active. Those recorded acceleration
traces are not counterfactual outputs of the corrected lead replay.

The six new mode/burst regression tests reject interrupted short glimpses and
still acquire after genuinely renewed confirmation. Existing short-gap and
real stopped-vehicle tests remain in the focused selection: **493 tests pass**.

The complete comparison covers **85 logs and 467 labels in each of modes 2
and 3**. With existing expectations unchanged, failures remain **39 to 39**:
this incident is fixed, but `ioniq5-e7d-3-closer-stationary-front-52` newly
misses its 15.45 s detection deadline. That regression is not omitted from
the comparison.

In Ioniq 5 segment `00000e7d--929c32ef9a--3`, support starts at 15.049 s,
expires at 15.150 s, and starts again at 15.249 s. The previous code admits
the real slow vehicle at 15.388 s, using only 0.139 s of renewed dwell.
The correction admits it at 15.540 s, after 0.291 s and six supported
frames. The delay is **0.152 s**; radar distance changes from about 95.37 m
to 92.44 m. No corresponding corner return supports an earlier admission.
This is a real cost of requiring fresh evidence, not a missed vehicle or
proof that braking performance is unchanged.

The maintained no-admission window is consequently extended from 15.24 s
through 15.50 s, and the positive deadline moves from 15.45 s to 15.60 s.
Both the original-criteria failure and this explicit policy change are
recorded in the report. These revised criteria require fresh confirmation
and then timely detection; they do not require the interrupted-timer bug.

Under those revised criteria, failures fall from **40 to 38** in each mode,
with no further new failures, no missing logs, and no pre-deceleration
failures. The 38 previously known failures remain: 37 manual trajectory
detection labels plus `gv80-218-2-adjacent-right-14` in mode 2 or
`ioniq9-234-6-scc-corroborated-early-l2` in mode 3. The full strict validator
still returns status 1. Exact IDs, original-criteria results, and incident
checks are in the
[comparison report](carrot_radar_interrupted_vision_validation.json).

```powershell
.\.venv\Scripts\python.exe openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --case carnival-178-62 --shadow-only --strict-shadow --strict-predecel
.\.venv\Scripts\python.exe openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --shadow-only --strict-shadow --strict-predecel --report replay-report.json
```
