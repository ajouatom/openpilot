# Moving front radar and uncertain vision range, 2026-09-05

Carnival segment `00000178--52f7f496c5--53` loses front radar 35 as L1 at
17.3498 s even though the front-to-vision matcher selects it. This is a
post-match rejection, not missing front radar or a corner-to-vision match.
Full-cereal rlog analysis and qcamera inspection preceded this correction.
Times use the service/frame clock, not the initData boot timestamp.

| At 17.3498 s | Vision | Measured front 35 |
|---|---:|---:|
| Aligned range | 81.1195 m | 90.1811 m |
| Absolute speed | 10.9512 m/s | 16.8632 m/s |
| Aligned lateral position (yRel) | -0.1585 m | 0.5838 m |
| Model distance uncertainty, xStd | 9.7041 m | — |

The range difference is 9.0616 m. The moving matcher already holds identity
`frontRadar/35` and passes its range, lateral, and speed gates. The controller
then discards that match solely because it is more than 8 m farther than
central vision. With no SCC fallback, output becomes vision L1, ID -1.
An earlier single-frame experiment bypassed only that guard and immediately
produced front 35 in modes 2 and 3. Separately, corner 4983 agrees with front
35 to 1.80 m in range and 0.42 m/s in speed; it is supporting evidence, not
the primary association being diagnosed.

The correction preserves the ordinary 8 m guard for fresh or discontinuous
matches. An accepted moving front can use a bounded uncertainty allowance:

```
farther-than-vision limit = max(8 m, min(15 m, 0.15 × vision range, 1.5 × xStd))
```

Both vision and radar must indicate motion above 4 m/s and be within 1 m of
the model path. Vision probability must be at least 0.40. The same measured
front identity must continue from the preceding accepted match, with a
positive time interval no longer than 0.15 s. Radar range and lateral position
must stay within 2.5 m and 0.75 m of their previous velocity projections;
absolute speed may not jump by more than 3 m/s. A nonfinite or nonpositive
xStd cannot expand the guard. The current frame must still pass the primary
matcher; this is not a stale-output hold or a new confirmation delay.

Rejected matches, lost measurements/matches, invalid paths, and unconditional
SCC mode clear this continuity. Fresh identities do not inherit it. A nearer
front that matches better immediately replaces the previous front. Precise
nearer vision, a stopped visual target, and excessive range differences still
reject the farther moving radar. Stationary confirmation and its existing
corner-corroborated short mismatch hold are unchanged. Retained output uses
the measured radar distance and speed, without averaging with vision.

| Replay, both modes 2 and 3 | Before | After |
|---|---:|---:|
| L1 at the reported frame | Vision -1, 81.12 m | Front 35, 90.18 m |
| Front 35 frames, 17.20–19.10 s | 26 / 38 | 38 / 38 |
| False stationary front 32 frames, 12.8–16.5 s | 0 | 0 |
| Real stopped front 35 frames, 27.5–45.0 s | 349 | 349 |

The maintained corpus adds one continuous-front case. Its validator checks
the requested radar ID on **every** frame of the continuity window; merely
having some L1, including vision, does not pass. Existing case windows and
deadlines are unchanged. The two positive unit regressions fail on baseline
`c570e74f9b`; the focused radar, replay, alert, and planner suite passes **537
tests**, including 44 new cases. Wiki generator checks pass 25 tests.

The complete replay covers **85 logs and 468 labels** per mode, with 432
applicable shadow labels. Failures fall from **39 to 38** in each mode: the
new continuity case is fixed, with no newly failing labels, missing logs, or
pre-deceleration failures. The remaining failures are the previously known
37 manual trajectory detection labels and `gv80-218-2-adjacent-right-14` in
mode 2 or `ioniq9-234-6-scc-corroborated-early-l2` in mode 3. The strict full
validator still exits 1; this is not an all-pass claim. Exact results and IDs
are in the [comparison report](carrot_radar_moving_front_range_validation.json).

These are offline lead-controller results. Recorded acceleration traces were
not recomputed by a closed-loop planner, and this does not measure vehicle
braking performance or prove that the model's uncertainty is calibrated in
every scene. Two vehicles closer than the association limits can remain
ambiguous; the bounded allowance is not permission to trust radar blindly.

```powershell
.\.venv\Scripts\python.exe openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --case carnival-178-53 --shadow-only --strict-shadow --strict-predecel
.\.venv\Scripts\python.exe openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --shadow-only --enable-radar-tracks 3 --strict-shadow --strict-predecel --report replay-report.json
```

A Windows desktop timing check replays six logs in before/after/after/before
order, with 14,390 timed controller updates per version. Mean update time is
**0.5552 → 0.5515 ms**, p95 **1.1537 → 1.1501 ms**, and p99
**1.4513 → 1.3767 ms**. These short-run measurements do not establish a
device latency guarantee. They exclude decoding, model construction,
production live-track parsing, IPC, and publication.
