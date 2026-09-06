# Near-field outer-body CUT-IN review (2026-09-06)

## Incident and correction

In Carnival route `00000182--5deb296c85--59`, the left silver sedan stays in
its lane while ego passes it. At 50.06 s, corner 7856 was nevertheless approved
as CUT-IN/leadTwo and pre-deceleration. The right black SUV is a different target.
Front 34 corroborates existence, but returns a different longitudinal body
position. The original predictor extrapolated the corner's inward drift toward
the ego corridor and treated the front association as sufficient corroboration.

The correction vetoes a predicted outer-body entry when all of these hold:

- The corner is within 12 m and closing (`vRel < -0.1 m/s`).
- Neither current path overlap nor visual corroboration supports entry.
- The paired front remains outside the 2.15 m near-field corridor and differs
  longitudinally by more than 1.25 m.
- Recent ego yaw magnitude reaches 0.020 rad/s, so apparent facet motion during
  the pass is ambiguous.

The veto immediately clears CUT-IN and pre-deceleration latches and persists
through a short missing front association (0.35 s). Actual path overlap, a front
return entering the near corridor, or visual corroboration resolves it. A body
offset on a straight road alone does not trigger the veto; the existing early
Ioniq 5 and K8 cut-ins retain their required detection times.

The regression case checks **48.0–53.5 s**, including the later 52 s recurrence,
and forbids CUT-IN, pre-deceleration, L1, and L2 for corner 7856/front 34. It
does not merely inspect the screenshot timestamp.

## Validation integrity

The validator now reports a scoped label as **unverified** when its measured
target is absent from the specified sensor/window. Such a label is excluded
from confusion-matrix counts and strict validation fails rather than calling
the missing target a true negative. Spatial-only labels must actually match
their position bounds. Existing labels and deadlines are preserved.

The review-video exporter derives cadence from full-schema `qRoadEncodeIdx`
exposure timestamps, trims sequential decoder frame indexes, verifies the
emitted frame count, and retains the exact camera-time mapping. This fixes a
review camera stream reported as 120 FPS despite having 1200 actual frames at
20 FPS. It does not change vehicle camera processing or model inference.

Run the exporter with a JSON array of `qRoadEncodeIdx` messages:

```text
python -m openpilot.selfdrive.carrot.radar.tools.radar_review_media --camera qcamera.ts --index camera-index.json --start 12.75 --end 21.90 --output review.mp4
```

Start/end are camera-relative exposure seconds, not arbitrary log-service
timestamps. The accompanying `.frames.json` maps clip frames to source exposure
times; a review player must apply its known camera/log time offset.

## Verification scope and unresolved work

Validation uses recorded model outputs and production lead-selection code;
it is not a closed-loop road test. The local report retains full mode 2/3
results, before/after sensitivity sweeps, and original user verdicts.

- 475 focused tests pass.
- Both modes replay all 87 logs / 474 labels, with no new failed expectations
  relative to the previous stationary-validation baseline and no failed
  applicable clear/pre-deceleration cases.
- Each mode retains 9 existing failed expectations; 208 labels have absent
  measured targets and are reported separately as unverified.
- The new Carnival case has no CUT-IN, pre-deceleration, L1, or L2 output for
  the specified target at any sensitivity from 1 through 5 in either mode.
- All 21 regenerated clips have the expected frame counts; their first,
  middle, and last frames match the corresponding originals (63 comparisons).

The old 38 failures per mode included 29 missing legacy corner IDs, so they
cannot be described as 38 independently verified missed vehicles. Missing
negative targets likewise cannot substantiate a claim of globally zero false
positives. The new report separates unverified input, failed expectations,
and applicable clear cases.

User-review scene 01's white SUV is C1079/F48; F38/C1069 is a separate primary
vehicle. The previous gallery title identifying C1079 as a black sedan was
incorrect. Its 13.55 s deadline and scene 09's 16.75 s lead-selection gap remain
unresolved. The user's saved notes have not been changed into passing labels.

A yaw-coordinate sign correction was also prototyped. Although it changed
the reported lateral velocity substantially, replay produced new outputs in
four existing clear windows. It is **not included** in this change; its
coordinate, road-curve, and label-scope implications need separate validation.

No setting is added or redefined, and no model artifact is changed. This is a
bounded false-entry correction plus offline review/validation tooling.
