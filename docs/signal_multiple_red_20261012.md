# Multiple red housings in signal preparation — 2026-10-12

The user pointed out that an intersection often has two to four signal housings.
The previous moving preparation gate required exactly one qualified red housing,
so a second otherwise eligible red cancelled preparation. The stopped departure
guard already accepted multiple reds; this change targets moving preparation.

## Behavior

Accept one to four independently qualified red housings as a group hypothesis.
Each still needs fresh confirmed red, at least three tracker observations, a
measured housing (`detected`), quality >=0.5, red score >=0.4 and green score <0.08.
Lamp-core inferred boxes and template-only evidence cannot supply metric width.
Existing calibrated road/lane geometry and full plausible range inside25–140m
remain required for every candidate.

All members must have an overlapping plausible distance interval. Pairwise
housing widths must differ by no more than2x, vertical centers by no more than
twice the larger housing height, and boxes must not overlap. Duplicate identities,
more than four qualified reds, disjoint ranges or incompatible rows/scales cancel
preparation; no arbitrary winning subset is selected. These are conservative
experimental grouping checks, not measured signal depth or lane/direction proof.
Aligned signals for different movements can still be ambiguous.

Each track accumulates its own fresh-frame history. At least one currently
present member must have four distinct frames spanning0.6s. Multiple lamps in one
frame never count as multiple time observations. Candidate ordering has no effect.
A disappearing member loses its history; remaining independently confirmed reds
can sustain preparation. Newly appearing/reappearing identities must earn their
own confirmation. A session change, capture gap over0.25s or failed gate clears
confirmation. The observation deadline remains0.2s. Frame ordering watermarks
persist through resets in the same session so rejected/repeated frames cannot
restart confirmation. This does not extend control validity using display hold.

A fresh confirmed green measured housing with quality >=0.5 that passes the same
geometry screen vetoes moving preparation when reds are present (`mixed_colors`).
The initial revision applied this across the eligible search corridor. The later
[central-red priority revision](signal_central_red_priority_20261012.md) permits
an exception for temporally confirmed central red support with clearly separated
lateral greens, using calibrated ego-lane geometry. Missing/ambiguous association
still vetoes preparation; there is no majority vote. Unqualified/unseen green is
not proof of its absence.

The separate stopped-red guard is unchanged: another forward red prevents a
green release. It retains a latched hold through unknown/stale observations;
confirmed green at the anchored housing removes only this veto when no competing
forward red is present. The original planner still determines departure. Existing
driver override, mode selection and guard eligibility remain in effect.

Modes remain0 auxiliary off,1 judgment/display/alerts plus stopped departure veto,
2 additionally permits the existing gradual acceleration cap toward−0.5m/s².
Lead presence, turning, pedal intervention, model stopping, speed limits and
handover remain unchanged. `track_ids` records all current group members;
`track_id` identifies a confirmed member when available, otherwise the lowest ID.
The group `distance_range` is an intersection of hypotheses, never a stop line.

## Verification and limits

396 focused signal/tracker/transport/planner tests pass on Windows with native
Params storage substituted. Added cases cover1/2/3/4 reds, order independence,
independent confirmation, membership changes, actual road projection of four
housings, red-majority/green conflict, duplicates, disjoint geometry, capture gaps,
session changes and unchanged cap/handover behavior. Production module Ruff passes;
the pre-existing test file is not globally lint-clean. Existing NumPy shape
deprecation warnings remain.

Stored-observation screening was rerun without new camera inference:

* Earlier3348 observations: six individual reds meet appearance/history thresholds,
  zero pass full geometry; no observation has two qualified reds before geometry.
* Corrected recorded-cadence K8 replay10a1-3:201 observations,89 individual reds
  meet appearance/history thresholds, zero pass full geometry; no observation
  has two qualified reds before geometry. The89 are track observations, not
 89 distinct signals or independently labeled examples.

These recordings therefore do not demonstrate an additional preparation entry or
better braking from this change. They show that geometry/appearance eligibility
remains a limitation beyond the former exactly-one rule. Tests establish the
requested multiple-candidate behavior, not physical stopping, correct movement
assignment, detector recall or zero false red/green.

Private screen script/results are retained in
`.analysis/archive/2026-10-12-signal-group/`. No ONNX weights, vehicle installation
or restart are part of this correction. Work remains on `carrot-wip` with a
single-parent commit.
