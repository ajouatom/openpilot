# SCC longitudinal lead continuity (2026-10-01)

## Evidence

The reported Casper EV segment `00001e75--ace5ac2325--9` was recorded on
`1dfe513b1606`, with `EnableRadarTracks=0`. Full cereal decoding finds 1,200
model, radarState and liveTracks messages. All 1,155 measured radar points are
SCC source/track 0, and every SCC `yRel` is zero. The remaining 45 liveTracks
messages have no measured SCC point. There are no raw front-radar points.

The 1,199-frame production-controller replay before this change loses leadOne
on 30 frames in the vehicle's mode 0. Every one of these frames has both a
measured SCC object and vision probability at least 0.40. SCC lateral likelihood,
lateral-distance and path gates can reject the OEM object; the final visual
fallback can then fail its separate 1 m dPath gate despite high probability.
Thus this window is not explained by an absent SCC object or low vision
probability. The 45 genuine SCC-absent replay frames already retain vision in
this log; curved/off-path dropout cases require separate synthetic coverage.

The public reviewer had an additional discrepancy: its deployed fingerprint
`4050e3b0b59ceca9fea3` always replayed mode 2, including this mode-0 log. That
replay loses leadOne on 42 frames. The screenshot alone cannot distinguish a
recorded decision from this different-policy recalculation.

## Correction

- Modes 0 and -1 always use the current measured SCC longitudinal object.
  No vision match or SCC lateral/path gate is required. The existing input-age,
  measured-object and valid-model-path requirements remain active.
- If SCC is absent, these SCC-only modes use model lead zero at the existing
  0.40 acquisition probability, with the existing above-0.35, ten-frame hold.
  There is no additional 1 m path gate for this SCC-only fallback. Low-probability
  new hypotheses and expired radar measurements do not create a lead.
- All SCC vision associations omit SCC lateral likelihood and lateral/path
  gates. Mode 2 retains the configured low-speed eligibility and longitudinal
  matching requirements; mode 1 still excludes SCC, and mode 3 retains its
  front-first then unconditional-SCC policy.
- SCC cannot provide geometric stationary/moving path-occupancy or cross-sensor
  evidence. In particular, a fixed zero must not prove a central stationary
  object in mode 2. Its separate L2 path still requires independent vision or
  corner support and the existing confirmation dwell. Real front/corner geometry
  remains subject to its existing checks.
- SCC lead output uses zero `yRel`, `dPath` and `vLat` as unused fields. These
  zeros are not measurements of a vehicle's lane-center position. Raw receive
  points and recorded logs remain unchanged.
- Web replay reads the recorded source setting from initData and vehicle policy
  from carParams, including metadata emitted later in a segment. Mode 0 is
  preserved as zero. Historical Hyundai logs without a valid setting retain
  analysis mode 2; non-Hyundai logs follow radar availability. The sensor selector
  does not silently change the source policy. The code fingerprint invalidates
  previous cached calculations.

## Validation

- New SCC-policy tests reproduce 13 failures against the previous controller;
  all 27 pass with the correction. They cover left/right curves, release and
  reacquisition, lateral-coordinate invariance, probability hold expiry,
  unmeasured/stale input, excluded source modes, and longitudinal mismatch.
- Focused radar/controller/lead-dynamics/cut-in/out suite: 938 passed.
- Radar preprocessing: 291 passed. Planner integration: 254 passed.
- Route-vault tests: 36 passed, four platform-dependent local skips. Recorded
  policy and metadata tests cover missing/invalid settings and non-Hyundai rules.
- Wiki generation tests: 25 passed. Settings descriptions and both localized
  radar/settings guides are synchronized with the requested mode-0 behavior.
- Corrected mode-0 replay has no absent leadOne across all 1,199 frames: measured
  SCC owns 1,154 frames and vision owns the 45 SCC-absent frames. No last SCC
  object is fabricated through those gaps. The web adapter uses the same mode.
- Full maintained mode-2 corpus: 100 logs / 497 labelled items. The complete
  before/after result objects are identical. Existing expectation failures stay
  at 10, pre-deceleration failures at zero, and 208 items lack their labelled
  target input. This is a no-regression comparison, not an all-pass claim.

Recorded-input replay verifies lead selection, not closed-loop braking or a new
driving test. The SCC output represents the OEM-selected object; always using it
does not prove that every OEM detection is a vehicle. Front-mode visual path
gates are unchanged. Private logs, replay outputs and scripts are retained under
`.analysis/archive/2026-10-01/scc-lead/` and are excluded from Git.
