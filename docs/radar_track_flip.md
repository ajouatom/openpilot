# Per-vehicle front-radar lateral orientation

`RadarTrackFlip` defaults to normal (`0`) and is manually enabled (`1`) only
after confirming a lateral mismatch for the individual vehicle. There is no
automatic model-name override. Compared Group1 logs showed opposite-side
front tracks on one Tucson and one Sportage, while a Staria's left passing
vehicle and right roadside returns had the expected signs. Another Sportage
parking-garage sample was inconclusive. These observations do not establish
upside-down mounting or a rule for every vehicle with the same fingerprint.

## Publication and replay

Radarcan reads the persistent setting once at onroad startup. After copying
the native RadarInterface result into its publication, it inverts `yRel` and
`yvRel` together on `frontRadar` points only. Native decoder/filter objects,
SCC and corner points, longitudinal values, validity, IDs and scheduling remain
unchanged. Every publication, including invalid input messages, records
`RadarData.radarTrackFlipped`; legacy messages decode this appended field as
false. A setting edit during a drive is deferred until the process restarts.

Replay defaults to the recorded orientation, without consulting current
vehicle settings or inferring orientation from a model name. The NAS reviewer
offers recorded/normal/flipped analysis options and keys its cache by this
choice. It preserves recorded leads as evidence of the original decision.
For Group3 raw-CAN identity matching, replay first restores native geometry
on a copy and applies the selected orientation after reconstruction. The
shared orientation helper and schema participate in source fingerprints and
the committed deployment bundle.

## Validation

- 785 focused radar/controller/replay tests passed, including 8 orientation
  tests covering both front slots, other sources, publication copy ownership,
  repeated application, recorded/override combinations and Group3 matching.
- 291 Group3/lead-filter tests, 28 vault tests and 25 Wiki tests passed.
- Five actual Group1 segments (four vehicles), totaling 5,668 replay frames,
  reproduced the pre-change default results exactly, excluding the newly
  added orientation metadata and source fingerprint. The forced-inversion
  comparison changed only front-point lateral position/velocity before lead
  recalculation; recorded leads, SCC/corner points and model inputs matched.
- The reported Tucson point 36 at approximately 30.6 seconds becomes
  `dRel=5.4 m, yRel=+2.7 m, yvRel=+0.09 m/s`. LeadOne identity remains 33;
  neither Tucson segment changes LeadOne identity in this comparison.
- The Linux radarcan IPC test covers both settings, input invalidation,
  recovery and setting latching; the built Linux runtime is required and this
  test is skipped on Windows. Windows cannot collect the native publication
  suite without `params_pyx`; publication-copy semantics are covered by the
  portable cereal test instead.

These are offline checks, not on-vehicle or closed-loop validation. Delivery
uses the Carrot Routes image workflow and NAS scheduled updater; verify the
intended commit and actual result-page recalculation, including both normal
and inverted variants, before considering server deployment complete.
