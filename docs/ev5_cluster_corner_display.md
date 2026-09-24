# EV5 cluster corner-object visibility (2026-09-24)

## Findings

Full-cereal decoding of EV5 segments `000000f3--f2596b8cdb--69` and
`000000f3--f2596b8cdb--97` separates two cases: missing display geometry and
present geometry carrying a hidden ccNC object type.

Both logs record `dc2d92cab69f03c00d06c67074b2cbb3578523f7` in
`initData.gitCommit`. The upload metadata supplied for segment 97 instead names
`2bc1f8c1`. They belong to the same running route; the upload's current checkout
commit is not evidence that the driving processes restarted into that version.
An update followed by no process restart explains this difference. Segment 97
must not be presented as vehicle validation of `2bc1f8c1`.

| Evidence | Segment 69 | Segment 97 |
| --- | --- | --- |
| Camera-bus `0x162` packets | 1,199 | 1,202 |
| LF/RF/LR/RR `0x162` DETECT | All zero | All zero |
| Nonzero corner distance | None | RF: 130 packets; RR: 156 packets |
| `0x1EA` corner information | All zero | Matching RF/RR geometry and DETECT=1 |
| Transmitted `0x162` packets | 1,200 | 1,201 |
| Transmitted front FF type | 4 in all 1,200 | 4 in 252; 0 in 949 |
| Transmitted corner types | All zero | All zero, despite RF/RR distances |
| `liveTracks` frames with `corner235` points | 1,200 / 1,200 | 1,202 / 1,202 |

Segment 97's transmitted RF distances are 36.4–38.8 m at approximately
11.44–12.19 seconds and 5.7–15.5 m at 20.40–26.05 seconds. RR distances are
0.2–18.5 m at 0.54–8.29 seconds. Times are relative to the first `can` event,
not the route's boot-time `initData` timestamp. The ccNC DETECT value remains
zero throughout those intervals, while `0x1EA` marks the objects visible.

These are not merely a loss of radar points or blocked CAN transmissions:
corner points remain in `liveTracks`, the relevant received/transmitted display
messages have valid checksums, and bus-128 transmit receipts contain the same
hidden corner types. Receipts establish bus transmission, not the dashboard's
actual rendering. No dashboard video is available in these uploads.

The boot Params in both logs are `EnableCornerRadar=2`, `HyundaiCameraSCC=2`,
`EnableRadarTracks=3`, `CanfdDebug=0`, and `PaddleMode=0`. The nearest preceding
September 23 settings snapshot and newest September 24 snapshot agree on these
values. The code also sends the existing `0x4B9` activation sequence; that does
not prove OEM object generation is active in every driving condition.

An earlier same-vehicle segment, `000000ed--42bf5941d2--22` on `c1849b50`, also
has nonzero geometry with zero ccNC corner DETECT. This condition predates the
September 23 restoration; that restoration is not established as its cause.

## Correction and scope

`1fbfe33156` incorrectly restored old box semantics (`1`) rather than the
requested car presentation. `2bc1f8c1` corrected the car type to `3` but still
required incoming DETECT > 0. That condition leaves the EV5's distance-bearing,
hidden objects invisible, even after restarting into that version.

For ccNC `0x162`, nonzero LF/RF/LR/RR distance now sets the corresponding DETECT
to `3` (gray car), including incoming DETECT=0. Zero-distance values retain
their incoming type. `0x1EA` keeps its separate existing normalization because
its enumerations differ. Front FF lead selection/presentation, received
geometry, scheduling, activation messages, and radar processing are unchanged.
Blinking and the old rear distance clamp remain removed.

This repairs the missing display request in segment 97. It cannot synthesize
side/rear display objects in segment 69, where both messages' geometry is zero.
Determining why the OEM display-object output is empty there requires separate
evidence; do not equate it with corner radar hardware being inactive.

## Validation

- 190 focused CCNC cluster, lead, and fault-filter tests pass with a Windows
  Params bootstrap. A distance-bearing DETECT=0 regression fails before the fix.
  Tests decode real CAN packets and check the DBC `GRAY_CAR` meaning.
- Repacking 7,202 recorded `0x162`/`0x1EA` transmit packets across the three
  segments changes only the intended ccNC corner types and checksum fields.
  Geometry, front-lead fields, and other signal values are preserved.
- Segment 69 changes no object types. Segment 97 changes 130 RF and 156 RR
  slots across 286 packets. The earlier segment changes 845 corner slots across
  320 packets. All repacked checksums validate; `0x1EA` object types stay intact.

These are offline CAN-output checks, not confirmation of physical dashboard
visibility. Local scripts, summaries, and captures are retained in the ignored
analysis archive, not in Git. This display-only correction does not change
radar detection, lead selection, or NAS replay dependencies.
