# Automatic indicator display ? 2026-11-01

The owner requested visible reasons and preparation status on C3, C4 and USB HUD.
The same bounded two-line presentation is drawn on all three renderers. Idle is
hidden; preparation, request, confirmed activation and two seconds of termination
are distinct. Current CAN lamp flags are shown separately from activation history:
normal flashing OFF is not reported as a new failure. Loss of telemetry expires the
badge within 0.6 seconds, rather than retaining an old success.

The source reads the already-evaluated policy, model candidate, ATC preparation and
250m helper. It does not alter any trigger, steering request, ownership, CAN message,
lease, model input, inference scheduling or deadline. Model preparation metadata is
additive at MetaData ordinals 33/34. A tentative candidate is not a lamp request.
The 250m display distinguishes BSD and front-gap gating; other geometry/admission
conditions remain grouped, not falsely diagnosed as one specific sensor fault.

A source worker writes at most five small /dev/shm snapshots per second. C3/C4 cache
reads at 5Hz. The existing HUD snapshot carries an allowlisted record below 1KB;
no new packet cadence or image stream is introduced. Optional metadata is discarded
before existing camera/NAV images on packet pressure. Jetson maps the source clock
and never renews a repeated observation. Source I/O failures cannot change control.
Text width measurements are bounded and cached. Display rendering has a small
nonzero cost; no physical timing or hard 50ms deadline guarantee is claimed.

No new option or automatic activation is added. The existing indicator modes and
model/manual options retain their meanings. UI support does not broaden the current
vehicle/transport support. This does not change the eGPU/ONNX model selection.

Validation: 679 local indicator/HUD checks passed, two platform checks skipped;
focused new display/schema/clock/budget checks included. New production code lint
passes. Existing unrelated modeld/hud lint findings were not changed. Four status
examples were rendered with the actual raylib font/draw path and inspected locally.
Additional C3 HUD regression checks: 29 passed. Native Signal CI 41267728745,
Jetson CI 41267728721, eGPU compatibility and documentation CI passed. No vehicle
write, restart, CAN actuation or physical display/road acceptance was performed.

Delivery: normal carrot-wip Git Pull and comma restart; Jetson USB display needs
the matching signed host update through the normal reboot updater. No reflash or
model replacement is required.

Published source: `5fd3d09fa48971583d717e036e32aacdf895b3e4`.
Signed immutable host packages preserve each existing model URL/hash/size and ABI:

| Selection | Host source | Bytes |
|---|---|---:|
| N16 | 5fd3d09fa48971583d717e036e32aacdf895b3e4 | 64862237 |
| Flip Flop | 18de2568b022cff450f9bbe8e1dbc67c43c27815 | 64862086 |
| Light speed | cda6a9f2bbb352edd28241eb7891ccc11f2c8fda | 64862136 |

Each archive's 1,167 payload files match its committed export (only the documented
bootstrap model aliases are substituted). Signatures, local/NAS and full HTTPS
bundle hashes are verified. The catalog is unchanged; retired MD v3 remains retired.
The historical `md-v3-release.json` filename is still the existing Flip Flop runtime
reference, not a reintroduced model. No global stable manifest or device was modified.
