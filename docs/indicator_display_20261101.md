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
Native CI and signed host releases will be recorded after publication. No vehicle
write, restart, CAN actuation or physical display/road acceptance was performed.

Delivery: normal carrot-wip Git Pull and comma restart; Jetson USB display needs
the matching signed host update through the normal reboot updater. No reflash or
model replacement is required.
