# DM hold on first surrounding traffic

The user clarified that the hold starts only when moving traffic appears after
an absence, to make standard DM warnings respond to distraction for twenty
seconds. Additional vehicles while traffic remains must not extend it.

TrafficContext keeps the existing observation region, ground-speed threshold,
0.2-second confirmation, duplicate matching and two-second dropout retention.
It now latches aggregate occupancy: all retained observations must disappear
before another confirmed appearance can start a hold. These are DM consumers of
radarState; radar detection, lead selection, outputs and NAS replay are unchanged.

The selected DriverMonitoringMode remains 1. While strict context is active,
effective camera monitoring uses stock vision and fallback timing, detection
thresholds and input handling. Experimental interaction grace, extra pose/phone
tolerance and special forward-attention resets are suspended. Entry expires old
grace and forward streaks without resetting awareness, warnings or lockout.
Without camera data, standard interaction timing remains 15/30/45 seconds.
Unhealthy context also selects standard criteria until healthy data returns.

After twenty seconds, experimental criteria resume. Continued occupancy still
prevents the empty-road multiplier. Old grace cannot resume and budget expansion
cannot clear orange/terminal alerts. Attentive drivers receive no forced warning
just because traffic appeared. A new physical interaction after the hold can
start a fresh experimental grace under existing rules.

212 desktop monitoring tests passed, including first occupancy, extra vehicles,
short dropout/lane-change association, rearming after absence, full twenty-second
lifecycle, stock equivalence during strict camera monitoring, retained terminal
state and existing touch/parking regressions. Windows adapters substitute native
IPC/Params/hardware imports while using real policy and cereal logic. This is not
vehicle or physical radar validation. Reproduction is archived locally under
`.analysis/archive/2026-09-29/dm-ui-sound/`.
