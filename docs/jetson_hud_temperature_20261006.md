# Jetson HUD activation and temperature, 2026-10-06

The user requested that a HUD attached to Jetson run independently of the
vehicle's External HUD Display switch, show Jetson temperature and warn when hot.

## Behavior

- Only the Jetson renderer's read-only Params adapter forces `ClusterHud=1`.
  No saved vehicle setting changes. Its existing onroad/debug output gate stays:
  ignition off blanks the panel unless an always-on debug mode was selected.
- Physical USB detection and unplug/retry behavior remain. The vehicle preview
  worker may start with `ClusterHud=0` only when a fresh Jetson HUD output
  heartbeat confirms a connected panel. Stale telemetry (three seconds), no
  panel or ignition off ends that worker, subject to the original direct-HUD gate.
  Vehicle state/camera transport remains on the vehicle; rendering and encoding
  for the Jetson-attached panel run on Jetson at the existing 10 FPS.
- The Jetson badge gains a second line showing the maximum valid internal sensor
  temperature. It follows left/right layout; modes without the driving status
  row use the upper-right corner. Expired/missing/invalid data shows `--°C`.
- Existing per-zone thermal policy is retained: warn at limit minus 5°C, error
  at the limit, excluding passive trips bound only to `hot-surface-alert`.
  Each temperature is compared with its own zone's configured limit, not another
  sensor's limit. The banner reports the nearest limit's temperature and limit.
- Thermal identity stays separate from storage/runtime faults. Korean/English
  warning strips request cooling checks. Vehicle alerts suppress the health
  strip and render last; no new audio, control event, reboot or shutdown is added.
  NVIDIA's existing thermal protection is unchanged.

NVIDIA's standard Orin Nano/NX table lists software throttling at 99°C, hardware
throttling at 103°C and software shutdown at 104.5°C. The HUD reads installed
sysfs limits rather than assuming every device uses those defaults. With a
99°C trip, the displayed warning starts at 94°C. A 70°C surface notification is
not treated as processor overheating.

Source: [NVIDIA Jetson Linux thermal specifications](https://docs.nvidia.com/jetson/archives/r36.4.3/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html#thermal-specifications).

## Validation and delivery

Focused desktop coverage verifies independent activation, forwarded ignition
and brightness, heartbeat creation/expiry/removal, per-zone 94/99°C boundaries,
custom limits, surface-only trips, invalid/stale temperature, storage-fault
identity, Korean/English banners and vehicle-alert precedence. Production
raylib renders cover normal/warning/error, vehicle alert, navigation, graph and
swapped panel layout. These synthetic checks do not establish physical USB,
real thermal sensor, loaded inference timing, or on-vehicle update validation.

The catalog and Korean/English guides explain direct-device versus Jetson
attachment. The Wiki setting's manual explanation is synchronized separately.
Host changes require a signed Jetson source bundle; a vehicle-only commit does
not update an already-installed host. Model, TensorRT/JetPack ABI and base image
stay pinned. Release hashes and local render/test evidence are retained in
`.analysis/archive/2026-10-06/jetson-hud/` (private, not committed).
