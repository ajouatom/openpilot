# Jetson health and application updates

Scope: `carrot-jetlink`, Jetson Orin Nano Super, L4T 36.4.7 / TensorRT
10.3.0. Initial installation still uses the sanitized SD image. Routine source
and model updates do not require reflashing. OS/ABI/QSPI changes remain separate.

## Health

The host samples sysfs and IPv4 interface addresses on a separate worker, once
per second. Inference reads a cached record. Temperature uses actual thermal
trip points: warning within 5 C of a passive/hot/critical trip, error at the
trip. A passive trip bound only to `hot-surface-alert` is excluded: on this
device its 70 C notification is distinct from the 99 C processor throttle
trip and 104.5 C critical trip. No kernel limit or fan policy is changed.
Unavailable sensors and stale records are unknown, never zero-temperature OK.
See [NVIDIA power/thermal documentation](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html).

Server/engine errors and invalid inference responses remain visible for 30 s.
The Jetson display has a separate host-warning strip; vehicle statistics still
describe C4. C4 gives faults precedence over the active Jetson badge. Carrot Web
shows host identity, fresh IP addresses, temperature and the reason in its eGPU
status area, including when no eGPU has ever been connected. This is a display
change, not a new engagement/fallback/temperature-control policy. The existing
model validity and vehicle alert behavior is unchanged. It does not diagnose
every possible hardware fault (e.g. a failed fan without a readable fault flag).

HELLO telemetry is never reused as a current sample. C4 records receipt times
for inference-piggybacked telemetry and idle state requests. Host sample age
and C4 receipt age are combined; old temperature/IP disappear after 3 s.

## Update format and trust

`build_host_bundle.py` exports committed sources and license notices only.
`sign_release.py` signs the exact source commit, bundle hash/length, pinned
ONNX hash/length and runtime ABI with Ed25519. The public key is included in
the image; the private signing key is kept outside Git and never distributed.
The updater accepts only signed manifests and HTTPS URLs on the configured NAS.

Immutable runtime bundles use the existing NAS model-file allowlist:

```
/models/jetlink-host/<source-commit>/precompiled-runtime.tar.gz
/models/jetlink-host/<source-commit>/manifest.json
/models/jetlink-host/stable/manifest.json
```

Only promote `stable/manifest.json` after testing the exact immutable release.
The image itself remains a separate larger download; do not advertise it as
available through the model-file endpoint (the image suffix is not allowed).

## Installation and activation

`install_updates.py` installs a stable bootstrap under `/opt/carrot-jetlink/updater`.
The timer checks every 15 minutes, starting after 2 minutes. Automatic staging
requires a fresh forwarded `IsOnroad=0`; missing telemetry does not authorize it.
Downloads and hashes run at low CPU/I/O priority and never switch active code.
Network failures leave the active release untouched and retry on the next timer.
An administrator may explicitly stage while parked:

```
sudo /opt/carrot-jetlink/venv/bin/python /opt/carrot-jetlink/updater/update_host.py stage
```

At the next normal boot, before inference/HUD start, the bootstrap checks the
signature and loads/builds the candidate's pinned engine in a separate process.
It checks the full model contract and three finite synthetic inferences, with
no vehicle USB/control connection. Only then does `current` switch atomically.
A rejected probe retains the previous release and remembered model. A durable
transaction record restores the previous release at the next boot after an
interrupted activation. Previous releases/models are retained for recovery.

Future model bundles can download new ONNX without a fresh SD image, but actual
vehicle model selection still requires matching C4 code/model contracts. Existing
strict Jetlink contract checks remain authoritative. This updater does not bypass
them or replace the Python/JetPack environment. A new model may require a long
first-boot compilation; the verified old release remains available if it fails.

This first implementation uses boot-time activation, not seamless live updating.
Its automatic recovery covers preparation failure and interrupted activation;
it is not a claim that every subsequent runtime/driving regression is detected
or automatically rolled back. Physical image boot and loaded-driving validation
must be reported separately from parked source-update and synthetic tests.

## Validation record

Focused tests cover thermal boundaries/recovery, stale handshake/telemetry,
fault precedence over active status, malformed/unsigned manifests, download
corruption, refusal to activate running services, missing offroad state and
failed candidate preservation. Live and image results are recorded after the
corresponding checks complete, with exact source IDs and artifact hashes.
