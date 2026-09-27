# Jetson health and application updates

Scope: `carrot-jetlink`, Jetson Orin Nano Super, L4T 36.4.7 / TensorRT
10.3.0. Initial installation still uses the sanitized SD image. Routine source
and model updates do not require reflashing. OS/ABI/QSPI changes remain separate.

## Deployment compatibility

The distributable image must remain unprovisioned. It contains no owner's Wi-Fi
profile, SSH authorization list, SSH host key, machine ID or C4 dongle binding.
First boot creates per-device identity and host keys; owners provide their own
network/access configuration. A personally provisioned test card is not the
distribution master.

USB discovery uses the Jetlink VID/PID, not a specific C4 serial number. The
connection still requires compatible C4 Jetlink code and the exact supported
model contract. This is not a universal image for arbitrary Jetson hardware,
JetPack versions, USB displays or unmodified openpilot branches. The supported
image hardware is the Orin Nano Super developer kit with matching QSPI firmware.
Another C4 connection and fresh-medium provisioning must be physically tested
before describing cross-device deployment as validated. Shared NAS manifests
and the embedded public signing key allow each compatible installation to update
without receiving the publisher's private signing key or a replacement image.

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
/models/jetlink-host-<source-commit>/precompiled-runtime.tar.gz
/models/jetlink-host-<source-commit>/manifest.json
/models/jetlink-host-stable/manifest.json
```

Only promote `jetlink-host-stable/manifest.json` after testing the exact immutable release.
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

### 2026-09-27 release candidate

The immutable host package is `e12376982fd0a522ea27bdd2a38966cd36e74f65`,
47,194,420 bytes, SHA-256
`64c787a90231675105a1b6512a45bb8aef194b8246a14fa7fbd38a6a139d7de5`.
The pinned model remains Cinque v2; this release does not select Cinque v3.
The NAS `jetlink-host-stable/manifest.json` channel serves this signed package;
an independent HTTPS readback matched the immutable manifest and passed signature
verification. This makes routine update discovery available, but is not evidence
of the still-pending exact-release reboot or new-medium validation below. The
initial-install image has not been promoted to a public download endpoint.

The initial health/status/updater/image suite passed 44 tests on the Jetson;
the web suite passed 9 tests. After the final timeout-cleanup change, all 12
updater/host-health tests passed on the actual Linux host. The timeout test
checks cleanup of the entire isolated probe process group, not just its
`runuser` parent. Signature-tampering and interrupted-activation tests also
exercise the installed updater. These are controlled software tests, not
physical power-cut or overheating tests.

The earlier `a5dc674a` package was downloaded through the real NAS endpoint,
staged with signature/hash checks and activated by a real parked reboot. The
boot probe produced `CANDIDATE_PROBE_OK` before normal inference/HUD startup.
Carrot Web was visually checked with the Korean IP/temperature/status line.
P-gear navigation hiding was retained.

A 180-second observation during image construction included one approximately
640 ms inference and a brief external-model fallback. Its cause is unproved;
that loaded build interval is not accepted as a clean steady-state timing test.
The first observer used conflated subscriptions, so its frame-ID discontinuities
must not be reported as independent camera-drop evidence. Final steady-state
verification uses non-conflated subscriptions after image work and transfer end.

At the owner's departure on September 27, the live C4 was on `e1237698`; the
Jetson was still running the successfully activated `a5dc674a` release, with
`e1237698` staged for the next normal boot and its final updater bootstrap
installed. A final parked snapshot showed active/fresh Jetson inference,
72.062 C, no diagnostic error, both inference/HUD services active, valid/alive
model and pose messages, valid CAN, and `inputsOK`/`posenetOK`. Speed was zero,
gear P and engagement false. No departure-time reboot or model switch was
performed. This snapshot does not establish clean driving timing.

The rebuilt `e1237698` image passed imports, model/privacy/filesystem checks.
A read-only image audit matched all eight checked boot/HUD/updater configuration
files to the working host, including Xorg setup, the writable HUD log directory
and the utmp delay override. Compression finished on the host, but its final
hash/transfer and the new-media write were still pending when the vehicle's
network disconnected. Partial PC captures must not be flashed or distributed.
The final isolated timing run, exact-release reboot check, complete PC image
verification and USB readback remain separate outstanding checks. Do not infer
their completion from the successful source tests or offline image audit.

New-media first boot remains a separate physical test. Updating and rebooting
the existing vehicle SD does not validate first provisioning, partition growth,
or the display on the newly written installation medium.
