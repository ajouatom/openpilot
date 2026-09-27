# Jetson health and application updates

> 현재 콤마 배포 기준은 `carrot-wip`입니다. 아래는 당시 실험 기록이며,
> 최신 안내는 [한글 통합 기록](jetson_wip_integration_20260927.md)과
> [초보자 설치 안내](INSTALL-WINDOWS-KO.md)를 보세요.

Scope: `carrot-jetlink`, Jetson Orin Nano Super, L4T 36.4.7 / TensorRT
10.3.0. Initial installation still uses the sanitized SD image. Routine source
and model updates do not require reflashing. OS/ABI/QSPI changes remain separate.

## Deployment compatibility

The distributable image must remain unprovisioned. It contains no owner's Wi-Fi
profile, SSH authorization list, SSH host key, machine ID or C4 dongle binding.
First boot creates per-device identity and host keys. With C4 `526f81421c` and
host `f2b22dc`, Wi-Fi configuration arrives automatically over the private USB
bootstrap channel, before model readiness. Owners may optionally supply SSH
public keys in setup.json; that is not required for Wi-Fi or inference. A
personally provisioned test card is not the distribution master.

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
P gear alone is not this condition. C4/Jetson power and networking must remain
available long enough for an offroad timer check and download; installations
that cut Jetson power immediately at ignition-off need a powered offroad window
or explicit parked administrator staging.
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
verification. This makes routine update discovery available; actual activation
and new-medium validation are recorded separately below. The
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
At that point the final isolated timing run, exact-release reboot check, complete
PC image verification and USB readback were outstanding. Subsequent completion
must be established by their own evidence, not inferred from the source tests.

### Completion after vehicle reconnection

On reconnection the Jetson was running `e1237698`. Its durable updater status
was `applied`, with no pending manifest or unfinished transaction. Both inference
and HUD had zero restarts in the current boot and systemd reported no failed
units. The normal startup had already applied the candidate; no additional
reboot was needed. A request to the default stable update URL passed on the
actual host and correctly kept the already-current version.

With image downloading still active, a non-conflated 180-second observation
recorded one missing model frame, one invalid cameraOdometry message and six
invalid pose-input samples. One health sample showed the external model inactive.
Mean/max model execution was 38.412/79.493 ms. These are retained as adverse
observations; attribution to the download is not established by this single pair.

After image construction and vehicle transfer both ended, a separate 180-second
parked observation recorded 3,600 model/odometry frames and 3,600 frames from
each camera, with no frame-ID gaps, model/odometry/CAN/pose invalidity or external
model inactivity. All 180 health samples were fresh and OK. Model execution
mean/max was 37.736/47.142 ms, with no execution above 50 ms; temperature was
69.437-70.375 C. Road/wide SOF maxima were 57.761/57.818 ms and model publication
maximum was 67.606 ms: this does **not** establish a hard 50 ms publication
deadline. Both observations recorded zero speed and disengagement; gear was P
when checked before testing. Driving is unvalidated.

The exact image was downloaded, decompressed and fully hash-verified on the PC.
The NAS candidate copy was independently read back and hash-verified.

| Artifact | Bytes | SHA-256 |
| --- | ---: | --- |
| `carrot-jetson.img` | 25,769,803,776 | `c395607c51a4ba8eb4a6d7da729eac76bede7811b8105dc2df45e84677ed2606` |
| `carrot-jetson.img.zst` | 8,249,732,184 | `9ff00027cb51d947eb8c2085db5dbc22f2af1104c4fe96358fc4b5b5286d3e71` |

After preserving build logs and confirming no image loop mount or helper remained,
the generated image workspace was removed from the Jetson. Its installed runtime,
model cache, identity and rollback releases were retained; root usage returned
to 18 GiB with about 99 GiB available. Final C4 telemetry still reported fresh,
active Jetson inference with no error. NAS images contain no personal setup;
the owner's installation medium is provisioned separately and is not a master
to clone for distribution.

New-media first boot remains a separate physical test. Updating and rebooting
the existing vehicle SD does not validate first provisioning, partition growth,
or the display on the newly written installation medium.

The owner's selected 128 GB installation medium was subsequently written and
all 25,769,803,776 bytes were read back with the exact original SHA-256 above.
No GPT/FAT exception or partial comparison was needed. The 64 MiB CARROTSETUP
partition and its `IMAGE.json` source commit were checked, and the separately
written personal provisioning file passed a byte-hash readback. Personal setup
is exclusive to this owner's medium and is excluded from the NAS master image.
This completes media preparation, not the new-media physical boot test or a
connection test with another C4.

### New-medium boot and navigation follow-up

The owner subsequently confirmed booting the newly written medium. SSH presented
a new per-device key, provisioning and root-growth services completed successfully,
and `/dev/mmcblk0p1` reported 116 GiB with 99 GiB available. The running release
was `e1237698`; inference, HUD and update timer were active, with no failed units.
The HUD reported a decoded 960x540 map and the existing C4 connection was active.
This establishes first provisioning/root growth and running display services on
the new medium. Another C4 and navigation smoothness remain separate checks.

The owner reported navigation stuttering while driving. A subsequent parked
25.084-second observation saw 222 map video messages, with a maximum serialized
event of 264,752 bytes and source-receive age below 8.2 ms. The forwarding child
reported no stale/abandoned events, but maximum enqueue duration grew to 652.6 ms.
The transport currently drains one 32 KiB navigation fragment per inference
window: a 264,752-byte event needs nine windows, spanning approximately 400 ms
at 20 Hz even before queueing. This is a concrete latency mechanism, not proof
that it explains every reported driving interruption. A separate 30-second HUD
status sample reached 402 ms map age (two samples above 300 ms, no stalled flag).
The 1 Hz status sampling does not measure every displayed frame or end-to-end
source latency. Host temperature was healthy at approximately 68 C. No transport
budget or inference timing policy was changed during this diagnosis.

At the owner's request, the P-gear trip-report override was temporarily disabled
only inside the Jetson renderer process, using a `/run` service override with a
180-second automatic restoration timer. Actual vehicle gear, Params, inference
and committed release files were unchanged. After the display restart, excluding
the first 20 seconds, 125.54 seconds contained 1,250 renderer updates and 814 new
map frame presentations (approximately 9.96 and 6.48 Hz respectively). There were
222 change intervals above 200 ms; 1 Hz diagnostic samples reached 1,004 ms map age
and five exceeded 300 ms. The initial restart/reacquisition maximum of 9.94 seconds
is excluded from the steady observation. This measures distinct frame selection
inside the renderer, not optical panel timing or source-to-panel latency.

During a simultaneous 120-second vehicle observation, all 2,400 model, odometry
and each-camera messages were valid with no frame-ID gaps, CAN/pose invalidity
or external-model inactivity. Model execution mean/max was 37.720/43.622 ms;
temperature ranged 69.656-70.437 C, speed remained zero and control was disengaged.
Forwarding stale count stayed at 86 during the later observed interval, with no
abandoned events; the earlier increase was not timestamped and cannot be assigned
to display startup or this steady interval. The original HUD service and P-gear
behavior were explicitly restored and verified after testing. This reproduces
uneven map updates with healthy inference while parked; the precise contributions
of transport queueing, source cadence and decoder scheduling remain unresolved.

### Navigation correction and standalone host repository

Host sources now have the public repository `https://github.com/ajouatom/carrot-jetson`.
Its initial exported runtime records the originating Carrot commit and pins the
NVIDIA ABI; OS images, weights, personal provisioning and signing private keys
are excluded. GitHub CI checks sources and builds an unsigned candidate. A push
alone does not authorize an installed vehicle update.

Carrot now carries the signed host manifest in
`openpilot/selfdrive/modeld/jetlink/host_release.json` and forwards it in the
existing bounded HUD snapshot. On a fresh offroad snapshot the host stages that
exact release/model, verifying its signature and compatibility. An invalid pin
does not fall back to a different release. Older Carrot builds without the field
retain the NAS stable fallback. The apply transaction and boot-time engine probe
are unchanged. Existing images need the updated updater helper installed once
to honor the pin; this owner's host was migrated as part of deployment. Future
images must use the current helper. Merely updating a runtime does not rewrite
the stable bootstrap files automatically.

The navigation correction keeps one small fragment during inference. Only after
returning the model reply may it admit additional ready fragments within a 2 ms
admission budget: two for legacy hosts, up to eight for hosts advertising the
independent media pump. A single transport write retains its existing watchdog,
so 2 ms is not a hard write-duration guarantee. The host receives/decodes on a
separate 100 Hz worker and publishes immutable latest snapshots to the unchanged
10 Hz display. Ordered H.264/keyframe recovery and all control/model validity
policies are retained.

Early rolling-process comparisons were discarded as evidence of the C4 fix:
the manager pre-imports Python modules before forking children, so restarting
only Jetlink reused old in-memory daemon code. C4 was then rebooted while parked.
The new `navigation_tail` counters and negotiated receiver capability confirmed
the corrected code was actually executing. Do not attribute early timing changes
to the C4 fix or repeat that process-only deployment method for changed modules.

The accepted post-reboot 180-second observation recorded 3,600 model/odometry,
road and wide frames, no gaps/invalidity/model inactivity, and execution mean/max
37.810/48.678 ms (none above 50 ms). Temperature was 68.781-71.062 C, speed zero,
and control disengaged. The source itself supplied 1,531 map frames (8.51 Hz),
with a maximum input gap of 458.1 ms. Over the overlapping display observation,
1,260 distinct map frames were selected in 190.91 seconds (6.60 Hz), no decoder
requests were dropped, and 1 Hz status samples reached 509 ms map age versus
1,004 ms in the original parked trial. These unequal live inputs do not establish
a controlled improvement ratio or elimination of all driving stutter. Earlier
startup stale-message counts remained unchanged during steady observation.
P-gear temporary display overrides were removed after the accepted test.

The first standalone release is `carrot-jetson` commit
`3f3142e01a77a7c5f6f6111c9dba8843300e3b50`, published as `v0.1.0-preview`.
Linux GitHub CI passed 28 tests. Its media/server/read-ahead sources are byte-for-
byte identical to the accepted vehicle trial; the added updater pin path has a
regression covering selection precedence and rejection without fallback.
The actual host downloaded the signed NAS bundle, passed `CANDIDATE_PROBE_OK`,
activated the release and installed the pin-aware bootstrap. The C4-forwarded
manifest's signature and source matched the running host; staging that exact
selection correctly performed no change. Both the NAS channel and GitHub release
manifest were independently downloaded and verified after publication.

After this final activation, a further 60-second parked observation recorded
1,201 model/odometry and 1,200 frames from each camera with no gaps, invalidity,
host health fault or model inactivity. Execution mean/max was 37.968/47.600 ms,
temperature 70.250-71.062 C, speed zero and control disengaged. The original
P-gear policy is restored. The initial SD master remains the separately hashed
`e1237698` candidate; updating the runtime does not change that image artifact.

### USB Wi-Fi bootstrap and distributable image correction

The personally configured test medium did not establish a setup-free installation.
The USB Wi-Fi extension now provisions before ensure_engine, so missing Internet
cannot prevent receipt of credentials. A separate normal-priority comma process
reads saved NetworkManager client profiles; a negotiated private message bypasses
HUD/Params/logs. Packets have local freshness checks, 16 KiB and eight-profile
bounds. The Jetson writes only 0600 private temporary data and managed NM profiles.
It imports WPA-PSK, SAE and open networks, preferring the comma active connection;
enterprise/captive-portal authentication is not implemented. The physically attached
comma is trusted to provide Wi-Fi configuration. This does not provide USB NAT.

Credentials and changes are refreshed without removing the SD card. Owner changes
replace only prior carrot-usb profiles; independent manual profiles remain intact.
Failed secret queries never send an empty deletion set. Model inference and camera
policies are unchanged. A secret-free bootstrap subset carries the signed release
and fresh explicit offroad state, supporting update selection before model readiness.
The new image includes the Wi-Fi service and current updater bootstrap. Existing
hosts need the service installed once; the owner's device is tested separately.

Initial desktop checks passed 48 tests with five Linux-only skips. The final
validation results below supersede that preliminary test count; physical image
boot remains a separate check.

The final host candidate is `f2b22dcf0bd0708658668f7efa2dfb29f81b3bd1`
with C4 `526f81421c`. Actual typed Params returns a boolean; the initial candidate
sent an unknown road state and correctly blocked automatic updates. The corrected
sender accepts typed booleans and legacy byte/string values. Actual USB reception
now shows onroad=true and the exact signed final manifest; automatic_stage leaves
pending absent while onroad. Linux CI passed 61 tests (run 36302510193); desktop
passed 56 with five platform skips, plus 24 image tests with one Linux-only skip.

Three parked clean-network trials backed up only local root-private profiles,
removed every NM Wi-Fi profile, and restarted the receiver. All recreated the two
comma profiles and connected over USB (6.36 s, 2.44 s, and 28.52 s respectively).
The first two trials failed immediate/bounded HTTPS checks and restored their
backup, so they are not end-to-end successes. The instrumented third trial recorded
EAI_AGAIN DNS preparation failures until about 49 s, then NAS HTTPS 200 on attempt
six (service total about 56 s). It passed with the original carrot-setup profile
absent. A separate systemd-launched HTTPS probe also passed. Original secret
backups were deleted from /run after success; no secret content was exported.
No DNS addresses were hardcoded or validation bypassed to obtain success.

After final C4 code activation, 120 s of unconflated observation recorded 2,400
model/odometry/road/wide messages, no frame gaps, CAN/pose validity failure, model
inactivity or host health fault. Execution mean/max was 37.958/47.799 ms, temperature
71.156-72.062 C, zero speed and control disengaged. This overlapped low-priority
image preparation. A candidate probe emitted an NvMap allocation warning but
completed all finite inference checks; the subsequent normal runtime observation
above is the accepted steady-state evidence. No driving benefit is claimed.

SSID/password rotation and different-owner profile replacement are covered by
synthetic NM-boundary tests; no router password or second physical comma was changed
for this test. The signed host channel and GitHub v0.2.0-preview now designate f2b22dc.
The new image was built from the hash-pinned unprovisioned e1237698 master,
not from this live host. Image/USB completion and first physical boot remain separate.

The refreshed unprovisioned image completed its offline checks and was downloaded
and decompressed with complete SHA256 verification on the Windows PC. The NAS
copy was independently read back and verified under
`\\DS1821P\openpilot\dev\jetson-images\candidates\20260927-f2b22dc-wifi`.
No Wi-Fi setup.json or SSH authorization key is appended to this owner's new USB.

| Artifact | Bytes | SHA256 |
| --- | ---: | --- |
| carrot-jetson.img | 25,769,803,776 | 5a1e7a3ba6156c621d8a01412f062b6c16ecb8ef2274b82b6acdaadf4b19a516 |
| carrot-jetson.img.zst | 8,249,905,714 | 61fce013d1fb9db548084ca4d2e9a3ea9d697717b1470725f0fec39830fc3285 |

The image includes the Wi-Fi service and current bootstrap updater, with the
same pinned model as the accepted runtime trial. Runtime imports, sudoers,
model/privacy checks, ext4/FAT checks and GPT validation passed. The remote build
workspace was removed after PC verification and log preservation, restoring
98 GiB of free root space. Inference, HUD, Wi-Fi and update timer remained active.
`v0.2.0-preview` includes image metadata/checksums and the explicit physical-boot
pending status; its `manifest.json` is the separately signed runtime release.

The authorized 128 GB medium was written with all 25,769,803,776 image bytes,
then independently read in full. Its SHA256 matched the raw image above;
the writer reported `SD_WRITE_COMPLETE_AND_VERIFIED`. No personal setup was
appended. PC and NAS BUILD-STATUS records now report full USB readback success
while retaining `physical_boot_verified=false`. The new medium still requires
insertion and first boot in the Jetson; the existing-host tests do not establish
that result or compatibility with a second physical comma.

### New-media C-to-C follow-up

After the owner confirmed booting the new 128 GB medium, C4 logs at 17:40
showed a Jetlink handshake, warmup and active inference, followed by USB
detachment. The connector used for that successful interval was not established.
Subsequent direct C-to-C reconnection showed `Powered cable w/ sink` on C4,
`0955:7020 NVIDIA L4T` on its USB host bus, and no attached gadget UDC.
Thus the current link has reversed roles, before Jetlink model loading; it is
not demonstrated to be a model/image payload failure or a defective cable.
The new Jetson responds to SSH at its previous address with a newly generated
host key; no management key is installed in the generic image, so authenticated
Jetson-side diagnostics are unavailable. No role registers were modified.

NVIDIA's developer-kit hardware guide documents USB-C host support. A role
fix requires checking the installed FUSB301 driver's semantics and a controlled
SuperSpeed/inference trial before inclusion. The current first-boot setup reads
SSH keys and optional Wi-Fi; it does not execute arbitrary hotfix files. Offline
media patching is feasible, but no C-to-C hotfix has been validated or delivered.

The optional-host badge no longer labels a fresh `waiting` link as ERROR when
the current model report is inactive with no error. The web diagnostic retains
`Host not connected`; real inference errors, active-model link loss, stale state,
retry failures and thermal faults remain visible. Control disengagement/fallback
logic is untouched. Fourteen focused status/health tests passed.

The user then booted the previous managed medium and connected C-to-C. SSH
confirmed FUSB301 SNK(4), Try.SNK=1 and the Jetson's device role. Source inspection
of NVIDIA jetson_36.4.7 established that fsw_trysnk=0 must precede fmode=1:
otherwise detach can restore the preferred-sink policy. The driver performs
error recovery and CC negotiation, then sets USB_ROLE_HOST after detecting Rd.
No direct role override, I2C register write, firmware or kernel replacement is used.
The trial enumerated the comma at 5000 Mbps in about six seconds. A guarded
P3768/36.4.7 boot service now applies that policy before inference and makes no
detach-inducing write if it is already selected. Reboot restored host/SRC(1),
5Gbps and active model inference without cable reconnection.

The first 120-second C-to-C trial had no frame-ID gaps or validity failure, but
one 84.711ms model execution and one inactive diagnostic sample. USB logs did
not show disconnection; their cause is unproved. A second 120-second capture
after reboot had 2,400 model/odometry/road/wide/pose messages each, no skipped
frames, validity failure, inactive sample or health fault. Execution mean/max
36.582/43.226ms; temperature 66.281-68.656 C, speed zero and disengaged.
Linux CI 36308578134 passed 68 tests on host source bae1927. Desktop image/role
checks passed 30 tests with two Linux-specific skips. Suspension, driving and
other carrier boards remain untested.

The user's new 128GB medium was separately identified by its zero-key first-boot
result. An owner public-key setup file and the 4,492-byte hotfix ZIP were added
and read back without rewriting the image. This is staged data, not execution:
the existing first-boot loader consumes the key but does not execute the ZIP.
Authenticated installation and validation on that medium follow its next boot.
Public base images and the hotfix ZIP contain no owner credentials.

The new medium subsequently booted as its own generated machine identity and
accepted the staged owner key under its previously observed SSH host key. Before
installation it reproduced SNK(4)/device and the C4 waiting state. Installing the
same hotfix restored 5Gbps, active inference and the USB display. A further reboot
automatically restored SSH, USB host/SRC(1), inference, HUD, USB Wi-Fi and the
update-stage timer. Two managed Wi-Fi profiles remained configured. The installed
helper SHA256 matched the published hotfix manifest; model/runtime stayed f2b22dc.

Final new-medium 120-second parked capture: 2,400 model/odometry/road/wide/pose
messages each; zero frame-ID gaps, validity failures, inactive samples or health
faults. Model mean/max37.362/44.149ms, no execution over50ms, camera SOF maximum
57.529ms and temperature65.093-68.343C. Speed zero, control disengaged. This
validates the new medium plus owner key plus USB-C hotfix, not unmodified-master
C-to-C operation or driving. The published base image bytes were not regenerated;
the public hotfix and updated image-building tools remain separate artifacts.
