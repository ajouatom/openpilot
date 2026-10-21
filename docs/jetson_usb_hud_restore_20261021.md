# Restore bounded USB display transport on Jetson

The owner requested returning comma-to-Jetson HUD/navigation data to USB after
Wi-Fi interruptions. The Jetson still drives its attached USB panel. ESP32-S3
is not restored. This supersedes the Wi-Fi-only display policy of October 20.
Internet access is still needed for downloads and the external phone navigation
connection is unchanged; removing the Jetson display Wi-Fi hop does not repair
phone hotspot reception.

## Models stay unchanged

| Branch/path | Selected model |
| --- | --- |
| carrot-wip Jetson | Cinque v2, checkpoint `1a421175/12864`, ONNX SHA `09d080f3…` |
| carrot-wip eGPU | MD v3, PR #39087, manifest `comma4-big-md-v3-1365126a`, SHA `d4c7a3fc…` |
| Owner's carrot-nav Jetson | N16 Nav `96244b1f`, ONNX SHA `01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a` |

The new runtime and pin are carrot-nav-specific. Carrot-wip already retains its
USB display transport and its own model/runtime; it receives this record only.
Neither branch's model weights, driver monitoring or control policy changes.

## Transport

New Jetson receivers explicitly advertise `carrot_hud_usb_v2` plus the existing
HUD, navigation and receive-pump capabilities. Only this complete contract enables
USB display publication on carrot-nav. Older Wi-Fi-only signed receivers continue
using their existing Wi-Fi path until updated. There is no unnegotiated fallback.

The low-priority snapshot child runs at 10 Hz with the existing 96 KiB snapshot
limit. Mode 7 includes the bounded 256 x 256 NAV preview. During each model GPU
window, the sender admits at most one snapshot, one 32 KiB navigation fragment,
and one extra ready 32 KiB fragment only within the first 4 ms. The host's existing
two-message read-ahead queue plus one reader-owned message bounds this overlap.
The 4 ms condition is an admission check, not a hard write-completion deadline.
No post-result video burst is added on this Jetson path. The response watchdog,
model input and pose validity rules remain unchanged. All early media bytes are
now included in runtime timing diagnostics.

## Parked physical comparison

Same comma boot, source `691566f0fa`, N16, automatic FIFO2 USB completion helper,
camera/model/DM processes and CPU placement. Fresh Park, zero speed, inactive
controls and disallowed Panda controls were checked continuously. The candidate
server ran from a bounded temporary folder with an independent rollback guardian;
operational source/model files were not overwritten for the comparison.

| Metric | Wi-Fi, 120 s | USB, 120 s |
| --- | ---: | ---: |
| Model frames / gaps / invalid | 2400 / 0 / 0 | 2400 / 0 / 0 |
| DM frames / gaps / invalid | 2401 / 0 / 0 | 2400 / 0 / 0 |
| Road and wide camera frames | 2400 each, consecutive | 2400 each, consecutive |
| Odometry/pose invalid or unhealthy | 0 | 0 |
| Model mean / p99, ms | 44.256 / 47.217 | 44.226 / 47.020 |
| Model maximum / count above 50 ms | 63.704 / 3 | 63.361 / 2 |
| USB exchange mean, ms | 37.231 | 37.319 |
| GPU mean, ms | 25.400 | 25.405 |
| Jetlink main-thread CPU, ms/frame | 4.882 | 6.514 |
| Whole-comma CPU busy, percent | 78.38 | 77.13 |

Model requests are 458,996 bytes/frame, about 9.180 MB/s at 20 Hz. USB display
adds approximately 0.644 MB/s (0.518 HUD/preview plus 0.126 navigation media),
about 7.0% of model-request payload. These are payload rates derived from logged
per-frame aggregates, not physical USB wire utilization. Jetlink's main thread
uses about 1.63 ms/frame more CPU, or 3.27 percentage points of one core; total
CPU did not increase in this bounded comparison. Separate workload variation
prevents attributing the entire total-CPU reduction to transport.

The host's 120 one-second USB samples all have fresh HUD and NAV-preview status.
Its 960 x 540 phone video advances throughout, with no recorded decoder drops;
the Wi-Fi display receiver is absent. Wi-Fi has fresh HUD in 120/120 samples and
preview in 119/120. Source NAV validity is not equivalent to preview freshness:
Wi-Fi samples are `ready` 81/120 and `route_position` 39/120, whereas the USB
window is `route_position` throughout. This validates real display traffic and
unchanged model execution, not valid-route driving behavior or a closed-loop NAV
comparison. No 50 ms hard deadline, complete locationd cure, hotspot diagnosis
or long road-stability claim is made.

A subsequent 60-second Wi-Fi return has 1200 consecutive valid model/DM/camera
frames each and healthy pose, model mean 44.381 ms, p99 47.250 ms, maximum
52.838 ms and two above 50 ms. NAV remains invalid (`route_position`/`position`).
The independently started host observer has fresh HUD 60/60 and preview 58/60.
This supports retaining USB without claiming it eliminates the existing outliers.

The first setup attempt did not exercise USB: an existing later-named systemd
drop-in overrode the diagnostic command. A short baseline also overlapped that
service restart and is excluded. Actual process arguments and absent Wi-Fi
receiver were checked before the USB interval above. Temporary overrides are
removed after measurement. Raw logs and location data remain private.

## Delivery and validation

The signed host source is `691566f0fa9a47d9e7907567c48da962975b1a4c`, bundle
64,681,407 bytes, SHA-256
`8f6c1fad31849bbc54555fe06a3bc2fc21ef9be692564ac6f3470a62c97f7e6b`.
The existing NAS publication path verifies signature, archive source identity,
local hashes and the complete HTTPS download. N16 is unchanged. Desktop focused
checks: 65 passed, two native skips; [Linux CI](https://github.com/ajouatom/openpilot/actions/runs/39543921540)
passed for the source commit.

Routine activation is comma Git Pull and reboot/restart, followed by Jetson
reboot with actual offroad state and Internet available while the signed boot
updater prepares the release. No SD reimage or USB patch is required for the
owner's installed bootstrap.

The owner's comma completed ordinary Git Pull to `682ceeb484` and reboot. With
actual offroad held during preparation, Jetson rebooted and its installed boot
updater selected, verified and applied `691566f0fa` without manual cache staging
or an SD/USB installer. The protected DATA filesystem remains active with
9,910,919,168 bytes free (41% used). Installed server/publisher hashes match the
published source, standard service commands use `/opt/carrot-jetlink/current`,
and no diagnostic systemd override remains. The temporary offroad hold was
restored and N16 became active with its original hash and automatic FIFO2 helper.

Release-pin [Jetson CI](https://github.com/ajouatom/openpilot/actions/runs/39546242778)
and eGPU compatibility checks pass. The wip record-only publication also passes
[its checks](https://github.com/ajouatom/openpilot/actions/runs/39546340850), user
docs and eGPU compatibility. A boot handoff logs one late bootstrap
`unknown_message: type 16387` and a reconnect before stable startup; this record
does not claim the whole startup is error-free.


## Final installed-state validation

After ordinary activation and warmup, a further 120.010-second Park/inactive
window has **2400 consecutive valid model, DM, road and wide-camera frames each**,
with zero invalid odometry/pose messages or unhealthy pose flags. All 120 NAV
samples are `ready`; this resolves the earlier trial's invalid-NAV coverage
limitation for this stationary window, not for road behavior. Model mean is
44.174 ms, p99 47.821 ms, maximum 57.566 ms, with two above 50 ms. DM mean is
17.033 ms, maximum 27.005 ms. USB exchange mean is 37.121 ms, GPU mean 25.445 ms.
Whole-comma CPU is 77.12%, Jetlink main-thread CPU 6.360 ms/frame.

Display payload is approximately **0.707 MB/s**, 7.70% of the 9.180 MB/s model
request payload. Actual display admission/send maximum reaches 26.059 ms in one
aggregate window, so GPU overlap must not be presented as a guaranteed display
write budget. The 120 host samples all show fresh HUD and NAV preview, advancing
960 x 540 phone video, zero decoder drops and no Wi-Fi display worker. Mode 7
and original N16 weights remain selected. USB stays enabled after the test;
no further source patch, temporary service override or user installation step
is needed on these devices. Raw final records remain private.
