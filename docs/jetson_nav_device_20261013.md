# Orin RL navigation engine check — 2026-10-13

The user connected/powered the Jetson and authorized reboot and testing.
Work and publication remain **carrot-nav only**. The vehicle already contained
the signed nav release selection. Fresh vehicle observations before maintenance
were valid/alive, Park, standstill, zero speed, selfdrive disabled/inactive and
carControl disabled.

## Device preparation

The existing Orin Nano Super / L4T 36.4.7 / TensorRT 10.3.0 host was reachable
over Wi-Fi. SSH identity matched the recorded October 7 key; older September
keys did not match and were not bypassed. Jetson was rebooted once. Its boot
gate continued waiting for USB, with production inference/HUD blocked.

The signed `955a6f2d39` runtime and original hash-pinned ONNX were staged using
the installed updater. An isolated probe built the engine without a vehicle
USB/control connection. TensorRT build time was 204.5 seconds. A transient
NvMap allocation error was logged near the end of tactic selection, but the
engine completed, deserialized and passed the three-frame recurrent probe.
This warning is retained as evidence, not omitted as a clean build claim.

The temporary preparation process attempted to restart the stopped transient
boot-gate unit by name; systemd had collected that unit. The gate was restored
through `carrot-jetlink-update-apply`, which recreates it normally. Final gate
status was active. The previous runtime remains selected until the real USB
boot handshake authorizes the staged release; readiness was not fabricated.

## Isolated engine performance

200 synthetic frames, paced at 20 Hz, exercise the real TensorRT engine,
float32 map/navigation inputs, all five state buffers and finite feedback,
with a reset at frame 100. CUDA graph capture was active. All frames passed.

| Measurement | Result |
| --- | ---: |
| Actual paced rate | 19.968 Hz |
| Input copy, mean | 2.417 ms |
| Engine execution/transfers, mean | 24.780 ms |
| State checks/feedback, mean | 5.956 ms |
| Total work, mean / p99 / maximum | 33.153 / 33.701 / 38.570 ms |
| Work above 50 ms | 0 / 200 |
| Process CPU, one-core convention | 17.17% |

These are synthetic standalone engine numbers, **not** camera-to-result USB
latency or road performance. Host state copies have a measurable ~6 ms cost;
GPU-resident feedback may be investigated if real-link profiling needs it.

Prebuilt engine SHA256:
`7ba226dbd894833212826ee381d7cba1711b63764092bd40b5c1298f8bbb0ff1`.
Compatibility is the measured TensorRT 10.3.0 / Orin-sm87 environment, not all
Jetson or TensorRT versions. Engine numerical parity against ONNX on TensorRT
has not been established by these finite/performance checks.

The 795002404-byte plan and matching specification/metadata were copied back,
checked against the device hash, packaged and signed with the existing release
authority. The [prebuilt archive](https://upload.shind0.synology.me/models/jetlink-nav-rl-80eedcdd-trt10-3-orin-7ba226db/precompiled-runtime.tar.gz)
and [signed manifest](https://upload.shind0.synology.me/models/jetlink-nav-rl-80eedcdd-trt10-3-orin-7ba226db/manifest.json)
are stored separately from the runtime. The complete HTTPS archive SHA256
`f687725ccf61b842cadfa40064e050aa13f1eb87d00396d1ffdc11b3dbd29b7e`
and manifest signature were verified. This device already has the plan cached,
so its later activation can reuse it. This archive does not yet change the
automatic download policy on other devices.

## Unresolved USB connection

Before and after reboot, comma reported `Powered cable w/ sink` / `dfp`,
enumerating its existing Genesys/display hub. Jetson's Type-C host policy was
SRC but its USB role was `none`; neither side enumerated the other. The user
confirmed power and USB-C were connected. A physical replug/port-layout check
was requested. No forced role write or fabricated connection was used.

While Jetson was inactive, a separate ten-second vehicle sample measured
72.1% aggregate CPU utilization, with the busiest core at 84.2%. This is the
internal-model baseline, not a Jetson comparison; the vehicle subsequently
rebooted independently. Native camera/USB/map inference, current DM cadence,
combined per-core load, physical display and road behavior remain unverified.

Private scripts/results are in `.analysis/scratch/2026-10-13-jetson-nav/`.

## USB reconnection and HUD failures (same evening)

USB now enumerates correctly and the signed RL Nav runtime became active.
The initial `955a6f2d39` HUD repeatedly failed to import `signal_display`.
`0b027229ba` includes that module, its mode dependency and the navigation preview
helper. Candidate probing now executes `hud.py --help`, not just `import hud`;
Linux CI also tests the actual entry point outside the repository in an exported
bundle. The vehicle boot client reconnects immediately if the gate hands USB to
the runtime before its final status response. A queued boot selection can still
produce one `unknown_message: type 16387` during handoff; this is distinct from
steady-state inference or the HUD failures.

The user explicitly approved signing/publishing/installing `0b027229ba` after
automatic approval review blocked that step. Its immutable HTTPS bundle and
signature were verified, then the normal parked reboot gate applied it and
printed `CANDIDATE_PROBE_OK`. The cached engine was reused without compilation.
Vehicle HEAD remains `5e85695197`; only `startup.py` and `host_release.json` were
installed from `c161d1a00d`, with prior file hashes checked and private backups.
Unrelated concurrent branch corrections were not installed on the vehicle.

The now-running renderer exposed two further failures:

* The immutable boot screen held a deleted `/dev/shm/.../owner.lock` inode while
  the HUD acquired a new inode and failed with USB busy. Login-session cleanup
  is consistent with the observed file removal: services use the same UID but
  remain alive after the last SSH/PAM session. `4300033190` moves ownership to
  the service's `/run` directory (legacy no-service fallback is `/tmp`) and uses
  a boot-local systemd override to run the signed boot-screen implementation.
  The immutable early recovery implementation remains unchanged.
* Once the conflicting boot screen was temporarily stopped, drawing a camera
  frame raised `RemoteRoadCamera.display_frame` AttributeError. `1c164b66bb`
  supplies the expected interface with absent original-frame evidence, and a
  regression exercises the exact signal-overlay renderer call. Compressed
  previews must not fabricate full-resolution signal observation metadata.

Both corrections are committed/pushed on carrot-nav only. At this checkpoint,
automatic approval review rejected signing/publishing the follow-up because it
interpreted the previous approval as limited to `0b027229ba`; a question asking
for follow-up publication/install authorization is pending. Neither `4300033190`
nor `1c164b66bb` is claimed installed. The installed HUD remains unvalidated.

### Live measurements before the final HUD repair

Two 30-second parked windows observed 535 and 537 valid modelV2 messages
(17.8--17.9 Hz), continuous active/ready Jetlink status, 600/601 valid DM outputs
and 600 valid road-camera outputs (~20 Hz). Mean model execution was 51.5--51.6ms,
maximum up to 69.9ms. These are live USB numbers and supersede any inference
that the isolated 20Hz engine probe established full-pipeline 20Hz operation.

A separate 20-second timing sample measured mean host GPU 24.87ms, input queue
2.52ms and host total 33.50ms; USB request send 7.12ms; model process total loop
55.89ms including 3.17ms postprocessing. Mean camera age at run was 51.33ms and
recorded dropped_frames averaged 0.121 per model loop. Scheduler queue waits
were small. The measured end-to-end path misses the 50ms budget; do not change
validity gates or conceal this as a healthy 20Hz result. Candidates for further
profiling are recurrent state validation/copy (~6ms in the isolated probe),
USB request copies/transfers and camera warp/parse. They are not yet optimized.

Separate ten-second CPU samples: comma aggregate 68.2%, cores 0--7
77.3/72.2/79.3/85.0/49.0/56.9/72.4/48.0%; Jetson aggregate 14.8%, cores 0--5
22.0/24.0/18.9/23.0/0.0/0.6%. These include a failing HUD and boot diagnostics,
not the final normal renderer load. Current navInstruction was invalid and the
map preview status was `route`; no live route-map or route-steering validation
is claimed. No closed-loop driving test was performed.

Validation: initial repair 25 desktop tests passed / 4 Linux-only skips;
lock repair 22 passed / 2 skips; camera/lock/export subset 12 passed / 3 skips.
Jetson/eGPU CI passed for `c161d1a00d` and `4300033190`. Private samples and scripts
remain under the scratch directory above, without publishing recordings or keys.

Pending-approval device disposition: inference remains active. The repeatedly
crashing HUD service was stopped for this boot and the boot diagnostic screen
restored; neither service was disabled. Final camera-adapter commit `1c164b66bb`
passed both Jetson integration and eGPU compatibility CI. Deployment remains
pending the follow-up publication approval, not a model build.
