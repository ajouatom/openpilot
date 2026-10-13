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
