# Jetson dedicated-appliance boot investigation

Scope: carrot-jetlink, Orin Nano Super on Jetson Linux R36.4.7, SD root
filesystem. Keep C4 native fallback, model artifacts, validity gates and USB
deadlines unchanged. Runtime timing comparisons are parked C4 observations.

## Baseline and ordering defect

The initial real reboot recovered external inference after66.73 seconds as
sampled on C4. This includes shutdown, firmware, boot and model initialization;
it is not a cold-power-on measurement. On Jetson's new boot clock, systemd
reported8.555 seconds kernel +14.984 seconds userspace, or23.539 seconds to
startup completion. The inference process started at18.199 seconds, engine
ready at36.112 seconds and actual USB/H.264 HUD output at44.356 seconds.
Firmware time is not included in systemd's reported23.539 seconds.

The HUD's dedicated Xorg started before nvpmodel. Its first GPU use created
the golden context, and nvpmodel failed with a reboot-required warning when
trying to apply the saved MAXN_SUPER mode. A mode query still said MAXN_SUPER;
that label alone did not establish successful application of all settings.
The Xorg unit now waits for and requires the performance unit, which in turn
requires nvpmodel. It cannot race GPU initialization ahead of the power setup.

`configure_fast_boot.py` adds these ordering requirements to existing installs,
adds kernel `quiet`, and changes the extlinux menu from30 to10 tenths of a
second. It preserves kernel arguments and a recovery-menu interval. Original
files are retained in `/var/lib/carrot-jetlink/boot-config-before.json`;
`--restore` restores those files. The installer templates also carry the
ordering requirements for new hosts.

The ordering/console/menu-only reboot succeeded: nvpmodel completed at17.094
seconds, with no golden-context error. Kernel8.411 + userspace14.680 =23.092
seconds; engine ready36.011 seconds. This fixes initialization correctness,
but does not materially reduce engine readiness by itself.

## Dedicated application cleanup

The root filesystem is on SD (`mmcblk0p1`); the compiled TensorRT engine is
about736 MiB. Initial server import and cold engine load compete with startup
services. Snap seeded/service took about8 seconds each and Docker4.5 seconds
in `systemd-analyze blame`; these overlap and must not be summed as savings.

The user explicitly approved the reviewed93-package removal plan after the
automatic approval review required concrete package authorization. The plan
covers desktop, office/mail/media/games, Docker, printing, Bluetooth, modem
and NoMachine applications. `trim_jetson.py` previews the apt transaction by
default; `--apply` performs it on a dedicated Jetson. Unexpected dependency
removals or existing Docker containers abort the operation. It never runs
autoremove. NVIDIA/CUDA/TensorRT, Xorg, FFmpeg, USB, Python, Wi-Fi and SSH
dependencies are retained and marked manual. Package versions and the plan
are saved in `/var/lib/carrot-jetlink/trim-before.json`; Snap removal snapshots
are copied outside snapd's package-owned directory before its purge.

## Deployment commands

Run only on a prepared, dedicated Jetlink host, with the vehicle stopped and
disengaged for disruptive work. Inspect the cleanup plan before applying it
to another owner's machine; the author's93-package approval is not a general
authorization to erase applications on other hosts.

```sh
python3 tools/jetlink/trim_jetson.py
sudo python3 tools/jetlink/trim_jetson.py --apply
sudo python3 tools/jetlink/configure_fast_boot.py
```

## Cleanup result

The reviewed transaction completed:93 apt packages removed and one dependency
(`policykit-1-gnome`) installed by apt. The nine listed Snap applications/bases
were removed separately, with removal snapshots preserved outside snapd.
No Docker containers existed. The final preview reports no remaining selected
packages or Snap applications. No package autoremove was performed; dependencies
used outside apt's dependency graph by the Python/GPU runtime remain available.

| Milestone, Jetson boot clock | Original | Ordering/menu only | After cleanup |
| --- | ---: | ---: | ---: |
| Kernel + userspace startup | 23.539 s | 23.092 s | 16.485 s |
| Inference process starts | 18.199 s | 17.328 s | 15.889 s |
| TensorRT engine ready | 36.112 s | 36.011 s | 31.084 s |
| First USB/H.264 HUD output | 44.356 s | about44 s | 39.626 s |

The cleanup removed about7 seconds from Linux startup and5 seconds from engine
readiness in these individual boots. Engine preloading still took about10.36
seconds; this contains file reading and GPU/runtime initialization and is not
an isolated SD throughput measurement. C4's sampled external-inactive interval
during the final reboot was about62.00 seconds, down from66.73 seconds in the
baseline. This includes shutdown and firmware time. It is not a62-second gap
in C4 inference: C4 used its native fallback while waiting. The initial IPC
timeout still caused one three-output frame gap, as in the preceding failure
tests; no deadline was changed to hide it.

All four dedicated services, Wi-Fi and SSH returned automatically; nvpmodel
completed successfully and `systemctl --failed` was empty. MAXN_SUPER mode2
was retained. These are individual parked-device boots, not a statistically
established maximum or a7-second power-to-inference result.

The final60-second steady observation recorded1,200 models,60/60 active
external-state samples, no frame-ID gaps, no camera SOF interval above75 ms,
and no message, pose-input/sensor/posenet or CAN invalidity. Model execution
mean37.853 ms, p9941.553 ms, maximum44.449 ms; no sample exceeded50 ms.
This short run does not supersede earlier observations above50 ms or establish
a hard deadline. DisableDM=2 and ClusterHudScreenMode=0 were preserved.
Focused tests cover boot-argument preservation/idempotence, rejecting unknown
boot layouts, read-only cleanup preview, and refusing unexpected runtime or
Snap removals and unrecognized apt output.

The upstream Jetlink package remains unmodified. Firmware reflashing, miniUEFI
and kernel replacement are separate work and were not performed. NVIDIA's
[boot-time guidance](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/BootTimeOptimization.html)
describes additional firmware and kernel customization; its AGX Orin login
timings are not an Orin Nano inference-ready guarantee.
