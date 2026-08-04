# Carrot Cluster

Standalone raylib cluster UI bundle for openpilot devices. The normal and road
camera HUDs show available tire pressures around a fixed vehicle diagram beside
the navigation panel; pressures below 31 psi are highlighted in red.

Run from the openpilot root:

```bash
python selfdrive/carrot/cluster_run.py --output usb

python selfdrive/carrot/cluster_run.py --output usb --profile-render
```

Useful options:

```bash
python selfdrive/carrot/cluster_run.py --output window --width 1920 --height 480
python selfdrive/carrot/cluster_run.py --input navi --output window --width 1920 --height 480 --fps 30
python selfdrive/carrot/cluster_run.py --input route --route /path/to/route --navi-overlay --screen-mode navi --output both --width 1920 --height 480 --fps 30
python selfdrive/carrot/cluster_run.py --output usb --live-no-can
python selfdrive/carrot/cluster_run.py --output usb --usb-codec jpeg --usb-jpeg-quality 68
python selfdrive/carrot/cluster_run.py --output usb --input route --route /data/media/0/realdata/0000012e--f190807d64--36 --route-overlay compact --usb-codec h264 --usb-h264-fps 30 --profile-render
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-test-pattern-nv12 --duration 20 --fps 10 --usb-h264-debug --usb-h264-slice-max-bytes 4096
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-backend ffmpeg --usb-h264-ffmpeg-encoder libx264 --usb-h264-test-pattern --duration 20 --fps 10 --usb-h264-debug
python selfdrive/carrot/cluster_run.py --output usb --fps 10 --usb-jpeg-quality 55 --route-overlay off
python selfdrive/carrot/cluster_run.py --output usb --profile-render --profile-interval 2
```

`--input navi` is the standalone Windows/live-device navigation screen. It
binds the Carrot WebSocket v2 receiver on TCP 7714, broadcasts its address on
UDP 7705, decodes MAP MAIN H.264 in-process, and displays all current JSON/PNG
surfaces in a dedicated 1920x480 layout. Use `--navi-advertise-ip 127.0.0.1`
with `adb reverse tcp:7714 tcp:7714`, or omit it for automatic LAN discovery.
Only one receiver can own TCP 7714 at a time.

Navigation map rendering requests `--navi-map-theme dark` by default. Use
`auto` or `light` to request another theme from the smartphone renderer. The
control WebSocket heartbeat is 5 seconds, and a `map_main` stream that stops
for more than 3 seconds is replaced by `MAP STREAM STALLED` instead of leaving
the last map frame frozen on screen.

`--navi-overlay` keeps the selected vehicle source and adds the live Carrot
navigation receiver. This is intended for editing the production navigation
layout on a PC: use `--input route` for recorded vehicle data and `--output
both` to render the same state to the PC window and a connected TURZX display.
The replay wrapper selects the full navigation screen automatically:

```bash
python selfdrive/carrot/cluster_replay_usb.py /path/to/route --navi-overlay --output both --fps 30
```

The replay wrapper opens camera/debug data and seek controls in a separate
`Carrot Cluster Replay Tools` window. The 1920x480 cluster window and USB frame
remain clean and use the same renderer output. Use `--route-tools overlay` for
the previous in-frame controls or `--route-tools off` to disable replay tools.
The maintained cut-in route regression set and expected results are documented
in [CUTIN_VALIDATION.md](CUTIN_VALIDATION.md); its 16 current-code checks can
be run together with `validate_cutin_routes.py` and reviewed sequentially with
`review_cutin_routes.py`.

Pass `--screen-mode default` to inspect the normal HUD with its navigation
panel instead. On a managed device, the permanent `carrot_navi` process owns
TCP 7714 and publishes structured state through `carrotNavi` and binary media
through `carrotNaviMedia`. Cluster autorun launches `--input live` and consumes
both cereal services; it does not create an embedded receiver. The direct
`--input navi` and `--navi-overlay` receiver paths remain available for explicit
CLI runs when TCP 7714 is free.

`--usb-jpeg-encoder auto` tries optional `turbojpeg` first and falls back to
Pillow. Route replay defaults to `--route-overlay compact`, which shows the
right-side debug panel. Use `--route-overlay off` for performance tests that
should match live rendering cost more closely.

`--usb-codec h264` uses the native Qualcomm V4L2 encoder wrapper in
`system/loggerd/encoder` or the ffmpeg/libx264 comparison path. Native H264
renders directly into the Qualcomm/Venus-aligned NV12 layout before submit, so
the cluster hardware path no longer depends on libyuv or a CPU RGBA-to-NV12
conversion. On TICI with the current native bridge, the pipeline leases a cached
ION/V4L2 input buffer. By default GLES imports that DMA-BUF as an ABGR8888
byte-view framebuffer and runs the existing packed-NV12 shader directly into
it. A bounded GLES fence completes before cached-ION synchronization and VIDC
submit. This removes `glReadPixels`, PBO mapping, and the full-frame CPU copy
without changing shader bytes, orientation, encoder geometry, or cadence.
Set `CLUSTER_NV12_DMABUF_OUTPUT=0` to select the persistent three-slot PBO/fence
fallback. An EGL/FBO/fence error also selects PBO automatically. PBO failure
falls back to synchronous direct-ION readback; an incompatible layout or
direct-readback failure retains the staged compatibility path. Set
`CLUSTER_ASYNC_NV12_READBACK=0` to force synchronous direct-ION A/B testing, or
`CLUSTER_DIRECT_NV12_READBACK=0` to force staged readback.
H264 defaults to the same exact portrait upload geometry used by
the working JPEG/PNG and earlier ffmpeg H264 paths. For a 9.2-inch panel that
means a 462x1920 H264 stream, with no 16-pixel render-size padding unless
`--usb-h264-align 16` is passed explicitly. Native hardware encoding pads only
the encoder input to a 16-pixel boundary by default, so 462x1920 display frames
are fed to V4L2 as 464x1920 and cropped back to 462x1920 in SPS metadata.
The default backend is the native Qualcomm hardware path. It patches hardware
SPS Baseline constraint flags to match the libx264 constrained-Baseline stream
that the TURZX panel accepts, and patches hardware SPS frame-crop metadata for
non-macroblock geometry such as 462x1920. It also asks the V4L2 encoder for
multi-slice output capped by `--usb-h264-slice-max-bytes` so the resulting NAL
sizes are closer to the ffmpeg/libx264 stream accepted by TURZX. The default
H264 bitrate is `auto`, which preserves the `7M` at 30 FPS per-frame budget
across the supported live-FPS modes and resolves to `14M` at 60 FPS. The native
default is all-I
(`--usb-h264-gop 1`) because TURZX panel corruption measurements improved as
P-frame references were removed. GOP 3 route replay was much better than the
earlier long-GOP runs, and GOP 2 further improved compact-overlay tests, but a
route replay without the overlay showed frequent small block artifacts at GOP 2.
GOP 1 at 6M removed the visible squares on the same route, with only slightly
softer compression detail, and a follow-up GOP 1 / 7M run also stayed clean, so
GOP 1 is the measured stability default for now.
An explicit `8M` route replay was worse and pushed H264 USB chunk writes into
large latency spikes. That result remains transport evidence, not a reason to
halve the per-frame quality budget at higher FPS. High-FPS testing therefore
keeps proportional bitrate and treats any resulting sender or receiver stall as
the implementation bottleneck. The
larger `--usb-h264-slice-max-bytes 8192` A/B also looked worse than the default
4096-byte slice cap, and `2048` caused smaller but more frequent smearing, so
keep the default slice setting for normal tests. The
hardware V4L2 rate-control default remains `--usb-h264-rate-control vbr-cfr`;
`cbr-cfr` made frequent small blocks and `--usb-h264-realtime-priority` landed
between VBR-CFR and CBR-CFR, so keep both off for normal tests. The
ffmpeg/libx264 path remains available as a known-good comparison path. Normal
TICI (`larch64`) SCons builds include both native bridges. To rebuild only
those targets before hardware testing:

```bash
scons system/loggerd/libcluster_h264_encoder_bridge.so
scons system/loggerd/libcluster_h264_decoder_bridge.so
```

Use `--usb-h264-backend ffmpeg --usb-h264-ffmpeg-encoder libx264` to compare
the known-good software stream, or `--usb-h264-backend auto` to try native and
fall back to ffmpeg.

The default V4L2 device is
`/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1`. Input format
defaults to `nv12`, matching the existing loggerd V4L2 encoder path. Native
NV12 input uses the same Qualcomm/Venus aligned stride, scanline count, and UV
offset calculation as camerad, rather than a compact width-by-height layout.
The previous direct and hidden 32-bit RGB diagnostic input paths have been
removed. The cluster H264 wrapper emits inline SPS/PPS on the first video
packet and on IDR frames, asks for VBR-CFR rate control, constrained
Baseline/CAVLC, and VUI timing when the V4L2 driver accepts those controls, and
the Python sender patches SPS VUI timing and bitstream restriction metadata when
the driver returns a short VUI without timing info. If those baseline controls
are rejected, the native path falls back internally to driver-compatible profile
controls.
`--usb-h264-debug` prints a detailed trace for each early hardware packet:
native callback flags/timestamps/keyframe state, raw and patched NAL summaries,
packetization results, TURZX chunk sizes, and a shutdown summary.
`--usb-h264-diagnose-interval N` prints a compact periodic summary that is less
noisy than debug mode: H264 unit count/keyframes, unit byte rate, chunks per
unit, NAL sizes, native access-unit queue depth/drop count, and USB send latency. Use it on both
native and ffmpeg runs when deciding whether artifacts line up with encoder
output size/cadence or with USB transport stalls.
Keep `--usb-h264-debug` and `--usb-h264-dump` off for FPS/CPU measurements;
they are diagnostic tools and add console/file I/O overhead. The compact
diagnostic log is lighter than debug/dump, but final FPS measurements should
still rerun without it after the suspect interval is identified. With
`--profile-render`, native hardware runs include C++ sub-stage samples such as
`usb_h264.native.convert` and `usb_h264.native.wait_input`.
`--usb-h264-encoder-align 1` disables hardware-only input padding for A/B
testing; the default `16` avoids feeding the Qualcomm encoder a 462-byte NV12
stride while its H264 SPS reports a 464-pixel coded width. In portrait H264
mode, the renderer reads back the aligned encoder size directly so the Python
sender can avoid a per-frame RGBA padding copy while SPS crop metadata keeps
the panel display at the requested 462-pixel width.
`--usb-h264-slice-max-bytes 0` disables the hardware multi-slice request.
Native hardware output is sent as encoder access units, matching the
known-good ffmpeg/libx264 command boundary. The TURZX H264 command `last` flag
is left off to match the working software path.

For a quick H264 transport smoke test, run:

```bash
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-test-pattern-nv12 --duration 20 --fps 10 --usb-h264-debug --usb-h264-slice-max-bytes 4096
```

The panel should show red/green/blue/white quadrants on the default NV12
hardware path.
`--usb-h264-orientation landscape` tests direct 1920x462 output, while
`--usb-h264-align 16` deliberately tests macroblock-aligned output such as
1920x464. When `--fps` is omitted, non-live H264 USB runs use
`--usb-h264-fps 30` as the render cap; live H264 runs follow
`ClusterHudLiveFps`. The TURZX display frame-rate command follows the effective
H264 FPS unless `--usb-display-fps 0` is passed explicitly. H264 chunks are no-ACK by
default like JPEG frame uploads; use
`--usb-h264-wait-ack` for strict response diagnostics, or
`--usb-h264-soft-ack` to mimic the vendor video sender's retry/status polling
without failing the run. If the hardware stream is still corrupted, rerun with
`--usb-h264-debug --usb-h264-dump /tmp/cluster_hw_native.h264` and keep the
native packet, packetize, chunk, and final summary lines. Then retry
`--usb-h264-slice-max-bytes 2048` and `1024`; the debug NAL summary should show
several smaller IDR/P NALs instead of one large slice. For 462x1920 streams,
the SPS summary should show `display=462x1920` rather than only the coded
464-pixel macroblock width.
If the hardware SPS summary shows `vui=0`, `timing=0`, or `timing=?`, the
default patch rebuilds SPS VUI timing and bitstream restriction info to match
the selected H264 FPS and the libx264-style no-reorder DPB metadata.

For route replay against a saved device route, run:

```bash
python selfdrive/carrot/cluster_run.py --input route --route /data/media/0/realdata/0000012e--f190807d64--36 --route-overlay compact --output usb --usb-codec h264 --duration 60 --fps 30 --profile-render --profile-interval 2
python selfdrive/carrot/cluster_run.py --input route --route /data/media/0/realdata/0000012e--f190807d64--36 --route-overlay off --output usb --usb-codec h264 --duration 60 --fps 30
python selfdrive/carrot/cluster_run.py --input route --route /data/media/0/realdata/0000012e--f190807d64--36 --route-overlay off --output usb --usb-codec h264 --duration 60 --fps 30 --usb-h264-bitrate 6M
```

For a PC-connected USB cluster display, the shorter replay wrapper defaults to
full rlog data, portable JPEG USB output, a local PC window, and the compact
route camera/data overlay. The camera overlay uses `ffmpeg` from PATH when
available and falls back to the `imageio-ffmpeg` package from requirements:

```bash
python -m pip install -r selfdrive/carrot/cluster/requirements.txt
python selfdrive/carrot/cluster_replay_usb.py /data/media/0/realdata/0000012e--f190807d64--36 --duration 60
python selfdrive/carrot/cluster_replay_usb.py /data/media/0/realdata/0000012e--f190807d64--36/rlog.zst --fps 20 --usb-brightness 80
python selfdrive/carrot/cluster_replay_usb.py /data/media/0/realdata/0000012e--f190807d64--36 --trip-report
```

From a Windows checkout whose repository root contains the `openpilot`
directory, use:

```powershell
.venv\Scripts\python.exe openpilot\selfdrive\carrot\cluster_replay_usb.py W:\routes\vehicle\segment\rlog.zst --output both
```

The default `--corner-source live` displays the `liveTracks` actually
published by the device. Use `--corner-source stable` to reconstruct
physically continuous corner tracks from raw CAN, or `--corner-source raw` to
show the untracked CAN slots. Comparing `live` and `stable` is useful when a
recorded cluster display is missing a corner object.

Use `--route-overlay full` for a larger replay debug panel, or
`--output usb --route-overlay off` when only the USB panel should be driven.
Use `--corner-yaw-comp-gain 0.6` to add replay-only corner-radar yaw
compensation, or a negative value to reduce compensation already present in
the logged `liveTracks` data.

On Windows, if the display is detected but opening it fails with access denied,
install a WinUSB driver for the TURZX device with Zadig: enable
`Options > List All Devices`, select the `1CBE:0092` or `1CBE:0123` display,
install `WinUSB`, then unplug/replug the display.

To compare native hardware output against ffmpeg/libx264 with the same USB
transport diagnostics, use:

```bash
python selfdrive/carrot/cluster_run.py --input route --route /data/media/0/realdata/0000012e--f190807d64--36 --route-overlay compact --output usb --usb-codec h264 --duration 30 --fps 30 --profile-render --profile-interval 2 --usb-h264-diagnose-interval 1
python selfdrive/carrot/cluster_run.py --input route --route /data/media/0/realdata/0000012e--f190807d64--36 --route-overlay compact --output usb --usb-codec h264 --usb-h264-backend ffmpeg --usb-h264-ffmpeg-encoder libx264 --duration 30 --fps 30 --profile-render --profile-interval 2 --usb-h264-diagnose-interval 1
```

The ffmpeg/libx264 path is the known-good H264 comparison mode. To make that
explicit while testing, run:

```bash
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-backend ffmpeg --usb-h264-ffmpeg-encoder libx264 --usb-h264-test-pattern --duration 20 --fps 10 --usb-h264-debug
```

When the panel still shows a corrupted picture, dump the outgoing stream and
compare it separately:

```bash
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-test-pattern-nv12 --duration 20 --fps 10 --usb-h264-debug --usb-h264-dump /tmp/cluster_hw_nv12.h264
ffprobe -show_streams /tmp/cluster_hw_nv12.h264
```

If the dump plays correctly but the panel is corrupted, the remaining issue is
TURZX stream compatibility or USB flow control. If the dump is corrupted too,
the issue is in the V4L2 NV12 submit path or encoder controls.

Keep `--usb-h264-input-format nv12` for native hardware testing. Direct RGB
USERPTR diagnostics were removed after measured device tests showed corrupted
output across direct and hidden 32-bit RGB variants.

Manager autostart omits `--fps` by default so live launches follow
`ClusterHudLiveFps`. JPEG/PNG runs apply setting changes while running; H264
runs exit and let `cluster_autorun` relaunch when the setting changes the
encoder FPS because the V4L2 encoder timing, SPS timing, and automatic bitrate
are fixed at startup. Set `CLUSTER_AUTORUN_FPS` only for fixed test overrides;
`0` means uncapped.
`ClusterHudDebug` controls the autorun output gate: `0` starts external HUD
rendering only while openpilot is onroad, and `1`, `2`, and `3` keep the
always-on debug behavior after power-up. In live input only, `2` also keeps the
top UI icons visible when source data is missing, and `3` also shows the navi
debug UI before navi data has arrived. Normal mode checks the onroad gate every
100 ms; when output is gated off, `cluster_autorun` sends TURZX brightness `0`
so a stale HUD frame does not remain visible.
The autorun watcher normalizes locale before this dim-only USB path too, so
vendor USB initialization does not fail before the renderer is launched.
Manager autostart always configures the cluster process through openpilot's
realtime helper. `ClusterHudCoreMode=0` maps to cores `1,2,3,4`, while mode `1`
maps to all initially allowed CPU cores. `ClusterHudPriority` always controls
the FIFO priority with range `1..99`, default `10`; `CLUSTER_REALTIME` is no
longer read.
Changing either param makes the running HUD exit so `cluster_autorun` can
relaunch it with the new affinity/scheduler settings, without a whole system restart.
Explicit `CLUSTER_REALTIME_CORES` or `CLUSTER_REALTIME_PRIORITY` environment
values still override the corresponding Params.
The HUD reads the local Git branch immediately on every platform and refreshes
the upstream update state asynchronously at most every 60 seconds. The Git
worker changes itself to `SCHED_OTHER` before `ls-remote`/`fetch`, so TICI does
not run those commands in the render process's FIFO scheduling class.
Native H.264 callback output is queued as complete access units. The bounded
queue retains the latest codec config, keyframe, and frame without waiting for
USB; stale access units are dropped and reported instead of failing the run.
When `--usb-brightness` is omitted, USB launches follow `ClusterHudBrightness`:
`0` auto follows live `wideRoadCameraState.exposureValPercent` after samples are
available, falling back to `deviceState.screenBrightnessPercent`; `1` through
`100` are fixed brightness percentages. `ClusterHudOrientation` supports `0`
(0 degrees) and `2` (180 degrees); values `1` and `3` are ignored. The existing
web settings UI stores both Params without a custom slider path. The running
HUD checks the stored brightness and orientation every 100 ms. Brightness
applies without restarting. A managed H.264 orientation change exits cleanly
and autorun relaunches immediately. The new stream uses the captured
`10, 111, 112, 13, 14, 52, 102, 15, 17` setup sequence, including the selected
raw orientation in command `13`.

Changed display settings follow the capture-derived command procedure:

- Brightness: command `10` (sync), wait about 20 ms, then command `14` with
  byte 8 set to `int(percent / 100 * 102)`.
- Screen rotation: command `10` (sync), wait about 20 ms, then command `13`
  with byte 8 set to supported raw orientation `0` or `2`.

The sync and setting write are one USB-locked transaction so an image frame
cannot split the pair. Both runtime writes are nonblocking on TICI; pending
responses are drained by the next bounded USB operation, so a missing sync ACK
cannot terminate the HUD. Initial orientation is stored locally before USB
open and carried by H.264 setup command `13`. H.264 startup waits for each captured
setup delay, uses captured finalizer command `52` instead of the
reference-library command `41`, clears the 464x1920 overlay, then applies FPS
and queries the chunk size. Setup writes remain nonblocking on TICI and drain
pending responses before subsequent writes; waiting synchronously for every
ACK prevented the H.264 stream from starting. Shutdown sends command `123`
followed by two bounded command-`122` status drains before releasing USB. The
panel does not visibly apply command `13` during an active H.264 stream, so
managed H.264 uses the automatic restart described above.
`--usb-h264-orientation` remains a separate diagnostic option controlling
encoder/render geometry.

The launcher defaults to `--input live`, subscribes to openpilot cereal services,
and renders live `carState`, `modelV2`, `radarState`, `liveTracks`,
`controlsState`, `selfdriveState`, `carControl`, and `deviceState`. Front radar
tracks normally come from `liveTracks`; Hyundai CAN-FD corner object frames
`0x235` through `0x248` on A-CAN (`src % 4 == 1`) override those radar points
when present. Manager/autostart leaves
the live CAN/sendcan subscriptions enabled, but exact LF/RF/LR/RR rear and side
corner distance remains on received Hyundai camera-bus `can` `0x162`/`0x1EA`
messages (`src % 4 == 2`). `sendcan`, ECAN copies generated by
`hyundaicanfd.py`, and returned/rejected `can` echo frames with `src >= 0x80`
are ignored for direct corner parsing so sent presentation frames do not
re-enter as received distance. `--live-no-can` remains a manual diagnostic
option; without raw received CAN, `carState` still provides LF/RF distance and
LR/RR distance when the current cereal schema exposes it. Blindspot booleans do
not create fallback vehicle boxes.
Cluster road speed-limit display treats `carState.speedLimit` from the
vehicle/HDA path as km/h. Navigation speed limits are accepted in either the
km/h values used by the current navigation integrations or the m/s values used
by upstream `navd`; km/h-looking values such as 50/100 stay unchanged. Empty
navigation speed-limit samples do not immediately clear the last valid
navigation limit; the cluster holds it briefly to avoid `n` source flicker
between `--` and the real limit during nav update timing gaps.
Turn-signal arrows are hidden while off and only draw during their blink-on
phase. The top HUD also uses `carState.gearShifter`, `gearStep`, `pcmCruiseGap`,
`selfdriveState.personality`, and `carControl.latActive` to show gear
(`P/R/N/D/1-8`) in a smaller transparent rounded-square outline, front gap bars,
cruise set speed, and the LFA active icon. This top
drive-status row uses the same top margin as the road speed-limit sign while
bottom-aligning gear, gap, cruise set, and LFA elements to the measured
bottom of the cruise-set text. The gap vehicle uses
`selfdrive/assets/icons_mici/carrot_cruse_gap_trimmed.png` at its source aspect
ratio and is taller than before while the gap bars keep their own size/spacing;
all four gap bars stay visible, sit close together, and bottom-align to the
vehicle while inactive bars are gray and active bars use `#bb3d91`. Cruise set
speed and `km/h` use the same font size and color; paused cruise keeps the set
speed but draws it gray, and inactive cruise draws gray `--- km/h`. The orange
deceleration override keeps the selected `carrotMan.desiredSource` control
value unchanged but annotates its displayed origin: navigation camera `cam:n`,
vehicle/HDA camera `cam:v`, vehicle-side route curvature `route:v`, and
comma-model turn prediction `turn:c`. Navigation TBT turn control displays
`turn:n`; the label font scales down slightly for longer values such as
`section:n` instead of dropping the origin suffix. The separate lane-change
icon is not drawn. The LFA icon uses
`selfdrive/assets/icons_mici/carrot_wheel_org.png`, rotates by
`-carState.steeringAngleDeg`, and recolors its white pixels green when LFA is
active. When `controlsState.activeLaneLine` is true, the fixed
`carrot_wheel_lane.png` left/right lane overlay is drawn in the same position
and color contract as the C4 HUD.
When `carState.evModeValid` and `carState.evModeActive` are both true, the HUD
shows a compact green `EV` indicator between the vehicle speed and cruise-set
speed in the normal HUD. Full navigation mode intentionally omits it. It is
also omitted when the decoded EV state is off, or for unsupported vehicles,
invalid samples, and stale `carState` data.
Hyundai/Kia CAN-FD hybrids enable this capability only when both `0xFA` and
`0x230` are present on ECAN with DLC32. The four-bit power-flow mode in `0x230`
shows `EV` for observed values 1, 2, and 6; all other values remain hidden.
The normal HUD also shows the current `longitudinalPlan.myDrivingMode` beside
the model traffic-state icon above vehicle speed: `1` eco in green, `2` safe
in orange, `3` normal in white, and `4` high speed in red. Unknown modes and
missing, invalid, or stale longitudinal plans hide the badge. The red and
green traffic-state icons share the slot immediately to its left and remain
independent of driving mode. Full navigation mode omits both speed-mode
indicators.
The normal and road camera HUDs use the same fixed toy-car TPMS diagram below
the acceleration, steering, fuel, and DEF gauges. Its transparent PNG is loaded
into one GPU texture at renderer startup, then each unchanged-size live pressure
value is drawn inside its corresponding enlarged tire. It remains hidden only
when all four pressure values are unavailable; individual missing values show
`--`, and values below 31 psi are red. The surrounding area stays transparent. When
external navigation is active or its dashboard is connected, the green `NAV`
status appears below the Wi-Fi icon instead of the former lower-right `NAVI`
label. The center clock, EV indicator, and fuel/DEF gauges are unchanged.
When `--fps` is omitted, `ClusterHudLiveFps` controls the render limit and is
polled about once per second while running: `0` uncapped diagnostic mode, `1`
10 Hz default, `2` 20 Hz, `3` 30 Hz, `4` 40 Hz, `5` 50 Hz, and `6` 60 Hz.
Direct route/replay CLI runs also apply nonzero values; mode `0` keeps non-live
H264 runs on the `--usb-h264-fps` safety cap. Explicit `--fps` remains a fixed
override. For H264 USB output, changing the effective FPS exits the current HUD
process so autostart can relaunch with a matching encoder FPS when a launcher
is present.

`ClusterNaviMapFps` independently controls the Android MAP MAIN request: mode
`0` is 5 Hz, `1` is the 10 Hz default, `2` is 20 Hz, and `3` is 30 Hz. At
960x540 these modes select 1500, 3000, 3000, and 6000 kbps respectively; other
resolutions scale the selected bitrate by pixel count within the protocol's
1..12000 kbps bounds. There is no user-facing map bitrate setting. A changed
MAP mode reconnects the Android navigation stream with a new manifest without
changing the HUD FPS.
Runs also show a compact lower-right cluster-process CPU overlay by current
core, formatted like `[0(10),1(25)]`, with 2 px bottom/right margins. The
sampler reads the current cluster process and direct child processes only,
avoiding a full `/proc` PID scan in the render loop. Use
`--cluster-core-usage-debug` with `--profile-render` to log the sampler scan
cost plus per-process/core CPU breakdown, or `--no-cluster-core-usage` for an
A/B run without the overlay.
`ClusterHudEncoder` controls the encoder used by manager autostart and by
direct USB CLI runs when `--usb-codec` is omitted: `0` auto tries
native hardware H264 first, then ffmpeg/libx264 software H264, then JPEG when
launched by `cluster_autorun`. Autorun advances that sequence only when pipeline
initialization fails; renderer, source, encoder-runtime, and USB errors exit the
run for supervisor retry without silently changing codec. Direct CLI auto uses native hardware H264 as the
first encoder choice. `1` forces JPEG, `2` forces native hardware H264, and `3`
forces ffmpeg/libx264 software H264.
Native hardware H264 always uses GPU NV12 packing. Supported TICI builds report
`dmabuf_output=on` and submit the leased encoder input without readback or an
intermediate copy. They also report the availability of `async_pbo` and
`direct_ion` fallbacks. Unsupported builds use the first available fallback. If backend
`auto` falls back to ffmpeg, the run uses the software RGBA pipe.
Changing this setting while the HUD is running makes the current HUD process
exit so `cluster_autorun` can relaunch it with the new encoder choice.
`ClusterHudScreenMode` controls the right-side content. `-1` uses the entire
display for either 3D camera view, suppresses information panels, and balances
the clock, side gauges, TPMS, traffic image, turn signals, and status footer
across the full width. In road-camera view it behaves exactly like mode `0`.
Mode `0` is the default mode that switches between navigation and the driving
report. While onroad, shifting into park (`P`) temporarily gives the completed
driving report priority over active navigation; leaving park restores navigation
immediately. Explicit modes such as report mode `5` and navigation mode `6`
remain fixed. Mode `1` shows the live debug panel with grouped `LIVE DELAY`, `LIVE TORQUE`,
`STEERING`, and `LATERAL PLAN` rows, `2` is the system-debug slot rendering commit
`c0a6773f794a5e4e86aeca8e14515232abc26b1b`'s mode-0 default system screen,
`3` shows a large debug graph selected by `ShowPlotMode` with the driving scene
disabled, and `4` shows the same graph in the information panel while keeping
the driving scene. Mode `4` keeps the acceleration, steering, fuel, and DEF
gauges immediately to the left of the graph instead of near the center of the
driving view; the gauge block follows the graph when the panel layout is
swapped, while TPMS remains with the driving view.
`5` shows the driving report in the information panel while keeping the driving scene. The
report uses a large trip/event summary card and a separate system-load card
with four 2-by-2 circular gauges. A lower target plots stored calibration pitch
vertically and yaw horizontally around the calibrated center while retaining
the numeric angles. In managed live input, trip statistics remain stopped until
`deviceState.started` is true, reset and start on that transition, freeze
immediately when it becomes false, and reset again at the next onroad start.
Replay and direct parser inputs retain their existing accumulation behavior.
The same mode can be validated with
`cluster_replay_usb.py ROUTE --trip-report`.
`ClusterHudPanelLayout=0` keeps the driving view on the left and the current
information panel on the right. Value `1` swaps the two regions without
restarting the HUD. The information region includes screen-mode debug panels,
the driving report, route diagnostics, and live navigation, including panels
made visible by `ClusterHudDebug`. Full-screen graph and navigation modes are
not rearranged. Route replay can validate the swapped layout with
`cluster_replay_usb.py ROUTE --trip-report --panel-layout driving-right`.
The renderer polls `LanguageSetting` and `IsMetric` about once per second.
Korean (`ko`) and English (`en`) localize driving-report, driving-mode, and
navigation status labels; unsupported language values fall back to English.
Metric mode renders speed/distance as `km/h`, `m`, and `km`, while imperial
mode converts the same internal kph/metre state to `mph`, `ft`, and `mi`.
Vehicle speed, cruise/override/limit values, navigation, radar labels, and the
trip report all use the selected units; acceleration and temperature remain
`m/s²` and `°C`. Route replay defaults to Korean/metric and can validate the
other presentation with
`cluster_replay_usb.py ROUTE --trip-report --language en --imperial`.
In default screen mode (`0`), the trip report is shown while no live navigation
is being received and the navigation panel returns automatically when reception
starts. System-debug mode (`2`) reproduces the reference commit's mode-0 system
screen and does not use the current automatic report fallback. It keeps the
navigation/disconnected-system panel whenever a navigation dashboard exists,
and falls back to the route overlay only when no navigation panel source exists.
Fullscreen-3D mode (`-1`) never reserves a navigation/report panel in either 3D
camera view, even when Navi data is available. Switching to road-camera view
re-enables the complete mode-0 panel selection and panel-layout behavior.
Left-edge HUD items keep their normal margins, right-edge gauges and TPMS keep
their small-3D-view right margins at the physical display edge, and the clock,
world, and turn signals use the full-display center axis.
Mode `5` keeps the branch, network address, and
frame-rate status in the lower-left camera area while omitting the lower-right
core-usage text that would overlap the report. In road-camera view, ungrouped radar detections are projected
as small transparent rounded source-colored markers, while detected vehicles
are enclosed by larger transparent rounded frames using their existing
detection colors. Vehicle frames use a single low-segment outline, ignore noisy
radar-derived yaw when calculating their screen width, and are discarded before
drawing when an incomplete or edge-clipped projection would create a stretched
frame.
Mode `3` also hides the speed, accel, clock, turn-signal, and git HUD so the
large graph uses the available center/right height with only a small margin.
Mode `4` keeps the driving HUD and uses the maximum right-side panel height with
the same margin. Modes `1`, `2`, `3`, `4`, and `5` suppress the route overlay so the
selected debug view remains visible.
`ClusterHudRadarInfo` controls world radar/vehicle speed and distance labels:
`0` off, `1` speed for vehicle boxes only, `2` speed and distance for vehicle
boxes only, `3` speed for all vehicle boxes and raw radar points, and `4` speed
and distance for all. `ClusterHudRadarDisplay` controls raw radar point
presentation: `0` averages nearby points with nearly matching speed/position,
and hides raw radar vehicle boxes that overlap already-rendered detected
vehicles such as front-center `radarState` leads; `1` leaves raw points
unmerged for detail checks, including radar vehicle candidate boxes and
radar-to-detected-vehicle speed merges. Vehicle/radar metric labels sit closer
to the point/box top so speed and distance are less high above the vehicle.
`LR`/`RR` rear-corner detections render as normal vehicle boxes at their actual
detected positions. The older fixed rear-tire-depth 2D arrow/label is removed.
The default drive camera sits closer to the ego roof, lower than the earlier
high view, tilted downward, shifted `5m` forward, and uses a `31` degree
vertical field of view so nearby vehicles retain a useful apparent size.
Detected vehicles, radar points, desired-distance markers, model lane markings,
the model-derived lane-change floor, model road edges, and the model planned
path compress signed longitudinal placement by `0.5` in the rendered 3D scene,
so actual `20m` and `-10m` draw at rendered `10m` and `-5m`. Road-camera mode
keeps those overlays at `1:1` for image alignment. Distance labels and all
lateral coordinates keep their actual values. The ego vehicle is drawn half a
vehicle length behind the raw `0m` reference so its front bumper aligns to that
reference. The temporary radar-zero, lane-start, and ego-zero debug marker bars
are no longer rendered.
`ClusterHudCameraViewMode=0` keeps this current camera. Mode `1` uses a
pulled-back ego-bottom camera view for cars without rear radar.
The console refresh line prints `cam=<mode>` so live param changes can be
confirmed while the HUD is running.
When both raw camera-bus ADRV `0x1EA` and CCNC `0x162` corner messages are
fresh, ADRV is preferred for LF/RF/LR/RR distance in the Hyundai `carState`
DBC parsing path. The cluster consumes the DBC-parsed `carState` corner fields
first; route replay/raw-CAN fallback also decodes `0x162`/`0x1EA` through the
Hyundai CAN-FD DBC instead of hand-coded bit positions.
Road/lane/radar geometry still starts far enough behind that rendered bound so
the rear lane-start seam sits below the visible bottom edge.
Front-center `radarState` lead overlap uses a wider vehicle-sized tolerance
than corner radar overlap, and default mode also collapses overlapping
front-center detected vehicle boxes so source-split front radar/model
reflections merge cleanly.
`ClusterHudRadarSourceColor` controls vehicle box colors:
`0` keeps all vehicle boxes gray, while `1` uses source colors: radar track
vehicles yellow, `radarState` front/SCC radar leads red, camera-sourced vehicle
leads light blue, comma model leads dark blue, and ADAS corner detections from
`0x162`/`0x1EA` green.
Radar samples whose distance and left/right offset are both zero are treated as
empty/default data and are not drawn as radar points or vehicle boxes.
Radar-track vehicle classification rejects points outside model road edges, but
does not require in-road points to sit near the road-edge line; center-lane
points can classify as vehicles when probability/in-lane data or moving radar
radar evidence is sufficient. Points near or slightly outside a road edge can
still classify as vehicles when their counter is stable, absolute speed is
vehicle-like, and acceleration stays within about +/-5 m/s^2, even if the radar
radar probability is low.
Measured radar tracks with at least `12 km/h` of combined longitudinal/lateral
motion remain visible as vehicles outside lane or road-edge boundaries, while
slower boundary reflections stay filtered. Signed negative `vLead` values are
preserved; oncoming tracks at or below `-12 km/h` render red. Tracks with at
least `12 km/h` combined motion rotate their vehicle box to follow the measured
longitudinal/lateral velocity vector, including after radar/model merging.
Stopped and slow vehicles remain aligned with the road. Tracks with at least
`2.0 m/s` lateral velocity render amber.
Lane and road-edge rendering keeps model geometry visible instead of filtering
by `laneLineProbs` or `roadEdgeStds`, avoiding distracting HUD flicker when
model confidence jitters. Lane markings are still suppressed when their
lateral offset falls outside a valid model road-edge boundary. Dashed lane
markings are phased so the visible rear bound starts with a line segment
instead of a gap.
When `carState.leftLaneLine` or `carState.rightLaneLine` carries camera/CAN
lane color codes, the current lane markings use that color first
(`+10=white`, `+20=yellow`) before falling back to the cluster model colors.
The planned path draws `longitudinalPlan.desiredDistance` as a magenta
horizontal bar across the current lane width at the matching forward position.
Changing `ClusterHud` away from supported mode `1`, including legacy value `2`
or `0`, makes the running HUD exit; cleanup sends TURZX brightness zero before
releasing the USB device. When autorun passes HUD mode `1`, USB open is pinned
to TURZX PID `0x0092` so a second connected TURZX panel is not opened by the
vendor library's generic device scan.
If frame or H264 chunk writes report that the USB device was disconnected, the
active HUD exits instead of trying to recover in-process. Autorun calls the
launcher in non-exiting mode so the error returns to the watcher loop, letting
`cluster_autorun` wait for the same PID and relaunch after replug.

The bundled TURZX code includes only the Python vendor library. The openpilot
device uses the system `libusb-1.0.so` through `pyusb`.

The renderer prefers
`/data/openpilot/selfdrive/assets/fonts/KaiGenGothicKR-Bold.ttf` for HUD text.
It falls back to the bundled/addon KaiGen copy, then JetBrainsMono and
system/platform fonts if KaiGen is not present.
Latin/numeric text uses the 160-pixel primary font. The complete Korean glyph
set uses a separate 32-pixel base atlas with bilinear filtering and no mip chain;
a host resource smoke produced `8192x4096` rather than the former `8192x8192`.
Dynamic stroked HUD text keeps the original eight outline draws plus one fill
draw, but calls the same raylib `DrawTextEx` C symbol directly after preparing
the font, measurement, UTF-8 text, anchor, and colors once. Set
`CLUSTER_RAW_STROKED_TEXT=0` to restore the pyray-wrapper path for an A/B or
recovery run. The switch does not select a different shader, glyph atlas,
outline algorithm, resolution, or text update cadence.
Incoming Navi H.264 is decoded on a bounded worker. The default path attempts
Qualcomm VIDC `/dev/video32` on every platform to produce leased linear NV12
DMA-BUFs, imports them as EGLImages, and composes them through an external OpenGL
texture without a decoded-pixel CPU copy or texture upload. Decoder-generation
changes discard stale EGLImages before importing the new capture pool. If native
decode or EGL import fails, the worker logs the reason and lazily falls back to
PyAV YUV420P; platform markers do not control selection. Set
`CLUSTER_HARDWARE_H264_DECODE=0` for an explicit software A/B. Diagnostic
overrides are `CLUSTER_H264_DECODER_LIBRARY`, `CLUSTER_H264_DECODER_DEVICE`,
`CLUSTER_H264_DECODE_TIMEOUT_MS`, and `CLUSTER_H264_DECODER_DEBUG=1`.

If the decode worker falls behind, dependent frames are discarded until the
next keyframe, and stale completed frames cannot replace a newer requested
sequence. The PyAV fallback preserves YUV420P plane strides and uploads three
reusable plane textures rather than expanding to RGBA.

USB frame upload runs in no-ACK mode by default because some TURZX panels accept
image data but never return a frame-upload response. Use `--usb-wait-frame-ack`
only when testing a panel/driver combination known to reply after each frame.
