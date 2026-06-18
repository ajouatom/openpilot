# Carrot Cluster

Standalone raylib cluster UI bundle for openpilot devices.

Run from the openpilot root:

```bash
python selfdrive/carrot/cluster_run.py --output usb

python selfdrive/carrot/cluster_run.py --output usb --profile-render
```

Useful options:

```bash
python selfdrive/carrot/cluster_run.py --output window --width 1920 --height 480
python selfdrive/carrot/cluster_run.py --output usb --live-no-can
python selfdrive/carrot/cluster_run.py --output usb --usb-codec jpeg --usb-jpeg-quality 68
python selfdrive/carrot/cluster_run.py --output usb --input route --route /data/media/0/realdata/0000012e--f190807d64--36 --route-overlay compact --usb-codec h264 --usb-h264-fps 30 --profile-render
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-test-pattern-nv12 --duration 20 --fps 10 --usb-h264-debug --usb-h264-slice-max-bytes 4096
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-backend ffmpeg --usb-h264-ffmpeg-encoder libx264 --usb-h264-test-pattern --duration 20 --fps 10 --usb-h264-debug
python selfdrive/carrot/cluster_run.py --output usb --fps 10 --usb-jpeg-quality 55 --route-overlay off
python selfdrive/carrot/cluster_run.py --output usb --profile-render --profile-interval 2
```

`--usb-jpeg-encoder auto` tries optional `turbojpeg` first and falls back to
Pillow. Route replay defaults to `--route-overlay compact`, which shows the
right-side debug panel. Use `--route-overlay off` for performance tests that
should match live rendering cost more closely.

`--usb-codec h264` uses the native Qualcomm V4L2 encoder wrapper in
`system/loggerd/encoder` or the ffmpeg/libx264 comparison path. Native H264
renders directly into the Qualcomm/Venus-aligned NV12 layout before submit, so
the cluster hardware path no longer depends on libyuv or a CPU RGBA-to-NV12
conversion. H264 defaults to the same exact portrait upload geometry used by
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
H264 bitrate is `auto`, which keeps roughly the same bits per frame as FPS
changes and resolves to `7M` at 30 FPS. The native default is all-I
(`--usb-h264-gop 1`) because TURZX panel corruption measurements improved as
P-frame references were removed. GOP 3 route replay was much better than the
earlier long-GOP runs, and GOP 2 further improved compact-overlay tests, but a
route replay without the overlay showed frequent small block artifacts at GOP 2.
GOP 1 at 6M removed the visible squares on the same route, with only slightly
softer compression detail, and a follow-up GOP 1 / 7M run also stayed clean, so
GOP 1 is the measured stability default for now.
An explicit `8M` route replay was worse and pushed H264 USB
chunk writes into large latency spikes, so the auto cap is limited to `7M`. The
larger `--usb-h264-slice-max-bytes 8192` A/B also looked worse than the default
4096-byte slice cap, and `2048` caused smaller but more frequent smearing, so
keep the default slice setting for normal tests. The
hardware V4L2 rate-control default remains `--usb-h264-rate-control vbr-cfr`;
`cbr-cfr` made frequent small blocks and `--usb-h264-realtime-priority` landed
between VBR-CFR and CBR-CFR, so keep both off for normal tests. The
ffmpeg/libx264 path remains available as a known-good comparison path. Build
the native library before hardware testing:

```bash
scons system/loggerd/libcluster_h264_encoder_bridge.so
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
unit, NAL sizes, native sender queue depth, and USB send latency. Use it on both
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
debug UI before navi data has arrived. When output is gated off,
`cluster_autorun` sends TURZX brightness `0` so a stale HUD frame does not
remain visible.
The autorun watcher normalizes locale before this dim-only USB path too, so
vendor USB initialization does not fail before the renderer is launched.
Manager autostart sets `CLUSTER_REALTIME=1` by default unless the environment
already overrides it. With realtime enabled, `cluster_autorun.py` uses
`ClusterHudCoreMode=0` by default, which maps to cores `1,2,3,4`; mode `1` maps
to all initially allowed CPU cores.
`ClusterHudPriority` controls the common openpilot realtime helper priority with
range `1..99`, default `10`.
Changing either param makes the running HUD exit so `cluster_autorun` can
relaunch it with the new affinity/priority, without a whole system restart.
Explicit `CLUSTER_REALTIME`, `CLUSTER_REALTIME_CORES`, or
`CLUSTER_REALTIME_PRIORITY` environment values still win.
When `--usb-brightness` is omitted, USB launches follow `ClusterHudBrightness`:
`0` auto follows live `deviceState.screenBrightnessPercent` after samples are
available, and `1` through `100` are fixed brightness percentages.
Brightness commands use no-ACK command `14` during USB initialization and when
the resolved brightness changes.

The launcher defaults to `--input live`, subscribes to openpilot cereal services,
and renders live `carState`, `modelV2`, `radarState`, `liveTracks`,
`controlsState`, `selfdriveState`, `carControl`, and `deviceState`. Front radar
tracks come from `liveTracks`; the cluster does not directly parse A-CAN CAN-FD
radar track frames for display. Manager/autostart leaves
the live CAN/sendcan subscriptions enabled, but exact LF/RF/LR/RR corner radar
distance now comes only from received Hyundai camera-bus `can` `0x162`/`0x1EA`
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
speed but draws it gray, and inactive cruise draws gray `--- km/h`. The
lane-change icon is not drawn; the LFA icon uses
`selfdrive/assets/icons_mici/carrot_wheel_org.png`, rotates by
`-carState.steeringAngleDeg`, and recolors its white pixels green when LFA is
active.
When `--fps` is omitted, `ClusterHudLiveFps` controls the render limit and is
polled about once per second while running: `0` uncapped diagnostic mode, `1`
10 Hz default, `2` 20 Hz, `3` 30 Hz, `4` 40 Hz, `5` 50 Hz, and `6` 60 Hz.
Direct route/replay CLI runs also apply nonzero values; mode `0` keeps non-live
H264 runs on the `--usb-h264-fps` safety cap. Explicit `--fps` remains a fixed
override. For H264 USB output, changing the effective FPS exits the current HUD
process so autostart can relaunch with a matching encoder FPS when a launcher
is present.
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
launched by `cluster_autorun`. Direct CLI auto uses native hardware H264 as the
first encoder choice. `1` forces JPEG, `2` forces native hardware H264, and `3`
forces ffmpeg/libx264 software H264.
Native hardware H264 always uses the direct GPU NV12 render/submit path. If
backend `auto` falls back to ffmpeg, the run uses the software RGBA pipe.
Changing this setting while the HUD is running makes the current HUD process
exit so `cluster_autorun` can relaunch it with the new encoder choice.
`ClusterHudScreenMode` controls optional debug views: `0` default, `1` shows
the live debug panel with grouped `LIVE DELAY`, `LIVE TORQUE`, `STEERING`, and
`LATERAL PLAN` rows, `2` shows the system information panel with memory and CPU
core usage, `3` shows a large debug graph selected by `ShowPlotMode` with the
driving scene disabled, and `4`
shows the same graph in the right-side panel while keeping the driving scene.
`5` shows the external navigation receiver debug panel while keeping the
driving scene.
Mode `3` also hides the speed, accel, clock, turn-signal, and git HUD so the
large graph uses the available center/right height with only a small margin.
Mode `4` keeps the driving HUD and uses the maximum right-side panel height with
the same margin. Mode `5` draws the received navigation route through the
normal planned-path renderer when route coordinates and current ego GPS are
available. Modes `1`, `2`, `3`, `4`, and `5` suppress the route overlay so the
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
high view, tilted downward, and shifted `5m` forward so route/live scene space
is easier to see. Detected vehicles, radar points, and desired-distance markers
compress signed longitudinal placement by `0.5` in the rendered scene only, so
actual `20m` and `-10m` draw at rendered `10m` and `-5m`. Distance labels keep
the actual signed longitudinal values. The ego vehicle is drawn half a vehicle
length behind the raw `0m` reference so its front bumper aligns to that
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
Changing `ClusterHud` to another supported mode or `0` makes the running HUD
exit; cleanup sends TURZX brightness zero before releasing the USB device.
When autorun passes a HUD mode, USB open is pinned to that mode's TURZX PID
(`1 -> 0x0092`, `2 -> 0x0123`) so a second connected TURZX panel is not opened
by the vendor library's generic device scan.
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

USB frame upload runs in no-ACK mode by default because some TURZX panels accept
image data but never return a frame-upload response. Use `--usb-wait-frame-ack`
only when testing a panel/driver combination known to reply after each frame.
