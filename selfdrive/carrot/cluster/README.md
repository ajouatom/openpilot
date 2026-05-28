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
python selfdrive/carrot/cluster_run.py --output usb --input route --route route --route-overlay off --usb-codec h264 --usb-h264-bitrate 1M --usb-h264-fps 30 --profile-render
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-test-pattern --duration 20 --fps 10 --usb-h264-debug --usb-h264-slice-max-bytes 4096
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-backend ffmpeg --usb-h264-ffmpeg-encoder libx264 --usb-h264-test-pattern --duration 20 --fps 10 --usb-h264-debug
python selfdrive/carrot/cluster_run.py --output usb --fps 10 --usb-jpeg-quality 55 --route-overlay off
python selfdrive/carrot/cluster_run.py --output usb --profile-render --profile-interval 2
```

`--usb-jpeg-encoder auto` tries optional `turbojpeg` first and falls back to
Pillow. Route replay defaults to `--route-overlay compact`, which shows the
right-side debug panel. Use `--route-overlay off` for performance tests that
should match live rendering cost more closely.

`--usb-codec h264` feeds RGBA frames to the Qualcomm V4L2 encoder wrapper in
`system/loggerd/encoder`, or to ffmpeg/libx264. H264 defaults to the same exact
portrait upload geometry used by the working JPEG/PNG and earlier ffmpeg H264
paths. For a 9.2-inch panel that means a 462x1920 H264 stream, with no
16-pixel render-size padding unless `--usb-h264-align 16` is passed explicitly.
Native/helper hardware encoding pads only the encoder input to a 16-pixel
boundary by default, so 462x1920 display frames are fed to V4L2 as 464x1920 and
cropped back to 462x1920 in SPS metadata.
The default backend is the native Qualcomm hardware path. It patches hardware
SPS Baseline constraint flags to match the libx264 constrained-Baseline stream
that the TURZX panel accepts, and patches hardware SPS frame-crop metadata for
non-macroblock geometry such as 462x1920. It also asks the V4L2 encoder for
multi-slice output capped by `--usb-h264-slice-max-bytes` so the resulting NAL
sizes are closer to the ffmpeg/libx264 stream accepted by TURZX. The default
H264 bitrate is `1M` because the Qualcomm hardware path tends to fill the
requested rate, unlike the libx264 comparison path on static HUD content. The
ffmpeg/libx264 path remains available as a known-good comparison path. Build
the native library and helper before hardware testing:

```bash
scons system/loggerd/libcluster_h264_encoder_bridge.so
scons system/loggerd/cluster_h264_encoder_cli
```

Use `--usb-h264-backend ffmpeg --usb-h264-ffmpeg-encoder libx264` to compare
the known-good software stream. Use `--usb-h264-backend helper` to compare the
hardware helper process path, or `--usb-h264-backend auto` to try native and
fall back to helper.

The default V4L2 device is
`/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1`. Input format
defaults to `nv12`, matching the existing loggerd V4L2 encoder path. The device
also reports RGB4 support, and `--usb-h264-input-format rgb4` remains available
for direct RGB input tests, but NV12 is the safer compatibility path for the
TURZX panel. The default RGB4 byte layout is `bgra`, matching the common
little-endian memory order for V4L2 `RGB4`. The cluster H264 wrapper emits
inline SPS/PPS on the first video packet and on IDR frames, asks for constrained
Baseline/CAVLC plus VUI timing when the V4L2 driver accepts those controls, and
falls back to the existing loggerd-compatible High/CABAC controls if needed.
Access-unit delimiter NALs
are off by default because some simple panel decoders expect SPS/PPS or slices
as the first NAL; use `--usb-h264-insert-aud` only as a compatibility test.
`--usb-h264-hardware-profile high` forces the loggerd-style High/CABAC path,
which can produce smaller hardware access units than Baseline/CAVLC on this
Qualcomm encoder; rebuild the native library/helper after changing C++ encoder
code before testing it.
`--usb-h264-debug` prints a detailed trace for each early hardware packet:
native callback flags/timestamps/keyframe state, raw and patched NAL summaries,
packetization results, TURZX chunk sizes, and a shutdown summary. The helper
backend also prints C++ packet metadata to stderr before Python reads stdout.
`--usb-h264-no-sps-patch` disables the hardware SPS constraint-byte patch.
`--usb-h264-no-sps-crop-patch` disables the hardware SPS frame-crop patch.
`--usb-h264-no-sps-vui-patch` disables the hardware SPS VUI timing patch.
`--usb-h264-encoder-align 1` disables hardware-only input padding for A/B
testing; the default `16` avoids feeding the Qualcomm encoder a 462-byte NV12
stride while its H264 SPS reports a 464-pixel coded width.
`--usb-h264-slice-max-bytes 0` disables the hardware multi-slice request.
`--usb-h264-slice-max-mb` is intentionally disabled because this Qualcomm V4L2
driver path can stall before producing capture buffers.
`--usb-h264-qp N` forces hardware QP/min-QP controls for compatibility tests;
higher values produce smaller hardware IDR/P frames.
`--usb-h264-packetize auto` sends native/helper hardware output as encoder
access units, matching the known-good ffmpeg/libx264 command boundary. Use
`--usb-h264-packetize nal` or `nal-groups` only for A/B tests. By default the
native hardware path leaves the TURZX H264 command `last` flag off to match the
known-good ffmpeg/libx264 path. Use `--usb-h264-mark-frame-end` only as a
compatibility test; `--usb-h264-debug` prints this as `last=1`.

For a quick H264 transport smoke test, run:

```bash
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-test-pattern --duration 20 --fps 10 --usb-h264-debug --usb-h264-slice-max-bytes 4096
```

The panel should show red/green/blue/white quadrants. If you force
`--usb-h264-input-format rgb4` and colors are swapped, retry with
`--usb-h264-rgb4-layout rgba` or `--usb-h264-rgb4-layout axrgb`. If RGB4 itself
looks suspicious, return to the default `--usb-h264-input-format nv12` path.
`--usb-h264-orientation landscape` tests direct 1920x462 output, while
`--usb-h264-align 16` deliberately tests macroblock-aligned output such as
1920x464. When `--fps` is omitted, H264 USB runs use `--usb-h264-fps 30` as the
render cap. H264 chunks are no-ACK by default like JPEG frame uploads; use
`--usb-h264-wait-ack` for strict response diagnostics, or
`--usb-h264-soft-ack` to mimic the vendor video sender's retry/status polling
without failing the run. If the hardware stream is still corrupted, rerun with
`--usb-h264-debug --usb-h264-dump /tmp/cluster_hw_native.h264` and keep the
native packet, packetize, chunk, and final summary lines. Then retry
`--usb-h264-slice-max-bytes 2048` and `1024`; the debug NAL summary should show
several smaller IDR/P NALs instead of one large slice. For 462x1920 streams,
the SPS summary should show `display=462x1920` rather than only the coded
464-pixel macroblock width.
If the first hardware IDR remains much larger than the ffmpeg/libx264 stream,
retry `--usb-h264-qp 38` and `--usb-h264-qp 44` to test whether the panel is
rejecting large access units.
If QP controls are rejected by the driver, keep `--usb-h264-packetize auto`
so each encoder access unit is sent with the same command boundary as the
software comparison path.
Also compare the loggerd-style profile with
`--usb-h264-hardware-profile high --usb-h264-bitrate 1M`; the SPS summary should
change from `profile=0x42` to `profile=0x64` when the rebuilt native library is
being used.
If the hardware SPS summary shows `vui=0`, the default patch adds timing info
matching the selected H264 FPS; use `--usb-h264-no-sps-vui-patch` only as an
A/B check.

The ffmpeg/libx264 path is the known-good H264 comparison mode. To make that
explicit while testing, run:

```bash
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-backend ffmpeg --usb-h264-ffmpeg-encoder libx264 --usb-h264-test-pattern --duration 20 --fps 10 --usb-h264-debug
```

When the panel still shows a corrupted picture, dump the outgoing stream and
compare it separately:

```bash
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-test-pattern --duration 20 --fps 10 --usb-h264-debug --usb-h264-input-format nv12 --usb-h264-dump /tmp/cluster_hw_nv12.h264
ffprobe -show_streams /tmp/cluster_hw_nv12.h264
```

If the dump plays correctly but the panel is corrupted, the remaining issue is
TURZX stream compatibility or USB flow control. If the dump is corrupted too,
the issue is in the V4L2 input conversion or encoder controls.

Manager autostart omits `--fps` by default so live launches follow
`ClusterHudLiveFps` setting changes while running. Set `CLUSTER_AUTORUN_FPS`
only for fixed test overrides; `0` means uncapped.
When `--usb-brightness` is omitted, USB launches follow `ClusterHudBrightness`:
`0` auto follows live `deviceState.screenBrightnessPercent` after samples are
available, and `1` through `100` are fixed brightness percentages.
Brightness commands use no-ACK command `14`; while USB output is active, the
current brightness is resent at least once every 5 seconds.

The launcher defaults to `--input live`, subscribes to openpilot cereal services,
and renders live `carState`, `modelV2`, `radarState`, `liveTracks`,
`controlsState`, `carControl`, `deviceState`, and raw Hyundai CAN-FD radar
points when CAN subscription is enabled.
When `--fps` is omitted for live input, `ClusterHudLiveFps` controls the render
limit and is polled about once per second while running: `0` uncapped,
`1` 10 Hz, `2` 20 Hz, and `3` 30 Hz. Explicit `--fps` remains a fixed
override.
`ClusterHudScreenMode` controls optional debug views: `0` default, `1` shows
the live debug panel with grouped `LIVE DELAY`, `LIVE TORQUE`, `STEERING`, and
`LATERAL PLAN` rows, `2` shows the system information panel, `3` shows a large
debug graph selected by `ShowPlotMode` with the driving scene disabled, and `4`
shows the same graph in the right-side panel while keeping the driving scene.
Mode `3` also hides the speed, accel, clock, turn-signal, and git HUD so the
large graph uses the available center/right height with only a small margin.
Mode `4` keeps the driving HUD and uses the maximum right-side panel height with
the same margin. Modes `1`, `2`, `3`, and `4` suppress the route overlay so the
selected debug view remains visible.
Changing `ClusterHud` to another supported mode or `0` makes the running HUD
exit; cleanup sends TURZX brightness zero before releasing the USB device.

The bundled TURZX code includes only the Python vendor library. The openpilot
device uses the system `libusb-1.0.so` through `pyusb`.

The renderer prefers
`/data/openpilot/selfdrive/assets/fonts/KaiGenGothicKR-Bold.ttf` for HUD text.
It falls back to the bundled/addon KaiGen copy, then JetBrainsMono and
system/platform fonts if KaiGen is not present.

USB frame upload runs in no-ACK mode by default because some TURZX panels accept
image data but never return a frame-upload response. Use `--usb-wait-frame-ack`
only when testing a panel/driver combination known to reply after each frame.
