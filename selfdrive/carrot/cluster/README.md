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
python selfdrive/carrot/cluster_run.py --output usb --input route --route route --route-overlay off --usb-codec h264 --usb-h264-bitrate 6M --usb-h264-fps 30 --profile-render
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-test-pattern --duration 20 --fps 10 --usb-h264-debug
python selfdrive/carrot/cluster_run.py --output usb --fps 10 --usb-jpeg-quality 55 --route-overlay off
python selfdrive/carrot/cluster_run.py --output usb --profile-render --profile-interval 2
```

`--usb-jpeg-encoder auto` tries optional `turbojpeg` first and falls back to
Pillow. Route replay defaults to `--route-overlay compact`, which shows the
right-side debug panel. Use `--route-overlay off` for performance tests that
should match live rendering cost more closely.

`--usb-codec h264` feeds portrait RGBA frames to the hardware helper at
`system/loggerd/cluster_h264_encoder_cli`. The helper uses the Qualcomm V4L2
encoder wrapper in `system/loggerd/encoder`, writes raw H264 to stdout, and the
Python path only chunks those bytes to the TURZX panel. Build the helper before
manual testing:

```bash
scons system/loggerd/cluster_h264_encoder_cli
```

The default V4L2 device is
`/dev/v4l/by-path/platform-aa00000.qcom_vidc-video-index1`. Input format
`auto` prefers RGB4 when the device reports it. If the H264 test pattern shows
swapped colors, retry with `--usb-h264-rgb4-layout rgba` or
`--usb-h264-rgb4-layout bgra`.

For a quick H264 transport smoke test, run:

```bash
python selfdrive/carrot/cluster_run.py --output usb --usb-codec h264 --usb-h264-test-pattern --duration 20 --fps 10 --usb-h264-debug
```

The panel should show red/green/blue/white quadrants. When `--fps` is omitted,
H264 USB runs use `--usb-h264-fps 30` as the render cap. H264 chunks are no-ACK
by default like JPEG frame uploads; use `--usb-h264-wait-ack` only for panels
known to reply, and `--usb-h264-debug` to print early chunk details.

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
