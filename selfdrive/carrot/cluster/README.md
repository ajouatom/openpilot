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
python selfdrive/carrot/cluster_run.py --output usb --fps 10 --usb-jpeg-quality 55 --route-overlay off
python selfdrive/carrot/cluster_run.py --output usb --profile-render --profile-interval 2
```

`--usb-jpeg-encoder auto` tries optional `turbojpeg` first and falls back to
Pillow. Route replay defaults to `--route-overlay compact`, which shows the
right-side qcamera/debug panel. Use `--route-overlay off` for performance tests
that should match live rendering cost more closely.

Manager autostart uses `--fps 20` by default through `CLUSTER_AUTORUN_FPS=20`.
Set `CLUSTER_AUTORUN_FPS=0` only for uncapped performance tests.
Set it to an empty value if autostart should follow `ClusterHudLiveFps`
setting changes while running.

The launcher defaults to `--input live`, subscribes to openpilot cereal services,
and renders live `carState`, `modelV2`, `radarState`, `liveTracks`, and raw
Hyundai CAN-FD radar points when CAN subscription is enabled.
When `--fps` is omitted for live input, `ClusterHudLiveFps` controls the render
limit and is polled about once per second while running: `0` uncapped,
`1` 10 Hz, `2` 20 Hz, and `3` 30 Hz. Explicit `--fps` remains a fixed
override.

The bundled TURZX code includes only the Python vendor library. The openpilot
device uses the system `libusb-1.0.so` through `pyusb`.

The renderer prefers
`/data/openpilot/selfdrive/assets/fonts/KaiGenGothicKR-Bold.ttf` for HUD text.
It falls back to the bundled/addon KaiGen copy, then JetBrainsMono and
system/platform fonts if KaiGen is not present.

USB frame upload runs in no-ACK mode by default because some TURZX panels accept
image data but never return a frame-upload response. Use `--usb-wait-frame-ack`
only when testing a panel/driver combination known to reply after each frame.
