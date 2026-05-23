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
python selfdrive/carrot/cluster_run.py --output usb --fps 10 --usb-jpeg-quality 55
python selfdrive/carrot/cluster_run.py --output usb --profile-render --profile-interval 2
```

The launcher defaults to `--input live`, subscribes to openpilot cereal services,
and renders live `carState`, `modelV2`, `radarState`, and raw Hyundai CAN-FD radar
points when CAN subscription is enabled.

The bundled TURZX code includes only the Python vendor library. The openpilot
device uses the system `libusb-1.0.so` through `pyusb`.

USB frame upload runs in no-ACK mode by default because some TURZX panels accept
image data but never return a frame-upload response. Use `--usb-wait-frame-ack`
only when testing a panel/driver combination known to reply after each frame.
