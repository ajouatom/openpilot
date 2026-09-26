#!/usr/bin/env bash
# Dedicated NVIDIA X server for the USB renderer; SSH/Wi-Fi remain independent.
set -euo pipefail
ROOT=$(realpath "${1:?prepared runtime directory required}")
OWNER=${2:?service account required}
test -x /usr/lib/xorg/Xorg
python3 "$ROOT/carrot/tools/jetlink/configure_headless.py"
if [[ ! -f "$ROOT/default-target-before-jetlink" ]]; then
  systemctl get-default > "$ROOT/default-target-before-jetlink"
fi
cat > /etc/systemd/system/carrot-jetlink-xorg.service <<EOF
[Unit]
Description=Headless NVIDIA graphics for Carrot USB display
After=local-fs.target nvpmodel.service carrot-jetlink-performance.service
Requires=carrot-jetlink-performance.service
Before=carrot-jetlink-hud.service
StartLimitIntervalSec=0

[Service]
RuntimeDirectory=carrot-jetlink-xorg
RuntimeDirectoryMode=0755
ExecStartPre=/usr/bin/python3 $ROOT/carrot/tools/jetlink/xorg_auth.py $OWNER
ExecStart=/usr/lib/xorg/Xorg :1 -config /etc/X11/xorg.conf -auth /run/carrot-jetlink-xorg/Xauthority -nolisten tcp -noreset -novtswitch vt3
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
mkdir -p /etc/systemd/system/carrot-jetlink-hud.service.d
cat > /etc/systemd/system/carrot-jetlink-hud.service.d/headless.conf <<'EOF'
[Unit]
Requires=carrot-jetlink-xorg.service
After=carrot-jetlink-xorg.service

[Service]
Environment=DISPLAY=:1
Environment=XAUTHORITY=/run/carrot-jetlink-xorg/Xauthority
EOF
systemctl daemon-reload
systemctl enable carrot-jetlink-xorg.service
systemctl set-default multi-user.target
systemctl stop carrot-jetlink-hud.service
systemctl stop display-manager.service
systemctl start carrot-jetlink-xorg.service carrot-jetlink-hud.service
