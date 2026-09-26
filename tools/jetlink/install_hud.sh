#!/usr/bin/env bash
set -euo pipefail
ROOT=$(realpath "${1:?prepared runtime directory required}")
OWNER=${2:?service account required}
UID_NUMBER=$(id -u "$OWNER")
test -f "$ROOT/carrot/tools/jetlink/hud.py"
cat > /etc/udev/rules.d/71-carrot-jetlink-hud.rules <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="1cbe", ATTR{idProduct}=="0092", GROUP="plugdev", MODE="0660"
SUBSYSTEM=="usb", ATTR{idVendor}=="1cbe", ATTR{idProduct}=="0123", GROUP="plugdev", MODE="0660"
EOF
cat > /etc/systemd/system/carrot-jetlink-hud.service <<EOF
[Unit]
Description=Carrot USB cluster renderer
After=display-manager.service carrot-jetlink.service
StartLimitIntervalSec=0

[Service]
User=$OWNER
SupplementaryGroups=plugdev video render
# The vendor USB library opens log.log relative to cwd. Keep release code
# root-owned and give only its dedicated log directory to the service account.
LogsDirectory=carrot-jetlink-hud
LogsDirectoryMode=0700
WorkingDirectory=/var/log/carrot-jetlink-hud
Environment=DISPLAY=:0
Environment=XAUTHORITY=/run/user/$UID_NUMBER/gdm/Xauthority
Environment=PYTHONUNBUFFERED=1
Environment=OPENBLAS_NUM_THREADS=1
Environment=OMP_NUM_THREADS=1
Environment=LANG=C.UTF-8
ExecStart=$ROOT/venv/bin/python $ROOT/carrot/tools/jetlink/hud.py
Nice=19
CPUAffinity=2 3 4 5
Restart=always
RestartSec=3
TimeoutStopSec=10
UMask=0077

[Install]
WantedBy=multi-user.target
EOF
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb
systemctl daemon-reload
systemctl enable --now carrot-jetlink-hud
