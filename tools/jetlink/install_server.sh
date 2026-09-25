#!/usr/bin/env bash
# Install a prepared Jetson runtime; no engine compilation at boot.
set -euo pipefail
ROOT=$(realpath "${1:?prepared runtime directory required}")
OWNER=${2:?service account required}
id "$OWNER" >/dev/null
test -x "$ROOT/venv/bin/python"
test -f "$ROOT/source/jetlink/server/main.py"
test -f "$ROOT/carrot/tools/jetlink/server.py"
test -f "$ROOT/cache/last-loaded.json"
getent group plugdev >/dev/null || groupadd --system plugdev
usermod -a -G plugdev "$OWNER"
cat > /etc/udev/rules.d/70-carrot-jetlink.rules <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0001", GROUP="plugdev", MODE="0660"
EOF
cat > /etc/systemd/system/carrot-jetlink.service <<EOF
[Unit]
Description=Carrot Jetlink inference server
After=local-fs.target carrot-jetlink-performance.service
RequiresMountsFor=$ROOT
StartLimitIntervalSec=0

[Service]
Type=simple
User=$OWNER
SupplementaryGroups=plugdev
WorkingDirectory=$ROOT/source
Environment=PYTHONUNBUFFERED=1
Environment=OPENBLAS_NUM_THREADS=1
Environment=OMP_NUM_THREADS=1
ExecStart=$ROOT/venv/bin/python $ROOT/carrot/tools/jetlink/server.py --backend trt --transport usb --cache $ROOT/cache --control-socket $ROOT/control.sock
Restart=always
RestartSec=3
TimeoutStopSec=15
KillSignal=SIGINT
UMask=0077

[Install]
WantedBy=multi-user.target
EOF
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb
systemctl daemon-reload
systemctl enable --now carrot-jetlink.service
systemctl --no-pager status carrot-jetlink.service
