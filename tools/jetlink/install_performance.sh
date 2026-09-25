#!/usr/bin/env bash
set -euo pipefail
test -f /etc/nv_tegra_release
/usr/sbin/nvpmodel -q | grep -q MAXN_SUPER
cat > /etc/systemd/system/carrot-jetlink-performance.service <<'EOF'
[Unit]
Description=Jetson inference clocks within the selected MAXN_SUPER mode
After=nvpmodel.service
Before=carrot-jetlink.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable --now carrot-jetlink-performance
