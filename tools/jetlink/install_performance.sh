#!/usr/bin/env bash
set -euo pipefail
test -f /etc/nv_tegra_release
SOURCE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 - "$SOURCE" <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from power_setup import configure
configure(Path('/'), Path(sys.argv[1]))
PY
systemctl daemon-reload
# Activate with the normal full reboot so no running GPU workload changes mode.
systemctl enable carrot-jetlink-performance
