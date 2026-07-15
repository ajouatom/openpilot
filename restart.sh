#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

git pull

# A Params key is compiled into params_pyx.so. The root prebuilt marker makes
# launch_chffrplus skip normal builds, so check this small target explicitly.
# SCons exits immediately when it is already current.
scons -u -j4 openpilot/common/params_pyx.so

# carrot_server runs outside the comma tmux session under a watchdog. Restart
# only its child so it reloads the newly built Params registry.
pkill -f "openpilot.selfdrive.carrot.carrot_server" 2>/dev/null || true

tmux kill-session -t comma 2>/dev/null || true
rm -f /tmp/safe_staging_overlay.lock
sleep 1
tmux new -s comma -d "bash -lc '$DIR/launch_openpilot.sh'"
