#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

git pull

# carrot_server runs outside the comma tmux session under a watchdog. Stop the
# watchdog too: otherwise a missing/stale pid file can let launch_chffrplus
# create a second watchdog while the first one keeps restarting its child.
pkill -f "[c]arrot_web_watchdog[.]sh" 2>/dev/null || true
pkill -f "openpilot.selfdrive.carrot.carrot_server" 2>/dev/null || true
rm -f /tmp/carrot_web_watchdog.pid

# restart.sh is normally launched from the running comma tmux session, which
# can still carry the previous branch's AGNOS_VERSION. Let the newly pulled
# launch_env.sh select its own version instead of inheriting that stale value.
unset AGNOS_VERSION
tmux set-environment -gu AGNOS_VERSION 2>/dev/null || true
tmux kill-session -t comma 2>/dev/null || true
rm -f /tmp/safe_staging_overlay.lock
sleep 1
tmux new -s comma -d "bash -lc '$DIR/launch_openpilot.sh'"
