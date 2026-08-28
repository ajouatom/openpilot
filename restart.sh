#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

git pull

wait_for_carrot_web_process_exit() {
  local pattern="$1"
  local label="$2"
  local attempt

  for ((attempt = 0; attempt < 50; attempt++)); do
    if ! pgrep -f "$pattern" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done

  echo "${label} did not stop after 5 seconds; forcing it to exit."
  pkill -KILL -f "$pattern" 2>/dev/null || true
  for ((attempt = 0; attempt < 20; attempt++)); do
    if ! pgrep -f "$pattern" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done

  echo "Failed to stop ${label}; refusing to start a competing Carrot Web watchdog."
  return 1
}

# carrot_server runs outside the comma tmux session under a watchdog. Stop the
# watchdog too, and wait for both processes to disappear before relaunching.
# Otherwise launch_chffrplus can observe the terminating watchdog, skip startup,
# and leave Carrot Web permanently down after that old watchdog finally exits.
pkill -f "[c]arrot_web_watchdog[.]sh" 2>/dev/null || true
pkill -f "[o]penpilot.selfdrive.carrot.carrot_server" 2>/dev/null || true
wait_for_carrot_web_process_exit "[c]arrot_web_watchdog[.]sh" "Carrot Web watchdog"
wait_for_carrot_web_process_exit "[o]penpilot.selfdrive.carrot.carrot_server" "Carrot Web server"
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
