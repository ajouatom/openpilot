#!/usr/bin/env bash
set -u

DIR="${1:-/data/openpilot}"
PY_BIN="${2:-$(command -v python3 || command -v python || true)}"
PID_FILE="${CARROT_WEB_PID_FILE:-/tmp/carrot_web_watchdog.pid}"
LOCK_FILE="${CARROT_WEB_LOCK_FILE:-/tmp/carrot_web_watchdog.lock}"
PORT="${CARROT_WEB_PORT:-7000}"
RESTART_DELAY="${CARROT_WEB_RESTART_DELAY:-2}"

if [ -z "$PY_BIN" ]; then
  echo "[carrot_web] python not found"
  exit 1
fi

if command -v flock >/dev/null 2>&1; then
  exec 9>"$LOCK_FILE"
  if ! flock -n 9; then
    echo "[carrot_web] another watchdog already holds ${LOCK_FILE}; exiting"
    exit 0
  fi
fi

existing_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
if [ -n "$existing_pid" ] && [ "$existing_pid" != "$$" ] && kill -0 "$existing_pid" >/dev/null 2>&1; then
  echo "[carrot_web] watchdog pid ${existing_pid} is already running; exiting"
  exit 0
fi

cleanup() {
  current_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [ "$current_pid" = "$$" ]; then
    rm -f "$PID_FILE"
  fi
}

printf '%s\n' "$$" > "$PID_FILE"
trap cleanup EXIT
trap 'exit 0' INT TERM
export CARROT_WEB_EXTERNAL=1

while true; do
  # The updater can replace /data/openpilot while this watchdog survives. In
  # that case its cwd points at the deleted checkout, and a logical `cd` can
  # keep using that stale inode when $PWD still has the same path. Re-enter the
  # physical checkout before every launch so Python always starts in the
  # current tree.
  if ! cd -P -- "$DIR"; then
    echo "[carrot_web] checkout unavailable at ${DIR}; retrying in ${RESTART_DELAY}s"
    sleep "$RESTART_DELAY"
    continue
  fi

  echo "[carrot_web] starting external carrot_server on port ${PORT}"
  "$PY_BIN" -m openpilot.selfdrive.carrot.carrot_server --host 0.0.0.0 --port "$PORT"
  rc=$?
  echo "[carrot_web] carrot_server exited rc=${rc}; restarting in ${RESTART_DELAY}s"
  sleep "$RESTART_DELAY"
done
