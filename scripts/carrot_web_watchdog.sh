#!/usr/bin/env bash
set -u

DIR="${1:-/data/openpilot}"
PY_BIN="${2:-$(command -v python3 || command -v python || true)}"
PID_FILE="${CARROT_WEB_PID_FILE:-/tmp/carrot_web_watchdog.pid}"
PORT="${CARROT_WEB_PORT:-7000}"
RESTART_DELAY="${CARROT_WEB_RESTART_DELAY:-2}"

if [ -z "$PY_BIN" ]; then
  echo "[carrot_web] python not found"
  exit 1
fi

echo "$$" > "$PID_FILE"
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
