git pull
tmux kill-session -t comma 2>/dev/null
rm -f /tmp/safe_staging_overlay.lock
tmux new -s comma -d "cd /data/openpilot && ./launch_openpilot.sh"

