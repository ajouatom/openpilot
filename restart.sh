git pull
tmux kill-session -t comma; rm -f /tmp/safe_staging_overlay.lock; sleep 1;tmux new -s comma -d "/data/openpilot/launch_openpilot.sh"
