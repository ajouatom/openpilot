from pathlib import Path

from openpilot.common.basedir import BASEDIR


def test_recovery_and_agnos_precede_params_build() -> None:
  launcher = Path(BASEDIR) / "launch_chffrplus.sh"
  source = launcher.read_text(encoding="utf-8")
  launch = source[source.index("function launch {"):]

  pythonpath = launch.index('export PYTHONPATH=')
  ssh_access = launch.index("/data/params/d/SshEnabled")
  recovery = launch.index("  start_carrot_recovery")
  agnos_update = launch.index("    if ! agnos_init; then")
  dependencies = launch.index("  if ! bootstrap_runtime_dependencies; then")
  params_build = launch.index('bash "$DIR/scripts/ensure_params_build.sh"')
  web = launch.index("  start_carrot_web")

  assert pythonpath < ssh_access < recovery < agnos_update < dependencies < params_build < web


def test_restart_drops_stale_agnos_version_before_new_tmux() -> None:
  restart = Path(BASEDIR) / "restart.sh"
  source = restart.read_text(encoding="utf-8")

  assert source.index("unset AGNOS_VERSION") < source.index("tmux new -s comma")


def test_restart_defers_params_build_to_dependency_aware_launcher() -> None:
  restart = Path(BASEDIR) / "restart.sh"
  source = restart.read_text(encoding="utf-8")

  assert "ensure_params_build.sh" not in source
  assert "launch_openpilot.sh" in source


def test_agnos_ui_failure_uses_python_fallback_and_blocks_old_os_boot() -> None:
  launcher = Path(BASEDIR) / "launch_chffrplus.sh"
  source = launcher.read_text(encoding="utf-8")

  assert 'python3 "$AGNOS_PY" --swap "$MANIFEST"' in source
  assert "$DIR/openpilot/system/hardware/tici/updater" not in source
  assert "if ! agnos_init; then" in source
  assert source.index("if ! agnos_init; then") < source.index('bash "$DIR/scripts/ensure_params_build.sh"')
