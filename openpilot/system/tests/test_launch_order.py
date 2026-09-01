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
  build = launch.index("    if ! ./build.py; then")
  manager = launch.index("  ./manager.py")

  assert pythonpath < ssh_access < recovery < agnos_update < dependencies < params_build < web < build < manager


def test_usbpd_kernel_is_supplied_only_by_the_agnos_manifest() -> None:
  launcher = Path(BASEDIR) / "launch_chffrplus.sh"
  source = launcher.read_text(encoding="utf-8")
  agnos = (Path(BASEDIR) / "openpilot/system/hardware/tici/agnos.py").read_text(encoding="utf-8")

  assert "install_usbpd_kernel_at_boot" not in source
  assert "start_usbpd_kernel_confirmation" not in source
  assert "AGNOS_POST_FLASH_HOOK" not in source
  assert "AGNOS_POST_FLASH_HOOK" not in agnos


def test_restart_drops_stale_agnos_version_before_new_tmux() -> None:
  restart = Path(BASEDIR) / "restart.sh"
  source = restart.read_text(encoding="utf-8")

  assert source.index("unset AGNOS_VERSION") < source.index("tmux new -s comma")


def test_restart_defers_params_build_to_dependency_aware_launcher() -> None:
  restart = Path(BASEDIR) / "restart.sh"
  source = restart.read_text(encoding="utf-8")

  assert "ensure_params_build.sh" not in source
  assert "launch_openpilot.sh" in source


def test_agnos_ui_failure_never_auto_installs_and_blocks_old_os_boot() -> None:
  launcher = Path(BASEDIR) / "launch_chffrplus.sh"
  source = launcher.read_text(encoding="utf-8")
  agnos_init = source[source.index("function agnos_init {"):source.index("function start_carrot_recovery {")]

  assert 'python3 "$AGNOS_PY" --swap "$MANIFEST"' not in agnos_init
  assert "$DIR/openpilot/system/hardware/tici/updater" not in source
  assert "AGNOS updater UI failed; automatic installation is disabled." in agnos_init
  assert "if ! agnos_init; then" in source
  assert source.index("if ! agnos_init; then") < source.index('bash "$DIR/scripts/ensure_params_build.sh"')
