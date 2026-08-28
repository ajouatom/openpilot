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
  usbpd_boot = launch.index("  install_usbpd_kernel_at_boot")
  usbpd_confirm = launch.index("  start_usbpd_kernel_confirmation")
  build = launch.index("    if ! ./build.py; then")
  manager = launch.index("  ./manager.py")

  assert pythonpath < ssh_access < recovery < agnos_update < dependencies < params_build < usbpd_boot < web < build < usbpd_confirm < manager

  hook_export = source.index('export AGNOS_POST_FLASH_HOOK=')
  assert hook_export < source.index("function agnos_init")


def test_usbpd_kernel_update_is_checked_at_boot_without_offroad_delay() -> None:
  launcher = Path(BASEDIR) / "launch_chffrplus.sh"
  source = launcher.read_text(encoding="utf-8")
  updater = source[source.index("function install_usbpd_kernel_at_boot {"):source.index("function start_usbpd_kernel_confirmation {")]

  assert '"$updater" ensure-boot' in updater
  assert 'sleep 5' not in updater
  assert 'IsOnroad' not in updater
  assert '"$result" = "10"' in updater


def test_agnos_applies_usbpd_boot_hook_before_slot_swap() -> None:
  agnos = (Path(BASEDIR) / "openpilot/system/hardware/tici/agnos.py").read_text(encoding="utf-8")
  main = agnos[agnos.index('if __name__ == "__main__":'):]

  verify_branch = main[main.index("if args.verify:"):main.index("elif args.swap:")]
  swap_branch = main[main.index("elif args.swap:"):]
  assert verify_branch.index("run_post_flash_hook") < verify_branch.index("swap(")
  assert swap_branch.index("run_post_flash_hook") < swap_branch.index("swap(")


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
