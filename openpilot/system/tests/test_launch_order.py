import ast
import os
from pathlib import Path
import shutil
import subprocess

import pytest

from openpilot.common.basedir import BASEDIR


def test_git_maintenance_policy_reaches_nested_services(tmp_path: Path) -> None:
  bash = shutil.which("bash")
  if bash is None:
    pytest.skip("bash is unavailable")
  source = (Path(BASEDIR) / "launch_chffrplus.sh").read_text(encoding="utf-8")
  policy = source[source.index("function disable_automatic_git_maintenance {"):source.index("function cleanup_stale_git_lfs_hooks {")]
  assert source.index("disable_automatic_git_maintenance\n") < source.index("function launch {")
  env = {**os.environ, "GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_NOSYSTEM": "1", "GIT_CONFIG_COUNT": "2",
         "GIT_CONFIG_KEY_0": "test.preserved", "GIT_CONFIG_VALUE_0": "keep",
         "GIT_CONFIG_KEY_1": "gc.auto", "GIT_CONFIG_VALUE_1": "1"}
  # A second shell models a web/recovery service starting its own Git child.
  probe = "bash -c 'for key in gc.auto gc.autoDetach maintenance.auto test.preserved; do git config --get \"$key\"; done'"
  result = subprocess.run([bash, "-c", policy + probe], cwd=tmp_path, env=env, capture_output=True, text=True, timeout=5)
  assert result.returncode == 0, result.stdout + result.stderr
  assert result.stdout.splitlines() == ["0", "false", "false", "keep"]


@pytest.mark.parametrize("external, expected", [(None, "1"), ("", "1"), ("0", "0"), ("1", "1")])
def test_web_mode_reaches_manager_without_inheriting_boot_lock(tmp_path: Path, external: str | None, expected: str) -> None:
  bash = shutil.which("bash")
  if bash is None:
    pytest.skip("bash is unavailable")

  source = (Path(BASEDIR) / "launch_chffrplus.sh").read_text(encoding="utf-8")
  launch = source[source.index("function launch {"):]
  web_startup = launch[launch.index("  # Build Params"):launch.index("  FORCE_REBUILD=0")]
  env = os.environ.copy()
  env.pop("CARROT_WEB_EXTERNAL", None)
  if external is not None:
    env["CARROT_WEB_EXTERNAL"] = external

  # Run the real startup call site, with service/build stubs. The manager probe
  # is a new process, so a shell-local assignment cannot accidentally pass.
  harness = r'''
set -eu
DIR="$PWD"
mkdir -p scripts
printf 'exit 0\n' > scripts/ensure_params_build.sh
exec 9>boot.lock
export CARROT_BOOT_LOCK_FD=9
start_carrot_web() {
  command bash -c 'printf "web=%s\n" "${CARROT_WEB_EXTERNAL-unset}"'
  if { true >&9; } 2>/dev/null; then
    echo 'web inherited boot lock' >&2
    exit 1
  fi
  test "${CARROT_BOOT_LOCK_FD-unset}" = unset
}
''' + web_startup + r'''
command bash -c 'printf "manager=%s\n" "${CARROT_WEB_EXTERNAL-unset}"'
test "$CARROT_BOOT_LOCK_FD" = 9
true >&9
'''
  result = subprocess.run([bash, "-c", harness], cwd=tmp_path, env=env, capture_output=True, text=True, timeout=5, check=False)
  assert result.returncode == 0, result.stdout + result.stderr
  assert result.stdout.splitlines() == [f"web={expected}", f"manager={expected}"]


def test_recovery_and_agnos_precede_params_build() -> None:
  launcher = Path(BASEDIR) / "launch_chffrplus.sh"
  source = launcher.read_text(encoding="utf-8")
  launch = source[source.index("function launch {"):]

  pythonpath = launch.index('export PYTHONPATH=')
  ssh_access = launch.index("/data/params/d/SshEnabled")
  recovery = launch.index("  start_carrot_recovery", ssh_access)
  agnos_update = launch.index("    if ! agnos_init; then")
  dependencies = launch.index("  if ! bootstrap_runtime_dependencies; then")
  params_build = launch.index('bash "$DIR/scripts/ensure_params_build.sh"')
  web = launch.index("  start_carrot_web")
  build = launch.index("    if ! ./build.py; then")
  manager = launch.index("  start_manager")

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


def test_agnos_ui_failure_never_auto_installs_and_shows_recovery_status() -> None:
  launcher = Path(BASEDIR) / "launch_chffrplus.sh"
  source = launcher.read_text(encoding="utf-8")
  agnos_init = source[source.index("function agnos_init {"):source.index("function start_carrot_recovery {")]

  assert 'python3 "$AGNOS_PY" --swap "$MANIFEST"' not in agnos_init
  assert "$DIR/openpilot/system/hardware/tici/updater" not in source
  assert "for attempt in 1 2 3" in agnos_init
  assert "show_agnos_update_failure" in source
  assert "/data/agnos-updater-ui.log" in source
  assert "--timeout 15 --retries 2 jeepney" in agnos_init
  assert 'rm -f "$AGNOS_UPDATE_CONFIRMATION_FILE"' in agnos_init
  assert "if ! agnos_init; then" in source
  assert source.index("if ! agnos_init; then") < source.index('bash "$DIR/scripts/ensure_params_build.sh"')


def test_agnos_update_ui_dispatch_uses_only_the_selected_device_ui() -> None:
  updater = Path(BASEDIR) / "openpilot/system/ui/updater.py"
  tree = ast.parse(updater.read_text(encoding="utf-8"))
  module_imports = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
  imported_modules = " ".join(ast.unparse(node) for node in module_imports)

  assert "tici_updater" not in imported_modules
  assert "mici_updater" not in imported_modules
