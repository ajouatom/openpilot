from pathlib import Path

from openpilot.common.basedir import BASEDIR


def test_recovery_and_agnos_precede_params_build() -> None:
  launcher = Path(BASEDIR) / "launch_chffrplus.sh"
  source = launcher.read_text(encoding="utf-8")
  launch = source[source.index("function launch {"):]

  pythonpath = launch.index('export PYTHONPATH=')
  ssh_access = launch.index("/data/params/d/SshEnabled")
  recovery = launch.index("  start_carrot_recovery")
  agnos_update = launch.index("    agnos_init")
  params_build = launch.index('bash "$DIR/scripts/ensure_params_build.sh"')
  web = launch.index("  start_carrot_web")

  assert pythonpath < ssh_access < recovery < agnos_update < params_build < web
