from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def test_build_spinner_uses_only_bootstrap_fonts():
  spinner = (REPO_ROOT / "openpilot/system/ui/spinner.py").read_text(encoding="utf-8")
  assert 'gui_app.init_window("Spinner", font_weights=(FontWeight.NORMAL, FontWeight.PRETENDARD))' in spinner
  assert "FontWeight.DISPLAY" not in spinner[spinner.index("def main():"):]
  assert "FontWeight.UNIFONT" not in spinner[spinner.index("def main():"):]


def test_continue_script_is_durable_before_installer_waits():
  installer = (REPO_ROOT / "openpilot/selfdrive/ui/installer/installer.cc").read_text(encoding="utf-8")
  write_start = installer.index("// write continue.sh")
  wait_start = installer.index("// wait for the installed software's UI")
  write = installer[write_start:wait_start]

  expected_order = (
    "fwrite(",
    "fflush(",
    "fchmod(",
    "fsync(fileno(of))",
    "fclose(",
    "rename(CONTINUE_NEW_PATH, CONTINUE_PATH)",
    'open("/data", O_RDONLY | O_DIRECTORY)',
    "fsync(data_dir_fd)",
  )
  positions = [write.index(token) for token in expected_order]
  assert positions == sorted(positions)
  assert 'run("mv /data/continue.sh.new' not in write
