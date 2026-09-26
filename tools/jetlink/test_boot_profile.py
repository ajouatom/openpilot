import json
from types import SimpleNamespace

import pytest

from configure_fast_boot import boot_menu
import trim_jetson as trim


def test_boot_menu_preserves_recovery_and_kernel_arguments():
  source = ('TIMEOUT 30\nDEFAULT primary\nLABEL primary\n'
            '  APPEND root=/dev/mmcblk0p1 rootwait console=tty0\n'
            '# APPEND backup-only\n')
  result = boot_menu(source)
  assert 'TIMEOUT 10\nDEFAULT primary' in result
  assert 'root=/dev/mmcblk0p1 rootwait console=tty0 quiet' in result
  assert '# APPEND backup-only' in result
  assert boot_menu(result) == result


@pytest.mark.parametrize('source', ['APPEND root=x\n', 'TIMEOUT 30\n# APPEND root=x\n'])
def test_unknown_boot_layout_is_rejected(source):
  with pytest.raises(ValueError):
    boot_menu(source)


def setup_plan(monkeypatch, apt_plan, snaps='Name Version\nchromium 1\n'):
  monkeypatch.setattr(trim, 'Path', lambda _: SimpleNamespace(is_file=lambda: True))
  monkeypatch.setattr(trim, 'installed', lambda: {'gdm3': '1', 'snapd': '1'})
  monkeypatch.setattr('sys.argv', ['trim_jetson.py'])
  monkeypatch.setenv('LC_ALL', 'fr_FR.UTF-8')
  def output(command, **kwargs):
    assert trim.os.environ['LC_ALL'] == 'C'
    if command[0] == 'apt-get':
      assert '-s' in command and '--no-auto-remove' in command
      return apt_plan
    assert command == ['snap', 'list']
    return snaps
  monkeypatch.setattr(trim.subprocess, 'check_output', output)
  def mutation(*args, **kwargs):
    raise AssertionError('Preview or rejected plan must not mutate the system')
  monkeypatch.setattr(trim.subprocess, 'run', mutation)


def test_preview_never_removes_packages(monkeypatch, capsys):
  setup_plan(monkeypatch, 'Purg gdm3 [1]\nPurg snapd [1]\n')
  trim.main()
  assert json.loads(capsys.readouterr().out)['snap_removals'] == ['chromium']


def test_unexpected_runtime_removal_is_rejected(monkeypatch):
  setup_plan(monkeypatch, 'Purg gdm3 [1]\nPurg snapd [1]\nPurg nvidia-l4t-core [1]\n')
  with pytest.raises(RuntimeError, match='unexpected removals'):
    trim.main()


def test_unrecognized_apt_output_is_rejected(monkeypatch):
  setup_plan(monkeypatch, 'unexpected localized output')
  with pytest.raises(RuntimeError, match='expected removal plan'):
    trim.main()


def test_other_snap_apps_are_not_silently_removed(monkeypatch):
  setup_plan(monkeypatch, 'Purg gdm3 [1]\nPurg snapd [1]\n', 'Name Version\ncustomer-app 1\n')
  with pytest.raises(RuntimeError, match='other installed Snap'):
    trim.main()
