from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from usbc_host import select_host, supported


def test_platform_guard(tmp_path):
  (tmp_path/'proc/device-tree').mkdir(parents=True)
  (tmp_path/'etc').mkdir()
  (tmp_path/'proc/device-tree/compatible').write_bytes(b'nvidia,p3768-0000+p3767-0005-super\0nvidia,tegra234\0')
  (tmp_path/'etc/nv_tegra_release').write_text('# R36 (release), REVISION: 4.7, GCID: 42132812')
  assert supported(tmp_path)
  (tmp_path/'etc/nv_tegra_release').write_text('# R36 (release), REVISION: 4.4,')
  assert not supported(tmp_path)
  assert not supported(tmp_path/'missing')


@pytest.mark.parametrize('mode', ['SNK(4)', 'DRP+ACC(32)'])
def test_negotiates_through_controller_in_order(tmp_path, monkeypatch, mode):
  (tmp_path/'fmode').write_text(mode)
  (tmp_path/'fsw_trysnk').write_text('1')
  writes = []
  original = Path.write_text
  def write(path, value, *args, **kwargs):
    writes.append((path.name, value.strip()))
    return original(path, value, *args, **kwargs)
  monkeypatch.setattr(Path, 'write_text', write)
  assert select_host(tmp_path)
  assert writes == [('fsw_trysnk', '0'), ('fmode', '1')]


def test_already_selected_does_not_detach(tmp_path, monkeypatch):
  (tmp_path/'fmode').write_text('SRC(1)')
  (tmp_path/'fsw_trysnk').write_text('0')
  monkeypatch.setattr(Path, 'write_text', lambda *a, **k: pytest.fail('unnecessary USB detach'))
  assert select_host(tmp_path) is False


def test_rejected_mode_restores_old_policy(tmp_path, monkeypatch):
  (tmp_path/'fmode').write_text('SNK(4)')
  (tmp_path/'fsw_trysnk').write_text('1')
  original = Path.write_text
  def write(path, value, *args, **kwargs):
    if path.name == 'fmode' and value.strip() == '1':
      raise OSError('synthetic kernel rejection')
    return original(path, value, *args, **kwargs)
  monkeypatch.setattr(Path, 'write_text', write)
  with pytest.raises(OSError):
    select_host(tmp_path)
  assert (tmp_path/'fmode').read_text().strip() == '4'
  assert (tmp_path/'fsw_trysnk').read_text().strip() == '1'


def test_unknown_driver_is_not_modified(tmp_path, monkeypatch):
  (tmp_path/'fmode').write_text('UNKNOWN(63)')
  (tmp_path/'fsw_trysnk').write_text('1')
  monkeypatch.setattr(Path, 'write_text', lambda *a, **k: pytest.fail('unexpected write'))
  with pytest.raises(ValueError):
    select_host(tmp_path)


@pytest.mark.skipif(sys.platform == 'win32', reason='Linux systemd symlink semantics')
def test_installer_orders_policy_before_inference(tmp_path):
  from install_usbc import configure
  configure(tmp_path)
  configure(tmp_path)
  systemd = tmp_path/'etc/systemd/system'
  name = 'carrot-jetlink-usbc-host.service'
  assert 'Before=carrot-jetlink.service' in (systemd/name).read_text()
  assert (systemd/'multi-user.target.wants'/name).resolve() == (systemd/name).resolve()
  assert (tmp_path/'usr/local/lib/carrot-jetlink/usbc_host.py').read_bytes() == Path(__file__).with_name('usbc_host.py').read_bytes()
