import hashlib
import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('canable_installer', Path(__file__).with_name('install_canable.py'))
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)

LISTING = '\n'.join('Found DFU: [0483:df11] ver=0200, devnum=9, cfg=1, intf=0, path="1-2.2", alt=%d, name="%s", serial="123456789ABC"' % (a, name)
                    for a, name in enumerate(('@Internal Flash /0x08000000/64*02Kg', '@Option Bytes /0x1FFF7800/01*40Be', '@OTP Memory /0x1FFF7000/01*01Ke')))


def test_exact_g431_memory_map_not_generic_stm32_dfu_id():
  assert installer.parse_devices(LISTING) == [{'serial':'123456789ABC','size':131072}]
  for changed in (LISTING.replace('1FFF7800','1FFFF800'), LISTING.replace('64*02Kg','128*01Kg'), LISTING.replace('0483:df11','0483:df12')):
    assert installer.parse_devices(changed) == []


def test_multiple_devices_require_explicit_selection(monkeypatch):
  monkeypatch.setattr(installer, 'run', lambda *a, **k:LISTING+'\n'+LISTING.replace('123456789ABC','ABCDEF123456'))
  with pytest.raises(RuntimeError):
    installer.detect('dfu-util', lambda _:None)
  assert installer.detect('dfu-util', lambda _:None, 'ABCDEF123456')['serial'] == 'ABCDEF123456'


def test_shipped_binary_has_pinned_hash():
  data = installer.image_bytes()
  assert len(data) == 26684 and hashlib.sha256(data).hexdigest() == installer.IMAGE_SHA256


@pytest.mark.parametrize('corrupt_backup,corrupt_readback', [(True,False),(False,True),(False,False)])
def test_backup_precedes_write_and_readback_is_required(tmp_path, monkeypatch, corrupt_backup, corrupt_readback):
  payload = b'firmware'
  monkeypatch.setattr(installer, 'image_bytes', lambda:payload)
  monkeypatch.setattr(installer, 'utility', lambda *_:'dfu-util')
  monkeypatch.setattr(installer, 'detect', lambda *a:{'serial':'123456789ABC','size':16})
  monkeypatch.setattr(installer.Path, 'home', lambda:tmp_path)
  if installer.sys.platform == 'linux':
    monkeypatch.setattr(installer.os, 'geteuid', lambda:0)
  original_is_dir = Path.is_dir
  monkeypatch.setattr(installer.Path, 'is_dir', lambda p:False if p == Path('/opt/carrot-jetlink') else original_is_dir(p))
  writes = []
  def run(arguments, *args, **kwargs):
    if '-U' in arguments:
      path = Path(arguments[-1])
      data = (b'o'*15 if corrupt_backup else b'o'*16) if path.name=='original.bin' else (b'bad' if corrupt_readback else payload)
      path.write_bytes(data)
    elif '-D' in arguments:
      writes.append(arguments)
    return ''
  monkeypatch.setattr(installer, 'run', run)
  if corrupt_backup or corrupt_readback:
    with pytest.raises(RuntimeError):installer.install(lambda _:None, board='mks-canable-v2')
  else:
    assert installer.install(lambda _:None, board='mks-canable-v2')['verified'] is True
  assert len(writes) == (0 if corrupt_backup else 1)
