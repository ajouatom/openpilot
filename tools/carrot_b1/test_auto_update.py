import asyncio
import os
import sys
from types import SimpleNamespace

import pytest

from tools.carrot_b1 import auto_update as auto  # noqa: TID251 -- board release package


def test_bundled_release():
  image = auto.IMAGE.read_bytes()
  version, elf = auto.release_id(image)
  assert version >= 1 and len(elf) == 64
  assert auto.decision(f'elf={elf} carrot-b1-release={version}'.encode(), (version, elf), {}, 'board', 0) == 'current'
  with pytest.raises(ValueError):
    auto.release_id(image[:-1] + bytes([image[-1] ^ 1]))


def test_policy():
  target = (2, 'a'*64)
  def report(number, elf):
    return f'elf={elf} carrot-b1-release={number}'.encode()
  def decide(raw, rec=None):
    return auto.decision(raw,target,rec or {},'unit',5000)
  record = {'mac':'unit','elf':target[1],'time':4900,'status':'attempt'}
  assert decide(b'')=='install'
  assert decide(report(1,'b'*64))=='install'
  assert decide(report(3,'b'*64))=='newer'
  assert decide(report(2,'b'*64))=='version conflict'
  assert decide(b'',record)=='retry later'
  assert decide(b'',{**record,'time':3000})=='install'
  assert decide(b'',{**record,'status':'uploaded','time':3000})=='waiting for build identity'
  assert decide(b'',{**record,'mac':'different'})=='install'


def test_migrate_verify_and_skip(monkeypatch,tmp_path):
  updater = SimpleNamespace(ParkGuard=None,Client=None)
  monkeypatch.setitem(sys.modules,'tools.carrot_b1.comma_update',updater)
  if os.name=='nt':
    monkeypatch.setitem(sys.modules,'fcntl',SimpleNamespace(flock=lambda *_:None, LOCK_EX=2, LOCK_NB=4))
  target = (1, 'c'*64)
  mac = 'registered'
  monitor = SimpleNamespace(session=12, owned=mac, client=None, attrs={auto.META:'m',auto.OTA:'o',auto.TRACE:'t'})
  for name in ('OTA_LOCK','RECEIPT','ENROLLMENT'):
    monkeypatch.setattr(auto,name,tmp_path/name)
  auto.atomic_json(auto.ENROLLMENT,{'address':mac,'key':'00'*32})
  board = SimpleNamespace(trace=b'seq=12 confirmed=0', writes=0, safe=True)
  class Guard:
    sm=SimpleNamespace(update=lambda _:None)
    def check(self):
      if not board.safe:
        raise RuntimeError('moved')
  class Client:
    def __init__(self,*args): pass
    async def read_gatt_char(self,uuid):
      assert uuid==auto.TRACE
      return board.trace
  async def install(*args,parked_check,quiet_bus):
    parked_check()
    assert not quiet_bus
    board.writes+=1
  async def noop(_): pass
  monkeypatch.setattr(auto.asyncio,'sleep',noop)
  monkeypatch.setattr(auto,'install',install)
  monkeypatch.setattr(updater,'ParkGuard',Guard)
  monkeypatch.setattr(updater,'Client',Client)
  def check():
    return asyncio.run(auto.check(monitor,b'fake',target,mac,12))
  board.safe=False
  with pytest.raises(RuntimeError,match='moved'):
    check()
  board.safe=True
  assert check()=='waiting for reboot'
  assert auto.read_json(auto.RECEIPT)['status']=='uploaded'
  assert check()=='waiting for build identity'
  board.trace=f'elf={target[1]} carrot-b1-release=1'.encode()
  assert check()=='current'
  assert auto.read_json(auto.RECEIPT)['status']=='current' and board.writes==1
