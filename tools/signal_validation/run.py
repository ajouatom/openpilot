"""Offline signal checks: real schemas/CAN/safety, isolated settings and no IPC."""
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'opendbc_repo'), str(ROOT / 'tools/jetlink')]


class IsolatedParams:
  """Only the documented defaults used by vehicle fixtures; never device IO."""
  def get_bool(self, key):
    assert key in {'TeslaAutoTurnSignals', 'NNFF', 'NNFFLite', 'DisableMinSteerSpeed'}, key
    return False

  def put_int(self, key, value):
    assert (key, value) == ('LongitudinalPersonalityMax', 3)


def blocked(*args, **kwargs):
  raise AssertionError('Live IPC is forbidden in offline signal tests')


def main():
  params = types.ModuleType('openpilot.common.params')
  params.Params = IsolatedParams
  sys.modules[params.__name__] = params
  messaging = types.ModuleType('openpilot.cereal.messaging')
  for name in ('sub_sock', 'pub_sock', 'drain_sock', 'drain_sock_raw', 'SubMaster', 'PubMaster'):
    setattr(messaging, name, blocked)
  sys.modules[messaging.__name__] = messaging
  import pytest
  return pytest.main(['-c', '/dev/null' if sys.platform == 'linux' else 'NUL', '--rootdir=.',  # noqa: TID251 - fresh process, one invocation
                      '--noconftest', '-q', *sys.argv[1:]])


if __name__ == '__main__':
  raise SystemExit(main())
