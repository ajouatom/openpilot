import pytest

from openpilot.selfdrive.modeld.jetlink.compat import ProtocolChoice


def test_default_tries_latest_then_legacy_on_new_session():
  choice = ProtocolChoice('auto')
  assert choice.version == 3
  choice.handshake_failed()
  assert choice.version == 2
  choice.handshake_failed()
  assert choice.version == 3


@pytest.mark.parametrize('version', ['2', '3'])
def test_explicit_protocol_never_downgrades(version):
  choice = ProtocolChoice(version)
  choice.handshake_failed()
  assert choice.version == int(version)


@pytest.mark.parametrize('setting', ['', '1', '4', 'latest'])
def test_invalid_protocol_is_explicit_error(setting):
  with pytest.raises(ValueError, match='JETLINK_PROTOCOL'):
    ProtocolChoice(setting)
