import importlib.util
import json
import runpy
import sys
from pathlib import Path
from types import ModuleType
from unittest.mock import Mock  # noqa: TID251  # stdlib mocks only; tests run with pytest
from urllib.parse import parse_qs, urlsplit

import pytest


@pytest.fixture
def auth(monkeypatch):
  # Isolate credentials and remote APIs, including platform-specific hardware imports.
  api = ModuleType('openpilot.tools.lib.api')
  api.CommaApi = Mock()
  api.UnauthorizedError = type('UnauthorizedError', (Exception,), {})
  config = ModuleType('openpilot.tools.lib.auth_config')
  config.set_token = Mock()
  config.get_token = Mock()
  monkeypatch.setitem(sys.modules, api.__name__, api)
  monkeypatch.setitem(sys.modules, config.__name__, config)
  spec = importlib.util.spec_from_file_location('_auth_under_test', Path(__file__).parents[1] / 'auth.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


@pytest.fixture
def login_server(auth, monkeypatch):
  server = Mock(server_port=43210, query_params={})
  server.__enter__ = Mock(return_value=server)
  server.__exit__ = Mock(return_value=False)
  monkeypatch.setattr(auth, 'ClientRedirectServer', Mock(return_value=server))
  monkeypatch.setattr(auth.subprocess, 'Popen', Mock(return_value=Mock(poll=Mock(return_value=0))))
  return server


@pytest.mark.parametrize('provider,provider_id', [('google', 'g'), ('apple', 'a'), ('github', 'h')])
def test_login_success(auth, login_server, provider, provider_id):
  login_server.query_params = {'code': ['test-code'], 'provider': [provider_id]}
  auth.CommaApi.return_value.post.return_value = {'access_token': 'test-token'}
  assert auth.login(provider) == {'success': True}
  auth.CommaApi.return_value.get.assert_called_once_with('v1/me', timeout=30)
  auth.set_token.assert_called_once_with('test-token')
  url = auth.subprocess.Popen.call_args.args[0][1]
  assert parse_qs(urlsplit(url).query)['state'] == ['service,localhost:43210']
  login_server.__exit__.assert_called_once()


@pytest.mark.parametrize('params', [
  {'error': ['access_denied']},
  {'code': [''], 'provider': ['g']},
  {'code': ['one', 'two'], 'provider': ['g']},
  {'code': ['test-code'], 'provider': ['h']},
  {'code': ['test-code']},
])
def test_login_rejects_invalid_callback(auth, login_server, params):
  login_server.query_params = params
  assert 'error' in auth.login('google')
  auth.CommaApi.assert_not_called()
  auth.set_token.assert_not_called()
  login_server.__exit__.assert_called_once()


@pytest.mark.parametrize('token', [None, '', ' ', 123])
def test_login_rejects_invalid_token(auth, login_server, token):
  login_server.query_params = {'code': ['test-code'], 'provider': ['g']}
  auth.CommaApi.return_value.post.return_value = {'access_token': token}
  assert 'error' in auth.login('google')
  auth.set_token.assert_not_called()


def test_login_rejects_unverified_token(auth, login_server):
  login_server.query_params = {'code': ['test-code'], 'provider': ['g']}
  auth.CommaApi.return_value.post.return_value = {'access_token': 'test-token'}
  auth.CommaApi.return_value.get.side_effect = auth.UnauthorizedError()
  assert 'error' in auth.login('google')
  auth.set_token.assert_not_called()


def test_login_timeout(auth, login_server):
  assert 'timed out' in auth.login('google', timeout=0)['error']
  auth.set_token.assert_not_called()
  login_server.__exit__.assert_called_once()


def test_browser_failure(auth, login_server):
  auth.subprocess.Popen.return_value.poll.return_value = 1
  assert 'browser' in auth.login('google')['error']
  auth.set_token.assert_not_called()


def test_json_cli_failure(auth, monkeypatch, capsys):
  # A GUI caller must get parseable JSON even when browser launch fails.
  monkeypatch.setattr(auth.subprocess, 'Popen', Mock(side_effect=OSError('No browser')))
  monkeypatch.setattr(sys, 'argv', ['auth.py', 'google', '--json'])
  with pytest.raises(SystemExit) as exc:
    runpy.run_path(auth.__file__, run_name='__main__')
  assert exc.value.code == 0
  assert 'error' in json.loads(capsys.readouterr().out)
  auth.set_token.assert_not_called()
