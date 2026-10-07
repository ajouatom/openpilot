"""Exercise mobile root-helper policy without touching host firewall/configfs."""
import os
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).with_name('setup_mobile.sh')


def firewall_functions():
  source = SCRIPT.read_text()
  return 'firewall_backend() {' + source.split('firewall_backend() {', 1)[1].split('\nleases_prepare() {', 1)[0]


HARNESS = r'''
set -euo pipefail
STATE=$1
ADDRESS=192.168.60.1
PORT=5599
FIREWALL_COMMENT=jetlink-mobile-owned-5599
dev=usb7
fail() { echo "$*" >&2; exit 1; }
mock_firewall() {
  local backend=$1
  shift
  printf '%s %s\n' "$backend" "$*" >> "$STATE/commands"
  case "$3" in
    -S)
      if [[ "$backend" == iptables && "$NFT_FAIL" == 1 ]]; then
        echo 'Could not fetch rule set generation id: Invalid argument' >&2
        return 4
      fi
      if [[ "$backend" == iptables-legacy && "$LEGACY_FAIL" == 1 ]]; then
        echo 'legacy unavailable' >&2
        return 4
      fi
      ;;
    -I)
      [[ "$INSERT_FAIL" == 0 ]] || return 3
      echo "$backend" > "$STATE/rule"
      ;;
    -C)
      [[ -f "$STATE/rule" && $(cat "$STATE/rule") == "$backend" ]]
      ;;
    -D)
      [[ "$DELETE_FAIL" == 0 ]] || return 3
      [[ $(cat "$STATE/rule") == "$backend" ]] || return 5
      rm "$STATE/rule"
      ;;
  esac
}
iptables() { mock_firewall iptables "$@"; }
iptables-legacy() { mock_firewall iptables-legacy "$@"; }
'''


def run_firewall(tmp_path, action, **settings):
  env = {**os.environ, 'NFT_FAIL': '0', 'LEGACY_FAIL': '0', 'INSERT_FAIL': '0', 'DELETE_FAIL': '0', **settings}
  return subprocess.run(['bash', '-c', HARNESS + firewall_functions() + '\n' + action,
                         'mobile-helper-test', str(tmp_path)], env=env, capture_output=True, text=True)


@pytest.mark.parametrize('nft_fails,expected', [('0', 'iptables'), ('1', 'iptables-legacy')])
def test_backend_probe_persists_and_cleanup_uses_same_backend(tmp_path, nft_fails, expected):
  result = run_firewall(tmp_path, 'firewall_up', NFT_FAIL=nft_fails)
  assert result.returncode == 0, result.stderr
  assert (tmp_path / 'firewall-backend').read_text().strip() == expected
  assert (tmp_path / 'firewall-interface').read_text().strip() == 'usb7'
  # Default backend availability changes after setup; the rule still belongs
  # to its original backend, not whichever happens to work at cleanup.
  cleanup = run_firewall(tmp_path, 'firewall_down', NFT_FAIL=nft_fails)
  assert cleanup.returncode == 0, cleanup.stderr
  commands = (tmp_path / 'commands').read_text().splitlines()
  exact = '-d 192.168.60.1/32 -p tcp --dport 5599 ! -i usb7 -m comment --comment jetlink-mobile-owned-5599 -j DROP'
  assert f'{expected} -w 2 -I INPUT 1 {exact}' in commands
  assert f'{expected} -w 2 -D INPUT {exact}' in commands
  assert not (tmp_path / 'firewall-interface').exists()
  assert not (tmp_path / 'firewall-backend').exists()
  assert not (tmp_path / 'rule').exists()


def test_no_working_backend_fails_before_creating_listener_isolation_state(tmp_path):
  result = run_firewall(tmp_path, 'firewall_up', NFT_FAIL='1', LEGACY_FAIL='1')
  assert result.returncode != 0
  assert 'refusing an unisolated' in result.stderr
  assert not (tmp_path / 'firewall-interface').exists()
  assert not (tmp_path / 'rule').exists()


@pytest.mark.parametrize('operation', ['INSERT_FAIL', 'DELETE_FAIL'])
def test_rule_operation_failure_retains_backend_and_ownership(tmp_path, operation):
  if operation == 'DELETE_FAIL':
    assert run_firewall(tmp_path, 'firewall_up', NFT_FAIL='1').returncode == 0
  result = run_firewall(tmp_path, 'firewall_down' if operation == 'DELETE_FAIL' else 'firewall_up',
                        NFT_FAIL='1', **{operation: '1'})
  assert result.returncode != 0
  assert (tmp_path / 'firewall-backend').read_text().strip() == 'iptables-legacy'
  assert (tmp_path / 'firewall-interface').exists()


def test_cleanup_cannot_switch_backends_when_owned_backend_is_broken(tmp_path):
  assert run_firewall(tmp_path, 'firewall_up').returncode == 0
  result = run_firewall(tmp_path, 'firewall_down', NFT_FAIL='1')
  assert result.returncode != 0 and 'not operational' in result.stderr
  assert (tmp_path / 'firewall-backend').read_text().strip() == 'iptables'
  commands = (tmp_path / 'commands').read_text()
  assert 'iptables-legacy' not in commands
  assert (tmp_path / 'rule').exists()


def test_old_unrecorded_rule_does_not_guess_legacy_backend(tmp_path):
  (tmp_path / 'firewall-interface').write_text('usb7\n')
  result = run_firewall(tmp_path, 'firewall_down', NFT_FAIL='1')
  assert result.returncode != 0
  assert (tmp_path / 'firewall-interface').exists()
  assert 'iptables-legacy' not in (tmp_path / 'commands').read_text()


def test_invalid_backend_record_is_rejected_without_executing_it(tmp_path):
  (tmp_path / 'firewall-interface').write_text('usb7\n')
  (tmp_path / 'firewall-backend').write_text('/bin/anything\n')
  result = run_firewall(tmp_path, 'firewall_down')
  assert result.returncode != 0 and 'invalid owned firewall backend' in result.stderr
  assert not (tmp_path / 'commands').exists()


@pytest.mark.parametrize('bound', [False, True])
def test_teardown_detaches_only_owned_link_and_never_destroys_ncm(tmp_path, bound):
  gadget, state = tmp_path / 'gadget', tmp_path / 'state'
  function = gadget / 'functions/ncm.jetlink'
  function.mkdir(parents=True)
  config = gadget / 'configs/c.1'
  config.mkdir(parents=True)
  link = config / 'ncm.jetlink'
  link.symlink_to(function)
  state.mkdir()
  (state / 'ncm-owned').write_text('ncm.jetlink\n')
  (gadget / 'UDC').write_text('controller' if bound else '')
  source = SCRIPT.read_text()
  owned = 'owned() {' + source.split('owned() {', 1)[1].split('\nnetdev() {', 1)[0]
  body = source.rsplit('\n  --teardown)\n', 1)[1].split('\n    ;;', 1)[0]
  harness = r'''
set -euo pipefail
GADGET=$1
STATE=$2
FUNCTION=ncm.jetlink
fail() { echo "$*" >&2; exit 1; }
net_down() { echo down > "$STATE/net-down"; }
rmdir() { echo 'unsafe function destruction' >&2; return 99; }
'''
  result = subprocess.run(['bash', '-c', harness + owned + '\n' + body,
                           'mobile-teardown-test', str(gadget), str(state)], capture_output=True, text=True)
  if bound:
    assert result.returncode != 0 and 'unbind UDC' in result.stderr
    assert link.is_symlink()
    assert not (state / 'net-down').exists()
  else:
    assert result.returncode == 0, result.stderr
    assert not link.is_symlink()
    assert (state / 'net-down').exists()
    assert (gadget / 'bDeviceClass').read_text().strip() == '0x00'
  assert function.is_dir() and (state / 'ncm-owned').exists()


def test_helper_is_valid_shell_and_has_no_ncm_rmdir():
  subprocess.run(['bash', '-n', str(SCRIPT)], check=True)
  assert 'rmdir "$GADGET/functions/$FUNCTION"' not in SCRIPT.read_text()


@pytest.mark.parametrize('owned_function', [False, True])
def test_gadget_reuses_retained_ncm_only_with_ownership_record(tmp_path, owned_function):
  gadget, state = tmp_path / 'gadget', tmp_path / 'state'
  function = gadget / 'functions/ncm.jetlink'
  function.mkdir(parents=True)
  config = gadget / 'configs/c.1'
  config.mkdir(parents=True)
  state.mkdir()
  (gadget / 'UDC').write_text('')
  if owned_function:
    (state / 'ncm-owned').write_text('ncm.jetlink\n')
  source = SCRIPT.read_text()
  owned = 'owned() {' + source.split('owned() {', 1)[1].split('\nnetdev() {', 1)[0]
  body = source.rsplit('\n  gadget)\n', 1)[1].split('\n    ;;', 1)[0]
  harness = r'''
set -euo pipefail
GADGET=$1
STATE=$2
FUNCTION=ncm.jetlink
SCRIPT_DIR=/not-executed
fail() { echo "$*" >&2; exit 1; }
bash() { echo setup > "$STATE/setup"; }
rmdir() { echo 'unsafe function destruction' >&2; return 99; }
'''
  result = subprocess.run(['bash', '-c', harness + owned + '\n' + body,
                           'mobile-reuse-test', str(gadget), str(state)], capture_output=True, text=True)
  if owned_function:
    assert result.returncode == 0, result.stderr
    assert (config / 'ncm.jetlink').resolve() == function
    assert (gadget / 'bDeviceClass').read_text().strip() == '0xEF'
    assert (state / 'ncm-owned').exists()
  else:
    assert result.returncode != 0 and 'not owned' in result.stderr
    assert not (config / 'ncm.jetlink').exists()
  assert function.is_dir()
