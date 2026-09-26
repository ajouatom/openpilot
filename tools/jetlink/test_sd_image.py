"""Image isolation, provisioning input and root-expansion safety regressions."""
import base64
import importlib.util
import io
from pathlib import Path
import tarfile

import pytest


def module(name):
  spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + '.py'))
  result = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(result)
  return result


first = module('image_first_boot')
final = module('finalize_sd_image')
grow = module('image_grow_root')
utmp = module('configure_utmp_delay')
KEY = 'ssh-ed25519 ' + base64.b64encode(b'\x00\x00\x00\x0bssh-ed25519\x00\x00\x00\x20' + bytes(32)).decode()


@pytest.mark.parametrize('value', [[], {'unknown': True}, {'hostname': '../etc/passwd'}, {'hostname': 2},
                                 {'ssh_public_keys': ['command="touch /tmp/x" ' + KEY]},
                                 {'ssh_public_keys': [KEY + '\n' + KEY]},
                                 {'ssh_public_keys': ['ssh-ed25519 invalid']},
                                 {'wifi': {'ssid': 'a\n[bad]', 'password': '12345678'}},
                                 {'wifi': {'ssid': 'a', 'password': 'short'}}])
def test_invalid_config(value):
  with pytest.raises((ValueError, TypeError)):
    first.validate_config(value)


def test_valid_config_and_wifi_escaping():
  value = {'hostname': 'carrot-jetson-123', 'ssh_public_keys': [KEY],
           'wifi': {'ssid': ' my wifi ', 'password': 'a\\b pass'}}
  assert first.validate_config(value) == value
  assert first.nm_escape(' my\\wifi ') == r'\smy\\wifi\s'


def test_accounts_do_not_inherit_human_accounts_or_passwords():
  passwd, group, shadow, gshadow = final.clean_accounts(
    'root:x:0:0:root:/root:/bin/bash\nyun:x:1000:1000:PERSON:/home/yun:/bin/bash\n'
    'other:x:1001:1001:PRIVATE:/home/other:/bin/bash\nnobody:x:65534:65534:nobody:/:/usr/sbin/nologin\n',
    'root:x:0:\nsudo:x:27:yun,other\nplugdev:x:46:yun\nyun:x:1000:\nother:x:1001:\n')
  assert 'yun' not in passwd + group + shadow + gshadow
  assert 'PRIVATE' not in passwd and 'other' not in group
  assert 'jetlink:x:1000:1000:' in passwd
  assert 'plugdev:x:46:jetlink' in group
  assert all(line.split(':')[1] == '!' for line in shadow.splitlines())


@pytest.mark.parametrize('name,link', [('../outside', None), ('a', '../outside')])
def test_archive_escape_rejected(tmp_path, name, link):
  bundle = tmp_path/'bundle.tgz'
  with tarfile.open(bundle, 'w:gz') as archive:
    member = tarfile.TarInfo(name)
    if link is not None:
      member.type = tarfile.SYMTYPE
      member.linkname = link
      archive.addfile(member)
    else:
      member.size = 1
      archive.addfile(member, io.BytesIO(b'x'))
  destination = tmp_path/'root'; destination.mkdir()
  with pytest.raises(ValueError):
    final.extract_bundle(bundle, destination)
  assert not (tmp_path/'outside').exists()


def layout():
  parts = [{'node': f'/dev/mmcblk0p{i}', 'start': i*100, 'size': 50, 'name': 'boot'} for i in range(2, 16)]
  parts += [{'node': '/dev/mmcblk0p16', 'start': 1600, 'size': 100, 'name': 'CARROT_SETUP'},
            {'node': '/dev/mmcblk0p1', 'start': 1700, 'size': 1000, 'name': 'APP'}]
  return {'label': 'gpt', 'sectorsize': 512, 'partitions': parts}


def test_growth_only_accepts_expected_last_app():
  assert grow.validate_layout(layout(), 1700, 4000)['size'] == 1000
  wrong = layout(); wrong['partitions'][0]['start'] = 3000
  with pytest.raises(ValueError):
    grow.validate_layout(wrong, 1700, 4000)
  with pytest.raises(ValueError):
    grow.validate_layout(layout(), 1600, 4000)
  with pytest.raises(ValueError):
    grow.validate_layout(layout(), 1700, 2500)


def test_write_refuses_outside_mount(tmp_path):
  root = tmp_path/'root'; root.mkdir()
  with pytest.raises(ValueError):
    final.write(root, '../outside', 'no')
  assert not (tmp_path/'outside').exists()


def test_utmp_override_refuses_unknown_pre_start_checks():
  utmp.validate_pre_commands('[Service]\nExecStartPre=/bin/sleep 2\n')
  for text in ['', 'ExecStartPre=/bin/sleep 3', 'ExecStartPre=/bin/sleep 2\nExecStartPre=/usr/bin/important-check']:
    with pytest.raises(ValueError):
      utmp.validate_pre_commands(text)
