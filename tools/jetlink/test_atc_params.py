import json
from pathlib import Path

import pytest

from openpilot.common.atc_params import migrate_atc_params

KEY = 'AutoTurnControlRightLaneChange'


class Params:
  def __init__(self, mode, addon=None):
    self.values = {'AutoTurnControl': mode}
    if addon is not None:
      self.values[KEY] = addon
    self.writes = []

  def get(self, key):
    return self.values.get(key)

  def get_int(self, key):
    return self.values.get(key, 0)

  def put_bool(self, key, value):
    self.writes.append((key, value))
    self.values[key] = value

  put_int = put_bool


@pytest.mark.parametrize('mode', [0, 1, 2, 3])
@pytest.mark.parametrize('addon', [None, False, True])
def test_existing_modes_unchanged(mode, addon):
  p = Params(mode, addon)
  before = dict(p.values)
  migrate_atc_params(p)
  assert p.values == before and not p.writes


@pytest.mark.parametrize('addon,expected', [(None, True), (False, False), (True, True)])
def test_retired_mode_migration_is_idempotent(addon, expected):
  p = Params(4, addon)
  migrate_atc_params(p)
  assert p.values == {'AutoTurnControl': 1, KEY: expected}
  before = list(p.writes)
  migrate_atc_params(p)
  assert p.writes == before


def test_interrupted_migration_retries_without_losing_addon():
  p = Params(4)
  def fail(*args):
    raise OSError('interrupted base-mode write')
  p.put_int = fail
  with pytest.raises(OSError):
    migrate_atc_params(p)
  assert p.values == {'AutoTurnControl': 4, KEY: True}
  p.put_int = p.put_bool
  migrate_atc_params(p)
  assert p.values == {'AutoTurnControl': 1, KEY: True}


def test_catalog_exposes_only_additional_detail_toggle():
  root = Path(__file__).resolve().parents[2]
  catalog = json.loads((root / 'openpilot/selfdrive/carrot_settings.json').read_text(encoding='utf8'))
  params = {item['name']: item for item in catalog['params']}
  assert params['AutoTurnControl']['max'] == 3
  assert params[KEY]['detail_parent'] == 'AutoTurnControl'
  assert params[KEY]['control'] == 'toggle'
  assert params[KEY]['default'] == 0
