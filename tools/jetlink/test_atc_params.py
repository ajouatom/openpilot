import json
from pathlib import Path

import pytest

from openpilot.common.atc_params import migrate_atc_params, right_lane_change_distance

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
  assert p.values == {**before, **({KEY: 250} if addon is True else {})}
  assert p.writes == ([(KEY, 250)] if addon is True else [])


@pytest.mark.parametrize('addon,expected', [(None, 250), (False, False), (True, 250), (350, 350)])
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
    if args[0] == 'AutoTurnControl':
      raise OSError('interrupted base-mode write')
    p.put_bool(*args)
  p.put_int = fail
  with pytest.raises(OSError):
    migrate_atc_params(p)
  assert p.values == {'AutoTurnControl': 4, KEY: 250}
  p.put_int = p.put_bool
  migrate_atc_params(p)
  assert p.values == {'AutoTurnControl': 1, KEY: 250}


def test_catalog_exposes_additional_distance_slider():
  root = Path(__file__).resolve().parents[2]
  catalog = json.loads((root / 'openpilot/selfdrive/carrot_settings.json').read_text(encoding='utf8'))
  params = {item['name']: item for item in catalog['params']}
  assert params['AutoTurnControl']['max'] == 3
  assert params[KEY]['detail_parent'] == 'AutoTurnControl'
  assert params[KEY]['control'] == 'slider'
  assert params[KEY]['default'] == 0


@pytest.mark.parametrize('stored,expected', [(0,0),(1,250),(250,250),(300,300),(350,350),(400,400),(500,500),(-1,0),(501,0)])
def test_runtime_distance_and_restored_legacy_value(stored, expected):
  assert right_lane_change_distance(Params(1, stored)) == expected


@pytest.mark.parametrize('distance', [0,50,250,300,350,400,500])
def test_migration_preserves_explicit_metres(distance):
  params = Params(1, distance)
  migrate_atc_params(params)
  assert params.values[KEY] == distance and not params.writes


def test_parameter_registry_uses_integer_storage():
  root = Path(__file__).resolve().parents[2]
  registry = (root / 'openpilot/common/params_keys.h').read_text(encoding='utf-8')
  assert '{"AutoTurnControlRightLaneChange", {PERSISTENT, INT, "0"}}' in registry
