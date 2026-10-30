import json
import sys
from types import SimpleNamespace
import threading

import pytest

from openpilot.common import big_model_catalog as C
from openpilot.selfdrive.modeld import model_library as library
from tools.jetlink import model_library as wire  # noqa: TID251 -- standalone host protocol


@pytest.fixture
def store(monkeypatch, tmp_path):
  monkeypatch.setenv('CARROT_BIG_MODEL_DIR', str(tmp_path))
  monkeypatch.setattr(C, 'boot_id', lambda: 'boot-one')
  library._SIZES.clear()
  return tmp_path


def test_selection_waits_for_real_reboot_and_recovers_corrupt_settings(store, monkeypatch):
  assert C.freeze_boot() == 'n16'
  C.select('flip-flop')
  assert C.preferences()['selected'] == 'flip-flop'
  assert C.freeze_boot() == C.effective_id() == 'n16'
  monkeypatch.setattr(C, 'boot_id', lambda: 'boot-two')
  assert C.freeze_boot() == C.effective_id() == 'flip-flop'
  (store / 'selection.json').write_text('{invalid')
  assert C.preferences()['selected'] == 'n16'
  assert C.effective_id() == 'flip-flop'


@pytest.mark.parametrize('key', ['../n16', '', 'unknown', 'https://example/model'])
def test_only_reviewed_models_selectable(store, key):
  with pytest.raises(ValueError):
    C.select(key)
  assert not (store / 'selection.json').exists()


def test_official_inputs_and_retired_legacy_alias():
  assert set(C.available()) == {'n16', 'flip-flop', 'light-speed'}
  assert C.catalog()['md-v3']['retired']
  assert C.noo('n16')
  assert not C.noo('md-v3') and not C.noo('flip-flop') and not C.noo('light-speed')
  assert C.model_spec('md-v3') == C.model_spec('flip-flop')
  assert C.catalog()['md-v3']['egpu']['sha256'] != C.catalog()['flip-flop']['egpu']['sha256']
  sys.path.insert(0, str(C.ROOT / 'third_party/jetlink'))
  from jetlink.spec import ModelSpec
  from openpilot.selfdrive.modeld.jetlink.contracts import validate_model_spec, model_name

  for key in C.catalog():
    spec = ModelSpec.from_dict(C.model_spec(key))
    assert validate_model_spec(spec) is spec
    assert model_name(spec) == ('N16 Nav (96244b1f)' if key == 'n16' else 'Light speed (f22b8b34)' if key == 'light-speed' else 'Flip Flop (1365126a)')
  from openpilot.selfdrive.modeld.generic_model_runtime import model_metadata

  for key, fixture in [('md-v3', 'md_v3_metadata.json'), ('flip-flop', 'flip_flop_metadata.json'), ('light-speed', 'light_speed_metadata.json')]:
    raw = json.loads((C.ROOT / 'openpilot/selfdrive/modeld/tests/fixtures' / fixture).read_text())
    checkpoint, slices, states, count = model_metadata(raw)
    expected = C.model_spec(key)
    assert checkpoint == expected['checkpoint']
    assert {k: [v.start, v.stop] for k, v in slices.items()} == expected['output_slices']
    assert count == expected['output_shapes']['outputs'][1]
    assert 'nav_map' not in raw['input_specs']


def test_delete_protects_boot_pending_and_active_but_removes_unused(store):
  C.freeze_boot()
  C.select('flip-flop')
  for key in ('n16', 'flip-flop'):
    with pytest.raises(ValueError):
      library._remove_cache(key)
  files = library.paths('md-v3')
  for path in files[:2]:
    path.write_bytes(b'model')
  files[2].mkdir(parents=True)
  (files[2] / 'model.pkl').write_bytes(b'copy')
  other = store / 'keep'
  other.write_text('keep')
  library._remove_cache('md-v3')
  assert not any(path.exists() for path in files)
  assert other.read_text() == 'keep'
  assert 'hidden' not in C.preferences()
  assert C.read_json(store / 'jetson-delete.json')['models'] == ['md-v3']
  assert 'md-v3' not in C.available()
  C.atomic_json(store / 'state.json', {'active': C.catalog()['md-v3']['egpu']})
  with pytest.raises(ValueError):
    library._remove_cache('md-v3')


def test_cache_inventory_counts_hardlink_once(store):
  import os

  first = store / 'first'
  first.write_bytes(b'123456')
  try:
    os.link(first, store / 'second')
  except OSError:
    pytest.skip('hardlinks unsupported')
  assert library.unique_bytes([store]) == 6


def test_host_deletes_only_unused_identity_and_rejects_inflight(store):
  sys.path.insert(0, str(C.ROOT / 'third_party/jetlink'))
  from jetlink.server.cache import EngineCache

  cache = EngineCache(store, SimpleNamespace(tag=lambda: 'test', suffix='.plan'))
  n16, md = C.model_spec('n16')['sha256'], C.model_spec('md-v3')['sha256']
  for sha in (n16, md):
    for path in (cache.model_path(sha), cache.entry(sha).path, cache.entry(sha).meta_path):
      path.write_bytes(b'123')
  host = SimpleNamespace(lock=threading.Lock(), cache=cache, job=None, loaded=SimpleNamespace(sha256=n16))
  result = wire.host_request(host, {'delete': [n16, md]}, C.ROOT)
  assert result['deleted'] == [md] and result['models'][n16] == 9 and result['models'][md] == 0
  with pytest.raises(ValueError):
    wire.host_request(host, {'delete': ['../unknown']}, C.ROOT)
  host.job = SimpleNamespace(state='building')
  with pytest.raises(ValueError):
    wire.host_request(host, {'delete': []}, C.ROOT)
  host.job.state = 'ready'
  assert wire.host_request(host, {'delete': []}, C.ROOT)['models'][n16] == 9


def test_client_does_no_io_onroad_or_with_old_peer(store, monkeypatch):
  monkeypatch.setattr(wire.time, 'monotonic', lambda: 100)
  params = SimpleNamespace(get_bool=lambda key: key == 'IsOnroad')
  client = object()  # Any attempted I/O would fail.
  wire.Client({wire.CAPABILITY: True}).idle(client, params)
  wire.Client({}).idle(client, SimpleNamespace(get_bool=lambda key: key == 'IsOffroad'))
  assert not (store / 'jetson-library.json').exists()


def test_jetson_inventory_is_not_presented_as_live_after_reboot(store):
  C.atomic_json(store / 'jetson-library.json', {'boot': 'old', 'updated': 100, 'models': {}})
  result = library.status()
  assert not result['jetson_inventory_fresh']
  assert all(row['jetson_bytes'] is None for row in result['models'])


@pytest.mark.parametrize('value', [{'selected': [], 'hidden': [{}]}, {'selected': None}, {'selected': 'gone'}])
def test_bad_preferences_never_select_unreviewed_model(store, value):
  C.atomic_json(store / 'selection.json', value)
  C.atomic_json(store / 'boot-selection.json', dict(value, boot=C.boot_id()))
  assert C.preferences()['selected'] == 'n16'
  assert C.freeze_boot() == C.effective_id() == 'n16'


def test_alias_delete_keeps_shared_selected_jetson_model(store):
  C.freeze_boot()
  C.select('flip-flop')
  library._remove_cache('md-v3')
  sent = []
  client = SimpleNamespace(
    _next_seq=lambda: 1,
    t=SimpleNamespace(send_json=lambda *args: sent.append(args)),
    _expect=lambda *args: SimpleNamespace(payload=json.dumps({'models': {}, 'deleted': []}).encode()),
  )
  wire.Client({wire.CAPABILITY: True})._exchange(client)
  assert sent[0][2] == {'delete': []}
  assert C.read_json(store / 'jetson-delete.json')['models'] == []


def test_catalog_releases_are_signed_and_match_exact_onnx_contracts():
  from update_host import validate_manifest, verify_signature
  for key, entry in C.catalog().items():
    release = json.loads((C.DIRECTORY / entry['jetson_release']).read_text())
    validate_manifest(release)
    verify_signature(release)
    assert release['model']['sha256'] == C.model_spec(key)['sha256']
    assert release['model']['size'] == C.model_spec(key)['nbytes']


@pytest.mark.parametrize('model', ['flip-flop', 'light-speed'])
def test_fresh_runtime_import_uses_same_model_for_both_backends(store, model):
  import os
  import subprocess
  code = '''
from pathlib import Path
from openpilot.common import big_model_catalog as C
C.select('flip-flop'); C.freeze_boot()
original = Path.is_file
Path.is_file = lambda path: path.as_posix() == '/TICI' or original(path)
from openpilot.common.jetson_model import read_spec, selected_release
from openpilot.selfdrive.modeld.big_model import DEFAULT_MANIFEST_URL, fetch_manifest
assert fetch_manifest(DEFAULT_MANIFEST_URL).sha256 == C.catalog()['flip-flop']['egpu']['sha256']
assert read_spec() == C.model_spec('flip-flop')
assert selected_release(C.DIRECTORY / '../jetlink/host_release.json', required=True)['model']['sha256'] == read_spec()['sha256']
C.select('n16')
assert read_spec() == C.model_spec('flip-flop')
assert C.effective_id() == 'flip-flop'
'''
  subprocess.run([sys.executable, '-P', '-c', code.replace('flip-flop', model)], env=dict(os.environ, PYTHONPATH=str(C.ROOT)), check=True)


def retire(monkeypatch, *keys):
  models = C.catalog()
  for key in keys:
    models[key]['retired'] = True
  monkeypatch.setattr(C, 'catalog', lambda: models)


def test_retirement_preserves_running_boot_and_falls_back_next_boot(store, monkeypatch):
  C.atomic_json(store / 'selection.json', {'selected': 'md-v3'})
  C.atomic_json(store / 'boot-selection.json', {'selected': 'md-v3', 'boot': C.boot_id()})
  retire(monkeypatch, 'md-v3')
  assert C.effective_id() == C.freeze_boot() == 'md-v3'
  assert C.preferences()['selected'] == 'n16'
  assert 'md-v3' not in C.available()
  with pytest.raises(ValueError):
    C.select('md-v3')
  offroad = SimpleNamespace(get_bool=lambda key: key == 'IsOffroad')
  assert library.reconcile_retired(offroad) == []
  monkeypatch.setattr(C, 'boot_id', lambda: 'next-boot')
  assert C.freeze_boot() == 'n16'
  assert library.reconcile_retired(offroad) == ['md-v3']


def test_curated_retirement_only_deletes_offroad_after_active_transition(store, monkeypatch):
  C.freeze_boot()
  retire(monkeypatch, 'md-v3')
  target = library.paths('md-v3')[0]
  target.write_bytes(b'old')
  keep = store / 'keep.txt'
  keep.write_text('keep')
  onroad = SimpleNamespace(get_bool=lambda key: key == 'IsOnroad')
  offroad = SimpleNamespace(get_bool=lambda key: key == 'IsOffroad')
  assert library.reconcile_retired(onroad) == [] and target.exists()
  C.atomic_json(store / 'state.json', {'active': C.catalog()['md-v3']['egpu']})
  assert library.reconcile_retired(offroad) == [] and target.exists()
  C.atomic_json(store / 'state.json', {'active': C.catalog()['n16']['egpu']})
  assert library.reconcile_retired(offroad) == ['md-v3']
  assert not target.exists() and keep.read_text() == 'keep'
  assert C.read_json(store / 'jetson-delete.json')['models'] == ['md-v3']
  assert 'md-v3' not in {m['id'] for m in library.status()['models']}


def test_hidden_preferences_migrate_to_full_curated_list(store):
  C.atomic_json(store / 'selection.json', {'selected': 'n16', 'hidden': ['md-v3', 'flip-flop']})
  assert {m['id'] for m in library.status()['models']} == set(C.available())


def test_retired_alias_never_deletes_available_flip_onnx(store, monkeypatch):
  C.freeze_boot()
  C.select('flip-flop')
  retire(monkeypatch, 'md-v3')
  library._remove_cache('md-v3')
  sent = []
  reply = {'models': {}, 'deleted': []}
  client = SimpleNamespace(_next_seq=lambda: 1, t=SimpleNamespace(send_json=lambda *args: sent.append(args)),
                           _expect=lambda *args: SimpleNamespace(payload=json.dumps(reply).encode()))
  wire.Client({wire.CAPABILITY: True})._exchange(client)
  assert sent[-1][2] == {'delete': []}
  assert C.read_json(store / 'jetson-delete.json')['models'] == []
  C.select('n16')
  reply['deleted'] = []
  wire.Client({wire.CAPABILITY: True})._exchange(client)
  assert sent[-1][2] == {'delete': []}
  assert C.read_json(store / 'jetson-delete.json')['models'] == []
