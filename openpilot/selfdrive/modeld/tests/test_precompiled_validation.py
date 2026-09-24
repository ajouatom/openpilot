import json

import pytest

from openpilot.selfdrive.modeld import precompiled_validation as pv


@pytest.mark.parametrize('device,sizes', [('mici', ((1344, 760),)), ('tici', ((1928, 1208),)), ('tizi', ((1928, 1208),)),
                                        ('unknown', ((1928, 1208), (1344, 760)))])
def test_camera_sizes(device, sizes):
  assert pv.camera_sizes(device) == sizes


@pytest.fixture
def receipt(tmp_path):
  path = tmp_path / 'model.pkl'
  catalog = {'pickle': {'sha256': 'a' * 64}, 'runtime': {'sha256': 'b' * 64}, 'model_checkpoint': 'checkpoint',
             'runtime_directory': 'runtime'}
  (tmp_path / 'installed.json').write_text(json.dumps(catalog))
  (tmp_path / 'runtime').mkdir()
  (tmp_path / 'runtime/model_runtime.py').write_text('# pinned runtime')
  device = {'device': 'mici', 'machine_id': 'c4-unit-1', 'os': '19.8', 'kernel': 'kernel-1', 'python': '3.12', 'numpy': '2.1'}
  key = pv.validation_key(path, device, pv.camera_sizes(device['device']))
  assert not pv.validation_cached(path, key)
  pv.save_validation(path, key)
  return path, device, key


def test_same_context_reuses_only_complete_receipt(receipt):
  path, device, key = receipt
  assert pv.validation_cached(path, pv.validation_key(path, device, pv.camera_sizes('mici')))
  (path.parent / 'boot_validation.json').write_text('{')
  assert not pv.validation_cached(path, key)
  pv.save_validation(path, key)
  (path.parent / 'rejected').write_text('a' * 64)
  assert not pv.validation_cached(path, key)


@pytest.mark.parametrize('field,value', [('device', 'tici'), ('machine_id', 'c4-unit-2'), ('os', '19.9'),
                                       ('kernel', 'kernel-2'), ('python', '3.13'), ('numpy', '2.2')])
def test_device_or_software_change_invalidates(receipt, field, value):
  path, device, old_key = receipt
  device[field] = value
  key = pv.validation_key(path, device, pv.camera_sizes(device['device']))
  assert key != old_key and not pv.validation_cached(path, key)


@pytest.mark.parametrize('artifact', ['pickle', 'runtime'])
def test_artifact_change_invalidates(receipt, artifact):
  path, device, old_key = receipt
  catalog_path = path.parent / 'installed.json'
  catalog = json.loads(catalog_path.read_text())
  catalog[artifact]['sha256'] = 'c' * 64
  catalog_path.write_text(json.dumps(catalog))
  key = pv.validation_key(path, device, pv.camera_sizes('mici'))
  assert key != old_key and not pv.validation_cached(path, key)


def test_source_change_invalidates_without_repository_head(receipt, monkeypatch):
  path, device, old_key = receipt
  original = pv.Path.read_bytes
  def changed_source(source):
    value = original(source)
    return value + b'\n# changed warp\n' if source.name == 'local_gpu_warp.py' else value
  monkeypatch.setattr(pv.Path, 'read_bytes', changed_source)
  key = pv.validation_key(path, device, pv.camera_sizes('mici'))
  assert key != old_key and not pv.validation_cached(path, key)


@pytest.mark.parametrize('field,value', [('device', 'unknown'), ('machine_id', ''), ('os', '')])
def test_incomplete_identity_never_caches(receipt, field, value):
  path, device, _ = receipt
  device[field] = value
  assert pv.validation_key(path, device, pv.camera_sizes(device['device'])) is None
  assert not pv.validation_cached(path, None)


def test_camera_change_invalidates(receipt):
  path, device, old_key = receipt
  assert pv.validation_key(path, device, pv.camera_sizes('tici')) != old_key


def test_extracted_runtime_edit_invalidates_but_bytecode_does_not(receipt):
  path, device, old_key = receipt
  (path.parent / 'runtime/model_runtime.pyc').write_bytes(b'bytecode')
  assert pv.validation_key(path, device, pv.camera_sizes('mici')) == old_key
  (path.parent / 'runtime/model_runtime.py').write_text('# modified runtime')
  assert pv.validation_key(path, device, pv.camera_sizes('mici')) != old_key
