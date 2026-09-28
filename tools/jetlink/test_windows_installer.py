import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
spec = importlib.util.spec_from_file_location('jetson_prepare', Path(__file__).parent / 'windows_installer/prepare.py')
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


def sha(data):
  return hashlib.sha256(data).hexdigest()


@pytest.fixture
def package(tmp_path):
  support = tmp_path / 'support'
  support.mkdir()
  raw = b'A' * 16384
  patched = raw[:4096] + b'N' * 512 + raw[4608:]
  patch = dict(format=1, root_offset=2048, root_bytes=len(raw)-2048, root_sha256=sha(raw[2048:]),
               partition_guard=dict(offset=1024, data=base64.b64encode(raw[1024:1536]).decode()),
               patches=[dict(offset=4096, before=base64.b64encode(raw[4096:4608]).decode(),
                             after=base64.b64encode(patched[4096:4608]).decode(),
                             before_sha256=sha(raw[4096:4608]), after_sha256=sha(patched[4096:4608]))])
  data = json.dumps(patch).encode()
  (support / 'offline-usbc.json').write_bytes(data)
  (support / 'carrot-jetson.img.zst').write_bytes(raw)
  release = dict(image_bytes=len(raw), image_sha256=sha(raw), prepared_sha256=sha(patched),
                 compressed_bytes=len(raw), compressed_sha256=sha(raw), patch_sha256=sha(data))
  (support / 'release.json').write_text(json.dumps(release))
  return tmp_path, raw, patched


def test_prepare_patch_and_repeat_without_touching_original(package):
  root, original, expected = package
  prepare.prepare(root, open)
  assert (root / 'prepared.img').read_bytes() == expected
  assert (root / 'support/carrot-jetson.img.zst').read_bytes() == original
  prepare.prepare(root, lambda *_: pytest.fail('Valid prepared file decompressed again'))


@pytest.mark.parametrize('name', ['carrot-jetson.img.zst', 'offline-usbc.json'])
def test_bad_input_never_produces_ready_image(package, name):
  root, *_ = package
  with (root / 'support' / name).open('ab') as stream:
    stream.write(b'corrupt')
  with pytest.raises(RuntimeError):
    prepare.prepare(root, open)
  assert not (root / 'prepared.img').exists()


def test_bad_prepared_image_is_never_accepted(package):
  root, *_ = package
  (root / 'prepared.img').write_bytes(b'corrupt')
  with pytest.raises(RuntimeError):
    prepare.prepare(root, open)


def test_interrupted_preparation_can_be_retried(package):
  root, _, expected = package
  (root / 'prepared.img.partial').write_bytes(b'interrupted')
  prepare.prepare(root, open)
  assert (root / 'prepared.img').read_bytes() == expected


def test_no_space_refuses_without_output(package, monkeypatch):
  root, *_ = package
  monkeypatch.setattr(prepare.shutil, 'disk_usage', lambda _: type('Usage', (), {'free': 0})())
  with pytest.raises(RuntimeError):
    prepare.prepare(root, open)
  assert not (root / 'prepared.img.partial').exists()


def test_wrong_decompression_never_prepares(package):
  import io
  root, *_ = package
  with pytest.raises(RuntimeError):
    prepare.prepare(root, lambda *_: io.BytesIO(b'wrong'))
  assert not (root / 'prepared.img').exists()


def test_integrated_image_requires_no_hotfix_and_keeps_full_verification(package):
  root, original, _ = package
  manifest = root / 'support/release.json'
  release = json.loads(manifest.read_text())
  release.update(preparation='integrated', prepared_sha256=sha(original))
  release.pop('patch_sha256')
  manifest.write_text(json.dumps(release))
  (root / 'support/offline-usbc.json').unlink()
  prepare.prepare(root, open)
  assert (root / 'prepared.img').read_bytes() == original
  prepare.prepare(root, lambda *_: pytest.fail('Valid image decompressed again'))
  (root / 'prepared.img').unlink()
  (root / 'support/carrot-jetson.img.zst').write_bytes(b'corrupt')
  with pytest.raises(RuntimeError):
    prepare.prepare(root, open)
  assert not (root / 'prepared.img').exists()


@pytest.mark.parametrize('mode', ['integrated', 'unknown'])
def test_invalid_preparation_metadata_refuses_before_writing(package, mode):
  root, *_ = package
  manifest = root / 'support/release.json'
  release = json.loads(manifest.read_text())
  release['preparation'] = mode
  manifest.write_text(json.dumps(release))
  with pytest.raises(RuntimeError):
    prepare.prepare(root, open)
  assert not (root / 'prepared.img.partial').exists()


def test_public_integrated_package_requires_checks_for_exact_image(tmp_path, monkeypatch):
  import build_windows_installer as publisher
  from types import SimpleNamespace
  image = SimpleNamespace(stat=lambda: SimpleNamespace(st_size=40 * (1 << 30)))
  compressed = SimpleNamespace(stat=lambda: SimpleNamespace(st_size=100))
  monkeypatch.setattr(publisher, 'digest', lambda path: 'a' * 64 if path is image else 'b' * 64)
  metadata = tmp_path / 'candidate.json'
  metadata.write_text(json.dumps(dict(storage_format=1, data_partition=17, image_bytes=40 * (1 << 30),
    source_commit='c' * 40, state='PROTECTED_CANDIDATE_NOT_BOOT_TESTED', image_sha256='a' * 64,
    compressed_bytes=100, compressed_sha256='b' * 64)))
  private = publisher.integrated_release(image, compressed, metadata)
  assert private['version'] == 'v0.4.0-storage-candidate'
  validation = tmp_path / 'validation.json'
  evidence = dict(image_sha256='a' * 64, card_readback=True, protected_boot=True, model_ready=True,
                  wifi_reconnect=True, ssh_persistence=True, data_update=True, power_cut_cycles=0)
  validation.write_text(json.dumps(evidence))
  released = publisher.integrated_release(image, compressed, metadata, validation)
  assert released['version'] == 'v0.4.0-protected-preview' and released['power_cut_cycles'] == 0
  for changes in ({'image_sha256': 'd' * 64}, {'protected_boot': False}, {'data_update': False}, {'power_cut_cycles': -1}):
    validation.write_text(json.dumps(evidence | changes))
    with pytest.raises(ValueError, match='Physical validation'):
      publisher.integrated_release(image, compressed, metadata, validation)
