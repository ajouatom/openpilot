import hashlib
import json
import subprocess
import sys
import tarfile

import pytest

import build_host_bundle as bundle


def test_bundle_tracks_exact_commit_and_refuses_uncommitted_sources(tmp_path, monkeypatch):
  repo = tmp_path / 'repo'
  repo.mkdir()
  git = ['git', '-c', f'safe.directory={repo.as_posix()}', '-C', str(repo)]
  subprocess.run([*git, 'init', '-q'], check=True)
  (repo / 'host').mkdir()
  source = repo / 'host/server.py'
  source.write_text('print("ready")\n')
  subprocess.run([*git, 'add', '.'], check=True)
  subprocess.run([*git, '-c', 'user.name=Bundle Test', '-c', 'user.email=bundle@example.invalid',
                  '-c', 'commit.gpgsign=false', 'commit', '-qm', 'fixture'], check=True)
  expected_commit = subprocess.check_output([*git, 'rev-parse', 'HEAD']).decode().strip()
  output = tmp_path / 'host.tar.gz'
  monkeypatch.setattr(bundle, 'ROOT', repo)
  monkeypatch.setattr(bundle, 'PATHS', ['host'])
  monkeypatch.setattr(sys, 'argv', ['build_host_bundle.py', str(output)])
  bundle.main()
  manifest = json.loads(output.with_suffix('.gz.json').read_text())
  assert manifest['sha256'] == hashlib.sha256(output.read_bytes()).hexdigest()
  assert manifest['source_commit'] == expected_commit
  assert manifest['size'] == output.stat().st_size
  # Linux sha256sum treats a Windows CR as part of the archive filename.
  assert output.with_suffix('.gz.sha256').read_bytes() == (
    f"{manifest['sha256']}  {output.name}\n".encode())
  with tarfile.open(output) as archive:
    assert archive.extractfile('SOURCE_COMMIT').read().decode().strip() == expected_commit
    assert archive.extractfile('host/server.py').read() == b'print("ready")\n'
  source.write_text('print("uncommitted")\n')
  with pytest.raises(subprocess.CalledProcessError):
    bundle.main()
  source.write_text('print("ready")\n')
  (repo / 'host/new.py').write_text('uncommitted\n')
  with pytest.raises(RuntimeError, match='untracked'):
    bundle.main()
