"""Old installed bootstraps must recognize the selected model before activation."""
import io
import json
from pathlib import Path
import sys
import tarfile

sys.path.insert(0, str(Path(__file__).parent))
from build_host_bundle import export_members


def test_selected_pin_is_exported_under_both_bootstrap_filenames():
  prefix = 'openpilot/selfdrive/modeld/jetlink/'
  selected = json.dumps({'sha256': 'b' * 64, 'nbytes': 814591106}).encode()
  files = {prefix + 'cinque_v2.json': b'{"sha256":"old"}', prefix + 'jetson_model.json': selected,
           'tools/jetlink/server.py': b'# committed runtime\n'}
  original, result = io.BytesIO(), io.BytesIO()
  with tarfile.open(fileobj=original, mode='w') as archive:
    for name, data in files.items():
      member = tarfile.TarInfo(name)
      member.size = len(data)
      archive.addfile(member, io.BytesIO(data))
  original.seek(0)
  with tarfile.open(fileobj=original) as source, tarfile.open(fileobj=result, mode='w') as target:
    export_members(source, target)
  result.seek(0)
  with tarfile.open(fileobj=result) as archive:
    assert archive.extractfile(prefix + 'cinque_v2.json').read() == selected
    assert archive.extractfile(prefix + 'jetson_model.json').read() == selected
    assert archive.extractfile('tools/jetlink/server.py').read() == files['tools/jetlink/server.py']
