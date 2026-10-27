"""Package committed host sources and a verified ONNX for one-time PC installation."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

from install_local_experiment import HELPERS, inspect_package


def identity(path):
  value = hashlib.sha256()
  with path.open('rb') as source:
    while block := source.read(4 << 20):
      value.update(block)
  return {'sha256': value.hexdigest(), 'size': path.stat().st_size}


def prepare(bundle, model, output):
  output.mkdir(parents=True, exist_ok=True)
  with tarfile.open(bundle) as archive:
    selection = json.load(archive.extractfile('openpilot/selfdrive/modeld/jetlink/local_host_release.json'))
    if identity(model) != selection['model']:
      raise ValueError('Model file is not the selected ONNX')
    for name in (*HELPERS, 'install_local_experiment.py'):
      (output / name).write_bytes(archive.extractfile('tools/jetlink/' + name).read())
  for source, name in ((bundle, 'host.tar.gz'), (model, 'model.onnx')):
    if source.resolve() != (output / name).resolve():
      if (output / name).exists():
        raise FileExistsError(output / name)
      shutil.copyfile(source, output / name)
  names = ('host.tar.gz', 'model.onnx', 'install_local_experiment.py', *HELPERS)
  manifest = {'format': 1, 'files': {name: identity(output / name) for name in names}}
  (output / 'package.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8', newline='\n')
  record = inspect_package(output)
  print('PACKAGE_VERIFIED', record['selection']['profile'], record['source_commit'], output, flush=True)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--bundle', type=Path, required=True)
  parser.add_argument('--model', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  prepare(args.bundle, args.model, args.output)
