"""Export only committed renderer/server sources; never export vehicle captures or credentials."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[2]
PATHS = ['openpilot/__init__.py', 'openpilot/common', 'openpilot/cereal',
         'openpilot/selfdrive/carrot/cluster', 'openpilot/selfdrive/carrot/carrot_navi.py',
         'openpilot/selfdrive/carrot/carrot_navi_cereal.py', 'openpilot/selfdrive/carrot/deceleration_source.py',
         'openpilot/selfdrive/controls/lib', 'openpilot/selfdrive/modeld/constants.py',
         'openpilot/selfdrive/modeld/jetlink/cinque_v2.json', 'openpilot/selfdrive/assets',
         'openpilot/system/hardware', 'openpilot/system/version.py',
         'opendbc_repo/opendbc/car/car.capnp', 'opendbc_repo/opendbc/car/include',
         'third_party/jetlink', 'tools/jetlink']


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  git = ['git', '-c', f'safe.directory={ROOT.as_posix()}', '-C', str(ROOT)]
  subprocess.run([*git, 'diff', '--quiet', 'HEAD', '--', *PATHS], check=True)
  untracked = subprocess.check_output([*git, 'ls-files', '--others', '--exclude-standard', '--', *PATHS])
  if untracked.strip():
    raise RuntimeError('Commit or remove untracked host source files before exporting')
  commit = subprocess.check_output([*git, 'rev-parse', 'HEAD'])
  # Inspect cleanliness using the checkout's own CRLF policy; only the Linux
  # export forces LF. Changing that policy for diff misclassifies Windows files.
  archive = subprocess.check_output([*git, '-c', 'core.autocrlf=false', '-c', 'core.eol=lf',
                                     'archive', 'HEAD', *PATHS])
  with tarfile.open(fileobj=io.BytesIO(archive), mode='r:') as source, tarfile.open(args.output, 'w:gz') as target:
    for member in source:
      target.addfile(member, source.extractfile(member) if member.isfile() else None)
    entry = tarfile.TarInfo('SOURCE_COMMIT')
    entry.size = len(commit)
    target.addfile(entry, io.BytesIO(commit))
  sha = hashlib.sha256()
  with args.output.open('rb') as bundle:
    for block in iter(lambda: bundle.read(4 << 20), b''):
      sha.update(block)
  args.output.with_suffix(args.output.suffix + '.sha256').write_text(
    f'{sha.hexdigest()}  {args.output.name}\n', encoding='utf-8')
  args.output.with_suffix(args.output.suffix + '.json').write_text(json.dumps({
    'source_commit': commit.decode().strip(), 'bundle': args.output.name,
    'size': args.output.stat().st_size, 'sha256': sha.hexdigest(),
  }, indent=2) + '\n', encoding='utf-8')
  print(commit.decode().strip(), args.output)


if __name__ == '__main__':
  main()
