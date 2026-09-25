"""Export only committed renderer/server sources; never export vehicle captures or credentials."""
import argparse
import io
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
  commit = subprocess.check_output([*git, 'rev-parse', 'HEAD'])
  archive = subprocess.check_output([*git, 'archive', 'HEAD', *PATHS])
  with tarfile.open(fileobj=io.BytesIO(archive), mode='r:') as source, tarfile.open(args.output, 'w:gz') as target:
    for member in source:
      target.addfile(member, source.extractfile(member) if member.isfile() else None)
    entry = tarfile.TarInfo('SOURCE_COMMIT')
    entry.size = len(commit)
    target.addfile(entry, io.BytesIO(commit))
  print(commit.decode().strip(), args.output)


if __name__ == '__main__':
  main()
