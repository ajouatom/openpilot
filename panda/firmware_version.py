"""Content identity for firmware, independent of unrelated openpilot commits."""
import hashlib
from pathlib import Path


def firmware_source_version(panda_root: Path, safety_root: Path) -> str:
  inputs = []
  for label, directory in (('board', panda_root / 'board'), ('safety', safety_root), ('crypto', panda_root / 'crypto')):
    if not directory.is_dir():
      raise FileNotFoundError(directory)
    for path in directory.rglob('*'):
      relative = path.relative_to(directory)
      if any(part in ('obj', '__pycache__', 'tests') for part in relative.parts):
        continue
      suffixes = ('.c', '.h', '.s', '.ld', '.py') if label == 'crypto' else ('.c', '.h', '.s', '.ld')
      if path.is_file() and (path.suffix.lower() in suffixes or path.name == 'SConscript'):
        inputs.append((f'{label}/{relative.as_posix()}', path))
  for name in ('SConscript', 'SConstruct', 'firmware_version.py', 'certs/debug.pub', 'certs/release.pub'):
    inputs.append((name, panda_root / name))
  digest = hashlib.sha256()
  for name, path in sorted(inputs):
    digest.update(name.encode() + b'\0')
    digest.update(hashlib.sha256(path.read_bytes()).digest())
  return digest.hexdigest()[:12]
