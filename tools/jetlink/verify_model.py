import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
spec = json.loads((ROOT / 'openpilot/selfdrive/modeld/jetlink/cinque_v2.json').read_text())
path = Path(sys.argv[1])
with path.open('rb') as f:
  digest = hashlib.sha256()
  for block in iter(lambda: f.read(4 << 20), b''):
    digest.update(block)
  actual = digest.hexdigest()
if path.stat().st_size != spec['nbytes'] or actual != spec['sha256']:
  raise SystemExit('Wrong model: this branch requires its pinned Cinque v2 ONNX')
print('Verified Cinque v2:', actual)
