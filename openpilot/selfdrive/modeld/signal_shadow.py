"""Opt-in internal-model comparison. Experimental predictions never enter control."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

SHADOW_DIR = Path('/data/signal-model-shadow')
BASE_SIZE = 2576
SHADOW_SIZE = 99
BASE_ROWS = 1576 + np.array([15 * t + c for t in range(33) for c in (0, 3, 6)])


def sha256_file(path):
  digest = hashlib.sha256()
  with open(path, 'rb') as stream:
    for data in iter(lambda: stream.read(1024 * 1024), b''):
      digest.update(data)
  return digest.hexdigest()


def validate_metadata(meta):
  if not isinstance(meta, dict) or meta.get('version') != 1 or meta.get('mode') != 'comparison_only':
    raise ValueError('not a comparison-only signal model')
  if meta.get('base_size') != BASE_SIZE or meta.get('shadow_size') != SHADOW_SIZE:
    raise ValueError('signal comparison output layout mismatch')
  for key in ('base_sha256', 'candidate_sha256'):
    value = meta.get(key, '')
    if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
      raise ValueError(f'invalid {key}')
  return meta


def selected_artifact(base_onnx, directory=SHADOW_DIR):
  """Read once at model startup; missing/off is the unchanged stock path."""
  directory = Path(directory)
  if not (directory / 'enabled').is_file() or (directory / 'enabled').read_text().strip() != '1':
    return None
  manifest = json.loads((directory / 'installed.json').read_text())
  meta = validate_metadata(manifest['signal_shadow'])
  artifact = directory / 'shadow_tinygrad.pkl'
  if sha256_file(base_onnx) != meta['base_sha256']:
    raise ValueError('installed shadow belongs to a different base ONNX')
  if sha256_file(artifact) != manifest['compiled_sha256']:
    raise ValueError('signal comparison compiled artifact checksum mismatch')
  return artifact, meta


def split_output(output, meta):
  validate_metadata(meta)
  if output.ndim != 1 or output.size != BASE_SIZE + SHADOW_SIZE:
    raise ValueError('signal comparison output length mismatch')
  # Return a distinct copy for diagnostics; never mutate baseline output/history.
  return output[:BASE_SIZE], (meta, output[BASE_ROWS].copy(), output[BASE_SIZE:].copy())


class ShadowLogger:
  def __init__(self, emit):
    self.emit = emit
    self.last_frame = None
    self.failed = False

  def record(self, payload, frame_id, frame_id_extra, timestamp_eof):
    if self.failed:
      return
    if self.last_frame is not None and 0 <= frame_id - self.last_frame < 4:
      return
    self.last_frame = frame_id
    try:
      meta, baseline, candidate = payload
      validate_metadata(meta)
      if baseline.size != SHADOW_SIZE or candidate.size != SHADOW_SIZE:
        raise ValueError('bad signal comparison sample shape')
      if not np.isfinite(baseline).all() or not np.isfinite(candidate).all():
        raise ValueError('nonfinite signal comparison sample')
      baseline = baseline.reshape(33, 3)
      candidate = candidate.reshape(33, 3)
      fields = {f'{name}_{axis}': values[:, i].astype(float).round(5).tolist()
                for name, values in [('baseline', baseline), ('candidate', candidate)]
                for i, axis in enumerate(('x', 'v', 'a'))}
      self.emit('signalModelShadow', mode='comparison_only', frame_id=int(frame_id),
                frame_id_extra=int(frame_id_extra), timestamp_eof=int(timestamp_eof),
                base_sha256=meta['base_sha256'], candidate_sha256=meta['candidate_sha256'], **fields)
    except Exception as exc:
      self.failed = True
      try:
        self.emit('signalModelShadowDisabled', reason=str(exc))
      except Exception:
        pass


def main():
  parser = argparse.ArgumentParser(description='Select comparison logging at the next modeld startup; never selects experimental control')
  parser.add_argument('mode', choices=['on', 'off', 'status'])
  args = parser.parse_args()
  if args.mode != 'status':
    if args.mode == 'on':
      validate_metadata(json.loads((SHADOW_DIR / 'installed.json').read_text())['signal_shadow'])
    SHADOW_DIR.mkdir(parents=True, exist_ok=True)
    temporary = SHADOW_DIR / 'enabled.tmp'
    temporary.write_text('1' if args.mode == 'on' else '0')
    temporary.replace(SHADOW_DIR / 'enabled')
  print(json.dumps({'comparison_requested': (SHADOW_DIR / 'enabled').is_file() and
                   (SHADOW_DIR / 'enabled').read_text().strip() == '1',
                   'applies': 'next modeld startup', 'control': 'original model only'}))


if __name__ == '__main__':
  main()
