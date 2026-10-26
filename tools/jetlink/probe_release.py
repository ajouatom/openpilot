"""Before activation only: imports, pinned engine load/build and finite inference."""
from pathlib import Path
from model_pin import read_spec
import sys
import os
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/jetlink')]


def main():
  from blink_driver import prepare_optional
  prepare_optional()
  import numpy as np
  # Match the standalone systemd entry point: no inherited checkout path.
  env = dict(os.environ)
  env.pop('PYTHONPATH', None)
  env.pop('PYTHONSAFEPATH', None)
  subprocess.run([sys.executable, str(ROOT / 'tools/jetlink/server.py'), '--check-imports'],
                 cwd='/', env=env, check=True, timeout=30, stdout=subprocess.DEVNULL)
  # Importing hud alone misses the renderer imports inside main(). Exercise
  # the real entry point without opening a panel or waiting for vehicle data.
  subprocess.run([sys.executable, str(ROOT / 'tools/jetlink/hud.py'), '--help'],
                 cwd=ROOT, check=True, timeout=30, stdout=subprocess.DEVNULL)
  import update_host  # noqa: F401
  from jetlink.server.backends.trt import TrtBackend
  from jetlink.server.cache import EngineCache
  from jetlink.server.session import EngineHost, Request
  spec = read_spec(ROOT)
  host = EngineHost(EngineCache(Path(sys.argv[1]), TrtBackend()))
  try:
    host.request(Request(spec['sha256'], spec['nbytes'], spec['frame_skip']), None)
    deadline = time.monotonic() + 900
    while time.monotonic() < deadline:
      status = host.status(spec['sha256'], spec['frame_skip'])
      if status['state'] == 'ready':
        break
      if status['state'] not in ('building', 'loading'):
        raise RuntimeError(str(status))
      time.sleep(.2)
    else:
      raise TimeoutError('Candidate engine preparation timed out')
    loaded = host.loaded
    if loaded.spec.to_dict() != spec:
      raise ValueError('Candidate engine contract mismatch')
    loaded.queues.reset()
    for _ in range(3):
      loaded.queues.step_into(np.zeros(loaded.spec.warped_shape, np.uint8),
                             np.zeros(loaded.spec.packed_nelem, np.float32), loaded.host_inputs)
      values = loaded.engine.run()
      if not values or not all(np.isfinite(v).all() for v in values.values()):
        raise ValueError('Candidate inference is not finite')
      loaded.queues.feedback(values)
    print('CANDIDATE_PROBE_OK', spec['sha256'], flush=True)
  finally:
    host.close()


if __name__ == '__main__':
  main()
