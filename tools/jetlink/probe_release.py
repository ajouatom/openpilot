"""Before activation only: imports, pinned engine load/build and finite inference."""
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/jetlink')]


def main():
  import numpy as np
  import server  # noqa: F401 -- verify the application entry point's imports
  import hud  # noqa: F401
  import update_host  # noqa: F401
  from jetlink.server.backends.trt import TrtBackend
  from jetlink.server.cache import EngineCache
  from jetlink.server.session import EngineHost, Request
  spec = json.loads((ROOT / 'openpilot/selfdrive/modeld/jetlink/cinque_v2.json').read_text())
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
    print('CANDIDATE_PROBE_OK', spec['sha256'], flush=True)
  finally:
    host.close()


if __name__ == '__main__':
  main()
