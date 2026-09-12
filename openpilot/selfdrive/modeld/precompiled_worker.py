"""Run a pinned compiled model in its own tinygrad interpreter and shared input buffer."""
import hashlib
import json
import mmap
import os
from pathlib import Path
import sys
import threading
import time
import traceback


def watch_parent(parent):
  # PR_SET_PDEATHSIG follows the creating *thread*. modeld's loading thread exits
  # after startup, so monitor the parent process instead of killing a healthy worker.
  while os.getppid() == parent:
    time.sleep(.2)
  os._exit(1)


def main():
  # Release the exclusive USB GPU when modeld exits, including a crash/SIGKILL.
  parent = os.getppid()
  if parent == 1:
    return
  threading.Thread(target=watch_parent, args=(parent,), daemon=True).start()
  control = sys.stdout.buffer
  sys.stdout = sys.stderr  # tinygrad diagnostics must not enter the control protocol
  pkl, shared_path, width, height = Path(sys.argv[1]), Path(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
  manifest = json.loads((pkl.parent / 'installed.json').read_text())
  with pkl.open('rb') as f:
    if hashlib.file_digest(f, 'sha256').hexdigest() != manifest['pickle']['sha256']:
      raise ValueError('precompiled PKL checksum mismatch')
  runtime = pkl.parent / ('runtime-' + manifest['runtime']['sha256'][:16])
  sys.path.insert(0, str(runtime))
  os.environ.update(DEV='USB+AMD:LLVM', GMMU='0', JIT_BATCH_SIZE='0')
  import numpy as np
  import model_runtime
  from tinygrad import Device, Tensor
  from openpilot.selfdrive.modeld.helpers import load_oob
  from openpilot.common.runtime_diagnostics import RuntimeDiagnostics
  from openpilot.common.swaglog import cloudlog
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

  with pkl.open('rb') as f:
    jits = load_oob(f)
    if f.read(1):
      raise ValueError('trailing precompiled model data')
  if 'run_policy' in jits or 'run_model' not in jits:
    raise ValueError('wrong precompiled runtime format')
  metadata = jits['metadata']
  if metadata['model_checkpoint'] != manifest['model_checkpoint']:
    raise ValueError('precompiled checkpoint mismatch')
  run_model = jits['run_model'][(width, height)]
  device = jits['input_devices']['model']
  if Device[device].arch != manifest['gpu_arch']:
    raise ValueError('precompiled GPU architecture mismatch')
  frame_size = model_runtime.nv12_copy_size(*get_nv12_info(width, height)[:3])
  queues, npy, frames = model_runtime.make_input_queues(metadata['input_shapes'], 4, device, frame_size)
  packed = frames['img'].base
  assert packed.dtype == np.uint8 and packed.ndim == 1 and packed.flags.c_contiguous
  views = npy | frames
  layout = {name: {'offset': view.ctypes.data - packed.ctypes.data, 'shape': list(view.shape), 'dtype': str(view.dtype)}
            for name, view in views.items()}
  count = max(section.stop for section in metadata['output_slices'].values())
  input_bytes = packed.nbytes
  total = input_bytes + count * 4
  with shared_path.open('r+b') as f:
    f.truncate(total)
    with mmap.mmap(f.fileno(), total) as shared:
      packed_shared = np.ndarray((input_bytes,), np.uint8, buffer=shared)
      packed_shared[:] = 0
      output = np.ndarray((count,), np.float32, buffer=shared, offset=input_bytes)
      queues['packed_npy_inputs'] = Tensor(packed_shared, device='NPY').realize()
      info = {'size': total, 'input_bytes': input_bytes, 'output_count': count, 'layout': layout,
              'input_shapes': metadata['input_shapes'],
              'output_slices': {k: [v.start, v.stop, v.step] for k, v in metadata['output_slices'].items()},
              'checkpoint': metadata['model_checkpoint'], 'frame_size': frame_size}
      control.write(json.dumps(info).encode() + b'\n')
      control.flush()
      diagnostics = RuntimeDiagnostics('precompiled_worker', cloudlog.event)
      while command := sys.stdin.buffer.read(1):
        if command == b'q':
          break
        if command != b'r':
          raise ValueError('invalid model worker command')
        started, cpu_started = time.monotonic(), time.thread_time()
        outs, = run_model(**{k: queues[k] for k in model_runtime.MODELD_INPUTS})
        dispatched = time.monotonic()
        result = outs.numpy().reshape(-1)
        if result.size != count or not np.isfinite(result).all():
          raise ValueError('invalid precompiled model output')
        output[:] = result
        finished, cpu_finished = time.monotonic(), time.thread_time()
        control.write(b'1\n')
        control.flush()
        # Timings include transfers/synchronization; these are not pure GPU
        # kernel durations. Keep the pipe protocol and the compiled graph intact.
        diagnostics.record(context={'gpu_arch': manifest['gpu_arch']},
                           run_model_ms=(dispatched - started) * 1000,
                           result_sync_ms=(finished - dispatched) * 1000,
                           work_ms=(finished - started) * 1000,
                           thread_cpu_ms=(cpu_finished - cpu_started) * 1000)
      del output, packed_shared, queues


if __name__ == '__main__':
  control = sys.stdout.buffer
  try:
    main()
  except Exception:
    # stderr alone loses the underlying error at the parent pipe's EOF. Send
    # the traceback over the control pipe so PCIe readiness remains retryable.
    error = traceback.format_exc()
    control.write(b'ERROR ' + json.dumps(error[-16384:]).encode() + b'\n')
    control.flush()
    raise
