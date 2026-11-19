"""Internal GPU owner using the exact Cheddar tinygrad, separate from DM/eGPU."""
import json
import mmap
import os
from pathlib import Path
import sys
import threading
import time
import traceback


def watch_parent(parent):
  # The inference thread inherits modeld's core 7 / FIFO 54 scheduling.
  if hasattr(os, 'sched_setscheduler'):
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  while os.getppid() == parent:
    time.sleep(.1)
  os._exit(1)


def main():
  control = sys.stdout.buffer
  sys.stdout = sys.stderr
  parent = os.getppid()
  if parent == 1:
    return
  threading.Thread(target=watch_parent, args=(parent,), daemon=True).start()
  pkl, shared_path, width, height = Path(sys.argv[1]), Path(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
  from openpilot.selfdrive.modeld.cheddar_runtime import activate, load_bundle
  activate(pkl.parent / '.cheddar-runtime')
  import numpy as np
  from openpilot.selfdrive.modeld.cheddar_adapter import CheddarAdapter
  from openpilot.selfdrive.modeld.output_validation import validate_compiled_output

  adapter = CheddarAdapter(load_bundle(pkl), width, height)
  packed = adapter.packed
  layout = {name: {'offset': value.ctypes.data - packed.ctypes.data, 'shape': list(value.shape), 'dtype': str(value.dtype)}
            for name, value in adapter.views.items()}
  input_bytes = packed.nbytes
  total = input_bytes + adapter.count * 4
  with shared_path.open('r+b') as file:
    file.truncate(total)
    with mmap.mmap(file.fileno(), total) as shared:
      # Bind NPY to the shared storage; no additional per-frame IPC image copy.
      shared_inputs = np.ndarray(packed.shape, np.uint8, buffer=shared)
      shared_inputs[:] = packed
      adapter.bind_shared(shared_inputs)
      output = np.ndarray((adapter.count,), np.float32, buffer=shared, offset=input_bytes)
      for _ in range(3):
        validate_compiled_output(adapter.run(), adapter.count)
      adapter.reset_state()
      info = {'size': total, 'layout': layout, 'input_bytes': input_bytes, 'output_count': adapter.count,
              'frame_size': adapter.frame_size, 'checkpoint': adapter.checkpoint,
              'output_slices': {name: [section.start, section.stop, section.step] for name, section in adapter.output_slices.items()},
              'affinity': sorted(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else [],
              'priority': os.sched_getparam(0).sched_priority if hasattr(os, 'sched_getparam') else 0}
      control.write(json.dumps(info).encode() + b'\n')
      control.flush()
      while command := sys.stdin.buffer.read(1):
        if command == b'r':
          result = adapter.run()
          validate_compiled_output(result, adapter.count)
          output[:] = result
          control.write(b'1\n')
        elif command == b'z':
          adapter.reset_state()
          shared_inputs[:] = 0
          output[:] = 0
          control.write(b'Z\n')
        else:
          raise ValueError('invalid internal model command')
        control.flush()
      # Tensor/NPY views own the mapping until process exit; don't close it early.
      os._exit(0)


if __name__ == '__main__':
  try:
    main()
  except BaseException:
    traceback.print_exc(file=sys.stderr)
    sys.stderr.flush()
    os._exit(1)
