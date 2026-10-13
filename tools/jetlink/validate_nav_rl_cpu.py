"""Compare the pinned original RL nav ONNX with the Orin adaptation on PC CPU.

Runs real Jetlink client/session/queues/Carrot parser over a loopback socket.
Replaces only the physical USB transport and TensorRT backend. This cannot
validate Orin timing, TensorRT numerics, native camera warp or road behavior.
Requires numpy, onnx and onnxruntime. Large graphs/reports stay outside Git.
"""
import argparse
import faulthandler
import gc
import hashlib
import json
import logging
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/jetlink')]

import numpy as np
import onnx
import onnxruntime as ort
from jetlink.client import JetlinkClient
from jetlink.onnx_meta import _parse_onnx
from jetlink.onnx_patch import strip_tinygrad_ops, patch_uint8_inputs
from jetlink.server.session import EngineHost, Session
from jetlink.spec import sha256_file, spec_from_meta
from jetlink.transport.base import LinkError
from jetlink.transport.tcp import TcpTransport
from openpilot.selfdrive.modeld.jetlink.contracts import JETSON_SPEC as SPEC, parse_outputs, validate_model_spec
from openpilot.selfdrive.modeld.jetlink.inputs import fill_inputs
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.navigation.helpers import navigation_input

TYPES = {'tensor(float16)': np.float16, 'tensor(float)': np.float32, 'tensor(uint8)': np.uint8}


def prepare_graphs(model_path, output):
  if sha256_file(str(model_path)) != (SPEC.sha256, SPEC.nbytes):
    raise ValueError('ONNX is not the pinned RL nav export')
  meta = _parse_onnx(str(model_path))
  assert spec_from_meta(meta, SPEC.sha256, SPEC.nbytes) == validate_model_spec(SPEC)
  expected_types = {name: ('uint8' if name in ('new_img', 'state_img_q') else 'float32') for name in SPEC.input_shapes}
  assert meta.input_types == expected_types
  assert meta.output_types == {name: ('uint8' if name == 'next_state_img_q' else 'float32') for name in SPEC.output_shapes}
  graph = onnx.load(model_path)
  onnx.checker.check_model(graph, full_check=True)
  stripped = strip_tinygrad_ops(graph)
  onnx.checker.check_model(graph, full_check=True)
  onnx.save(graph, output / 'reference.onnx')
  patch_uint8_inputs(graph)
  onnx.checker.check_model(graph, full_check=True)
  onnx.save(graph, output / 'patched.onnx')
  del graph
  gc.collect()
  return {'sha256_verified': True, 'onnx_full_check': True, 'input_output_pin_matches': True,
          'layout_hints_removed': stripped, 'original_input_types': meta.input_types}


def load(path, threads):
  options = ort.SessionOptions()
  options.intra_op_num_threads, options.inter_op_num_threads = threads, 1
  options.log_severity_level = 3
  return ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider'])


class CpuEngine:
  def __init__(self, path, threads):
    self.session = load(path, threads)
    self.inputs = {x.name: NS(shape=tuple(x.shape)) for x in self.session.get_inputs()}
    self.outputs = {x.name: NS(shape=tuple(x.shape)) for x in self.session.get_outputs()}
    self.buffers = {x.name: np.zeros(x.shape, TYPES[x.type]) for x in self.session.get_inputs()}
    self.result = {x.name: np.zeros(x.shape, TYPES[x.type]) for x in self.session.get_outputs()}
    self.last_gpu_us = 0
    self.fault = False

  def host_input(self, name):
    return self.buffers[name]

  def run(self):
    start = time.perf_counter()
    values = self.session.run(None, self.buffers)
    for name, value in zip(self.result, values, strict=True):
      np.copyto(self.result[name], value)
    if self.fault:
      self.result['next_state_map_features_q'].flat[0] = np.nan
    # The wire field carries CPU backend time in this test only.
    self.last_gpu_us = int((time.perf_counter() - start) * 1e6)
    return self.result

  def warm(self):
    self.run()
    return 'PC CPU warmup'

  def close(self):
    pass


class PortableTransport(TcpTransport):
  def _write(self, bufs):
    if hasattr(self.sock, 'sendmsg'):
      return super()._write(bufs)
    self._set_timeout(self._write_timeout())
    return self.sock.send(b''.join(bufs))  # Windows has no sendmsg; same framed bytes.


def frame_input(frame):
  if frame % 4 == 0:
    image = np.zeros(SPEC.warped_shape, np.uint8)
  elif frame % 4 == 1:
    image = np.full(SPEC.warped_shape, 255, np.uint8)
  elif frame % 4 == 2:
    image = np.arange(SPEC.warped_nbytes, dtype=np.uint8).reshape(SPEC.warped_shape)
  else:
    image = np.random.default_rng(39037 + frame).integers(0, 256, SPEC.warped_shape, dtype=np.uint8)
  inputs = {'desire_pulse': np.zeros(8, np.float32),
            'traffic_convention': np.array([frame < 70, frame >= 70], np.float32),
            'action_t': np.array([.2 + (frame % 9) * .05, .3 + (frame % 5) * .025], np.float32)}
  if frame in (5, 6, 30, 67, 131):
    inputs['desire_pulse'][1 + (frame if frame != 6 else 5) % 7] = 1
  # Exercise map+command, command without tiles, missing/stale route and recovery.
  if not 76 <= frame < 91:
    inputs['navigation'] = navigation_input([NS(type='turn', modifier='left' if frame < 70 else 'right',
                                               distance=100 + frame)], 0)
    if not 46 <= frame < 76:
      pixels = np.arange(65536, dtype=np.uint8).reshape(256, 256)
      inputs['nav_map'] = np.roll(pixels, frame % 256, axis=1)
  return image, inputs


def validate(model_path, output, frames=140, replay=8, threads=8):
  if frames <= 132 or not 1 <= replay <= frames:
    raise ValueError('Compare more than the full 132-frame history, then replay a nonempty prefix')
  output.mkdir(parents=True, exist_ok=True)
  report_path = output / 'full-cpu-report.json'
  report_path.unlink(missing_ok=True)
  logging.getLogger('jetlink.server').setLevel(logging.ERROR)
  print('Preparing and checking original/patched graphs', flush=True)
  graph_checks = prepare_graphs(model_path, output)
  print('Loading CPU reference and candidate', flush=True)
  faulthandler.dump_traceback_later(60, repeat=True)
  reference = load(output / 'reference.onnx', threads)
  print('Reference loaded', flush=True)
  reference_inputs = {x.name: np.zeros(x.shape, TYPES[x.type]) for x in reference.get_inputs()}
  engine = CpuEngine(output / 'patched.onnx', threads)
  print('Candidate loaded', flush=True)
  backend = NS(describe=lambda: {'backend': 'pc-ort-validation', 'runtime_version': ort.__version__, 'device': 'CPU'})
  host = EngineHost(NS(backend=backend, inventory=list))
  host.loaded = host._warm(engine, SPEC)
  faulthandler.cancel_dump_traceback_later()
  ours, peer = socket.socketpair()
  server_transport = PortableTransport(peer)
  session = Session(server_transport, host)
  thread = threading.Thread(target=session.serve_forever, daemon=True)
  thread.start()
  client = JetlinkClient(PortableTransport(ours), deadline=15)
  timings, maxima, history, rows = [], {}, {}, []
  packed = np.zeros(SPEC.packed_nelem, np.float32)
  views = {name: array.reshape(shape) for (name, shape), array in zip(
    SPEC.packed_shapes.items(), np.split(packed, np.cumsum(SPEC.packed_sizes[:-1])), strict=True)}
  previous, reference_previous = np.zeros(8, np.float32), np.zeros(8, np.float32)
  try:
    protocol = client.hello()
    assert protocol['protocol'] == 2 and protocol['stateful_v2'] is True
    assert client.ensure_engine(SPEC.sha256, SPEC.nbytes, frame_skip=4) == SPEC
    for frame in range(frames + replay):
      logical = frame if frame < frames else frame - frames
      reset = frame in (0, frames)
      if reset:
        previous.fill(0)
        reference_previous.fill(0)
        for name in SPEC.state_pairs:
          reference_inputs[name].fill(0)
      image, inputs = frame_input(logical)
      fill_inputs(views, inputs, previous)
      # Reference assignments are independent of the packed layout/queue code.
      reference_inputs['new_img'][:] = image
      desire = inputs['desire_pulse'].copy()
      desire[0] = 0
      reference_inputs['desire'][:] = np.where(desire - reference_previous > .99, desire, 0)
      reference_previous[:] = desire
      for name in ('traffic_convention', 'action_t', 'nav_map', 'navigation'):
        reference_inputs[name][...] = inputs.get(name, 0)
      expected = dict(zip([x.name for x in reference.get_outputs()], reference.run(None, reference_inputs), strict=True))
      start = time.perf_counter()
      actual = client.infer(image, packed, frame_id=frame + 1, reset=reset, want_state=(frame % 20 == 0))
      elapsed = (time.perf_counter() - start) * 1000
      timings.append(elapsed)
      for name in SPEC.output_shapes:
        left, right = expected[name], engine.result[name]
        assert np.isfinite(left).all() and np.isfinite(right).all(), (frame, name, 'non-finite')
        error = float(np.max(np.abs(left.astype(np.float64) - right.astype(np.float64))))
        maxima[name] = max(maxima.get(name, 0.), error)
        np.testing.assert_array_equal(right, left, err_msg=f'frame {frame}: {name}')
      np.testing.assert_array_equal(actual, expected['outputs'].reshape(-1))
      parsed_ref = parse_outputs(Parser(), SPEC, expected['outputs'].reshape(-1).copy())
      parsed_wire = parse_outputs(Parser(), SPEC, actual.copy())
      assert parsed_ref.keys() == parsed_wire.keys()
      for name, value in parsed_wire.items():
        assert np.isfinite(value).all(), (frame, name, 'parser non-finite')
        np.testing.assert_array_equal(value, parsed_ref[name])
      for name, next_name in SPEC.state_pairs.items():
        np.copyto(reference_inputs[name], expected[next_name])
        np.testing.assert_array_equal(host.loaded.queues.state[name], reference_inputs[name])
      checksum = hashlib.sha256(actual.tobytes()).hexdigest()
      if frame < replay:
        history[frame] = checksum
      elif frame >= frames:
        assert checksum == history[logical], 'reset replay is not deterministic'
      rows.append({'frame': frame + 1, 'reset': reset, 'output_sha256': checksum, 'cpu_loopback_ms': round(elapsed, 3)})
      if frame % 10 == 0 or frame == frames + replay - 1:
        print('FRAME', frame + 1, '/', frames + replay, 'max_abs_error', max(maxima.values()), flush=True)
    engine.fault = True
    try:
      client.infer(image, packed, frame_id=frames + replay + 1)
      raise AssertionError('Non-finite map history was accepted')
    except LinkError:
      assert client.dead and all(not np.any(x) for x in host.loaded.queues.state.values())
    report = {'status': 'passed', 'model_sha256': SPEC.sha256, 'model_bytes': SPEC.nbytes,
              'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'source_patch_sha256': hashlib.sha256(subprocess.check_output(['git', 'diff', 'HEAD'], cwd=ROOT)).hexdigest(),
              'validator_sha256': sha256_file(__file__)[0], 'graph_checks': graph_checks,
              'provider': 'CPUExecutionProvider', 'onnxruntime': ort.__version__, 'numpy': np.__version__,
              'frames_compared': frames + replay, 'continuous_frames': frames, 'reset_replay_frames': replay,
              'max_absolute_error': maxima, 'all_outputs_and_states_exact_equal': True,
              'parsed_outputs_finite_and_equal': True, 'nonfinite_map_state_rejected': True,
              'protocol': 2, 'stateful_v2_negotiated': True,
              'cpu_loopback_ms': {'median': float(np.median(timings)), 'p99': float(np.percentile(timings, 99)), 'max': max(timings)},
              'rows': rows, 'not_validated': ['TensorRT/Orin', 'USB hardware', 'native camera warp', 'road replay', 'closed-loop driving']}
    report_path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print('PASSED', report_path, flush=True)
  finally:
    client.close()
    thread.join(5)
    session.close()
    server_transport.close()
    host.close()
  assert not thread.is_alive()


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--model', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--frames', type=int, default=140)
  parser.add_argument('--replay', type=int, default=8)
  parser.add_argument('--threads', type=int, default=8)
  args = parser.parse_args()
  validate(args.model, args.output, args.frames, args.replay, args.threads)
