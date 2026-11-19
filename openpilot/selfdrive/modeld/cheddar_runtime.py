"""Pinned internal model runtime; imported only in its compiler/worker process."""
import hashlib
import os
from pathlib import Path
import sys
import tarfile
import tempfile

REVISION = 'ec91b74b5b6b0fc924b72d3f27689679ca2438b9'
ARCHIVE_SHA256 = 'c29c0343a113e4f051087143968d05eb99d52f9a7875ba65f1d274361facc29d'
MODEL_SHA256 = '350a47ea36e554fb71634c7361b2c2a2a3da07ecb211a4c6cff3d2c3013d1d82'
CHECKPOINT = 'c8198a2a-674c-4c0b-af1d-e7c4bb28cbf8/40960/26ada517-d522-4c97-b358-c91c34bf40b1/16384'
ARCHIVE = Path(__file__).with_name('runtimes') / f'tinygrad-{REVISION}.tar.gz'


def ensure_runtime(directory):
  directory = Path(directory)
  target = directory / f'tinygrad-{REVISION}'
  marker = target / '.archive-sha256'
  if marker.is_file() and marker.read_text().strip() == ARCHIVE_SHA256:
    return target
  with ARCHIVE.open('rb') as stream:
    if hashlib.file_digest(stream, 'sha256').hexdigest() != ARCHIVE_SHA256:
      raise ValueError('internal tinygrad archive checksum mismatch')
  directory.mkdir(parents=True, exist_ok=True)
  with tempfile.TemporaryDirectory(prefix='.tinygrad-', dir=directory) as temporary:
    with tarfile.open(ARCHIVE) as archive:
      archive.extractall(temporary, filter='data')
    source = Path(temporary) / target.name
    (source / marker.name).write_text(ARCHIVE_SHA256)
    if target.exists():
      raise ValueError('incomplete internal tinygrad runtime; rebuild the runtime directory')
    os.replace(source, target)
  return target


def activate(directory):
  if 'tinygrad' in sys.modules:
    raise RuntimeError('internal tinygrad must be isolated from the legacy interpreter')
  runtime = ensure_runtime(directory)
  sys.path.insert(0, str(runtime))
  return runtime


def load_bundle(path):
  """Upstream persistent-buffer format, accepting our build's chunked files."""
  import io
  import pickle
  import struct
  import numpy as np
  from tinygrad import Tensor, Device
  from openpilot.common.file_chunker import open_file_chunked
  with open_file_chunked(path) as stream:
    size, = struct.unpack('<q', stream.read(8))
    if not 0 < size < 64 * 1024**2:
      raise ValueError('invalid internal model pickle header')
    opcodes = stream.read(size)
    if len(opcodes) != size:
      raise EOFError('incomplete internal model pickle')
    arena = Tensor(np.frombuffer(stream.read(), np.uint8).copy(), device=Device.DEFAULT).realize().uop.buffer
  unpickler = pickle.Unpickler(io.BytesIO(opcodes))
  unpickler.persistent_load = lambda pid: arena.view(*pid)
  return unpickler.load()


def dump_bundle(bundle, path):
  # Same upstream persistent-buffer encoding. Loaded arena views must first
  # bind their existing allocation before being serialized a second time.
  import io
  import pickle
  import struct
  import shutil
  from tinygrad import Device
  from tinygrad.device import Buffer
  with tempfile.TemporaryFile(dir=Path(path).parent) as arena:
    opcodes = io.BytesIO()
    def persistent_id(buffer):
      if not isinstance(buffer, Buffer) or buffer.device != Device.DEFAULT:
        return None
      arena.write(bytes((-arena.tell()) % 256))
      offset = arena.tell()
      arena.write(buffer.ensure_allocated().as_memoryview())
      return buffer.nbytes, offset
    pickler = pickle.Pickler(opcodes)
    pickler.persistent_id = persistent_id
    pickler.dump(bundle)
    with open(path, 'wb') as stream:
      stream.write(struct.pack('<q', len(opcodes.getvalue())))
      stream.write(opcodes.getvalue())
      arena.seek(0)
      shutil.copyfileobj(arena, stream)


def compile_bundle(onnx, output, cameras):
  import pickle
  import subprocess
  runtime = activate(Path(output).parent / '.cheddar-runtime')
  with open(onnx, 'rb') as stream:
    if hashlib.file_digest(stream, 'sha256').hexdigest() != MODEL_SHA256:
      raise ValueError('unexpected internal Cheddar ONNX')
  env = os.environ | {'PYTHONPATH': str(runtime) + os.pathsep + os.environ.get('PYTHONPATH', '')}
  subprocess.run([sys.executable, str(runtime / 'examples/openpilot/compile_onnx.py'), str(onnx), str(output),
                  '--out-of-band', '--benchmark-runs', '1'], env=env, check=True)
  from examples.openpilot.helpers import load_pickle
  from examples.openpilot.compile_warp import NV12Frame
  from openpilot.selfdrive.modeld.precompiled_artifact import compile_warp
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  bundle = load_pickle(output, out_of_band=True)
  bundle['model_sha256'] = MODEL_SHA256
  bundle['runtime_revision'] = REVISION
  bundle['warps'] = {}
  for width, height in cameras:
    stride, yh, uvh, _ = get_nv12_info(width, height)
    frame = NV12Frame(width, height, stride, yh, uvh, stride * (yh + uvh))
    # Keep each graph's pickle memo independent, just like upstream's separate
    # warp artifacts. Sharing a memo with the policy aliases HCQ parameter UOps.
    bundle['warps'][(width, height)] = pickle.dumps(
      compile_warp(frame, (512, 256), layout='yuv420', frames=2, benchmark_runs=1))
  temporary = str(output) + '.tmp'
  dump_bundle(bundle, temporary)
  # Read back the same persistent-buffer format before replacing the build output.
  check = load_bundle(temporary)
  if check['model_sha256'] != MODEL_SHA256 or set(check['warps']) != set(cameras):
    raise ValueError('internal model roundtrip failed')
  os.replace(temporary, output)


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--onnx', required=True)
  parser.add_argument('--output', required=True)
  parser.add_argument('--camera-resolutions', nargs='+', required=True)
  args = parser.parse_args()
  compile_bundle(args.onnx, args.output, [tuple(map(int, camera.split('x'))) for camera in args.camera_resolutions])
