"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Thin wrapper over the CUDA runtime API.

cuda-python moved the runtime bindings between releases (`cuda.cudart` ->
`cuda.bindings.runtime`), so import both and normalise the "(err, value)"
return convention into something that raises.
"""
from __future__ import annotations

try:
  from cuda.bindings import runtime as _rt  # cuda-python >= 12.8
except ImportError:  # pragma: no cover - depends on the wheel on the device
  from cuda import cudart as _rt  # type: ignore[no-redef]

cudaSuccess = _rt.cudaError_t.cudaSuccess
cudaMemcpyHostToDevice = _rt.cudaMemcpyKind.cudaMemcpyHostToDevice
cudaMemcpyDeviceToHost = _rt.cudaMemcpyKind.cudaMemcpyDeviceToHost
cudaHostAllocDefault = 0


class CudaError(RuntimeError):
  pass


def check(result):
  """cuda-python returns (err, *values). Raise on error, unwrap on success."""
  err, *values = result if isinstance(result, tuple) else (result,)
  if err != cudaSuccess:
    _, name = _rt.cudaGetErrorName(err)
    _, msg = _rt.cudaGetErrorString(err)
    raise CudaError(f"{name.decode() if isinstance(name, bytes) else name}: "
                    f"{msg.decode() if isinstance(msg, bytes) else msg}")
  if not values:
    return None
  return values[0] if len(values) == 1 else tuple(values)


def malloc(nbytes: int) -> int:
  return check(_rt.cudaMalloc(nbytes))


def free(ptr: int) -> None:
  check(_rt.cudaFree(ptr))


def host_alloc(nbytes: int) -> int:
  return check(_rt.cudaHostAlloc(nbytes, cudaHostAllocDefault))


def host_free(ptr: int) -> None:
  check(_rt.cudaFreeHost(ptr))


def stream_create() -> int:
  return check(_rt.cudaStreamCreate())


def stream_destroy(stream: int) -> None:
  check(_rt.cudaStreamDestroy(stream))


def stream_sync(stream: int) -> None:
  check(_rt.cudaStreamSynchronize(stream))


def memcpy_h2d_async(dst: int, src: int, nbytes: int, stream: int) -> None:
  check(_rt.cudaMemcpyAsync(dst, src, nbytes, cudaMemcpyHostToDevice, stream))


def memcpy_d2h_async(dst: int, src: int, nbytes: int, stream: int) -> None:
  check(_rt.cudaMemcpyAsync(dst, src, nbytes, cudaMemcpyDeviceToHost, stream))


def set_device(device: int) -> None:
  """Which GPU this thread's context is. A laptop has one; a desktop may not."""
  check(_rt.cudaSetDevice(device))


def device_name(device: int = 0) -> tuple[str, int, int]:
  """(name, cc_major, cc_minor) for a CUDA device."""
  props = check(_rt.cudaGetDeviceProperties(device))
  name = props.name
  return (name.decode() if isinstance(name, bytes) else str(name), props.major, props.minor)


# --- CUDA graphs -----------------------------------------------------------
# Replaying the per-frame sequence (H2D, enqueue, D2H) as a graph removes the
# per-launch CPU work, which is both latency and jitter. Safe only because every
# buffer is preallocated and never moves.

cudaStreamCaptureModeThreadLocal = 1


def stream_begin_capture(stream: int) -> None:
  check(_rt.cudaStreamBeginCapture(stream, cudaStreamCaptureModeThreadLocal))


def stream_end_capture(stream: int):
  return check(_rt.cudaStreamEndCapture(stream))


def graph_instantiate(graph):
  # Signature moved across cuda-python releases: newer takes (graph, flags),
  # older takes (graph, errNode, logBuffer, bufferSize).
  try:
    return check(_rt.cudaGraphInstantiate(graph, 0))
  except TypeError:
    return check(_rt.cudaGraphInstantiate(graph, None, None, 0))


def graph_launch(exec_graph, stream: int) -> None:
  check(_rt.cudaGraphLaunch(exec_graph, stream))


def graph_destroy(graph) -> None:
  try:
    check(_rt.cudaGraphDestroy(graph))
  except Exception:
    pass


def graph_exec_destroy(exec_graph) -> None:
  try:
    check(_rt.cudaGraphExecDestroy(exec_graph))
  except Exception:
    pass
