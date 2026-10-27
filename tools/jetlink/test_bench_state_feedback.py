"""Offline experiment guards, copy direction/order and reset semantics."""
from types import SimpleNamespace as NS
import subprocess

import numpy as np
import pytest

from bench_state_feedback import enqueue_candidate, reset_device_feedback, require_stopped_service, digest_outputs, digest_file


def fixture():
  calls = []
  def binding(name, base):
    return NS(name=name, device_ptr=base, host_ptr=base+1, nbytes=8, host=np.zeros(2, np.float32))
  image, state, next_state, output = [binding(n, p) for n, p in
                                       (('image', 10), ('state', 20), ('next', 30), ('output', 40))]
  engine = NS(inputs={'image': image, 'state': state}, outputs={'next': next_state, 'output': output},
              _state_host_sources={'state': next_state}, stream=7,
              context=NS(execute_async_v3=lambda stream: calls.append(('execute', stream)) or True))
  cuda = NS(memcpy_h2d_async=lambda *a: calls.append(('h2d', *a)),
            memcpy_d2h_async=lambda *a: calls.append(('d2h', *a)),
            stream_sync=lambda *a: calls.append(('sync', *a)), check=lambda x: x,
            _rt=NS(cudaMemcpyKind=NS(cudaMemcpyDeviceToDevice=3),
                   cudaMemcpyAsync=lambda *a: calls.append(('d2d', *a))))
  return engine, cuda, calls


def test_candidate_keeps_nonstate_upload_and_all_output_checks_possible():
  engine, cuda, calls = fixture()
  enqueue_candidate(engine, cuda, True)
  assert calls == [('h2d', 10, 11, 8, 7), ('d2d', 20, 30, 8, 3, 7), ('execute', 7),
                   ('d2h', 31, 30, 8, 7), ('d2h', 41, 40, 8, 7)]


def test_host_comparison_uses_prior_output_staging():
  engine, cuda, calls = fixture()
  enqueue_candidate(engine, cuda, False)
  assert calls[1] == ('h2d', 20, 31, 8, 7)


def test_reset_targets_next_state_device_source_and_synchronizes():
  engine, cuda, calls = fixture()
  reset_device_feedback(engine, cuda)
  assert calls == [('h2d', 30, 31, 8, 7), ('sync', 7)]
  calls.clear()
  engine._state_host_sources['state'].host[0] = 1
  with pytest.raises(ValueError, match='CPU state must be reset'):
    reset_device_feedback(engine, cuda)
  assert not calls


def test_failed_enqueue_never_publishes_device_outputs():
  engine, cuda, calls = fixture()
  engine.context.execute_async_v3 = lambda stream: False
  with pytest.raises(RuntimeError):
    enqueue_candidate(engine, cuda, True)
  assert all(row[0] != 'd2h' for row in calls)


@pytest.mark.parametrize('state', ['active', 'activating', 'deactivating', 'unknown', ''])
def test_running_or_uncertain_service_is_rejected(state):
  with pytest.raises(RuntimeError):
    require_stopped_service(lambda *a, **kw: NS(stdout=f'LoadState=loaded\nActiveState={state}\n'))


def test_missing_service_or_systemctl_failure_is_not_permission():
  with pytest.raises(RuntimeError):
    require_stopped_service(lambda *a, **kw: NS(stdout='LoadState=not-found\nActiveState=inactive\n'))
  def failed(*a, **kw):
    raise subprocess.CalledProcessError(1, a[0])
  with pytest.raises(subprocess.CalledProcessError):
    require_stopped_service(failed)
  require_stopped_service(lambda *a, **kw: NS(stdout='LoadState=loaded\nActiveState=inactive\n'))


def test_digest_checks_bits_in_every_output_including_negative_zero():
  a = {'output': np.ones(2, np.float32), 'state': np.zeros(2, np.float16)}
  expected = digest_outputs(a)
  a['state'][0] = -0.
  assert digest_outputs(a)['output'] == expected['output']
  assert digest_outputs(a)['state'] != expected['state']


def test_engine_digest_supports_jetson_python_without_file_digest(tmp_path):
  path = tmp_path / 'engine.plan'
  path.write_bytes(b'abc')
  assert digest_file(path) == 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad'
