import threading
import time

from openpilot.selfdrive.carrot.cluster.cluster_system_monitor import (
  ClusterProcessCoreUsageSampler,
  NetworkAddressProvider,
  SystemStats,
  SystemStatsSampler,
)


def _wait_for(predicate, timeout_s: float = 1.0) -> None:
  deadline = time.monotonic() + timeout_s
  while not predicate():
    if time.monotonic() >= deadline:
      raise AssertionError("asynchronous sampler did not publish before timeout")
    time.sleep(0.005)


def test_system_stats_sampling_returns_cached_value_without_waiting_for_proc(monkeypatch):
  sampler = SystemStatsSampler(refresh_interval_s=10.0)
  started = threading.Event()
  release = threading.Event()
  expected = SystemStats(memory_total_bytes=100, memory_used_bytes=25, memory_used_percent=25.0)

  def slow_sample():
    started.set()
    assert release.wait(1.0)
    return expected

  monkeypatch.setattr(sampler, "_sample_linux", slow_sample)
  try:
    assert sampler.sample(0.0) == SystemStats()
    assert started.wait(1.0)
    release.set()
    _wait_for(lambda: sampler.sample(0.0) == expected)
  finally:
    release.set()
    sampler.close()


def test_cluster_core_scan_runs_on_named_worker(monkeypatch):
  sampler = ClusterProcessCoreUsageSampler(refresh_interval_s=10.0)
  started = threading.Event()
  release = threading.Event()
  worker_names = []

  def slow_scan():
    worker_names.append(threading.current_thread().name)
    started.set()
    assert release.wait(1.0)
    return (), 1, 1

  monkeypatch.setattr(sampler, "_read_cluster_thread_samples", slow_scan)
  try:
    assert sampler.sample_text(0.0) is None
    assert started.wait(1.0)
    assert worker_names == ["cluster-core-usage"]
    release.set()
    _wait_for(lambda: sampler._previous_thread_ticks == {})
  finally:
    release.set()
    sampler.close()


def test_network_address_lookup_returns_cached_value_without_waiting_for_probe(monkeypatch):
  provider = NetworkAddressProvider(refresh_interval_s=10.0)
  started = threading.Event()
  release = threading.Event()

  def slow_param_address():
    started.set()
    assert release.wait(1.0)
    return "192.168.1.26"

  monkeypatch.setattr(provider, "_param_address", slow_param_address)
  monkeypatch.setattr(provider, "_socket_address", lambda: None)
  try:
    assert provider.address(0.0) is None
    assert started.wait(1.0)
    release.set()
    _wait_for(lambda: provider.address(0.0) == "192.168.1.26")
  finally:
    release.set()
    provider.close()
