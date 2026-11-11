"""Correlate cached Jetson health with model timing in the normal route log."""
import json
import math
import time


class PerformanceLog:
  def __init__(self, emit, peer, model):
    self.emit, self.peer, self.model = emit, peer, model
    self.last = float('-inf')

  def record(self, state, received_at, frame, seq, timings):
    now = time.monotonic()
    if self.peer.get('carrot_host') != 'jetson' or now - self.last < 1:
      return
    self.last = now
    try:
      health = (state or {}).get('carrot_health') or {}
      performance = health.get('performance')
      receipt_age = now - received_at
      host_age = float(health.get('age_s', float('inf')))
      sample_age = float((performance or {}).get('age_s', float('inf')))
      age = receipt_age + host_age + sample_age
      status = 'unavailable'
      sample = None
      if isinstance(performance, dict) and performance.get('version') == 1:
        status = 'stale'
        if all(math.isfinite(v) and v >= 0 for v in (receipt_age, host_age, sample_age)) and age < 3:
          # Bound older/third-party peers too. No addresses, credentials or argv.
          sample = {key: performance[key] for key in (
            'version', 'sample_mono', 'board', 'power_mode', 'cpu', 'gpu', 'emc', 'cooling',
            'power_rails', 'memory', 'top_processes', 'server_threads', 'schedstats_enabled',
            'unavailable', 'sample_duration_ms', 'status', 'error') if key in performance}
          if len(json.dumps(sample)) > 16384:
            sample = None
            status = 'oversized'
          else:
            status = performance.get('status', 'ok')
      self.emit('jetsonPerformance', mono_time=now, frame_id=frame, usb_seq=seq,
                source_commit=self.peer.get('carrot_source_commit'), model_sha256=self.model,
                status=status, sample_age_s=round(age, 3) if math.isfinite(age) else None,
                server_execution_us=timings[0], server_prepare_us=timings[1], server_total_us=timings[2],
                sample=sample, temperature_c=health.get('temp_c'), thermal_status=health.get('thermal_severity'))
    except Exception:
      pass  # Diagnostics cannot interrupt the inference reply path.
