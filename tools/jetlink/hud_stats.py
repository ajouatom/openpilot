"""Vehicle statistics from forwarded cereal, never from the rendering host."""
import math

from cluster_system_monitor import SystemStats


def percent(value):
  try:
    value = float(value)
    return value if math.isfinite(value) and 0 <= value <= 100 else None
  except (TypeError, ValueError):
    return None


class VehicleSystemStats:
  def __init__(self, refresh_interval_s=1., *, sm=None):
    if sm is None:
      from hud import RemoteSubMaster
      sm = RemoteSubMaster(['deviceState'])
    self.sm = sm

  def sample(self, now=None):
    self.sm.update(0)
    if not self.sm.valid['deviceState'] or not self.sm.alive['deviceState']:
      return SystemStats()
    ds = self.sm['deviceState']
    cores = tuple(percent(v) for v in ds.cpuUsagePercent)
    valid = [v for v in cores if v is not None]
    free = percent(ds.freeSpacePercent)
    return SystemStats(cpu_core_percents=cores,
                       cpu_used_percent=sum(valid) / len(valid) if valid else None,
                       memory_used_percent=percent(ds.memoryUsagePercent),
                       disk_used_percent=100 - free if free is not None else None)

  def close(self):
    pass


class VehicleCpuOverlay(VehicleSystemStats):
  def __init__(self, refresh_interval_s=1., debug=False, **kwargs):
    super().__init__(refresh_interval_s, **kwargs)

  def sample_text(self, now=None):
    average = self.sample(now).cpu_used_percent
    return 'CPU --' if average is None else f'CPU {average:.0f}%'
