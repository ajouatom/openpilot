"""Keep optional Xiaoge inference out of onroad startup."""


class XiaogeStartupGate:
  SERVICES = ('deviceState', 'selfdriveState', 'modelV2', 'carState', 'pandaStates')
  SETTLE_SECONDS = 0.5

  def __init__(self):
    self.start_messages = None
    self.ready_since = None
    self.last_check = None
    self.ready = False

  def update(self, started, sm, CP, controls_ready, now):
    if not started:
      self.__init__()
      return False
    if self.start_messages is None:
      # Compare each publisher against itself; publishers may use different
      # monotonic clock domains across suspend. Require new onroad observations.
      self.start_messages = {s: sm.logMonoTime[s] for s in self.SERVICES}
      return False
    if self.ready:
      return True

    # Do not count a gap in manager observations as continuous readiness.
    if self.last_check is not None and now - self.last_check > 1.0:
      self.ready_since = None
    self.last_check = now
    fresh = (sm.all_checks(self.SERVICES) and
             all(sm.logMonoTime[s] > self.start_messages[s] for s in self.SERVICES))
    pandas = sm['pandaStates']
    configs = CP.safetyConfigs
    safety_ready = bool(configs) and len(pandas) >= len(configs) and all(
      (p.safetyModel == configs[i].safetyModel and p.safetyParam == configs[i].safetyParam)
      if i < len(configs) else str(p.safetyModel) in ('silent', 'noOutput')
      for i, p in enumerate(pandas))
    healthy = (controls_ready and fresh and safety_ready and sm['selfdriveState'].engageable
               and sm['carState'].canValid and not sm['carState'].canTimeout)
    if not healthy:
      self.ready_since = None
      return False
    if self.ready_since is None:
      self.ready_since = now
    self.ready = now - self.ready_since >= self.SETTLE_SECONDS
    return self.ready
