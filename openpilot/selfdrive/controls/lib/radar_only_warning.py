"""Advisory only: independently measured front/corner obstacle without a lead.

TTC is constant-velocity approach time, not a safe stopping-distance estimate.
The detector never changes radar selection, longitudinal targets or FCW.
"""
import math
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.primary import snapshot_live_radar_points
from openpilot.selfdrive.carrot.radar_motion.stationary_lane import confident_ego_lane
from openpilot.selfdrive.carrot.radar_motion.controller import CORNER_RADAR_MEASUREMENT_DELAY_S


class RadarOnlyWarning:
  def __init__(self, front_delay_s: float = 0.0):
    self.front_delay_s = front_delay_s
    self.previous = None
    self.since = None
    self.last_sample = None
    self.last_pair_time = None
    self.last_alert = -math.inf
    self.armed = True
    self.last_radar_time = None

  def clear_confirmation(self):
    self.previous = self.since = self.last_sample = None

  def update(self, now: float, v_ego: float, model: Any, points: tuple,
             *, enabled: bool, selected_lead: bool = False) -> bool:
    if not math.isfinite(now):
      self.clear_confirmation()
      return False
    if self.last_sample is not None and now <= self.last_sample:
      self.clear_confirmation()
      return False
    context = confident_ego_lane(model) if enabled else None
    leads = tuple(getattr(model, 'leadsV3', ()))
    vision_absent = bool(leads) and all(math.isfinite(l.prob) and 0.0 <= l.prob < .4 for l in leads)
    qualified = []
    if context is not None and math.isfinite(v_ego) and v_ego >= 10.0:
      lane = context[0]
      for p in points:
        if (p.measured and all(math.isfinite(v) for v in (p.d_rel, p.y_rel, p.v_rel, p.yv_rel, p.v_lead))
            and 15.0 <= p.d_rel <= 150.0 and abs(p.v_lead) <= 2.0 and p.v_rel <= -8.0
            and lane.offset(p.d_rel, p.y_rel) is not None):
          qualified.append(p)
    fronts = [p for p in qualified if p.source == 'frontRadar' and p.radar_track_state >= 2]
    corners = [p for p in qualified if p.source.startswith('corner')]
    pairs = [(f, c) for f in fronts for c in corners
             if abs(f.d_rel - c.d_rel) <= 3.0 and abs(f.y_rel - c.y_rel) <= .75
             and abs(f.v_lead - c.v_lead) <= 2.0]
    if pairs:
      self.last_pair_time = now
    elif self.last_pair_time is not None and now - self.last_pair_time >= 5.0 and now - self.last_alert >= 30.0:
      self.armed = True
    if not enabled or not vision_absent or selected_lead or len(pairs) != 1:
      self.clear_confirmation()
      return False
    pair = pairs[0]
    dt = now - self.last_sample if self.last_sample is not None else math.inf
    continuous = self.previous is not None and 0 < dt <= .15 and all(
      p.source == old.source and p.track_id == old.track_id
      and abs(p.d_rel - old.d_rel - old.v_rel * dt) <= 1.0
      and abs(p.y_rel - old.y_rel - old.yv_rel * dt) <= .3
      and abs(p.v_lead - old.v_lead) <= 1.5
      for p, old in zip(pair, self.previous))
    if not continuous:
      self.since = now
    self.previous, self.last_sample = pair, now
    ttc = max(p.d_rel / -p.v_rel for p in pair)
    if (self.armed and now - self.since >= .5 and ttc <= 4.8
        and max(p.d_rel for p in pair) <= 120.0 and now - self.last_alert >= 30.0):
      self.last_alert, self.armed = now, False
      return True
    return False

  def update_live(self, now: float, cs: Any, sm: Any, enabled: bool) -> bool:
    services = ('modelV2', 'liveTracks', 'radarState', 'carControl')
    fresh = all(sm.valid[s] and 0 <= now - sm.logMonoTime[s] * 1e-9 <= .20 for s in services)
    model = sm['modelV2']
    mt = model.timestampEof * 1e-9
    rt = sm.logMonoTime['liveTracks'] * 1e-9
    enabled = (enabled and fresh and 0 <= now - mt <= .25 and abs(mt - rt) <= .15
               and sm['carControl'].longActive and cs.canValid and not cs.gasPressed and not cs.brakePressed
               and not sm['longitudinalPlan'].fcw and not sm['modelV2'].meta.hardBrakePredicted)
    if not enabled:
      self.clear_confirmation()
      return False
    # A reused radar sample must never accumulate confirmation time.
    if not sm.updated['modelV2'] or rt == self.last_radar_time:
      return False
    self.last_radar_time = rt
    raw = tuple(p for p in sm['liveTracks'].points
                if all(math.isfinite(v) for v in (p.dRel, p.yRel, p.vRel, p.yvRel)))
    points = tuple(q for p in raw for q in snapshot_live_radar_points(
      (p,), cs.vEgo, mt - rt + (CORNER_RADAR_MEASUREMENT_DELAY_S
                              if str(p.radarSource).startswith('corner') else self.front_delay_s)))
    radar = sm['radarState']
    return self.update(mt, cs.vEgo, model, points, enabled=True,
                       selected_lead=radar.leadOne.status or radar.leadTwo.status)
