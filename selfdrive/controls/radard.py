#!/usr/bin/env python3
import importlib
from typing import Optional

from cereal import messaging, car, log
from openpilot.common.numpy_fast import interp, clip
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper, Priority, config_realtime_process, DT_CTRL
from openpilot.common.swaglog import cloudlog


# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CENTER = 2.7   # (deprecated) RADAR is ~ 2.7m ahead from center of car
RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame

def makeRadarState(trackId, data, v_ego):
  return {
    "dRel": float(data[0]),
    "yRel": float(data[1]),
    "vRel":float(data[2]),
    "vLead": float(data[2]) + v_ego,
    "radarTrackId": trackId,
    "status": True,
    "radar": True,
    }
def get_lead_side(v_ego, points, md, lane_width):
  lead_msg = md.leadsV3[0]
  leadLeft = {'status': False}
  leadRight = {'status': False}

  if md is None or len(md.position.x) != 33:
    return [[],[],[],leadLeft,leadRight]

  md_y = md.position.y
  md_x = md.position.x

  leads_center = {}
  leads_left = {}
  leads_right = {}
  next_lane_y = lane_width / 2 + lane_width * 0.8
  for trackId, data in points.items():
    # d_y :  path_y - traks_y 의 diff값
    # yRel값은 왼쪽이 +값, lead.y[0]값은 왼쪽이 -값
    d_y = -data[1] - interp(data[0], md_x, md_y)
    if abs(d_y) < lane_width/2:
      leads_center[data[0]] = makeRadarState(trackId, data, v_ego) #c.get_RadarState(lead_msg.prob, float(-lead_msg.y[0]))
    elif -next_lane_y < d_y < 0:
      leads_left[data[0]] = makeRadarState(trackId, data, v_ego) #c.get_RadarState(0.0)
    elif 0 < d_y < next_lane_y:
      leads_right[data[0]] = makeRadarState(trackId, data, v_ego) #c.get_RadarState(0.0)

  if lead_msg.prob > 0.5:
    leads_center[lead_msg.x[0]] = {
      "dRel": lead_msg.x[0],
      "yRel": -lead_msg.y[0],
      "vRel": lead_msg.v[0] - v_ego,
      "vLead": lead_msg.v[0],
      "radarTrackId": -1,
      "status": True,
      "radar": False,
      }
  #ll,lr = [[l[k] for k in sorted(list(l.keys()))] for l in [leads_left,leads_right]]
  #lc = sorted(leads_center.values(), key=lambda c:c["dRel"])
  ll = list(leads_left.values())
  lr = list(leads_right.values())

  if leads_center:
    dRel_min = min(leads_center.keys())
    lc = [leads_center[dRel_min]]
  else:
    lc = {}

  leadLeft = min((lead for dRel, lead in leads_left.items() if lead['dRel'] > 5.0), key=lambda x: x['dRel'], default=leadLeft)
  leadRight = min((lead for dRel, lead in leads_right.items() if lead['dRel'] > 5.0), key=lambda x: x['dRel'], default=leadRight)

  #filtered_leads_left = {dRel: lead for dRel, lead in leads_left.items() if lead['dRel'] > 5.0}
  #if filtered_leads_left:
  #  dRel_min = min(filtered_leads_left.keys())
  #  leadLeft = filtered_leads_left[dRel_min]

  #filtered_leads_right = {dRel: lead for dRel, lead in leads_right.items() if lead['dRel'] > 5.0}
  #if filtered_leads_right:
  #  dRel_min = min(filtered_leads_right.keys())
  #  leadRight = filtered_leads_right[dRel_min]

  return [ll,lc,lr, leadLeft, leadRight]

class RadarD:
  def __init__(self, radar_ts: float, delay: int = 0):
    self.points: dict[int, tuple[float, float, float]] = {}

    self.radar_tracks_valid = False

  def update(self, sm: messaging.SubMaster, rr: Optional[car.RadarData]):
    radar_points = []
    radar_errors = []
    if rr is not None:
      radar_points = rr.points
      radar_errors = rr.errors

    self.radar_tracks_valid = len(radar_errors) == 0

    # carrot
    if len(sm['modelV2'].temporalPose.trans):
      model_v_ego = sm['modelV2'].temporalPose.trans[0]
    else:
      model_v_ego = 0
    leads_v3 = sm['modelV2'].leadsV3
    y_rel = 0.0
    if len(leads_v3) > 1 and leads_v3[0].prob > 0.5:
      y_rel = leads_v3[0].y[0]

    self.points = {}
    for pt in radar_points:
      y_rel = y_rel if pt.trackId == 0 else pt.yRel
      self.points[pt.trackId] = (pt.dRel, y_rel, pt.vRel)

    self.radar_state = log.RadarState.new_message()
    if len(leads_v3) > 1:
      ll, lc, lr, self.radar_state.leadLeft, self.radar_state.leadRight = get_lead_side(model_v_ego, self.points, sm['modelV2'], 2.8)
      self.radar_state.leadsLeft = list(ll)
      self.radar_state.leadsCenter = list(lc)
      self.radar_state.leadsRight = list(lr)

  def publish(self, pm: messaging.PubMaster):
    #carrot
    assert self.radar_state is not None
    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = True #self.radar_state_valid
    radar_msg.radarState = self.radar_state
    #radar_msg.radarState.cumLagMs = lag_ms
    pm.send("radarState", radar_msg)

    tracks_msg = messaging.new_message('liveTracks', len(self.points))
    tracks_msg.valid = self.radar_tracks_valid
    for index, tid in enumerate(sorted(self.points.keys())):
      tracks_msg.liveTracks[index] = {
        "trackId": tid,
        "dRel": float(self.points[tid][0]),
        "yRel": float(self.points[tid][1]),
        "vRel": float(self.points[tid][2]),
      }
    pm.send('liveTracks', tracks_msg)
    #return tracks_msg


# publishes radar tracks
def main():
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  with car.CarParams.from_bytes(Params().get("CarParams", block=True)) as msg:
    CP = msg
  cloudlog.info("radard got CarParams")

  # import the radar from the fingerprint
  cloudlog.info("radard is importing %s", CP.carName)
  RadarInterface = importlib.import_module(f'selfdrive.car.{CP.carName}.radar_interface').RadarInterface

  # *** setup messaging
  can_sock = messaging.sub_sock('can')
  #pub_sock = messaging.pub_sock('liveTracks')
  sm = messaging.SubMaster(['modelV2'], frequency=int(1./DT_CTRL))
  pm = messaging.PubMaster(['radarState', 'liveTracks'])

  RI = RadarInterface(CP)

  # TODO timing is different between cars, need a single time step for all cars
  # TODO just take the fastest one for now, and keep resending same messages for slower radars
  rk = Ratekeeper(1.0 / CP.radarTimeStep, print_delay_threshold=None)
  RD = RadarD(CP.radarTimeStep, RI.delay)

  while 1:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    rr = RI.update(can_strings)
    sm.update(0)
    if rr is None:
      continue

    RD.update(sm, rr)
    #msg = RD.publish()
    #pub_sock.send(msg.to_bytes())
    RD.publish(pm)

    rk.monitor_time()


if __name__ == "__main__":
  main()
