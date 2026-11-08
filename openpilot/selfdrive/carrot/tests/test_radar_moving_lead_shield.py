"""11-08 tunnel FCW (02_logs/measurements.md s49, s53): the vision lead flickered 19-41 m frame to frame and for 1-3
frames landed next to a stationary tunnel reflector 18-32 m ahead; _match_moving adopted the reflector as a fresh lead
while the real lead (frontRadar, 40 m, central, continuously measured) was still there -> FCW, accel -4.

Fix: a fresh stationary-class track closer than a central moving lead we keep measuring waits for _match_stationary;
inside a navi-announced tunnel every fresh stationary-class track does. Everything else must stay as it was: a lead
braking to a stop, a stopped car revealed by a cutting-out lead, a jam tail, a moving cut-in.
"""
from dataclasses import dataclass
from types import SimpleNamespace

from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarController

DT = 0.05
V_EGO = 20.8  # 75 km/h


@dataclass(frozen=True)
class Point:
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float = 0.0
  a_rel: float = 0.0
  measured: bool = True
  source: str = "frontRadar"
  v_lead: float | None = None
  a_lead: float | None = None
  yv_rel: float = 0.0
  j_lead: float = 0.0
  trackState: int = 3


def model(lead_d, lead_v, probability=0.95, lead_y=0.0):
  leads = () if lead_d is None else (SimpleNamespace(
    prob=probability, x=(lead_d + 1.52,), y=(-lead_y,), v=(lead_v,), a=(0.0,), xStd=(2.0,), yStd=(0.5,), vStd=(1.5,),
  ),)
  return SimpleNamespace(
    position=SimpleNamespace(x=tuple(float(x) for x in range(0, 200, 2)), y=(0.0,) * 100),
    leadsV3=leads,
    velocity=SimpleNamespace(x=(V_EGO,)),
    meta=SimpleNamespace(laneChangeState="off"),
  )


def moving(tid, d, v_lead, y=0.0):
  return Point(tid, d, y, v_rel=v_lead - V_EGO)


def stopped(tid, d, y=0.0):
  return Point(tid, d, y, v_rel=-V_EGO)


def run(frames, in_tunnel=False, v_ego=None):
  """frames: list of (points, model) at 20 Hz -> leadOne (radarTrackId, dRel) per frame, None when empty."""
  controller = DPathRadarController(enable_radar_tracks=3)
  out = []
  for i, (points, md) in enumerate(frames):
    lead = controller.update(i * DT, V_EGO if v_ego is None else v_ego[i], points, md, in_tunnel=in_tunnel).lead_one
    out.append(None if not lead or not lead["status"] else (lead["radarTrackId"], round(lead["dRel"], 1)))
  return out


def test_tunnel_reflector_next_to_jumped_vision_never_becomes_lead():
  # 0.5 s: lead track 38 at 40 m with its SCC object, reflector track 42 stationary at 29 m beside the path
  frames = []
  for i in range(10):
    frames.append(([moving(38, 40.0, 22.3), Point(0, 40.0, 0.0, v_rel=22.3 - V_EGO, source="scc"), stopped(42, 29.0 - i, -0.1)],
                   model(40.0, 22.3)))
  # the vision lead jumps onto the reflector for 3 frames (recorded 22:20:51.06-51.15), then returns
  for i in range(3):
    d = 19.0 - i
    frames.append(([moving(38, 40.0, 22.3), Point(0, 40.0, 0.0, v_rel=22.3 - V_EGO, source="scc"), stopped(42, d + 3.5, -0.1)],
                   model(d, 3.8, probability=0.995)))
  frames.append(([moving(38, 40.0, 22.3), Point(0, 40.0, 0.0, v_rel=22.3 - V_EGO, source="scc"), stopped(42, 15.0, -0.1)], model(40.0, 22.3)))
  leads = run(frames)
  assert all(lead is not None and lead[0] in (38, 0) and lead[1] > 35.0 for lead in leads[5:]), leads[5:]


def test_lead_braking_to_a_stop_is_kept_through_vision_flicker():
  # lead brakes -8 m/s^2 from t=2.0, ego -6 m/s^2 from t=2.3 (stops ~25 m behind); vision prob drops to 0.2 every 7th frame
  frames = []; v_ego = []; d = 35.0; v = V_EGO; ve = V_EGO
  for i in range(140):
    if i >= 40:
      v = max(0.0, v - 8.0 * DT)
    if i >= 46:
      ve = max(0.0, ve - 6.0 * DT)
    d += (v - ve) * DT
    prob = 0.2 if (i > 40 and i % 7 == 0) else 0.95
    frames.append(([Point(40, d, 0.0, v_rel=v - ve)], model(d, v, probability=prob)))
    v_ego.append(ve)
  leads = run(frames, v_ego=v_ego)
  assert all(lead is not None and lead[0] == 40 for lead in leads[10:]), [(i, lead) for i, lead in enumerate(leads) if not lead or lead[0] != 40]


def test_stopped_car_revealed_by_cutting_out_lead_is_adopted_at_once():
  frames = []; d_b = 60.0
  for i in range(80):
    t = i * DT
    y_a = 0.0 if t < 2.0 else min(3.5, 3.5 * (t - 2.0))
    vis = model(30.0, V_EGO, lead_y=y_a) if y_a < 1.5 else model(d_b, 0.0, probability=0.9)
    frames.append(([moving(10, 30.0, V_EGO, y=y_a), stopped(20, d_b)], vis))
    d_b -= V_EGO * DT
  leads = run(frames)
  switch = next(i for i, (_, md) in enumerate(frames) if md.leadsV3[0].v[0] == 0.0)
  assert leads[switch][0] == 20, leads[switch - 1:switch + 3]  # same frame the vision moves to it


def test_jam_tail_from_no_lead_is_adopted_at_once_outside_a_tunnel():
  frames = []; d = 90.0
  for i in range(60):
    if i < 20:
      frames.append(([], model(None, 0.0)))
    else:
      frames.append(([stopped(30, d)], model(d, 0.0, probability=min(0.9, 0.3 + 1.2 * (i - 20) * DT)))); d -= V_EGO * DT
  leads = run(frames)
  first = next(i for i, lead in enumerate(leads) if lead and lead[0] == 30)
  assert first <= 23, leads[18:30]
  # in a tunnel the same car waits for the stationary matcher but is still taken within 0.5 s
  tunnel = run(frames, in_tunnel=True)
  first_tunnel = next(i for i, lead in enumerate(tunnel) if lead and lead[0] == 30)
  assert first < first_tunnel <= 30, tunnel[18:32]


def test_moving_cut_in_is_untouched():
  frames = []; d_f = 25.0
  for i in range(80):
    t = i * DT
    y_f = 3.5 if t < 2.0 else max(0.0, 3.5 - 3.5 * (t - 2.0))
    vis = model(45.0, V_EGO) if y_f > 1.5 else model(d_f, 15.0, probability=0.9, lead_y=y_f)
    frames.append(([moving(50, 45.0, V_EGO), moving(60, d_f, 15.0, y=y_f)], vis))
    if t >= 2.0:
      d_f += (15.0 - V_EGO) * DT
  leads = run(frames)
  switch = next(i for i, (_, md) in enumerate(frames) if md.leadsV3[0].v[0] == 15.0)
  assert leads[switch][0] == 60, leads[switch - 1:switch + 3]
