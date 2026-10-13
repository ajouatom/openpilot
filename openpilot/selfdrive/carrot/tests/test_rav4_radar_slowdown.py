"""RAV4 전방 레이더와 공통 감속 계산의 연결 회귀 검사.

합성 ARS CAN을 실제 수신기·MyTrack·DPath에 전달한다. 커브·감속 함수는
원본을 사용하고, 미빌드 IPC/네이티브 MPC를 요구하는 두 메서드는 AST로
그대로 읽어 MPC 입력 직전까지 실행한다. 실제 차량 제동이나 MPC 계산,
Toyota 코너 센서 지원을 검증하는 검사가 아니다.

미빌드 환경은 tools.ars510_validation.offline_io.prepare()를 명시적으로
먼저 실행하고 이 tests 폴더를 --confcutdir로 지정한다. 이 모듈 자체는
Params·통신 모듈을 대체하거나 차량 설정을 변경하지 않는다.
"""

import ast
from functools import lru_cache
from pathlib import Path
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiExtFlags
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from opendbc.car.toyota.ars510.objects import encode_slot
from opendbc.car.toyota.ars510_carrot import Ars510RadarInterface
from opendbc.car.toyota.tests.test_ars510_carrot import make_record, packets_for
from opendbc.car.toyota.values import CAR, ToyotaFlags
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.carrot.carrot_man_input import get_carrot_man
from openpilot.selfdrive.carrot.curve_speed import curve_speed
from openpilot.selfdrive.carrot.radar_motion.controller import DPathRadarController
from openpilot.selfdrive.controls.lib.cutin_predecel import apply_cutin_predecel_accel_limit, get_cutin_predecel_accel_limit
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.turn_accel import get_future_curvature, limit_accel_in_turns
from openpilot.selfdrive.modeld.constants import ModelConstants


ROOT = Path(__file__).resolve().parents[4]
TIMES = np.asarray(ModelConstants.T_IDXS)


def source_function(relative, name, namespace, cls=None):
  path = ROOT / relative
  tree = ast.parse(path.read_text(encoding='utf-8'))
  body = tree.body if cls is None else next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls).body
  fn = next(n for n in body if isinstance(n, ast.FunctionDef) and n.name == name)
  exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), namespace)
  return namespace[name]


def source_literal(relative, name):
  tree = ast.parse((ROOT / relative).read_text(encoding='utf-8'))
  node = next(n for n in tree.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in n.targets))
  return ast.literal_eval(node.value)


@lru_cache(maxsize=1)
def functions():
  flags = int(HyundaiExtFlags.CORNER_RADAR_OBJECTS_235 | HyundaiExtFlags.CORNER_RADAR_OBJECTS_180 | HyundaiExtFlags.CORNER_RADAR_OBJECTS_430)
  gate = source_function('openpilot/selfdrive/carrot/radar/radard_dpath.py', 'corner_radar_enabled',
                         {'CORNER_RADAR_FLAGS': flags, 'car': NS(CarParams=object)})
  nav = source_function('openpilot/selfdrive/carrot/carrot_functions.py', '_update_carrot_man',
                        {'get_carrot_man': get_carrot_man}, 'CarrotPlanner')
  planner_path = 'openpilot/selfdrive/controls/lib/longitudinal_planner.py'
  namespace = {'np': np, 'CV': CV, 'V_CRUISE_MAX': V_CRUISE_MAX, 'V_CRUISE_UNSET': V_CRUISE_UNSET,
               'LongCtrlState': LongCtrlState, 'ACCEL_MAX': ACCEL_MAX, 'ACCEL_MIN': ACCEL_MIN,
               'A_CRUISE_MIN': source_literal(planner_path, 'A_CRUISE_MIN'),
               'get_future_curvature': get_future_curvature, 'limit_accel_in_turns': limit_accel_in_turns,
               'get_cutin_predecel_accel_limit': get_cutin_predecel_accel_limit,
               'apply_cutin_predecel_accel_limit': apply_cutin_predecel_accel_limit,
               'RESET_DECEL_RAMP_TIME': source_literal(planner_path, 'RESET_DECEL_RAMP_TIME'),
               'N': source_literal('openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py', 'N')}
  update = source_function(planner_path, 'update', namespace, 'LongitudinalPlanner')
  return gate, nav, update


def model(curvature=0., speed=15.):
  return NS(position=NS(x=TIMES*speed, y=np.zeros_like(TIMES), z=np.zeros_like(TIMES)),
            velocity=NS(x=np.full_like(TIMES, speed)), orientationRate=NS(z=np.full_like(TIMES, curvature*speed)),
            leadsV3=(), laneLineProbs=(1., 1., 1., 1.))


class NavInput(dict):
  def __init__(self, cap, stale=False):
    super().__init__(carrotMan=NS(trafficState=0, atcType='none', activeCarrot=0, xDistToTurn=0, desiredSpeed=cap),
                     carState=NS(leftBlinker=False))
    self.seen = {'carrotMan': True}
    self.alive = {'carrotMan': True}
    self.valid = {'carrotMan': True}
    self.recv_time = {'carrotMan': time.monotonic() - (2. if stale else 0.)}


def consume_cap(cap, stale=False):
  carrot = NS(signal_detection_suspended=False, trafficState_carrot=0)
  return functions()[1](carrot, NavInput(cap, stale), 80., 80.)[0]


def planner_input(brand, cap=80., curvature=0., risk=None, gas=False, inactive=False):
  captured = {}

  class StopBeforeSolver(Exception):
    pass

  def capture(*limits):
    captured['accel_limits'] = limits

  def stop(*unused):
    raise StopBeforeSolver

  sm = {'modelV2': model(curvature), 'selfdriveState': NS(experimentalMode=False, enabled=not inactive),
        'carState': NS(vEgo=15., aEgo=0., vCruise=80., vCluRatio=1., standstill=False, gasPressed=gas),
        'carControl': NS(orientationNED=[]), 'radarState': NS(leadCutInRisk=NS(**risk) if risk else None),
        'controlsState': NS(longControlState=LongCtrlState.off if inactive else LongCtrlState.pid,
                            forceDecel=False, desiredCurvature=0., curvature=0.)}
  carrot = NS(update=lambda *args: consume_cap(cap), mode='acc', soft_hold_active=False,
              get_carrot_accel=lambda _: 1.6, leadAccelResponse=5, lane_change_active=False)
  planner = NS(CP=NS(brand=brand, openpilotLongitudinalControl=True),
               mpc=NS(set_accel_limits=capture, set_cur_state=stop), coasting_param_time=0., dt=DT_MDL,
               a_desired=0., reset_decel_timer=0, output_should_stop=False,
               v_desired_filter=NS(update=lambda value: value), parse_model=lambda _: (0., 0., 0., 0., 1.),
               update_lead_tracks=lambda _: (0, 0), vCluRatio=1.)
  with pytest.raises(StopBeforeSolver):
    functions()[2](planner, sm, carrot)
  captured['v_cruise_kph'] = planner.v_cruise_kph
  return captured


@pytest.mark.parametrize('brand', ('toyota', 'hyundai'))
@pytest.mark.parametrize('cap', (35., 40., 250.))
def test_navigation_and_curve_caps_reach_planner_without_brand_exclusion(brand, cap):
  curve_cap = curve_speed(model(.012), 15.).approach_kph
  assert 30. < curve_cap < 80.
  assert planner_input(brand, cap)['v_cruise_kph'] == min(80., cap)
  assert planner_input(brand, curve_cap)['v_cruise_kph'] == curve_cap
  assert planner_input(brand, curvature=.012)['accel_limits'][1] < planner_input(brand)['accel_limits'][1]


@pytest.mark.parametrize('cap,stale', ((0., False), (30., True)))
def test_invalid_navigation_does_not_replace_cruise_speed(cap, stale):
  assert consume_cap(cap, stale) == 80.


@pytest.mark.parametrize('brand', ('toyota', 'hyundai'))
def test_predecel_respects_override_reset_and_risk_disappearance(brand):
  risk = {'status': True, 'radar': True, 'score': .9, 'dRel': 20., 'vRel': -6.5}
  assert planner_input(brand, risk=risk)['accel_limits'][1] == -.15
  assert planner_input(brand, curvature=.012, risk=risk)['accel_limits'][1] == -.15
  for kwargs in ({'risk': risk, 'gas': True}, {'risk': risk, 'inactive': True}, {'risk': None}):
    assert planner_input(brand, **kwargs)['accel_limits'][1] == 1.6


@pytest.mark.parametrize('setting', (0, 1, 2))
@pytest.mark.parametrize('motion', ('inward', 'parallel'))
def test_rav4_front_cutin_reaches_predecel_with_corner_setting(setting, motion):
  cp = structs.CarParams(brand='toyota', carFingerprint=str(CAR.TOYOTA_RAV4_TSS2_2022),
                        flags=int(ToyotaFlags.RADAR_ACC | ToyotaFlags.DISABLE_RADAR), radarUnavailable=False,
                        radarTimeStep=.06, radarDelay=0., openpilotLongitudinalControl=True)
  cp_before = cp.as_reader().as_builder().to_bytes()
  receiver = Ars510RadarInterface(cp)
  controller = DPathRadarController(prefer_corner_radar=functions()[0](cp, setting), enable_radar_tracks=1,
                                   cut_in_sensitivity=3, production_live_tracks=True)
  risks = cutins = lead_twos = 0
  for i in range(60):
    t = i*.06
    distance = 14.-2.*t
    lateral = 3.6-.8*t if motion == 'inward' else 3.6
    slot = encode_slot(long_dist=160+round(distance*16), lat_dist_left=2048+round(lateral/.015),
                       long_vel_over_ground=round(510.5+13./.15), age_cycles=60+i)
    ret = receiver.update_carrot(15., 0., 1.+t+.0315, packets_for(make_record({0: slot}), 1.+t, speed=True))
    assert ret is not None and len(ret.points) == 1
    assert all(point.radarSource == 'frontRadar' for point in ret.points)
    output = controller.update(time_s=1.+t+.0315, v_ego=15., radar_points=ret.points, model=model())
    cutins += len(output.leads_cutin)
    lead_twos += bool(output.lead_two and output.lead_two.get('status'))
    if output.lead_cutin_risk:
      risks += 1
      assert planner_input('toyota', risk=output.lead_cutin_risk)['accel_limits'][1] < 0.
  if motion == 'inward':
    assert risks > 0 and cutins > 0 and lead_twos > 0
  else:
    assert (risks, cutins, lead_twos) == (0, 0, 0)
  assert controller.motion_sensor == 'front'
  assert controller.update(time_s=10., v_ego=15., radar_points=(), model=model()).lead_cutin_risk is None
  assert cp.as_reader().as_builder().to_bytes() == cp_before


@pytest.mark.parametrize('setting', (0, 1, 2))
def test_hyundai_corner_hardware_gate_is_preserved(setting):
  gate = functions()[0]
  flag = int(HyundaiExtFlags.CORNER_RADAR_OBJECTS_235)
  assert gate(NS(brand='hyundai', extFlags=flag), setting) == (setting > 0)
  assert not gate(NS(brand='hyundai', extFlags=0), setting)
  assert not gate(NS(brand='toyota', extFlags=0), setting)
