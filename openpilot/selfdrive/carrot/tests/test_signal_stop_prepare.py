import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare, PrepareAccel, housing_range, road_corridor
from openpilot.selfdrive.carrot.signal_stop_prepare_runtime import publish_prepare_observation
from tools.signal_analysis.replay_signal_assist import planner_class, REPO
from tools.signal_analysis.test_signal_assist_planner import inputs


def observation(t, state='red'):
  return dict(version=1, stream='road', size=[1344, 760], timestamp=t, frame_id=round(t*1000), session='test', tracks=[
    dict(id=1, box=[667, 260, 707, 270], age=0., state=state, observations=10,
         evidence=dict(raw=state, source='detected', quality=.7, red_score=.8, green_score=0.))])


def context():
  return dict(enabled=True, valid=True, drive=True, speed=15., gas=False, brake=False,
              lead=False, turning=False, model_stopping=False, entry_allowed=True, geometry=lambda box: (30., 70.))


def confirmed():
  p = SignalStopPrepare()
  for t in np.arange(0, .701, .1):
    d = p.update(float(t), observation(float(t)), **context())
  assert d.active
  return p


@pytest.mark.parametrize('field,value', [('lead', True), ('gas', True), ('brake', True), ('valid', False),
  ('enabled', False), ('drive', False), ('turning', True), ('model_stopping', True), ('entry_allowed', False),
  ('speed', 0.), ('speed', 82/3.6), ('speed', float('nan'))])
def test_context_cancels(field, value):
  p = confirmed(); c = context(); c[field] = value
  assert not p.update(.8, observation(.8), **c).active


@pytest.mark.parametrize('state', ['green', 'unknown'])
def test_loss_or_change_cancels_without_hold(state):
  assert not confirmed().update(.8, observation(.8, state), **context()).active


def test_stale_duplicate_reorder_session_and_audio():
  p=SignalStopPrepare(); sounds=[]
  for t in np.arange(0, 1.51, .05):
    d=p.update(float(t), observation(float(t)), **context()); sounds.append(d.entered)
  assert sum(sounds)==1
  assert not p.update(1.8, observation(1.5), **context()).active
  p=SignalStopPrepare()
  for t in [.0,.05,.1,.15]:
    assert not p.update(t, observation(0), **context()).active
  p=confirmed(); o=observation(.8);o['session']='new'
  assert not p.update(.8,o,**context()).active
  assert not confirmed().update(.8, observation(.6), **context()).active


def test_gas_cooldown_and_sound_cooldown():
  p=confirmed();c=context();c['gas']=True
  assert not p.update(.8,observation(.8),**c).active
  assert not p.update(1.,observation(1.),**context()).active
  p=confirmed();p.update(.8,observation(.8,'unknown'),**context())
  for t in np.arange(.9,1.71,.1):
    d=p.update(float(t),observation(float(t)),**context())
    assert not d.entered
  assert d.active


@pytest.mark.parametrize('source', ['night_lamp_core', 'day_color_core', 'causal_template'])
def test_inferred_housing_not_metric_distance(source):
  p=confirmed();o=observation(.8);o['tracks'][0]['evidence']['source']=source
  assert not p.update(.8,o,**context()).active


def test_geometry_and_conflict_fail_closed():
  p=confirmed();c=context();c['geometry']=lambda box: None
  assert not p.update(.8,observation(.8),**c).active
  p=confirmed();o=observation(.8);o['tracks'].append(dict(o['tracks'][0],id=2))
  assert not p.update(.8,o,**context()).active
  for malformed in (None, {}, {'timestamp':float('nan')}, {'timestamp':.8,'frame_id':float('inf')}):
    assert not SignalStopPrepare().update(.8,malformed,**context()).active


def group_observation(t, ids=(1, 2, 3, 4), green_ids=()):
  o = observation(t)
  o['tracks'] = [dict(observation(t, 'green' if tid in green_ids else 'red')['tracks'][0],
                      id=tid, box=[487+60*tid, 260, 527+60*tid, 270]) for tid in ids]
  return o


@pytest.mark.parametrize('size', [1, 2, 3, 4])
def test_multiple_reds_need_time_not_votes(size):
  p = SignalStopPrepare()
  ids = tuple(range(1, size+1))
  sounds = []
  for i in range(11):
    t = i*.1
    o = group_observation(t, ids if i % 2 else ids[::-1])
    d = p.update(t, o, **context())
    assert d.active == (i >= 6)
    assert d.track_ids == ids and d.distance_range == (30., 70.)
    sounds.append(d.entered)
  assert sum(sounds) == 1


def test_group_membership_retains_only_each_members_own_confirmation():
  p = SignalStopPrepare()
  for i in range(8):
    t = i*.1
    d = p.update(t, group_observation(t, (1, 2)), **context())
  assert d.active
  # One independently confirmed housing remains; no interruption or new sound.
  d = p.update(.8, group_observation(.8, (2, 3)), **context())
  assert d.active and not d.entered and d.track_id == 2
  # Newly joined 3 cannot inherit 2's history when 2 disappears.
  assert not p.update(.9, group_observation(.9, (3,)), **context()).active
  for i in range(10, 15):
    t = i*.1
    d = p.update(t, group_observation(t, (3,)), **context())
  assert d.active and d.track_id == 3
  # A lost member's previous confirmation is discarded, even if its ID returns.
  assert not p.update(1.5, group_observation(1.5, (1,)), **context()).active


@pytest.mark.parametrize('kind', ['duplicate_id', 'overlap', 'row', 'scale', 'distance', 'five'])
def test_incompatible_groups_cancel(kind):
  p = confirmed()
  o = group_observation(.8, (1, 2))
  c = context()
  if kind == 'duplicate_id':
    o['tracks'][1]['id'] = 1
  if kind == 'overlap':
    o['tracks'][1]['box'] = o['tracks'][0]['box']
  if kind == 'row':
    o['tracks'][1]['box'] = [607, 310, 647, 320]
  if kind == 'scale':
    o['tracks'][1]['box'] = [607, 250, 697, 272]
  if kind == 'distance':
    c['geometry'] = lambda b: (30., 50.) if b[0] < 600 else (60., 90.)
  if kind == 'five':
    o = group_observation(.8, (1, 2, 3, 4, 5))
  assert not p.update(.8, o, **c).active


def test_green_vetoes_red_majority_without_issuing_go():
  p = confirmed()
  d = p.update(.8, group_observation(.8, green_ids=(4,)), **context())
  assert not d.active and d.reason == 'mixed_colors'
  # Removing green requires fresh red confirmation again.
  assert not p.update(.9, group_observation(.9, (1, 2, 3)), **context()).active


def test_green_outside_geometry_is_not_part_of_search_corridor():
  p = SignalStopPrepare()
  c = context()
  c['geometry'] = lambda b: (30., 70.) if b[0] < 700 else None
  for i in range(8):
    t = i*.1
    d = p.update(t, group_observation(t, green_ids=(4,)), **c)
  assert d.active and d.track_ids == (1, 2, 3)


def test_group_repeats_gaps_session_and_alternating_ids_cannot_accumulate():
  p = SignalStopPrepare()
  for t in (0., .05, .1, .15):
    assert not p.update(t, group_observation(0.), **context()).active
  p = SignalStopPrepare()
  for i in range(20):
    t = i*.1
    assert not p.update(t, group_observation(t, (1, 2) if i % 2 else (3, 4)), **context()).active
  p = SignalStopPrepare()
  for i in range(8):
    t = i*.1
    d = p.update(t, group_observation(t), **context())
  assert d.active
  assert not p.update(1.1, group_observation(1.1), **context()).active
  for i in range(12, 19):
    t = i*.1
    d = p.update(t, group_observation(t), **context())
  assert d.active
  o = group_observation(1.9)
  o['session'] = 'new'
  assert not p.update(1.9, o, **context()).active


def geometry():
  xs=np.linspace(0,200,33)
  curve=lambda y: NS(x=xs,y=np.full(33,y),z=np.zeros(33))
  return dict(focal=1141.5,rpy=[0.,0.,0.],edges=[curve(-4),curve(4)],edge_stds=[.2,.2],
              lane_lines=[curve(-5.4),curve(-1.8),curve(1.8),curve(5.4)],lane_probs=[.9]*4,lane_stds=[.2]*4)


def test_four_housings_with_actual_road_projection():
  p = SignalStopPrepare()
  c = context()
  c['geometry'] = lambda b: housing_range(b, **geometry())
  for i in range(8):
    t = i*.1
    o = group_observation(t)
    for track, x in zip(o['tracks'], (590, 635, 680, 725), strict=True):
      track['box'] = [x, 260, x+40, 270]
    d = p.update(t, o, **c)
  assert d.active and d.track_ids == (1, 2, 3, 4)
  assert 25 <= d.distance_range[0] < d.distance_range[1] <= 140


def test_housing_height_road_edges_lanes_and_uncertainty():
  box=[667,260,707,270];g=geometry()
  result=housing_range(box,**g)
  assert result is not None and 25<=result[0]<result[1]<=140
  assert housing_range([667,400,707,410],**g) is None  # below horizon / taillights
  assert housing_range([1100,260,1140,270],**g) is None
  g['edge_stds']=[2.,.2]
  assert housing_range(box,**g) is not None  # lanes replace uncertain road edges
  g['lane_probs']=[.1]*4
  assert housing_range(box,**g) is None
  g=geometry()
  for curve in (*g['edges'],*g['lane_lines']):curve.x=[0.]*33
  assert housing_range(box,**g) is None


def test_corridor_source_conflict_and_no_extrapolation():
  g=geometry();args={k:g[k] for k in ('edges','edge_stds','lane_lines','lane_probs','lane_stds')}
  assert road_corridor(np.array([50.]),**args)[-1]=='lanes+edges'
  args['edge_stds']=[2.,2.]
  assert road_corridor(np.array([50.]),**args)[-1]=='lanes'
  args['lane_probs']=[.1]*4
  assert road_corridor(np.array([50.]),**args) is None
  args['edge_stds']=[.2,.2]
  assert road_corridor(np.array([50.]),**args)[-1]=='edges'
  assert not road_corridor(np.array([250.]),**args)[3][0]
  args['lane_probs']=[.9]*4
  args['lane_lines'][1].y=np.full(33,8.)
  args['lane_lines'][2].y=np.full(33,11.6)
  assert not road_corridor(np.array([50.]),**args)[3][0]


def test_road_z_and_camera_rotation_define_height_without_position():
  g=geometry();box=[667,310,707,320]
  base=housing_range(box,**g)
  assert base is not None
  for curve in (*g['edges'],*g['lane_lines']):curve.z=np.full(33,-5.)
  assert housing_range(box,**g) is None
  g=geometry();g['rpy']=[0.,.3,0.]
  assert housing_range(box,**g) is None


def test_lane_margin_accepts_offset_housing_and_keeps_road_limit():
  g=geometry();box=[750,260,790,270]
  assert housing_range(box,**g,lane_margin=0.) is None
  assert housing_range(box,**g) is not None
  # Road edges still bound the expanded lane region, including their clearance.
  args={k:g[k] for k in ('edges','edge_stds','lane_lines','lane_probs','lane_stds')}
  left,right,_,valid,_=road_corridor(np.array([50.]),**args)
  assert valid[0] and left[0]==pytest.approx(-3.7) and right[0]==pytest.approx(3.7)
  args['edge_stds']=[2.,2.]
  left,right,_,valid,_=road_corridor(np.array([50.]),**args)
  assert valid[0] and left[0]==pytest.approx(-5.3) and right[0]==pytest.approx(5.3)
  g['lane_probs']=[.1]*4
  assert housing_range(box,**g) is not None


@pytest.mark.parametrize('margin', [-1., 5.1, float('nan'), float('inf')])
def test_invalid_lane_margin_rejected(margin):
  assert housing_range([667,260,707,270],**geometry(),lane_margin=margin) is None


def test_margin_does_not_reconcile_conflicting_source_geometry():
  g=geometry()
  # Expansion overlaps the road, but the ORIGINAL ego lane is entirely outside.
  g['lane_lines'][1].y=np.full(33,4.2)
  g['lane_lines'][2].y=np.full(33,7.8)
  args={k:g[k] for k in ('edges','edge_stds','lane_lines','lane_probs','lane_stds')}
  left,right,_,valid,_=road_corridor(np.array([50.]),**args)
  assert left[0]<right[0] and not valid[0]
  assert housing_range([722,260,762,270],**g) is None


def test_lane_margin_does_not_make_taillights_or_far_side_targets_eligible():
  g=geometry();g['edge_stds']=[2.,2.]
  assert housing_range([667,400,707,410],**g) is None
  assert housing_range([1100,260,1140,270],**g) is None


def test_acceleration_ramp_cancellation_and_stronger_brake():
  p=PrepareAccel();out=[]
  for i in range(40):out.append(p.apply(1.,0.,.05,True))
  assert out[0]==pytest.approx(-.0125) and out[-1]==pytest.approx(-.5)
  assert min(out)>=-.5
  assert p.apply(-2.,-.5,.05,True)==-2.
  assert p.apply(.8,-2.,.05,False)==.8 and p.cap is None
  assert p.apply(1.,.5,.05,True)==pytest.approx(.4875)
  fast=PrepareAccel()
  for _ in range(80):value=fast.apply(1.,0.,.025,True)
  assert value==pytest.approx(-.5)


def test_planner_prepare_handover_cancel_and_one_sound():
  cls,_=planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  p=cls(stop_prepare=SignalStopPrepare());sm=inputs(speed=15.,distance=160.);sounds=[]
  for i in range(25):
    t=i*.05
    p.update(sm,80.,'acc',prepare_observation=observation(t),prepare_context=dict(now=t,enabled=True,valid=True,drive=True,turning=False,geometry=lambda b:(30.,70.)))
    sounds+=p.events.events
  assert p.xState.name=='e2eStopPrepare' and p.stop_dist==1000.
  assert sounds.count('trafficSignChanged')==1
  p.update(sm,80.,'acc',prepare_observation=observation(1.3,'unknown'),prepare_context=dict(now=1.3,enabled=True,valid=True,drive=True,turning=False,geometry=lambda b:(30.,70.)))
  assert p.xState.name=='e2eCruise' and not p.prepare_decision.active
  # Original model recognizes stopping; handover must win over preparation.
  sm['modelV2'].position.x=[25*i/32 for i in range(33)]
  sm['modelV2'].velocity.x=[15.]+[0.]*32
  for i in range(20):
    t=1.4+i*.05
    p.update(sm,80.,'acc',prepare_observation=observation(t),prepare_context=dict(now=t,enabled=True,valid=True,drive=True,turning=False,geometry=lambda b:(30.,70.)))
  assert p.xState.name=='e2eStop' and not p.prepare_decision.active and p.stop_dist<1000


def test_dedicated_publisher_off_by_default(tmp_path):
  path=tmp_path/'obs';o=observation(1.);record=dict(o,timestamp_eof=1_000_000_000)
  publish_prepare_observation(record,tmp_path,path)
  assert not path.exists()
  (tmp_path/'stop_prepare_enabled').write_text('1')
  publish_prepare_observation(record,tmp_path,path)
  assert json.loads(path.read_text())['tracks'][0]['evidence']['source']=='detected'


@pytest.mark.parametrize('reset,lead,gas,brake',[(False,False,False,False),(True,False,False,False),
  (False,True,False,False),(False,False,True,False),(False,False,False,True)])
def test_final_output_cap_after_coasting(reset,lead,gas,brake):
  path=REPO/'openpilot/selfdrive/controls/lib/longitudinal_planner.py'
  tree=ast.parse(path.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LongitudinalPlanner')
  method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='apply_signal_stop_prepare')
  import time
  ns={'time':time}
  exec(compile(ast.Module([method],type_ignores=[]),str(path),'exec'),ns)
  p=NS(output_a_target=1.,output_v_target_now=20.,output_j_target_now=1.,output_should_stop=False,mpc=NS(mode='acc'),dt=.05,a_desired=1.,coasting_target=70.)
  p._signal_prepare_accel = PrepareAccel()
  p._signal_prepare_time = p._signal_prepare_previous = None
  sm={'carState':NS(gasPressed=gas,brakePressed=brake,aEgo=0.,vEgo=15.,softHoldActive=0),'radarState':NS(leadOne=NS(status=lead,dRel=10.),leadTwo=NS(status=False))}
  ns['apply_signal_stop_prepare'](p,sm,NS(prepare_decision=NS(active=True),prepare_control_active=True),reset)
  if not (reset or lead or gas or brake):
    assert p.output_a_target==pytest.approx(-.0125) and p.coasting_target==0
    assert p.output_j_target_now==pytest.approx(-.25)
  else:assert p.output_a_target==1.
  assert not p.output_should_stop


@pytest.mark.parametrize('yaw,age,calibrated,expected', [(0.,0.,True,True), (4.,0.,True,False),
  (0.,.3,True,False),(0.,0.,False,False)])
def test_runtime_pose_camera_and_calibration(monkeypatch,yaw,age,calibrated,expected):
  from openpilot.cereal import log
  from openpilot.selfdrive.carrot.signal_stop_prepare_runtime import SignalPrepareRuntime
  from openpilot.selfdrive.carrot.signal_assist_runtime import SignalAssistRuntime
  class SM(dict):
    def update(self, timeout): pass
  extra=SM(liveCalibration=NS(calStatus='calibrated' if calibrated else 'uncalibrated',rpyCalib=[0.,0.,0.]),
           roadCameraState=NS(sensor='os04c10'),deviceState=NS(deviceType='mici'))
  extra.seen=extra.valid=extra.alive=dict.fromkeys(extra,True);extra.recv_time=dict.fromkeys(extra,1.)
  pose=log.LivePose.new_message(timestamp=int((1.-age)*1e9),inputsOK=True,posenetOK=True,sensorsOK=True,
       angularVelocityDevice=dict(valid=True,x=0.,y=0.,z=float(np.deg2rad(yaw)),xStd=.001,yStd=.001,zStd=.001),orientationNED=dict(valid=True))
  g=geometry()
  curve=lambda c:{k:getattr(c,k).tolist() for k in ('x','y','z')}
  model=log.ModelDataV2.new_message(roadEdges=[curve(c) for c in g['edges']],roadEdgeStds=[.2,.2],
      laneLines=[curve(c) for c in g['lane_lines']],laneLineProbs=[.9]*4,laneLineStds=[.2]*4)
  assert len(model.position.x)==0  # road/lane geometry does not require a plan
  sm=SM(livePose=pose.as_reader(),modelV2=model.as_reader())
  sm.seen=sm.valid=sm.alive=dict.fromkeys(sm,True);sm.recv_time=dict.fromkeys(sm,1.)
  monkeypatch.setattr(SignalAssistRuntime,'read',lambda *a:(observation(1.),dict(now=1.,enabled=True,valid=True,drive=True)))
  runtime=SignalPrepareRuntime.__new__(SignalPrepareRuntime);runtime.enabled=True;runtime.extra=extra
  runtime.mode, runtime.mode_since = None, -float('inf')
  runtime.prepare=SignalStopPrepare()
  _,ctx=runtime.read(sm,1.)
  assert (ctx['valid'] and not ctx['turning'])==expected
  if expected:
    distance = ctx['geometry']([667,260,707,270])
    assert distance is not None
    assert ctx['alignment']([667,260,707,270], distance) is not None
