"""Observation-only candidates must never enter the lamp/desire output path."""
import ast
import copy
import math
from pathlib import Path

import pytest

from openpilot.selfdrive.controls.lib.desire_lib.boundary_observer import BoundaryObserver
from test_blinker_boundary import scene


def run(direction=1, mode='crossing', emit=None):
  records=[]
  observer=BoundaryObserver(emit or (lambda name, **data: records.append((name, data))))
  cs, md=scene(direction)
  before=copy.deepcopy(vars(cs))
  for i in range(100):
    now=10+i*.05
    progress=max(0,i-20)*.04
    y=1.8-progress
    if mode=='curve':
      y=1.8
    if mode=='correction':
      y=1.8-min(progress, .2) if i<35 else 1.6+min((i-35)*.04,.2)
    if mode=='retreat':
      y=1.8-progress if i<40 else 1.+min((i-40)*.04,.8)
    if mode in ('release','countersteer') and i>=29:
      cs.steeringPressed=mode=='countersteer'
      cs.steeringTorque=(-300 if direction==1 else 300) if mode=='countersteer' else 0
    if mode=='no_driver':
      cs.steeringPressed=False
    md.frameId=i
    md.laneLines[1].y=[-y if direction==1 else -(3.6-y)]
    md.laneLines[2].y=[3.6-y if direction==1 else y]
    if mode=='widening':
      md.laneLines[2].y=[1.8+progress]
      md.laneLines[1].y=[-1.8-progress]
    if mode=='jump' and i>=26:
      md.laneLines[1].y=[-.2]
      md.laneLines[2].y=[3.4]
    if mode=='unknown':
      md.laneLineProbs=[0,0,0,0]
    cs_snapshot,md_snapshot=copy.deepcopy(vars(cs)),copy.deepcopy(vars(md))
    assert observer.observe(now,cs,md,enabled=True,valid=True,busy=False) is None
    assert vars(cs)==cs_snapshot and vars(md)==md_snapshot
  # Flush pending diagnostics only; no request can be returned.
  observer.observe(16,cs,md,enabled=False,valid=True,busy=False)
  events=[e for _,r in records for e in r['events']]
  return observer,records,events,before


@pytest.mark.parametrize('direction',[1,2])
def test_continuous_driver_crossing_is_observed_without_actuation(direction):
  observer,records,events,_=run(direction)
  candidates=[e for e in events if e['reason']=='would_signal']
  assert len(candidates)==1 and candidates[0]['direction']==direction
  assert any(e['reason']=='centre_crossed' for e in events)
  assert all(r['observation_only'] and r['half_width_proxy_m']==.9 for _,r in records)
  assert not hasattr(observer,'request_direction')
  assert len(records)<=7


@pytest.mark.parametrize('mode',['release','countersteer'])
@pytest.mark.parametrize('direction',[1,2])
def test_recent_driver_intent_survives_release_while_vehicle_still_approaches(mode,direction):
  _,_,events,_=run(direction,mode)
  assert any(e['reason']=='would_signal' for e in events)


@pytest.mark.parametrize('mode',['curve','correction','no_driver','widening','jump','unknown'])
def test_corrections_and_uncertain_geometry_do_not_predict_signal(mode):
  _,_,events,_=run(mode=mode)
  assert not any(e['reason']=='would_signal' for e in events)


def test_return_after_prediction_is_logged_without_guessing_driver_motive():
  _,_,events,_=run(mode='retreat')
  assert any(e['reason']=='would_signal' for e in events)
  assert any(e['reason']=='retreated' and e['predicted'] for e in events)
  assert not any('avoid' in e['reason'] or 'safe' in e['reason'] for e in events)


@pytest.mark.parametrize('fault',['disabled','invalid','busy','lamp','lever','park','slow','trailer','steer_fault','nan'])
def test_protected_or_invalid_conditions_never_predict(fault):
  cs,md=scene()
  records=[]
  observer=BoundaryObserver(lambda _,**kw:records.append(kw))
  if fault=='lamp':
    cs.leftBlinker=True
  if fault=='lever':
    cs.blinkerLever=1
  if fault=='park':
    cs.gearShifter='park'
  if fault=='slow':
    cs.vEgo=1
  if fault=='trailer':
    cs.trailerConnected=True
  if fault=='steer_fault':
    cs.steerFaultTemporary=True
  if fault=='nan':
    cs.steeringTorque=math.nan
  for i in range(100):
    md.frameId=i
    md.laneLines[1].y=[-1.8+i*.02]
    md.laneLines[2].y=[1.8+i*.02]
    observer.observe(10+i*.05,cs,md,enabled=fault!='disabled',valid=fault!='invalid',busy=fault=='busy')
  assert not any(e['reason']=='would_signal' for r in records for e in r['events'])


def test_logging_failure_does_not_propagate():
  def fail(*a,**kw):
    raise OSError('logging unavailable')
  run(emit=fail)


def test_bounded_event_buffer_and_frame_gap_reset():
  cs,md=scene()
  observer=BoundaryObserver(lambda *a,**kw:None)
  for i in range(100):
    observer._event(10,'candidate',candidate=i)
  assert len(observer.events)==8 and observer.dropped_events==92
  observer.observe(10,cs,md,enabled=True,valid=True,busy=False)
  observer.observe(11,cs,md,enabled=True,valid=True,busy=False)
  assert observer.previous is None and observer.candidate is None


def test_actual_modeld_observation_call_does_not_assign_control_fields():
  path=Path(__file__).resolve().parents[2]/'openpilot/selfdrive/modeld/modeld.py'
  tree=ast.parse(path.read_text(encoding='utf-8'))
  calls=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=='boundary_observer is not None']
  assert len(calls)==1
  node=calls[0]
  assert len(node.body)==1 and isinstance(node.body[0],ast.Expr)
  assert ast.unparse(node.body[0].value.func)=='boundary_observer.observe'
  # This caller cannot assign predictions into manualLaneIndicator/desire.
  assert not any(isinstance(n,(ast.Assign,ast.AnnAssign,ast.AugAssign)) for n in ast.walk(node))


def test_actual_modeld_call_with_schema_preserves_published_metadata():
  pytest.importorskip('capnp')
  from types import SimpleNamespace as N
  from openpilot.cereal import log
  path=Path(__file__).resolve().parents[2]/'openpilot/selfdrive/modeld/modeld.py'
  tree=ast.parse(path.read_text(encoding='utf-8'))
  node=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=='boundary_observer is not None')
  msg=log.Event.new_message(valid=True)
  md=msg.init('modelV2')
  md.meta.manualLaneIndicator='left'
  md.meta.manualLaneIndicatorMonoTime=123
  md.meta.desire='laneChangeRight'
  cs,_=scene()
  class SM(dict):
    pass
  sm=SM(carState=cs)
  sm.valid={'carState':True}
  sm.alive={'carState':True}
  sm.recv_time={'carState':10.}
  calls=[]
  observer=N(observe=lambda *args,**kw:calls.append((args,kw)))
  dh=N(boundary_indicator_enabled=True,lane_change_state=log.LaneChangeState.off,
       boundary_indicator=N(request_direction=1))
  before=msg.to_bytes()
  msg.clear_write_flag()
  context={'boundary_observer': observer,'indicator_now': 10.1,'sm': sm,'modelv2_send': msg,'DH': dh,'log': log}
  exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),context)
  assert len(calls)==1 and calls[0][1]['valid'] and not calls[0][1]['busy']
  assert calls[0][1]['legacy_direction']==1
  assert msg.to_bytes()==before
