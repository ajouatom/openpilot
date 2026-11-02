"""Signal-only observations, turn distance and explicit driver cancellation."""
from types import SimpleNamespace as N
import math

import pytest

from blink_observation import ObservationSignals, LowSpeedOwnership, wire_hold
from blink_turn_journey import Travel, TurnJourney
from openpilot.selfdrive.controls.lib.desire_lib.indicator_observer import ModelIndicator, Movement, path_direction
from openpilot.selfdrive.controls.lib.desire_lib.boundary_indicator import BoundaryIndicator


def scene():
  xs = [float(i)*5 for i in range(13)]
  cs = N(canValid=True, gearShifter='drive', trailerConnected=False, steerFaultTemporary=False,
    steerFaultPermanent=False, blinkerLever=0, vEgo=15., steeringPressed=False, steeringTorque=0.,
    steeringAngleDeg=-5., leftBlinker=False, rightBlinker=False, leftLaneLine=10, laneLineInfoMonoTime=int(10e9))
  md = N(frameId=0, laneLines=[N(x=xs,y=[y]*13) for y in (-5.4,-1.8,1.8,5.4)],
    roadEdges=[N(y=[-8.]*13),N(y=[8.]*13)], laneLineProbs=[.9,.9,.10,.9],
    position=N(t=[0.,1.,1.5,2.,2.5,3.,4.], x=[0.,10.,15.,20.,25.,30.,40.],y=[0.,.1,.7,1.5,2.,2.4,3.]),
    action=N(desiredCurvature=.001), meta=N(desireState=[1.,0.,0.,0.,0.,0.,0.,0.]))
  return cs,md


def run(detector, cs, md, **kwargs):
  isolation=BoundaryIndicator()
  args=dict(valid=True,lateral_active=True,desire_enabled=False,path_enabled=True,busy=False,isolation=isolation,command_curvature=md.action.desiredCurvature)
  args.update(kwargs)
  for i in range(10):
    now=10.+i*.05
    md.frameId=i
    cs.laneLineInfoMonoTime=int(now*1e9)
    detector.update(now,cs,md,**args)
  return isolation


def test_path_signal_accepts_ten_percent_and_owns_feedback_before_lamp():
  cs,md=scene();d=ModelIndicator();isolation=run(d,cs,md)
  assert d.active and d.active.direction==2 and d.active.source=='model_path'
  assert isolation.filter_lamp(2)==0
  assert md.meta.desireState[0]==1  # No desire write or synthetic steering request.


def test_actual_schema_path_and_new_metadata_roundtrip():
  from openpilot.cereal import log
  _,md=scene()
  model=log.ModelDataV2.new_message()
  model.position={'t':md.position.t,'x':md.position.x,'y':md.position.y}
  model.laneLines=[{'x':lane.x,'y':lane.y} for lane in md.laneLines]
  model.laneLineProbs=md.laneLineProbs
  model.meta.observedIndicator='right'
  model.meta.observedIndicatorMonoTime=123
  model.meta.observedIndicatorSource='model_path'
  model.meta.indicatorIsolationMask=2
  with log.ModelDataV2.from_bytes(model.to_bytes()) as decoded:
    assert path_direction(decoded,.001)==2
    assert str(decoded.meta.observedIndicator)=='right' and decoded.meta.indicatorIsolationMask==2


@pytest.mark.parametrize('fault',['confidence','nan','curvature','curve','returns','duplicate_x','disabled','inactive','busy','driver','edge'])
def test_path_counterexamples_do_not_request(fault):
  cs,md=scene();args={}
  if fault=='confidence':md.laneLineProbs[2]=.099
  if fault=='nan':md.position.y[3]=math.nan
  if fault=='curvature':md.action.desiredCurvature=-.001
  if fault=='curve':md.laneLines[2].y=[1.8+.004*x*x for x in md.laneLines[2].x]
  if fault=='returns':md.position.y[-2]=0.
  if fault=='duplicate_x':md.laneLines[2].x[2]=md.laneLines[2].x[1]
  if fault=='disabled':args['path_enabled']=False
  if fault=='inactive':args['lateral_active']=False
  if fault=='busy':args['busy']=True
  if fault=='driver':cs.steeringPressed=True
  if fault=='edge':md.roadEdges[1].y=[2.]*13
  d=ModelIndicator();run(d,cs,md,**args)
  assert d.active is None


def test_desire_nonzero_is_not_event_and_turn_does_not_require_adjacent_lane():
  cs,md=scene();md.meta.desireState=[.98,.005,.005,.005,.005,0.,0.,0.]
  d=ModelIndicator();run(d,cs,md,path_enabled=False,desire_enabled=True)
  assert d.active is None
  md.meta.desireState=[.1,0.,.9,0.,0.,0.,0.,0.];md.roadEdges[1].y=[2.]*13
  d=ModelIndicator();run(d,cs,md,path_enabled=False,desire_enabled=True)
  assert d.active and d.active.source=='model_turn'


def test_left_crossing_on_a_right_curve_uses_relative_curvature():
  _,md=scene()
  for line in md.laneLines:
    line.y=[y+.002*x*x for x,y in zip(line.x,line.y,strict=True)]
  md.position.y=[.002*x*x-y for x,y in zip(md.position.x,md.position.y,strict=True)]
  assert path_direction(md,.002)==1  # Still turning right, but less than the road.


def test_confirmation_can_bridge_brief_low_confidence_but_not_a_single_frame():
  cs,md=scene();d=ModelIndicator();isolation=BoundaryIndicator()
  for i,prob in enumerate((.10,.10,.01,.01,.01,.10,.10,.10)):
    md.frameId=i;md.laneLineProbs[2]=prob
    d.update(10+i*.05,cs,md,valid=True,lateral_active=True,desire_enabled=False,path_enabled=True,
             busy=False,isolation=isolation,command_curvature=.001)
  assert d.active
  d=ModelIndicator();isolation=BoundaryIndicator()
  for i in range(10):
    md.frameId=i;md.laneLineProbs[2]=.1 if i==3 else .01
    d.update(10+i*.05,cs,md,valid=True,lateral_active=True,desire_enabled=False,path_enabled=True,
             busy=False,isolation=isolation,command_curvature=.001)
  assert d.active is None


def test_movement_return_cancels_but_countersteer_while_crossing_does_not():
  cs,md=scene();m=Movement(10.,2,'manual',cs,md)
  for i in range(1,12):
    md.laneLines[1].y=[-1.8-i*.1]*13;md.laneLines[2].y=[1.8-i*.1]*13
    cs.steeringAngleDeg=5.  # opposite correction while moving right
    assert m.update(10.+i*.05,cs,md)
  md.laneLines[1].y=[-2.6]*13;md.laneLines[2].y=[1.]*13
  assert not m.update(10.6,cs,md) and m.reason=='returned'


def test_relabel_after_crossing_needs_stable_new_lane_before_completion():
  cs,md=scene();md.laneLines[1].y=[-2.9]*13;md.laneLines[2].y=[.7]*13
  m=Movement(10.,2,'model_path',cs,md)
  md.laneLines[1].y=[-.4]*13;md.laneLines[2].y=[3.2]*13
  assert m.update(10.05,cs,md) and m.crossed
  for i in range(1,14):
    md.laneLines[1].y=[-.4-i*.1]*13;md.laneLines[2].y=[3.2-i*.1]*13
    assert m.update(10.05+i*.05,cs,md)
  for i in range(8):
    assert m.update(10.75+i*.05,cs,md)
  assert not m.update(11.16,cs,md) and m.reason=='completed'


def test_observer_journals_before_start_never_replays_and_maps_existing_wire():
  o=ObservationSignals();saved=[]
  meta=N(observedIndicator='none',observedIndicatorMonoTime=0,indicatorIsolationMask=0)
  assert o.update(10,meta,allowed=True,busy=False,record=saved.append) is None
  meta.observedIndicator='right';meta.observedIndicatorMonoTime=int(10.1e9);meta.indicatorIsolationMask=2
  assert o.update(10.1,meta,allowed=True,busy=False,record=saved.append)==('observation','right')
  assert saved[-1]['direction']==2
  assert o.update(10.2,meta,allowed=True,busy=True,record=saved.append) is None
  assert o.update(10.3,meta,allowed=True,busy=False,record=saved.append) is None
  assert ObservationSignals().update(10.15,meta,allowed=True,busy=False,record=saved.append) is None
  hold=dict(source='observation',id='x',direction='right',time=10.1)
  assert wire_hold(hold)['source']=='lane' and hold['source']=='observation'


def test_low_speed_never_transmits_before_model_ownership_ack():
  o=LowSpeedOwnership();saved=[];meta=N(indicatorJournalMonoTime=0,indicatorIsolationMask=0)
  assert not o.ready(10.,10.,meta,saved.append)
  assert saved and not o.ready(10.5,10.,meta,saved.append) and len(saved)==1
  meta.indicatorJournalMonoTime=int(10e9);meta.indicatorIsolationMask=2
  assert o.ready(10.6,10.,meta,saved.append)


def turn():
  j=TurnJourney()
  step=lambda t,**kw:j.update(t,1,True,kw.pop('cue',11.),kw.pop('direction','left'),kw.pop('atc','turn left'),
    travel=kw.pop('travel',(0,0.)),distance=kw.pop('distance',20.),speed=kw.pop('speed',3.),
    angle=kw.pop('angle',0.),heading=kw.pop('heading',0.),pressed=kw.pop('pressed',False),torque=kw.pop('torque',0.),**kw)
  step(10.,cue=0.)
  assert step(11.)==('turn','left')
  return j,step


def test_turn_distance_freezes_at_stop_and_survives_desire_navigation_advance():
  j,s=turn()
  assert s(12.,travel=(0,10.))
  assert s(100.,travel=(0,10.),speed=0.,atc='none')
  assert s(101.,travel=(0,49.),atc='straight')
  assert s(102.,travel=(0,50.),atc='straight') is None
  assert j.reason=='completed'


def test_opposite_driver_motion_not_zero_crossing_cancels_turn():
  j,s=turn()
  assert s(12.,angle=60.,heading=-.5)
  assert s(12.1,angle=-40.,heading=-.5)  # Outside the new wheel-return band.
  assert s(12.2,angle=-40.,heading=-.3,pressed=True,torque=-100.)
  assert s(12.45,angle=-40.,heading=-.2,pressed=True,torque=-100.) is None
  assert j.reason=='driver_opposite'
  assert s(12.5,cue=12.5) is None


def test_same_direction_next_turn_is_queued_without_extending_repeated_cue():
  j,s=turn()
  assert s(12.,cue=12.,travel=(0,5.),distance=15.)
  assert j.active['end']==50.
  assert s(13.,cue=13.,travel=(0,15.),distance=30.)
  assert j.active['end']==50. and j.pending[1]==45.
  assert s(14.,travel=(0,51.))
  assert j.active['end']==75.


def test_next_opposite_turn_waits_for_current_turn_to_complete():
  j,s=turn()
  assert s(12.,travel=(0,18.),angle=60.,heading=-.6)==('turn','left')
  assert s(13.,cue=13.,direction='right',atc='turn right',travel=(0,20.),distance=30.,angle=45.,heading=-1.)==('turn','left')
  assert s(13.1,travel=(0,21.),angle=0.,heading=-1.)==('turn','right')
  assert s(13.6,travel=(0,23.),angle=0.,heading=-1.)==('turn','right')
  assert j.active['end']==80.


def test_same_direction_model_to_tbt_handover_keeps_request_id():
  from blink_policy import Policy
  p=Policy()
  p.update(10,2,True,[],10.,0.,'drive',False)
  h=p.update(11,2,True,[],10.,0.,'drive',False,('observation','right'))[1]
  ident=h['id']
  assert p.update(11.1,2,True,[],10.,0.,'drive',False,('turn','right'))[1]['id']==ident


@pytest.mark.parametrize('origin', ['observation', 'tbt_display'])
def test_confirmed_display_can_relight_once_but_never_retries_an_unaccepted_start(origin):
  from blink_policy import Policy
  p=Policy();p.start('right',origin,10.)
  ident=p.hold['id']
  assert not p.relight(11.,None,True,False,False)
  assert not p.relight(12.,None,True,False,False)
  assert not p.relight(13.,ident,True,False,False)
  assert p.relight(14.6,ident,True,False,False)
  assert not p.relight(17.,p.hold['id'],True,False,False)


def test_low_speed_tbt_stays_display_only_after_accelerating():
  from blink_tbt import TbtBlinker
  from test_blinker_tbt import scene as tbt_scene, arm
  s=tbt_scene();s['cs'].vEgo=10/3.6;t=TbtBlinker()
  assert arm(t,s)==('tbt_display','right')
  s['cs'].vEgo=20.
  assert t.update(11.1,**s)==('tbt_display','right')
  s['cs'].vEgo=9.9/3.6
  assert t.update(11.15,**s) is None


def test_travel_gap_invalidates_anchor_without_extrapolating_stale_speed():
  t=Travel()
  assert t.update(10.,10.,True)==(0,0.)
  assert t.update(10.1,10.,True)[1]==pytest.approx(1.)
  assert t.update(11.,10.,True) is None
  assert t.distance==pytest.approx(1.)
  j,s=turn()
  assert s(12.,travel=(1,1.)) is None
