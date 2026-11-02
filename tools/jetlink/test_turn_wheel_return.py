"""Turn wheel-return completion without cancelling approach/stop or losing next cues."""
import math
import pytest
from blink_turn_journey import TurnJourney
from blink_policy import Policy


def turn(direction='left'):
  j=TurnJourney()
  def step(now,**kw):
    return j.update(now,2,kw.pop('valid',True),kw.pop('cue',11.),kw.pop('direction',direction),kw.pop('atc','turn '+direction),
      travel=kw.pop('travel',(0,0.)),distance=kw.pop('distance',40.),speed=kw.pop('speed',5.),angle=kw.pop('angle',0.),
      heading=kw.pop('heading',0.),pressed=kw.pop('pressed',False),torque=kw.pop('torque',0.),**kw)
  step(10.,cue=0.);assert step(11.)==('turn',direction)
  return j,step


@pytest.mark.parametrize('direction,sign',[('left',1),('right',-1)])
def test_actual_turn_returns_at_fifteen_without_waiting_for_post_turn_distance(direction,sign):
  j,s=turn(direction)
  assert s(12.,travel=(0,30.),angle=160*sign,heading=-.5*sign)
  assert s(13.,travel=(0,42.),angle=15.1*sign,heading=-1.5*sign)
  assert s(13.05,travel=(0,42.25),angle=15*sign,heading=-1.5*sign) is None
  assert j.reason=='completed' and j.active is None
  assert s(13.1,travel=(0,42.5),angle=14*sign,heading=-1.5*sign) is None


@pytest.mark.parametrize('fault',['no_rotation','no_steer','only_thirty','stopped'])
def test_straight_approach_and_traffic_light_wait_are_not_turn_completion(fault):
  j,s=turn()
  angle=30 if fault=='only_thirty' else 0 if fault=='no_steer' else 100
  heading=0 if fault=='no_rotation' else -.7
  assert s(12.,angle=angle,heading=heading,travel=(0,20.))
  assert s(13.,angle=0.,heading=heading,travel=(0,21.),speed=0. if fault=='stopped' else 5.)
  assert j.active


@pytest.mark.parametrize('next_direction',['left','right'])
def test_fresh_next_turn_hands_over_at_wheel_return(next_direction):
  j,s=turn();policy=Policy();policy.update(10,2,True,[],5.,0.,'drive',False)
  original=policy.update(11,2,True,[],5.,0.,'drive',False,('turn','left'))[1]['id']
  assert s(12.,travel=(0,35.),angle=160.,heading=-.5)
  assert s(13.,cue=13.,direction=next_direction,atc='turn '+next_direction,travel=(0,40.),distance=25.,angle=80.,heading=-1.3)==('turn','left')
  assert j.active['turn']==40. and j.pending[1]==65.
  result=s(13.1,travel=(0,41.),angle=15.,heading=-1.5)
  assert result==('turn',next_direction) and not j.active['entered'] and j.active['end']==95.
  h=policy.update(13.1,2,True,[],5.,15.,'drive',False,result)[1]
  assert h and (h['id']==original)==(next_direction=='left')
  assert s(13.2,travel=(0,41.5),angle=0.,heading=-1.5)==result


def test_invalid_pose_and_three_hundred_second_expiry_still_cancel():
  j,s=turn();assert s(12.,angle=100.,heading=-.7)
  assert s(12.1,angle=80.,heading=math.nan) is None
  j,s=turn();assert s(311.,angle=80.,heading=-.7) is None
  assert j.reason=='route_or_distance'


def test_wheel_return_removes_source_hold_immediately_without_new_tap():
  j,s=turn();p=Policy();p.update(10,2,True,[],5.,0.,'drive',False)
  p.update(11,2,True,[],5.,0.,'drive',False,('turn','left'))
  assert s(12.,angle=100.,heading=-.7)
  output=s(13.,angle=15.,heading=-1.5,travel=(0,45.))
  commands,hold=p.update(13,2,True,[],5.,15.,'drive',False,output)
  assert output is None and hold is None and commands==[]


@pytest.mark.parametrize('direction,sign', [('left', 1), ('right', -1)])
def test_driver_cancel_requires_started_steering_and_opposite_zero_crossing(direction, sign):
  journey, step = turn(direction)
  assert step(11.1, angle=-5*sign, pressed=True, torque=-300*sign)
  assert step(11.2, angle=30*sign)
  assert step(11.3, angle=0, pressed=True, torque=-300*sign)
  assert step(11.4, angle=-.1*sign, pressed=True, torque=-300*sign) is None
  assert journey.reason == 'driver_opposite'


def test_opposite_angle_without_driver_input_keeps_unfinished_turn():
  journey, step = turn()
  assert step(11.1, angle=30)
  assert step(11.2, angle=-1, pressed=False, torque=-100)
  assert journey.reason == 'approach'
