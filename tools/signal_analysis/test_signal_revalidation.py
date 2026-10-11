import cv2,numpy as np,pytest
from unittest.mock import patch
from tools.signal_analysis import signal_tracker as tracker_module
from openpilot.selfdrive.carrot.signal_assist import SignalAssist
from openpilot.selfdrive.carrot.tests.test_signal_assist import observation,step

def supported_red(t,**changes):
 o=observation(t);o['tracks'][0].update(seen_red=True,support_state='red',support_since=t-.4,support_count=3)
 o['tracks'][0].update(changes);return o

def test_red_history_can_arm_on_fresh_stopped_sample():
 a=SignalAssist(producer_red_history=True);r=step(a,1.59,supported_red(1.4));assert r.hold

def test_red_history_requires_separate_trial_opt_in():
 assert not step(SignalAssist(),1.59,supported_red(1.4)).hold

@pytest.mark.parametrize('changes',[dict(support_state='green'),dict(support_count=2),dict(support_since=1.2),dict(support_since=0),dict(support_since=float('nan')),dict(seen_red=False),dict(age=.05),dict(state='unknown'),dict(evidence={'raw':'green'})])
def test_bad_red_history_cannot_arm(changes):
 assert not step(SignalAssist(producer_red_history=True),1.59,supported_red(1.4,**changes)).hold

def test_expired_red_history_cannot_arm():
 assert not step(SignalAssist(producer_red_history=True),1.601,supported_red(1.4)).hold

def picture(color,seed=19):
 gray=np.random.default_rng(seed).integers(15,100,(760,1344),dtype=np.uint8)
 rgb=np.repeat(gray[:,:,None],3,axis=2);rgb[230:242,630:678]=10
 if color in ('red','green'):
  cv2.circle(rgb,(636 if color=='red' else 672,236),3,(240,0,0) if color=='red' else (0,240,0),-1)
 return rgb

def sample(tracker,t,color,seed=19,offset=0):
 im=picture(color,seed)
 if offset:im=np.roll(im,offset,axis=1)
 det=dict(box=[630+offset,230,678+offset,242],raw=color,quality=.9,source='test')
 with patch.object(tracker_module,'detect',return_value=[det] if color!='unknown' else []):
  return tracker.process(im,t)

def armed():
 t=tracker_module.SignalTracker(daytime_cores=True, robust_tracking=True)
 for ts in [1.,1.15,1.3]:r=sample(t,ts,'red')
 assert r['state']=='red';return t,r['tracks'][0]['id']

def test_switch_with_delayed_frame_preserves_verified_identity_not_color_streak():
 t,ident=armed();r=sample(t,1.6,'green')
 assert r['tracks'][0]['id']==ident and r['tracks'][0]['seen_red']
 assert r['tracks'][0]['support_count']==1 and r['state']=='unknown'
 for ts in [1.75,1.9,2.05]:r=sample(t,ts,'green')
 assert r['state']=='green'

@pytest.mark.parametrize('time,seed,offset',[(1.81,19,0),(1.6,77,0),(1.6,19,50)])
def test_long_gap_changed_context_or_distant_object_cannot_inherit_red(time,seed,offset):
 t,ident=armed()
 for ts in [time,time+.15,time+.30,time+.45]:r=sample(t,ts,'green',seed,offset)
 assert r['state']=='unknown'
 assert all(not v['seen_red'] for v in r['tracks'])

def test_unknown_bridge_never_keeps_old_green():
 t,ident=armed()
 for ts in [1.45,1.6,1.75]:r=sample(t,ts,'green')
 assert r['state']=='green'
 r=sample(t,2.05,'unknown')
 assert r['state']=='unknown'

def test_textureless_context_cannot_bridge():
 t,ident=armed();t.previous_gray[:]=20
 for ts in [1.6,1.75,1.9,2.05]:r=sample(t,ts,'green')
 assert r['state']=='unknown'

def test_one_missed_image_reacquires_only_tightly_matching_detected_housing():
 t,ident=armed()
 # Do not allow a failed template to act as new evidence.
 t.previous_gray=None
 with patch.object(tracker_module,'detect',return_value=[]):r=t.process(picture('unknown'),1.5)
 assert r['state']=='unknown'
 def det(box):return dict(box=box,raw='green',quality=.9,source='detected')
 r=t.update(1.65,[det([631,231,677,242])]);assert r['tracks'][0]['id']==ident
 assert r['state']=='unknown'
 for ts in [1.8,1.95,2.1]:r=t.update(ts,[det([631,231,677,242])])
 assert r['state']=='green'

@pytest.mark.parametrize('box',[[641,230,689,242],[630,230,700,242],[630,235,678,250]])
def test_missed_image_does_not_relax_geometry_for_other_housing(box):
 t,ident=armed();t.update(1.5,[])
 for ts in [1.65,1.8,1.95,2.1]:r=t.update(ts,[dict(box=box,raw='green',quality=.9,source='detected')])
 assert r['state']=='unknown'

def test_ambiguous_old_housings_do_not_supply_identity():
 t,ident=armed()
 from copy import deepcopy
 duplicate=deepcopy(t.tracks[0]);duplicate.ident=99;t.tracks.append(duplicate)
 t.update(1.5,[])
 r=t.update(1.65,[dict(box=[631,231,677,242],raw='green',quality=.9,source='detected')])
 assert all(v['state']=='unknown' for v in r['tracks'])
 fresh=next(v for v in r['tracks'] if v['age']==0)
 assert not fresh['seen_red']
