import pytest

from openpilot.selfdrive.carrot.signal_assist import SignalAssist


def observation(t, state='red', ident=1, session='worker1', box=(640, 250, 680, 270)):
  return dict(timestamp=t, frame_id=round(t * 20), session=session, tracks=[
    dict(id=ident, box=box, state=state, age=0., observations=10, evidence={'raw': state})])


def step(a, t, o=None, **kw):
  context = dict(enabled=True, valid=True, drive=True, gas=False, speed=0., model_distance=3.,
                 model_y=0., comfort_brake=2.4, lead=False, entry_allowed=True)
  context.update(kw)
  return a.update(t, o, **context)


def confirm(a, start=0., state='red', ident=1, **kw):
  for i in range(12):
    t = start + i * .05
    result = step(a, t, observation(t, state, ident), **kw)
  return result


def test_hold_survives_unknown_and_a_moving_lead():
  a = SignalAssist()
  assert confirm(a).hold
  assert step(a, 10., lead=True).hold
  assert step(a, 100., observation(100., 'unknown'), lead=True).hold


def test_green_must_be_same_track_and_continuous():
  a = SignalAssist()
  confirm(a)
  assert confirm(a, 1., 'green', ident=2).hold
  assert not confirm(a, 2., 'green', ident=1).hold
  assert not step(a, 2.6).red_sign


def test_restart_cannot_supply_green_permission():
  a = SignalAssist()
  confirm(a)
  for i in range(20):
    t = 1 + i * .05
    assert step(a, t, observation(t, 'green', session='restarted')).hold


@pytest.mark.parametrize('bad', ['duplicate', 'old', 'future', 'malformed'])
def test_bad_evidence_cannot_acquire_a_stop(bad):
  a = SignalAssist()
  for i in range(20):
    t = i * .05
    o = observation(0 if bad == 'duplicate' else t - 1 if bad == 'old' else t + 1 if bad == 'future' else t)
    if bad == 'malformed':
      o['tracks'][0]['box'] = [float('nan')] * 4
    r = step(a, t, o)
    assert not r.hold and not r.red_sign


def test_one_green_flash_cannot_release():
  a = SignalAssist()
  confirm(a)
  assert step(a, .6, observation(.6, 'green')).hold
  assert step(a, .65, observation(.65, 'unknown')).hold
  assert step(a, 2).hold


@pytest.mark.parametrize('override', [dict(enabled=False), dict(valid=False), dict(drive=False), dict(gas=True)])
def test_explicit_exit_overrides_hold(override):
  a = SignalAssist()
  confirm(a)
  assert not step(a, .6, **override).hold


def test_driver_gas_has_ten_second_reacquisition_cooldown():
  a = SignalAssist()
  confirm(a)
  step(a, .6, gas=True)
  assert not confirm(a, 1.).hold
  assert confirm(a, 11.).hold


def test_moving_red_uses_existing_plausible_distance():
  a = SignalAssist()
  r = confirm(a, speed=10., model_distance=40.)
  assert r.red_sign and not r.hold
  assert step(a, 5., speed=5., model_distance=20.).red_sign
  assert step(a, 10., speed=0.).hold


@pytest.mark.parametrize('context', [
  dict(model_distance=1.), dict(model_distance=500.), dict(model_distance=float('nan')),
  dict(lead=True), dict(entry_allowed=False), dict(turning=True), dict(model_y=10.),
  dict(speed=30.), dict(comfort_brake=0.),
])
def test_no_new_moving_red_for_ineligible_context(context):
  a = SignalAssist()
  args = dict(speed=10., model_distance=40.)
  args.update(context)
  assert not confirm(a, **args).red_sign


def test_off_axis_signal_does_not_arm():
  a = SignalAssist()
  for i in range(20):
    t = i * .05
    assert not step(a, t, observation(t, box=(50, 250, 100, 270))).red_sign


def test_confirmation_breaks_on_gap():
  a = SignalAssist()
  for t in (0., .1, .2, .6, .7):
    assert not step(a, t, observation(t)).hold


def test_rewind_clears_latch():
  a = SignalAssist()
  confirm(a, 10.)
  assert not step(a, 1.).hold


def test_nested_red_lamp_does_not_replace_complete_housing():
  a = SignalAssist()
  for i in range(12):
    t = i * .05
    o = observation(t, box=(640, 250, 700, 280))
    o['tracks'].extend(observation(t, ident=2, box=(644, 256, 660, 264))['tracks'])
    r = step(a, t, o)
  assert r.hold and r.track_id == 1
  assert not confirm(a, 1., 'green', ident=1).hold
