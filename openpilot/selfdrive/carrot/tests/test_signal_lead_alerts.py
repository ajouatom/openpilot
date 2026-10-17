import pytest
from openpilot.selfdrive.carrot.tests.test_signal_lead_handover import setup, update
from tools.signal_analysis.test_signal_assist_planner import inputs
from types import SimpleNamespace as NS

@pytest.mark.parametrize('event', ['trafficSignChanged','trafficSignGreen','trafficStopping','audioRefuse'])
@pytest.mark.parametrize('immediate', [False, True])
def test_lead_mutes_only_signal_notices_and_does_not_consume_cooldown(event, immediate):
  p, sm = setup()
  sm['radarState'].leadOne = NS(status=True, dRel=100.)
  update(p, sm, 1.)
  assert p.start_decision.hold  # far lead: the control veto is unchanged
  assert not p.events.events
  p.last_event_time = 0.
  p.add_signal_event(event, immediate=immediate)
  assert not p.events.events and p.last_event_time == 0.
  p.add_event('radarStationaryLead')
  assert p.events.events == ['radarStationaryLead']
  sm['radarState'].leadOne.status = False
  update(p, sm, 2.)
  p.last_event_time = 0.
  p.add_signal_event(event, immediate=immediate)
  assert event in p.events.events


def test_navigation_signal_notice_uses_current_lead_before_navigation_update():
  p, sm = setup()
  seen = []
  def nav(sm, speed, cruise):
    p.add_signal_event('trafficSignChanged', immediate=True)
    seen.append(p.signal_lead_present)
    return cruise, False
  p._update_carrot_man = nav
  sm['radarState'].leadOne.status = True
  update(p, sm, 1.)
  assert seen == [True] and 'trafficSignChanged' not in p.events.events
  sm['radarState'].leadOne.status = False
  update(p, sm, 2.)
  assert seen == [True, False] and 'trafficSignChanged' in p.events.events
