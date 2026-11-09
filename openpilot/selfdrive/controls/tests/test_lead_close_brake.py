from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib import lead_close_brake as lcb

KPH = 1 / 3.6


def lead(dRel=8.0, vLead=10.0, aLeadK=0.0, status=True, radar=True):
  return SimpleNamespace(dRel=dRel, vLead=vLead, aLeadK=aLeadK, status=status, radar=radar)


def test_request_when_closing_inside_follow_distance():
  assert lcb.tf_extra_request(40 * KPH, lead(dRel=8.0, vLead=40 * KPH - 0.5), 10.0) == lcb.TF_EXTRA


@pytest.mark.parametrize('kw', (
  dict(dRel=10.5),                        # outside the follow distance
  dict(vLead=40 * KPH - 0.2),             # not closing fast enough
  dict(aLeadK=0.5, vLead=40 * KPH - 0.5),  # lead pulling away: acceleration is left alone
  dict(status=False),
))
def test_no_request(kw):
  base = dict(dRel=8.0, vLead=40 * KPH - 0.5)
  assert lcb.tf_extra_request(40 * KPH, lead(**(base | kw)), 10.0) == 0.0


@pytest.mark.parametrize(('kph', 'scale'), ((30, 1.0), (45, 1.0), (55, 0.5), (65, 0.0), (100, 0.0)))
def test_linear_speed_fade(kph, scale):
  # gentle closing (0.5 m/s, wide gap): full up to 45 km/h, linear to none at 65 km/h
  assert lcb.tf_extra_request(kph * KPH, lead(dRel=8.0, vLead=kph * KPH - 0.5), 30.0) == pytest.approx(lcb.TF_EXTRA * scale)
  assert lcb.a_change_cost_factor(kph * KPH, lead(aLeadK=-1.0), 200.) == pytest.approx(1 - scale * 0.75)


def test_ramps_and_obstacle_weight_follow_extra():
  dt = 0.05
  assert lcb.ramp_extra(0.0, 0.3, dt) == pytest.approx(0.03)   # rises 0.6 s/s
  assert lcb.ramp_extra(0.3, 0.0, dt) == pytest.approx(0.295)  # falls 0.1 s/s
  assert lcb.ramp_extra(0.29, 0.3, dt) == pytest.approx(0.3)
  assert lcb.obstacle_cost_factor(0.0) == 1.0 and lcb.obstacle_cost_factor(lcb.TF_EXTRA) == lcb.OBSTACLE_COST_MAX
  assert lcb.obstacle_cost_factor(lcb.TF_EXTRA / 2) == pytest.approx(1 + (lcb.OBSTACLE_COST_MAX - 1) / 2)


def test_a_change_cost_only_while_lead_brakes():
  assert lcb.a_change_cost_factor(30 * KPH, lead(aLeadK=-0.4), 200.) == 1.0
  assert lcb.a_change_cost_factor(30 * KPH, lead(aLeadK=-1.0, status=False), 200.) == 1.0
  assert lcb.a_change_cost_factor(30 * KPH, lead(aLeadK=-1.0), 200.) * 200. == pytest.approx(lcb.BRAKING_A_CHANGE_COST)


def test_highway_short_gap_recorded():
  # 11-08 21:25:28.5 (route 00000089 seg14): 0.5 s gap setting, lead braking, no extra before this branch
  v = 79.7 * KPH
  assert lcb.tf_extra_request(v, lead(dRel=14.3, vLead=v - 1.53, aLeadK=-0.5), 28.4) == lcb.TF_EXTRA


@pytest.mark.parametrize(('dRel', 'vRel', 'expected'), (
  (0.9 * 85 * KPH - 0.1, -1.5, lcb.TF_EXTRA),
  (0.9 * 85 * KPH + 0.1, -1.5, 0.0),  # TIME_GAP
  (18.0, -1.1, lcb.TF_EXTRA),
  (18.0, -0.9, 0.0),                  # HIGHWAY_CLOSING_SPEED
))
def test_highway_short_gap_boundaries(dRel, vRel, expected):
  assert lcb.tf_extra_request(85 * KPH, lead(dRel=dRel, vLead=85 * KPH + vRel, aLeadK=-0.4), 40.0) == expected


def test_short_gap_ignores_fade_band():
  assert lcb.speed_fade(55 * KPH) == pytest.approx(0.5)
  assert lcb.tf_extra_request(55 * KPH, lead(dRel=10.0, vLead=55 * KPH - 1.5), 30.0) == lcb.TF_EXTRA


def test_highway_cut_in_recorded_unchanged():
  # 11-08 22:21:59 (route 0000008b seg7): faster car cut in at 8.2 m, then one vision-only lead frame
  v = 97.4 * KPH
  assert lcb.tf_extra_request(v, lead(dRel=8.2, vLead=v + 0.29, aLeadK=-0.02), 40.2) == 0.0
  assert lcb.tf_extra_request(v, lead(dRel=23.6, vLead=v - 1.38, aLeadK=0.01, radar=False), 57.7) == 0.0
