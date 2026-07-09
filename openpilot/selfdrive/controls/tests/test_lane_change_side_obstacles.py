from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.desire_lib.side_state import SideState
from openpilot.selfdrive.controls.radard import pick_side_lead, select_side_leads


def lead(d_rel: float, v_rel: float, v_ego: float, d_path: float = 2.0):
  return SimpleNamespace(
    status=True,
    dRel=d_rel,
    vRel=v_rel,
    vLead=v_ego + v_rel,
    dPath=d_path,
  )


def settle_clear(side: SideState, frames: int = 10):
  for _ in range(frames):
    side.update_obstacles(25.0, SimpleNamespace(status=False), False, False)


class TestLaneChangeSideObstacles:
  def test_blocks_closing_side_lead_by_ttc(self):
    side = SideState("left")
    settle_clear(side)

    side.update_obstacles(25.0, lead(20.0, -10.0, 25.0), False, False)

    assert side.side_object_detected

  def test_allows_side_lead_pulling_away_with_gap(self):
    side = SideState("left")
    settle_clear(side)

    side.update_obstacles(25.0, lead(20.0, 5.0, 25.0), False, False)

    assert not side.side_object_detected

  def test_blocks_close_side_lead_even_when_pulling_away(self):
    side = SideState("left")
    settle_clear(side)

    side.update_obstacles(25.0, lead(5.0, 5.0, 25.0), False, False)

    assert side.side_object_detected


class TestSideLeadSelection:
  def test_corner_side_leads_replace_front_side_leads_when_available(self):
    front = [{"dRel": 8.0, "dPath": 2.0}]
    corner = [{"dRel": 12.0, "dPath": 2.0}]

    assert select_side_leads(front, corner, True) == corner

  def test_front_side_leads_are_fallback_without_corner_tracks(self):
    front = [{"dRel": 8.0, "dPath": 2.0}]
    corner = [{"dRel": 12.0, "dPath": 2.0}]

    assert select_side_leads(front, corner, False) == front

  def test_pick_side_lead_prefers_nearest_valid_candidate(self):
    leads = [
      {"dRel": 20.0, "dPath": 2.0},
      {"dRel": 10.0, "dPath": 2.0},
      {"dRel": 7.0, "dPath": 5.0},
    ]

    assert pick_side_lead(leads)["dRel"] == 10.0
