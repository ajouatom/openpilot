from types import SimpleNamespace

from openpilot.selfdrive.controls.radard import (
  CORNER_CENTER_MIN_AGE,
  CORNER_STOPPED_MIN_AGE,
  RadarD,
  Track,
  is_radar_center_promotion_safe,
  is_vision_radar_lateral_match_sane,
)


def lead(d_rel: float, y_rel: float, d_path: float, v_rel: float) -> dict:
  return {
    "dRel": d_rel,
    "yRel": y_rel,
    "dPath": d_path,
    "vRel": v_rel,
  }


class TestRadarCenterPromotion:
  def test_vision_radar_lateral_match_rejects_adjacent_lane_distance_match(self):
    assert not is_vision_radar_lateral_match_sane(2.8, -0.1, 3.3)

  def test_vision_radar_lateral_match_keeps_curved_path_and_real_cutin(self):
    assert is_vision_radar_lateral_match_sane(2.8, -0.1, 0.4)
    assert is_vision_radar_lateral_match_sane(2.8, 2.5, 3.0)

  def test_accepts_close_lead(self):
    assert is_radar_center_promotion_safe(lead(30.0, -0.4, 0.2, 2.0))

  def test_accepts_far_closing_lead(self):
    assert is_radar_center_promotion_safe(lead(70.0, 0.2, -0.3, -1.0))

  def test_rejects_ambiguous_curved_lane(self):
    assert not is_radar_center_promotion_safe(lead(58.5, -1.6, 0.4, 2.5))

  def test_rejects_far_receding_lead(self):
    assert not is_radar_center_promotion_safe(lead(60.0, -0.2, 0.1, 1.0))

  def test_far_corner_center_requires_front_radar_match(self):
    radar = RadarD.__new__(RadarD)
    radar.lane_line_available = True
    track = SimpleNamespace(
      is_corner_radar=True,
      cnt=CORNER_CENTER_MIN_AGE,
      dRel=68.0,
      vLead=20.0,
      cut_in_count=0,
      in_lane_prob=1.0,
      dPath=0.0,
    )

    assert not radar._is_corner_center_candidate(track)
    assert radar._is_corner_center_candidate(track, matched_front=True)

  def test_close_born_corner_center_requires_support_or_cutin_history(self):
    radar = RadarD.__new__(RadarD)
    radar.lane_line_available = True
    track = SimpleNamespace(
      is_corner_radar=True,
      cnt=CORNER_CENTER_MIN_AGE,
      dRel=41.0,
      vLead=11.0,
      cut_in_count=0,
      in_lane_prob=1.0,
      dPath=0.0,
    )

    assert not radar._is_corner_center_candidate(track)
    assert radar._is_corner_center_candidate(track, matched_front=True)
    track.cut_in_count = 1
    assert radar._is_corner_center_candidate(track)

  def test_discontinuous_track_restarts_age(self):
    track = Track(1189)

    def point(d_rel: float, v_lead: float):
      return SimpleNamespace(
        dRel=d_rel,
        yRel=-0.2,
        vRel=v_lead - 25.0,
        vLead=v_lead,
        aLead=0.0,
        jLead=0.0,
        yvRel=0.0,
        measured=True,
        radarSource="corner235",
      )

    for _ in range(6):
      track.update(None, point(71.4, 0.5), False, 1.0, 0.0, 0.0)
    assert track.cnt == 6

    track.update(None, point(64.1, 20.8), False, 1.0, 0.0, 0.0)
    assert track.cnt == 1

  def test_close_unmatched_corner_stopped_reflection_is_rejected(self):
    radar = RadarD.__new__(RadarD)
    radar.lane_line_available = True
    track = SimpleNamespace(
      is_corner_radar=True,
      cnt=CORNER_STOPPED_MIN_AGE,
      dRel=54.0,
      vLead=0.5,
      yvLead=0.0,
      in_lane_prob=1.0,
      dPath=0.0,
      corner_stopped_acquired=False,
    )

    assert not radar._is_corner_stopped_candidate(track)
    assert radar._is_corner_stopped_candidate(track, matched_front=True)

  def test_far_acquired_corner_stopped_lead_is_held_at_close_range(self):
    radar = RadarD.__new__(RadarD)
    radar.lane_line_available = True
    track = SimpleNamespace(
      is_corner_radar=True,
      cnt=CORNER_STOPPED_MIN_AGE,
      dRel=80.0,
      vLead=0.5,
      yvLead=0.0,
      in_lane_prob=1.0,
      dPath=0.0,
      corner_stopped_acquired=False,
    )

    assert radar._is_corner_stopped_candidate(track)
    track.dRel = 54.0
    assert radar._is_corner_stopped_candidate(track)
