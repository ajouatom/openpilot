from openpilot.selfdrive.controls.radard import is_radar_center_promotion_safe


def lead(d_rel: float, y_rel: float, d_path: float, v_rel: float) -> dict:
  return {
    "dRel": d_rel,
    "yRel": y_rel,
    "dPath": d_path,
    "vRel": v_rel,
  }


class TestRadarCenterPromotion:
  def test_accepts_close_lead(self):
    assert is_radar_center_promotion_safe(lead(30.0, -0.4, 0.2, 2.0))

  def test_accepts_far_closing_lead(self):
    assert is_radar_center_promotion_safe(lead(70.0, 0.2, -0.3, -1.0))

  def test_rejects_ambiguous_curved_lane(self):
    assert not is_radar_center_promotion_safe(lead(58.5, -1.6, 0.4, 2.5))

  def test_rejects_far_receding_lead(self):
    assert not is_radar_center_promotion_safe(lead(60.0, -0.2, 0.1, 1.0))
