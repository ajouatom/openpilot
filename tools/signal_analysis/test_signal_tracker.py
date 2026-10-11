import unittest
import cv2
import numpy as np
from signal_tracker import SignalTracker, night_proposals


def detection(state, box=None):
  return dict(raw=state, box=box or [100,100,140,110], quality=.8, source='test')


class TestCausalState(unittest.TestCase):
  def setUp(self):
    self.observer=SignalTracker()
    self.time=1.

  def step(self, state, box=None):
    result=self.observer.update(self.time, [] if state is None else [detection(state,box)])
    self.time+=.05
    return result

  def red_then_green(self):
    for _ in range(5):self.step('red')
    for _ in range(5):result=self.step('green')
    self.assertEqual(result['state'],'green')

  def test_startup_green_is_unarmed(self):
    for _ in range(20):result=self.step('green')
    self.assertEqual(result['state'],'unknown')

  def test_confirmation_not_single_frame(self):
    for _ in range(5):self.step('red')
    self.assertEqual(self.step('green')['state'],'unknown')
    for _ in range(3):result=self.step('green')
    self.assertEqual(result['state'],'green')

  def test_contradictory_red_removes_green_immediately(self):
    self.red_then_green()
    self.assertNotEqual(self.step('red')['state'],'green')

  def test_unknown_does_not_extend_support(self):
    self.red_then_green()
    self.assertEqual(self.step('unknown')['state'],'green')
    for _ in range(4):result=self.step('unknown')
    self.assertEqual(result['state'],'unknown')

  def test_lost_track_expires(self):
    self.red_then_green()
    for _ in range(4):result=self.step(None)
    self.assertEqual(result['state'],'unknown')

  def test_input_gap_resets_arming(self):
    self.red_then_green();self.time+=.3
    for _ in range(6):result=self.step('green')
    self.assertEqual(result['state'],'unknown')

  def test_new_location_does_not_inherit_green(self):
    self.red_then_green()
    for _ in range(8):result=self.step('green',[300,100,340,110])
    self.assertEqual(result['state'],'unknown')

  def test_disagreeing_confirmed_tracks_abstain(self):
    for _ in range(5):
      self.observer.update(self.time,[detection('red'),detection('red',[300,100,340,110])]);self.time+=.05
    for _ in range(5):
      result=self.observer.update(self.time,[detection('green'),detection('red',[300,100,340,110])]);self.time+=.05
    self.assertEqual(result['state'],'unknown')

  def test_two_frame_cadence_confirms_but_long_gap_restarts(self):
    for t in [1., 1.1004, 1.2008]:
      result = self.observer.update(t, [detection('red')])
    self.assertEqual(result['state'], 'red')
    for t in [1.3009, 1.4011, 1.5012]:
      result = self.observer.update(t, [detection('green')])
    self.assertEqual(result['state'], 'green')
    result = self.observer.update(1.64, [])
    self.assertEqual(result['state'], 'unknown')
    result = self.observer.update(1.65, [detection('red')])
    self.assertEqual(result['state'], 'unknown')

  def test_recorded_internal_worker_cadence_confirms_each_color(self):
    # The 150-200 ms measured duty-limited cadence used to reset every sample.
    for t in [1., 1.2, 1.35]:
      result = self.observer.update(t, [detection('red')])
    self.assertEqual(result['state'], 'red')
    for t in [1.55, 1.75]:
      result = self.observer.update(t, [detection('green')])
      self.assertEqual(result['state'], 'unknown')
    result = self.observer.update(1.95, [detection('green')])
    self.assertEqual(result['state'], 'green')
    # Continuity tolerance does not extend output freshness.
    self.assertEqual(self.observer.update(2.09, [])['state'], 'unknown')

  def test_unknown_breaks_color_count_despite_long_track_history(self):
    for t in [1., 1.2, 1.4, 1.6]:
      self.observer.update(t, [detection('red')])
    self.observer.update(1.8, [detection('green')])
    self.observer.update(2., [detection('unknown')])
    self.observer.update(2.2, [detection('green')])
    self.assertEqual(self.observer.update(2.4, [detection('green')])['state'], 'unknown')
    self.assertEqual(self.observer.update(2.6, [detection('green')])['state'], 'green')

  def test_late_result_is_invalid_without_erasing_red_identity(self):
    from openpilot.selfdrive.modeld.signal_tracking_shadow import result_fields
    for t in [1., 1.2, 1.4]:
      result = self.observer.update(t, [detection('red')])
    fields = result_fields(result, 208.)
    self.assertFalse(fields['fresh'])
    self.assertEqual(fields['prediction'], 'unknown')
    for t in [1.6, 1.8, 2.]:
      result = self.observer.update(t, [detection('green')])
    self.assertEqual(result['state'], 'green')
    # A true camera gap still expires identity, even without a worker reset.
    self.assertEqual(self.observer.update(2.3, [detection('green')])['state'], 'unknown')


class TestNightProposals(unittest.TestCase):
  @staticmethod
  def frame(color=(255, 0, 0), reflected=False, background=0):
    rgb = np.full((760, 1344, 3), background, dtype=np.uint8)
    cv2.circle(rgb, (650, 260), 9, color, -1)
    if reflected:
      rgb[:260] = background
    cv2.circle(rgb, (650, 260), 3, (255, 255, 255), -1)
    return rgb

  def test_white_center_retains_red_halo_evidence(self):
    proposals = night_proposals(self.frame())
    self.assertEqual(len(proposals), 1)
    self.assertEqual(proposals[0]['raw'], 'red')

  def test_one_sided_red_reflection_does_not_seed(self):
    self.assertEqual(night_proposals(self.frame(reflected=True)), [])

  def test_white_core_without_color_is_not_signal(self):
    self.assertEqual(night_proposals(self.frame(color=(0, 0, 0))), [])

  def test_night_extension_is_inactive_in_bright_scene(self):
    self.assertEqual(night_proposals(self.frame(background=100)), [])

  def test_night_green_does_not_grant_startup_green(self):
    proposals = night_proposals(self.frame(color=(0, 255, 0)))
    self.assertTrue(proposals)
    tracker = SignalTracker()
    for t in np.arange(1., 2., .05):
      result = tracker.update(float(t), proposals)
    self.assertEqual(result['state'], 'unknown')


if __name__=='__main__':unittest.main()
