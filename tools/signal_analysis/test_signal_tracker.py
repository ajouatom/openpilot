import unittest
from signal_tracker import SignalTracker


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


if __name__=='__main__':unittest.main()
