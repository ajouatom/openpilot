"""Compare receipt-cached frequency checks with the original per-loop contract."""
import random

import openpilot.cereal.messaging as messaging
from openpilot.cereal.services import SERVICE_LIST


def test_health_matches_uncached_checks_through_dropouts(monkeypatch):
  # No live subscribers or publishers: only real Cap'n Proto messages and clocks.
  class Poller:
    def registerSocket(self, *args):
      pass
  monkeypatch.setattr(messaging, 'Poller', Poller)
  monkeypatch.setattr(messaging, 'sub_sock', lambda *args, **kwargs: object())
  services = ['carState', 'controlsState', 'modelV2', 'radarState', 'livePose', 'liveCalibration',
              'managerState', 'deviceState', 'carParams', 'onroadEvents', 'userBookmark']
  sm = messaging.SubMaster(services, frequency=100)
  rng = random.Random(20261013)
  for frame in range(2500):
    now = 1.0 + frame / 100
    if frame in (300, 1100):
      sm.ignore_alive.append('modelV2')
      sm.ignore_average_freq.append('carState')
      sm.ignore_valid.append('radarState')
    if frame in (950, 1600):
      sm.ignore_alive.clear()
      sm.ignore_average_freq.clear()
      sm.ignore_valid.clear()
    sm.simulation = 1700 <= frame < 2100
    msgs = []
    for service in services:
      if 500 <= frame < 800 or 1800 <= frame < 2000:
        continue
      if rng.random() > min(1., max(SERVICE_LIST[service].frequency, .8) / 100):
        continue
      msg = messaging.new_message(service, 0) if service in ('onroadEvents',) else messaging.new_message(service)
      msg.valid = not (1000 <= frame < 1150 or rng.random() < .03)
      msg.logMonoTime = int(now * 1e9)
      msgs.append(msg.as_reader())
    sm.update_msgs(now, msgs)
    for service in sm.static_freq_services:
      assert sm.freq_ok[service] == (sm.freq_tracker[service].valid or sm.simulation)
      assert sm.alive[service] == (now - sm.recv_time[service] < 10. / SERVICE_LIST[service].frequency or
                                   (sm.seen[service] and sm.simulation))
    for subset in (None, [], ['modelV2'], ['carState', 'modelV2', 'livePose'], services):
      checked = subset or services
      alive = all(sm.alive[s] for s in checked if s not in sm.ignore_alive)
      freq_ok = all(sm.freq_ok[s] for s in checked if sm._check_avg_freq(s))
      valid = all(sm.valid[s] for s in checked if s not in sm.ignore_valid)
      assert sm.all_alive(subset) == alive
      assert sm.all_freq_ok(subset) == freq_ok
      assert sm.all_valid(subset) == valid
      assert sm.all_checks(subset) == (alive and freq_ok and valid)
