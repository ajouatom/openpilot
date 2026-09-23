"""Exercise the actual worker/IPC lifecycle on the built Linux runtime."""
import os
import subprocess
import sys
import time

import pytest


@pytest.mark.skipif(sys.platform != 'linux', reason='requires built Linux msgq/Params runtime')
@pytest.mark.parametrize('radar_track_flip', [False, True])
def test_radarcan_publishes_invalid_on_missing_input_and_recovers(radar_track_flip):
  from openpilot.cereal import car, messaging
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  from opendbc.car.mock.values import CAR

  with OpenpilotPrefix():
    cp = car.CarParams.new_message(carFingerprint=str(CAR.MOCK), brand='mock', radarTimeStep=0.05)
    Params().put('CarParams', cp.to_bytes())
    Params().put_bool('RadarTrackFlip', radar_track_flip)
    pm = messaging.PubMaster(['can', 'carState'])
    tracks = messaging.sub_sock('liveTracks', conflate=False)
    env = dict(os.environ)
    env.pop('REPLAY', None)
    proc = subprocess.Popen([sys.executable, '-m', 'openpilot.selfdrive.carrot.radar.radarcan'],
                            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
      def drive_until(valid, *, send_can=True, empty_batch=False, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
          assert proc.poll() is None, 'radarcan exited'
          can = messaging.new_message('can', 1)
          cs = messaging.new_message('carState')
          cs.carState.vEgo = 10.0
          cs.carState.radarInput.firstCanMonoTime = 0 if empty_batch else can.logMonoTime
          cs.carState.radarInput.lastCanMonoTime = 0 if empty_batch else can.logMonoTime
          cs.carState.radarInput.canPacketCount = 0 if empty_batch else 1
          cs.carState.radarInput.receiveMonoTime = time.monotonic_ns()
          if send_can:
            pm.send('can', can)
          pm.send('carState', cs)
          time.sleep(0.01)
          for msg in messaging.drain_sock(tracks):
            assert msg.liveTracks.radarTrackFlipped == radar_track_flip
            if msg.valid == valid:
              if not valid:
                assert msg.liveTracks.errors.canError
              return
        pytest.fail(f'no liveTracks valid={valid}, send_can={send_can}')

      drive_until(True)
      # A setting edit cannot change the coordinate system until restart.
      Params().put_bool('RadarTrackFlip', not radar_track_flip)
      messaging.drain_sock(tracks)
      drive_until(False, send_can=False)
      messaging.drain_sock(tracks)
      drive_until(True)
      messaging.drain_sock(tracks)
      drive_until(False, send_can=False, empty_batch=True)
      messaging.drain_sock(tracks)
      drive_until(True)
      # Complete input loss must also invalidate, without a carState wakeup.
      messaging.drain_sock(tracks)
      deadline = time.monotonic() + 2.0
      while time.monotonic() < deadline:
        if any(not msg.valid for msg in messaging.drain_sock(tracks)):
          break
        time.sleep(0.02)
      else:
        pytest.fail('input loss did not invalidate liveTracks')
    finally:
      proc.terminate()
      proc.wait(timeout=10)
