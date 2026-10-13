"""Exercise the actual worker/IPC lifecycle on the built Linux runtime."""
import os
import subprocess
import sys
import time

import pytest

from openpilot.selfdrive.carrot.radar.radarcan import _rebuild_radar_interface, _set_bosch_context


def test_bosch_context_preserves_validity_and_timestamps():
  class Radar:
    def set_bosch_context(self, *args):
      self.context = args

  pose = object()
  model = object()
  sm = type('Context', (), {
    'valid': {'livePose': True, 'modelV2': False},
    'logMonoTime': {'livePose': 90, 'modelV2': 80},
    '__getitem__': lambda self, service: {'livePose': pose, 'modelV2': model}[service],
  })()
  radar = Radar()

  _set_bosch_context(radar, sm, 100)

  assert radar.context == (100, pose, 90, None, 80)


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


def test_rebuild_initial_creation_keeps_the_default_constructor():
  calls = []
  cp = object()

  class Radar:
    ARS510_INPUT_RECOVERY_API = 1

    def recover_from_input_fault(self, previous):
      pytest.fail("최초 기동에서는 복구 callback을 호출하지 않는다")

  def factory(value):
    calls.append(value)
    return Radar()

  assert isinstance(_rebuild_radar_interface(factory, cp), Radar)
  assert calls == [cp]


@pytest.mark.parametrize('api', [None, 0, 2, '1', True, 1.0])
def test_rebuild_requires_a_strict_integer_recovery_api(api):
  calls = []

  class Radar:
    ARS510_INPUT_RECOVERY_API = api

    def recover_from_input_fault(self, previous):
      calls.append(previous)

  rebuilt = _rebuild_radar_interface(lambda cp: Radar(), object(), object())
  assert isinstance(rebuilt, Radar) and calls == []


@pytest.mark.parametrize('callback', [None, True])
def test_rebuild_requires_a_callable_recovery_callback(callback):
  class Radar:
    ARS510_INPUT_RECOVERY_API = 1
    recover_from_input_fault = callback

  assert isinstance(_rebuild_radar_interface(lambda cp: Radar(), object(), object()), Radar)


def test_rebuild_an_ordinary_toyota_keeps_the_original_constructor():
  from opendbc.car import structs
  from opendbc.car.toyota.radar_interface import RadarInterface
  from opendbc.car.toyota.values import CAR

  cp = structs.CarParams(brand='toyota', carFingerprint=str(CAR.TOYOTA_RAV4_TSS2), radarUnavailable=True)
  before = cp.to_dict()
  previous = RadarInterface(cp)
  calls = []

  def factory(value):
    calls.append(value)
    return RadarInterface(value)

  rebuilt = _rebuild_radar_interface(factory, cp, previous)
  assert type(rebuilt) is RadarInterface
  assert calls == [cp] and cp.to_dict() == before
  assert not hasattr(rebuilt, 'ARS510_INPUT_RECOVERY_API')
  assert rebuilt.rcp is None and rebuilt.valid_cnt == previous.valid_cnt


def test_rebuild_after_actual_batch_fault_waits_for_a_new_native_record():
  from opendbc.car import structs
  from opendbc.car.toyota.ars510_carrot import Ars510RadarInterface
  from opendbc.car.toyota.radar_interface import RadarInterface
  from opendbc.car.toyota.tests.test_ars510_carrot import run_record, speed_frame
  from opendbc.car.toyota.values import CAR, ToyotaFlags
  from openpilot.selfdrive.carrot.radar.can_batch import RadarCanBatches, RadarEgoSample

  cp = structs.CarParams(brand='toyota', carFingerprint=str(CAR.TOYOTA_RAV4_TSS2_2022),
      flags=int(ToyotaFlags.RADAR_ACC), radarUnavailable=True, radarTimeStep=0.0,
      radarDelay=0.0, openpilotLongitudinalControl=False)
  before = cp.to_dict()
  previous = _rebuild_radar_interface(RadarInterface, cp)
  first_id = run_record(previous).points[0].trackId
  buffer = RadarCanBatches()
  packets = [(1_040_000_000, [speed_frame()])]
  state = structs.CarState(vEgo=15.0, aEgo=0.0, radarInput={
      'firstCanMonoTime': packets[0][0], 'lastCanMonoTime': packets[-1][0],
      'canPacketCount': 1, 'receiveMonoTime': 1_040_000_000})
  buffer.add_can(packets)
  buffer.add_state(RadarEgoSample.from_car_state(state))
  assert buffer.take(1_200_000_000)[2] == 'staleEgoState'
  restarted = _rebuild_radar_interface(RadarInterface, cp, previous)
  assert type(restarted) is Ars510RadarInterface

  reports = []
  for index in range(80):
    now_ns = 1_210_000_000 + index * 10_000_000
    stock = [(now_ns, [speed_frame()])]
    fresh = RadarEgoSample(now_ns, now_ns, 1, now_ns, 15.0, 0.0)
    buffer.add_can(stock)
    buffer.add_state(fresh)
    ego, exact, error = buffer.take(now_ns)
    assert error is None and exact == stock
    result = restarted.update_carrot(ego.v_ego, ego.a_ego, ego.receive_ns * 1e-9, exact)
    if result is not None:
      reports.append(result)
      assert result.errors.canError and result.errors.radarUnavailableTemporary
      assert len(result.points) == 0
  assert len(reports) == 16
  recovered = run_record(restarted, 1, start=2.1)
  assert len(recovered.points) == 1 and not any(recovered.errors.to_dict().values())
  assert recovered.points[0].trackId > first_id
  assert restarted.tracks[recovered.points[0].trackId].cnt == 1
  assert cp.to_dict() == before
