"""표시 객체를 제어 fused·MyTrack과 분리하고 기존 유효성 경계를 유지한다.

신호 정의는 vendor encode_slot과 기존 CRC/프레임/SPEED 생성 함수를 사용한다.
yaw 입력은 원형 parse_toyota_yaw_rate의 공개 인코딩 역변환이다.
미빌드 환경은 tools.ars510_validation.offline_io.prepare()를 명시적으로 호출한다.
실제 차량·영상·제동 또는 전체 표시 투영을 검증하는 시험은 아니다.
"""

from dataclasses import asdict
import math

import pytest

from opendbc.car import structs
from opendbc.car.toyota.ars510.interface import FUSED_CONFIG
from opendbc.car.toyota.ars510.objects import encode_slot
from opendbc.car.toyota.ars510_carrot import Ars510RadarInterface, TRACK_ID_LIMIT
from opendbc.car.toyota.tests.test_ars510_carrot import make_record, speed_frame, to_frames
from opendbc.car.toyota.values import CAR, ToyotaFlags


def make_cp():
  return structs.CarParams(brand="toyota", carFingerprint=str(CAR.TOYOTA_RAV4_TSS2_2022),
                          flags=int(ToyotaFlags.RADAR_ACC), radarUnavailable=True,
                          radarTimeStep=0., radarDelay=.2, openpilotLongitudinalControl=False)


def slot(y=0., distance=20., ground=15., age=100, uncertainty=0):
  return encode_slot(long_dist=160 + round(distance * 16), lat_dist_left=2048 + round(y / .015),
                     long_vel_over_ground=round(510.5 + ground / .15), age_cycles=age,
                     vel_uncertainty_candidate=uncertainty)


def packets(start=1., *, slots=None, ego=15., yaw=True):
  values = [(round(start * 1e9), [speed_frame(ego)])]
  if yaw:
    raw = round(125. / .244)
    values[0][1].append((0x24, bytes([(raw >> 8) & 3, raw & 255, 0, 0, 0, 0, 0, 0]), 0))
  record = make_record({0: slot(), 1: slot(3.5)} if slots is None else slots)
  values.extend((round((start + .0315 * index / 105) * 1e9), [(0x80, data, 1)])
                for index, data in enumerate(to_frames(record)))
  return values


def step(radar, start=1., *, slots=None, ego=15., yaw=True):
  return radar.update_carrot(ego, 0., start + .0315, packets(start, slots=slots, ego=ego, yaw=yaw))


def wire(cp):
  return cp.as_reader().as_builder().to_bytes()


def points(result, display=False):
  values = result.displayPoints if display else result.points
  return [point.to_dict() for point in values]


def canonical(value):
  if isinstance(value, float) and math.isnan(value):
    return "nan"
  if isinstance(value, dict):
    return {key: canonical(item) for key, item in value.items()}
  if isinstance(value, list):
    return [canonical(item) for item in value]
  return value


def test_display_changes_only_path_gate_config_and_declares_capability():
  radar = Ars510RadarInterface(make_cp())
  original = asdict(FUSED_CONFIG)
  display = asdict(radar.display_decoder.config)
  assert {key for key in original if original[key] != display[key]} == {"path_gate_m"}
  assert display["path_gate_m"] == 0.
  assert radar.decoder.config is FUSED_CONFIG
  assert radar.DISPLAY_POINTS_SUPPORTED is True


@pytest.mark.parametrize("ego,ground", [(0., 0.), (15., 15.), (15., 0.)])
@pytest.mark.parametrize("lateral,control_count", [(0., 1), (3.5, 0), (-3.5, 0)])
def test_side_points_are_displayed_without_entering_control_tracks(ego, ground, lateral, control_count):
  cp = make_cp()
  before = wire(cp)
  radar = Ars510RadarInterface(cp)
  result = step(radar, slots={0: slot(lateral, ground=ground)}, ego=ego)
  assert result.hasDisplayData
  assert len(result.points) == len(radar.pts) == len(radar.tracks) == control_count
  assert len(result.displayPoints) == 1
  assert result.displayPoints[0].yRel == pytest.approx(lateral, abs=.008)
  assert str(result.displayPoints[0].radarSource) == "frontRadar"
  assert result.displayPoints[0].trackState == 0
  assert not result.errors.canError and not result.errors.radarUnavailableTemporary
  assert wire(cp) == before


@pytest.mark.parametrize("values", [
  {0: slot(age=59)},
  {0: slot(3.5, distance=65.)},
  {0: slot(3.5, uncertainty=100)},
])
def test_display_keeps_age_far_and_speed_quality_gates(values):
  radar = Ars510RadarInterface(make_cp())
  result = step(radar, slots=values)
  assert result.hasDisplayData and not result.displayPoints
  assert not result.points and not radar.pts and not radar.tracks
  assert not result.errors.canError and not result.errors.radarUnavailableTemporary
  assert radar.display_decoder.crc_failures == 0


def test_display_contains_the_whole_point_set_and_does_not_create_duplicate_ids():
  radar = Ars510RadarInterface(make_cp())
  result = step(radar)
  assert len(result.points) == len(radar.pts) == len(radar.tracks) == 1
  assert len(result.displayPoints) == 2
  assert len({point.trackId for point in result.displayPoints}) == 2
  assert all(1000 <= point.trackId < TRACK_ID_LIMIT for point in result.displayPoints)
  assert sorted(round(point.yRel, 2) for point in result.displayPoints) == [0., 3.49]


def test_display_empty_record_removes_previous_display_objects():
  radar = Ars510RadarInterface(make_cp())
  assert len(step(radar).displayPoints) == 2
  result = step(radar, 1.06, slots={})
  assert result.hasDisplayData and not result.displayPoints
  assert not result.points and not radar.pts and not radar.tracks


def test_cold_start_empty_publication_keeps_display_marker():
  radar = Ars510RadarInterface(make_cp())
  published = []
  for index in range(1, 11):
    result = radar.update_carrot(0., 0., index * .01, [])
    if result is not None:
      published.append(result)
  assert len(published) == 2
  assert all(value.hasDisplayData and not value.displayPoints and not value.points for value in published)
  assert all(not value.errors.canError and not value.errors.radarUnavailableTemporary for value in published)


@pytest.mark.parametrize("fault", ["crc", "sequence", "dlc", "speed_checksum", "receive_time"])
def test_common_input_fault_clears_control_and_display(fault):
  radar = Ars510RadarInterface(make_cp())
  assert len(step(radar).displayPoints) == 2
  bad = packets(1.06)
  if fault == "receive_time":
    result = radar.update_carrot(15., 0., 1., [])
  else:
    if fault == "sequence":
      bad.pop(8)
    else:
      packet_index, frame_index = (0, 0) if fault == "speed_checksum" else (8, 0)
      timestamp, frames = bad[packet_index]
      address, payload, bus = frames[frame_index]
      data = bytearray(payload)
      if fault == "dlc":
        data.pop()
      else:
        data[-1] ^= 1
      frames[frame_index] = address, bytes(data), bus
      bad[packet_index] = timestamp, frames
      if fault == "speed_checksum":
        # 뒤의 새 정상 CRC record는 원형 복구 조건이다. 장애 직후를 따로 검사한다.
        bad = bad[:1]
    result = radar.update_carrot(15., 0., 1.0915, bad)
  assert result.hasDisplayData and result.errors.canError
  assert result.errors.radarUnavailableTemporary
  assert not result.points and not result.displayPoints
  assert not radar.pts and not radar.tracks and not radar._display_points


def test_timeout_clears_both_and_next_record_uses_new_independent_ids():
  radar = Ars510RadarInterface(make_cp())
  previous = step(radar)
  control_max = max(point.trackId for point in previous.points)
  display_max = max(point.trackId for point in previous.displayPoints)
  result = radar.update_carrot(15., 0., 1.51, [])
  assert result.hasDisplayData and result.errors.radarUnavailableTemporary
  assert not result.points and not result.displayPoints
  result = step(radar, 1.6)
  assert min(point.trackId for point in result.points) > control_max
  assert min(point.trackId for point in result.displayPoints) > display_max
  assert not result.errors.canError and not result.errors.radarUnavailableTemporary


def test_input_recovery_preserves_separate_display_namespace_and_waits_for_crc():
  cp = make_cp()
  previous = Ars510RadarInterface(cp)
  result = step(previous)
  control_max = max(point.trackId for point in result.points)
  display_max = max(point.trackId for point in result.displayPoints)
  recovered = Ars510RadarInterface(cp)
  recovered.recover_from_input_fault(previous)
  for index in range(5):
    waiting = recovered.update_carrot(15., 0., 1.1 + index * .01, [])
  assert waiting.hasDisplayData and waiting.errors.canError and waiting.errors.radarUnavailableTemporary
  assert not waiting.displayPoints and not waiting.points
  result = step(recovered, 1.2)
  assert min(point.trackId for point in result.points) > control_max
  assert min(point.trackId for point in result.displayPoints) > display_max
  assert not result.errors.canError and not result.errors.radarUnavailableTemporary


def test_display_decoder_failure_does_not_change_control_result_or_mytrack(monkeypatch):
  reference = Ars510RadarInterface(make_cp())
  radar = Ars510RadarInterface(make_cp())
  original = radar.display_decoder.update_frame

  def fail_at_record_start(time_s, bus, address, data):
    # 표시 전용 실패를 주입한다. 제어 디코더와 판단 결과는 원형을 실행한다.
    if address == 0x80:
      raise ValueError("표시 입력 장애 시험")
    return original(time_s, bus, address, data)

  monkeypatch.setattr(radar.display_decoder, "update_frame", fail_at_record_start)
  expected, result = step(reference), step(radar)
  assert canonical(points(result)) == canonical(points(expected))
  assert result.errors.to_dict() == expected.errors.to_dict()
  assert result.hasDisplayData and not result.displayPoints
  assert list(radar.tracks) == list(reference.tracks)
  for key in radar.tracks:
    for field in ("cnt", "dRel", "yRel", "yvRel", "vLead", "aLead", "jLead"):
      assert getattr(radar.tracks[key], field) == getattr(reference.tracks[key], field)
  assert radar.diagnostics["display_decoder_exceptions"] == 1
  assert radar.diagnostics["history_resets"] == 0
  assert len(step(radar, 1.06).displayPoints) == 2


def test_display_namespace_overflow_is_isolated_from_control():
  radar = Ars510RadarInterface(make_cp())
  reference = Ars510RadarInterface(make_cp())
  step(radar)
  step(reference)
  radar._display_max_track_id = TRACK_ID_LIMIT - 1
  radar._display_namespace_offset = TRACK_ID_LIMIT - 1
  result, expected = step(radar, 1.06), step(reference, 1.06)
  assert canonical(points(result)) == canonical(points(expected))
  assert result.errors.to_dict() == expected.errors.to_dict()
  assert result.hasDisplayData and not result.displayPoints
  recovered = Ars510RadarInterface(make_cp())
  recovered.recover_from_input_fault(radar)
  result = step(recovered, 1.2)
  assert len(result.points) == 1 and not result.displayPoints
  assert not result.errors.canError and not result.errors.radarUnavailableTemporary


def test_legacy_radar_data_keeps_display_marker_false_by_default():
  result = structs.RadarData()
  assert not result.hasDisplayData and not result.displayPoints
