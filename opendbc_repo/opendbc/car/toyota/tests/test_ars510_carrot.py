"""공개 합성 레코드로 실제 ARS510→Carrot 수신 계약을 검증한다.

레코드/프레임 helper는 ds-sebastian/ars510-radar fb5b29c2의
tests/test_decode.py make_record/to_frames와 같은 정의다(MIT, vendor/LICENSE).
슬롯 신호는 같은 vendor의 encode_slot을 사용하며 SPEED helper는 공개
tests/test_interface.py의 정의에 기존 Toyota 체크섬을 채운다.
실제 ECU·영상·차량 동작·제어 권한 또는 미검사 header 의미의 증거는 아니다.

미빌드 환경에서는 tools.ars510_validation.offline_io.prepare()를 먼저
명시적으로 실행하고 이 tests 폴더를 --confcutdir로 지정한다.
빌드된 환경의 기본 pytest에서는 Params 모듈을 대체하지 않는다.
"""

import math
from pathlib import Path

import pytest

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.radar_tracks import MyTrack
from opendbc.car.toyota.ars510.constants import ID80_IDLE_SLOT, ID80_RECORD_LEN
from opendbc.car.toyota.ars510.interface import Ars510NativeRadarInterface, FUSED_CONFIG
from opendbc.car.toyota.ars510.objects import encode_slot
from opendbc.car.toyota.ars510.record import id80_crc, id80_crc_ok
from opendbc.car.toyota.ars510_carrot import Ars510RadarInterface, RECORD_ASSEMBLY_TIMEOUT_S, is_supported_ars510
from opendbc.car.toyota.toyotacan import toyota_checksum
from opendbc.car.toyota.values import CAR, ToyotaFlags
from opendbc.dbc.generator.generator import create_dbc


def make_record(slots):
  rec = bytearray(ID80_RECORD_LEN)
  rec[0] = 0xe4
  for slot in range(20):
    rec[17 + 36 * slot:17 + 36 * (slot + 1)] = slots.get(slot, ID80_IDLE_SLOT)
  rec[737:741] = id80_crc(bytes(rec[1:737])).to_bytes(4, "little")
  return bytes(rec)


def to_frames(record):
  frames = [bytes([0x12]) + record[0:7]]
  for index in range(1, len(record) // 7):
    frames.append(bytes([0x20 | (index & 0xf)]) + record[7 * index:7 * index + 7])
  return frames


def speed_frame(speed=15.0):
  code = round(speed * 3.6 / 0.01)
  data = bytearray(bytes(5) + code.to_bytes(2, "big") + b"\x00")
  data[-1] = toyota_checksum(0xb4, None, data)
  return 0xb4, bytes(data), 0


def synthetic_record(index=0, *, age=None, speed=None, slots=True):
  elapsed = index * .06
  velocity = 18.0 + .3 * elapsed + .4 * elapsed**2 if speed is None else speed
  slot = encode_slot(long_dist=160 + round(40.0 * 16), lat_dist_left=2048,
                     long_vel_over_ground=round(510.5 + velocity / .15),
                     age_cycles=72 + index if age is None else age)
  return make_record({0: slot} if slots else {})


def packets_for(record, start, *, span=.0315, speed=True):
  frames = to_frames(record)
  packets = [(round(start * 1e9), [speed_frame()])] if speed else []
  packets.extend((round((start + span * index / 105) * 1e9), [(0x80, data, 1)]) for index, data in enumerate(frames))
  return packets


def run_record(radar, index=0, *, start=None, record=None, speed=True, span=.0315):
  start = 1.0 + .06 * index if start is None else start
  record = synthetic_record(index) if record is None else record
  return radar.update_carrot(15.0, 0.0, start + span, packets_for(record, start, span=span, speed=speed))


def cp_wire(CP):
  # 새 builder를 직렬화해 반복 to_bytes의 pycapnp 경고를 피한다.
  return CP.as_reader().as_builder().to_bytes()


def assert_input_error(result, radar):
  assert result is not None
  assert result.errors.canError
  assert result.errors.radarUnavailableTemporary
  assert len(result.points) == 0
  assert not radar.pts and not radar.tracks


@pytest.fixture
def CP():
  return structs.CarParams(brand="toyota", carFingerprint=str(CAR.TOYOTA_RAV4_TSS2_2022),
                           flags=int(ToyotaFlags.RADAR_ACC), radarUnavailable=True,
                           radarTimeStep=0.0, radarDelay=.2, openpilotLongitudinalControl=False)


@pytest.fixture
def radar(CP):
  return Ars510RadarInterface(CP)


def test_cold_start_has_no_firmware_or_fake_tracks_and_does_not_learn_20hz(CP, radar):
  before = cp_wire(CP)
  results = [radar.update_carrot(15.0, 0.0, 1.0 + index * .01, []) for index in range(150)]
  emitted = [result for result in results if result is not None]
  assert len(CP.carFw) == 0
  assert len(emitted) == 30
  assert all(len(result.points) == 0 and not any(result.errors.to_dict().values()) for result in emitted)
  assert radar.dt == pytest.approx(.06) and radar.init_done
  assert radar.init_samples == []
  assert not radar.pts and not radar.tracks
  assert cp_wire(CP) == before


def test_cold_start_warning_is_once_and_does_not_create_error(radar, monkeypatch):
  # The built runtime routes carlog to its own handler without root propagation.
  # Check the logger boundary instead of depending on pytest's root-log capture.
  warnings = []
  monkeypatch.setattr('opendbc.car.toyota.ars510_carrot.carlog.warning', warnings.append)
  for index in range(1601):
    radar.update_carrot(15.0, 0.0, 1.0 + index * .01, [])
  assert radar.diagnostics["cold_start_warnings"] == 1
  assert len(warnings) == 1 and "첫 CRC 정상" in warnings[0]
  assert not radar.pts and not radar.tracks
  assert radar.diagnostics["history_resets"] == 0


@pytest.mark.parametrize("unavailable", [False, True])
def test_constructor_accepts_supported_cp_without_mutating_unavailable(CP, unavailable):
  CP.radarUnavailable = unavailable
  before = cp_wire(CP)
  assert is_supported_ars510(CP)
  receiver = Ars510RadarInterface(CP)
  assert receiver.CP is CP
  assert cp_wire(CP) == before


@pytest.mark.parametrize("field,value", [("brand", "hyundai"), ("carFingerprint", str(CAR.TOYOTA_RAV4_TSS2)), ("flags", 0)])
def test_unsupported_cp_cannot_construct_receiver(CP, field, value):
  setattr(CP, field, value)
  before = cp_wire(CP)
  assert not is_supported_ars510(CP)
  with pytest.raises(ValueError):
    Ars510RadarInterface(CP)
  assert cp_wire(CP) == before


def test_known_crc_point_has_ground_speed_and_real_mytrack(radar):
  record = synthetic_record()
  assert id80_crc_ok(record)
  speed_data = speed_frame()[1]
  assert speed_data[-1] == toyota_checksum(0xb4, None, bytearray(speed_data))
  packets = packets_for(record, 1.0)
  native = Ars510NativeRadarInterface(FUSED_CONFIG)
  expected = None
  for timestamp, frames in packets:
    for address, data, bus in frames:
      value = native.update_frame(timestamp * 1e-9, bus, address, data)
      if value is not None:
        expected = value["radarData"]["points"][0]
  result = radar.update_carrot(15.0, 0.0, 1.0315, packets)
  assert len(result.points) == len(radar.pts) == len(radar.tracks) == 1
  point = result.points[0]
  assert point.dRel == pytest.approx(expected["dRel"])
  assert point.yRel == pytest.approx(expected["yRel"])
  assert point.vRel == pytest.approx(expected["vRel"])
  assert point.vLead == pytest.approx(expected["v_long_ground"])
  assert point.vLead == pytest.approx(point.vRel + 15.0)
  assert point.vLead != 0.0
  assert str(point.radarSource) == "frontRadar" and point.trackState == 0
  assert point.measured and isinstance(radar.tracks[point.trackId], MyTrack)
  assert radar.tracks[point.trackId].cnt == 1


def test_dynamic_acceleration_and_jerk_are_current_mytrack_values_and_serialize(CP, radar):
  before = cp_wire(CP)
  accelerations, jerks = [], []
  for index in range(24):
    result = run_record(radar, index)
    point = result.points[0]
    track = radar.tracks[point.trackId]
    assert track.cnt == index + 1
    assert point.aLead == pytest.approx(track.aLead if track.cnt >= 6 else 0.0)
    assert point.jLead == pytest.approx(track.jLead if track.cnt >= 6 else 0.0)
    assert all(math.isfinite(getattr(point, field)) for field in ("dRel", "yRel", "vRel", "vLead", "aLead", "jLead"))
    with structs.RadarData.from_bytes(result.to_bytes()) as reread:
      assert reread.points[0].aLead == point.aLead
      assert reread.points[0].jLead == point.jLead
      assert reread.points[0].vLead == point.vLead
    accelerations.append(point.aLead)
    jerks.append(point.jLead)
  assert any(abs(value) > .01 for value in accelerations[6:])
  assert any(abs(value) > .01 for value in jerks[6:])
  assert cp_wire(CP) == before


def test_normal_empty_record_removes_missing_tracks(radar):
  run_record(radar)
  result = run_record(radar, 1, record=synthetic_record(1, slots=False))
  assert len(result.points) == 0 and not any(result.errors.to_dict().values())
  assert not radar.pts and not radar.tracks
  assert radar.diagnostics["published_records"] == 2


def test_young_track_and_missing_ego_are_withheld_without_fabrication(radar):
  young = run_record(radar, record=synthetic_record(age=40))
  assert len(young.points) == 0 and not radar.tracks
  assert radar.diagnostics["settling_suppressed"] > 0
  missing_ego = run_record(radar, 1, start=2.0, speed=False)
  assert len(missing_ego.points) == 0 and not radar.tracks
  assert radar.diagnostics["unresolved_suppressed"] > 0


@pytest.mark.parametrize("kind", ["crc", "sequence", "type", "dlc7", "dlc9"])
def test_record_corruption_clears_state_then_valid_record_uses_new_id(radar, kind):
  old_id = run_record(radar).points[0].trackId
  record = bytearray(synthetic_record(1))
  if kind == "crc":
    record[100] ^= 0x40
  packets = packets_for(bytes(record), 1.06)
  timestamp, frames = packets[2]
  address, data, bus = frames[0]
  if kind == "sequence":
    data = b"\x22" + data[1:]
  elif kind == "type":
    data = b"\x30" + data[1:]
  elif kind == "dlc7":
    data = data[:-1]
  elif kind == "dlc9":
    data += b"\x00"
  packets[2] = timestamp, [(address, data, bus)]
  assert_input_error(radar.update_carrot(15.0, 0.0, 1.0915, packets), radar)
  counter = {"crc": "crc_failures", "sequence": "sequence_errors", "type": "bad_frame_type",
             "dlc7": "bad_frame_length", "dlc9": "bad_frame_length"}[kind]
  assert radar.diagnostics[counter] == 1
  recovered = run_record(radar, 2)
  assert not any(recovered.errors.to_dict().values())
  assert recovered.points[0].trackId > old_id
  assert radar.tracks[recovered.points[0].trackId].cnt == 1


@pytest.mark.parametrize("address,bus", [(0xb4, 0), (0x24, 0), (0x235, 1), (0x237, 1), (0x192, 1), (0x194, 1)])
def test_support_dlc_rejection_includes_disabled_fused_summary_inputs(radar, address, bus):
  run_record(radar)
  invalid_length = 3 if address in (0x192, 0x194) else 7
  result = radar.update_carrot(15.0, 0.0, 1.04, [(1_040_000_000, [(address, bytes(invalid_length), bus)])])
  assert_input_error(result, radar)
  assert radar.diagnostics["support_frame_length"] == 1


@pytest.mark.parametrize("corruption", ["checksum_only", "speed_payload"])
def test_speed_checksum_matches_standard_parser_and_requires_new_crc_record(radar, tmp_path, corruption):
  root = Path(__file__).resolve().parents[5]
  create_dbc(str(root / "opendbc_repo/opendbc/dbc/generator/toyota"), "toyota_nodsu_pt.dbc", str(tmp_path))
  parser = CANParser(str(tmp_path / "toyota_nodsu_pt_generated.dbc"), [("SPEED", 100)], 0)
  old_id = run_record(radar).points[0].trackId
  address, original, bus = speed_frame()
  damaged = bytearray(original)
  damaged[-1 if corruption == "checksum_only" else 5] ^= 0x08
  assert damaged[-1] != toyota_checksum(address, None, damaged)
  packets = [(1_040_000_000, [(address, bytes(damaged), bus)])]
  assert parser.update(packets) == set()
  assert_input_error(radar.update_carrot(15.0, 0.0, 1.04, packets), radar)
  assert radar.diagnostics["support_checksum_errors"] == 1
  assert radar.diagnostics["history_resets"] == 1

  # 정상이 된 속도 입력과 부분 프레임만으로 객체·오류 복구를 선언하지 않는다.
  result = radar.update_carrot(15.0, 0.0, 1.05, [(1_050_000_000, [speed_frame()])])
  assert result is None or result.errors.canError
  assert not radar.pts and not radar.tracks
  partial = packets_for(synthetic_record(1), 1.06)[:3]
  result = radar.update_carrot(15.0, 0.0, 1.0606, partial)
  assert result is None or result.errors.canError
  assert not radar.pts and not radar.tracks

  recovered = run_record(radar, 2)
  assert not any(recovered.errors.to_dict().values())
  assert recovered.points[0].trackId > old_id
  assert radar.tracks[recovered.points[0].trackId].cnt == 1
  assert parser.update([(1_120_000_000, [speed_frame()])]) == {0xb4}


@pytest.mark.parametrize("bus", [1, 2])
def test_other_bus_speed_checksum_does_not_change_valid_front_history(radar, bus):
  first = run_record(radar)
  track_id = first.points[0].trackId
  address, original, _ = speed_frame()
  damaged = bytearray(original)
  damaged[5] ^= 0x08
  resets = radar.diagnostics["history_resets"]
  radar.update_carrot(15.0, 0.0, 1.04, [(1_040_000_000, [(address, bytes(damaged), bus)])])
  assert radar.diagnostics["support_checksum_errors"] == 0
  assert radar.diagnostics["history_resets"] == resets
  second = run_record(radar, 1)
  assert not any(second.errors.to_dict().values())
  assert second.points[0].trackId == track_id
  assert radar.tracks[track_id].cnt == 2


@pytest.mark.parametrize("address", [0x192, 0x194])
def test_valid_four_byte_range_summary_does_not_reset_object_history(radar, address):
  first = run_record(radar)
  old_id = first.points[0].trackId
  resets = radar.diagnostics["history_resets"]
  radar.update_carrot(15.0, 0.0, 1.04, [(1_040_000_000, [(address, bytes(4), 1)])])
  assert radar.diagnostics["history_resets"] == resets
  assert radar.diagnostics["support_frame_length"] == 0
  second = run_record(radar, 2)
  assert second.points[0].trackId == old_id
  assert radar.tracks[old_id].cnt == 2


def test_all_seven_upstream_inputs_reach_actual_decoder_without_enabling_summary_filter(radar, monkeypatch):
  calls = []
  original = radar.decoder.update_frame

  def observe(time_s, bus, address, data):
    # 관측 목록만 추가하고 입력·반환·계산 본문은 원형으로 전달한다.
    calls.append((bus, address))
    return original(time_s, bus, address, data)

  monkeypatch.setattr(radar.decoder, "update_frame", observe)
  support = [(0x24, bytes(8), 0), (0x235, bytes(8), 1), (0x237, bytes(8), 1),
             (0x192, bytes(8), 1), (0x194, bytes(8), 1)]
  packets = packets_for(synthetic_record(), 1.0)
  packets[0][1].extend(support)
  result = radar.update_carrot(15.0, 0.0, 1.0315, packets)
  assert {(0, 0xb4), (0, 0x24), (1, 0x80), (1, 0x235), (1, 0x237), (1, 0x192), (1, 0x194)} <= set(calls)
  assert radar.decoder.config.summary_sigma_mps == 0.0
  # 0x24의 0 코드는 회전율 -125deg/s다. 정상 원형 경로 판정이 점을 제외한다.
  assert len(result.points) == 0 and not any(result.errors.to_dict().values())
  assert radar.diagnostics["records"] == radar.diagnostics["path_gated"] == 1


@pytest.mark.parametrize("data", [None, 8])
def test_relevant_nonbyte_frame_is_rejected_without_allocating_or_decoding_it(radar, data):
  result = radar.update_carrot(15.0, 0.0, 1.0, [(1_000_000_000, [(0x80, data, 1)])])
  assert_input_error(result, radar)
  assert radar.diagnostics["invalid_frame_data"] == 1


def test_wrong_bus_and_unrelated_can_do_not_create_points_or_reset_history(radar):
  packets = packets_for(synthetic_record(), 1.0)
  packets[0][1].extend([(0x80, b"\x00", 0), (0x7ff, None, 1)])
  result = radar.update_carrot(15.0, 0.0, 1.0315, packets)
  assert len(result.points) == 1
  assert radar.diagnostics["history_resets"] == 0


@pytest.mark.parametrize("delta,rejected", [(-.0001, False), (.0001, True)])
def test_assembly_deadline_compares_continuation_to_first_frame(radar, delta, rejected):
  span = RECORD_ASSEMBLY_TIMEOUT_S + delta
  result = run_record(radar, span=span)
  if rejected:
    assert_input_error(result, radar)
    assert radar.diagnostics["assembly_timeout"] == 1
  else:
    assert len(result.points) == 1
    assert radar.diagnostics["assembly_timeout"] == 0


def test_loss_deadline_uses_record_start_even_when_no_can_packets_arrive(radar):
  old_id = run_record(radar).points[0].trackId
  assert radar.update_carrot(15.0, 0.0, 1.4999, []) is None
  assert old_id in radar.pts
  result = radar.update_carrot(15.0, 0.0, 1.5001, [])
  assert result.errors.radarUnavailableTemporary and not result.errors.canError
  assert len(result.points) == 0 and not radar.pts and not radar.tracks
  assert radar.diagnostics["timeouts"] == 1
  recovery = run_record(radar, 1, start=1.6)
  assert recovery.points[0].trackId > old_id
  assert not any(recovery.errors.to_dict().values())


def test_partial_record_does_not_claim_recovery_after_crc_error(radar):
  corrupt = bytearray(synthetic_record())
  corrupt[100] ^= 1
  assert_input_error(run_record(radar, record=bytes(corrupt)), radar)
  first = to_frames(synthetic_record(1))[0]
  radar.update_carrot(15.0, 0.0, 1.04, [(1_040_000_000, [(0x80, first, 1)])])
  reports = [radar.update_carrot(15.0, 0.0, 1.05 + .01 * index, []) for index in range(10)]
  assert all(result.errors.canError and result.errors.radarUnavailableTemporary for result in reports if result is not None)
  assert not radar.pts and not radar.tracks


def test_timestamp_preserved_batch_matches_sequential_mytrack_and_adds_one_ego_sample(CP):
  sequential = Ars510RadarInterface(CP)
  batch = Ars510RadarInterface(CP)
  packets = []
  for index in range(24):
    packets.extend(packets_for(synthetic_record(index), 1.0 + .06 * index))
    expected = run_record(sequential, index)
  result = batch.update_carrot(15.0, 0.0, 2.4115, packets)
  assert batch.diagnostics["published_records"] == 24
  assert len(batch.v_ego_hist) == len(batch.a_ego_hist) == 2
  assert list(batch.v_ego_hist) == [0.0, 15.0]
  assert batch.tracks[result.points[0].trackId].cnt == 24
  for field in ("dRel", "yRel", "vRel", "vLead", "aLead", "jLead"):
    assert getattr(result.points[0], field) == pytest.approx(getattr(expected.points[0], field))
  assert batch.tracks[result.points[0].trackId].jLead == pytest.approx(sequential.tracks[expected.points[0].trackId].jLead)


def test_same_packet_multiple_record_timestamp_is_rejected(radar):
  frames = [(0x80, data, 1) for data in to_frames(synthetic_record()) + to_frames(synthetic_record(1))]
  result = radar.update_carrot(15.0, 0.0, 1.0, [(1_000_000_000, [speed_frame(), *frames])])
  assert_input_error(result, radar)
  assert radar.diagnostics["ambiguous_batch_time"] == 1


def test_same_record_start_with_distinct_completion_is_rejected(radar):
  first = [(0x80, data, 1) for data in to_frames(synthetic_record())]
  packets = [(1_000_000_000, [speed_frame(), *first])]
  packets.extend(packets_for(synthetic_record(age=72, speed=19.0), 1.0, speed=False))
  result = radar.update_carrot(15.0, 0.0, 1.0315, packets)
  assert_input_error(result, radar)
  assert radar.diagnostics["ambiguous_batch_time"] == 1


@pytest.mark.parametrize("field", ["v_ego", "a_ego", "rcv_time"])
@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_nonfinite_external_state_is_rejected(radar, field, value):
  args = {"v_ego": 15.0, "a_ego": 0.0, "rcv_time": 1.0, "can_packets": []}
  args[field] = value
  assert_input_error(radar.update_carrot(**args), radar)
  assert radar.diagnostics["nonfinite_input"] == 1


def test_reversed_receive_time_clears_tracks(radar):
  run_record(radar)
  assert_input_error(radar.update_carrot(15.0, 0.0, 1.0, []), radar)
  assert radar.diagnostics["reversed_receive_time"] == 1


def test_reversed_packet_time_clears_tracks_without_rewriting_time(radar):
  run_record(radar)
  assert_input_error(radar.update_carrot(15.0, 0.0, 1.04, [(1_000_000_000, [speed_frame()])]), radar)
  assert radar.diagnostics["invalid_packet_time"] == 1


@pytest.mark.parametrize("timestamp", [float("nan"), float("inf"), "잘못된 시각"])
def test_invalid_packet_time_is_rejected_before_native_filter(radar, timestamp):
  assert_input_error(radar.update_carrot(15.0, 0.0, 1.0, [(timestamp, [speed_frame()])]), radar)
  assert radar.diagnostics["invalid_packet_time"] == 1
  assert radar.diagnostics["records"] == 0


def test_slot_age_restart_does_not_reuse_previous_object_id(radar):
  first_id = run_record(radar).points[0].trackId
  restarted = run_record(radar, 1, record=synthetic_record(age=61))
  assert restarted.points[0].trackId != first_id
  assert radar.tracks[restarted.points[0].trackId].cnt == 1


@pytest.mark.parametrize("offset,rejected", [(2**31 - 2, False), (2**31 - 1, True)])
def test_downstream_int32_id_limit_does_not_wrap(radar, offset, rejected):
  # 장시간 namespace 소진의 경계 fixture이며 권한·측정·판정 코드는 대체하지 않는다.
  radar._namespace_offset = offset
  result = run_record(radar)
  if rejected:
    assert_input_error(result, radar)
    assert radar.diagnostics["invalid_track_id"] == 1
  else:
    assert result.points[0].trackId == 2**31 - 1


def test_input_fault_recovery_preserves_seen_state_until_new_crc_record(CP, radar):
  first_id = run_record(radar).points[0].trackId
  previous_diagnostics = radar.diagnostics.copy()
  before = cp_wire(CP)
  restarted = Ars510RadarInterface(CP)
  restarted.recover_from_input_fault(radar)
  assert restarted._seen_record and restarted._fault_latched
  assert not restarted.pts and not restarted.tracks
  assert restarted.diagnostics["input_recovery_resets"] == 1

  emitted = []
  for index in range(80):
    time_ns = 1_040_000_000 + index * 10_000_000
    result = restarted.update_carrot(15.0, 0.0, time_ns * 1e-9, [(time_ns, [speed_frame()])])
    if result is not None:
      emitted.append(result)
      assert_input_error(result, restarted)
  assert len(emitted) == 16
  assert restarted.diagnostics["cold_start_publications"] == 0
  partial = packets_for(synthetic_record(1), 1.9)[:3]
  result = restarted.update_carrot(15.0, 0.0, 1.9006, partial)
  assert result is None or (result.errors.canError and result.errors.radarUnavailableTemporary)
  assert not restarted.pts and not restarted.tracks

  recovered = run_record(restarted, 2, start=2.0)
  assert not any(recovered.errors.to_dict().values())
  assert recovered.points[0].trackId > first_id
  assert restarted.tracks[recovered.points[0].trackId].cnt == 1
  assert radar.diagnostics == previous_diagnostics
  assert cp_wire(CP) == before


def test_input_fault_during_cold_start_does_not_claim_healthy_empty_state(CP, radar):
  restarted = Ars510RadarInterface(CP)
  restarted.recover_from_input_fault(radar)
  assert not restarted._seen_record and restarted._fault_latched
  reports = [restarted.update_carrot(15.0, 0.0, 1.0 + index * .01, []) for index in range(10)]
  assert sum(value is not None for value in reports) == 2
  for result in reports:
    if result is not None:
      assert_input_error(result, restarted)
  assert restarted.diagnostics["cold_start_publications"] == 0
  recovered = run_record(restarted, start=1.2)
  assert len(recovered.points) == 1 and not any(recovered.errors.to_dict().values())


def test_input_fault_recovery_preserves_exhausted_int32_namespace(CP, radar):
  # 실제 번호 소진 경계의 fixture이며 CAN 신호·객체·판정 함수를 대체하지 않는다.
  radar._max_track_id = 2**31 - 1
  radar._namespace_offset = 2**31
  restarted = Ars510RadarInterface(CP)
  restarted.recover_from_input_fault(radar)
  assert restarted._namespace_offset == 2**31
  assert_input_error(run_record(restarted), restarted)
  assert restarted.diagnostics["invalid_track_id"] == 1
  assert restarted._namespace_offset == 2**31


def test_input_fault_recovery_rejects_an_unrelated_receiver(radar):
  with pytest.raises(ValueError, match="같은 수신기"):
    radar.recover_from_input_fault(object())
  assert not radar._fault_latched and not radar.pts and not radar.tracks
