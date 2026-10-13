"""RAV4 2022 RADAR_ACC의 ARS510 fused 수신을 Carrot MyTrack에 연결한다.

디코더 9개 파일은 ds-sebastian/ars510-radar의 fb5b29c2 커밋을 그대로
포함하며 라이선스는 ars510/LICENSE다. 이 파일만 Carrot 수신 계약을 추가한다.
CP·제어 필터 계수·Panda·PCS를 변경하지 않으며 CAN 송신/ECU 초기화 API가 없다.
"""

from collections import Counter
from dataclasses import replace
from math import isfinite

from opendbc.car import structs
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.toyota.ars510.constants import (
  ACC_TARGET_POS_ADDR, ACC_TARGET_VREL_ADDR, ID80_ADDR, RADAR_CYCLE_S, SUMMARY_ADDRS, TOYOTA_KINEMATICS_ADDR, TOYOTA_SPEED_ADDR,
)
from opendbc.car.toyota.ars510.interface import Ars510NativeRadarInterface, FUSED_CONFIG
from opendbc.car.toyota.ars510.transport import Id80RecordAssembler
from opendbc.car.toyota.toyotacan import toyota_checksum
from opendbc.car.toyota.values import CAR, ToyotaFlags


# 기존 레코드 조립 검증 한계를 유지한다. 모델 시각 차이 문턱과는 별개다.
RECORD_ASSEMBLY_TIMEOUT_S = 0.15
RADAR_TIMEOUT_S = 0.5
NO_RECORD_WARN_S = 15.0
EMPTY_PERIOD = 5
ARS510_RECEIVER_API = 1
TRACK_ID_LIMIT = 2**31


def _finite(value):
  try:
    return isfinite(value)
  except (TypeError, OverflowError):
    return False


def _packet_time(timestamp_ns):
  try:
    return timestamp_ns * 1e-9
  except (TypeError, OverflowError):
    return float("nan")


def is_supported_ars510(CP):
  """모델/기능만 지원 판정에 사용한다. CP.radarUnavailable은 변경하지 않는다."""
  return (getattr(CP, "brand", None) == "toyota"
          and str(CP.carFingerprint) == str(CAR.TOYOTA_RAV4_TSS2_2022)
          and bool(int(CP.flags) & ToyotaFlags.RADAR_ACC.value))


class StrictId80RecordAssembler(Id80RecordAssembler):
  """0x80의 프레임 길이·순서·시각을 확인한 뒤 원형 조립기를 호출한다."""

  def __init__(self):
    self.counters = Counter()
    self.rejections = 0
    self._input_time_s = None
    super().__init__()

  def _reject(self, reason):
    self.counters[reason] += 1
    self.rejections += 1
    self.reset()

  def push(self, time_s, data):
    if not _finite(time_s):
      self._reject("nonfinite_time")
      return None
    if self._input_time_s is not None and time_s < self._input_time_s:
      self._reject("reversed_time")
      self._input_time_s = time_s
      return None
    self._input_time_s = time_s
    if len(data) != 8:
      self._reject("bad_frame_length")
      return None

    if self._is_start(data):
      if self._chunks:
        self.counters["interrupted_records"] += 1
      self.counters["starts"] += 1
      return super().push(time_s, data)

    if data[0] >> 4 != 2:
      self._reject("bad_frame_type")
      return None
    if not self._chunks:
      self.counters["orphan_continuations"] += 1
      return None
    if time_s - self._start_time_s > RECORD_ASSEMBLY_TIMEOUT_S:
      self._reject("assembly_timeout")
      return None
    # 실로그의 시작 직후는 0x21이며, 0x2f 다음은 0x20이다.
    if (data[0] & 0xf) != (len(self._chunks) & 0xf):
      self._reject("sequence_errors")
      return None
    complete = super().push(time_s, data)
    if complete is not None:
      self.counters["completed_records"] += 1
    return complete


class Ars510RadarInterface(RadarInterfaceBase):
  """원형 fused 결과를 실제 Carrot RadarData와 MyTrack 처리에 연결한다."""

  ARS510_INPUT_RECOVERY_API = 1
  DISPLAY_POINTS_SUPPORTED = True

  def __init__(self, CP):
    if not is_supported_ars510(CP):
      raise ValueError("ARS510 수신기는 Toyota RAV4 TSS2 2022 RADAR_ACC만 지원합니다")
    super().__init__(CP)
    # 원본 CP의 radarTimeStep을 수정하지 않고 검증한 record 주기를 사용한다.
    self.dt = RADAR_CYCLE_S
    self.init_done = True
    self.decoder = self._new_decoder()
    self.display_decoder = self._new_display_decoder()
    self._display_points = {}
    self._display_namespace_offset = 1000
    self._display_max_track_id = 999
    self._display_counters = Counter()
    self._display_decoder_counters = Counter()
    self._counters = Counter()
    self._decoder_counters = Counter()
    self._assembler_counters = Counter()
    self._record_time_s = None
    self._packet_time_s = None
    self._receive_time_s = None
    self._completion_time_s = None
    self._blocked_packet_time_s = None
    # MyTrack의 코너 슬롯 예약 범위와 겹치지 않는 전방 객체 번호를 사용한다.
    self._namespace_offset = 1000
    self._max_track_id = 999
    self._unavailable = True
    self._seen_record = False
    self._fault_latched = False
    self._start_time_s = None
    self._warned = False
    self._outer_frame = 0
    self._inside_carrot_update = False

  @staticmethod
  def _new_decoder():
    # v_long_ground가 필요하므로 원형 FUSED_CONFIG의 metadata=True를 보존한다.
    decoder = Ars510NativeRadarInterface(FUSED_CONFIG)
    decoder._assembler = StrictId80RecordAssembler()
    return decoder

  @staticmethod
  def _new_display_decoder():
    # 제어 fused는 그대로 둔다. 표시는 경로 폭만 해제하고 나이·거리·속도 품질을 유지한다.
    decoder = Ars510NativeRadarInterface(replace(FUSED_CONFIG, path_gate_m=0.0))
    decoder._assembler = StrictId80RecordAssembler()
    return decoder

  def recover_from_input_fault(self, previous):
    """재구성 후에도 오류를 유지한다. 이전 객체·필터는 정상 기록까지 재사용하지 않는다."""
    if not isinstance(previous, Ars510RadarInterface):
      raise ValueError("ARS510 복구 상태는 같은 수신기에서만 전달할 수 있습니다")
    self._seen_record = self._seen_record or previous._seen_record
    # 새 namespace가 이전 정상 객체나 아직 소진 중인 번호 경계로 돌아가지 않게 한다.
    self._max_track_id = max(self._max_track_id, self._namespace_offset - 1,
                             previous._max_track_id, previous._namespace_offset - 1)
    self._display_max_track_id = max(self._display_max_track_id, self._display_namespace_offset - 1,
                                    previous._display_max_track_id, previous._display_namespace_offset - 1)
    self._fault_latched = True
    self._reset_history()
    self._counters["input_recovery_resets"] += 1

  @property
  def diagnostics(self):
    """재초기화 전후를 합친 관측 수치이며 실차 안전 검증 결과는 아니다."""
    values = self._counters.copy()
    for key in ("records", "crc_failures", "settling_suppressed", "unresolved_suppressed", "path_gated"):
      values[key] = self._decoder_counters[key] + getattr(self.decoder, key)
    for key, value in self.decoder._assembler.counters.items():
      values[key] = self._assembler_counters[key] + value
    for key, value in self._assembler_counters.items():
      if key not in self.decoder._assembler.counters:
        values[key] = value
    values["namespace_offset"] = self._namespace_offset
    values["active_points"] = len(self.pts)
    for key in ("records", "crc_failures", "settling_suppressed", "unresolved_suppressed", "path_gated"):
      values[f"display_{key}"] = self._display_decoder_counters[key] + getattr(self.display_decoder, key)
    values.update({f"display_{key}": value for key, value in self._display_counters.items()})
    values["display_namespace_offset"] = self._display_namespace_offset
    values["active_display_points"] = len(self._display_points)
    for key in ("starts", "completed_records", "sequence_errors", "bad_frame_length", "bad_frame_type",
                "assembly_timeout", "nonfinite_time", "reversed_time", "interrupted_records",
                "orphan_continuations", "no_record_updates", "timeouts", "history_resets",
                "published_records", "published_points", "invalid_points", "received_id80_frames",
                "invalid_native_record", "invalid_packet_time", "nonfinite_input", "reversed_receive_time",
                "support_frame_length", "support_checksum_errors", "invalid_frame_data", "ambiguous_batch_time", "invalid_track_id",
                "cold_start_publications", "cold_start_warnings", "input_recovery_resets"):
      values.setdefault(key, 0)
    return dict(values)

  def _reset_display_history(self):
    for key in ("records", "crc_failures", "settling_suppressed", "unresolved_suppressed", "path_gated"):
      self._display_decoder_counters[key] += getattr(self.display_decoder, key)
    self.display_decoder = self._new_display_decoder()
    self._display_points.clear()
    self._display_namespace_offset = self._display_max_track_id + 1
    self._display_counters["history_resets"] += 1

  def _display_error(self, reason):
    # 표시 장애가 제어 오류·기존 MyTrack·제어 객체 번호를 바꾸지 않게 격리한다.
    self._display_counters[reason] += 1
    self._reset_display_history()

  def _convert_display(self, decoded):
    points = {}
    for value in decoded["radarData"]["points"]:
      required = (value["dRel"], value["yRel"], value["vRel"], value["v_long_ground"])
      if not all(_finite(v) for v in required) or value["dRel"] <= 0:
        self._display_error("invalid_points")
        return
      native_id = value["trackId"]
      if not isinstance(native_id, int) or isinstance(native_id, bool):
        self._display_error("invalid_track_id")
        return
      track_id = native_id + self._display_namespace_offset
      if native_id < 0 or track_id >= TRACK_ID_LIMIT or track_id in points:
        self._display_error("invalid_track_id")
        return
      self._display_max_track_id = max(self._display_max_track_id, track_id)
      yv_rel = value.get("yvRel", 0.0)
      points[track_id] = structs.RadarData.RadarPoint(
        trackId=track_id,
        dRel=float(value["dRel"]),
        yRel=float(value["yRel"]),
        vRel=float(value["vRel"]),
        vLead=float(value["v_long_ground"]),
        aRel=float(value.get("aRel", float("nan"))),
        yvRel=float(yv_rel) if _finite(yv_rel) else 0.0,
        measured=bool(value["measured"]),
        radarSource="frontRadar",
        trackState=0,
      )
    self._display_points = points
    self._display_counters["published_records"] += 1
    self._display_counters["published_points"] += len(points)

  def _update_display_frame(self, time_s, src, address, data, control_decoded):
    try:
      assembler = self.display_decoder._assembler
      rejected_before = assembler.rejections
      crc_before = self.display_decoder.crc_failures
      decoded = self.display_decoder.update_frame(time_s, src, address, data)
      if assembler.rejections > rejected_before or self.display_decoder.crc_failures > crc_before:
        self._display_error("invalid_record")
      elif control_decoded is None:
        if decoded is not None:
          self._display_error("record_mismatch")
      elif decoded is None or decoded["time_s"] != control_decoded["time_s"]:
        self._display_error("record_mismatch")
      else:
        self._convert_display(decoded)
    except Exception:
      self._display_error("decoder_exceptions")

  def _reset_history(self):
    for key in ("records", "crc_failures", "settling_suppressed", "unresolved_suppressed", "path_gated"):
      self._decoder_counters[key] += getattr(self.decoder, key)
    self._assembler_counters.update(self.decoder._assembler.counters)
    self.decoder = self._new_decoder()
    self._reset_display_history()
    self.pts.clear()
    self.tracks.clear()
    self._namespace_offset = self._max_track_id + 1
    self._record_time_s = None
    self._completion_time_s = None
    self._unavailable = True
    self._counters["history_resets"] += 1

  def _input_error(self):
    self._fault_latched = True
    self._reset_history()
    return self._unavailable_result(can_error=True)

  def _waiting_result(self, now_s):
    """빈 초기상태/손실은 외부 호출 100Hz 기준 20Hz로만 발행한다."""
    if not self._unavailable or now_s is None:
      return None
    if self._start_time_s is None:
      self._start_time_s = now_s
    if not self._seen_record and not self._fault_latched and not self._warned and now_s - self._start_time_s > NO_RECORD_WARN_S:
      carlog.warning("ARS510: 첫 CRC 정상 객체 레코드의 수신을 기다리고 있습니다")
      self._warned = True
      self._counters["cold_start_warnings"] += 1
    if self._outer_frame % EMPTY_PERIOD:
      return None
    if self._seen_record or self._fault_latched:
      return self._unavailable_result(can_error=self._fault_latched)
    # 새 레코드/객체를 만들지 않는다. 원형 upstream의 cold-start 빈 상태다.
    self._counters["cold_start_publications"] += 1
    return self._new_result()

  @staticmethod
  def _new_result():
    return structs.RadarData(hasDisplayData=True)

  @staticmethod
  def _unavailable_result(*, can_error=False):
    ret = Ars510RadarInterface._new_result()
    ret.errors.canError = can_error
    ret.errors.radarUnavailableTemporary = True
    return ret

  def _convert(self, decoded):
    points = {}
    for value in decoded["radarData"]["points"]:
      required = (value["dRel"], value["yRel"], value["vRel"], value["v_long_ground"])
      if not all(isfinite(v) for v in required) or value["dRel"] <= 0:
        self._counters["invalid_points"] += 1
        continue
      native_id = value["trackId"]
      if not isinstance(native_id, int) or isinstance(native_id, bool):
        self._counters["invalid_points"] += 1
        self._counters["invalid_track_id"] += 1
        return self._input_error()
      track_id = native_id + self._namespace_offset
      # RadarPoint는 UInt64지만 downstream RadarState의 객체 번호는 Int32다.
      if native_id < 0 or track_id >= TRACK_ID_LIMIT or track_id in points:
        self._counters["invalid_points"] += 1
        self._counters["invalid_track_id"] += 1
        return self._input_error()
      self._max_track_id = max(self._max_track_id, track_id)
      yv_rel = value.get("yvRel", 0.0)
      # 기존 Toyota 경로의 횡속도 기본값과 같이, 미해결 횡속도는 0으로 표시한다.
      point = structs.RadarData.RadarPoint(
        trackId=track_id,
        dRel=float(value["dRel"]),
        yRel=float(value["yRel"]),
        vRel=float(value["vRel"]),
        vLead=float(value["v_long_ground"]),
        aRel=float(value.get("aRel", float("nan"))),
        yvRel=float(yv_rel) if isfinite(yv_rel) else 0.0,
        measured=bool(value["measured"]),
        radarSource="frontRadar",
        trackState=0,
      )
      points[track_id] = point
    # ret.points만 만들면 부모 클래스가 MyTrack을 갱신하지 못한다.
    # 각 정상 record의 전체 집합으로 바꿔 누락된 객체는 즉시 제거한다.
    self.pts = points
    self._counters["published_records"] += 1
    self._counters["published_points"] += len(points)
    ret = self._new_result()
    ret.points = list(points.values())
    ret.displayPoints = list(self._display_points.values())
    return ret

  def update_carrot(self, v_ego, a_ego, rcv_time, can_packets):
    """각 record를 실제 수신 시각으로 부모 필터에 순서대로 전달한다."""
    if not all(_finite(value) for value in (v_ego, a_ego, rcv_time)):
      self._counters["nonfinite_input"] += 1
      return self._input_error()
    if self._receive_time_s is not None and rcv_time < self._receive_time_s:
      self._counters["reversed_receive_time"] += 1
      self._input_error()
      self._packet_time_s = None
      self._receive_time_s = rcv_time
      return self._unavailable_result(can_error=True)
    result = None
    self._outer_frame += 1
    if self._start_time_s is None:
      self._start_time_s = rcv_time
    ego_history = self.v_ego_hist.copy()
    accel_history = self.a_ego_hist.copy()

    def parent_step(time_s, packets):
      # 외부 상태는 한 샘플이다. 내부 프레임 분할로 지연 보상 이력을 늘리지 않는다.
      self.v_ego_hist = ego_history.copy()
      self.a_ego_hist = accel_history.copy()
      return super(Ars510RadarInterface, self).update_carrot(v_ego, a_ego, time_s, packets)

    def retain(value):
      nonlocal result
      if value is not None:
        if result is not None and result.errors.canError and value.errors.radarUnavailableTemporary:
          # 오류 뒤의 부분 프레임이나 무입력 검사만으로 정상 복구를 선언하지 않는다.
          value.errors.canError = True
        result = value

    self._inside_carrot_update = True
    try:
      for timestamp_ns, frames in can_packets:
        packet_time_s = _packet_time(timestamp_ns)
        step_time_s = packet_time_s if _finite(packet_time_s) else rcv_time
        # 앞선 레코드를 미래 batch 수신 시각으로 미리 만료시키지 않는다.
        self._receive_time_s = step_time_s
        if not frames:
          retain(parent_step(step_time_s, [(timestamp_ns, [])]))
        for frame in frames:
          retain(parent_step(step_time_s, [(timestamp_ns, [frame])]))
      self._receive_time_s = rcv_time
      retain(parent_step(rcv_time, []))
    finally:
      self._inside_carrot_update = False
    return result if result is not None else self._waiting_result(rcv_time)

  def update(self, can_packets):
    """(수신 나노초, CAN 프레임 목록) 입력만 읽고 CAN 송신은 하지 않는다."""
    result = None
    direct = not self._inside_carrot_update
    if direct:
      self._outer_frame += 1
    now_s = self._receive_time_s
    for timestamp_ns, frames in can_packets:
      time_s = _packet_time(timestamp_ns)
      if not _finite(time_s) or (self._packet_time_s is not None and time_s < self._packet_time_s):
        self._counters["invalid_packet_time"] += 1
        self._input_error()
        self._packet_time_s = time_s if _finite(time_s) else None
        result = self._unavailable_result(can_error=True)
        continue
      self._packet_time_s = time_s
      now_s = time_s if now_s is None else max(now_s, time_s)
      if time_s == self._blocked_packet_time_s:
        result = self._unavailable_result(can_error=True)
        continue
      self._blocked_packet_time_s = None
      if self._record_time_s is not None and time_s - self._record_time_s > RADAR_TIMEOUT_S:
        self._counters["timeouts"] += 1
        self._reset_history()
        result = self._unavailable_result()
      for address, data, src in frames:
        if self._blocked_packet_time_s == time_s:
          break
        support_frame = (
          src == FUSED_CONFIG.ego_speed_bus and address in (TOYOTA_SPEED_ADDR, TOYOTA_KINEMATICS_ADDR)
        ) or (
          src == FUSED_CONFIG.radar_bus and address in (ACC_TARGET_POS_ADDR, ACC_TARGET_VREL_ADDR, *SUMMARY_ADDRS)
        )
        object_frame = src == FUSED_CONFIG.radar_bus and address == ID80_ADDR
        if not support_frame and not object_frame:
          continue
        if not isinstance(data, (bytes, bytearray, memoryview)):
          self._counters["invalid_frame_data"] += 1
          result = self._input_error()
          continue
        # 범위 요약은 원형 디코더가 4바이트 이상을 받는다. 실제 로그도 DLC4다.
        summary_frame = src == FUSED_CONFIG.radar_bus and address in SUMMARY_ADDRS
        invalid_support_length = not 4 <= len(data) <= 8 if summary_frame else len(data) != 8
        if support_frame and invalid_support_length:
          self._counters["support_frame_length"] += 1
          result = self._input_error()
          continue
        if src == FUSED_CONFIG.ego_speed_bus and address == TOYOTA_SPEED_ADDR:
          # SPEED의 원형 DBC에 정의된 Toyota CHECKSUM만 검증한다.
          speed_data = bytes(data)
          if len(speed_data) != 8:
            self._counters["support_frame_length"] += 1
            result = self._input_error()
            continue
          if toyota_checksum(address, None, bytearray(speed_data)) != speed_data[-1]:
            self._counters["support_checksum_errors"] += 1
            result = self._input_error()
            continue
        if object_frame:
          self._counters["received_id80_frames"] += 1
        assembler = self.decoder._assembler
        rejected_before = assembler.rejections
        crc_before = self.decoder.crc_failures
        complete_before = assembler.counters["completed_records"]
        try:
          decoded = self.decoder.update_frame(time_s, src, address, bytes(data))
        except ValueError:
          # 동일 시각의 상충하는 객체 나이 등은 원형 lifecycle의 거절을 유지한다.
          ambiguous = assembler.counters["completed_records"] > complete_before and time_s == self._completion_time_s
          self._counters["ambiguous_batch_time" if ambiguous else "invalid_native_record"] += 1
          self._input_error()
          if ambiguous:
            self._blocked_packet_time_s = time_s
          result = self._unavailable_result(can_error=True)
          continue
        if assembler.rejections > rejected_before or self.decoder.crc_failures > crc_before:
          result = self._input_error()
          continue
        # 공통 길이·체크섬·순서·CRC를 통과한 동일 프레임만 표시 디코더에도 전달한다.
        self._update_display_frame(time_s, src, address, bytes(data), decoded)
        if decoded is not None:
          record_time_s = decoded["time_s"]
          if (time_s == self._completion_time_s
              or (self._record_time_s is not None and record_time_s == self._record_time_s)):
            # 시작 또는 완료 시각이 같은 복수 record에 임의 측정 주기를 만들지 않는다.
            self._counters["ambiguous_batch_time"] += 1
            self._input_error()
            self._blocked_packet_time_s = time_s
            result = self._unavailable_result(can_error=True)
            continue
          if not _finite(record_time_s) or (self._record_time_s is not None and record_time_s < self._record_time_s):
            self._counters["invalid_native_record"] += 1
            result = self._input_error()
            continue
          self._completion_time_s = time_s
          self._record_time_s = record_time_s
          self._seen_record = True
          self._fault_latched = False
          self._unavailable = False
          result = self._convert(decoded)

    if self._record_time_s is not None and now_s is not None and now_s - self._record_time_s > RADAR_TIMEOUT_S:
      self._counters["timeouts"] += 1
      self._reset_history()
      result = self._unavailable_result()
    if self._unavailable and result is None:
      self._counters["no_record_updates"] += 1
    return result if result is not None or not direct else self._waiting_result(now_s)
