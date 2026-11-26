from pathlib import Path
import re
import sys
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
CLUSTER_DIR = REPO_ROOT / "openpilot" / "selfdrive" / "carrot" / "cluster"
if str(CLUSTER_DIR) not in sys.path:
  sys.path.insert(0, str(CLUSTER_DIR))

from opendbc.car import Bus, structs
from opendbc.can.packer import CANPacker
from opendbc.car.hyundai import carstate as hyundai_carstate
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.carrot.carrot_functions import CarrotPlanner
from openpilot.selfdrive.carrot.carrot_serv import CarrotServ
from openpilot.selfdrive.carrot.deceleration_source import (
  deceleration_source_presentation,
  format_vehicle_navi_junction,
)
from openpilot.selfdrive.carrot.realtime.compact_state import SERVICE_SCHEMAS
from openpilot.selfdrive.carrot.tests.test_vehicle_speed_camera_control import _car_state as _serv_cs
from openpilot.selfdrive.carrot.tests.test_vehicle_speed_camera_control import _serv as _make_serv
from openpilot.selfdrive.carrot.tests.test_cluster_scene import LIGHT_CLUSTER_THEME, _cluster_state
import cluster_renderer


class _FakeNaviParams:
  def __init__(self, can_control=1, school_zone=False, ramp_control=True, curve_factor=100, ctrl_end=3):
    self.can_control = can_control
    self.school_zone = school_zone
    self.ramp_control = ramp_control
    self.curve_factor = curve_factor
    self.ctrl_end = ctrl_end

  def get_int(self, key):
    return {
      "HyundaiCameraSCC": 0,
      "AutoEngage": 0,
      "VehicleSpeedCameraDistanceTime": 60,
      "VehicleNaviCanControl": int(self.can_control),
      "AutoNaviSpeedCtrlMode": 2,
      "AutoNaviSpeedSafetyFactor": 100,
      "AutoNaviSpeedBumpSpeed": 35,
      "AutoNaviSpeedBumpTime": 1,
      "AutoNaviSpeedBumpEndDistance": 200,
      "AutoNaviSpeedDecelRate": 120,
      "CruiseButtonLongDelay": 40,
      "VehicleNaviCurveSpeedFactor": self.curve_factor,
      "AutoCurveSpeedLowerLimit": 30,
      "VehicleNaviCurveCtrlEnd": self.ctrl_end,
    }.get(key, 0)

  def get_bool(self, key):
    if key == "VehicleNaviRampCurveControl":
      return self.ramp_control
    if key == "VehicleNaviSchoolZoneControl":
      return self.school_zone
    return key in ("ControlsReady", "RoundaboutLaneChangeHold", "TunnelLaneChangeHold")

  def put_bool(self, key, value):
    pass

  def get(self, key):
    return repr({k: [] for k in range(-4, 8)}) if key == "FingerPrints" else ""


def _make_hyundai_state(monkeypatch, fingerprint, *, ramp_control=True, can_control=1):
  fake_params = _FakeNaviParams(can_control=can_control, ramp_control=ramp_control)
  monkeypatch.setattr(hyundai_carstate, "Params", lambda: fake_params)
  CP = structs.CarParams()
  CP.carFingerprint = fingerprint
  CP.flags = int(HyundaiFlags.CANFD)
  CS = CarState(CP)
  CS.controls_ready_count = 122
  parsers = CS.get_can_parsers_canfd(CP)
  CS.cp = parsers[Bus.pt]
  CS.cp_cam = parsers[Bus.cam]
  CS.cp_alt = parsers.get(Bus.alt)
  return CS, parsers, CANPacker("hyundai_canfd_generated")


def _segment_values(calc_route, path_idx=8, road_class=1):
  raw = (800 & 0x1FFF) | ((path_idx & 0x3F) << 13) | ((calc_route & 0x3) << 22) | ((road_class & 0x7) << 24)
  return {f"BYTE_{i + 1}": (raw >> (i * 8)) & 0xFF for i in range(8)}


def _step(CS, parsers, t_s, frames=(), *, v_ego=25.0, left_blinker=False, right_blinker=False):
  ts_ns = int(round(t_s * 1e9))
  bus_msgs = {bus: [] for bus in parsers}
  for bus, dat in frames:
    bus_msgs[bus].append(dat)
  for bus, parser in parsers.items():
    parser.update([(ts_ns, bus_msgs[bus])])
  CS.monitor_fingerprint(parsers, True)
  CS.totalDistance += v_ego * 0.01
  ret = SimpleNamespace(
    vEgo=v_ego,
    speedLimit=100.0,
    leftBlinker=left_blinker,
    rightBlinker=right_blinker,
  )
  CS._update_vehicle_navi_events(parsers[Bus.pt], ret, False, parsers.get(Bus.alt))
  return ret


def test_compact_state_carrot_man_schema_synced_across_python_cpp_js():
  cpp_text = (REPO_ROOT / "openpilot" / "selfdrive" / "carrot" / "realtime" / "compact_state_native.cc").read_text()
  js_text = (REPO_ROOT / "openpilot" / "selfdrive" / "carrot" / "web" / "js" / "realtime" / "vision_compact.js").read_text()

  cpp_match = re.search(r"void encode_carrot_man\([^\)]*\)\s*\{(.*?)\n\}", cpp_text, re.DOTALL)
  assert cpp_match is not None
  cpp_fields = re.findall(r'append_(\w+)\(out,\s*value,\s*"([^"]+)"\)', cpp_match.group(1))
  cpp_norm = [(name, typ) for typ, name in cpp_fields]

  js_match = re.search(r'\[\s*5\s*,\s*\["carrotMan"\s*,\s*\[(.*?)\]\s*\]\s*\]', js_text, re.DOTALL)
  assert js_match is not None
  js_fields = re.findall(r'\["([^"]+)"\s*,\s*"([^"]+)"\]', js_match.group(1))

  py_fields = list(SERVICE_SCHEMAS["carrotMan"][1])
  assert py_fields == cpp_norm == js_fields
  assert ("vehicleNaviCurveCurrentSpeed", "f32") in py_fields
  assert ("vehicleNaviCurveIsRamp", "bool") in py_fields
  assert ("vehicleNaviJunctionType", "u8") in py_fields
  assert ("vehicleNaviJunctionSide", "u8") in py_fields
  assert ("vehicleNaviJunctionDistance", "i32") in py_fields


@pytest.mark.parametrize(
  ("raw_v0", "expected_sign", "expected_radius"),
  (
    (611, 1.0, 440.0),
    (411, -1.0, 440.0),
    (521, 1.0, 44.0),
    (501, -1.0, 44.0),
  ),
)
def test_decode_hyundai_navi_curvature_scale_4_4_m_per_count(raw_v0, expected_sign, expected_radius):
  curv = CarState._decode_hyundai_navi_curvature(raw_v0)
  assert curv == pytest.approx(expected_sign / expected_radius)


@pytest.mark.parametrize("sentinel", (-1, 511, 1023, 1024))
def test_decode_hyundai_navi_curvature_rejects_sentinels(sentinel):
  assert CarState._decode_hyundai_navi_curvature(sentinel) is None


def test_ev4_curve_linkclass_gate_and_recalculation_reset(monkeypatch):
  CS, parsers, packer = _make_hyundai_state(monkeypatch, CAR.KIA_EV4)
  bus = parsers[Bus.pt].bus
  seg_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(calc_route=0, path_idx=8)))
  hda_hwy = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1, "Frwinfo": 0}))
  curve_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_PROSHORT_094", bus, {
    "PROSHORT_OFFSET": 200, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 20,
    "PROSHORT_VALUE_0": 521, "PROSHORT_VALUE_1": 0, "PROSHORT_PROFILE_TYPE": 1,
  }))
  ret = _step(CS, parsers, 1.0, [seg_frame, hda_hwy, curve_frame])
  assert ret.vehicleNaviCurveRoadAllowed is True
  assert ret.vehicleNaviCurveRouteState == 0
  assert ret.vehicleNaviCurveIsRamp is False
  assert ret.vehicleNaviCurveCurvature == pytest.approx(1.0 / 44.0)
  assert ret.vehicleNaviCurveSpeed == pytest.approx(CarState._vehicle_navi_curve_reference_speed(1.0 / 44.0))

  # Switching to local road (LinkClass=0) must block curve control immediately.
  hda_local = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 0, "Frwinfo": 0}))
  ret_local = _step(CS, parsers, 1.1, [hda_local])
  assert ret_local.vehicleNaviCurveRoadAllowed is False
  assert ret_local.vehicleNaviCurveSpeed == 0.0

  # Back on highway -> cached curve still valid until CalculatedRoute == 2 resets it.
  ret_back = _step(CS, parsers, 1.2, [hda_hwy])
  assert ret_back.vehicleNaviCurveSpeed > 0.0

  seg_recalc = (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(calc_route=2, path_idx=8)))
  ret_reset = _step(CS, parsers, 1.3, [seg_recalc, hda_hwy])
  assert ret_reset.vehicleNaviCurveRouteState == 2
  assert ret_reset.vehicleNaviCurveSpeed == 0.0
  assert CS.vehicleNaviCurves == []


def test_ev4_094_simultaneous_section_taper_curve_and_conditional_098_ramp(monkeypatch):
  CS, parsers, packer = _make_hyundai_state(monkeypatch, CAR.KIA_EV4, can_control=1)
  bus = parsers[Bus.pt].bus
  seg_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(calc_route=0, path_idx=8)))
  hda_fork = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1, "Frwinfo": 2}))
  ramp_curve = (Bus.pt, packer.make_can_msg("CANFD_NAVI_RAMP_098", bus, {
    "PROSHORT_OFFSET": 250, "PROSHORT_PATH_INDEX": 9, "PROSHORT_DISTANCE": 5,
    "PROSHORT_VALUE_0": 521, "PROSHORT_VALUE_1": 0, "PROSHORT_PROFILE_TYPE": 1,
  }))
  stub_right = (Bus.pt, packer.make_can_msg("CANFD_NAVI_STUB_09A", bus, {
    "STUB_OFFSET": 240, "STUB_PATH_INDEX": 8, "STUB_BRANCH_TYPE": 0x49, "STUB_TURN_ANGLE": 16.875,
  }))
  taper_diverge = (Bus.pt, packer.make_can_msg("CANFD_NAVI_PROSHORT_094", bus, {
    "PROSHORT_OFFSET": 150, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 293,
    "PROSHORT_VALUE_0": 1, "PROSHORT_VALUE_1": (6 << 1), "PROSHORT_PROFILE_TYPE": 16,
  }))

  # 1. Driving straight past exit with no blinker -> 0x098 MUST NOT arm.
  ret_straight = _step(CS, parsers, 1.0, [seg_frame, hda_fork, ramp_curve, stub_right, taper_diverge])
  assert ret_straight.vehicleNaviCurveSpeed == 0.0
  assert ret_straight.vehicleNaviCurveIsRamp is False
  assert ret_straight.vehicleNaviJunctionType == 1
  assert ret_straight.vehicleNaviJunctionSide == 2
  assert ret_straight.vehicleNaviJunctionLane == 6

  # 2. Hazard flashers (both blinkers) or left overtake blinker -> 0x098 MUST NOT arm.
  assert _step(CS, parsers, 1.1, left_blinker=True, right_blinker=True).vehicleNaviCurveSpeed == 0.0
  assert _step(CS, parsers, 1.2, left_blinker=True, right_blinker=False).vehicleNaviCurveSpeed == 0.0

  # 3. Single right blinker matching STUB right exit + Frwinfo=2 -> 0x098 arms cleanly!
  ret_right = _step(CS, parsers, 1.3, left_blinker=False, right_blinker=True)
  assert ret_right.vehicleNaviCurveIsRamp is True
  assert ret_right.vehicleNaviCurveSpeed > 0.0

  # 4. Left-side exit (negative turn angle) -> left blinker arms, right does not.
  stub_left = (Bus.pt, packer.make_can_msg("CANFD_NAVI_STUB_09A", bus, {
    "STUB_OFFSET": 220, "STUB_PATH_INDEX": 8, "STUB_BRANCH_TYPE": 0x49, "STUB_TURN_ANGLE": -19.6875,
  }))
  assert _step(CS, parsers, 1.4, [stub_left], left_blinker=False, right_blinker=True).vehicleNaviCurveIsRamp is False
  assert _step(CS, parsers, 1.5, left_blinker=True, right_blinker=False).vehicleNaviCurveIsRamp is True

  # 5. EV4 section (ptype=16 v0=0) still latches cleanly right alongside curve/junction traffic.
  section_msg = (Bus.pt, packer.make_can_msg("CANFD_NAVI_PROSHORT_094", bus, {
    "PROSHORT_OFFSET": 100, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 50,
    "PROSHORT_VALUE_0": 0, "PROSHORT_VALUE_1": (19 << 5), "PROSHORT_PROFILE_TYPE": 16,
  }))
  _step(CS, parsers, 1.6, [section_msg])
  assert CS.ev4Section is not None and CS.ev4Section["speed"] == 90


def test_pv5_curve_keeps_camera_route_exclusion_and_legacy_4ba_4bf_works(monkeypatch):
  pv5_cs, pv5_parsers, packer = _make_hyundai_state(monkeypatch, CAR.KIA_PV5)
  bus = pv5_parsers[Bus.pt].bus
  seg_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_SEGMENT_099", bus, _segment_values(calc_route=1, path_idx=8)))
  hda_hwy = (Bus.pt, packer.make_can_msg("CANFD_HDA_INFO_364", bus, {"LinkClass": 1, "Frwinfo": 0}))
  curve_frame = (Bus.pt, packer.make_can_msg("CANFD_NAVI_PROSHORT_094", bus, {
    "PROSHORT_OFFSET": 200, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 20,
    "PROSHORT_VALUE_0": 521, "PROSHORT_VALUE_1": 0, "PROSHORT_PROFILE_TYPE": 1,
  }))
  ret_pv5 = _step(pv5_cs, pv5_parsers, 1.0, [seg_frame, hda_hwy, curve_frame])
  assert pv5_cs.navi_segment_4b9 is None
  assert ret_pv5.speedBumpDistance == 0.0
  assert ret_pv5.vehicleNaviCurveSpeed > 0.0

  # Legacy CAN-FD (Ioniq 5): 0x4BA + 0x4B9 + 0x4BF + 0x4A3
  leg_cs, leg_parsers, _ = _make_hyundai_state(monkeypatch, CAR.HYUNDAI_IONIQ_5)
  leg_bus = leg_parsers[Bus.pt].bus
  if "HDA_INFO_4A3" not in leg_parsers[Bus.pt].vl:
    leg_parsers[Bus.pt]._add_message("HDA_INFO_4A3", float("nan"))
  leg_cs.hda_info_4a3 = leg_parsers[Bus.pt].vl["HDA_INFO_4A3"]
  ret_leg = _step(leg_cs, leg_parsers, 1.0, [
    (Bus.pt, packer.make_can_msg("NEW_MSG_4B9", leg_bus, _segment_values(calc_route=1, path_idx=8))),
    (Bus.pt, packer.make_can_msg("HDA_INFO_4A3", leg_bus, {"LinkClass": 1, "Frwinfo": 2})),
    (Bus.pt, packer.make_can_msg("NEW_MSG_4BA", leg_bus, {
      "PROSHORT_OFFSET": 220, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 20,
      "PROSHORT_VALUE_0": 526, "PROSHORT_VALUE_1": 0, "PROSHORT_PROFILE_TYPE": 1,
    })),
    (Bus.pt, packer.make_can_msg("NEW_MSG_4BF", leg_bus, {
      "STUB_OFFSET": 300, "STUB_PATH_INDEX": 8, "STUB_BRANCH_TYPE": 0x01, "STUB_TURN_ANGLE": -158.9,
    })),
  ])
  assert ret_leg.vehicleNaviCurveSpeed > 0.0
  assert ret_leg.vehicleNaviJunctionType == 2
  assert ret_leg.vehicleNaviJunctionSide == 1


def test_carrot_serv_curve_gates_scaling_and_external_nav_priority():
  serv = _make_serv(1)
  serv.vehicleNaviCurveControl = False
  serv.vehicleNaviCurveMppControl = True
  serv.vehicleNaviRampCurveControl = True
  serv.vehicleNaviCurveSpeedFactor = 1.0
  serv.vehicleNaviCurveControlEnd = 3.0
  serv.autoCurveSpeedLowerLimit = 30
  serv.external_navigation_active = False

  CS = _serv_cs()
  CS.vehicleNaviCurveRoadAllowed = True
  CS.vehicleNaviCurveRouteState = 0
  CS.vehicleNaviCurveRouteActive = False
  CS.vehicleNaviCurveDistance = 120.0
  CS.vehicleNaviCurveSpeed = 55.0
  CS.vehicleNaviCurveCurvature = 1.0 / 60.0
  CS.vehicleNaviCurveIsRamp = True
  CS.vehicleNaviJunctionType = 1
  CS.vehicleNaviJunctionSide = 2
  CS.vehicleNaviJunctionDistance = 180.0

  # Opt-in off: speed control inactive (250), diagnostic display active
  assert serv._vehicle_navi_curve_speed(CS) == 250.0
  assert 55.0 <= serv._vehicle_navi_curve_display_speed(CS) < 250.0

  # Opt-in on + scaling + ramp source label
  serv.vehicleNaviCurveControl = True
  expected = serv.calculate_current_speed(120.0, 55.0, 3.0, serv.autoNaviSpeedDecelRate)
  assert serv._vehicle_navi_curve_speed(CS) == pytest.approx(expected)
  assert deceleration_source_presentation("hda_ramp") == ("ramp", 3)
  assert deceleration_source_presentation("hda_curve") == ("curve", 3)

  # Ramp gate off -> ignores ramp curve
  serv.vehicleNaviRampCurveControl = False
  assert serv._vehicle_navi_curve_speed(CS) == 250.0
  serv.vehicleNaviRampCurveControl = True

  # Local road gate (RoadAllowed=False) -> blocked
  CS.vehicleNaviCurveRoadAllowed = False
  assert serv._vehicle_navi_curve_speed(CS) == 250.0
  assert serv._vehicle_navi_curve_display_speed(CS) == 0.0
  CS.vehicleNaviCurveRoadAllowed = True

  # External navigation active -> supersedes both curve and junction display
  serv.external_navigation_active = True
  assert serv._vehicle_navi_curve_speed(CS) == 250.0
  assert serv._vehicle_navi_curve_display_speed(CS) == 0.0
  assert serv._vehicle_navi_junction_display(CS) == (0, 0, 0)


def test_carrot_planner_junction_accel_smoothing():
  planner = object.__new__(CarrotPlanner)
  planner.cruiseMaxVals0 = 1.6
  planner.cruiseMaxVals1 = 1.4
  planner.cruiseMaxVals2 = 1.2
  planner.cruiseMaxVals3 = 1.1
  planner.cruiseMaxVals4 = 1.0
  planner.cruiseMaxVals5 = 0.9
  planner.cruiseMaxVals6 = 0.8
  planner.myDrivingMode = 3
  planner.myHighModeFactor = 1.0
  planner.mySafeFactor = 1.0

  planner.vehicle_navi_junction_assist = False
  planner.vehicle_navi_junction_accel_limited = False
  assert planner.get_carrot_accel(25.0) > 0.8

  planner.vehicle_navi_junction_accel_limited = True
  assert planner.get_carrot_accel(25.0) == pytest.approx(0.8)


def test_hud_and_cluster_render_ramp_and_junction_badges(monkeypatch):
  assert format_vehicle_navi_junction(1, 2, 220) == "FORK -> 220m"
  assert format_vehicle_navi_junction(2, 1, 0) == "<- MERGE NOW"
  assert format_vehicle_navi_junction(0, 2, 100) == ""

  renderer = object.__new__(cluster_renderer.ClusterUiRenderer)
  renderer._current_theme = lambda: LIGHT_CLUSTER_THEME
  renderer._speed_bg_texture = None
  renderer.is_metric = True
  draws = []
  renderer._draw_text_with_stroke = lambda *args, **kwargs: draws.append(args)

  renderer._draw_speed_block(_cluster_state(
    vehicle_navi_curve_speed_kph=72.0,
    vehicle_navi_curve_is_ramp=True,
    vehicle_navi_junction_badge="FORK -> 220m",
  ))
  texts = [d[0] for d in draws]
  assert "ramp" in texts
  assert "72" in texts
  assert "FORK -> 220m" in texts


def test_gv80_4b7_curve_4b5_stub_and_4b4_reset(monkeypatch):
  CS, parsers, packer = _make_hyundai_state(monkeypatch, CAR.GENESIS_GV80)
  bus = parsers[Bus.pt].bus
  # Real GV80 0x4B5 exit stub (b3=0x49, right turn, guidance active bit 0x08 in byte 4, frc=1)
  stub_4b5_dat = bytes.fromhex("29073949c9fc1500")
  frames = [
    (Bus.pt, packer.make_can_msg("HDA_INFO_4A3", bus, {"LinkClass": 1, "Frwinfo": 2, "SPEED_LIMIT": 100})),
    (Bus.pt, (0x4B5, stub_4b5_dat, bus)),
    (Bus.pt, packer.make_can_msg("NEW_MSG_4B7", bus, {
      "PROSHORT_OFFSET": 220, "PROSHORT_PATH_INDEX": 8, "PROSHORT_DISTANCE": 15,
      "PROSHORT_VALUE_0": 511 + 60, "PROSHORT_VALUE_1": 1023, "PROSHORT_PROFILE_TYPE": 1,
    })),
  ]
  ret = _step(CS, parsers, 1.0, frames, v_ego=28.0)
  assert ret.vehicleNaviCurveRoadAllowed is True
  assert ret.vehicleNaviCurveRouteState in (0, 1)
  assert ret.vehicleNaviCurveDistance == pytest.approx(220.0)
  assert 60.0 < ret.vehicleNaviCurveTargetSpeed < 110.0
  assert ret.vehicleNaviCurveIsRamp is False
  assert ret.vehicleNaviJunctionType == 1

  # Real GV80 0x4B4 path_index=0 reset clears active curves
  ret_reset = _step(CS, parsers, 1.5, [
    (Bus.pt, packer.make_can_msg("HDA_INFO_4A3", bus, {"LinkClass": 1, "Frwinfo": 0, "SPEED_LIMIT": 100})),
    (Bus.pt, (0x4B4, bytes.fromhex("0000000100000000"), bus)),
  ], v_ego=28.0)
  assert ret_reset.vehicleNaviCurveTargetSpeed == 0.0


def test_gv80_4b0_conditional_ramp_with_frwinfo_and_blinker(monkeypatch):
  CS, parsers, packer = _make_hyundai_state(monkeypatch, CAR.GENESIS_GV80, ramp_control=True)
  bus = parsers[Bus.pt].bus
  # Activate path=8 via 0x4B5, send 0x4B0 sharp right ramp curve (path=9, v0=511-20=491 -> R~=88m)
  frames = [
    (Bus.pt, packer.make_can_msg("HDA_INFO_4A3", bus, {"LinkClass": 1, "Frwinfo": 2, "SPEED_LIMIT": 100})),
    (Bus.pt, (0x4B5, bytes.fromhex("29073949c9101500"), bus)),
    (Bus.pt, packer.make_can_msg("NEW_MSG_4B0", bus, {
      "PROSHORT_OFFSET": 200, "PROSHORT_PATH_INDEX": 9, "PROSHORT_DISTANCE": 5,
      "PROSHORT_VALUE_0": 511 - 20, "PROSHORT_VALUE_1": 1023, "PROSHORT_PROFILE_TYPE": 1,
    })),
  ]
  # Without blinker -> ramp curve must stay suppressed on mainline
  ret_straight = _step(CS, parsers, 1.0, frames, v_ego=28.0, right_blinker=False)
  assert ret_straight.vehicleNaviCurveIsRamp is False
  assert ret_straight.vehicleNaviCurveTargetSpeed == 0.0

  # With matching right blinker + Frwinfo=2 -> ramp curve activates
  ret_exit = _step(CS, parsers, 1.2, [
    (Bus.pt, packer.make_can_msg("HDA_INFO_4A3", bus, {"LinkClass": 1, "Frwinfo": 2, "SPEED_LIMIT": 100})),
  ], v_ego=28.0, right_blinker=True)
  assert ret_exit.vehicleNaviCurveIsRamp is True
  assert 35.0 < ret_exit.vehicleNaviCurveTargetSpeed < 70.0

