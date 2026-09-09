import re
import unittest

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.selected_car import get_selected_car_platform
from opendbc.car.structs import CarParams
from opendbc.car.tesla.carstate import CarState, STOCK_ACC_CANCEL_PULSE_FRAMES, STOCK_ACC_CANCEL_STATES
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR, CANBUS, TeslaFlags, TeslaSafetyFlags

Ecu = CarParams.Ecu

# Fields prefixed unknown_* we observe structurally but don't know the meaning of.
# Only `platform` has evidence-backed semantic meaning (matches car_model in FW_VERSIONS).
#
# unknown_prefix is everything before the comma; we don't split it because we don't know what its
# parts mean, but observed shape is: <family>_<package>_<triplet> (<build>), e.g.
#   TeMYG4 _ Main     _ 0.0.0 (78)     or     TeM3 _ SP_XP002p2 _ 0.0.0 (23)
#   family   package    triplet build           family  package    triplet build
#
# After the comma, the version string decomposes into:
#   platform             : E/Y/X = car model (Model 3 / Y / X). The only field with known meaning.
#   variant_code         : differentiator WITHIN a platform — hardware/trim/calibration bits packed
#                          into <digit?><letters?><3-digit series>, e.g. '4HP015', '4003', 'L014'.
#                          We don't fully know what the parts mean individually, but the
#                          whole string identifies a specific variant within the car model.
#   software_major/minor : numeric components after the first '.' — conventional release numbers.
#                          minor is optional (e.g. 'E4S014.27' has no minor).
#
# Suspected (not confirmed): for M3/MY, `TeM3_*` outer + no-leading-digit variant_code == HW3, and
# `TeMYG4_*` outer + leading-'4' variant_code == HW4 (the 'G4' in TeMYG4 likely denotes Gen 4).
#
# Example full parse of 'TeMYG4_Main_0.0.0 (78),E4HP015.05.0':
#   unknown_prefix='TeMYG4_Main_0.0.0 (78)'
#   platform=E  variant_code=4HP015  software_major=05  software_minor=0
FW_RE = re.compile(
  rb'^(?P<unknown_prefix>.+),' +
  rb'(?P<platform>[EYX])' +
  rb'(?P<variant_code>\d?[A-Z]*\d{3})' +
  rb'\.(?P<software_major>\d+)' +
  rb'(?:\.(?P<software_minor>\d+))?$'
)

PLATFORM_TO_CAR = {
  b'E': CAR.TESLA_MODEL_3,
  b'Y': CAR.TESLA_MODEL_Y,
  b'X': CAR.TESLA_MODEL_X,
}


class TestTeslaFingerprint(unittest.TestCase):
  def test_model_3_and_y_manual_selection_enables_control(self):
    for candidate in (CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y):
      for doc in candidate.config.car_docs:
        self.assertEqual(get_selected_car_platform(doc.name), candidate)

      CP = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], False, False, False)
      self.assertFalse(CP.dashcamOnly)
      self.assertIsNotNone(CarInterface.CarController)

  def test_brake_status_uses_common_esp_message(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, gen_empty_fingerprint(), [], False, False, False)
    car_state = CarState(CP)
    can_parsers = CarState.get_can_parsers(CP)

    car_state.update(can_parsers)

    self.assertIn(0x145, can_parsers[Bus.party].addresses)
    self.assertNotIn(0x39D, can_parsers[Bus.party].addresses)

  def test_sccm_validity_zero_does_not_invalidate_vehicle_sensors(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, gen_empty_fingerprint(), [], False, False, False)
    car_state = CarState(CP)
    can_parsers = CarState.get_can_parsers(CP)
    can_parsers[Bus.ap_party].vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleValidity"] = 0

    ret = car_state.update(can_parsers)

    self.assertFalse(ret.vehicleSensorsInvalid)

  def test_standstill_uses_esp_not_cruise_state(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, gen_empty_fingerprint(), [], False, False, False)
    car_state = CarState(CP)
    can_parsers = CarState.get_can_parsers(CP)

    can_parsers[Bus.party].vl["DI_state"]["DI_cruiseState"] = 0  # UNAVAILABLE
    can_parsers[Bus.party].vl["ESP_B"]["ESP_vehicleStandstillSts"] = 1
    self.assertTrue(car_state.update(can_parsers).standstill)

    can_parsers[Bus.party].vl["DI_state"]["DI_cruiseState"] = 3  # STANDSTILL
    can_parsers[Bus.party].vl["ESP_B"]["ESP_vehicleStandstillSts"] = 0
    self.assertFalse(car_state.update(can_parsers).standstill)

  def test_stock_tacc_does_not_trigger_autosteer_conflict(self):
    fingerprint = gen_empty_fingerprint()
    fingerprint[CANBUS.autopilot_party][0x293] = 8  # DAS_settings
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, fingerprint, [], False, False, False)
    car_state = CarState(CP)
    can_parsers = CarState.get_can_parsers(CP)

    # Normal stock TACC reports ACTIVE_NOMINAL. It must not be treated as
    # Autosteer/FSD, otherwise the stalk's pcmEnable event is rejected.
    can_parsers[Bus.ap_party].vl["DAS_status"]["DAS_autopilotState"] = 3
    can_parsers[Bus.ap_party].vl["DAS_settings"]["DAS_autosteerEnabled"] = 0
    self.assertFalse(car_state.update(can_parsers).invalidLkasSetting)

    can_parsers[Bus.ap_party].vl["DAS_settings"]["DAS_autosteerEnabled"] = 1
    self.assertTrue(car_state.update(can_parsers).invalidLkasSetting)

  def test_idle_stock_acc_state_does_not_cancel_new_engagement(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, gen_empty_fingerprint(), [], True, False, False)
    car_state = CarState(CP)
    can_parsers = CarState.get_can_parsers(CP)
    das_control = can_parsers[Bus.ap_party].vl["DAS_control"]

    # Real vehicles continuously emit ACC_CANCEL_GENERIC (0) while ACC is
    # unavailable. Treating that idle value as an active cancellation makes CP
    # transmit ACC_CANCEL_GENERIC_SILENT and abort a stalk engagement.
    das_control["DAS_accState"] = 0
    car_state.update(can_parsers)
    self.assertFalse(car_state.das_accCancel)
    car_state.update(can_parsers)
    self.assertFalse(car_state.das_accCancel)

    # A transition away from active stock ACC is a real cancellation and must
    # still be forwarded briefly.
    das_control["DAS_accState"] = 4
    car_state.update(can_parsers)
    self.assertFalse(car_state.das_accCancel)
    das_control["DAS_accState"] = 0
    car_state.update(can_parsers)
    self.assertTrue(car_state.das_accCancel)

  def test_stock_acc_cancel_pulse_is_bounded_and_includes_hold(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, gen_empty_fingerprint(), [], True, False, False)
    for active_state in (3, 4):
      for cancel_state in STOCK_ACC_CANCEL_STATES:
        with self.subTest(active_state=active_state, cancel_state=cancel_state):
          car_state = CarState(CP)
          parsers = CarState.get_can_parsers(CP)
          das = parsers[Bus.ap_party].vl["DAS_control"]
          das["DAS_accState"] = active_state
          car_state.update(parsers)
          das["DAS_accState"] = cancel_state
          events = []
          for frame in range(STOCK_ACC_CANCEL_PULSE_FRAMES + 5):
            ret = car_state.update(parsers)
            self.assertEqual(car_state.das_accCancel, frame < STOCK_ACC_CANCEL_PULSE_FRAMES)
            events.extend((event.type, event.pressed) for event in ret.buttonEvents)
          cancel = structs.CarState.ButtonEvent.Type.cancel
          self.assertEqual(events, [(cancel, True), (cancel, False)])

  def test_cancel_pulse_reaches_each_longitudinal_send_phase(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, gen_empty_fingerprint(), [], True, False, False)
    CC = structs.CarControl(enabled=True, longActive=True).as_reader()
    for phase in range(STOCK_ACC_CANCEL_PULSE_FRAMES):
      with self.subTest(phase=phase):
        car_state = CarState(CP)
        parsers = CarState.get_can_parsers(CP)
        controller = CarInterface.CarController({Bus.party: "tesla_model3_party"}, CP)
        controller.frame = phase
        das = parsers[Bus.ap_party].vl["DAS_control"]
        das["DAS_accState"] = 3
        car_state.update(parsers)
        das["DAS_accState"] = 0
        commands = []
        for frame in range(STOCK_ACC_CANCEL_PULSE_FRAMES):
          car_state.out = car_state.update(parsers)
          _, sends = controller.update(CC, car_state, (frame + 1) * 10_000_000)
          commands.extend(data[1] >> 4 for address, data, _ in sends if address == 0x2B9)
        self.assertEqual(commands, [13])

  def test_fw_platform_code(self):
    # Every EPS FW must parse and its platform letter must match the car it's filed under.
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)

        assert m is not None, f"Unparsable FW: {fw}"
        assert PLATFORM_TO_CAR[m['platform']] == car_model, f"Platform letter {m['platform']!r} != {car_model.value}: {fw}"

  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_no_radar_car(self):
    # When radar DBC is available but no radar signal present, should report unavailable
    fingerprint = gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
    assert CP.radarUnavailable  # No radar signal -> unavailable

  def test_auto_speed_limit_requires_longitudinal_and_vehicle_bus(self):
    fingerprint = gen_empty_fingerprint()
    fingerprint[CANBUS.vehicle][0x3DF] = 8

    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], True, False, False)
    assert CP.flags & TeslaFlags.HAS_VEHICLE_BUS
    assert CP.flags & TeslaFlags.AUTO_SPEED_LIMIT
    assert CP.safetyConfigs[0].safetyParam & TeslaSafetyFlags.AUTO_SPEED_LIMIT

    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
    assert CP.flags & TeslaFlags.HAS_VEHICLE_BUS
    assert not (CP.flags & TeslaFlags.AUTO_SPEED_LIMIT)
    assert not (CP.safetyConfigs[0].safetyParam & TeslaSafetyFlags.AUTO_SPEED_LIMIT)

    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], True, False, False)
    assert not (CP.flags & TeslaFlags.AUTO_SPEED_LIMIT)
    assert not (CP.safetyConfigs[0].safetyParam & TeslaSafetyFlags.AUTO_SPEED_LIMIT)

  def test_speed_limit_normalizes_display_units(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    car_state = CarState(CP)
    can_parsers = CarState.get_can_parsers(CP)
    for units, speed_limit, unit_to_ms in ((1, 100, CV.KPH_TO_MS), (0, 65, CV.MPH_TO_MS)):
      with self.subTest(units=units):
        can_parsers[Bus.party].vl["DI_state"]["DI_speedUnits"] = units
        can_parsers[Bus.ap_party].vl["DAS_status"]["DAS_fusedSpeedLimit"] = speed_limit
        can_parsers[Bus.ap_party].ts_nanos["DAS_status"]["DAS_fusedSpeedLimit"] = 2_000_000_000

        ret = car_state.update(can_parsers)
        self.assertAlmostEqual(ret.speedLimit, speed_limit * unit_to_ms * CV.MS_TO_KPH, places=5)
        self.assertAlmostEqual(car_state.tesla_speed_limit_target, speed_limit * unit_to_ms)
        self.assertTrue(car_state.tesla_speed_limit_target_valid)

    for speed_limit, timestamp in ((0, 2_000_000_000), (155, 2_000_000_000), (65, 0)):
      with self.subTest(speed_limit=speed_limit, timestamp=timestamp):
        can_parsers[Bus.ap_party].vl["DAS_status"]["DAS_fusedSpeedLimit"] = speed_limit
        can_parsers[Bus.ap_party].ts_nanos["DAS_status"]["DAS_fusedSpeedLimit"] = timestamp
        self.assertEqual(car_state.update(can_parsers).speedLimit, 0.0)
        self.assertFalse(car_state.tesla_speed_limit_target_valid)

  def test_vehicle_bus_tpms_display(self):
    fingerprint = gen_empty_fingerprint()
    fingerprint[CANBUS.vehicle][0x3DF] = 8
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
    car_state = CarState(CP)
    can_parsers = CarState.get_can_parsers(CP)

    self.assertIn(0x25A, can_parsers[Bus.adas].addresses)
    tpms = can_parsers[Bus.adas].vl["VCSEC_TPMSDisplay"]
    tpms["VCSEC_TPMSDisplayPressureFL"] = 2.575
    tpms["VCSEC_TPMSDisplayPressureFR"] = 2.725
    tpms["VCSEC_TPMSDisplayPressureRL"] = 2.625
    tpms["VCSEC_TPMSDisplayPressureRR"] = 2.650

    ret = car_state.update(can_parsers)
    self.assertAlmostEqual(ret.tpms.fl, 37.3, places=1)
    self.assertAlmostEqual(ret.tpms.fr, 39.5, places=1)
    self.assertAlmostEqual(ret.tpms.rl, 38.1, places=1)
    self.assertAlmostEqual(ret.tpms.rr, 38.4, places=1)

    tpms["VCSEC_TPMSDisplayPressureFR"] = 6.375
    self.assertEqual(car_state.update(can_parsers).tpms.fr, 0.0)
