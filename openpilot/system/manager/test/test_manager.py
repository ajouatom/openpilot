import os
import pytest
import signal
import time

from openpilot.cereal import car
from openpilot.common.params import Params
import openpilot.system.manager.manager as manager
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes, procs
from openpilot.system.hardware import HARDWARE

os.environ['FAKEUPLOAD'] = "1"

MAX_STARTUP_TIME = 3
BLACKLIST_PROCS = ['manage_athenad', 'pandad', 'pigeond']


class TestManager:
  def setup_method(self):
    HARDWARE.set_power_save(False)

    # ensure clean CarParams
    params = Params()
    params.clear_all()

  def teardown_method(self):
    manager.manager_cleanup()

  def test_manager_prepare(self):
    os.environ['PREPAREONLY'] = '1'
    manager.main()

  def test_duplicate_procs(self):
    assert len(procs) == len(managed_processes), "Duplicate process names"

  def test_radard_modes_are_mutually_exclusive(self):
    CP = car.CarParams.new_message()
    params = Params()

    params.put("CarrotRadarMode", "0")
    assert not managed_processes["radard"].should_run(False, params, CP)
    assert not managed_processes["radard_dpath"].should_run(False, params, CP)
    assert managed_processes["radard"].should_run(True, params, CP)
    assert not managed_processes["radard_dpath"].should_run(True, params, CP)

    params.put("CarrotRadarMode", "1")
    # A live parameter write must not replace the radar publisher mid-drive.
    assert managed_processes["radard"].should_run(True, params, CP)
    assert not managed_processes["radard_dpath"].should_run(True, params, CP)

    # The next off-road-to-on-road transition applies the new mode.
    assert not managed_processes["radard"].should_run(False, params, CP)
    assert not managed_processes["radard_dpath"].should_run(False, params, CP)
    assert not managed_processes["radard"].should_run(True, params, CP)
    assert managed_processes["radard_dpath"].should_run(True, params, CP)
    assert not managed_processes["radard"].should_run(False, params, CP)
    assert not managed_processes["radard_dpath"].should_run(False, params, CP)

  def test_legacy_radar_motion_mode_is_migrated_once(self):
    params = Params()
    params.remove("CarrotRadarMode")
    params.put("RadarMotionMode", "1")

    manager.migrate_legacy_carrot_radar_mode(params)

    assert params.get_int("CarrotRadarMode") == 1
    assert params.get("RadarMotionMode") is None

    params.put("CarrotRadarMode", "0")
    params.put("RadarMotionMode", "1")
    manager.migrate_legacy_carrot_radar_mode(params)

    assert params.get_int("CarrotRadarMode") == 0
    assert params.get("RadarMotionMode") is None

  def test_carrot_navi_is_permanent_7714_owner(self):
    process = managed_processes["carrot_navi"]
    CP = car.CarParams.new_message()
    params = Params()

    params.put("ClusterHud", "0")
    assert process.should_run(False, params, CP)
    params.put("ClusterHud", "1")
    assert process.should_run(False, params, CP)

  def test_cluster_hud_excludes_carrot_vision_processes(self):
    CP = car.CarParams.new_message()
    CP.notCar = False
    params = Params()
    params.put("DisableDM", "2")
    params.put_bool("CarrotVisionActive", True)

    params.put("ClusterHud", "0")
    assert managed_processes["carrot_webrtcd"].should_run(True, params, CP)
    assert managed_processes["carrot_vision_encoderd"].should_run(True, params, CP)

    params.put("ClusterHud", "1")
    assert managed_processes["carrot_cluster"].should_run(True, params, CP)
    assert not managed_processes["carrot_webrtcd"].should_run(True, params, CP)
    assert not managed_processes["carrot_vision_encoderd"].should_run(True, params, CP)

  def test_removed_cluster_hud_mode_is_inactive(self):
    CP = car.CarParams.new_message()
    CP.notCar = False
    params = Params()
    params.put("DisableDM", "2")
    params.put_bool("CarrotVisionActive", True)
    params.put("ClusterHud", "2")

    assert not managed_processes["carrot_cluster"].should_run(True, params, CP)
    assert managed_processes["carrot_webrtcd"].should_run(True, params, CP)
    assert managed_processes["carrot_vision_encoderd"].should_run(True, params, CP)

  def test_blacklisted_procs(self):
    # TODO: ensure there are blacklisted procs until we have a dedicated test
    assert len(BLACKLIST_PROCS), "No blacklisted procs to test not_run"

  def test_set_params_with_default_value(self):
    params = Params()
    params.clear_all()

    os.environ['PREPAREONLY'] = '1'
    manager.main()
    for k in params.all_keys():
      default_value = params.get_default_value(k)
      if default_value is not None:
        assert params.get(k) == default_value
    assert params.get("OpenpilotEnabledToggle")
    assert params.get("RouteCount") == 0

  @pytest.mark.skip("this test is flaky the way it's currently written, should be moved to test_onroad")
  def test_clean_exit(self, subtests):
    """
      Ensure all processes exit cleanly when stopped.
    """
    HARDWARE.set_power_save(False)
    manager.manager_init()

    CP = car.CarParams.new_message()
    procs = ensure_running(managed_processes.values(), True, Params(), CP, not_run=BLACKLIST_PROCS)

    time.sleep(10)

    for p in procs:
      with subtests.test(proc=p.name):
        state = p.get_process_state_msg()
        assert state.running, f"{p.name} not running"
        exit_code = p.stop(retry=False)

        assert p.name not in BLACKLIST_PROCS, f"{p.name} was started"

        assert exit_code is not None, f"{p.name} failed to exit"

        # TODO: interrupted blocking read exits with 1 in cereal. use a more unique return code
        exit_codes = [0, 1]
        if p.sigkill:
          exit_codes = [-signal.SIGKILL]
        assert exit_code in exit_codes, f"{p.name} died with {exit_code}"
