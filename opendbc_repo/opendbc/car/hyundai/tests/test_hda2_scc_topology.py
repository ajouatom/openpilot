import pytest

from opendbc.car import gen_empty_fingerprint, interfaces
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.tests import test_cluster_direct_tx
from opendbc.car.hyundai.values import CANFD_CAR, HyundaiFlags, HyundaiSafetyFlags

configuration = test_cluster_direct_tx.configuration


@pytest.mark.parametrize('candidate', [c for c in CANFD_CAR if c in interfaces.get_torque_params()])
@pytest.mark.parametrize('hda2', [0, 1])
@pytest.mark.parametrize('camera_scc', [0, 1, 2, 3])
def test_harness_selection_owns_scc_bus(configuration, candidate, hda2, camera_scc):
  values, make = configuration
  values.update(CanfdHDA2=hda2, HyundaiCameraSCC=camera_scc)
  cp = make(candidate)
  expected_camera = bool(camera_scc or (not hda2 and candidate.config.flags & HyundaiFlags.CAMERA_SCC))
  assert bool(cp.flags & HyundaiFlags.CAMERA_SCC) == expected_camera
  assert bool(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CAMERA_SCC) == expected_camera
  assert CanBus(cp).ECAN == (1 if hda2 and not camera_scc else 0)
  # With alpha longitudinal disabled, an inherited camera flag must not force
  # host longitudinal control on a stock HDA2 harness.
  assert cp.openpilotLongitudinalControl == (expected_camera and camera_scc < 3)
  enabled = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], True, False, False)
  assert enabled.openpilotLongitudinalControl == (camera_scc < 3)
