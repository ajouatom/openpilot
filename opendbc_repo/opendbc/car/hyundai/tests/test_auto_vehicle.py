import pytest

from openpilot.common.auto_vehicle import IONIQ5, PORTER, select_vehicle
from opendbc.car.hyundai.porter_fingerprint import PORTER_BUS0, PORTER_CAMERA, matches_porter


def porter():
  return {0: dict(PORTER_BUS0), 1: {}, 2: dict(PORTER_CAMERA)}


def test_full_porter_signature_and_startup_forwarded_camera_are_accepted():
  assert select_vehicle(None, porter(), True) == PORTER
  fingerprint = porter()
  fingerprint[0].update(PORTER_CAMERA)
  fingerprint[0][0x7d4] = 8
  assert matches_porter(fingerprint)


@pytest.mark.parametrize('address', list(PORTER_BUS0))
def test_missing_porter_message_cannot_select_last_vehicle(address):
  fingerprint = porter()
  del fingerprint[0][address]
  assert select_vehicle(None, fingerprint, True) is None


@pytest.mark.parametrize('address', list(PORTER_BUS0))
def test_wrong_dlc_cannot_select_porter(address):
  fingerprint = porter()
  fingerprint[0][address] += 1
  assert not matches_porter(fingerprint)


@pytest.mark.parametrize('bus', [0, 1, 2, 4, 5, 6])
def test_canfd_traffic_prevents_porter_fallback(bus):
  fingerprint = porter()
  fingerprint.setdefault(bus, {})[0x50] = 16
  assert not matches_porter(fingerprint)


def test_unseen_application_id_and_missing_camera_prevent_porter():
  fingerprint = porter()
  fingerprint[0][0x123] = 8
  assert not matches_porter(fingerprint)
  fingerprint = porter()
  fingerprint[2] = {}
  assert not matches_porter(fingerprint)


@pytest.mark.parametrize('candidate', ['KIA_EV6', 'HYUNDAI_IONIQ_5_PE', 'HYUNDAI_IONIQ_5_N'])
def test_other_firmware_candidates_never_fall_back_to_porter(candidate):
  assert select_vehicle(candidate, porter(), True) is None


@pytest.mark.parametrize(('address', 'size'), [(0x50, 16), (0x110, 32)])
def test_ioniq_requires_exact_firmware_and_hda2_lkas(address, size):
  fingerprint = {0: {}, 1: {}, 2: {address: size}}
  assert select_vehicle(IONIQ5, fingerprint, True) == IONIQ5
  assert select_vehicle(IONIQ5, fingerprint, False) is None
  assert select_vehicle(None, fingerprint, True) is None
  assert select_vehicle(IONIQ5, {0: {}, 1: {}, 2: {}}, True) is None
