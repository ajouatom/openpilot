import binascii
import copy
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.car.hyundai.hyundaicanfd import create_lfahda_cluster


def state(popup=3):
  return SimpleNamespace(
    lfahda_cluster={"COUNTER": 254, "CHECKSUM": 1234, "HDA_InfoPUDis": popup,
                    "HDA_InfoPUDis1": 0, "HDA_LFA_WrnSnd": 0, "HDA_OptUsmSta": 1},
    lfa={"FCA_SYSWARN": 1, "VALUE63": 15},
    mdps={"LKA_FAULT": 0, "LFA2_FAULT": 0},
    scc_control={"SysFailState": 0},
  )


def send(cs, *, enabled=True, lateral=True, longitudinal=False, packer=None):
  return create_lfahda_cluster(
    packer or CANPacker("hyundai_canfd_generated"), cs, SimpleNamespace(ECAN=0), longitudinal, lateral,
    suppress_camera_auto_disengage=enabled,
  )


@pytest.mark.parametrize("popup", range(8))
def test_only_observed_popup_changes(popup):
  cs = state(popup)
  original = copy.deepcopy(cs)
  address, data, bus = send(cs)[0]
  assert cs == original
  expected = cs.lfahda_cluster | {"COUNTER": 255, "HDA_CntrlModSta": 0, "HDA_LFA_SymSta": 2,
                                "HDA_InfoPUDis": 0 if popup == 3 else popup}
  assert (address, data, bus) == CANPacker("hyundai_canfd_generated").make_can_msg("LFAHDA_CLUSTER", 0, expected)


@pytest.mark.parametrize("snapshot,field,value", [
  ("lfa", "FCA_SYSWARN", 0), ("lfa", "VALUE63", 0),
  ("mdps", "LKA_FAULT", 1), ("mdps", "LFA2_FAULT", 1),
  ("scc_control", "SysFailState", 1),
  *[("lfahda_cluster", "HDA_InfoPUDis1", x) for x in range(1, 8)],
  *[("lfahda_cluster", "HDA_LFA_WrnSnd", x) for x in range(1, 4)],
])
def test_other_signatures_and_faults_pass_through(snapshot, field, value):
  cs = state()
  getattr(cs, snapshot)[field] = value
  assert send(cs) == send(cs, enabled=False)


@pytest.mark.parametrize("snapshot", ["lfa", "mdps", "scc_control"])
@pytest.mark.parametrize("missing", [None, {}])
def test_missing_evidence_does_not_suppress(snapshot, missing):
  cs = state()
  setattr(cs, snapshot, missing)
  assert send(cs) == send(cs, enabled=False)


@pytest.mark.parametrize("lateral,longitudinal", [(False, False), (False, True), (True, True)])
def test_other_control_states_pass_through(lateral, longitudinal):
  cs = state()
  assert send(cs, lateral=lateral, longitudinal=longitudinal) == send(
    cs, enabled=False, lateral=lateral, longitudinal=longitudinal,
  )


def test_default_call_preserves_popup_and_does_not_require_camera_snapshots():
  cs = SimpleNamespace(lfahda_cluster=state().lfahda_cluster)
  messages = create_lfahda_cluster(CANPacker("hyundai_canfd_generated"), cs, SimpleNamespace(ECAN=0), False, True)
  assert messages[0][1][4] & 7 == 3
  cs.lfahda_cluster = None
  assert send(cs, enabled=False) == []


def test_counter_crc_and_input_survive_suppression_transitions():
  packer = CANPacker("hyundai_canfd_generated")
  cs = state()
  for i, warning in enumerate([1, 0, 1, 0]):
    cs.lfa["FCA_SYSWARN"] = warning
    original = copy.deepcopy(cs)
    address, data, bus = send(cs, packer=packer)[0]
    assert cs == original
    assert bus == 0 and data[2] == (255 + i) % 256
    assert data[4] & 7 == (0 if warning else 3)
    assert len(data) == 16
    crc = binascii.crc_hqx(data[2:] + address.to_bytes(2, "little"), 0) ^ 0x041D
    assert int.from_bytes(data[:2], "little") == crc
