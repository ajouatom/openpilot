import copy
from types import SimpleNamespace as NS

import pytest

from opendbc.can import CANPacker, CANDefine
from opendbc.can.parser import get_raw_value
from opendbc.car import structs
from opendbc.car.hyundai import hyundaican, hyundaicanfd
from opendbc.car.hyundai.values import CAR, HyundaiFlags


def decode(packer, message):
  address, data, bus = message
  return {key: get_raw_value(data, sig) * sig.factor + sig.offset
          for key, sig in packer.dbc.addr_to_msg[address].sigs.items()}


def source(packer, name):
  return {key: 0 for key in packer.dbc.name_to_msg[name].sigs}


@pytest.mark.parametrize('camera_scc', [False, True])
@pytest.mark.parametrize('stock_alert', [0, 1, 2, 5, 7, 8, 9, 10, 12, 14, 21])
def test_fd_warning_transitions_preserve_raw_stock_and_integrity(monkeypatch, camera_scc, stock_alert):
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: NS(get_int=lambda key: 0, get=lambda key: '0'))
  packer = CANPacker('hyundai_canfd_generated')
  stock = source(packer, 'ADRV_0x161') | {'COUNTER': 254, 'ALERTS_2': stock_alert}
  cs = NS(adrv_0x161=stock, out=structs.CarState(), modelV2=None, lfahda_cluster=None,
          cruise_buttons_msg=None, adrv_0x200=None, adrv_0x1ea=None, ccnc_0x162=None,
          is_metric=True, paddle_button_prev=0, softHoldActive=0, trailer_connected=False)
  original = copy.deepcopy(stock)
  cc = structs.CarControl()
  can = NS(ECAN=0, CAM=2)
  for i, level in enumerate((0, 1, 2, 3, 0, 1, 0)):
    cc.hudControl.driverMonitoringAlert = level
    if camera_scc:
      messages = hyundaicanfd.create_ccnc_messages(NS(flags=HyundaiFlags.CAMERA_SCC), packer, can, i * 5,
                                                  cc, cs, cc.hudControl, 0, False, False, 0, False, 0, 0)
    else:
      messages = hyundaicanfd.create_lfa_icon_non_camera_scc(packer, cs, can, cc)
    assert len(messages) == 1
    msg = messages[0]
    assert (msg[0], msg[2]) == (0x161, 0)
    values = decode(packer, msg)
    baseline = 0 if stock_alert in (1, 2, 5, 6, 10, 21, 22) else stock_alert
    expected = baseline
    if level:
      expected = stock_alert if stock_alert in (7, 8, 9, 10, 14, 21) else baseline or (2 if level == 3 else 1)
    assert values['ALERTS_2'] == expected
    assert values['DAW_ICON'] == 0
    dm_popup = level and not baseline and stock_alert not in (10, 21)
    if dm_popup:
      label = CANDefine('hyundai_canfd_generated').dv['ADRV_0x161']['ALERTS_2'][values['ALERTS_2']]
      assert label == ('KEEP_HANDS_ON_STEERING_WHEEL_RED' if level == 3 else 'KEEP_HANDS_ON_STEERING_WHEEL')
    assert values['COUNTER'] == (255 + i) % 256
    assert values['CHECKSUM'] == hyundaicanfd.hkg_can_fd_checksum(msg[0], None, bytearray(msg[1]))
    chime = 3 if dm_popup and level == 3 else 0
    assert values['SOUNDS_2'] == chime
    assert all(values[f'SOUNDS_{n}'] == 0 for n in (1, 3, 4))
    if chime:
      assert CANDefine('hyundai_canfd_generated').dv['ADRV_0x161']['SOUNDS_2'][chime] == 'CONSTANT_CHIME'
    assert stock == original


@pytest.mark.parametrize('level', [0, 1, 2, 3])
def test_classic_lfa_uses_dbc_warning_without_chime_or_control_change(level):
  packer = CANPacker('hyundai_kia_generic')
  cc = structs.CarControl()
  baseline = decode(packer, hyundaican.create_lfahda_mfc(packer, cc, False))
  cc.hudControl.driverMonitoringAlert = level
  values = decode(packer, hyundaican.create_lfahda_mfc(packer, cc, False))
  expected = (6 if level == 3 else 5) if level else 0
  assert values == baseline | {'LFA_SysWarning': expected}
  enums = CANDefine('hyundai_kia_generic').dv['LFAHDA_MFC']['LFA_SysWarning']
  assert enums[expected] == ('KEEP_HANDS_ON_WHEEL_RED' if level == 3 else 'KEEP_HANDS_ON_WHEEL_ORANGE' if level else 'NO_MESSAGE')


@pytest.mark.parametrize('fingerprint,flags', [
  (CAR.HYUNDAI_SANTA_FE, HyundaiFlags.CHECKSUM_CRC8),
  (CAR.KIA_OPTIMA_G4, 0), (CAR.HYUNDAI_SONATA_LF, HyundaiFlags.CHECKSUM_6B),
  (CAR.HYUNDAI_SONATA, HyundaiFlags.SEND_LFA | HyundaiFlags.CHECKSUM_CRC8),
])
def test_classic_lkas_warning_keeps_torque_lanes_counter_and_checksum(fingerprint, flags):
  packer = CANPacker('hyundai_kia_generic')
  cp = NS(carFingerprint=fingerprint, flags=flags)
  stock = source(packer, 'LKAS11')
  original = copy.deepcopy(stock)
  baseline = None
  for level in (0, 1, 2, 3, 0):
    msg = hyundaican.create_lkas11(packer, 7, cp, 123, True, False, stock, False, 3, True,
                                  True, True, 0, 0, False, dm_alert=level)
    values = decode(packer, msg)
    expected = 0
    if level and not flags & HyundaiFlags.SEND_LFA:
      expected = (5 if level == 3 else 4) if fingerprint == CAR.HYUNDAI_SANTA_FE else 3
      if fingerprint == CAR.KIA_OPTIMA_G4:
        expected = 4 if level >= 2 else 0
    assert values['CF_Lkas_SysWarning'] == expected
    unchanged = {k: v for k, v in values.items() if k not in ('CF_Lkas_SysWarning', 'CF_Lkas_Chksum')}
    if baseline is None:
      baseline = unchanged
    assert unchanged == baseline
    data = msg[1]
    if flags & HyundaiFlags.CHECKSUM_CRC8:
      checksum = hyundaican.hyundai_checksum(data[:6] + data[7:8])
    else:
      checksum = (sum(data[:6]) + (0 if flags & HyundaiFlags.CHECKSUM_6B else data[7])) % 256
    assert values['CF_Lkas_Chksum'] == checksum
    assert stock == original


@pytest.mark.parametrize('popup', range(8))
@pytest.mark.parametrize('secondary', [0, 1, 2, 7])
def test_fd_fallback_retains_existing_popups(popup, secondary):
  packer = CANPacker('hyundai_canfd_generated')
  stock = source(packer, 'LFAHDA_CLUSTER') | {'HDA_InfoPUDis': popup, 'HDA_InfoPUDis1': secondary}
  original = stock.copy()
  cs, can = NS(lfahda_cluster=stock), NS(ECAN=0)
  for level in (1, 2, 3, 0):
    values = decode(packer, hyundaicanfd.create_lfahda_cluster(packer, cs, can, False, False, dm_alert=level)[0])
    assert values['HDA_InfoPUDis'] == (5 if level and popup == secondary == 0 else popup)
    assert values['HDA_InfoPUDis1'] == secondary
    assert values['HDA_LFA_WrnSnd'] == 0
    assert stock == original


@pytest.mark.parametrize('level', [1, 2, 3])
def test_dm_never_clears_stock_emergency_sound(level):
  packer = CANPacker('hyundai_canfd_generated')
  stock = source(packer, 'ADRV_0x161') | {'ALERTS_2': 21, 'SOUNDS_1': 6, 'SOUNDS_2': 3, 'SOUNDS_3': 5, 'SOUNDS_4': 2}
  cs = NS(adrv_0x161=stock, out=structs.CarState())
  cc = structs.CarControl()
  cc.hudControl.driverMonitoringAlert = level
  values = decode(packer, hyundaicanfd.create_lfa_icon_non_camera_scc(packer, cs, NS(ECAN=0), cc)[0])
  assert values['ALERTS_2'] == 21
  assert all(values[f'SOUNDS_{i}'] == stock[f'SOUNDS_{i}'] for i in range(1, 5))


@pytest.mark.parametrize('channel,sound', [(1, 6), (2, 2), (3, 5), (4, 2)])
def test_terminal_dm_does_not_add_chime_over_another_stock_sound(channel, sound):
  packer = CANPacker('hyundai_canfd_generated')
  stock = source(packer, 'ADRV_0x161') | {'ALERTS_1': 1, 'ALERTS_3': 1, f'SOUNDS_{channel}': sound}
  original = stock.copy()
  cs = NS(adrv_0x161=stock, out=structs.CarState())
  cc = structs.CarControl()
  cc.hudControl.driverMonitoringAlert = 3
  values = decode(packer, hyundaicanfd.create_lfa_icon_non_camera_scc(packer, cs, NS(ECAN=0), cc)[0])
  assert values['ALERTS_2'] == 2
  assert all(values[f'SOUNDS_{i}'] == stock[f'SOUNDS_{i}'] for i in range(1, 5))
  assert stock == original
