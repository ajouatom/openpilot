from types import SimpleNamespace



def test_park_assist_stand_aside_only_at_parking_speeds():
  from opendbc.car.hyundai.carcontroller import park_assist_active
  cs = lambda value63, kph: SimpleNamespace(lfa={'VALUE63': value63}, out=SimpleNamespace(vEgoRaw=kph / 3.6))
  assert park_assist_active(cs(15, 2))
  assert not park_assist_active(cs(15, 20))      # never on the road, even if the flag stays latched
  assert not park_assist_active(cs(0, 2))
  assert not park_assist_active(SimpleNamespace(lfa=None, out=SimpleNamespace(vEgoRaw=0.)))


def test_steer_angle_test_sends_an_active_angle_without_engagement():
  from opendbc.can import CANPacker
  from opendbc.can.parser import get_raw_value
  from opendbc.car.hyundai import hyundaicanfd
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['LFA_ALT']
  state = SimpleNamespace(mdps=None, adrv_0x161=None, lfa=None, steer_touch_2af=None,
                          lfa_alt={key: 0 for key in definition.sigs})
  cc = SimpleNamespace(latActive=False)
  for override, active, torque in ((False, 1, 0), (True, 2, 100)):
    msgs = hyundaicanfd.create_steering_messages_camera_scc(0, packer, None, SimpleNamespace(CAM=2, ECAN=0), cc, False, 0, state,
                                                            120.0, 100, True, lat_override=override)
    _, data, _ = next(m for m in msgs if m[0] == definition.address)
    assert get_raw_value(data, definition.sigs['LKAS_ANGLE_ACTIVE']) == active
    assert get_raw_value(data, definition.sigs['LKAS_ANGLE_MAX_TORQUE']) == torque
