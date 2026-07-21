import math

from opendbc.car.hyundai.angle_steering import HyundaiAngleSteering


MAX_TORQUE = 250
MIN_TORQUE = 25
STEER_THRESHOLD = 250


def update(controller, desired=0.0, limited=0.0, actual=0.0, torque=0.0, eps_torque=0.0,
           pressed=False, speed=10.0, active=True):
  return controller.update(active, desired, limited, actual, torque, eps_torque, pressed, speed)


def recover_full_authority(controller):
  for _ in range(20):
    update(controller)
  assert controller.max_torque_command == MAX_TORQUE


def test_same_direction_eps_load_is_not_driver_override():
  controller = HyundaiAngleSteering(MAX_TORQUE, MIN_TORQUE, STEER_THRESHOLD)
  recover_full_authority(controller)

  for _ in range(100):
    command, max_torque = update(controller, desired=20.0, limited=20.0, actual=10.0,
                                 torque=320.0, eps_torque=6.0, pressed=True)

  assert command == 20.0
  assert max_torque == MAX_TORQUE
  assert not controller.driver_override_active


def test_opposing_driver_override_yields_and_rejoins_aligned_path_quickly():
  controller = HyundaiAngleSteering(MAX_TORQUE, MIN_TORQUE, STEER_THRESHOLD)
  recover_full_authority(controller)

  for _ in range(20):
    command, _ = update(controller, desired=20.0, limited=20.0, actual=10.0,
                        torque=-320.0, eps_torque=6.0, pressed=True)
  assert controller.driver_override_active
  assert command == 10.0

  for _ in range(20):
    command, max_torque = update(controller, desired=10.0, limited=10.0, actual=10.0,
                                 torque=-320.0, eps_torque=0.0, pressed=True)
  assert command == 10.0
  assert max_torque == MIN_TORQUE

  # Let the torque envelope and release latch clear, then verify that matching
  # the path restores the 250 ceiling in well under one second.
  for _ in range(80):
    command, max_torque = update(controller, desired=10.0, limited=10.0, actual=10.0)
    if max_torque == MAX_TORQUE and not controller.driver_override_active:
      break

  assert command == 10.0
  assert max_torque == MAX_TORQUE
  assert not controller.recapturing


def test_unaligned_release_bounds_angle_error_while_restoring_authority():
  controller = HyundaiAngleSteering(MAX_TORQUE, MIN_TORQUE, STEER_THRESHOLD)
  recover_full_authority(controller)

  for _ in range(20):
    update(controller, desired=30.0, limited=30.0, actual=10.0,
           torque=-320.0, eps_torque=5.0, pressed=True)
  for _ in range(60):
    command, max_torque = update(controller, desired=30.0, limited=30.0, actual=10.0)

  assert max_torque == MAX_TORQUE
  assert command <= 15.0  # 5 degree tracking window at 10 m/s
  assert controller.recapturing


def test_brief_opposing_reaction_does_not_confirm_driver_override():
  controller = HyundaiAngleSteering(MAX_TORQUE, MIN_TORQUE, STEER_THRESHOLD)
  recover_full_authority(controller)

  for _ in range(7):
    update(controller, desired=-20.0, limited=-20.0, actual=-5.0,
           torque=600.0, eps_torque=-4.0, pressed=True)

  assert not controller.driver_override_active
  assert controller.max_torque_command >= MAX_TORQUE * 0.5


def test_physical_oscillation_unloads_eps_but_torque_noise_alone_does_not():
  noise_only = HyundaiAngleSteering(MAX_TORQUE, MIN_TORQUE, STEER_THRESHOLD)
  recover_full_authority(noise_only)
  for frame in range(100):
    update(noise_only, torque=600.0 if frame % 7 < 3 else -600.0)
  assert noise_only.max_torque_command == MAX_TORQUE
  assert not noise_only.oscillation_active

  controller = HyundaiAngleSteering(MAX_TORQUE, MIN_TORQUE, STEER_THRESHOLD)
  recover_full_authority(controller)
  oscillation_seen = False
  for frame in range(150):
    phase = 2.0 * math.pi * 7.0 * frame * 0.01
    actual = 1.5 * math.sin(phase)
    torque = 650.0 if math.sin(phase) >= 0.0 else -650.0
    eps_torque = 5.0 * math.sin(phase)
    command, _ = update(controller, actual=actual, torque=torque, eps_torque=eps_torque)
    if controller.oscillation_active:
      oscillation_seen = True
      assert command == actual

  assert oscillation_seen
  assert controller.max_torque_command == MIN_TORQUE

  for _ in range(200):
    update(controller)

  assert not controller.oscillation_active
  assert controller.max_torque_command == MAX_TORQUE
