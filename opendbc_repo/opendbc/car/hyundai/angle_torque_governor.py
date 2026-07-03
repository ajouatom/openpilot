import numpy as np

from opendbc.car import DT_CTRL


class AngleTorqueGovernor:
  def __init__(self, max_torque, min_torque, max_angle):
    self.max_torque = float(max_torque)
    self.min_torque = float(min_torque)
    self.max_angle = float(max_angle)
    self.driver_unwind_frames = 0

  def reset(self):
    self.driver_unwind_frames = 0

  def update(self, active, CS, desired_angle, applied_angle):
    if not active:
      self.reset()
      return self.max_torque

    desired_clip_error = abs(float(desired_angle) - applied_angle)
    tracking_error = abs(applied_angle - CS.steeringAngleDeg)
    eps_torque = abs(CS.steeringTorqueEps)
    near_angle_cap = abs(applied_angle) > self.max_angle - 2.0
    low_speed_large_angle = CS.vEgo < 7.0 and abs(CS.steeringAngleDeg) > 70.0
    stalled_tracking = abs(CS.steeringRateDeg) < 8.0 and tracking_error > 8.0
    eps_loaded = eps_torque > 18.0
    launch_large_angle = CS.vEgo < 7.0 and abs(CS.steeringAngleDeg) > 20.0
    low_speed_fast_steer = CS.vEgo < 8.0 and abs(CS.steeringRateDeg) > 35.0
    post_driver_unwind = self.driver_unwind_frames > 0 and CS.vEgo < 12.0 and abs(CS.steeringAngleDeg) > 15.0

    torque_cap = self.max_torque
    if launch_large_angle:
      launch_angle = abs(CS.steeringAngleDeg)
      launch_angle_cap = float(np.interp(launch_angle, [20.0, 30.0, 70.0, 120.0],
                                         [self.max_torque, 160.0, 100.0, 60.0]))
      launch_speed_cap = float(np.interp(CS.vEgo, [0.0, 1.0, 3.0, 6.0],
                                         [30.0, 50.0, 100.0, self.max_torque]))
      launch_target_cap = min(launch_angle_cap, launch_speed_cap)
      launch_angle_blend = float(np.interp(launch_angle, [20.0, 30.0], [0.0, 1.0]))
      launch_cap = float(np.interp(launch_angle_blend, [0.0, 1.0],
                                   [self.max_torque, launch_target_cap]))
      launch_cap = float(np.interp(CS.vEgo, [6.0, 7.0], [launch_cap, self.max_torque]))
      torque_cap = min(torque_cap, max(self.min_torque, launch_cap))

    if low_speed_fast_steer:
      steering_rate = abs(CS.steeringRateDeg)
      rate_cap = float(np.interp(steering_rate, [35.0, 45.0, 90.0, 140.0],
                                 [self.max_torque, 150.0, 90.0, 55.0]))
      rate_speed_cap = float(np.interp(CS.vEgo, [0.0, 2.0, 7.0],
                                       [45.0, 80.0, self.max_torque]))
      rate_target_cap = min(rate_cap, rate_speed_cap)
      rate_blend = float(np.interp(steering_rate, [35.0, 45.0], [0.0, 1.0]))
      fast_steer_cap = float(np.interp(rate_blend, [0.0, 1.0],
                                      [self.max_torque, rate_target_cap]))
      fast_steer_cap = float(np.interp(CS.vEgo, [7.0, 8.0], [fast_steer_cap, self.max_torque]))
      torque_cap = min(torque_cap, max(self.min_torque, fast_steer_cap))

    authority_recovery = (not CS.steeringPressed and not post_driver_unwind and not near_angle_cap and
                          3.0 < CS.vEgo < 7.0 and abs(applied_angle) > 30.0 and tracking_error > 8.0 and
                          eps_torque < 18.0)
    if authority_recovery:
      tracking_need = float(np.interp(tracking_error, [8.0, 20.0, 40.0], [0.0, 0.6, 1.0]))
      eps_headroom = float(np.interp(eps_torque, [10.0, 14.0, 18.0], [1.0, 0.5, 0.0]))
      speed_authority = float(np.interp(CS.vEgo, [3.0, 4.2, 7.0], [105.0, 120.0, 140.0]))
      authority_floor = 88.0 + tracking_need * eps_headroom * (speed_authority - 88.0)
      torque_cap = max(torque_cap, min(self.max_torque, authority_floor))

    if post_driver_unwind:
      unwind_angle = abs(CS.steeringAngleDeg)
      unwind_angle_cap = float(np.interp(unwind_angle, [15.0, 25.0, 70.0, 120.0],
                                         [self.max_torque, 80.0, 50.0, 35.0]))
      unwind_speed_cap = float(np.interp(CS.vEgo, [0.0, 4.0, 10.0], [35.0, 55.0, 120.0]))
      unwind_target_cap = min(unwind_angle_cap, unwind_speed_cap)
      unwind_angle_blend = float(np.interp(unwind_angle, [15.0, 25.0], [0.0, 1.0]))
      unwind_cap = float(np.interp(unwind_angle_blend, [0.0, 1.0],
                                   [self.max_torque, unwind_target_cap]))
      unwind_cap = float(np.interp(CS.vEgo, [10.0, 12.0], [unwind_cap, self.max_torque]))
      torque_cap = min(torque_cap, max(self.min_torque, unwind_cap))

    stalled_loaded = stalled_tracking and eps_torque >= 14.0
    if low_speed_large_angle and (near_angle_cap or stalled_loaded or eps_loaded):
      clip_factor = float(np.interp(desired_clip_error, [0.0, 5.0, 20.0, 60.0], [1.0, 0.75, 0.45, 0.25]))
      tracking_factor = float(np.interp(tracking_error, [3.0, 8.0, 15.0, 30.0], [1.0, 0.75, 0.55, 0.35]))
      eps_factor = float(np.interp(eps_torque, [10.0, 18.0, 28.0], [1.0, 0.75, 0.5]))
      speed_factor = float(np.interp(CS.vEgo, [3.0, 7.0], [0.65, 1.0]))
      load_torque_cap = max(self.min_torque, self.max_torque * min(clip_factor, tracking_factor, eps_factor, speed_factor))
      torque_cap = min(torque_cap, load_torque_cap)

    if near_angle_cap and desired_clip_error > 20.0:
      low_load_near_cap = (not CS.steeringPressed and not post_driver_unwind and
                           3.0 < CS.vEgo < 7.0 and eps_torque < 14.0)
      if low_load_near_cap:
        near_cap_target = float(np.interp(desired_clip_error, [20.0, 40.0, 80.0], [88.0, 70.0, 50.0]))
        near_cap_blend = float(np.interp(abs(applied_angle), [173.0, self.max_angle], [0.0, 1.0]))
        near_cap_torque = float(np.interp(near_cap_blend, [0.0, 1.0], [torque_cap, near_cap_target]))
        torque_cap = min(torque_cap, max(self.min_torque, near_cap_torque))
      else:
        torque_cap = min(torque_cap, self.min_torque)

    if CS.steeringPressed:
      self.driver_unwind_frames = int(1.5 / DT_CTRL)
    elif self.driver_unwind_frames > 0:
      self.driver_unwind_frames -= 1
    return torque_cap
