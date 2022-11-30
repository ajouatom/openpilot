#!/usr/bin/env python3
# a simple steering only driver monitor module for C2

import gc

import cereal.messaging as messaging
from common.realtime import set_realtime_priority
from selfdrive.controls.lib.events import Events
from selfdrive.dragonpilot.driver_monitor import DriverStatus
from cereal import car

from common.realtime import DT_DMON
MAX_TERMINAL_ALERTS = 3  # not allowed to engage after 3 terminal alerts
MAX_TERMINAL_DURATION = int(30 / DT_DMON)  # not allowed to engage after 30s of terminal alerts
from common.realtime import Ratekeeper

def dpmonitoringd_thread(sm=None, pm=None):
  rk = Ratekeeper(10, print_delay_threshold=None)  # Keeps rate at 10 hz
  gc.disable()
  set_realtime_priority(2)

  if pm is None:
    pm = messaging.PubMaster(['driverMonitoringState'])

  if sm is None:
    sm = messaging.SubMaster(['carState', 'controlsState'])

  driver_status = DriverStatus()

  sm['carState'].buttonEvents = []
  sm['carState'].standstill = True

  v_cruise_last = 0
  driver_engaged = False

  while True:
    sm.update()

    # Get interaction
    if sm.updated['carState']:
      v_cruise = sm['carState'].cruiseState.speed
      driver_engaged = len(sm['carState'].buttonEvents) > 0 or \
                        v_cruise != v_cruise_last or \
                        sm['carState'].steeringPressed or \
                       sm['carState'].gasPressed or \
                       sm['carState'].brakePressed
      v_cruise_last = v_cruise

    events = Events()

    if driver_status.terminal_alert_cnt >= MAX_TERMINAL_ALERTS or \
       driver_status.terminal_time >= MAX_TERMINAL_DURATION:
      events.add(car.CarEvent.EventName.tooDistracted)

    # Update events from driver state
    driver_status.update_events(events, driver_engaged, sm['controlsState'].enabled, sm['carState'].standstill)
    # driver_status.update_events(events, driver_engaged, True, False)

    # build driverMonitoringState packet
    dat = messaging.new_message('driverMonitoringState')
    dat.driverMonitoringState = {
      "events": events.to_msg(),
      # "faceDetected": False,
      # "isDistracted": True,
      # "distractedType": sum(driver_status.distracted_types),
      "awarenessStatus": driver_status.awareness,
      # "posePitchOffset": driver_status.pose.pitch_offseter.filtered_stat.mean(),
      # "posePitchValidCount": driver_status.pose.pitch_offseter.filtered_stat.n,
      # "poseYawOffset": driver_status.pose.yaw_offseter.filtered_stat.mean(),
      # "poseYawValidCount": driver_status.pose.yaw_offseter.filtered_stat.n,
      # "stepChange": driver_status.step_change,
      # "awarenessActive": driver_status.awareness_active,
      # "awarenessPassive": driver_status.awareness_passive,
      # "isLowStd": driver_status.pose.low_std,
      # "hiStdCount": driver_status.hi_stds,
      "isActiveMode": driver_status.awareness <= driver_status.threshold_prompt,
      # "isRHD": driver_status.wheel_on_right,
    }
    pm.send('driverMonitoringState', dat)
    rk.keep_time()


def main(sm=None, pm=None):
  dpmonitoringd_thread(sm, pm)


if __name__ == '__main__':
  main()
