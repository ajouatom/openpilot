#!/usr/bin/env python3
# a simple steering only driver monitor module for C2

from cereal import car
EventName = car.CarEvent.EventName
from common.realtime import DT_DMON # 0.05 = 20hz

# ref (page15-16): https://eur-lex.europa.eu/legal-content/EN/TXT/PDF/?uri=CELEX:42018X1947&rid=2
_AWARENESS_TIME = 30.*10.  # 30 secs limit without user touching steering wheels make the car enter a terminal status
_AWARENESS_PRE_TIME_TILL_TERMINAL = 15.  # a first alert is issued 15s before expiration
_AWARENESS_PROMPT_TIME_TILL_TERMINAL = 6.  # a second alert is issued 6s before start decelerating the car

class DriverStatus():
  def __init__(self):
    self.terminal_alert_cnt = 0
    self.terminal_time = 0

    self.awareness = 1.

    self.ts_last_check = 0.

    self.threshold_pre = _AWARENESS_PRE_TIME_TILL_TERMINAL / _AWARENESS_TIME
    self.threshold_prompt = _AWARENESS_PROMPT_TIME_TILL_TERMINAL / _AWARENESS_TIME
    self.step_change = DT_DMON / _AWARENESS_TIME

  def update_events(self, events, driver_engaged, ctrl_active, standstill):
    if (driver_engaged and self.awareness > 0) or not ctrl_active:
      # always reset if driver is in control (unless we are in red alert state) or op isn't active
      self.awareness = 1.
      return

    awareness_prev = self.awareness

    if not (standstill and self.awareness - self.step_change <= self.threshold_prompt):
      self.awareness = max(self.awareness - self.step_change, -0.1)

    alert = None
    if self.awareness <= 0.:
      # terminal red alert: disengagement required
      alert = EventName.driverUnresponsive
      self.terminal_time += 1
      if awareness_prev > 0.:
        self.terminal_alert_cnt += 1
    elif self.awareness <= self.threshold_prompt:
      # prompt orange alert
      alert = EventName.promptDriverUnresponsive
    elif self.awareness <= self.threshold_pre:
      # pre green alert
      alert = EventName.preDriverUnresponsive

    # print("trigger to green in: %s secs" % ((self.threshold_pre - self.awareness) / self.step_change * DT_DMON))
    # print("trigger to orange in: %s secs" % ((self.threshold_prompt - self.awareness) / self.step_change * DT_DMON))
    # print("trigger to red in: %s secs" % ((0 - self.awareness) / self.step_change * DT_DMON))
    # print("---------------------------------------------------------------------------")

    if alert is not None:
      events.add(alert)

if __name__ == "__main__":
  pass

