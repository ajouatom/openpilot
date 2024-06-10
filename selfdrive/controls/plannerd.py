#!/usr/bin/env python3
import os
import numpy as np
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.lateral_planner import LateralPlanner
import cereal.messaging as messaging

# carrot
from openpilot.selfdrive.carrot.carrot_functions import CarrotPlannerHelper

def cumtrapz(x, t):
  return np.concatenate([[0], np.cumsum(((x[0:-1] + x[1:])/2) * np.diff(t))])

def plannerd_thread():
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  with car.CarParams.from_bytes(params.get("CarParams", block=True)) as msg:
    CP = msg
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  debug_mode = bool(int(os.getenv("DEBUG", "0")))

  longitudinal_planner = LongitudinalPlanner(CP)
  lateral_planner = LateralPlanner(CP, debug=debug_mode)

  carrot_planner = CarrotPlannerHelper()

  pm = messaging.PubMaster(['longitudinalPlan', 'lateralPlan'])
  sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'radarState', 'modelV2', 'navInstruction', 'roadLimitSpeed'],
                           poll='modelV2', ignore_avg_freq=['radarState', 'navInstruction', 'roadLimitSpeed'])

  while True:
    sm.update()
    #print("planner= {:.2f}".format(sm.avg_freq['carState']))
    if sm.updated['modelV2']:
      lateral_planner.update(sm, carrot_planner)
      lateral_planner.publish(sm, pm)
      longitudinal_planner.update(sm, carrot_planner)
      longitudinal_planner.publish(sm, pm, carrot_planner)

def main():
  plannerd_thread()


if __name__ == "__main__":
  main()
