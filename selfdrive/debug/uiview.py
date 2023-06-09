#!/usr/bin/env python3
import time

from cereal import car, log, messaging
from common.params import Params
from selfdrive.manager.process_config import managed_processes

if __name__ == "__main__":
  CP = car.CarParams(notCar=True)
  Params().put("CarParams", CP.to_bytes())

  procs = ['camerad', 'ui', 'modeld', 'calibrationd']
  for p in procs:
    managed_processes[p].start()

  pm = messaging.PubMaster(['controlsState', 'deviceState', 'pandaStates', 'carParams', 'longitudinalPlan', 'carState', 'lateralPlan', 'carControl'])

  msgs = {s: messaging.new_message(s) for s in ['controlsState', 'deviceState', 'carParams', 'longitudinalPlan', 'carState', 'lateralPlan', 'carControl']}
  msgs['deviceState'].deviceState.started = True
  msgs['carParams'].carParams.openpilotLongitudinalControl = True
  msgs['longitudinalPlan'].longitudinalPlan.trafficState = 0
  msgs['longitudinalPlan'].longitudinalPlan.xStop = 101
  msgs['carState'].carState.vEgoCluster = 123
  msgs['lateralPlan'].lateralPlan.desire = 1
  msgs['lateralPlan'].lateralPlan.laneChangeDirection = 1
  msgs['lateralPlan'].lateralPlan.desireEvent = 0

  msgs['pandaStates'] = messaging.new_message('pandaStates', 1)
  msgs['pandaStates'].pandaStates[0].ignitionLine = True
  msgs['pandaStates'].pandaStates[0].pandaType = log.PandaState.PandaType.uno

  try:
    while True:
      time.sleep(1 / 100)  # continually send, rate doesn't matter
      for s in msgs:
        pm.send(s, msgs[s])
  except KeyboardInterrupt:
    for p in procs:
      managed_processes[p].stop()
