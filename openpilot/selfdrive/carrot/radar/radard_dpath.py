#!/usr/bin/env python3
"""Independent RadarD using vision primary matching and physical dPath leadTwo."""

from __future__ import annotations

import math
from typing import Any

from openpilot.cereal import car, log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from opendbc.car.hyundai.values import HyundaiExtFlags
from openpilot.selfdrive.carrot.radar_motion import (
  DPathRadarController,
)


CORNER_RADAR_FLAGS = int(
  HyundaiExtFlags.CORNER_RADAR_OBJECTS_235
  | HyundaiExtFlags.CORNER_RADAR_OBJECTS_180
  | HyundaiExtFlags.CORNER_RADAR_OBJECTS_430
)
EMPTY_LEAD = {
  "dRel": 0.0,
  "yRel": 0.0,
  "vRel": 0.0,
  "aRel": 0.0,
  "vLead": 0.0,
  "aLead": 0.0,
  "dPath": 0.0,
  "vLat": 0.0,
  "vLeadK": 0.0,
  "aLeadK": 0.0,
  "fcw": False,
  "status": False,
  "aLeadTau": 0.0,
  "modelProb": 0.0,
  "radar": False,
  "radarTrackId": -1,
  "jLead": 0.0,
  "score": 0.0,
}


def empty_lead() -> dict[str, Any]:
  return EMPTY_LEAD.copy()


def corner_radar_enabled(CP: car.CarParams, enable_corner_radar: int) -> bool:
  return (
    CP.brand == "hyundai"
    and enable_corner_radar > 0
    and bool(int(CP.extFlags) & CORNER_RADAR_FLAGS)
  )


def _yaw_rate(live_pose: Any) -> float:
  angular_velocity = getattr(live_pose, "angularVelocityDevice", None)
  if (
    angular_velocity is not None
    and bool(getattr(angular_velocity, "valid", False))
    and bool(getattr(live_pose, "inputsOK", False))
    and bool(getattr(live_pose, "sensorsOK", False))
  ):
    value = float(getattr(angular_velocity, "z", 0.0))
    return value if math.isfinite(value) else 0.0
  return 0.0


def _model_measurement_time_s(sm: messaging.SubMaster) -> float:
  """Use the camera exposure time, not model publication time."""
  timestamp_eof_ns = int(getattr(sm["modelV2"], "timestampEof", 0))
  model_log_ns = int(sm.logMonoTime["modelV2"])
  return float(timestamp_eof_ns if timestamp_eof_ns > 0 else model_log_ns) * 1e-9


class DPathRadarD:
  """Own and publish radarState without importing controls.radard."""

  def __init__(self, CP: car.CarParams) -> None:
    params = Params()
    self.controller = DPathRadarController(
      prefer_corner_radar=corner_radar_enabled(
        CP,
        params.get_int("EnableCornerRadar"),
      ),
      enable_radar_tracks=params.get_int("EnableRadarTracks"),
      cut_in_sensitivity=params.get_int(
        "CarrotRadarCutInSensitivity",
      ),
      front_radar_measurement_delay_s=float(CP.radarDelay),
    )
    self.params = params
    self.radar_state = log.RadarState.new_message()
    self.radar_state_valid = False

  def update(self, sm: messaging.SubMaster, rr: car.RadarData) -> None:
    self.radar_state_valid = sm.all_checks()
    # Reuse the builder like conventional radard. Reallocating every 20 Hz
    # frame is measurable on device and does not change any published field.
    self.radar_state.mdMonoTime = sm.logMonoTime["modelV2"]
    self.radar_state.carStateMonoTime = sm.logMonoTime["carState"]
    self.radar_state.radarErrors = rr.errors

    model_time_s = _model_measurement_time_s(sm)
    radar_time_s = float(sm.logMonoTime["liveTracks"]) * 1e-9
    time_s = (
      model_time_s
      if model_time_s > 0.0
      else 1e-9 * max(sm.logMonoTime.values())
    )
    output = self.controller.update(
      time_s=time_s,
      v_ego=float(sm["carState"].vEgo),
      radar_points=rr.points,
      model=sm["modelV2"],
      yaw_rate_rad_s=_yaw_rate(sm["livePose"]),
      radar_to_model_time_s=model_time_s - radar_time_s,
      radar_reaction_factor=(
        self.params.get_float("RadarReactionFactor") * 0.01
      ),
    )
    self.radar_state.leadOne = output.lead_one or empty_lead()
    self.radar_state.leadTwo = output.lead_two or empty_lead()
    self.radar_state.leadLeft = output.lead_left or empty_lead()
    self.radar_state.leadRight = output.lead_right or empty_lead()
    self.radar_state.leadsLeft = output.leads_left
    self.radar_state.leadsCenter = output.leads_center
    self.radar_state.leadsRight = output.leads_right
    self.radar_state.leadsCutIn = output.leads_cutin
    self.radar_state.leadsLeft2 = output.leads_left2
    self.radar_state.leadsRight2 = output.leads_right2

  def publish(self, pm: messaging.PubMaster) -> None:
    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("dPath radard is waiting for CarParams")
  CP = messaging.log_from_bytes(
    Params().get("CarParams", block=True),
    car.CarParams,
  )
  cloudlog.info("dPath radard got CarParams")

  sm = messaging.SubMaster(
    ["modelV2", "carState", "liveTracks", "livePose"],
    poll="modelV2",
    ignore_alive=["livePose"],
    ignore_valid=["livePose"],
  )
  pm = messaging.PubMaster(["radarState"])
  radar = DPathRadarD(CP)

  while True:
    sm.update()
    if sm.updated["modelV2"]:
      radar.update(sm, sm["liveTracks"])
      radar.publish(pm)


if __name__ == "__main__":
  main()
