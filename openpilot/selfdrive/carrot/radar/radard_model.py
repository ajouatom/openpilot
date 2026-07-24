#!/usr/bin/env python3
"""Independent model-based radard implementation."""

from __future__ import annotations

from openpilot.cereal import car, log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from opendbc.car.hyundai.values import HyundaiExtFlags
from openpilot.selfdrive.carrot.radar.radar_lead_runtime import RadarLeadRuntime
from openpilot.selfdrive.carrot.radar.radar_vision_model_controller import RadarLeadModelOutput, VisionModelRadarController


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


def empty_lead() -> dict:
  return EMPTY_LEAD.copy()


CORNER_RADAR_FLAGS = int(
  HyundaiExtFlags.CORNER_RADAR_OBJECTS_235
  | HyundaiExtFlags.CORNER_RADAR_OBJECTS_180
  | HyundaiExtFlags.CORNER_RADAR_OBJECTS_430
)


def corner_radar_enabled(CP: car.CarParams, enable_corner_radar: int) -> bool:
  return (
    CP.brand == "hyundai"
    and enable_corner_radar > 0
    and bool(int(CP.extFlags) & CORNER_RADAR_FLAGS)
  )


class ModelRadarD:
  """Build radarState with vision-matched leadOne and model secondary leads."""

  def __init__(self, CP: car.CarParams) -> None:
    params = Params()
    radar_reaction_factor = params.get_float("RadarReactionFactor") * 0.01
    self.controller = VisionModelRadarController(radar_reaction_factor)
    self.controller.runtime = RadarLeadRuntime(
      include_scc=True,
      enable_radar_tracks=params.get_int("EnableRadarTracks"),
      corner_radar_enabled=corner_radar_enabled(CP, params.get_int("EnableCornerRadar")),
    )
    self.radar_state = log.RadarState.new_message()
    self.radar_state_valid = False

  @staticmethod
  def _assign_output(state, output: RadarLeadModelOutput) -> None:
    state.leadOne = output.lead_one or empty_lead()
    state.leadTwo = output.lead_two or empty_lead()
    state.leadLeft = output.lead_left or empty_lead()
    state.leadRight = output.lead_right or empty_lead()
    state.leadsLeft = output.leads_left
    state.leadsCenter = output.leads_center
    state.leadsRight = output.leads_right
    state.leadsCutIn = output.leads_cutin
    state.leadsLeft2 = output.leads_left2
    state.leadsRight2 = output.leads_right2

  def update(self, sm: messaging.SubMaster, rr: car.RadarData) -> None:
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime["modelV2"]
    self.radar_state.carStateMonoTime = sm.logMonoTime["carState"]
    self.radar_state.radarErrors = rr.errors

    current_time = 1e-9 * max(sm.logMonoTime.values())
    output = self.controller.update(current_time, float(sm["carState"].vEgo), rr.points, sm["modelV2"])
    self._assign_output(self.radar_state, output)

  def publish(self, pm: messaging.PubMaster) -> None:
    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("model radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("model radard got CarParams")

  sm = messaging.SubMaster(
    ["modelV2", "carState", "liveTracks", "livePose"], poll="modelV2",
    ignore_alive=["livePose"], ignore_valid=["livePose"],
  )
  pm = messaging.PubMaster(["radarState"])
  radar = ModelRadarD(CP)

  while True:
    sm.update()
    if sm.updated["modelV2"]:
      radar.update(sm, sm["liveTracks"])
      radar.publish(pm)


if __name__ == "__main__":
  main()
