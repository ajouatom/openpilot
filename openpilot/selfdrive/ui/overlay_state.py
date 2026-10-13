"""One shared, cheap invalidation snapshot for both onroad display layers."""
from openpilot.system.ui.lib.application import gui_app


class OverlayState:
  def __init__(self):
    self.key = None
    self.enabled = False
    self.urgent = False

  def update(self, ui, camera, stream, mode):
    sm = ui.sm
    cs, ss, lp = sm['carState'], sm['selfdriveState'], sm['longitudinalPlan']
    cm, radar = sm['carrotMan'], sm['radarState']
    # Warnings, BSD and touch processing never wait for a layer's next turn.
    self.enabled = ui.started
    self.urgent = (ss.alertSize.raw != 0 or ss.alertHudVisual.raw != 0 or cs.leftBlindspot or cs.rightBlindspot or
                   sm['modelV2'].meta.laneChangeState.raw != 0 or bool(gui_app.mouse_events))
    self.key = (
      ui.started_frame, ui.status, camera, camera.is_connected(), stream, mode,
      ss.enabled, ss.experimentalMode, cs.vCruiseCluster, cs.gearShifter.raw,
      cs.brakePressed, cs.gasPressed, cs.brakeHoldActive, cs.softHoldActive, cs.carrotCruise,
      lp.xState, lp.trafficState, lp.allowThrottle, cm.trafficState,
      getattr(lp, 'signalDetectionSuspended', False),
      radar.leadOne.status, radar.leadOne.radarTrackId, radar.leadTwo.status,
      sm.recv_frame['liveCalibration'],
      sm.valid['modelV2'], sm.alive['modelV2'], sm.valid['carState'], sm.alive['carState'],
      sm.valid['radarState'], sm.alive['radarState'], sm.valid['longitudinalPlan'], sm.alive['longitudinalPlan'],
      sm.valid['carrotMan'], sm.alive['carrotMan'],
      ui.usbgpu_active, ui.usbgpu_loading, ui.usbgpu_startup_failed,
    )
