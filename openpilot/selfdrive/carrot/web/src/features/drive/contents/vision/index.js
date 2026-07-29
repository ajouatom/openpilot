"use strict";

import { installDriveVisionHudContentFacade } from "./hud_content.js";
import { installDriveVisionReplayRenderBridgeFacade } from "./replay_render_bridge.js";
import { installDriveVisionReplayRenderControllerFacade } from "./replay_render_controller.js";
import { installDriveVisionPresentedFrameChannelFacade } from "./presented_frame_channel.js";
import { installDriveVisionHudLayoutFacade } from "./hud_layout.js";
import { installDriveVisionHudCanvasFacade } from "./hud_canvas.js";
import { installDriveVisionHudModelFacade } from "./hud_model.js";
import { installDriveVisionRtcPerfHudFacade } from "./hud_rtc_perf.js";
import { installDriveVisionRoadOverlayPolicyFacade } from "./road_overlay_policy.js";
import { installDriveVisionRoadOverlayProjectionFacade } from "./road_overlay_projection.js";
import { installDriveVisionRoadOverlayGeometryRendererFacade } from "./road_overlay_geometry_renderer.js";
import { installDriveVisionRoadOverlayAuxRendererFacade } from "./road_overlay_aux_renderer.js";
import { installDriveVisionRoadOverlayLeadModelFacade } from "./road_overlay_lead_model.js";
import { installDriveVisionRoadOverlayLeadRendererFacade } from "./road_overlay_lead_renderer.js";
import { installDriveVisionSessionPolicyFacade } from "./session_policy.js";
import { installDriveVisionSessionControllerFacade } from "./session_controller.js";
import { installDriveVisionConnectionTransactionFacade } from "./connection_transaction.js";
import { installDriveVisionNetworkRecoveryFacade } from "./network_recovery.js";
import { finalizeDriveVisionFacade } from "./facade.js";
import {
  finalizeDriveVisionCompatibility,
  finalizeDriveVisionContent,
} from "./content.js";
import { installCarrotHudOverlay } from "./hud/index.js";

const installedTargets = new WeakMap();
const finalizedTargets = new WeakMap();
const scheduledTargets = new WeakMap();

export function installDriveVisionLeafFacades(target = globalThis, options = {}) {
  const existing = installedTargets.get(target);
  if (existing) return existing;

  // HUD content owns activation/suppression and hands rendering to the Carrot HUD
  // overlay. HomeDrive is intentionally not required in this phase.
  const hudContent = installDriveVisionHudContentFacade(target, options.hudContent || {});
  if (!hudContent) return null;

  const installed = Object.freeze({
    hudContent,
    presentedFrames: installDriveVisionPresentedFrameChannelFacade(target),
    replayRenderBridge: installDriveVisionReplayRenderBridgeFacade(target),
    replayRenderController: installDriveVisionReplayRenderControllerFacade(target),
    hudLayout: installDriveVisionHudLayoutFacade(target),
    hudCanvas: installDriveVisionHudCanvasFacade(target),
    hudModel: installDriveVisionHudModelFacade(target),
    rtcPerfHud: installDriveVisionRtcPerfHudFacade(target),
    roadOverlayPolicy: installDriveVisionRoadOverlayPolicyFacade(target),
    roadOverlayProjection: installDriveVisionRoadOverlayProjectionFacade(target),
    roadOverlayGeometryRenderer: installDriveVisionRoadOverlayGeometryRendererFacade(target),
    roadOverlayAuxRenderer: installDriveVisionRoadOverlayAuxRendererFacade(target),
    roadOverlayLeadModel: installDriveVisionRoadOverlayLeadModelFacade(target),
    roadOverlayLeadRenderer: installDriveVisionRoadOverlayLeadRendererFacade(target),
    sessionPolicy: installDriveVisionSessionPolicyFacade(target),
    sessionController: installDriveVisionSessionControllerFacade(target),
    connectionTransactions: installDriveVisionConnectionTransactionFacade(target),
    networkRecovery: installDriveVisionNetworkRecoveryFacade(target),
  });
  installedTargets.set(target, installed);

  // 신규 Carrot HUD 오버레이(클러스터 룩) 마운트. 스테이지 DOM 준비 전이면 내부에서 지연 재시도.
  try { installCarrotHudOverlay(target, options.hudOverlay || {}); }
  catch (e) { target.console?.error?.("[carrot hud overlay] install failed", e); }

  return installed;
}

export function finalizeDriveVisionPlatform(target = globalThis, options = {}) {
  const existing = finalizedTargets.get(target);
  if (existing) return existing;
  const leafFacades = installDriveVisionLeafFacades(target, options);
  if (!leafFacades) return null;

  const facade = finalizeDriveVisionFacade(target, options.facade || {});
  if (!facade) return null;
  const contentOptions = {
    ...(options.content || {}),
    renderer: facade,
    hud: leafFacades.hudContent,
  };
  const content = options.compatibilityRegister === false
    ? finalizeDriveVisionContent(target, contentOptions)
    : finalizeDriveVisionCompatibility(target, contentOptions);
  if (!content) return null;

  const finalized = Object.freeze({ leafFacades, facade, content });
  finalizedTargets.set(target, finalized);
  return finalized;
}

export function scheduleDriveVisionFinalize(target = globalThis, options = {}) {
  const existing = scheduledTargets.get(target);
  if (existing) {
    if (!existing.completed || existing.result) return existing;
    scheduledTargets.delete(target);
  }
  if (typeof target?.setTimeout !== "function") return null;

  let timerId = null;
  let result = null;
  let error = null;
  let completed = false;
  const task = Object.freeze({
    get timerId() { return timerId; },
    get result() { return result; },
    get error() { return error; },
    get completed() { return completed; },
  });
  scheduledTargets.set(target, task);
  timerId = target.setTimeout(() => {
    timerId = null;
    try {
      result = finalizeDriveVisionPlatform(target, options);
      if (!result) throw new Error("Drive Vision finalization dependencies are unavailable");
      options.onFinalized?.(result);
    } catch (caught) {
      error = caught;
      options.onError?.(caught);
      target.console?.error?.("[drive vision] finalization failed", caught);
    } finally {
      completed = true;
    }
  }, 0);
  return task;
}

export function getDriveVisionBootStatus(target = globalThis) {
  return Object.freeze({
    installed: installedTargets.has(target),
    finalized: finalizedTargets.has(target),
    scheduled: scheduledTargets.get(target) || null,
  });
}

export * from "./content.js";
export * from "./facade.js";
export * from "./hud_canvas.js";
export * from "./hud_content.js";
export * from "./hud_layout.js";
export * from "./hud_model.js";
export * from "./hud_rtc_perf.js";
export * from "./registry_adapter.js";
export * from "./replay_render_bridge.js";
export * from "./replay_render_controller.js";
export * from "./presented_frame_channel.js";
export * from "./road_overlay_aux_renderer.js";
export * from "./road_overlay_geometry_renderer.js";
export * from "./road_overlay_lead_model.js";
export * from "./road_overlay_lead_renderer.js";
export * from "./road_overlay_policy.js";
export * from "./road_overlay_projection.js";
export * from "./connection_transaction.js";
export * from "./network_recovery.js";
export * from "./session_controller.js";
export * from "./session_policy.js";
