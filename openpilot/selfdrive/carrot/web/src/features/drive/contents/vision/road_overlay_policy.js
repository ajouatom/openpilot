import { DriveVisionHudModel } from "./hud_model.js";

function finiteNumber(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function firstFinite(values, fallback = 0) {
  if (!Array.isArray(values)) return fallback;
  for (const value of values) {
    const number = Number(value);
    if (Number.isFinite(number)) return number;
  }
  return fallback;
}

export function createRoadOverlayPolicy(options = {}) {
  const hudModel = options.hudModel || DriveVisionHudModel;
  if (!hudModel?.isLaneMode || !hudModel?.isLongActive) return null;

  function resolve(overlayState, hudState, params = {}) {
    // Geometry follows the actual planner result, never the requested mode
    // alone. The HUD icon separately acknowledges requested/fallback state.
    const laneModeState = hudModel.resolveLaneModeState?.(hudState);
    const laneMode = laneModeState?.pathActive ?? hudModel.isLaneMode(hudState);
    const model = overlayState?.modelV2 || null;
    const lateralPlan = overlayState?.lateralPlan || null;
    const lanePath = lateralPlan?.position;
    const hasLanePath = Array.isArray(lanePath?.x) && lanePath.x.length > 2;
    const selectedPath = {
      model,
      pathData: laneMode && hasLanePath ? lanePath : model?.position || null,
      pathSource: laneMode && hasLanePath ? "lateralPlan" : "modelV2",
      laneMode,
    };

    let mode = finiteNumber(laneMode ? params.ShowPathModeLane : params.ShowPathMode, 0);
    let colorIndex = finiteNumber(laneMode ? params.ShowPathColorLane : params.ShowPathColor, 13);
    const isCruiseOff = !hudModel.isLongActive(overlayState);

    if (isCruiseOff) {
      colorIndex = finiteNumber(params.ShowPathColorCruiseOff, 19);
    } else if (colorIndex >= 20) {
      const leadOne = overlayState?.radarState?.leadOne || {};
      const accel = firstFinite(hudState?.longitudinalPlan?.accels, 0);
      colorIndex = 13;
      if (leadOne.status) {
        if (Math.abs(accel) < 0.5) colorIndex = 12;
        else if (accel >= 0.5) colorIndex = 11;
        else colorIndex = 10;
      }
    }

    const pathStyle = {
      mode,
      colorIndex,
      paletteIndex: colorIndex % 10,
      emphasisStroke: colorIndex >= 10 || Boolean(hudState?.carState?.brakeLights),
      strokeColor: hudState?.carState?.brakeLights
        ? "rgba(255, 76, 76, 0.96)"
        : "rgba(255, 255, 255, 0.92)",
      isCruiseOff,
      laneMode,
    };

    return { selectedPath, pathStyle };
  }

  return Object.freeze({ resolve });
}

export const DriveVisionRoadOverlayPolicy = Object.freeze({ create: createRoadOverlayPolicy });

export function installDriveVisionRoadOverlayPolicyFacade(target = globalThis) {
  target.DriveVisionRoadOverlayPolicy = DriveVisionRoadOverlayPolicy;
  return DriveVisionRoadOverlayPolicy;
}
