function finiteNumber(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function hasNearbyAssistLead(lead, speedMps) {
  const speed = finiteNumber(speedMps, 0);
  if (speed <= 0) return false;
  const threshold = speed * 3.0;
  return Boolean(lead?.status)
    && finiteNumber(lead?.dRel, Infinity) > 0
    && finiteNumber(lead?.dRel, Infinity) < threshold;
}

export function createRoadOverlayAuxRenderer(options = {}) {
  const getParams = typeof options.getParams === "function" ? options.getParams : () => ({});
  const pathZOffset = finiteNumber(options.pathZOffset, 1.22);
  const geometry = options.geometry || {};
  const ui = options.ui || {};

  if (
    typeof geometry.buildVerticalRibbon !== "function"
    || typeof geometry.drawPolygon !== "function"
    || typeof geometry.samplePathY !== "function"
    || typeof geometry.interpolate !== "function"
    || typeof geometry.projectPoint !== "function"
    || typeof geometry.drawPolyline !== "function"
    || typeof ui.getScale !== "function"
    || typeof ui.displayDistance !== "function"
    || typeof ui.clampTextAnchor !== "function"
    || typeof ui.drawText !== "function"
  ) {
    return null;
  }

  function drawBlindspotBarriers(modelPath, overlayState, hudState, calibTransform) {
    if (!modelPath || !Array.isArray(modelPath.x) || modelPath.x.length < 2) return;

    const radarState = overlayState?.radarState || {};
    const carState = hudState?.carState || {};
    const lateralPlan = overlayState?.lateralPlan || {};
    const speedMps = finiteNumber(carState?.vEgo, finiteNumber(carState?.vEgoCluster, 0));
    const laneChangeState = finiteNumber(lateralPlan?.laneChangeState, 0);
    const laneChangeDirection = finiteNumber(lateralPlan?.laneChangeDirection, 0);
    const leftBlindspot = Boolean(carState?.leftBlindspot);
    const rightBlindspot = Boolean(carState?.rightBlindspot);
    const leftAssistWarn = !leftBlindspot
      && laneChangeState === 1
      && laneChangeDirection === 1
      && hasNearbyAssistLead(radarState?.leadLeft, speedMps);
    const rightAssistWarn = !rightBlindspot
      && laneChangeState === 1
      && laneChangeDirection === 2
      && hasNearbyAssistLead(radarState?.leadRight, speedMps);
    if (!leftBlindspot && !rightBlindspot && !leftAssistWarn && !rightAssistWarn) return;

    const goldFill = "rgba(255, 215, 0, 0.48)";
    const goldStroke = "rgba(255, 215, 0, 0.84)";
    const greenFill = "rgba(0, 204, 0, 0.44)";
    const greenStroke = "rgba(0, 204, 0, 0.80)";
    const drawRibbon = (shift, fill, stroke) => {
      const ribbon = geometry.buildVerticalRibbon(calibTransform, modelPath, shift, 1.15, 0.60, 40);
      if (ribbon.length < 8) return;
      geometry.drawPolygon(ribbon, fill, stroke, 1.2);
    };

    if (leftBlindspot) drawRibbon(-1.7, goldFill, goldStroke);
    else if (leftAssistWarn) drawRibbon(-1.7, greenFill, greenStroke);

    if (rightBlindspot) drawRibbon(1.7, goldFill, goldStroke);
    else if (rightAssistWarn) drawRibbon(1.7, greenFill, greenStroke);
  }

  function drawProjectedTfMarker(modelPath, longitudinalPlan, calibTransform, videoWidth, videoHeight) {
    if (finiteNumber(getParams().ShowPathEnd, 0) <= 0) return;

    const tfDistance = finiteNumber(longitudinalPlan?.desiredDistance, 0);
    if (!Number.isFinite(tfDistance) || tfDistance <= 0) return;

    const xs = Array.isArray(modelPath?.x) ? modelPath.x : [];
    if (xs.length < 2) return;
    const lastX = finiteNumber(xs[xs.length - 1], 0);
    if (!lastX || tfDistance > lastX) return;

    const lineY = geometry.samplePathY(modelPath, tfDistance);
    const lineZ = geometry.interpolate(tfDistance, xs, Array.isArray(modelPath?.z) ? modelPath.z : []);
    if (!Number.isFinite(lineY) || !Number.isFinite(lineZ)) return;

    const left = geometry.projectPoint(calibTransform, tfDistance, lineY - 1.0, lineZ + pathZOffset);
    const right = geometry.projectPoint(calibTransform, tfDistance, lineY + 1.0, lineZ + pathZOffset);
    if (!left || !right) return;

    const uiScale = ui.getScale(videoWidth, videoHeight);
    geometry.drawPolyline([left, right], "rgba(255,255,255,0.92)", Math.max(3.0 * uiScale, 1.6));
    const labelText = `${ui.displayDistance(tfDistance).toFixed(1)}(${finiteNumber(longitudinalPlan?.tFollow, 0).toFixed(2)})`;
    const labelFontSize = Math.max(20 * uiScale, 12);
    const labelAnchor = ui.clampTextAnchor(
      { x: right.x + 10, y: right.y - 4 },
      labelText,
      labelFontSize,
      videoWidth,
      videoHeight,
    );
    ui.drawText(labelText, labelAnchor.x, labelAnchor.y, {
      fontSize: labelFontSize,
      fontWeight: 800,
      align: "left",
      strokeWidth: Math.max(3.4 * uiScale, 1.8),
    });
  }

  return Object.freeze({ drawBlindspotBarriers, drawProjectedTfMarker });
}

export const DriveVisionRoadOverlayAuxRenderer = Object.freeze({
  create: createRoadOverlayAuxRenderer,
});

export function installDriveVisionRoadOverlayAuxRendererFacade(target = globalThis) {
  target.DriveVisionRoadOverlayAuxRenderer = DriveVisionRoadOverlayAuxRenderer;
  return DriveVisionRoadOverlayAuxRenderer;
}
