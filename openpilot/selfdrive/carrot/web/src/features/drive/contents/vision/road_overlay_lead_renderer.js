import { createRoadOverlayLeadModel } from "./road_overlay_lead_model.js";

const LEAD_HOLD_MS = 1500;

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function finiteNumber(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

export function createRoadOverlayLeadRenderer(options = {}) {
  const context = options.context;
  const model = options.model || createRoadOverlayLeadModel(options.modelOptions);
  const getParams = typeof options.getParams === "function" ? options.getParams : () => ({});
  const isMetricDisplay = typeof options.isMetricDisplay === "function" ? options.isMetricDisplay : () => true;
  const displayDistance = typeof options.displayDistance === "function" ? options.displayDistance : (value) => Number(value);
  const baseCamera = options.baseCamera || { width: 1928, height: 1208 };
  const fontFamily = String(options.fontFamily || "system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif");
  const projection = options.projection || {};
  const geometry = options.geometry || {};
  const ui = options.ui || {};
  const temporal = options.temporal || {};

  if (
    !context
    || !model?.resolveLeadState
    || typeof projection.projectLeadBox !== "function"
    || typeof projection.projectLeadTwoBox !== "function"
    || typeof projection.projectPathEndAnchorBox !== "function"
    || typeof projection.samplePathZ !== "function"
    || typeof projection.projectPointPrecise !== "function"
    || typeof geometry.drawPolyline !== "function"
    || typeof geometry.drawPolygon !== "function"
    || typeof geometry.circlePolygon !== "function"
    || typeof ui.getScale !== "function"
    || typeof ui.getBadgeOffsets !== "function"
    || typeof ui.getVisibleRect !== "function"
    || typeof ui.fillRoundedRect !== "function"
    || typeof ui.strokeRoundedRect !== "function"
    || typeof ui.drawText !== "function"
    || typeof temporal.now !== "function"
    || typeof temporal.resetLeadSlot !== "function"
    || typeof temporal.resetLeadTwo !== "function"
  ) {
    return null;
  }

  let holdState;
  let renderState;

  function reset() {
    holdState = {
      lastValidMs: 0,
      box: null,
      strokeColor: null,
      isLeadScc: false,
      radarDist: 0,
      visionDist: 0,
      badgeTextColor: "#ffffff",
    };
    renderState = {
      lastCameraFrameId: NaN,
      lastModelFrameId: NaN,
      lastSourceWidth: NaN,
      lastSourceHeight: NaN,
    };
  }

  function getHeldBox(nowMs = temporal.now()) {
    if (!holdState.box) return null;
    if ((nowMs - finiteNumber(holdState.lastValidMs, 0)) >= LEAD_HOLD_MS) return null;
    return holdState.box;
  }

  function syncRenderState(videoWidth, videoHeight, modelFrameId, cameraFrameId) {
    const nextWidth = finiteNumber(videoWidth, NaN);
    const nextHeight = finiteNumber(videoHeight, NaN);
    const nextModelFrameId = finiteNumber(modelFrameId, NaN);
    const nextCameraFrameId = finiteNumber(cameraFrameId, NaN);
    const previous = renderState;
    const sourceChanged = Number.isFinite(previous.lastSourceWidth)
      && Number.isFinite(previous.lastSourceHeight)
      && (Math.abs(previous.lastSourceWidth - nextWidth) > 0.5 || Math.abs(previous.lastSourceHeight - nextHeight) > 0.5);
    const cameraFrameRewind = Number.isFinite(previous.lastCameraFrameId)
      && Number.isFinite(nextCameraFrameId)
      && nextCameraFrameId < previous.lastCameraFrameId;
    const modelFrameRewind = Number.isFinite(previous.lastModelFrameId)
      && Number.isFinite(nextModelFrameId)
      && nextModelFrameId < previous.lastModelFrameId;
    if (sourceChanged || cameraFrameRewind || modelFrameRewind) {
      temporal.resetLeadSlot(0);
      temporal.resetLeadSlot(1);
      temporal.resetLeadTwo();
      holdState.box = null;
    }

    renderState = {
      lastCameraFrameId: nextCameraFrameId,
      lastModelFrameId: nextModelFrameId,
      lastSourceWidth: nextWidth,
      lastSourceHeight: nextHeight,
    };
  }

  function drawLeadBoxCard(box, strokeColor, fillColor, primary = true) {
    if (!box?.rect) return;
    const { x, y, width, height } = box.rect;
    const uiScale = ui.getScale(box.videoWidth || baseCamera.width, box.videoHeight || baseCamera.height);
    const radius = Math.max((primary ? 15 : 12) * uiScale, primary ? 7 : 6);
    const strokeWidth = Math.max((primary ? 3.0 : 2.2) * uiScale, primary ? 1.7 : 1.3);
    ui.fillRoundedRect(context, x, y, width, height, radius, fillColor);
    ui.strokeRoundedRect(context, x, y, width, height, radius, strokeColor, strokeWidth);
  }

  function eraseLeadBoxOcclusion(box, primary = true) {
    if (!box?.rect) return;
    const { x, y, width, height } = box.rect;
    const uiScale = ui.getScale(box.videoWidth || baseCamera.width, box.videoHeight || baseCamera.height);
    context.save();
    context.globalCompositeOperation = "destination-out";
    ui.fillRoundedRect(
      context,
      x,
      y,
      width,
      height,
      Math.max((primary ? 15 : 12) * uiScale, primary ? 7 : 6),
      "rgba(0,0,0,1)",
    );
    context.restore();
  }

  function drawLeadDistanceBadges(box, radarDist, visionDist, isLeadScc, textColor, videoWidth, videoHeight, stageWidth, stageHeight, transform) {
    if (!box?.rect) return;
    const offsets = ui.getBadgeOffsets(videoWidth, videoHeight);
    const badgeHeight = offsets.badgeHeight;
    const fontSize = offsets.fontSize;
    const visibleRect = ui.getVisibleRect(videoWidth, videoHeight, stageWidth, stageHeight, transform);
    const baseY = Math.min(
      box.centerY + offsets.rectTopOffset,
      Math.max(visibleRect.top + 4, visibleRect.bottom - badgeHeight - 4),
    );
    const uiScale = ui.getScale(videoWidth, videoHeight);

    const drawBadge = (text, centerX, backgroundColor) => {
      const charWidth = fontSize * 0.62;
      const width = Math.max(52 * uiScale, 16 * uiScale + text.length * charWidth);
      const x = centerX - width * 0.5;
      ui.fillRoundedRect(context, x, baseY, width, badgeHeight, offsets.radius, backgroundColor);
      context.save();
      context.font = `800 ${fontSize}px ${fontFamily}`;
      context.textAlign = "center";
      context.textBaseline = "middle";
      context.lineJoin = "round";
      context.strokeStyle = "rgba(0,0,0,0.82)";
      context.fillStyle = textColor;
      context.lineWidth = offsets.strokeWidth;
      context.strokeText(text, centerX, baseY + badgeHeight * 0.54);
      context.fillText(text, centerX, baseY + badgeHeight * 0.54);
      context.restore();
    };

    const radarColor = isLeadScc ? "rgba(255,0,0,0.92)" : "rgba(255,175,3,0.92)";
    const visionColor = "rgba(0,0,255,0.92)";
    const radarCenterX = box.centerX - offsets.dx;
    const visionCenterX = box.centerX + offsets.dx;
    const radarText = radarDist < 10 ? radarDist.toFixed(1) : radarDist.toFixed(0);
    const visionText = visionDist < 10 ? visionDist.toFixed(1) : visionDist.toFixed(0);

    if (radarDist > 0 && visionDist > 0) {
      drawBadge(radarText, radarCenterX, radarColor);
      drawBadge(visionText, visionCenterX, visionColor);
    } else if (radarDist > 0) {
      drawBadge(radarText, radarCenterX, radarColor);
    } else if (visionDist > 0) {
      drawBadge(visionText, visionCenterX, visionColor);
    }
  }

  function drawLeadStateBadge(box, text, videoWidth, videoHeight, stageWidth, stageHeight, transform) {
    if (!box?.rect || !text) return;
    const uiScale = ui.getScale(videoWidth, videoHeight);
    const offsets = ui.getBadgeOffsets(videoWidth, videoHeight);
    const visibleRect = ui.getVisibleRect(videoWidth, videoHeight, stageWidth, stageHeight, transform);
    const textY = Math.min(
      box.centerY + offsets.textBaselineOffset,
      Math.max(visibleRect.top + 6 * uiScale, visibleRect.bottom - 6 * uiScale),
    );
    ui.drawText(text, box.centerX, textY, {
      fontSize: Math.max(50 * uiScale, 22),
      fontWeight: 900,
      fillStyle: "#ffffff",
      strokeStyle: "rgba(0,0,0,0.88)",
      strokeWidth: Math.max(4.0 * uiScale, 2.2),
      align: "center",
      baseline: "bottom",
    });
  }

  function drawRadarSpeedBadge(center, text, accentColor, videoWidth, videoHeight) {
    if (!center || !text) return;
    const uiScale = ui.getScale(videoWidth, videoHeight);
    const badgeWidth = Math.max(40 * uiScale, 35 * uiScale * String(text).length);
    const badgeHeight = Math.max(42 * uiScale, 20);
    const badgeX = center.x - badgeWidth * 0.5;
    const badgeY = center.y - 35 * uiScale;
    ui.fillRoundedRect(context, badgeX, badgeY, badgeWidth, badgeHeight, Math.max(15 * uiScale, 7), accentColor);
    ui.drawText(String(text), center.x, center.y, {
      fontSize: Math.max(40 * uiScale, 18),
      fontWeight: 900,
      strokeWidth: Math.max(4.2 * uiScale, 2.2),
    });
  }

  function drawPathStatusText(modelPath, hudState, calibTransform, videoWidth, videoHeight, anchorBox = null) {
    const text = model.getPathStatusText(hudState);
    if (!text) return;
    const anchor = anchorBox || projection.projectPathEndAnchorBox(modelPath, calibTransform, videoWidth, videoHeight);
    if (!anchor) return;

    const scale = ui.getScale(videoWidth, videoHeight);
    const fontSize = Math.max(50 * scale, 22);
    const baselineY = clamp(anchor.centerY + 60 * scale, fontSize + 8 * scale, videoHeight - 10 * scale);
    ui.drawText(text, anchor.centerX, baselineY, {
      fontSize,
      fontWeight: 900,
      fillStyle: "#ffffff",
      strokeStyle: "rgba(0,0,0,0.88)",
      strokeWidth: Math.max(4.0 * scale, 2.2),
      align: "center",
      baseline: "bottom",
    });
  }

  function drawRadarTargets(radarState, sourceModel, calibTransform, videoWidth, videoHeight) {
    const params = getParams();
    const showRadarInfo = finiteNumber(params.ShowRadarInfo, 0);
    if (showRadarInfo <= 0) return;
    const projectionLine = model.getRadarProjectionLine(sourceModel);
    if (!projectionLine) return;
    const radarLatFactor = finiteNumber(params.RadarLatFactor, 0) / 100.0;
    const isMetric = isMetricDisplay();

    for (const radar of model.getRadarTracks(radarState)) {
      const dRel = finiteNumber(radar?.dRel, 0);
      if (!Number.isFinite(dRel) || dRel <= 2.5) continue;

      const yRel = finiteNumber(radar?.yRel, 0);
      const z = projection.samplePathZ(projectionLine, dRel) - 0.61;
      const center = projection.projectPointPrecise(calibTransform, dRel, -yRel, z);
      if (!center) continue;

      const vLead = finiteNumber(radar?.vLeadK, finiteNumber(radar?.vRel, 0));
      const vLat = finiteNumber(radar?.vLat, 0);
      const vAbs = Math.sqrt((vLead * vLead) + (vLat * vLat));
      const vSigned = vLead >= 0 ? vAbs : -vAbs;
      const radarDetected = Boolean(radar?.radar);
      const modelProb = finiteNumber(radar?.modelProb, 0);

      if (vAbs > 3.0) {
        const futureDRel = Math.max(2.0, dRel + vLead * radarLatFactor);
        const futureYRel = yRel + vLat * radarLatFactor;
        const future = Math.abs(vLead) > 3.0
          ? projection.projectPointPrecise(calibTransform, futureDRel, -futureYRel, z)
          : null;
        if (future) {
          const vectorColor = vSigned >= 0 ? "rgba(35,213,93,0.94)" : "rgba(255,59,48,0.94)";
          const uiScale = ui.getScale(videoWidth, videoHeight);
          geometry.drawPolyline([center, future], vectorColor, Math.max(3.0 * uiScale, 1.6));
          geometry.drawPolygon(geometry.circlePolygon(future.x, future.y, Math.max(10 * uiScale, 5)), vectorColor);
        }

        let badgeColor = "rgba(255,59,48,0.96)";
        if (!radarDetected) badgeColor = "rgba(61,123,255,0.96)";
        else if (Math.abs(modelProb - 0.01) < 1e-3) badgeColor = "rgba(35,213,93,0.96)";
        else if (vSigned > 0) badgeColor = "rgba(255,167,38,0.96)";

        const speedValue = vSigned * (isMetric ? 3.6 : 2.2369363);
        drawRadarSpeedBadge(center, speedValue.toFixed(0), badgeColor, videoWidth, videoHeight);

        if (showRadarInfo >= 2) {
          const uiScale = ui.getScale(videoWidth, videoHeight);
          ui.drawText(displayDistance(finiteNumber(radar?.yRel, 0)).toFixed(1), center.x, center.y - 40 * uiScale, {
            fontSize: Math.max(30 * uiScale, 15),
            fontWeight: 900,
            strokeWidth: Math.max(3.8 * uiScale, 2.0),
          });
          ui.drawText(displayDistance(dRel).toFixed(1), center.x, center.y + 30 * uiScale, {
            fontSize: Math.max(30 * uiScale, 15),
            fontWeight: 900,
            strokeWidth: Math.max(3.8 * uiScale, 2.0),
          });
        }
      } else if (showRadarInfo >= 3) {
        const uiScale = ui.getScale(videoWidth, videoHeight);
        ui.drawText("*", center.x, center.y, {
          fontSize: Math.max(40 * uiScale, 18),
          fontWeight: 900,
          strokeWidth: Math.max(4.2 * uiScale, 2.2),
        });
      }
    }
  }

  function draw(sourceModel, overlayState, hudState, calibTransform, videoWidth, videoHeight, stageWidth, stageHeight, transform) {
    const radarState = overlayState?.radarState || {};
    const modelPath = sourceModel?.position || null;
    const leadState = model.resolveLeadState(overlayState, hudState);
    const roadCameraState = overlayState?.roadCameraState || null;
    const leadTwoState = model.resolveLeadTwo(radarState, hudState?.longitudinalPlan || {});
    syncRenderState(videoWidth, videoHeight, sourceModel?.frameId, roadCameraState?.frameId);

    const leadOneBox = projection.projectLeadBox(
      radarState?.leadOne,
      modelPath,
      calibTransform,
      videoWidth,
      videoHeight,
      0,
      stageWidth,
      stageHeight,
      transform,
      {
        includeDistanceBadge: leadState?.showDistanceBadge !== false,
        includeStateText: Boolean(leadState?.text),
      },
    );
    const nowMs = temporal.now();
    let primaryStatusAnchorBox = null;

    if (leadOneBox) {
      const isLeadScc = leadOneBox.radarTrackId < 1;
      const strokeColor = !leadOneBox.radarDetected
        ? "rgba(0,0,255,0.96)"
        : (isLeadScc ? "rgba(255,0,0,0.96)" : "rgba(255,175,3,0.96)");
      eraseLeadBoxOcclusion(leadOneBox, true);
      drawLeadBoxCard(leadOneBox, strokeColor, "rgba(0,0,0,0.20)", true);

      holdState.lastValidMs = nowMs;
      holdState.box = leadOneBox;
      holdState.strokeColor = strokeColor;
      holdState.isLeadScc = isLeadScc;

      if (leadState?.showDistanceBadge !== false) {
        const radarDist = Boolean(radarState?.leadOne?.radar)
          ? Math.max(0, finiteNumber(radarState?.leadOne?.dRel, 0))
          : 0;
        const visionDist = model.getPrimaryVisionDistance(sourceModel);
        holdState.radarDist = radarDist;
        holdState.visionDist = visionDist;
        if (radarDist > 0 || visionDist > 0) {
          const badgeTextColor = model.getDistanceBadgeTextColor(leadState?.xState);
          holdState.badgeTextColor = badgeTextColor;
          drawLeadDistanceBadges(
            leadOneBox,
            radarDist,
            visionDist,
            isLeadScc,
            badgeTextColor,
            videoWidth,
            videoHeight,
            stageWidth,
            stageHeight,
            transform,
          );
        }
      }

      if (leadState?.text) {
        drawLeadStateBadge(leadOneBox, leadState.text, videoWidth, videoHeight, stageWidth, stageHeight, transform);
      }
      primaryStatusAnchorBox = leadOneBox;
    } else if (holdState.box && (nowMs - holdState.lastValidMs) < LEAD_HOLD_MS) {
      const held = holdState;
      eraseLeadBoxOcclusion(held.box, true);
      drawLeadBoxCard(held.box, held.strokeColor, "rgba(0,0,0,0.20)", true);
      if (leadState?.showDistanceBadge !== false && (held.radarDist > 0 || held.visionDist > 0)) {
        drawLeadDistanceBadges(
          held.box,
          held.radarDist,
          held.visionDist,
          held.isLeadScc,
          held.badgeTextColor,
          videoWidth,
          videoHeight,
          stageWidth,
          stageHeight,
          transform,
        );
      }
      if (leadState?.text) {
        drawLeadStateBadge(held.box, leadState.text, videoWidth, videoHeight, stageWidth, stageHeight, transform);
      }
      primaryStatusAnchorBox = held.box;
    } else {
      holdState.box = null;
    }

    if (leadTwoState.valid) {
      const leadTwoBox = projection.projectLeadTwoBox(
        leadTwoState.lead,
        modelPath,
        calibTransform,
        videoWidth,
        videoHeight,
        stageWidth,
        stageHeight,
        transform,
      );
      if (leadTwoBox) {
        eraseLeadBoxOcclusion(leadTwoBox, false);
        drawLeadBoxCard(
          leadTwoBox,
          "rgba(218,111,37,0.96)",
          leadTwoState.status === 2 ? "rgba(255,0,0,0.20)" : "rgba(0,0,0,0.20)",
          false,
        );
      }
    } else {
      temporal.resetLeadSlot(1);
      temporal.resetLeadTwo();
    }

    drawPathStatusText(modelPath, hudState, calibTransform, videoWidth, videoHeight, primaryStatusAnchorBox);
    drawRadarTargets(radarState, sourceModel, calibTransform, videoWidth, videoHeight);
  }

  reset();
  return Object.freeze({ draw, reset, getHeldBox });
}

export const DriveVisionRoadOverlayLeadRenderer = Object.freeze({
  create: createRoadOverlayLeadRenderer,
});

export function installDriveVisionRoadOverlayLeadRendererFacade(target = globalThis) {
  target.DriveVisionRoadOverlayLeadRenderer = DriveVisionRoadOverlayLeadRenderer;
  return DriveVisionRoadOverlayLeadRenderer;
}
