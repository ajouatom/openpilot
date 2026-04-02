"use strict";

window.HomeDrive = (() => {
  const stageEl = document.getElementById("carrotStage");
  const videoEl = document.getElementById("carrotRoadVideo");
  const canvasEl = document.getElementById("carrotOverlayCanvas");
  const hudCanvasEl = document.getElementById("carrotHudCanvas");
  const statusEl = document.getElementById("carrotStageStatus");
  const metaEl = document.getElementById("carrotStageMeta");
  const debugEl = document.getElementById("carrotStageDebug");
  const driveHudCardEl = document.getElementById("driveHudCard");
  const sourceVideoEl = document.getElementById("rtcVideo");
  const zoomButtons = Array.from(document.querySelectorAll(".carrot-zoom__btn"));

  if (!stageEl || !videoEl || !canvasEl || !hudCanvasEl || !statusEl || !metaEl || !debugEl || !sourceVideoEl) {
    return {};
  }

  const ctx = canvasEl.getContext("2d");
  const hudCtx = hudCanvasEl.getContext("2d");
  if (!ctx || !hudCtx) {
    return {};
  }

  // Rendering policy:
  // - Keep geometry/projection in the web renderer.
  // - Mirror native openpilot/Carrot style and animation rules instead of inventing web-only variants.
  // - When UI params exist on-device, treat them as the source of truth so web follows native behavior automatically.
  // Future visual changes should extend the native rule port first, not add disconnected styling here.

  const VIEW_FROM_DEVICE = [
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 0],
  ];
  const BASE_CAMERA = {
    width: 1928,
    height: 1208,
    focalX: 2648,
    focalY: 2648,
  };
  const DISPLAY_MODES = [
    { key: "fit", label: "축소" },
    { key: "normal", label: "정사이즈" },
    { key: "crop", label: "크롭" },
  ];
  const HUD_TEXT_FONT = "system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif";
  const DISPLAY_MODE_STORAGE_KEY = "home_drive_display_mode_index";
  const PATH_PALETTE = [
    { r: 255, g: 82, b: 82 },
    { r: 255, g: 153, b: 0 },
    { r: 255, g: 214, b: 74 },
    { r: 83, g: 220, b: 118 },
    { r: 78, g: 144, b: 255 },
    { r: 0, g: 0, b: 128 },
    { r: 139, g: 0, b: 255 },
    { r: 191, g: 150, b: 87 },
    { r: 255, g: 255, b: 255 },
    { r: 28, g: 28, b: 28 },
  ];
  const PLOT_SERIES = [
    { color: "#f6d94f", label: "Y" },
    { color: "#58d97d", label: "G" },
    { color: "#ff9f43", label: "O" },
  ];
  const PLOT_MAX_POINTS = 240;
  const PATH_HALF_WIDTH = 0.9;
  const PATH_Z_OFFSET = 1.22;
  const MIN_DRAW_DISTANCE = 10;
  const MAX_DRAW_DISTANCE = 100;
  const RADAR_INTERPOLATION_MIN_MS = 16;
  const RADAR_INTERPOLATION_DEFAULT_MS = 50;
  const RADAR_INTERPOLATION_MAX_MS = 120;
  const RADAR_INTERPOLATION_LEAD_MS = 12;
  const TEST_PATH_VISIBILITY_SOLID_ALPHA = 0.50;
  const TEST_PATH_VISIBILITY_MID_ALPHA = 0.24;
  const TEST_LANE_PROB_MIN = 0.003;
  const TEST_LANE_PROB_BOOST = 6;

  const defaultParams = {
    ShowPathEnd: 0,
    ShowLaneInfo: 2,
    ShowPathMode: 0,
    ShowPathColor: 13,
    ShowPathModeLane: 0,
    ShowPathColorLane: 13,
    ShowPathColorCruiseOff: 19,
    ShowPathWidth: 100,
    ShowPlotMode: 0,
    ShowRadarInfo: 0,
    CustomSR: 0,
  };

  let paramsState = { ...defaultParams };
  let displayModeIndex = 1;
  let loopToken = 0;
  let overlaySizeSignature = "";
  let hudSizeSignature = "";
  let transformSignature = "";
  let lastStatus = "";
  let lastMeta = "";
  let lastDebug = "";
  let lastPlotMode = -1;
  let plotHistory = [[], [], []];
  let radarInterpolationState = {
    signature: "",
    previous: null,
    current: null,
    previousAtMs: 0,
    currentAtMs: 0,
  };

  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  function finiteNumber(value, fallback = 0) {
    const num = Number(value);
    return Number.isFinite(num) ? num : fallback;
  }

  function finiteParamNumber(value, fallback = 0) {
    if (value == null) return fallback;
    if (typeof value === "string" && !value.trim()) return fallback;
    const num = Number(value);
    return Number.isFinite(num) ? num : fallback;
  }

  function finiteList(value) {
    if (!Array.isArray(value)) return [];
    return value.map((item) => Number(item)).filter((item) => Number.isFinite(item));
  }

  function firstFinite(values, fallback = 0) {
    if (!Array.isArray(values)) return fallback;
    for (const value of values) {
      const num = Number(value);
      if (Number.isFinite(num)) return num;
    }
    return fallback;
  }

  function shortText(value, maxLength = 88) {
    const text = String(value || "").trim();
    if (!text) return "";
    return text.length > maxLength ? `${text.slice(0, maxLength - 1)}...` : text;
  }

  function rgba(rgb, alpha) {
    return `rgba(${rgb.r}, ${rgb.g}, ${rgb.b}, ${clamp(alpha, 0, 1).toFixed(3)})`;
  }

  function paletteColor(index) {
    const normalized = ((Number(index) % PATH_PALETTE.length) + PATH_PALETTE.length) % PATH_PALETTE.length;
    return PATH_PALETTE[normalized] || PATH_PALETTE[3];
  }

  function mixPoint(a, b, ratio) {
    return {
      x: a.x + (b.x - a.x) * ratio,
      y: a.y + (b.y - a.y) * ratio,
    };
  }

  function mat3Multiply(a, b) {
    return [
      [
        a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
        a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
        a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
      ],
      [
        a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
        a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
        a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
      ],
      [
        a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
        a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
        a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
      ],
    ];
  }

  function mat3Vector(a, v) {
    return [
      a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
      a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
      a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ];
  }

  function rotFromEuler(roll, pitch, yaw) {
    const sr = Math.sin(roll);
    const cr = Math.cos(roll);
    const sp = Math.sin(pitch);
    const cp = Math.cos(pitch);
    const sy = Math.sin(yaw);
    const cy = Math.cos(yaw);

    return [
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp, cp * sr, cp * cr],
    ];
  }

  function getCalibrationMatrix(liveCalibration) {
    const rpy = finiteList(liveCalibration?.rpyCalib);
    if (rpy.length < 3) return VIEW_FROM_DEVICE;
    return mat3Multiply(VIEW_FROM_DEVICE, rotFromEuler(rpy[0], rpy[1], rpy[2]));
  }

  function getIntrinsics(videoWidth, videoHeight) {
    const scaleX = videoWidth / BASE_CAMERA.width;
    const scaleY = videoHeight / BASE_CAMERA.height;
    return [
      [BASE_CAMERA.focalX * scaleX, 0, videoWidth / 2],
      [0, BASE_CAMERA.focalY * scaleY, videoHeight / 2],
      [0, 0, 1],
    ];
  }

  function getDisplayScale(videoWidth, videoHeight, stageWidth, stageHeight) {
    const containScale = Math.min(stageWidth / videoWidth, stageHeight / videoHeight);
    const coverScale = Math.max(stageWidth / videoWidth, stageHeight / videoHeight);
    const mode = DISPLAY_MODES[displayModeIndex] || DISPLAY_MODES[1];
    const isPortrait = stageHeight > stageWidth;

    let scale = containScale;
    if (mode.key === "fit") {
      scale = containScale * 0.94;
    } else if (mode.key === "crop") {
      scale = coverScale;
    } else if (mode.key === "normal" && isPortrait) {
      scale = containScale * 0.985;
    }

    return {
      mode,
      containScale,
      coverScale,
      scale: Math.max(scale, 0.01),
    };
  }

  function getStageTransform(videoWidth, videoHeight, stageWidth, stageHeight, calibration) {
    const intrinsics = getIntrinsics(videoWidth, videoHeight);
    const calibTransform = mat3Multiply(intrinsics, calibration);
    const display = getDisplayScale(videoWidth, videoHeight, stageWidth, stageHeight);
    const scale = display.scale;
    const centerX = intrinsics[0][2];
    const centerY = intrinsics[1][2];
    const infinity = mat3Vector(calibTransform, [1000, 0, 0]);
    const projectedX = infinity[2] > 1e-3 ? infinity[0] / infinity[2] : centerX;
    const projectedY = infinity[2] > 1e-3 ? infinity[1] / infinity[2] : centerY;
    const maxXOffset = Math.max(0, centerX * scale - stageWidth / 2 - 5);
    const maxYOffset = Math.max(0, centerY * scale - stageHeight / 2 - 5);
    const xOffset = clamp((projectedX - centerX) * scale, -maxXOffset, maxXOffset);
    const yOffset = clamp((projectedY - centerY) * scale, -maxYOffset, maxYOffset);

    return {
      calibTransform,
      displayMode: display.mode,
      scale,
      containScale: display.containScale,
      coverScale: display.coverScale,
      tx: (stageWidth / 2 - xOffset) - (centerX * scale),
      ty: (stageHeight / 2 - yOffset) - (centerY * scale),
    };
  }

  function getHudViewportRect(videoWidth, videoHeight, stageWidth, stageHeight, transform) {
    const rawLeft = finiteNumber(transform?.tx, 0);
    const rawTop = finiteNumber(transform?.ty, 0);
    const rawRight = rawLeft + videoWidth * Math.max(finiteNumber(transform?.scale, 1), 0.01);
    const rawBottom = rawTop + videoHeight * Math.max(finiteNumber(transform?.scale, 1), 0.01);
    const left = clamp(Math.min(rawLeft, rawRight), 0, stageWidth);
    const right = clamp(Math.max(rawLeft, rawRight), 0, stageWidth);
    const top = clamp(Math.min(rawTop, rawBottom), 0, stageHeight);
    const bottom = clamp(Math.max(rawTop, rawBottom), 0, stageHeight);

    if (right - left < 2 || bottom - top < 2) {
      return {
        left: 0,
        top: 0,
        right: stageWidth,
        bottom: stageHeight,
        width: stageWidth,
        height: stageHeight,
        centerX: stageWidth / 2,
        centerY: stageHeight / 2,
      };
    }

    return {
      left,
      top,
      right,
      bottom,
      width: right - left,
      height: bottom - top,
      centerX: (left + right) / 2,
      centerY: (top + bottom) / 2,
    };
  }

  function projectPoint(calibTransform, x, y, z) {
    const projected = mat3Vector(calibTransform, [x, y, z]);
    if (!Number.isFinite(projected[2]) || projected[2] <= 1e-3) return null;
    return {
      x: projected[0] / projected[2],
      y: projected[1] / projected[2],
    };
  }

  function getPathLengthIdx(line, maxDistance) {
    const xs = Array.isArray(line?.x) ? line.x : [];
    let maxIdx = 0;
    for (let i = 1; i < xs.length; i += 1) {
      if (Number(xs[i]) > maxDistance) break;
      maxIdx = i;
    }
    return maxIdx;
  }

  function getModelMaxDistance(model) {
    const positionXs = Array.isArray(model?.position?.x) ? model.position.x : [];
    const tailX = finiteNumber(positionXs[positionXs.length - 1], MIN_DRAW_DISTANCE);
    let maxDistance = clamp(tailX, MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

    const leads = Array.isArray(model?.leadsV3) ? model.leadsV3 : [];
    const leadCandidates = leads
      .map((lead) => ({
        x: finiteNumber(lead?.x?.[0], NaN),
        prob: finiteNumber(lead?.prob, 0),
      }))
      .filter((lead) => Number.isFinite(lead.x) && lead.x > 0 && lead.prob > 0.35)
      .sort((a, b) => a.x - b.x);

    if (leadCandidates.length) {
      const leadDistance = leadCandidates[0].x * 2;
      maxDistance = clamp(leadDistance - Math.min(leadDistance * 0.35, 10), 0, maxDistance);
    }
    return maxDistance;
  }

  function buildRibbon(calibTransform, line, halfWidth, zOffset, maxDistance, allowInvert = false, centerShift = 0) {
    const xs = Array.isArray(line?.x) ? line.x : [];
    const ys = Array.isArray(line?.y) ? line.y : [];
    const zs = Array.isArray(line?.z) ? line.z : [];
    const left = [];
    const right = [];
    const center = [];
    const maxIdx = getPathLengthIdx(line, maxDistance);

    for (let i = 0; i <= maxIdx; i += 1) {
      const x = finiteNumber(xs[i], NaN);
      if (!Number.isFinite(x) || x < 0) continue;

      const y = finiteNumber(ys[i], 0) + centerShift;
      const z = finiteNumber(zs[i], 0) + zOffset;
      const leftPt = projectPoint(calibTransform, x, y - halfWidth, z);
      const rightPt = projectPoint(calibTransform, x, y + halfWidth, z);
      const centerPt = projectPoint(calibTransform, x, y, z);
      if (!leftPt || !rightPt || !centerPt) continue;
      if (!allowInvert && center.length && centerPt.y > center[center.length - 1].y) continue;

      left.push(leftPt);
      right.push(rightPt);
      center.push(centerPt);
    }

    return {
      left,
      right,
      center,
      polygon: left.length >= 2 && right.length >= 2 ? left.concat([...right].reverse()) : [],
    };
  }

  function drawPolygon(points, fillStyle, strokeStyle = "", lineWidth = 1) {
    if (!Array.isArray(points) || points.length < 3) return;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i += 1) {
      ctx.lineTo(points[i].x, points[i].y);
    }
    ctx.closePath();
    if (fillStyle) {
      ctx.fillStyle = fillStyle;
      ctx.fill();
    }
    if (strokeStyle) {
      ctx.lineWidth = lineWidth;
      ctx.strokeStyle = strokeStyle;
      ctx.stroke();
    }
  }

  function drawPolyline(points, strokeStyle, lineWidth, dashPattern = [], dashOffset = 0) {
    if (!Array.isArray(points) || points.length < 2) return;
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i += 1) {
      ctx.lineTo(points[i].x, points[i].y);
    }
    if (dashPattern.length) {
      ctx.setLineDash(dashPattern);
      ctx.lineDashOffset = dashOffset;
    }
    ctx.lineWidth = lineWidth;
    ctx.strokeStyle = strokeStyle;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.stroke();
    ctx.restore();
  }

  function buildBandPolygon(left, right, startRatio, endRatio) {
    if (!Array.isArray(left) || !Array.isArray(right) || left.length < 2 || right.length < 2 || left.length !== right.length) {
      return [];
    }

    const first = [];
    const second = [];
    for (let i = 0; i < left.length; i += 1) {
      first.push(mixPoint(left[i], right[i], startRatio));
      second.push(mixPoint(left[i], right[i], endRatio));
    }
    return first.concat(second.reverse());
  }

  function interp1D(x, xs, ys) {
    if (!Array.isArray(xs) || !Array.isArray(ys) || xs.length < 2 || ys.length < 2) return NaN;
    const target = finiteNumber(x, NaN);
    if (!Number.isFinite(target)) return NaN;
    const lastIdx = Math.min(xs.length, ys.length) - 1;
    if (target <= finiteNumber(xs[0], 0)) return finiteNumber(ys[0], NaN);

    for (let i = 1; i <= lastIdx; i += 1) {
      const x0 = finiteNumber(xs[i - 1], NaN);
      const x1 = finiteNumber(xs[i], NaN);
      if (!Number.isFinite(x0) || !Number.isFinite(x1)) continue;
      if (target > x1 && i < lastIdx) continue;

      const y0 = finiteNumber(ys[i - 1], NaN);
      const y1 = finiteNumber(ys[i], NaN);
      if (!Number.isFinite(y0) || !Number.isFinite(y1)) return NaN;
      if (Math.abs(x1 - x0) < 1e-5) return y1;

      const ratio = clamp((target - x0) / (x1 - x0), 0, 1);
      return y0 + (y1 - y0) * ratio;
    }

    return finiteNumber(ys[lastIdx], NaN);
  }

  function normalizeVisualParams(values, fallback = defaultParams) {
    const source = values && typeof values === "object" ? values : {};
    return {
      ShowPathEnd: finiteParamNumber(source.ShowPathEnd, fallback.ShowPathEnd),
      ShowLaneInfo: finiteParamNumber(source.ShowLaneInfo, fallback.ShowLaneInfo),
      ShowPathMode: finiteParamNumber(source.ShowPathMode, fallback.ShowPathMode),
      ShowPathColor: finiteParamNumber(source.ShowPathColor, fallback.ShowPathColor),
      ShowPathModeLane: finiteParamNumber(source.ShowPathModeLane, fallback.ShowPathModeLane),
      ShowPathColorLane: finiteParamNumber(source.ShowPathColorLane, fallback.ShowPathColorLane),
      ShowPathColorCruiseOff: finiteParamNumber(source.ShowPathColorCruiseOff, fallback.ShowPathColorCruiseOff),
      ShowPathWidth: finiteParamNumber(source.ShowPathWidth, fallback.ShowPathWidth),
      ShowPlotMode: finiteParamNumber(source.ShowPlotMode, fallback.ShowPlotMode),
      ShowRadarInfo: finiteParamNumber(source.ShowRadarInfo, fallback.ShowRadarInfo),
      CustomSR: finiteParamNumber(source.CustomSR, fallback.CustomSR),
    };
  }

  function readLiveRuntimeParams() {
    const runtimeParams = window.CarrotLiveRuntimeState?.runtime?.params;
    if (!runtimeParams || typeof runtimeParams !== "object") return null;

    const normalized = normalizeVisualParams(runtimeParams, paramsState);
    const hasPathKeys = (
      runtimeParams.ShowPathEnd != null ||
      runtimeParams.ShowPathMode != null ||
      runtimeParams.ShowPathColor != null ||
      runtimeParams.ShowPathModeLane != null ||
      runtimeParams.ShowPathColorLane != null ||
      runtimeParams.ShowLaneInfo != null ||
      runtimeParams.ShowRadarInfo != null ||
      runtimeParams.ShowPlotMode != null
    );
    if (!hasPathKeys) return null;
    return normalized;
  }

  function readLiveRuntimeServices() {
    const services = window.CarrotLiveRuntimeState?.services;
    return services && typeof services === "object" ? services : {};
  }

  function mergeServiceState(rawState, liveState) {
    const raw = rawState && typeof rawState === "object" ? rawState : {};
    const live = liveState && typeof liveState === "object" ? liveState : {};
    return { ...raw, ...live };
  }

  function mergeDefinedState(baseState, preferredState) {
    const merged = baseState && typeof baseState === "object" ? { ...baseState } : {};
    if (!preferredState || typeof preferredState !== "object") return merged;
    for (const [key, value] of Object.entries(preferredState)) {
      if (value !== undefined && value !== null) merged[key] = value;
    }
    return merged;
  }

  function mergeRadarLead(rawLead, liveLead) {
    return mergeDefinedState(liveLead, rawLead);
  }

  function mergeRadarState(rawState, liveState) {
    const raw = rawState && typeof rawState === "object" ? rawState : {};
    const live = liveState && typeof liveState === "object" ? liveState : {};

    return {
      ...live,
      ...raw,
      leadOne: mergeRadarLead(raw.leadOne, live.leadOne),
      leadTwo: mergeRadarLead(raw.leadTwo, live.leadTwo),
      leadLeft: mergeRadarLead(raw.leadLeft, live.leadLeft),
      leadRight: mergeRadarLead(raw.leadRight, live.leadRight),
      leadsLeft: Array.isArray(raw.leadsLeft) && raw.leadsLeft.length ? raw.leadsLeft : live.leadsLeft,
      leadsCenter: Array.isArray(raw.leadsCenter) && raw.leadsCenter.length ? raw.leadsCenter : live.leadsCenter,
      leadsRight: Array.isArray(raw.leadsRight) && raw.leadsRight.length ? raw.leadsRight : live.leadsRight,
    };
  }

  function cloneRadarLead(lead) {
    if (!lead || typeof lead !== "object") return null;
    return {
      dRel: finiteNumber(lead.dRel, 0),
      yRel: finiteNumber(lead.yRel, 0),
      vRel: finiteNumber(lead.vRel, 0),
      aRel: finiteNumber(lead.aRel, 0),
      vLead: finiteNumber(lead.vLead, 0),
      dPath: finiteNumber(lead.dPath, 0),
      vLat: finiteNumber(lead.vLat, 0),
      vLeadK: finiteNumber(lead.vLeadK, 0),
      aLead: finiteNumber(lead.aLead, 0),
      aLeadK: finiteNumber(lead.aLeadK, 0),
      aLeadTau: finiteNumber(lead.aLeadTau, 0),
      modelProb: finiteNumber(lead.modelProb, 0),
      score: finiteNumber(lead.score, 0),
      jLead: finiteNumber(lead.jLead, 0),
      fcw: Boolean(lead.fcw),
      status: Boolean(lead.status),
      radar: Boolean(lead.radar),
      radarTrackId: finiteNumber(lead.radarTrackId, -1),
    };
  }

  function cloneRadarState(radarState) {
    const source = radarState && typeof radarState === "object" ? radarState : {};
    return {
      ...source,
      leadOne: cloneRadarLead(source.leadOne),
      leadTwo: cloneRadarLead(source.leadTwo),
      leadLeft: cloneRadarLead(source.leadLeft),
      leadRight: cloneRadarLead(source.leadRight),
      leadsLeft: Array.isArray(source.leadsLeft) ? source.leadsLeft.slice() : source.leadsLeft,
      leadsCenter: Array.isArray(source.leadsCenter) ? source.leadsCenter.slice() : source.leadsCenter,
      leadsRight: Array.isArray(source.leadsRight) ? source.leadsRight.slice() : source.leadsRight,
    };
  }

  function radarLeadSignature(lead) {
    if (!lead || typeof lead !== "object") return "null";
    return [
      Boolean(lead.status) ? 1 : 0,
      Boolean(lead.radar) ? 1 : 0,
      finiteNumber(lead.radarTrackId, -1),
      finiteNumber(lead.dRel, 0).toFixed(3),
      finiteNumber(lead.yRel, 0).toFixed(3),
      finiteNumber(lead.vRel, 0).toFixed(3),
      finiteNumber(lead.modelProb, 0).toFixed(3),
      finiteNumber(lead.score, 0).toFixed(3),
    ].join("|");
  }

  function radarStateSignature(radarState) {
    const source = radarState && typeof radarState === "object" ? radarState : {};
    return [
      radarLeadSignature(source.leadOne),
      radarLeadSignature(source.leadTwo),
      radarLeadSignature(source.leadLeft),
      radarLeadSignature(source.leadRight),
    ].join("||");
  }

  function lerpNumber(a, b, t) {
    return a + (b - a) * t;
  }

  function canInterpolateRadarLead(previousLead, currentLead) {
    if (!previousLead || !currentLead) return false;
    if (!previousLead.status || !currentLead.status) return false;

    const previousTrackId = finiteNumber(previousLead.radarTrackId, -1);
    const currentTrackId = finiteNumber(currentLead.radarTrackId, -1);
    if (previousTrackId >= 0 && currentTrackId >= 0) {
      return previousTrackId === currentTrackId;
    }

    const distanceDelta = Math.abs(finiteNumber(previousLead.dRel, 0) - finiteNumber(currentLead.dRel, 0));
    const lateralDelta = Math.abs(finiteNumber(previousLead.yRel, 0) - finiteNumber(currentLead.yRel, 0));
    return distanceDelta < 12 && lateralDelta < 2.5;
  }

  function lerpRadarLead(previousLead, currentLead, t) {
    if (!previousLead) return cloneRadarLead(currentLead);
    if (!currentLead) return cloneRadarLead(previousLead);
    if (!canInterpolateRadarLead(previousLead, currentLead)) {
      return cloneRadarLead(currentLead);
    }

    return {
      dRel: lerpNumber(previousLead.dRel, currentLead.dRel, t),
      yRel: lerpNumber(previousLead.yRel, currentLead.yRel, t),
      vRel: lerpNumber(previousLead.vRel, currentLead.vRel, t),
      aRel: lerpNumber(previousLead.aRel, currentLead.aRel, t),
      vLead: lerpNumber(previousLead.vLead, currentLead.vLead, t),
      dPath: lerpNumber(previousLead.dPath, currentLead.dPath, t),
      vLat: lerpNumber(previousLead.vLat, currentLead.vLat, t),
      vLeadK: lerpNumber(previousLead.vLeadK, currentLead.vLeadK, t),
      aLead: lerpNumber(previousLead.aLead, currentLead.aLead, t),
      aLeadK: lerpNumber(previousLead.aLeadK, currentLead.aLeadK, t),
      aLeadTau: lerpNumber(previousLead.aLeadTau, currentLead.aLeadTau, t),
      modelProb: lerpNumber(previousLead.modelProb, currentLead.modelProb, t),
      score: lerpNumber(previousLead.score, currentLead.score, t),
      jLead: lerpNumber(previousLead.jLead, currentLead.jLead, t),
      fcw: t < 0.5 ? previousLead.fcw : currentLead.fcw,
      status: t < 0.5 ? previousLead.status : currentLead.status,
      radar: t < 0.5 ? previousLead.radar : currentLead.radar,
      radarTrackId: t < 0.5 ? previousLead.radarTrackId : currentLead.radarTrackId,
    };
  }

  function getInterpolatedRadarState(radarState, nowMs) {
    const signature = radarStateSignature(radarState);
    if (!radarInterpolationState.current) {
      const initial = cloneRadarState(radarState);
      radarInterpolationState = {
        signature,
        previous: initial,
        current: initial,
        previousAtMs: nowMs,
        currentAtMs: nowMs,
      };
      return initial;
    }

    if (signature !== radarInterpolationState.signature) {
      radarInterpolationState = {
        signature,
        previous: radarInterpolationState.current,
        current: cloneRadarState(radarState),
        previousAtMs: radarInterpolationState.currentAtMs || nowMs,
        currentAtMs: nowMs,
      };
    }

    const previous = radarInterpolationState.previous || radarInterpolationState.current;
    const current = radarInterpolationState.current || cloneRadarState(radarState);
    if (!previous || !current) return radarState;
    if (previous === current) return current;

    const intervalMs = clamp(
      radarInterpolationState.currentAtMs - radarInterpolationState.previousAtMs || RADAR_INTERPOLATION_DEFAULT_MS,
      RADAR_INTERPOLATION_MIN_MS,
      RADAR_INTERPOLATION_MAX_MS,
    );
    const t = clamp((nowMs - radarInterpolationState.currentAtMs + RADAR_INTERPOLATION_LEAD_MS) / intervalMs, 0, 1);

    return {
      ...current,
      leadOne: lerpRadarLead(previous.leadOne, current.leadOne, t),
      leadTwo: lerpRadarLead(previous.leadTwo, current.leadTwo, t),
      leadLeft: lerpRadarLead(previous.leadLeft, current.leadLeft, t),
      leadRight: lerpRadarLead(previous.leadRight, current.leadRight, t),
    };
  }

  function mergeRuntimeState(rawHudState, rawOverlayState) {
    const liveServices = readLiveRuntimeServices();
    const radarState = mergeRadarState(rawOverlayState?.radarState, liveServices.radarState);
    const mergedHudState = {
      ...rawHudState,
      carState: mergeServiceState(rawHudState?.carState, liveServices.carState),
      controlsState: mergeServiceState(rawHudState?.controlsState, liveServices.controlsState),
      selfdriveState: mergeServiceState(rawHudState?.selfdriveState, liveServices.selfdriveState),
      longitudinalPlan: mergeServiceState(rawHudState?.longitudinalPlan, liveServices.longitudinalPlan),
      carrotMan: mergeServiceState(rawHudState?.carrotMan, liveServices.carrotMan),
      lateralPlan: mergeServiceState(rawOverlayState?.lateralPlan, liveServices.lateralPlan),
      radarState,
    };

    const mergedOverlayState = {
      ...rawOverlayState,
      radarState: mergedHudState.radarState,
      lateralPlan: mergedHudState.lateralPlan,
      carrotMan: mergedHudState.carrotMan,
    };

    return {
      brokerServices: liveServices,
      hudState: mergedHudState,
      overlayState: mergedOverlayState,
    };
  }

  function firstNonEmptyText(...values) {
    for (const value of values) {
      const text = String(value || "").trim();
      if (text) return text;
    }
    return "";
  }

  function setStatus(text) {
    if (lastStatus === text) return;
    lastStatus = text;
    statusEl.textContent = text;
  }

  function setMeta(text) {
    if (lastMeta === text) return;
    lastMeta = text;
    metaEl.textContent = text;
  }

  function setDebug(text) {
    if (lastDebug === text) return;
    lastDebug = text;
    debugEl.textContent = text;
  }

  function syncDisplayModeButtons() {
    zoomButtons.forEach((button, index) => {
      button.classList.toggle("is-active", index === displayModeIndex);
    });
  }

  function setDisplayModeIndex(nextIndex) {
    displayModeIndex = clamp(nextIndex, 0, DISPLAY_MODES.length - 1);
    try {
      localStorage.setItem(DISPLAY_MODE_STORAGE_KEY, String(displayModeIndex));
    } catch {}
    transformSignature = "";
    syncDisplayModeButtons();
  }

  function syncSourceStream() {
    const stream = sourceVideoEl.srcObject || null;
    if (videoEl.srcObject !== stream) {
      videoEl.srcObject = stream;
    }

    const hasStream = Boolean(stream);
    if (hasStream && videoEl.paused) {
      videoEl.play().catch(() => {});
    }
    return hasStream;
  }

  function setStageReady(ready) {
    stageEl.classList.toggle("is-stream-ready", Boolean(ready));
    videoEl.style.display = ready ? "block" : "none";
  }

  function resetCarrotHudLayout() {
    if (!driveHudCardEl) return;
    driveHudCardEl.style.removeProperty("--carrot-hud-left");
    driveHudCardEl.style.removeProperty("--carrot-hud-bottom");
  }

  function applyCarrotHudLayout(viewportRect) {
    if (!driveHudCardEl) return;
    if (window.matchMedia("(orientation: portrait)").matches) {
      driveHudCardEl.style.removeProperty("--carrot-hud-left");
      driveHudCardEl.style.removeProperty("--carrot-hud-bottom");
      return;
    }
    const stageRect = stageEl.getBoundingClientRect();
    if (!stageRect.width || !stageRect.height) return;
    const overlayInsetX = clamp(stageRect.width * 0.028, 16, 28);
    const overlayInsetY = clamp(stageRect.height * 0.034, 16, 24);
    const stageLeft = Math.round(overlayInsetX);
    const stageBottom = Math.round(overlayInsetY);

    driveHudCardEl.style.setProperty("--carrot-hud-left", `${stageLeft}px`);
    driveHudCardEl.style.setProperty("--carrot-hud-bottom", `${stageBottom}px`);
  }

  function syncCanvasSize(videoWidth, videoHeight, stageWidth, stageHeight) {
    const dpr = window.devicePixelRatio || 1;

    const nextOverlaySignature = `${videoWidth}x${videoHeight}@${dpr.toFixed(2)}`;
    if (overlaySizeSignature !== nextOverlaySignature) {
      overlaySizeSignature = nextOverlaySignature;
      videoEl.style.width = `${videoWidth}px`;
      videoEl.style.height = `${videoHeight}px`;
      canvasEl.style.width = `${videoWidth}px`;
      canvasEl.style.height = `${videoHeight}px`;
      canvasEl.width = Math.max(1, Math.round(videoWidth * dpr));
      canvasEl.height = Math.max(1, Math.round(videoHeight * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    const nextHudSignature = `${stageWidth}x${stageHeight}@${dpr.toFixed(2)}`;
    if (hudSizeSignature !== nextHudSignature) {
      hudSizeSignature = nextHudSignature;
      hudCanvasEl.style.width = `${stageWidth}px`;
      hudCanvasEl.style.height = `${stageHeight}px`;
      hudCanvasEl.width = Math.max(1, Math.round(stageWidth * dpr));
      hudCanvasEl.height = Math.max(1, Math.round(stageHeight * dpr));
      hudCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
  }

  function applyStageTransform(transform) {
    const nextSignature = `${transform.scale.toFixed(6)}|${transform.tx.toFixed(2)}|${transform.ty.toFixed(2)}`;
    if (transformSignature === nextSignature) return;

    transformSignature = nextSignature;
    const cssMatrix = `matrix(${transform.scale}, 0, 0, ${transform.scale}, ${transform.tx}, ${transform.ty})`;
    videoEl.style.transform = cssMatrix;
    canvasEl.style.transform = cssMatrix;
  }

  function clearOverlay(videoWidth, videoHeight) {
    ctx.clearRect(0, 0, videoWidth, videoHeight);
  }

  function clearHud(stageWidth, stageHeight) {
    hudCtx.clearRect(0, 0, stageWidth, stageHeight);
  }

  function fitSingleLineHudFontSize(text, preferredSize, maxWidth, minSize = 4.5, fontWeight = 900) {
    const label = String(text || "").trim();
    if (!label) return preferredSize;
    let fontSize = preferredSize;
    hudCtx.save();
    hudCtx.font = `${fontWeight} ${fontSize}px ${HUD_TEXT_FONT}`;
    const measured = hudCtx.measureText(label).width;
    hudCtx.restore();
    if (measured > maxWidth && measured > 1.0) {
      fontSize = clamp(fontSize * ((maxWidth / measured) * 0.985), minSize, fontSize);
    }
    return fontSize;
  }

  function drawOutlinedHudText({
    text,
    x,
    y,
    color = "rgba(244, 244, 244, 0.94)",
    strokeColor = "rgba(0, 0, 0, 0.94)",
    strokeWidth = 3,
    fontSize = 24,
    fontWeight = 900,
    alignX = "left",
    alignY = "top",
    maxWidth,
  }) {
    const label = String(text || "").trim();
    if (!label) return;

    hudCtx.save();
    hudCtx.font = `${fontWeight} ${fontSize}px ${HUD_TEXT_FONT}`;
    hudCtx.fillStyle = color;
    hudCtx.strokeStyle = strokeColor;
    hudCtx.lineWidth = strokeWidth;
    hudCtx.lineJoin = "round";
    hudCtx.miterLimit = 2;
    hudCtx.textAlign = alignX === "center" ? "center" : alignX === "right" ? "right" : "left";
    hudCtx.textBaseline = alignY === "middle" ? "middle" : alignY === "bottom" ? "bottom" : alignY === "baselineBottom" ? "alphabetic" : "top";
    if (strokeWidth > 0) {
      hudCtx.strokeText(label, x, y, maxWidth);
    }
    hudCtx.fillText(label, x, y, maxWidth);
    hudCtx.restore();
  }

  function drawSegmentBands(left, right, style, step) {
    if (!Array.isArray(left) || left.length < 3 || left.length !== right.length) return;
    const baseColor = paletteColor(style.paletteIndex);
    const stroke = style.emphasisStroke ? style.strokeColor : "";
    for (let i = 0; i < left.length - 2; i += step) {
      const next = Math.min(i + 2, left.length - 1);
      const segment = [left[i], left[next], right[next], right[i]];
      drawPolygon(segment, rgba(baseColor, 0.28), stroke, 1.1);
    }
  }

  function createPathGradient(baseColor, canvasHeight, style) {
    const gradient = ctx.createLinearGradient(0, canvasHeight, 0, canvasHeight * 0.32);
    const mode = finiteNumber(style?.mode, 0);
    let solidAlpha = mode === 0 ? 0.40 : 0.24;
    let midAlpha = mode === 0 ? 0.20 : 0.12;

    // Test-page visibility floor:
    // keep native path mode/color selection intact, but avoid a nearly invisible
    // ribbon during stationary or indoor checks where only the web test view is used.
    if (mode === 0 || style?.isCruiseOff) {
      solidAlpha = Math.max(solidAlpha, TEST_PATH_VISIBILITY_SOLID_ALPHA);
      midAlpha = Math.max(midAlpha, TEST_PATH_VISIBILITY_MID_ALPHA);
    }

    gradient.addColorStop(0, rgba(baseColor, solidAlpha));
    gradient.addColorStop(0.55, rgba(baseColor, midAlpha));
    gradient.addColorStop(1, rgba(baseColor, 0.0));
    return gradient;
  }

  function drawPathRibbon(ribbon, style, canvasHeight) {
    if (!ribbon.polygon.length) return;
    const baseColor = paletteColor(style.paletteIndex);
    if (style.mode === 0) {
      drawPolygon(
        ribbon.polygon,
        rgba(baseColor, 0.42),
        style.emphasisStroke ? style.strokeColor : "",
        style.emphasisStroke ? 2.0 : 0,
      );
      return;
    }
    const fill = createPathGradient(baseColor, canvasHeight, style);
    drawPolygon(ribbon.polygon, fill, style.emphasisStroke ? style.strokeColor : "", style.emphasisStroke ? 1.7 : 0);
  }

  function drawAnimatedPath(ribbon, style) {
    if (!ribbon.center.length) return;
    const dashPresets = {
      1: [14, 11],
      2: [10, 8],
      3: [26, 12],
      4: [18, 10, 5, 10],
      5: [30, 12],
      6: [12, 7],
      7: [20, 8, 4, 8],
      8: [8, 6],
    };
    const baseColor = paletteColor(style.paletteIndex);
    const dash = dashPresets[style.mode] || dashPresets[1];
    const dashLength = dash.reduce((sum, value) => sum + value, 0);
    const dashOffset = -((performance.now() / 120) % dashLength);
    drawPolyline(ribbon.center, rgba(baseColor, 0.78), 5.5, dash, dashOffset);
    if (style.emphasisStroke) {
      drawPolyline(ribbon.center, style.strokeColor, 1.6);
    }
  }

  function drawComplexPath(ribbon, style) {
    const step = style.mode === 9 ? 3 : style.mode === 10 ? 4 : style.mode === 11 ? 5 : 6;
    drawSegmentBands(ribbon.left, ribbon.right, style, step);
  }

  function drawSpecialPath(ribbon, style) {
    const baseColor = paletteColor(style.paletteIndex);
    const bands = [];
    if (style.mode === 13 || style.mode === 14) {
      bands.push(buildBandPolygon(ribbon.left, ribbon.right, 0.0, 0.18));
      bands.push(buildBandPolygon(ribbon.left, ribbon.right, 0.82, 1.0));
    }
    if (style.mode === 13 || style.mode === 15) {
      bands.push(buildBandPolygon(ribbon.left, ribbon.right, 0.38, 0.62));
    }

    for (const band of bands) {
      drawPolygon(band, rgba(baseColor, 0.34), style.emphasisStroke ? style.strokeColor : "", style.emphasisStroke ? 1.4 : 0);
    }
  }

  function getPathHalfWidth() {
    const widthRatio = clamp(finiteNumber(paramsState.ShowPathWidth, 100) / 100, 0.1, 2.0);
    return PATH_HALF_WIDTH * widthRatio;
  }

  function drawPath(pathData, model, calibTransform, canvasHeight, style) {
    if (!pathData || !Array.isArray(pathData.x) || !pathData.x.length) return;
    const ribbon = buildRibbon(calibTransform, pathData, getPathHalfWidth(), PATH_Z_OFFSET, getModelMaxDistance(model), false);
    if (ribbon.polygon.length < 3) return;

    drawPathRibbon(ribbon, style, canvasHeight);
    if (style.mode === 0) return;
    if (style.mode >= 13 && style.mode <= 15) {
      drawSpecialPath(ribbon, style);
      return;
    }
    if (style.mode >= 9) {
      drawComplexPath(ribbon, style);
      return;
    }
    drawAnimatedPath(ribbon, style);
  }

  function drawLaneLines(model, hudState, calibTransform) {
    const laneLines = Array.isArray(model?.laneLines) ? model.laneLines : [];
    const laneLineProbs = Array.isArray(model?.laneLineProbs) ? model.laneLineProbs : [];
    if (!laneLines.length) return;

    const leftLaneLine = finiteNumber(hudState?.carState?.leftLaneLine, 0);
    const rightLaneLine = finiteNumber(hudState?.carState?.rightLaneLine, 0);
    const maxIdx = getPathLengthIdx(laneLines[0], getModelMaxDistance(model));
    for (let i = 0; i < laneLines.length; i += 1) {
      const prob = clamp(finiteNumber(laneLineProbs[i], 0), 0, 0.9);
      const renderProb = prob >= 0.02 ? prob : (prob >= TEST_LANE_PROB_MIN ? clamp(prob * TEST_LANE_PROB_BOOST, 0.02, 0.12) : 0);
      if (renderProb <= 0) continue;

      // Keep native probability-driven lanes, but allow a small test-page floor so
      // very low-confidence indoor/stationary lanes are still inspectable.
      const highlightedLeft = i === 1 && leftLaneLine >= 20;
      const highlightedRight = i === 2 && rightLaneLine >= 20;
      const laneColor = highlightedLeft || highlightedRight ? { r: 255, g: 217, b: 94 } : { r: 255, g: 255, b: 255 };
      const halfWidth = Math.max(highlightedLeft || highlightedRight ? 0.025 : 0.010, 0.025 * renderProb);
      const fillAlpha = prob >= 0.02 ? clamp(renderProb, 0.12, 0.7) : clamp(renderProb * 3.0, 0.16, 0.26);
      const strokeAlpha = prob >= 0.02 ? 0.20 : 0.24;
      const ribbon = buildRibbon(calibTransform, laneLines[i], halfWidth, 0, finiteNumber(laneLines[i]?.x?.[maxIdx], MAX_DRAW_DISTANCE), false);
      drawPolygon(
        ribbon.polygon,
        `rgba(${laneColor.r},${laneColor.g},${laneColor.b},${fillAlpha.toFixed(3)})`,
        `rgba(${laneColor.r},${laneColor.g},${laneColor.b},${strokeAlpha.toFixed(3)})`,
        1,
      );

      if ((i === 1 && (leftLaneLine % 10) === 4) || (i === 2 && (rightLaneLine % 10) === 4)) {
        const shift = i === 1 ? -0.3 : 0.3;
        const doubleRibbon = buildRibbon(
          calibTransform,
          laneLines[i],
          halfWidth,
          0,
          finiteNumber(laneLines[i]?.x?.[maxIdx], MAX_DRAW_DISTANCE),
          false,
          shift,
        );
        drawPolygon(
          doubleRibbon.polygon,
          `rgba(${laneColor.r},${laneColor.g},${laneColor.b},${fillAlpha.toFixed(3)})`,
          `rgba(${laneColor.r},${laneColor.g},${laneColor.b},${strokeAlpha.toFixed(3)})`,
          1,
        );
      }
    }
  }

  function drawRoadEdges(model, calibTransform) {
    const roadEdges = Array.isArray(model?.roadEdges) ? model.roadEdges : [];
    const roadEdgeStds = Array.isArray(model?.roadEdgeStds) ? model.roadEdgeStds : [];
    if (!roadEdges.length) return;

    const maxIdx = getPathLengthIdx(roadEdges[0], getModelMaxDistance(model));
    for (let i = 0; i < roadEdges.length; i += 1) {
      const edgeStd = clamp(finiteNumber(roadEdgeStds[i], 0.4), 0, 1);
      const alpha = clamp(1 - edgeStd, 0.12, 0.66);
      const ribbon = buildRibbon(calibTransform, roadEdges[i], 0.025, 0, finiteNumber(roadEdges[i]?.x?.[maxIdx], MAX_DRAW_DISTANCE), false);
      drawPolygon(
        ribbon.polygon,
        `rgba(255,78,59,${alpha.toFixed(3)})`,
        "rgba(255,124,104,0.28)",
        1,
      );
    }
  }

  function samplePathZ(position, distance) {
    const zs = Array.isArray(position?.z) ? position.z : [];
    const idx = getPathLengthIdx(position, distance);
    return finiteNumber(zs[idx], 0);
  }

  function samplePathY(position, distance) {
    return interp1D(
      distance,
      Array.isArray(position?.x) ? position.x : [],
      Array.isArray(position?.y) ? position.y : [],
    );
  }

  function circlePolygon(cx, cy, radius, points = 12) {
    const polygon = [];
    for (let i = 0; i < points; i += 1) {
      const theta = (Math.PI * 2 * i) / points;
      polygon.push({
        x: cx + Math.cos(theta) * radius,
        y: cy + Math.sin(theta) * radius,
      });
    }
    return polygon;
  }

  function buildVerticalRibbon(calibTransform, line, centerShift, topZOffset, bottomZOffset, maxDistance) {
    const xs = Array.isArray(line?.x) ? line.x : [];
    const ys = Array.isArray(line?.y) ? line.y : [];
    const zs = Array.isArray(line?.z) ? line.z : [];
    const top = [];
    const bottom = [];
    const maxIdx = getPathLengthIdx(line, maxDistance);

    for (let i = 0; i <= maxIdx; i += 1) {
      const x = finiteNumber(xs[i], NaN);
      if (!Number.isFinite(x) || x < 0) continue;

      const y = finiteNumber(ys[i], 0) + centerShift;
      const z = finiteNumber(zs[i], 0);
      const topPoint = projectPoint(calibTransform, x, y, z + topZOffset);
      const bottomPoint = projectPoint(calibTransform, x, y, z + bottomZOffset);
      if (!topPoint || !bottomPoint) continue;
      if (top.length && topPoint.y > top[top.length - 1].y) continue;

      top.push(topPoint);
      bottom.unshift(bottomPoint);
    }

    return top.length >= 2 && bottom.length >= 2 ? top.concat(bottom) : [];
  }

  function hasNearbyAssistLead(lead, speedMps) {
    const speed = finiteNumber(speedMps, 0);
    if (speed <= 0) return false;
    const threshold = speed * 3.0;
    return Boolean(lead?.status) && finiteNumber(lead?.dRel, Infinity) > 0 && finiteNumber(lead?.dRel, Infinity) < threshold;
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
    const leftAssistWarn = !leftBlindspot && laneChangeState === 1 && laneChangeDirection === 1 && hasNearbyAssistLead(radarState?.leadLeft, speedMps);
    const rightAssistWarn = !rightBlindspot && laneChangeState === 1 && laneChangeDirection === 2 && hasNearbyAssistLead(radarState?.leadRight, speedMps);
    if (!leftBlindspot && !rightBlindspot && !leftAssistWarn && !rightAssistWarn) return;

    const goldFill = "rgba(255, 215, 0, 0.48)";
    const goldStroke = "rgba(255, 215, 0, 0.84)";
    const greenFill = "rgba(0, 204, 0, 0.44)";
    const greenStroke = "rgba(0, 204, 0, 0.80)";

    const drawRibbon = (shift, fill, stroke) => {
      const ribbon = buildVerticalRibbon(calibTransform, modelPath, shift, 1.15, 0.60, 40);
      if (ribbon.length < 8) return;
      drawPolygon(ribbon, fill, stroke, 1.2);
    };

    if (leftBlindspot) drawRibbon(-1.7, goldFill, goldStroke);
    else if (leftAssistWarn) drawRibbon(-1.7, greenFill, greenStroke);

    if (rightBlindspot) drawRibbon(1.7, goldFill, goldStroke);
    else if (rightAssistWarn) drawRibbon(1.7, greenFill, greenStroke);
  }

  function projectLeadBox(lead, modelPath, calibTransform, videoWidth, videoHeight) {
    if (!lead?.status) return null;
    const distance = finiteNumber(lead.dRel, NaN);
    if (!Number.isFinite(distance) || distance <= 0) return null;

    const yCenter = -finiteNumber(lead.yRel, 0);
    const z = samplePathZ(modelPath, distance) + PATH_Z_OFFSET;
    const left = projectPoint(calibTransform, distance, yCenter - 1.2, z);
    const right = projectPoint(calibTransform, distance, yCenter + 1.2, z);
    if (!left || !right) return null;

    const rawWidth = Math.abs(right.x - left.x);
    if (!Number.isFinite(rawWidth) || rawWidth <= 1) return null;

    const centerX = (left.x + right.x) * 0.5;
    const centerY = (left.y + right.y) * 0.5;
    const width = clamp(rawWidth, 120, 800);
    const sidePad = 10;
    const height = Math.max(width * 0.8, 12);
    const marginX = Math.min(videoWidth * 0.35, 350);
    const topMargin = Math.min(videoHeight * 0.28, 200);
    const bottomMargin = Math.max(videoHeight * 0.14, 80);
    const clampedCenterX = clamp(centerX, marginX, Math.max(marginX, videoWidth - marginX));
    const clampedCenterY = clamp(centerY, topMargin, Math.max(topMargin, videoHeight - bottomMargin));

    return {
      rect: {
        x: clampedCenterX - width * 0.5 - sidePad,
        y: clampedCenterY - height,
        width: width + sidePad * 2,
        height,
      },
      centerX: clampedCenterX,
      centerY: clampedCenterY,
      radar: Boolean(lead.radar),
      radarTrackId: finiteNumber(lead.radarTrackId, -99999),
      dRel: distance,
      modelProb: finiteNumber(lead.modelProb, 0),
    };
  }

  function drawLeadBoxCard(box, strokeColor, fillColor, primary = true) {
    if (!box?.rect) return;
    const { x, y, width, height } = box.rect;
    fillRoundedRect(ctx, x, y, width, height, primary ? 16 : 12, fillColor);
    strokeRoundedRect(ctx, x, y, width, height, primary ? 16 : 12, strokeColor, primary ? 3.2 : 2.2);
    if (primary) {
      strokeRoundedRect(ctx, x + 5, y + 5, width - 10, height - 10, 12, "rgba(255,255,255,0.12)", 1.2);
    }
  }

  function drawLeadDistanceBadge(box, text, accentColor, textColor = "#ffffff") {
    if (!box?.rect || !text) return;
    const badgeWidth = 72;
    const badgeHeight = 34;
    const badgeX = box.centerX - badgeWidth * 0.5;
    const badgeY = box.rect.y - badgeHeight - 12;
    fillRoundedRect(ctx, badgeX, badgeY, badgeWidth, badgeHeight, 14, "rgba(18, 23, 31, 0.80)");
    strokeRoundedRect(ctx, badgeX, badgeY, badgeWidth, badgeHeight, 14, accentColor, 2.2);
    ctx.save();
    ctx.font = `800 22px ${HUD_TEXT_FONT}`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.lineJoin = "round";
    ctx.strokeStyle = "rgba(0,0,0,0.82)";
    ctx.fillStyle = textColor;
    ctx.lineWidth = 4;
    ctx.strokeText(text, box.centerX, badgeY + badgeHeight * 0.54);
    ctx.fillText(text, box.centerX, badgeY + badgeHeight * 0.54);
    ctx.restore();
  }

  function leadStateAccentColor(xState) {
    switch (xState) {
      case 3:
      case 5:
        return "rgba(255, 167, 38, 0.82)";
      case 4:
        return "rgba(35, 213, 93, 0.82)";
      case 1:
        return "rgba(145, 164, 191, 0.82)";
      default:
        return "rgba(255, 255, 255, 0.82)";
    }
  }

  function getLeadStateText(overlayState, hudState) {
    const carrotMan = overlayState?.carrotMan || hudState?.carrotMan || {};
    const xStateRaw = String(carrotMan?.xState || "").trim();
    const trafficStateRaw = String(carrotMan?.trafficState || "").trim();
    const xState = finiteNumber(xStateRaw, -1);
    const trafficState = finiteNumber(trafficStateRaw, -1);
    const longActive = Boolean(overlayState?.carControl?.longActive);
    const vEgoMps = finiteNumber(hudState?.carState?.vEgo, finiteNumber(hudState?.carState?.vEgoCluster, 0));

    if (!longActive) return null;
    if (xState === 3 || xState === 5) {
      return {
        text: vEgoMps < 1.0 ? (trafficState >= 1000 ? "Signal Error" : "Signal Ready") : "Signal slowing",
        xState,
        showDistanceBadge: false,
      };
    }
    if (xState === 4) {
      return {
        text: "E2E 주행중",
        xState,
        showDistanceBadge: false,
      };
    }
    if (xState === 0 || xState === 1 || xState === 2) {
      return {
        text: "",
        xState,
        showDistanceBadge: true,
      };
    }
    if (xStateRaw) {
      return {
        text: xStateRaw,
        xState,
        showDistanceBadge: false,
      };
    }
    return {
      text: "",
      xState,
      showDistanceBadge: false,
    };
  }

  function drawLeadStateBadge(box, text, xState) {
    if (!box?.rect || !text) return;
    const badgeWidth = Math.max(108, Math.min(170, box.rect.width * 0.72));
    const badgeHeight = 32;
    const badgeX = box.centerX - badgeWidth * 0.5;
    const badgeY = box.rect.y + box.rect.height + 10;
    const accentColor = leadStateAccentColor(xState);
    fillRoundedRect(ctx, badgeX, badgeY, badgeWidth, badgeHeight, 14, "rgba(16, 21, 28, 0.88)");
    strokeRoundedRect(ctx, badgeX, badgeY, badgeWidth, badgeHeight, 14, accentColor, 2.2);
    ctx.save();
    ctx.font = `900 18px ${HUD_TEXT_FONT}`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.lineJoin = "round";
    ctx.strokeStyle = "rgba(0,0,0,0.88)";
    ctx.fillStyle = "#ffffff";
    ctx.lineWidth = 3.6;
    ctx.strokeText(text, box.centerX, badgeY + badgeHeight * 0.56);
    ctx.fillText(text, box.centerX, badgeY + badgeHeight * 0.56);
    ctx.restore();
  }

  function drawCanvasOutlinedText(text, x, y, {
    fontSize = 18,
    fontWeight = 800,
    fillStyle = "#ffffff",
    strokeStyle = "rgba(0,0,0,0.86)",
    strokeWidth = 3.4,
    align = "center",
    baseline = "middle",
  } = {}) {
    if (!text) return;
    ctx.save();
    ctx.font = `${fontWeight} ${fontSize}px ${HUD_TEXT_FONT}`;
    ctx.textAlign = align;
    ctx.textBaseline = baseline;
    ctx.lineJoin = "round";
    ctx.strokeStyle = strokeStyle;
    ctx.fillStyle = fillStyle;
    ctx.lineWidth = strokeWidth;
    ctx.strokeText(text, x, y);
    ctx.fillText(text, x, y);
    ctx.restore();
  }

  function clampTextAnchor(point, text, fontSize, videoWidth, videoHeight) {
    const anchor = { x: finiteNumber(point?.x, 0), y: finiteNumber(point?.y, 0) };
    ctx.save();
    ctx.font = `800 ${fontSize}px ${HUD_TEXT_FONT}`;
    const textWidth = Math.max(ctx.measureText(String(text || "")).width, 1);
    ctx.restore();
    const padding = 8;
    anchor.x = clamp(anchor.x, padding, Math.max(padding, videoWidth - textWidth - padding));
    anchor.y = clamp(anchor.y, fontSize + padding, Math.max(fontSize + padding, videoHeight - padding));
    return anchor;
  }

  function drawRadarSpeedBadge(center, text, accentColor) {
    if (!center || !text) return;
    const badgeWidth = Math.max(52, 28 + String(text).length * 18);
    const badgeHeight = 34;
    const badgeX = center.x - badgeWidth * 0.5;
    const badgeY = center.y - badgeHeight * 0.5;
    fillRoundedRect(ctx, badgeX, badgeY, badgeWidth, badgeHeight, 14, accentColor);
    drawCanvasOutlinedText(String(text), center.x, badgeY + badgeHeight * 0.56, {
      fontSize: 21,
      fontWeight: 900,
      strokeWidth: 3.6,
    });
  }

  function drawProjectedTfMarker(modelPath, longitudinalPlan, calibTransform, videoWidth, videoHeight) {
    if (finiteNumber(paramsState.ShowPathEnd, 0) <= 0) return;

    const tfDistance = finiteNumber(longitudinalPlan?.desiredDistance, 0);
    if (!Number.isFinite(tfDistance) || tfDistance <= 0) return;

    const xs = Array.isArray(modelPath?.x) ? modelPath.x : [];
    if (xs.length < 2) return;
    const lastX = finiteNumber(xs[xs.length - 1], 0);
    if (!lastX || tfDistance > lastX) return;

    const lineY = samplePathY(modelPath, tfDistance);
    const lineZ = interp1D(tfDistance, xs, Array.isArray(modelPath?.z) ? modelPath.z : []);
    if (!Number.isFinite(lineY) || !Number.isFinite(lineZ)) return;

    const left = projectPoint(calibTransform, tfDistance, lineY - 1.0, lineZ + PATH_Z_OFFSET);
    const right = projectPoint(calibTransform, tfDistance, lineY + 1.0, lineZ + PATH_Z_OFFSET);
    if (!left || !right) return;

    drawPolyline([left, right], "rgba(255,255,255,0.92)", 3.0);
    const labelText = `${tfDistance.toFixed(1)}(${finiteNumber(longitudinalPlan?.tFollow, 0).toFixed(2)})`;
    const labelAnchor = clampTextAnchor(
      { x: right.x + 10, y: right.y - 4 },
      labelText,
      20,
      videoWidth,
      videoHeight,
    );
    drawCanvasOutlinedText(labelText, labelAnchor.x, labelAnchor.y, {
      fontSize: 20,
      fontWeight: 800,
      align: "left",
    });
  }

  function getRadarTracks(radarState) {
    const source = radarState && typeof radarState === "object" ? radarState : {};
    const tracks = [
      ...(Array.isArray(source.leadsLeft) ? source.leadsLeft : []),
      ...(Array.isArray(source.leadsCenter) ? source.leadsCenter : []),
      ...(Array.isArray(source.leadsRight) ? source.leadsRight : []),
    ];
    if (tracks.length) return tracks;

    const fallback = [];
    if (source.leadOne?.status) fallback.push(source.leadOne);
    if (source.leadTwo?.status) fallback.push(source.leadTwo);
    return fallback;
  }

  function drawRadarTargets(radarState, modelPath, calibTransform) {
    const showRadarInfo = finiteNumber(paramsState.ShowRadarInfo, 0);
    if (showRadarInfo <= 0) return;

    for (const radar of getRadarTracks(radarState)) {
      const dRel = finiteNumber(radar?.dRel, 0);
      if (!Number.isFinite(dRel) || dRel <= 2.5) continue;

      const z = samplePathZ(modelPath, dRel) - 0.61;
      const center = projectPoint(calibTransform, dRel, -finiteNumber(radar?.yRel, 0), z);
      if (!center) continue;

      const vLead = finiteNumber(radar?.vLeadK, finiteNumber(radar?.vRel, 0));
      const vLat = finiteNumber(radar?.vLat, 0);
      const vAbs = Math.sqrt((vLead * vLead) + (vLat * vLat));
      const vSigned = vLead >= 0 ? vAbs : -vAbs;
      const radarDetected = Boolean(radar?.radar);
      const modelProb = finiteNumber(radar?.modelProb, 0);

      if (vAbs > 3.0) {
        const futureDRel = Math.max(2.0, dRel + vLead * 0.35);
        const futureYRel = finiteNumber(radar?.yRel, 0) + vLat * 0.35;
        const futureZ = samplePathZ(modelPath, futureDRel) - 0.61;
        const future = projectPoint(calibTransform, futureDRel, -futureYRel, futureZ);
        if (future) {
          const vectorColor = vSigned >= 0 ? "rgba(35,213,93,0.94)" : "rgba(255,59,48,0.94)";
          drawPolyline([center, future], vectorColor, 3.0);
          drawPolygon(circlePolygon(future.x, future.y, 7, 18), vectorColor);
        }

        let badgeColor = "rgba(255,59,48,0.96)";
        if (!radarDetected) badgeColor = "rgba(61,123,255,0.96)";
        else if (Math.abs(modelProb - 0.01) < 1e-3) badgeColor = "rgba(35,213,93,0.96)";
        else if (vSigned > 0) badgeColor = "rgba(255,167,38,0.96)";

        drawRadarSpeedBadge({ x: center.x, y: center.y - 18 }, (vSigned * 3.6).toFixed(0), badgeColor);

        if (showRadarInfo >= 2) {
          drawCanvasOutlinedText(finiteNumber(radar?.yRel, 0).toFixed(1), center.x, center.y - 48, {
            fontSize: 18,
            fontWeight: 800,
          });
          drawCanvasOutlinedText(dRel.toFixed(1), center.x, center.y + 30, {
            fontSize: 18,
            fontWeight: 800,
          });
        }
      } else if (showRadarInfo >= 3) {
        drawCanvasOutlinedText("*", center.x, center.y, {
          fontSize: 28,
          fontWeight: 900,
        });
      }
    }
  }

  function drawRadarLeadBoxes(model, overlayState, hudState, calibTransform, videoWidth, videoHeight) {
    const radarState = overlayState?.radarState || {};
    const modelPath = model?.position || null;
    const showRadarInfo = finiteNumber(paramsState.ShowRadarInfo, defaultParams.ShowRadarInfo);
    const leadState = getLeadStateText(overlayState, hudState);

    const leadOneBox = projectLeadBox(radarState?.leadOne, modelPath, calibTransform, videoWidth, videoHeight);
    if (leadOneBox) {
      const isLeadScc = leadOneBox.radarTrackId < 1;
      const strokeColor = !leadOneBox.radar ? "#3d7bff" : (isLeadScc ? "#ff3b30" : "#ffa726");
      drawLeadBoxCard(leadOneBox, strokeColor, "rgba(0,0,0,0.20)", true);

      if (showRadarInfo > 0 && leadState?.showDistanceBadge !== false) {
        const radarDist = leadOneBox.radar ? Math.max(0, finiteNumber(radarState?.leadOne?.dRel, 0)) : 0;
        const visionDist = leadOneBox.modelProb > 0.5 ? Math.max(0, leadOneBox.dRel - 1.52) : 0;
        const primaryDistance = radarDist > 0 ? radarDist : visionDist;
        if (primaryDistance > 0) {
          const badgeTextColor = leadState?.xState === 0 ? "#ffffff" : (leadState?.xState === 1 ? "#b0b0b0" : "#23d55d");
          drawLeadDistanceBadge(leadOneBox, primaryDistance.toFixed(1), strokeColor, badgeTextColor);
        }
      }

      if (leadState?.text) {
        drawLeadStateBadge(leadOneBox, leadState.text, leadState.xState);
      }
    }

    const leadTwo = radarState?.leadTwo;
    const validLeadTwo = Boolean(leadTwo?.status) &&
      Boolean(leadTwo?.radar) &&
      finiteNumber(leadTwo?.dRel, 0) > (finiteNumber(radarState?.leadOne?.dRel, 0) + 3) &&
      finiteNumber(leadTwo?.radarTrackId, -99999) !== finiteNumber(radarState?.leadOne?.radarTrackId, -99998);
    if (validLeadTwo) {
      const leadTwoBox = projectLeadBox(leadTwo, modelPath, calibTransform, videoWidth, videoHeight);
      if (leadTwoBox) {
        drawLeadBoxCard(leadTwoBox, "#b68a3a", "rgba(0,0,0,0.20)", false);
      }
    }
    drawRadarTargets(radarState, modelPath, calibTransform);
  }

  function roundedRectPath(context, x, y, width, height, radius) {
    const r = Math.min(radius, width / 2, height / 2);
    context.beginPath();
    context.moveTo(x + r, y);
    context.lineTo(x + width - r, y);
    context.quadraticCurveTo(x + width, y, x + width, y + r);
    context.lineTo(x + width, y + height - r);
    context.quadraticCurveTo(x + width, y + height, x + width - r, y + height);
    context.lineTo(x + r, y + height);
    context.quadraticCurveTo(x, y + height, x, y + height - r);
    context.lineTo(x, y + r);
    context.quadraticCurveTo(x, y, x + r, y);
    context.closePath();
  }

  function fillRoundedRect(context, x, y, width, height, radius, fillStyle) {
    roundedRectPath(context, x, y, width, height, radius);
    context.fillStyle = fillStyle;
    context.fill();
  }

  function strokeRoundedRect(context, x, y, width, height, radius, strokeStyle, lineWidth = 1) {
    roundedRectPath(context, x, y, width, height, radius);
    context.strokeStyle = strokeStyle;
    context.lineWidth = lineWidth;
    context.stroke();
  }

  function getHudLabelAlpha(pathMode, phaseShift = 0) {
    if (pathMode < 1 || pathMode > 8) return 0.94;
    const wave = (Math.sin((performance.now() / 1000) * 0.78 + phaseShift) + 1.0) * 0.5;
    const eased = Math.pow(wave, 1.7);
    return clamp(0.14 + eased * 0.86, 0.14, 1.0);
  }

  function formatBottomCenterText(text, hudState) {
    const raw = String(text || "").trim();
    const modeText = laneModeLabel(hudState);
    if (!raw) return modeText;
    const normalized = raw.toLowerCase();
    if (normalized.startsWith("lanemode") || normalized.startsWith("laneless")) {
      return raw;
    }
    return `${modeText} | ${raw}`;
  }

  function drawHudTopRightText(stageWidth, stageHeight, viewportRect, text, pathMode) {
    const label = shortText(text, 160);
    if (!label) return;
    const exactC3Mode = stageWidth >= 1280 && stageHeight >= 720;
    const baseScale = Math.min(stageWidth / 1920, stageHeight / 1080);
    const scale = clamp(baseScale, 0.48, 1.0);
    const edgeInsetX = exactC3Mode ? 1.5 : clamp(2.0 * scale, 1.0, 2.5);
    const edgeInsetTop = exactC3Mode ? 1.5 : clamp(2.0 * scale, 1.0, 2.5);
    const maxWidth = Math.max(120.0, viewportRect.width - edgeInsetX * 2.0);
    const fontSize = fitSingleLineHudFontSize(
      label,
      exactC3Mode ? 24.0 : clamp(24.0 * scale, 7.0, 24.0),
      maxWidth,
      4.5,
      900,
    );
    const alpha = getHudLabelAlpha(pathMode, 0.0);

    drawOutlinedHudText({
      text: label,
      x: viewportRect.right - edgeInsetX,
      y: viewportRect.top + edgeInsetTop,
      color: `rgba(244, 244, 244, ${alpha.toFixed(3)})`,
      strokeColor: `rgba(0, 0, 0, ${clamp(alpha + 0.08, 0.0, 1.0).toFixed(3)})`,
      strokeWidth: clamp(4.2 * scale, 2.8, 5.4),
      fontSize,
      fontWeight: 900,
      alignX: "right",
      alignY: "top",
      maxWidth,
    });
  }

  function drawHudLeftCenterLogs(stageWidth, stageHeight, viewportRect, statusText, metaText, pathMode) {
    const statusLabel = shortText(statusText, 96);
    const metaLabel = shortText(metaText, 160);
    if (!statusLabel && !metaLabel) return;

    const exactC3Mode = stageWidth >= 1280 && stageHeight >= 720;
    const baseScale = Math.min(stageWidth / 1920, stageHeight / 1080);
    const scale = clamp(baseScale, 0.48, 1.0);
    const edgeInsetX = exactC3Mode ? 1.5 : clamp(2.0 * scale, 1.0, 2.5);
    const maxWidth = Math.max(180.0, viewportRect.width * 0.52);
    const alpha = getHudLabelAlpha(pathMode, Math.PI * 0.5);
    const centerY = viewportRect.centerY;
    const statusFontSize = fitSingleLineHudFontSize(
      statusLabel,
      exactC3Mode ? 24.0 : clamp(24.0 * scale, 9.0, 24.0),
      maxWidth,
      6.0,
      900,
    );
    const metaFontSize = fitSingleLineHudFontSize(
      metaLabel,
      exactC3Mode ? 20.0 : clamp(20.0 * scale, 8.0, 20.0),
      maxWidth,
      6.0,
      800,
    );
    const statusY = centerY - (exactC3Mode ? 16.0 : clamp(18.0 * scale, 10.0, 18.0));
    const metaY = centerY + (exactC3Mode ? 14.0 : clamp(16.0 * scale, 9.0, 16.0));

    drawOutlinedHudText({
      text: statusLabel,
      x: viewportRect.left + edgeInsetX,
      y: statusY,
      color: `rgba(244, 244, 244, ${alpha.toFixed(3)})`,
      strokeColor: `rgba(0, 0, 0, ${clamp(alpha + 0.08, 0.0, 1.0).toFixed(3)})`,
      strokeWidth: clamp(4.2 * scale, 2.8, 5.4),
      fontSize: statusFontSize,
      fontWeight: 900,
      alignX: "left",
      alignY: "bottom",
      maxWidth,
    });
    drawOutlinedHudText({
      text: metaLabel,
      x: viewportRect.left + edgeInsetX,
      y: metaY,
      color: `rgba(236, 236, 236, ${alpha.toFixed(3)})`,
      strokeColor: `rgba(0, 0, 0, ${clamp(alpha + 0.08, 0.0, 1.0).toFixed(3)})`,
      strokeWidth: clamp(4.0 * scale, 2.8, 5.2),
      fontSize: metaFontSize,
      fontWeight: 800,
      alignX: "left",
      alignY: "top",
      maxWidth,
    });
  }

  function drawHudBottomText(stageWidth, stageHeight, viewportRect, text, hudState, pathMode) {
    const label = formatBottomCenterText(text, hudState);
    if (!label) return;
    const exactC3Mode = stageWidth >= 1280 && stageHeight >= 720;
    const baseScale = Math.min(stageWidth / 1920, stageHeight / 1080);
    const scale = clamp(baseScale, 0.48, 1.0);
    const maxWidth = Math.max(120.0, viewportRect.width - 4.0);
    const fontSize = fitSingleLineHudFontSize(
      label,
      exactC3Mode ? 24.0 : clamp(24.0 * scale, 7.0, 24.0),
      maxWidth,
      4.5,
      900,
    );
    const bottomInset = exactC3Mode ? 1.5 : clamp(2.0 * scale, 1.0, 2.5);
    const alpha = getHudLabelAlpha(pathMode, Math.PI);

    drawOutlinedHudText({
      text: label,
      x: viewportRect.centerX,
      y: viewportRect.bottom - bottomInset,
      color: `rgba(236, 236, 236, ${alpha.toFixed(3)})`,
      strokeColor: `rgba(0, 0, 0, ${clamp(alpha + 0.08, 0.0, 1.0).toFixed(3)})`,
      strokeWidth: clamp(4.0 * scale, 2.8, 5.2),
      fontSize,
      fontWeight: 900,
      alignX: "center",
      alignY: "baselineBottom",
      maxWidth,
    });
  }

  function formatDebugText(overlayState) {
    const liveDelay = overlayState.liveDelay || {};
    const liveTorqueParameters = overlayState.liveTorqueParameters || {};
    const liveParameters = overlayState.liveParameters || {};
    const customSr = finiteNumber(paramsState.CustomSR, 0) / 10.0;

    return `LD[${finiteNumber(liveDelay.calPerc, 0).toFixed(0)}%,${finiteNumber(liveDelay.lateralDelay, 0).toFixed(2)}] ` +
      `LT[${finiteNumber(liveTorqueParameters.calPerc, 0).toFixed(0)}%,${liveTorqueParameters.liveValid ? "ON" : "OFF"}]` +
      `(${finiteNumber(liveTorqueParameters.latAccelFactorFiltered, 0).toFixed(2)}/${finiteNumber(liveTorqueParameters.frictionCoefficientFiltered, 0).toFixed(2)}) ` +
      `SR(${finiteNumber(liveParameters.steerRatio, 0).toFixed(1)},${customSr.toFixed(1)})`;
  }

  function isLongActive(overlayState) {
    return Boolean(overlayState?.carControl?.longActive);
  }

  function isLaneMode(hudState) {
    return Boolean(hudState?.controlsState?.activeLaneLine) || finiteNumber(hudState?.carState?.useLaneLineSpeed, 0) > 0;
  }

  function getPathStyle(overlayState, hudState) {
    // Mirror carrot.cc path mode/color selection so Carrot params drive web visuals too.
    const laneMode = isLaneMode(hudState);
    let mode = finiteNumber(laneMode ? paramsState.ShowPathModeLane : paramsState.ShowPathMode, 0);
    let colorIndex = finiteNumber(laneMode ? paramsState.ShowPathColorLane : paramsState.ShowPathColor, 13);
    const isCruiseOff = !isLongActive(overlayState);

    if (isCruiseOff) {
      colorIndex = finiteNumber(paramsState.ShowPathColorCruiseOff, 19);
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

    return {
      mode,
      colorIndex,
      paletteIndex: colorIndex % 10,
      emphasisStroke: colorIndex >= 10 || Boolean(hudState?.carState?.brakeLights),
      strokeColor: hudState?.carState?.brakeLights ? "rgba(255, 76, 76, 0.96)" : "rgba(255, 255, 255, 0.92)",
      isCruiseOff,
      laneMode,
    };
  }

  function getSelectedPath(overlayState, hudState) {
    const model = overlayState?.modelV2 || null;
    const lateralPlan = overlayState?.lateralPlan || null;
    const laneMode = isLaneMode(hudState);
    const lanePath = lateralPlan?.position;
    const hasLanePath = Array.isArray(lanePath?.x) && lanePath.x.length > 2;
    if (laneMode && hasLanePath) {
      return {
        model,
        pathData: lanePath,
        pathSource: "lateralPlan",
        latDebugText: lateralPlan?.latDebugText || "",
        laneMode,
      };
    }

    return {
      model,
      pathData: model?.position || null,
      pathSource: "modelV2",
      latDebugText: lateralPlan?.latDebugText || "",
      laneMode,
    };
  }

  function buildPlotData(overlayState, hudState) {
    const mode = finiteNumber(paramsState.ShowPlotMode, 0);
    if (!mode) return null;

    const carState = hudState?.carState || {};
    const controlsState = hudState?.controlsState || {};
    const longPlan = hudState?.longitudinalPlan || {};
    const carControl = overlayState?.carControl || {};
    const actuators = carControl?.actuators || {};
    const model = overlayState?.modelV2 || {};
    const radarLead = overlayState?.radarState?.leadOne || {};
    const liveParameters = overlayState?.liveParameters || {};

    const aEgo = finiteNumber(carState?.aEgo, 0);
    const vEgo = finiteNumber(carState?.vEgo, finiteNumber(carState?.vEgoCluster, 0));
    const accelTarget = firstFinite(longPlan?.accels, 0);
    const speedTarget = firstFinite(longPlan?.speeds, 0);
    const modelPos32 = finiteNumber(model?.position?.x?.[32], firstFinite(model?.position?.x, 0));
    const modelVel32 = finiteNumber(model?.velocity?.x?.[32], firstFinite(model?.velocity?.x, 0));
    const modelVel0 = finiteNumber(model?.velocity?.x?.[0], 0);

    switch (mode) {
      case 1:
        return {
          mode,
          title: "1.Accel (Y:a_ego, G:a_target, O:a_out)",
          values: [aEgo, accelTarget, finiteNumber(actuators?.accel, 0)],
        };
      case 2:
        return {
          mode,
          title: "2.Speed/Accel (Y:speed_0, G:v_ego, O:a_ego)",
          values: [speedTarget, vEgo, aEgo],
        };
      case 3:
        return {
          mode,
          title: "3.Model (Y:pos_32, G:vel_32, O:vel_0)",
          values: [modelPos32, modelVel32, modelVel0],
        };
      case 4:
        return {
          mode,
          title: "4.Lead (Y:accel, G:a_lead, O:v_rel)",
          values: [accelTarget, finiteNumber(radarLead?.aLeadK, 0), finiteNumber(radarLead?.vRel, 0)],
        };
      case 5:
        return {
          mode,
          title: "5.Lead (Y:a_ego, G:a_lead, O:j_lead)",
          values: [aEgo, finiteNumber(radarLead?.aLead, 0), finiteNumber(radarLead?.jLead, 0)],
        };
      case 6:
        return {
          mode,
          title: "6.Steer (web raw torqueState unavailable)",
          values: [0, 0, 0],
        };
      case 7:
        return {
          mode,
          title: "7.SteerA (Y:Actual, G:Target, O:Offset*10)",
          values: [
            finiteNumber(carState?.steeringAngleDeg, 0),
            finiteNumber(actuators?.steeringAngleDeg, 0),
            finiteNumber(liveParameters?.angleOffsetDeg, 0) * 10.0,
          ],
        };
      case 8:
        return {
          mode,
          title: "8.Curvature (Y:G:O:cmd*10000)",
          values: [
            finiteNumber(actuators?.curvature, 0) * 10000,
            finiteNumber(controlsState?.desiredCurvature, 0) * 10000,
            finiteNumber(controlsState?.curvature, 0) * 10000,
          ],
        };
      default:
        return {
          mode,
          title: "no data",
          values: [0, 0, 0],
        };
    }
  }

  function updatePlotHistory(plotData) {
    if (!plotData) {
      lastPlotMode = -1;
      plotHistory = [[], [], []];
      return;
    }

    if (plotData.mode !== lastPlotMode) {
      lastPlotMode = plotData.mode;
      plotHistory = [[], [], []];
    }

    for (let i = 0; i < 3; i += 1) {
      plotHistory[i].push(finiteNumber(plotData.values[i], 0));
      if (plotHistory[i].length > PLOT_MAX_POINTS) {
        plotHistory[i].shift();
      }
    }
  }

  function getPlotBounds() {
    let min = Number.POSITIVE_INFINITY;
    let max = Number.NEGATIVE_INFINITY;
    for (const series of plotHistory) {
      for (const value of series) {
        min = Math.min(min, value);
        max = Math.max(max, value);
      }
    }

    if (!Number.isFinite(min) || !Number.isFinite(max)) {
      min = -2;
      max = 2;
    }
    if (min > -2) min = -2;
    if (max < 2) max = 2;
    if (Math.abs(max - min) < 1e-3) {
      max += 1;
      min -= 1;
    }
    return { min, max };
  }

  function drawPlot(stageWidth, stageHeight, viewportRect, plotData) {
    if (!plotData) return;
    const viewportWidth = finiteNumber(viewportRect?.width, stageWidth);
    const viewportHeight = finiteNumber(viewportRect?.height, stageHeight);
    if (viewportWidth < 560 || viewportHeight < 260) return;

    const panelWidth = clamp(viewportWidth * 0.42, 340, 560);
    const panelHeight = clamp(viewportHeight * 0.22, 156, 200);
    const panelX = finiteNumber(viewportRect?.left, 0) + Math.max(14, viewportWidth * 0.018);
    const panelY = finiteNumber(viewportRect?.top, 0) + Math.max(46, viewportHeight * 0.05);
    const graphX = panelX + 16;
    const graphY = panelY + 40;
    const graphWidth = panelWidth - 32;
    const graphHeight = panelHeight - 58;
    const bounds = getPlotBounds();
    const range = Math.max(bounds.max - bounds.min, 1);

    hudCtx.save();
    fillRoundedRect(hudCtx, panelX, panelY, panelWidth, panelHeight, 18, "rgba(15, 20, 28, 0.58)");
    strokeRoundedRect(hudCtx, panelX, panelY, panelWidth, panelHeight, 18, "rgba(255,255,255,0.10)");

    hudCtx.fillStyle = "rgba(255,255,255,0.95)";
    hudCtx.font = "600 16px system-ui";
    hudCtx.textAlign = "left";
    hudCtx.textBaseline = "middle";
    hudCtx.fillText(plotData.title, panelX + 16, panelY + 20);

    hudCtx.strokeStyle = "rgba(255,255,255,0.10)";
    hudCtx.lineWidth = 1;
    for (let i = 0; i <= 4; i += 1) {
      const y = graphY + (graphHeight / 4) * i;
      hudCtx.beginPath();
      hudCtx.moveTo(graphX, y);
      hudCtx.lineTo(graphX + graphWidth, y);
      hudCtx.stroke();
    }

    for (let seriesIndex = 0; seriesIndex < plotHistory.length; seriesIndex += 1) {
      const series = plotHistory[seriesIndex];
      if (series.length < 2) continue;

      hudCtx.beginPath();
      for (let i = 0; i < series.length; i += 1) {
        const x = graphX + (graphWidth * i) / Math.max(1, PLOT_MAX_POINTS - 1);
        const y = graphY + graphHeight - ((series[i] - bounds.min) / range) * graphHeight;
        if (i === 0) hudCtx.moveTo(x, y);
        else hudCtx.lineTo(x, y);
      }
      hudCtx.lineWidth = 2.5;
      hudCtx.strokeStyle = PLOT_SERIES[seriesIndex].color;
      hudCtx.stroke();

      const currentValue = series[series.length - 1];
      hudCtx.fillStyle = PLOT_SERIES[seriesIndex].color;
      hudCtx.font = "600 15px system-ui";
      hudCtx.fillText(
        `${PLOT_SERIES[seriesIndex].label}:${currentValue.toFixed(2)}`,
        graphX + 8 + seriesIndex * 130,
        panelY + panelHeight - 16,
      );
    }

    hudCtx.restore();
  }

  function laneModeLabel(hudState) {
    return isLaneMode(hudState) ? "LaneMode" : "Laneless";
  }

  function refreshParams(force = false) {
    const runtimeParams = readLiveRuntimeParams();
    if (runtimeParams) {
      paramsState = runtimeParams;
      return;
    }

    if (force) {
      paramsState = normalizeVisualParams({}, paramsState);
    }
  }

  function isActive() {
    return document.body?.dataset?.page === "carrot";
  }

  function renderActiveFrame() {
    refreshParams();
    const stageWidth = Math.max(1, stageEl.clientWidth);
    const stageHeight = Math.max(1, stageEl.clientHeight);

    const hasStream = syncSourceStream();
    if (!hasStream || !videoEl.videoWidth || !videoEl.videoHeight) {
      setStageReady(false);
      clearOverlay(canvasEl.width || 1, canvasEl.height || 1);
      clearHud(hudCanvasEl.width || 1, hudCanvasEl.height || 1);
      setStatus("waiting road camera stream...");
      setMeta("road:- model:- path:-");
      setDebug("LD:- LT:- SR:-");
      applyCarrotHudLayout({
        left: 0,
        top: 0,
        right: stageWidth,
        bottom: stageHeight,
        width: stageWidth,
        height: stageHeight,
      });
      return;
    }

    const videoWidth = videoEl.videoWidth;
    const videoHeight = videoEl.videoHeight;
    syncCanvasSize(videoWidth, videoHeight, stageWidth, stageHeight);

    const rawOverlayState = window.CarrotOverlayState || {};
    const rawHudState = window.CarrotHudState || {};
    const runtimeState = mergeRuntimeState(rawHudState, rawOverlayState);
    let overlayState = runtimeState.overlayState;
    const hudState = runtimeState.hudState;
    const brokerServices = runtimeState.brokerServices;
    overlayState = {
      ...overlayState,
      radarState: getInterpolatedRadarState(overlayState?.radarState, performance.now()),
    };
    const model = overlayState.modelV2 || null;
    const liveCalibration = overlayState.liveCalibration || null;
    const roadCameraState = overlayState.roadCameraState || null;
    const selectedPath = getSelectedPath(overlayState, hudState);
    const pathStyle = getPathStyle(overlayState, hudState);
    const plotData = buildPlotData(overlayState, hudState);
    const showLaneInfo = finiteNumber(paramsState.ShowLaneInfo, defaultParams.ShowLaneInfo);

    updatePlotHistory(plotData);

    const calibration = getCalibrationMatrix(liveCalibration);
    const transform = getStageTransform(videoWidth, videoHeight, stageWidth, stageHeight, calibration);
    const viewportRect = getHudViewportRect(videoWidth, videoHeight, stageWidth, stageHeight, transform);

    applyStageTransform(transform);
    setStageReady(true);
    applyCarrotHudLayout(viewportRect);
    clearOverlay(videoWidth, videoHeight);
    clearHud(stageWidth, stageHeight);

    if (model) {
      if (showLaneInfo >= 1) drawLaneLines(model, hudState, transform.calibTransform);
      if (showLaneInfo > 1) drawRoadEdges(model, transform.calibTransform);
      if (showLaneInfo >= 0) drawPath(selectedPath.pathData, model, transform.calibTransform, videoHeight, pathStyle);
      drawProjectedTfMarker(model?.position, hudState?.longitudinalPlan, transform.calibTransform, videoWidth, videoHeight);
      drawBlindspotBarriers(model?.position, overlayState, hudState, transform.calibTransform);
      drawRadarLeadBoxes(model, overlayState, hudState, transform.calibTransform, videoWidth, videoHeight);
    }

    drawPlot(stageWidth, stageHeight, viewportRect, plotData);

    const laneCount = Array.isArray(model?.laneLines) ? model.laneLines.length : 0;
    const edgeCount = Array.isArray(model?.roadEdges) ? model.roadEdges.length : 0;
    const leadCount = Array.isArray(model?.leadsV3) ? model.leadsV3.length : 0;
    const rpy = finiteList(liveCalibration?.rpyCalib).slice(0, 3).map((value) => value.toFixed(3));
    const modeLabel = transform.displayMode?.label || DISPLAY_MODES[1].label;
    const laneLabel = laneModeLabel(hudState);

    if (!model) {
      setStatus(`road ${videoWidth}x${videoHeight} · waiting modelV2... · ${laneLabel}`);
    } else {
      setStatus(`road ${videoWidth}x${videoHeight} · model ${model.frameId ?? "-"} · ${laneLabel} · ${modeLabel}`);
    }

    setMeta(
      `road:${roadCameraState?.frameId ?? "-"} model:${model?.frameId ?? "-"} path:${selectedPath.pathSource}/${pathStyle.mode}:${pathStyle.colorIndex} width:${finiteNumber(paramsState.ShowPathWidth, 100)} laneInfo:${showLaneInfo} lane:${laneCount} edge:${edgeCount} lead:${leadCount} plot:${finiteNumber(paramsState.ShowPlotMode, 0)} rpy:${rpy.join(",") || "-"} h:${finiteNumber(liveCalibration?.height?.[0], 0).toFixed(2)}`,
    );
    setDebug(firstNonEmptyText(brokerServices?.carrotMan?.stockDebugTopRightText, overlayState?.carrotMan?.stockDebugTopRightText, formatDebugText(overlayState)));
    drawHudTopRightText(stageWidth, stageHeight, viewportRect, lastDebug, pathStyle.mode);
    drawHudBottomText(stageWidth, stageHeight, viewportRect, selectedPath.latDebugText, hudState, pathStyle.mode);
  }

  function scheduleNext() {
    if (isActive()) {
      loopToken = window.requestAnimationFrame(runLoop);
      return;
    }
    loopToken = window.setTimeout(() => runLoop(performance.now()), 180);
  }

  function runLoop() {
    if (isActive()) {
      renderActiveFrame();
    } else {
      resetCarrotHudLayout();
      syncSourceStream();
    }
    scheduleNext();
  }

  function refresh() {
    transformSignature = "";
    overlaySizeSignature = "";
    hudSizeSignature = "";
  }

  zoomButtons.forEach((button) => {
    button.addEventListener("click", () => {
      const index = Number(button.dataset.displayIndex);
      if (!Number.isFinite(index)) return;
      setDisplayModeIndex(index);
      renderActiveFrame();
    });
  });

  window.addEventListener("resize", refresh);

  try {
    const stored = Number(localStorage.getItem(DISPLAY_MODE_STORAGE_KEY));
    if (Number.isFinite(stored)) {
      displayModeIndex = clamp(stored, 0, DISPLAY_MODES.length - 1);
    }
  } catch {}

  syncDisplayModeButtons();
  refreshParams(true);

  runLoop();

  return {
    refresh,
    setDisplayModeIndex,
  };
})();
