"use strict";

window.HomeDrive = (() => {
  const stageEl = document.getElementById("carrotStage");
  const videoEl = document.getElementById("carrotRoadVideo");
  const videoHoldEl = document.getElementById("carrotLastFrameCanvas");
  const canvasEl = document.getElementById("carrotOverlayCanvas");
  const hudCanvasEl = document.getElementById("carrotHudCanvas");
  const onroadAlertEl = document.getElementById("carrotOnroadAlert");
  const onroadAlertBoxEl = document.getElementById("carrotOnroadAlertBox");
  const onroadAlertText1El = document.getElementById("carrotOnroadAlertText1");
  const onroadAlertText2El = document.getElementById("carrotOnroadAlertText2");
  const stageLoadingEl = document.getElementById("carrotStageLoading");
  const stageLoadingTextEl = document.getElementById("carrotStageLoadingText");
  const statusEl = document.getElementById("carrotStageStatus");
  const metaEl = document.getElementById("carrotStageMeta");
  const debugEl = document.getElementById("carrotStageDebug");
  const driveHudCardEl = document.getElementById("driveHudCard");
  const sourceVideoEl = videoEl;
  const displayModeButton = document.getElementById("btnDisplayModeCycle");

  if (!stageEl || !videoEl || !canvasEl || !hudCanvasEl || !statusEl || !metaEl || !debugEl) {
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
    { key: "fit", labelKey: "display_fit", fallbackLabel: "Fit" },
    { key: "normal", labelKey: "display_normal", fallbackLabel: "Normal" },
    { key: "crop", labelKey: "display_crop", fallbackLabel: "Crop" },
  ];
  const HUD_TEXT_FONT = "system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif";
  const DISPLAY_MODE_STORAGE_KEY = "home_drive_display_mode_index";
  const PHONE_PORTRAIT_DPR_CAP = 1.0;
  const MOBILE_DPR_CAP = 1.25;
  const DESKTOP_DPR_CAP = 1.5;
  const RENDER_INTERVAL_MS = 33;  // ~30fps for denser plot data (C3: 20Hz/50ms)
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
  const PLOT_MAX_POINTS = 600;  // ~20s at 30fps (C3: 400 at 20fps = 20s)
  const PATH_HALF_WIDTH = 0.9;
  const PATH_Z_OFFSET = 1.22;
  const LONG_PLAN_SOURCE_LEAD1 = 2;
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
  const ALERT_STATUS_NORMAL = 0;
  const ALERT_STATUS_USER_PROMPT = 1;
  const ALERT_STATUS_CRITICAL = 2;
  const ALERT_SIZE_NONE = 0;
  const ALERT_SIZE_SMALL = 1;
  const ALERT_SIZE_MID = 2;
  const ALERT_SIZE_FULL = 3;
  const ONROAD_ALERT_SCALE = 1.5;
  const POLYLINE_SMOOTH_NEAR_DISTANCE = 16;
  const POLYLINE_SMOOTH_FAR_DISTANCE = 52;
  const POLYLINE_SMOOTH_MAX_STRENGTH = 0.34;
  const POLYLINE_CENTER_SMOOTH_MAX_STRENGTH = 0.24;
  const GEOMETRY_QUALITY_DEFAULT = "default";
  const GEOMETRY_QUALITY_LANE = "lane";
  const GEOMETRY_QUALITY_ROAD_EDGE = "road-edge";

  const defaultParams = {
    IsMetric: 1,
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
    RadarLatFactor: 0,
    CustomSR: 0,
  };
  const overlayInfoState = {
    carLabel: "",
    branchLabel: "",
    lastSignature: "",
    loading: false,
    nextRetryAt: 0,
  };

  let paramsState = { ...defaultParams };
  let displayModeIndex = 1;
  let overlaySizeSignature = "";
  let hudSizeSignature = "";
  let transformSignature = "";
  let lastStatus = "";
  let lastMeta = "";
  let lastDebug = "";
  let lastPlotMode = -1;
  let radarInterpolationState = {
    signature: "",
    previous: null,
    current: null,
    previousAtMs: 0,
    currentAtMs: 0,
  };

  /* ── EMA state for lead box smoothing ──
   * C3 uses alpha=0.85 at a stable 20Hz UI loop.  The web's actual frame
   * rate varies, so we use time-based EMA: alpha_adj = 0.85^(dt/50ms).
   * This guarantees the same wall-clock convergence speed as C3.          */
  const LEAD_EMA_ALPHA = 0.85;
  const C3_FRAME_MS = 50;  // C3 UI loop interval (~20Hz)
  let leadEmaState = [
    { fx: NaN, fy: NaN, fw: NaN, trackId: -1, lastMs: 0 },  // slot 0: leadOne
    { fx: NaN, fy: NaN, fw: NaN, trackId: -1, lastMs: 0 },  // slot 1: leadTwo
  ];
  let leadTwoEmaState = { xl: NaN, xr: NaN, y: NaN, lastMs: 0 };
  /* ── Lead hold state: keeps box visible briefly when status flickers false ──
   * C3 reads SubMaster at stable 20Hz; web's merged WebSocket state can
   * briefly lose status between messages.  Hold last valid box for up to
   * LEAD_HOLD_MS so the visual experience matches C3's stability.           */
  const LEAD_HOLD_MS = 1500;
  let leadHoldState = {
    lastValidMs: 0,
    box: null,
    strokeColor: null,
    isLeadScc: false,
    radarDist: 0,
    visionDist: 0,
    badgeTextColor: "#ffffff",
  };
  let leadRenderState = {
    lastCameraFrameId: NaN,
    lastModelFrameId: NaN,
    lastSourceWidth: NaN,
    lastSourceHeight: NaN,
  };

  /* ── Phase 1-2: dirty check ── */
  let _lastOverlaySig = "";
  let _lastHudSig = "";
  let _lastPlotInputSig = "";
  let _lastAlertSig = "";
  let _forceNextRender = true;
  let _lastRenderTime = 0;
  let _renderRafId = null;
  let _renderTimerId = null;
  let _renderVideoFrameId = null;
  let _pendingRenderState = {
    force: true,
    overlayDirty: true,
    hudDirty: true,
  };
  const _mergeRuntimeCache = {
    refs: null,
    result: null,
  };
  let _frameProjectionCache = {
    pathLengthIdx: new WeakMap(),
    ribbon: new WeakMap(),
    verticalRibbon: new WeakMap(),
    pathZ: new WeakMap(),
    pathY: new WeakMap(),
  };

  function pathDataSignature(pathData) {
    const x = Array.isArray(pathData?.x) ? pathData.x : [];
    const y = Array.isArray(pathData?.y) ? pathData.y : [];
    if (!x.length || !y.length) return "none";
    const lastIndex = Math.min(x.length, y.length) - 1;
    const midIndex = Math.min(lastIndex, Math.max(0, Math.floor(lastIndex / 2)));
    const farIndex = Math.min(lastIndex, 16);
    return [
      x.length,
      finiteNumber(x[0], 0).toFixed(2),
      finiteNumber(y[0], 0).toFixed(2),
      finiteNumber(x[midIndex], 0).toFixed(2),
      finiteNumber(y[midIndex], 0).toFixed(2),
      finiteNumber(x[farIndex], 0).toFixed(2),
      finiteNumber(y[farIndex], 0).toFixed(2),
      finiteNumber(x[lastIndex], 0).toFixed(2),
      finiteNumber(y[lastIndex], 0).toFixed(2),
    ].join("|");
  }

  function plotInputSignature(plotData) {
    if (!plotData) return "off";
    return [
      plotData.mode,
      plotData.title,
      finiteNumber(plotData.values?.[0], 0).toFixed(3),
      finiteNumber(plotData.values?.[1], 0).toFixed(3),
      finiteNumber(plotData.values?.[2], 0).toFixed(3),
    ].join("|");
  }

  function overlayDataSignature(hudState, overlayState, selectedPath, pathStyle, showLaneInfo) {
    const model = overlayState?.modelV2;
    const radar = overlayState?.radarState;
    const liveCalibration = overlayState?.liveCalibration;
    const carState = hudState?.carState;
    const controlsState = hudState?.controlsState;
    const longPlan = hudState?.longitudinalPlan;
    return [
      model?.frameId ?? "-",
      selectedPath?.pathSource || "none",
      pathDataSignature(selectedPath?.pathData),
      radarStateSignature(radar),
      finiteNumber(liveCalibration?.rpyCalib?.[0], 0).toFixed(3),
      finiteNumber(liveCalibration?.rpyCalib?.[1], 0).toFixed(3),
      finiteNumber(liveCalibration?.rpyCalib?.[2], 0).toFixed(3),
      finiteNumber(liveCalibration?.height?.[0], 0).toFixed(2),
      Boolean(controlsState?.activeLaneLine) ? 1 : 0,
      Boolean(controlsState?.enabled) ? 1 : 0,
      finiteNumber(carState?.useLaneLineSpeed, 0).toFixed(2),
      Boolean(carState?.brakeLights) ? 1 : 0,
      Boolean(overlayState?.carControl?.longActive) ? 1 : 0,
      longPlan?.xState ?? "-",
      longPlan?.trafficState ?? "-",
      longPlan?.longitudinalPlanSource ?? "-",
      showLaneInfo,
      pathStyle?.mode ?? 0,
      pathStyle?.colorIndex ?? 0,
      paramsState.ShowPathMode,
      paramsState.ShowPathColor,
      paramsState.ShowPathModeLane,
      paramsState.ShowPathColorLane,
      paramsState.ShowPathColorCruiseOff,
      paramsState.ShowPathWidth,
      paramsState.ShowPathEnd,
      paramsState.ShowLaneInfo,
      paramsState.ShowRadarInfo,
    ].join("|");
  }

  function hudDataSignature(hudState, overlayState, plotData, selectedPath, debugText) {
    const carState = hudState?.carState;
    const carrotMan = hudState?.carrotMan;
    const longPlan = hudState?.longitudinalPlan;
    const selfdriveState = hudState?.selfdriveState;
    const rtcPerfText = formatRtcPerfLabel();
    return [
      finiteNumber(carState?.vEgo, 0).toFixed(3),
      finiteNumber(carState?.vEgoCluster, 0).toFixed(3),
      finiteNumber(carState?.vCruiseCluster, 0).toFixed(2),
      carrotMan?.xSpdLimit ?? "-",
      carrotMan?.nRoadLimitSpeed ?? "-",
      carrotMan?.desiredSpeed ?? "-",
      carrotMan?.activeCarrot ?? "-",
      selfdriveState?.personality ?? "-",
      selfdriveState?.alertStatus ?? "-",
      selfdriveState?.alertSize ?? "-",
      selfdriveState?.alertType || "",
      selfdriveState?.alertText1 || "",
      selfdriveState?.alertText2 || "",
      longPlan?.myDrivingMode ?? "-",
      longPlan?.tFollow ?? "-",
      longPlan?.desiredDistance ?? "-",
      overlayState?.roadCameraState?.frameId ?? "-",
      selectedPath?.latDebugText || "",
      debugText || "",
      rtcPerfText,
      plotInputSignature(plotData),
      paramsState.ShowPlotMode,
      paramsState.CustomSR,
    ].join("|");
  }

  /* ── Phase 1-3: gradient cache ── */
  const _gradientCache = new Map();
  const GRADIENT_CACHE_MAX = 16;
  const _hudGradientCache = new Map();
  const HUD_GRADIENT_CACHE_MAX = 8;
  const _roundedRectPathCache = new Map();
  const ROUNDED_RECT_CACHE_MAX = 24;
  const _textWidthCache = new Map();
  const TEXT_WIDTH_CACHE_MAX = 256;

  function getCachedGradient(key, factory) {
    const cached = _gradientCache.get(key);
    if (cached) return cached;
    const g = factory();
    if (_gradientCache.size >= GRADIENT_CACHE_MAX) {
      const firstKey = _gradientCache.keys().next().value;
      _gradientCache.delete(firstKey);
    }
    _gradientCache.set(key, g);
    return g;
  }

  function getCachedHudGradient(key, factory) {
    const cached = _hudGradientCache.get(key);
    if (cached) return cached;
    const gradient = factory();
    if (_hudGradientCache.size >= HUD_GRADIENT_CACHE_MAX) {
      const firstKey = _hudGradientCache.keys().next().value;
      _hudGradientCache.delete(firstKey);
    }
    _hudGradientCache.set(key, gradient);
    return gradient;
  }

  function getCachedTextWidth(canvasCtx, font, text) {
    const label = String(text || "");
    const key = `${font}|${label}`;
    const cached = _textWidthCache.get(key);
    if (cached != null) return cached;
    canvasCtx.save();
    canvasCtx.font = font;
    const width = canvasCtx.measureText(label).width;
    canvasCtx.restore();
    if (_textWidthCache.size >= TEXT_WIDTH_CACHE_MAX) {
      const firstKey = _textWidthCache.keys().next().value;
      _textWidthCache.delete(firstKey);
    }
    _textWidthCache.set(key, width);
    return width;
  }

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

  function hasEnumerableKeys(value) {
    if (!value || typeof value !== "object") return false;
    for (const _key in value) {
      return true;
    }
    return false;
  }

  function readRpyTriplet(liveCalibration) {
    const source = Array.isArray(liveCalibration?.rpyCalib) ? liveCalibration.rpyCalib : null;
    if (!source) return null;
    const roll = Number(source[0]);
    const pitch = Number(source[1]);
    const yaw = Number(source[2]);
    if (!Number.isFinite(roll) || !Number.isFinite(pitch) || !Number.isFinite(yaw)) return null;
    return [roll, pitch, yaw];
  }

  function formatRpyTriplet(liveCalibration) {
    const rpy = readRpyTriplet(liveCalibration);
    if (!rpy) return "-";
    return `${rpy[0].toFixed(3)},${rpy[1].toFixed(3)},${rpy[2].toFixed(3)}`;
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

  /* Phase 3: rgba string cache */
  const _rgbaCache = new Map();
  const RGBA_CACHE_MAX = 64;
  const _emptyDash = [];

  function rgba(rgb, alpha) {
    const a = clamp(alpha, 0, 1);
    const key = (rgb.r << 20) | (rgb.g << 10) | rgb.b | ((a * 1000 | 0) << 24);
    let s = _rgbaCache.get(key);
    if (s) return s;
    s = `rgba(${rgb.r}, ${rgb.g}, ${rgb.b}, ${a.toFixed(3)})`;
    if (_rgbaCache.size >= RGBA_CACHE_MAX) {
      _rgbaCache.delete(_rgbaCache.keys().next().value);
    }
    _rgbaCache.set(key, s);
    return s;
  }

  function paletteColor(index) {
    const normalized = ((Number(index) % PATH_PALETTE.length) + PATH_PALETTE.length) % PATH_PALETTE.length;
    return PATH_PALETTE[normalized] || PATH_PALETTE[3];
  }

  function resetFrameProjectionCache() {
    _frameProjectionCache = {
      pathLengthIdx: new WeakMap(),
      ribbon: new WeakMap(),
      verticalRibbon: new WeakMap(),
      pathZ: new WeakMap(),
      pathY: new WeakMap(),
    };
  }

  function getWeakCacheBucket(weakMap, target) {
    if (!target || typeof target !== "object") return null;
    let bucket = weakMap.get(target);
    if (!bucket) {
      bucket = new Map();
      weakMap.set(target, bucket);
    }
    return bucket;
  }

  function getProjectionSampleStride(distance, maxDistance) {
    const dist = finiteNumber(distance, 0);
    const maxDist = finiteNumber(maxDistance, MAX_DRAW_DISTANCE);
    if (maxDist <= 36) return 1;
    if (dist >= Math.min(maxDist * 0.72, 62)) return 3;
    if (dist >= Math.min(maxDist * 0.44, 34)) return 2;
    return 1;
  }

  function getProjectionSampleStrideForQuality(distance, maxDistance, quality = GEOMETRY_QUALITY_DEFAULT) {
    const baseStride = getProjectionSampleStride(distance, maxDistance);
    const dist = finiteNumber(distance, 0);

    if (quality === GEOMETRY_QUALITY_ROAD_EDGE) {
      if (dist >= 64) return Math.min(baseStride + 2, 5);
      if (dist >= 40) return Math.min(baseStride + 1, 4);
      return baseStride;
    }
    if (quality === GEOMETRY_QUALITY_LANE) {
      if (dist >= 58) return Math.min(baseStride + 1, 4);
      return baseStride;
    }
    return baseStride;
  }

  function forEachProjectedSampleIndex(xs, maxIdx, maxDistance, visitor, quality = GEOMETRY_QUALITY_DEFAULT) {
    if (maxIdx < 0) return;
    let i = 0;
    let lastVisited = -1;
    while (i <= maxIdx) {
      visitor(i);
      lastVisited = i;
      i += getProjectionSampleStrideForQuality(xs[i], maxDistance, quality);
    }
    if (lastVisited !== maxIdx) visitor(maxIdx);
  }

  function getGeometrySmoothingGain(quality, axis = "side") {
    if (quality === GEOMETRY_QUALITY_ROAD_EDGE) {
      return axis === "center" ? 1.12 : 1.22;
    }
    if (quality === GEOMETRY_QUALITY_LANE) {
      return axis === "center" ? 1.06 : 1.12;
    }
    return 1.0;
  }

  function getCachedRoundedRectPath(width, height, radius) {
    if (typeof Path2D !== "function") return null;
    const w = Math.max(0, finiteNumber(width, 0));
    const h = Math.max(0, finiteNumber(height, 0));
    const r = Math.min(finiteNumber(radius, 0), w / 2, h / 2);
    const key = `${Math.round(w * 2)}|${Math.round(h * 2)}|${Math.round(r * 2)}`;
    const cached = _roundedRectPathCache.get(key);
    if (cached) return cached;

    const path = new Path2D();
    path.moveTo(r, 0);
    path.lineTo(w - r, 0);
    path.quadraticCurveTo(w, 0, w, r);
    path.lineTo(w, h - r);
    path.quadraticCurveTo(w, h, w - r, h);
    path.lineTo(r, h);
    path.quadraticCurveTo(0, h, 0, h - r);
    path.lineTo(0, r);
    path.quadraticCurveTo(0, 0, r, 0);
    path.closePath();

    if (_roundedRectPathCache.size >= ROUNDED_RECT_CACHE_MAX) {
      _roundedRectPathCache.delete(_roundedRectPathCache.keys().next().value);
    }
    _roundedRectPathCache.set(key, path);
    return path;
  }

  function mixPoint(a, b, ratio) {
    return {
      x: a.x + (b.x - a.x) * ratio,
      y: a.y + (b.y - a.y) * ratio,
    };
  }

  function getPolylineSmoothingStrength(distance, maxDistance, maxStrength) {
    const dist = finiteNumber(distance, 0);
    const smoothFarDistance = Math.max(
      POLYLINE_SMOOTH_FAR_DISTANCE,
      Math.min(finiteNumber(maxDistance, MAX_DRAW_DISTANCE), MAX_DRAW_DISTANCE),
    );
    if (dist <= POLYLINE_SMOOTH_NEAR_DISTANCE) return 0;
    const ratio = clamp(
      (dist - POLYLINE_SMOOTH_NEAR_DISTANCE) /
      Math.max(1, smoothFarDistance - POLYLINE_SMOOTH_NEAR_DISTANCE),
      0,
      1,
    );
    return maxStrength * ratio;
  }

  function smoothProjectedPolyline(points, distances, maxDistance, maxStrength = POLYLINE_SMOOTH_MAX_STRENGTH) {
    if (!Array.isArray(points) || points.length < 3) return points;

    let smoothed = null;
    for (let i = 1; i < points.length - 1; i += 1) {
      const strength = getPolylineSmoothingStrength(distances?.[i], maxDistance, maxStrength);
      if (strength <= 0.001) continue;

      const prev = points[i - 1];
      const current = points[i];
      const next = points[i + 1];
      const targetX = prev.x * 0.25 + current.x * 0.5 + next.x * 0.25;
      const targetY = prev.y * 0.25 + current.y * 0.5 + next.y * 0.25;
      if (!smoothed) {
        smoothed = points.map((point) => ({ x: point.x, y: point.y }));
      }
      smoothed[i].x = current.x + (targetX - current.x) * strength;
      smoothed[i].y = current.y + (targetY - current.y) * strength;
    }
    return smoothed || points;
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

  /* Phase 3: reuse array to avoid per-call allocation */
  const _mv3Out = [0, 0, 0];
  function mat3Vector(a, v) {
    _mv3Out[0] = a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2];
    _mv3Out[1] = a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2];
    _mv3Out[2] = a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2];
    return _mv3Out;
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
    const rpy = readRpyTriplet(liveCalibration);
    if (!rpy) return VIEW_FROM_DEVICE;
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

  function getVisibleSourceRect(videoWidth, videoHeight, stageWidth = videoWidth, stageHeight = videoHeight, transform = null) {
    const scale = Math.max(finiteNumber(transform?.scale, 1), 0.01);
    const tx = finiteNumber(transform?.tx, 0);
    const ty = finiteNumber(transform?.ty, 0);
    const rawLeft = (0 - tx) / scale;
    const rawTop = (0 - ty) / scale;
    const rawRight = (stageWidth - tx) / scale;
    const rawBottom = (stageHeight - ty) / scale;
    const left = clamp(Math.min(rawLeft, rawRight), 0, videoWidth);
    const right = clamp(Math.max(rawLeft, rawRight), 0, videoWidth);
    const top = clamp(Math.min(rawTop, rawBottom), 0, videoHeight);
    const bottom = clamp(Math.max(rawTop, rawBottom), 0, videoHeight);

    if (right - left < 2 || bottom - top < 2) {
      return {
        left: 0,
        top: 0,
        right: videoWidth,
        bottom: videoHeight,
        width: videoWidth,
        height: videoHeight,
      };
    }

    return {
      left,
      top,
      right,
      bottom,
      width: right - left,
      height: bottom - top,
    };
  }

  function projectPoint(calibTransform, x, y, z) {
    mat3Vector(calibTransform, [x, y, z]);
    if (!Number.isFinite(_mv3Out[2]) || _mv3Out[2] <= 1e-3) return null;
    return {
      x: (_mv3Out[0] / _mv3Out[2]) | 0,
      y: (_mv3Out[1] / _mv3Out[2]) | 0,
    };
  }

  function projectPointPrecise(calibTransform, x, y, z) {
    mat3Vector(calibTransform, [x, y, z]);
    if (!Number.isFinite(_mv3Out[2]) || _mv3Out[2] <= 1e-3) return null;
    return {
      x: _mv3Out[0] / _mv3Out[2],
      y: _mv3Out[1] / _mv3Out[2],
    };
  }

  function getPathLengthIdx(line, maxDistance) {
    const cacheBucket = getWeakCacheBucket(_frameProjectionCache.pathLengthIdx, line);
    const cacheKey = Number.isFinite(Number(maxDistance))
      ? Math.round(Number(maxDistance) * 100)
      : "default";
    if (cacheBucket?.has(cacheKey)) return cacheBucket.get(cacheKey);

    const xs = Array.isArray(line?.x) ? line.x : [];
    let maxIdx = 0;
    for (let i = 1; i < xs.length; i += 1) {
      if (Number(xs[i]) > maxDistance) break;
      maxIdx = i;
    }
    cacheBucket?.set(cacheKey, maxIdx);
    return maxIdx;
  }

  function getHeldLeadBox(nowMs = performance.now()) {
    if (!leadHoldState.box) return null;
    if ((nowMs - finiteNumber(leadHoldState.lastValidMs, 0)) >= LEAD_HOLD_MS) return null;
    return leadHoldState.box;
  }

  function getPrimaryLeadDistance(overlayState = null, nowMs = performance.now()) {
    const leadOne = overlayState?.radarState?.leadOne;
    const liveDistance = finiteNumber(leadOne?.dRel, NaN);
    if (Boolean(leadOne?.status) && Number.isFinite(liveDistance) && liveDistance > 0) {
      return liveDistance;
    }

    const heldDistance = finiteNumber(getHeldLeadBox(nowMs)?.dRel, NaN);
    if (Number.isFinite(heldDistance) && heldDistance > 0) {
      return heldDistance;
    }
    return NaN;
  }

  function getSceneMaxDistance(model, overlayState = null) {
    const positionXs = Array.isArray(model?.position?.x) ? model.position.x : [];
    const tailX = finiteNumber(positionXs[positionXs.length - 1], MIN_DRAW_DISTANCE);
    let maxDistance = clamp(tailX, MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

    const leadDistance = getPrimaryLeadDistance(overlayState);
    if (Number.isFinite(leadDistance) && leadDistance > 0) {
      maxDistance = Math.min(maxDistance, leadDistance);
    }
    return maxDistance;
  }

  function getPathMaxDistance(sceneMaxDistance) {
    return Math.max(0, finiteNumber(sceneMaxDistance, 0) - 2.0);
  }

  function buildRibbon(calibTransform, line, halfWidth, zOffset, maxDistance, allowInvert = false, centerShift = 0, quality = GEOMETRY_QUALITY_DEFAULT) {
    const cacheBucket = getWeakCacheBucket(_frameProjectionCache.ribbon, line);
    const cacheKey = [
      Math.round(finiteNumber(halfWidth, 0) * 1000),
      Math.round(finiteNumber(zOffset, 0) * 1000),
      Math.round(finiteNumber(maxDistance, MAX_DRAW_DISTANCE) * 100),
      allowInvert ? 1 : 0,
      Math.round(finiteNumber(centerShift, 0) * 1000),
      quality,
    ].join("|");
    if (cacheBucket?.has(cacheKey)) return cacheBucket.get(cacheKey);

    const xs = Array.isArray(line?.x) ? line.x : [];
    const ys = Array.isArray(line?.y) ? line.y : [];
    const zs = Array.isArray(line?.z) ? line.z : [];
    const left = [];
    const right = [];
    const center = [];
    const distances = [];
    const maxIdx = getPathLengthIdx(line, maxDistance);

    forEachProjectedSampleIndex(xs, maxIdx, maxDistance, (i) => {
      const x = finiteNumber(xs[i], NaN);
      if (!Number.isFinite(x) || x < 0) return;

      const y = finiteNumber(ys[i], 0) + centerShift;
      const z = finiteNumber(zs[i], 0) + zOffset;
      const leftPt = projectPoint(calibTransform, x, y - halfWidth, z);
      const rightPt = projectPoint(calibTransform, x, y + halfWidth, z);
      const centerPt = projectPoint(calibTransform, x, y, z);
      if (!leftPt || !rightPt || !centerPt) return;
      if (!allowInvert && center.length && centerPt.y > center[center.length - 1].y) return;

      left.push(leftPt);
      right.push(rightPt);
      center.push(centerPt);
      distances.push(x);
    }, quality);

    const smoothMaxDistance = finiteNumber(maxDistance, MAX_DRAW_DISTANCE);
    const sideSmoothGain = getGeometrySmoothingGain(quality, "side");
    const centerSmoothGain = getGeometrySmoothingGain(quality, "center");
    const smoothedLeft = smoothProjectedPolyline(left, distances, smoothMaxDistance, POLYLINE_SMOOTH_MAX_STRENGTH * sideSmoothGain);
    const smoothedRight = smoothProjectedPolyline(right, distances, smoothMaxDistance, POLYLINE_SMOOTH_MAX_STRENGTH * sideSmoothGain);
    const smoothedCenter = smoothProjectedPolyline(center, distances, smoothMaxDistance, POLYLINE_CENTER_SMOOTH_MAX_STRENGTH * centerSmoothGain);
    const result = {
      left: smoothedLeft,
      right: smoothedRight,
      center: smoothedCenter,
      polygon: smoothedLeft.length >= 2 && smoothedRight.length >= 2 ? smoothedLeft.concat([...smoothedRight].reverse()) : [],
    };
    cacheBucket?.set(cacheKey, result);
    return result;
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
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i += 1) {
      ctx.lineTo(points[i].x, points[i].y);
    }
    if (dashPattern.length) {
      ctx.setLineDash(dashPattern);
      ctx.lineDashOffset = dashOffset;
    } else {
      ctx.setLineDash(_emptyDash);
    }
    ctx.lineWidth = lineWidth;
    ctx.strokeStyle = strokeStyle;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.stroke();
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
      IsMetric: finiteParamNumber(source.IsMetric, fallback.IsMetric),
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
      RadarLatFactor: finiteParamNumber(source.RadarLatFactor, fallback.RadarLatFactor),
      CustomSR: finiteParamNumber(source.CustomSR, fallback.CustomSR),
    };
  }

  function readLiveRuntimeParams() {
    const runtimeParams = window.CarrotLiveRuntimeState?.runtime?.params;
    if (!runtimeParams || typeof runtimeParams !== "object") return null;

    const normalized = normalizeVisualParams(runtimeParams, paramsState);
    const hasPathKeys = (
      runtimeParams.IsMetric != null ||
      runtimeParams.ShowPathEnd != null ||
      runtimeParams.ShowPathMode != null ||
      runtimeParams.ShowPathColor != null ||
      runtimeParams.ShowPathModeLane != null ||
      runtimeParams.ShowPathColorLane != null ||
      runtimeParams.ShowLaneInfo != null ||
      runtimeParams.ShowRadarInfo != null ||
      runtimeParams.RadarLatFactor != null ||
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
    if (raw === live) return raw;
    if (!hasEnumerableKeys(live)) return raw;
    if (!hasEnumerableKeys(raw)) return live;
    return { ...raw, ...live };
  }

  function mergeDefinedState(baseState, preferredState) {
    const base = baseState && typeof baseState === "object" ? baseState : {};
    if (!preferredState || typeof preferredState !== "object") return base;
    let merged = null;
    for (const [key, value] of Object.entries(preferredState)) {
      if (value === undefined || value === null) continue;
      if (!merged) merged = { ...base };
      merged[key] = value;
    }
    return merged || base;
  }

  function mergeRadarLead(rawLead, liveLead) {
    return mergeDefinedState(liveLead, rawLead);
  }

  function mergeRadarState(rawState, liveState) {
    const raw = rawState && typeof rawState === "object" ? rawState : {};
    const live = liveState && typeof liveState === "object" ? liveState : {};
    if (raw === live) return raw;
    if (!hasEnumerableKeys(live)) return raw;
    if (!hasEnumerableKeys(raw)) return live;

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
      fcw: currentLead.fcw,
      status: currentLead.status,
      radar: currentLead.radar,
      radarTrackId: currentLead.radarTrackId,
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
    const cachedRefs = _mergeRuntimeCache.refs;
    if (
      cachedRefs &&
      cachedRefs.rawHudState === rawHudState &&
      cachedRefs.rawOverlayState === rawOverlayState &&
      cachedRefs.liveServices === liveServices &&
      cachedRefs.rawCarState === rawHudState?.carState &&
      cachedRefs.rawControlsState === rawHudState?.controlsState &&
      cachedRefs.rawDeviceState === rawHudState?.deviceState &&
      cachedRefs.rawPeripheralState === rawHudState?.peripheralState &&
      cachedRefs.rawSelfdriveState === rawHudState?.selfdriveState &&
      cachedRefs.rawGpsLocationExternal === rawHudState?.gpsLocationExternal &&
      cachedRefs.rawLongitudinalPlan === rawHudState?.longitudinalPlan &&
      cachedRefs.rawCarrotMan === rawHudState?.carrotMan &&
      cachedRefs.rawModelV2 === rawOverlayState?.modelV2 &&
      cachedRefs.rawLiveCalibration === rawOverlayState?.liveCalibration &&
      cachedRefs.rawRoadCameraState === rawOverlayState?.roadCameraState &&
      cachedRefs.rawRadarState === rawOverlayState?.radarState &&
      cachedRefs.rawLateralPlan === rawOverlayState?.lateralPlan &&
      cachedRefs.rawCarControl === rawOverlayState?.carControl &&
      cachedRefs.rawLiveDelay === rawOverlayState?.liveDelay &&
      cachedRefs.rawLiveTorqueParameters === rawOverlayState?.liveTorqueParameters &&
      cachedRefs.rawLiveParameters === rawOverlayState?.liveParameters &&
      cachedRefs.liveCarState === liveServices?.carState &&
      cachedRefs.liveControlsState === liveServices?.controlsState &&
      cachedRefs.liveSelfdriveState === liveServices?.selfdriveState &&
      cachedRefs.liveLongitudinalPlan === liveServices?.longitudinalPlan &&
      cachedRefs.liveCarrotMan === liveServices?.carrotMan &&
      cachedRefs.liveLateralPlan === liveServices?.lateralPlan &&
      cachedRefs.liveRadarState === liveServices?.radarState
    ) {
      return _mergeRuntimeCache.result;
    }

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

    const result = {
      brokerServices: liveServices,
      hudState: mergedHudState,
      overlayState: mergedOverlayState,
    };
    _mergeRuntimeCache.refs = {
      rawHudState,
      rawOverlayState,
      liveServices,
      rawCarState: rawHudState?.carState,
      rawControlsState: rawHudState?.controlsState,
      rawDeviceState: rawHudState?.deviceState,
      rawPeripheralState: rawHudState?.peripheralState,
      rawSelfdriveState: rawHudState?.selfdriveState,
      rawGpsLocationExternal: rawHudState?.gpsLocationExternal,
      rawLongitudinalPlan: rawHudState?.longitudinalPlan,
      rawCarrotMan: rawHudState?.carrotMan,
      rawModelV2: rawOverlayState?.modelV2,
      rawLiveCalibration: rawOverlayState?.liveCalibration,
      rawRoadCameraState: rawOverlayState?.roadCameraState,
      rawRadarState: rawOverlayState?.radarState,
      rawLateralPlan: rawOverlayState?.lateralPlan,
      rawCarControl: rawOverlayState?.carControl,
      rawLiveDelay: rawOverlayState?.liveDelay,
      rawLiveTorqueParameters: rawOverlayState?.liveTorqueParameters,
      rawLiveParameters: rawOverlayState?.liveParameters,
      liveCarState: liveServices?.carState,
      liveControlsState: liveServices?.controlsState,
      liveSelfdriveState: liveServices?.selfdriveState,
      liveLongitudinalPlan: liveServices?.longitudinalPlan,
      liveCarrotMan: liveServices?.carrotMan,
      liveLateralPlan: liveServices?.lateralPlan,
      liveRadarState: liveServices?.radarState,
    };
    _mergeRuntimeCache.result = result;
    return result;
  }

  function firstNonEmptyText(...values) {
    for (const value of values) {
      const text = String(value || "").trim();
      if (text) return text;
    }
    return "";
  }

  function buildOverlayCarLabel(values = {}) {
    let label = firstNonEmptyText(values.CarName, values.CarSelected3);
    if (!label) return "";
    if (finiteParamNumber(values.HyundaiCameraSCC, 0) > 0) {
      label += "(CAMERA SCC)";
    }
    if (firstNonEmptyText(values.NNFFModelName)) {
      label += ",NNFF";
    }
    return label;
  }

  function buildOverlayBranchLabel(values = {}) {
    const branch = firstNonEmptyText(values.GitBranch);
    const commit = firstNonEmptyText(values.GitCommit);
    if (!branch && !commit) return "";
    if (!branch) return shortText(commit, 12);
    return commit ? `${branch} (${commit.slice(0, 7)})` : branch;
  }

  async function refreshOverlayInfo(force = false) {
    if (!force && overlayInfoState.loading) return;
    if (!force && overlayInfoState.nextRetryAt > Date.now()) return;
    if (typeof bulkGet !== "function") return;

    overlayInfoState.loading = true;
    try {
      const values = await bulkGet([
        "CarName",
        "CarSelected3",
        "HyundaiCameraSCC",
        "NNFFModelName",
        "GitBranch",
        "GitCommit",
      ]);
      const carLabel = buildOverlayCarLabel(values);
      const branchLabel = buildOverlayBranchLabel(values);
      const signature = `${carLabel}|${branchLabel}`;
      overlayInfoState.carLabel = carLabel;
      overlayInfoState.branchLabel = branchLabel;
      overlayInfoState.nextRetryAt = signature ? 0 : (Date.now() + 5000);
      if (signature !== overlayInfoState.lastSignature) {
        overlayInfoState.lastSignature = signature;
        requestRender({ force: true, hudDirty: true });
      }
    } catch {
      overlayInfoState.nextRetryAt = Date.now() + 5000;
    }
    overlayInfoState.loading = false;
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

  function hideOnroadAlert() {
    if (!onroadAlertEl || !onroadAlertBoxEl || !onroadAlertText1El || !onroadAlertText2El) return;
    if (_lastAlertSig === "hidden") return;
    _lastAlertSig = "hidden";
    onroadAlertEl.hidden = true;
    onroadAlertEl.className = "carrot-stage__alert";
    onroadAlertText1El.textContent = "";
    onroadAlertText2El.textContent = "";
    onroadAlertText2El.hidden = true;
  }

  function getAlertStatusClass(status) {
    switch (finiteNumber(status, ALERT_STATUS_NORMAL)) {
      case ALERT_STATUS_USER_PROMPT:
        return "alert-status--user-prompt";
      case ALERT_STATUS_CRITICAL:
        return "alert-status--critical";
      default:
        return "alert-status--normal";
    }
  }

  function getAlertSizeClass(size) {
    switch (finiteNumber(size, ALERT_SIZE_NONE)) {
      case ALERT_SIZE_SMALL:
        return "alert-size--small";
      case ALERT_SIZE_MID:
        return "alert-size--mid";
      case ALERT_SIZE_FULL:
        return "alert-size--full";
      default:
        return "alert-size--none";
    }
  }

  function renderOnroadAlert(stageWidth, stageHeight, selfdriveState) {
    if (!onroadAlertEl || !onroadAlertBoxEl || !onroadAlertText1El || !onroadAlertText2El) return;

    const text1 = String(selfdriveState?.alertText1 || "").trim();
    const text2 = String(selfdriveState?.alertText2 || "").trim();
    const alertType = String(selfdriveState?.alertType || "").trim();
    const alertSize = finiteNumber(selfdriveState?.alertSize, ALERT_SIZE_NONE);
    const alertStatus = finiteNumber(selfdriveState?.alertStatus, ALERT_STATUS_NORMAL);

    if (alertSize === ALERT_SIZE_NONE || (!text1 && !text2 && !alertType)) {
      hideOnroadAlert();
      return;
    }

    const isPortrait = stageHeight > stageWidth;
    const longPrimaryText = text1.length > 15;
    const stageScale = clamp(Math.min(stageWidth / BASE_CAMERA.width, stageHeight / BASE_CAMERA.height), 0.52, 0.90);
    const displayModeScale = displayModeIndex === 0 ? 0.88 : displayModeIndex === 2 ? 0.96 : 0.92;
    const orientationScale = isPortrait ? 0.80 : 0.90;
    const widthScale = isPortrait ? clamp(stageWidth / 420, 0.72, 0.94) : 1.0;
    const resolutionScale = isPortrait
      ? clamp(stageHeight / BASE_CAMERA.height, 0.98, 1.06)
      : clamp(stageWidth / BASE_CAMERA.width, 1.00, 1.08);
    const textScale = stageScale * displayModeScale * orientationScale * widthScale * resolutionScale * 1.24;
    const primaryBase = alertSize === ALERT_SIZE_SMALL
      ? 32
      : alertSize === ALERT_SIZE_MID
        ? 40
        : (longPrimaryText ? 44 : 48);
    const secondaryBase = alertSize === ALERT_SIZE_SMALL ? 0 : (alertSize === ALERT_SIZE_MID ? 20 : 24);
    const fontSize1 = clamp(
      Math.round(primaryBase * textScale * ONROAD_ALERT_SCALE),
      Math.round((alertSize === ALERT_SIZE_SMALL ? 16 : alertSize === ALERT_SIZE_MID ? 20 : 22) * ONROAD_ALERT_SCALE),
      Math.round((alertSize === ALERT_SIZE_SMALL ? 32 : alertSize === ALERT_SIZE_MID ? 40 : 44) * ONROAD_ALERT_SCALE),
    );
    const fontSize2 = alertSize === ALERT_SIZE_SMALL
      ? 0
      : clamp(
        Math.round(secondaryBase * textScale * ONROAD_ALERT_SCALE),
        Math.round(14 * ONROAD_ALERT_SCALE),
        Math.round(29 * ONROAD_ALERT_SCALE),
      );
    const offsetRatio = isPortrait ? 0.06 : 0.11;
    const displayModeYOffset = displayModeIndex === 0 ? -6 : displayModeIndex === 2 ? 6 : 0;
    const offsetY = Math.round(stageHeight * offsetRatio) + displayModeYOffset;
    const gap = text2 && alertSize !== ALERT_SIZE_SMALL
      ? clamp(Math.round(fontSize1 * 0.16), Math.round(6 * ONROAD_ALERT_SCALE), Math.round(14 * ONROAD_ALERT_SCALE))
      : 0;
    const maxWidthRatio = alertSize === ALERT_SIZE_FULL
      ? (isPortrait ? 0.94 : 0.84)
      : (isPortrait ? 0.90 : 0.74);
    const maxWidth = Math.round(stageWidth * maxWidthRatio);
    const primaryColor = alertStatus === ALERT_STATUS_CRITICAL ? "#ff5a63" : "#ffb12a";
    const secondaryColor = alertStatus === ALERT_STATUS_CRITICAL ? "#ffe3e5" : "#ffffff";

    const signature = [
      Math.round(stageWidth),
      Math.round(stageHeight),
      displayModeIndex,
      alertSize,
      alertStatus,
      text1,
      text2,
      alertType,
      fontSize1,
      fontSize2,
      offsetY,
      gap,
      maxWidth,
      ONROAD_ALERT_SCALE,
      primaryColor,
      secondaryColor,
    ].join("|");
    if (_lastAlertSig === signature) return;
    _lastAlertSig = signature;

    onroadAlertEl.hidden = false;
    onroadAlertEl.className = `carrot-stage__alert is-visible ${getAlertSizeClass(alertSize)} ${getAlertStatusClass(alertStatus)}`;
    onroadAlertEl.classList.toggle("has-text2", Boolean(text2));
    onroadAlertEl.classList.toggle("is-long-text1", longPrimaryText);
    onroadAlertEl.style.setProperty("--carrot-alert-offset-y", `${offsetY}px`);
    onroadAlertEl.style.setProperty("--carrot-alert-gap", `${gap}px`);
    onroadAlertEl.style.setProperty("--carrot-alert-max-width", `${maxWidth}px`);
    onroadAlertEl.style.setProperty("--carrot-alert-font1", `${fontSize1}px`);
    onroadAlertEl.style.setProperty("--carrot-alert-font2", `${Math.max(fontSize2, 0)}px`);
    onroadAlertEl.style.setProperty("--carrot-alert-scale", `${ONROAD_ALERT_SCALE}`);
    onroadAlertEl.style.setProperty("--carrot-alert-primary-color", primaryColor);
    onroadAlertEl.style.setProperty("--carrot-alert-secondary-color", secondaryColor);

    onroadAlertText1El.textContent = text1;
    onroadAlertText2El.textContent = text2;
    onroadAlertText2El.hidden = !text2 || alertSize === ALERT_SIZE_SMALL;
  }

  function syncDisplayModeButtons() {
    if (!displayModeButton) return;
    const mode = DISPLAY_MODES[displayModeIndex] || DISPLAY_MODES[1];
    const label = getDisplayModeLabel(mode);
    displayModeButton.textContent = label || "Normal";
    displayModeButton.setAttribute("aria-label", `${getUIText("display_mode", "Display mode")}: ${label}`);
    displayModeButton.title = label;
  }

  function setDisplayModeIndex(nextIndex) {
    displayModeIndex = clamp(nextIndex, 0, DISPLAY_MODES.length - 1);
    try {
      localStorage.setItem(DISPLAY_MODE_STORAGE_KEY, String(displayModeIndex));
    } catch {}
    transformSignature = "";
    syncDisplayModeButtons();
  }

  function getDisplayModeLabel(mode) {
    if (!mode) return getUIText("display_normal", "Normal");
    return getUIText(mode.labelKey, mode.fallbackLabel || mode.key || "Normal");
  }

  function syncSourceStream() {
    const stream = sourceVideoEl.srcObject || null;
    if (sourceVideoEl !== videoEl && videoEl.srcObject !== stream) {
      videoEl.srcObject = stream;
    }

    const hasStream = Boolean((sourceVideoEl === videoEl ? sourceVideoEl : videoEl).srcObject || stream);
    if (hasStream && videoEl.paused) {
      videoEl.play().catch(() => {});
    }
    return hasStream;
  }

  let _lastStageReady = null;
  function setStageReady(ready) {
    const r = Boolean(ready);
    if (_lastStageReady === r) return;
    _lastStageReady = r;
    stageEl.classList.toggle("is-stream-ready", r);
    videoEl.style.display = r ? "block" : "none";
  }

  let _lastStageLoading = null;
  let _lastStageLoadingText = "";

  function setStageLoading(loading, text = getUIText("connecting", "Connecting...")) {
    const l = Boolean(loading);
    if (_lastStageLoading !== l) {
      _lastStageLoading = l;
      stageEl.classList.toggle("is-loading", l);
      if (driveHudCardEl) driveHudCardEl.classList.toggle("driveHudCard--loading", l);
      if (stageLoadingEl) stageLoadingEl.hidden = !l;
    }
    if (!l) return;
    if (stageLoadingTextEl && _lastStageLoadingText !== text) {
      _lastStageLoadingText = text;
      stageLoadingTextEl.textContent = text;
    }
  }

  function resetCarrotHudLayout() {
    if (!driveHudCardEl) return;
    driveHudCardEl.style.removeProperty("--carrot-hud-left");
    driveHudCardEl.style.removeProperty("--carrot-hud-bottom");
  }

  let _lastHudLeft = "";
  let _lastHudBottom = "";

  function applyCarrotHudLayout(viewportRect) {
    if (!driveHudCardEl) return;
    if (window.matchMedia("(orientation: portrait)").matches) {
      if (_lastHudLeft !== "" || _lastHudBottom !== "") {
        driveHudCardEl.style.removeProperty("--carrot-hud-left");
        driveHudCardEl.style.removeProperty("--carrot-hud-bottom");
        _lastHudLeft = "";
        _lastHudBottom = "";
      }
      return;
    }
    const stageWidth = stageEl?.clientWidth || viewportRect?.width || 0;
    const stageHeight = stageEl?.clientHeight || viewportRect?.height || 0;
    if (!stageWidth || !stageHeight) return;
    const overlayInsetX = clamp(stageWidth * 0.028, 16, 28);
    const overlayInsetY = clamp(stageHeight * 0.038, 20, 30);
    const leftVal = `${Math.round(overlayInsetX)}px`;
    const bottomVal = `${Math.round(overlayInsetY)}px`;

    if (_lastHudLeft !== leftVal) {
      _lastHudLeft = leftVal;
      driveHudCardEl.style.setProperty("--carrot-hud-left", leftVal);
    }
    if (_lastHudBottom !== bottomVal) {
      _lastHudBottom = bottomVal;
      driveHudCardEl.style.setProperty("--carrot-hud-bottom", bottomVal);
    }
  }

  function getCanvasDpr() {
    const rawDpr = window.devicePixelRatio || 1;
    const portrait = window.matchMedia("(orientation: portrait)").matches;
    const shortSide = Math.min(window.innerWidth || 0, window.innerHeight || 0);
    if (portrait && shortSide > 0 && shortSide <= 540) {
      return Math.min(rawDpr, PHONE_PORTRAIT_DPR_CAP);
    }
    const cap = shortSide >= 960 ? DESKTOP_DPR_CAP : MOBILE_DPR_CAP;
    return Math.min(rawDpr, cap);
  }

  function syncCanvasSize(videoWidth, videoHeight, stageWidth, stageHeight) {
    const dpr = getCanvasDpr();

    const nextOverlaySignature = `${videoWidth}x${videoHeight}@${dpr.toFixed(2)}`;
    if (overlaySizeSignature !== nextOverlaySignature) {
      overlaySizeSignature = nextOverlaySignature;
      videoEl.style.width = `${videoWidth}px`;
      videoEl.style.height = `${videoHeight}px`;
      if (videoHoldEl) {
        videoHoldEl.style.width = `${videoWidth}px`;
        videoHoldEl.style.height = `${videoHeight}px`;
        videoHoldEl.width = Math.max(1, Math.round(videoWidth * dpr));
        videoHoldEl.height = Math.max(1, Math.round(videoHeight * dpr));
      }
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
    if (videoHoldEl) videoHoldEl.style.transform = cssMatrix;
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
    const font = `${fontWeight} ${fontSize}px ${HUD_TEXT_FONT}`;
    const measured = getCachedTextWidth(hudCtx, font, label);
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
    const mode = finiteNumber(style?.mode, 0);
    let solidAlpha = mode === 0 ? 0.40 : 0.24;
    let midAlpha = mode === 0 ? 0.20 : 0.12;

    if (mode === 0 || style?.isCruiseOff) {
      solidAlpha = Math.max(solidAlpha, TEST_PATH_VISIBILITY_SOLID_ALPHA);
      midAlpha = Math.max(midAlpha, TEST_PATH_VISIBILITY_MID_ALPHA);
    }

    const cacheKey = `g:${baseColor.r},${baseColor.g},${baseColor.b}|${Math.round(canvasHeight)}|${solidAlpha}|${midAlpha}`;
    return getCachedGradient(cacheKey, () => {
      const gradient = ctx.createLinearGradient(0, canvasHeight, 0, canvasHeight * 0.32);
      gradient.addColorStop(0, rgba(baseColor, solidAlpha));
      gradient.addColorStop(0.55, rgba(baseColor, midAlpha));
      gradient.addColorStop(1, rgba(baseColor, 0.0));
      return gradient;
    });
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

  function drawPath(pathData, model, overlayState, calibTransform, canvasHeight, style) {
    if (!pathData || !Array.isArray(pathData.x) || !pathData.x.length) return;
    const sceneMaxDistance = getSceneMaxDistance(model, overlayState);
    const ribbon = buildRibbon(calibTransform, pathData, getPathHalfWidth(), PATH_Z_OFFSET, getPathMaxDistance(sceneMaxDistance), false);
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

  function drawLaneLines(model, overlayState, hudState, calibTransform) {
    const laneLines = Array.isArray(model?.laneLines) ? model.laneLines : [];
    const laneLineProbs = Array.isArray(model?.laneLineProbs) ? model.laneLineProbs : [];
    if (!laneLines.length) return;

    const leftLaneLine = finiteNumber(hudState?.carState?.leftLaneLine, 0);
    const rightLaneLine = finiteNumber(hudState?.carState?.rightLaneLine, 0);
    const sceneMaxDistance = getSceneMaxDistance(model, overlayState);
    const maxIdx = getPathLengthIdx(laneLines[0], sceneMaxDistance);
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
      const ribbon = buildRibbon(
        calibTransform,
        laneLines[i],
        halfWidth,
        0,
        finiteNumber(laneLines[i]?.x?.[maxIdx], MAX_DRAW_DISTANCE),
        false,
        0,
        GEOMETRY_QUALITY_LANE,
      );
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
          GEOMETRY_QUALITY_LANE,
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

  function drawRoadEdges(model, overlayState, calibTransform) {
    const roadEdges = Array.isArray(model?.roadEdges) ? model.roadEdges : [];
    const roadEdgeStds = Array.isArray(model?.roadEdgeStds) ? model.roadEdgeStds : [];
    if (!roadEdges.length) return;

    const sceneMaxDistance = getSceneMaxDistance(model, overlayState);
    const maxIdx = getPathLengthIdx(roadEdges[0], sceneMaxDistance);
    for (let i = 0; i < roadEdges.length; i += 1) {
      const edgeStd = clamp(finiteNumber(roadEdgeStds[i], 0.4), 0, 1);
      const alpha = clamp(1 - edgeStd, 0.12, 0.66);
      const ribbon = buildRibbon(
        calibTransform,
        roadEdges[i],
        0.025,
        0,
        finiteNumber(roadEdges[i]?.x?.[maxIdx], MAX_DRAW_DISTANCE),
        false,
        0,
        GEOMETRY_QUALITY_ROAD_EDGE,
      );
      drawPolygon(
        ribbon.polygon,
        `rgba(255,78,59,${alpha.toFixed(3)})`,
        "rgba(255,124,104,0.28)",
        1,
      );
    }
  }

  function samplePathZ(position, distance) {
    const cacheBucket = getWeakCacheBucket(_frameProjectionCache.pathZ, position);
    const cacheKey = Math.round(finiteNumber(distance, 0) * 100);
    if (cacheBucket?.has(cacheKey)) return cacheBucket.get(cacheKey);

    const zs = Array.isArray(position?.z) ? position.z : [];
    const idx = getPathLengthIdx(position, distance);
    const value = finiteNumber(zs[idx], 0);
    cacheBucket?.set(cacheKey, value);
    return value;
  }

  function samplePathY(position, distance) {
    const cacheBucket = getWeakCacheBucket(_frameProjectionCache.pathY, position);
    const cacheKey = Math.round(finiteNumber(distance, 0) * 100);
    if (cacheBucket?.has(cacheKey)) return cacheBucket.get(cacheKey);

    const value = interp1D(
      distance,
      Array.isArray(position?.x) ? position.x : [],
      Array.isArray(position?.y) ? position.y : [],
    );
    cacheBucket?.set(cacheKey, value);
    return value;
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
    const cacheBucket = getWeakCacheBucket(_frameProjectionCache.verticalRibbon, line);
    const cacheKey = [
      Math.round(finiteNumber(centerShift, 0) * 1000),
      Math.round(finiteNumber(topZOffset, 0) * 1000),
      Math.round(finiteNumber(bottomZOffset, 0) * 1000),
      Math.round(finiteNumber(maxDistance, MAX_DRAW_DISTANCE) * 100),
    ].join("|");
    if (cacheBucket?.has(cacheKey)) return cacheBucket.get(cacheKey);

    const xs = Array.isArray(line?.x) ? line.x : [];
    const ys = Array.isArray(line?.y) ? line.y : [];
    const zs = Array.isArray(line?.z) ? line.z : [];
    const top = [];
    const bottom = [];
    const distances = [];
    const maxIdx = getPathLengthIdx(line, maxDistance);

    forEachProjectedSampleIndex(xs, maxIdx, maxDistance, (i) => {
      const x = finiteNumber(xs[i], NaN);
      if (!Number.isFinite(x) || x < 0) return;

      const y = finiteNumber(ys[i], 0) + centerShift;
      const z = finiteNumber(zs[i], 0);
      const topPoint = projectPoint(calibTransform, x, y, z + topZOffset);
      const bottomPoint = projectPoint(calibTransform, x, y, z + bottomZOffset);
      if (!topPoint || !bottomPoint) return;
      if (top.length && topPoint.y > top[top.length - 1].y) return;

      top.push(topPoint);
      bottom.push(bottomPoint);
      distances.push(x);
    });

    const smoothMaxDistance = finiteNumber(maxDistance, MAX_DRAW_DISTANCE);
    const smoothedTop = smoothProjectedPolyline(top, distances, smoothMaxDistance, POLYLINE_SMOOTH_MAX_STRENGTH);
    const smoothedBottom = smoothProjectedPolyline(bottom, distances, smoothMaxDistance, POLYLINE_SMOOTH_MAX_STRENGTH);
    const result = smoothedTop.length >= 2 && smoothedBottom.length >= 2 ? smoothedTop.concat([...smoothedBottom].reverse()) : [];
    cacheBucket?.set(cacheKey, result);
    return result;
  }

  function resetLeadEmaSlot(slot) {
    if (slot !== 0 && slot !== 1) return;
    leadEmaState[slot] = { fx: NaN, fy: NaN, fw: NaN, trackId: -1, lastMs: 0 };
  }

  function resetLeadTwoEma() {
    leadTwoEmaState = { xl: NaN, xr: NaN, y: NaN, lastMs: 0 };
  }

  function syncLeadRenderState(videoWidth, videoHeight, modelFrameId, cameraFrameId) {
    const nextWidth = finiteNumber(videoWidth, NaN);
    const nextHeight = finiteNumber(videoHeight, NaN);
    const nextModelFrameId = finiteNumber(modelFrameId, NaN);
    const nextCameraFrameId = finiteNumber(cameraFrameId, NaN);
    const prev = leadRenderState;

    const sourceChanged =
      Number.isFinite(prev.lastSourceWidth) &&
      Number.isFinite(prev.lastSourceHeight) &&
      (Math.abs(prev.lastSourceWidth - nextWidth) > 0.5 || Math.abs(prev.lastSourceHeight - nextHeight) > 0.5);
    const cameraFrameRewind =
      Number.isFinite(prev.lastCameraFrameId) &&
      Number.isFinite(nextCameraFrameId) &&
      nextCameraFrameId < prev.lastCameraFrameId;
    const modelFrameRewind =
      Number.isFinite(prev.lastModelFrameId) &&
      Number.isFinite(nextModelFrameId) &&
      nextModelFrameId < prev.lastModelFrameId;
    if (sourceChanged || cameraFrameRewind || modelFrameRewind) {
      resetLeadEmaSlot(0);
      resetLeadEmaSlot(1);
      resetLeadTwoEma();
      leadHoldState.box = null;
    }

    leadRenderState = {
      lastCameraFrameId: nextCameraFrameId,
      lastModelFrameId: nextModelFrameId,
      lastSourceWidth: nextWidth,
      lastSourceHeight: nextHeight,
    };
  }

  function getLeadBadgeOffsets(videoWidth, videoHeight) {
    const scaleX = videoWidth / BASE_CAMERA.width;
    const scaleY = videoHeight / BASE_CAMERA.height;
    return {
      dx: 80 * scaleX,
      rectTopOffset: 25 * scaleY,
      textBaselineOffset: 60 * scaleY,
      badgeHeight: Math.max(42 * scaleY, 26),
      fontSize: Math.max(40 * scaleY, 20),
    };
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

  function getLeadBoxClampMargins(videoWidth, videoHeight, stageWidth = videoWidth, stageHeight = videoHeight, transform = null, options = {}) {
    const visibleRect = getVisibleSourceRect(videoWidth, videoHeight, stageWidth, stageHeight, transform);
    // C3 fixed margins: top=200, bottom=80, marginX=350
    const topMargin = Math.max(200.0, visibleRect.top + 6);

    // C3 base: maxCenterY = fb_h - 80
    let maxCenterY = videoHeight - 80.0;

    // In crop/fit modes, also keep badges inside visible area
    const offsets = getLeadBadgeOffsets(videoWidth, videoHeight);
    let badgeReserve = 0;
    if (options.includeDistanceBadge !== false) {
      badgeReserve = Math.max(badgeReserve, offsets.rectTopOffset + offsets.badgeHeight + 8);
    }
    if (options.includeStateText) {
      badgeReserve = Math.max(badgeReserve, offsets.textBaselineOffset + Math.max(offsets.fontSize * 0.28, 8));
    }
    maxCenterY = Math.min(maxCenterY, visibleRect.bottom - Math.max(badgeReserve, 80.0));
    maxCenterY = Math.max(topMargin, maxCenterY);

    return {
      marginX: 350.0,
      topMargin,
      bottomMargin: Math.max(80.0, videoHeight - maxCenterY),
      maxCenterY,
      visibleRect,
      bottomReserve: badgeReserve,
    };
  }

  function projectLeadBox(lead, modelPath, calibTransform, videoWidth, videoHeight, slot = 0, stageWidth = videoWidth, stageHeight = videoHeight, transform = null, options = {}) {
    if (!lead?.status) return null;
    const distance = finiteNumber(lead.dRel, NaN);
    if (!Number.isFinite(distance) || distance <= 0) return null;

    const yCenter = -finiteNumber(lead.yRel, 0);
    const z = samplePathZ(modelPath, distance) + PATH_Z_OFFSET;
    const left = projectPointPrecise(calibTransform, distance, yCenter - 1.2, z);
    const right = projectPointPrecise(calibTransform, distance, yCenter + 1.2, z);
    if (!left || !right) return null;

    const rawWidth = Math.abs(right.x - left.x);
    if (!Number.isFinite(rawWidth) || rawWidth <= 1) return null;

    const rawCenterX = (left.x + right.x) * 0.5;
    const rawCenterY = (left.y + right.y) * 0.5;
    const { marginX, topMargin, maxCenterY } = getLeadBoxClampMargins(
      videoWidth,
      videoHeight,
      stageWidth,
      stageHeight,
      transform,
      options,
    );

    // Match CarrotLink's adaptive bottom margin while keeping carrot.cc clamp policy.
    const _path_x = clamp(rawCenterX, marginX, Math.max(marginX, videoWidth - marginX));
    const _path_y = clamp(rawCenterY, topMargin, maxCenterY);
    const _path_width = clamp(rawWidth, 120, 800);

    // ── Step 2: Time-based EMA on clamped values ──
    // C3 uses alpha=0.85 at stable 20Hz.  Web frame rate varies, so
    // alpha_adj = 0.85^(dt/50ms) gives identical wall-clock convergence.
    const ema = leadEmaState[slot] || { fx: NaN, fy: NaN, fw: NaN, trackId: -1, lastMs: 0 };
    const currentTrackId = (lead.radarTrackId != null) ? finiteNumber(lead.radarTrackId, -1) : -1;
    const nowMs = performance.now();
    const dt = ema.lastMs > 0 ? clamp(nowMs - ema.lastMs, 1, 500) : C3_FRAME_MS;
    const alpha = Math.pow(LEAD_EMA_ALPHA, dt / C3_FRAME_MS);
    const hasPrev = Number.isFinite(ema.fx) && Number.isFinite(ema.fy) && Number.isFinite(ema.fw);
    const path_fx = hasPrev ? (ema.fx * alpha + _path_x * (1 - alpha)) : _path_x;
    const path_fy = hasPrev ? (ema.fy * alpha + _path_y * (1 - alpha)) : _path_y;
    const path_fw = hasPrev ? (ema.fw * alpha + _path_width * (1 - alpha)) : _path_width;
    leadEmaState[slot] = { fx: path_fx, fy: path_fy, fw: path_fw, trackId: currentTrackId, lastMs: nowMs };

    // ── Step 3: Build box from smoothed values ──
    const path_x = Math.trunc(path_fx);
    const path_y = Math.trunc(path_fy);
    const width = Math.max(Math.trunc(path_fw), 1);
    const sidePad = 10;
    const height = Math.max(Math.trunc(width * 0.8), 12);
    // capnp default is -1; Number(null)=0 would falsely trigger radar-detected, so guard null
    const radarTrackId = (lead.radarTrackId != null) ? finiteNumber(lead.radarTrackId, -1) : -1;
    const radarDetected = radarTrackId >= 0;

    return {
      rect: {
        x: path_x - width * 0.5 - sidePad,
        y: path_y - height,
        width: width + sidePad * 2,
        height,
      },
      centerX: path_x,
      centerY: path_y,
      radar: Boolean(lead.radar),
      radarDetected,
      radarTrackId,
      dRel: distance,
      modelProb: finiteNumber(lead.modelProb, 0),
      width,
    };
  }

  function projectLeadTwoBox(lead, modelPath, calibTransform, videoWidth, videoHeight, stageWidth = videoWidth, stageHeight = videoHeight, transform = null) {
    if (!lead?.status || !lead?.radar) return null;
    const distance = finiteNumber(lead.dRel, NaN);
    if (!Number.isFinite(distance) || distance <= 0) return null;

    const yCenter = -finiteNumber(lead.yRel, 0);
    const z = samplePathZ(modelPath, distance) + PATH_Z_OFFSET;
    const left = projectPointPrecise(calibTransform, distance, yCenter - 1.2, z);
    const right = projectPointPrecise(calibTransform, distance, yCenter + 1.2, z);
    if (!left || !right) return null;

    const prev = leadTwoEmaState;
    const hasPrev = Number.isFinite(prev.xl) && Number.isFinite(prev.xr) && Number.isFinite(prev.y);
    // C3 lead_two uses alpha=0.8 at 20Hz; apply same time-compensation
    const nowMs2 = performance.now();
    const dt2 = prev.lastMs > 0 ? clamp(nowMs2 - prev.lastMs, 1, 500) : C3_FRAME_MS;
    const a2 = Math.pow(0.8, dt2 / C3_FRAME_MS);
    const xl = hasPrev ? (prev.xl * a2 + left.x * (1 - a2)) : left.x;
    const xr = hasPrev ? (prev.xr * a2 + right.x * (1 - a2)) : right.x;
    const y = hasPrev ? (prev.y * a2 + left.y * (1 - a2)) : left.y;
    leadTwoEmaState = { xl, xr, y, lastMs: nowMs2 };

    const { marginX, topMargin, maxCenterY } = getLeadBoxClampMargins(
      videoWidth,
      videoHeight,
      stageWidth,
      stageHeight,
      transform,
      { includeDistanceBadge: false, includeStateText: false },
    );
    const clampedCenterX = clamp((xl + xr) * 0.5, marginX, Math.max(marginX, videoWidth - marginX));
    const rawWidth = Math.max(xr - xl, 1);
    const width = Math.max(Math.trunc(clamp(rawWidth, 120, 800)), 1);
    const yInt = Math.trunc(clamp(y, topMargin, maxCenterY));
    const xlInt = Math.trunc(clampedCenterX - width * 0.5);
    const height = Math.max(Math.trunc(width * 0.8), 1);
    return {
      rect: {
        x: xlInt - 10,
        y: yInt - height,
        width: width + 20,
        height,
      },
      dRel: distance,
    };
  }

  function getPrimaryVisionDistance(model) {
    const lead = Array.isArray(model?.leadsV3) ? model.leadsV3[0] : null;
    const prob = finiteNumber(lead?.prob, 0);
    const distance = finiteNumber(lead?.x?.[0], 0);
    return prob > 0.5 && distance > 0 ? Math.max(0, distance - 1.52) : 0;
  }

  function drawLeadBoxCard(box, strokeColor, fillColor, primary = true) {
    if (!box?.rect) return;
    const { x, y, width, height } = box.rect;
    const r = primary ? 15 : 12;
    const sw = primary ? 3.0 : 2.2;
    // C3 style: fill + stroke (carrot.cc ui_fill_rect with stroke color)
    fillRoundedRect(ctx, x, y, width, height, r, fillColor);
    strokeRoundedRect(ctx, x, y, width, height, r, strokeColor, sw);
  }

  function eraseLeadBoxOcclusion(box, primary = true) {
    if (!box?.rect) return;
    const { x, y, width, height } = box.rect;
    ctx.save();
    ctx.globalCompositeOperation = "destination-out";
    fillRoundedRect(ctx, x, y, width, height, primary ? 15 : 12, "rgba(0,0,0,1)");
    ctx.restore();
  }

  // C3-style dual distance badges: radar left (red/orange) + vision right (blue)
  function drawLeadDistanceBadgesC3(box, radarDist, visionDist, isLeadScc, textColor = "#ffffff", videoWidth = BASE_CAMERA.width, videoHeight = BASE_CAMERA.height, stageWidth = videoWidth, stageHeight = videoHeight, transform = null) {
    if (!box?.rect) return;
    const cx = box.centerX;
    const offsets = getLeadBadgeOffsets(videoWidth, videoHeight);
    const badgeH = offsets.badgeHeight;
    const fontSize = offsets.fontSize;
    const visibleRect = getVisibleSourceRect(videoWidth, videoHeight, stageWidth, stageHeight, transform);
    const baseY = Math.min(box.centerY + offsets.rectTopOffset, Math.max(visibleRect.top + 4, visibleRect.bottom - badgeH - 4));

    const drawBadge = (text, centerX, bgColor) => {
      const charW = fontSize * 0.62;
      const w = Math.max(52, 16 + text.length * charW);
      const bx = centerX - w * 0.5;
      fillRoundedRect(ctx, bx, baseY, w, badgeH, 12, bgColor);
      ctx.save();
      ctx.font = `800 ${fontSize}px ${HUD_TEXT_FONT}`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.lineJoin = "round";
      ctx.strokeStyle = "rgba(0,0,0,0.82)";
      ctx.fillStyle = textColor;
      ctx.lineWidth = 4;
      ctx.strokeText(text, centerX, baseY + badgeH * 0.54);
      ctx.fillText(text, centerX, baseY + badgeH * 0.54);
      ctx.restore();
    };

    const hasRadar = radarDist > 0;
    const hasVision = visionDist > 0;
    // C3 exact colors: COLOR_RED(255,0,0), COLOR_ORANGE(255,175,3), COLOR_BLUE(0,0,255)
    const radarColor = isLeadScc ? "rgba(255,0,0,0.92)" : "rgba(255,175,3,0.92)";
    const visionColor = "rgba(0,0,255,0.92)";
    const radarCenterX = cx - offsets.dx;
    const visionCenterX = cx + offsets.dx;

    if (hasRadar && hasVision) {
      const rText = radarDist < 10 ? radarDist.toFixed(1) : radarDist.toFixed(0);
      const vText = visionDist < 10 ? visionDist.toFixed(1) : visionDist.toFixed(0);
      drawBadge(rText, radarCenterX, radarColor);
      drawBadge(vText, visionCenterX, visionColor);
    } else if (hasRadar) {
      const rText = radarDist < 10 ? radarDist.toFixed(1) : radarDist.toFixed(0);
      drawBadge(rText, radarCenterX, radarColor);
    } else if (hasVision) {
      const vText = visionDist < 10 ? visionDist.toFixed(1) : visionDist.toFixed(0);
      drawBadge(vText, visionCenterX, visionColor);
    }
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
    const longPlan = hudState?.longitudinalPlan || {};
    const carrotMan = overlayState?.carrotMan || hudState?.carrotMan || {};
    const carState = hudState?.carState || {};
    const xState = finiteNumber(longPlan?.xState, finiteNumber(carrotMan?.xState, -1));
    const trafficState = finiteNumber(longPlan?.trafficState, finiteNumber(carrotMan?.trafficState, -1));
    const longActive = Boolean(hudState?.controlsState?.enabled);
    const vEgoMps = finiteNumber(carState?.vEgo, finiteNumber(carState?.vEgoCluster, 0));
    const brakeHoldActive = Boolean(carState?.brakeHoldActive);
    const softHoldActive = finiteNumber(carState?.softHoldActive, 0) > 0;
    const carrotCruise = finiteNumber(carState?.carrotCruise, 0) > 0;

    if (brakeHoldActive || softHoldActive || carrotCruise) {
      return {
        text: brakeHoldActive ? "AUTOHOLD" : (softHoldActive ? "SOFTHOLD" : "CARROT"),
        xState,
        showDistanceBadge: false,
      };
    }

    if (!longActive) {
      return {
        text: "",
        xState,
        showDistanceBadge: true,
      };
    }
    if (xState === 3 || xState === 5) {
      return {
        text: vEgoMps < 1.0 ? (trafficState >= 1000 ? "Signal Error" : "Signal Ready") : "Signal slowing",
        xState,
        showDistanceBadge: false,
      };
    }
    if (xState === 4) {
      return {
        text: getUIText("e2e_driving", "E2E driving"),
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
    return {
      text: "",
      xState,
      showDistanceBadge: false,
    };
  }

  function drawLeadStateBadge(box, text, _xState, videoWidth = BASE_CAMERA.width, videoHeight = BASE_CAMERA.height, stageWidth = videoWidth, stageHeight = videoHeight, transform = null) {
    if (!box?.rect || !text) return;
    const offsets = getLeadBadgeOffsets(videoWidth, videoHeight);
    const visibleRect = getVisibleSourceRect(videoWidth, videoHeight, stageWidth, stageHeight, transform);
    const textY = Math.min(box.centerY + offsets.textBaselineOffset, Math.max(visibleRect.top + 6, visibleRect.bottom - 6));
    drawCanvasOutlinedText(text, box.centerX, textY, {
      fontSize: Math.max(50 * (videoHeight / BASE_CAMERA.height), 24),
      fontWeight: 900,
      fillStyle: "#ffffff",
      strokeStyle: "rgba(0,0,0,0.88)",
      strokeWidth: Math.max(4.0 * (videoHeight / BASE_CAMERA.height), 3.4),
      align: "center",
      baseline: "bottom",
    });
  }

  function drawCanvasOutlinedText(text, x, y, {
    fontSize = 18,
    fontWeight = 800,
    fillStyle = "#ffffff",
    strokeStyle = "rgba(0,0,0,0.86)",
    strokeWidth = 3.4,
    align = "center",
    baseline = "middle",
    canvasCtx = ctx,
  } = {}) {
    if (!text) return;
    canvasCtx.save();
    canvasCtx.font = `${fontWeight} ${fontSize}px ${HUD_TEXT_FONT}`;
    canvasCtx.textAlign = align;
    canvasCtx.textBaseline = baseline;
    canvasCtx.lineJoin = "round";
    canvasCtx.strokeStyle = strokeStyle;
    canvasCtx.fillStyle = fillStyle;
    canvasCtx.lineWidth = strokeWidth;
    canvasCtx.strokeText(text, x, y);
    canvasCtx.fillText(text, x, y);
    canvasCtx.restore();
  }

  function clampTextAnchor(point, text, fontSize, videoWidth, videoHeight) {
    const anchor = { x: finiteNumber(point?.x, 0), y: finiteNumber(point?.y, 0) };
    const font = `800 ${fontSize}px ${HUD_TEXT_FONT}`;
    const textWidth = Math.max(getCachedTextWidth(ctx, font, String(text || "")), 1);
    const padding = 8;
    anchor.x = clamp(anchor.x, padding, Math.max(padding, videoWidth - textWidth - padding));
    anchor.y = clamp(anchor.y, fontSize + padding, Math.max(fontSize + padding, videoHeight - padding));
    return anchor;
  }

  function drawRadarSpeedBadge(center, text, accentColor) {
    if (!center || !text) return;
    const badgeWidth = Math.max(40, 35 * String(text).length);
    const badgeHeight = 42;
    const badgeX = center.x - badgeWidth * 0.5;
    const badgeY = center.y - 35;
    fillRoundedRect(ctx, badgeX, badgeY, badgeWidth, badgeHeight, 15, accentColor);
    drawCanvasOutlinedText(String(text), center.x, center.y, {
      fontSize: 40,
      fontWeight: 900,
      strokeWidth: 4.2,
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

  function getPathStatusText(hudState) {
    const carState = hudState?.carState || {};
    if (Boolean(carState?.brakeHoldActive)) return "AUTOHOLD";
    if (finiteNumber(carState?.softHoldActive, 0) > 0) return "SOFTHOLD";
    if (finiteNumber(carState?.carrotCruise, 0) > 0) return "CARROT";
    return "";
  }

  function projectPathEndAnchorBox(modelPath, calibTransform, videoWidth, videoHeight) {
    const xs = Array.isArray(modelPath?.x) ? modelPath.x : [];
    const ys = Array.isArray(modelPath?.y) ? modelPath.y : [];
    const zs = Array.isArray(modelPath?.z) ? modelPath.z : [];
    if (!xs.length || !ys.length || !zs.length) return null;

    const tailDistance = clamp(finiteNumber(xs[xs.length - 1], 0), MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);
    const idx = getPathLengthIdx(modelPath, tailDistance);
    const distance = finiteNumber(xs[idx], NaN);
    const centerLineY = finiteNumber(ys[idx], NaN);
    const centerLineZ = finiteNumber(zs[idx], NaN);
    if (!Number.isFinite(distance) || !Number.isFinite(centerLineY) || !Number.isFinite(centerLineZ)) return null;

    const left = projectPointPrecise(calibTransform, distance, centerLineY - 1.2, centerLineZ + PATH_Z_OFFSET);
    const right = projectPointPrecise(calibTransform, distance, centerLineY + 1.2, centerLineZ + PATH_Z_OFFSET);
    if (!left || !right) return null;

    const rawWidth = Math.abs(right.x - left.x);
    if (!Number.isFinite(rawWidth) || rawWidth <= 1) return null;

    const rawCenterX = (left.x + right.x) * 0.5;
    const rawCenterY = (left.y + right.y) * 0.5;
    const { marginX, topMargin, bottomMargin } = getLeadBoxClampMargins(videoWidth, videoHeight);
    const width = clamp(rawWidth, 120, 800);
    const centerX = clamp(rawCenterX, marginX, Math.max(marginX, videoWidth - marginX));
    const centerY = clamp(rawCenterY, topMargin, Math.max(topMargin, videoHeight - bottomMargin));
    return {
      centerX: Math.trunc(centerX),
      centerY: Math.trunc(centerY),
      width: Math.trunc(width),
    };
  }

  function drawPathStatusText(modelPath, hudState, calibTransform, videoWidth, videoHeight, anchorBox = null) {
    const text = getPathStatusText(hudState);
    if (!text) return;

    const anchor = anchorBox || projectPathEndAnchorBox(modelPath, calibTransform, videoWidth, videoHeight);
    if (!anchor) return;

    const scale = videoHeight / BASE_CAMERA.height;
    const fontSize = Math.max(50 * scale, 24);
    const baselineY = clamp(anchor.centerY + 60 * scale, fontSize + 8, videoHeight - 10);
    drawCanvasOutlinedText(text, anchor.centerX, baselineY, {
      fontSize,
      fontWeight: 900,
      fillStyle: "#ffffff",
      strokeStyle: "rgba(0,0,0,0.88)",
      strokeWidth: Math.max(4.0 * scale, 3.4),
      align: "center",
      baseline: "bottom",
    });
  }

  function getRadarTracks(radarState) {
    const source = radarState && typeof radarState === "object" ? radarState : {};
    const tracks = [
      ...(Array.isArray(source.leadsLeft) ? source.leadsLeft : []),
      ...(Array.isArray(source.leadsRight) ? source.leadsRight : []),
      ...(Array.isArray(source.leadsCenter) ? source.leadsCenter : []),
    ];
    if (tracks.length) return tracks;

    const fallback = [];
    if (source.leadOne?.status) fallback.push(source.leadOne);
    if (source.leadTwo?.status) fallback.push(source.leadTwo);
    return fallback;
  }

  function getRadarProjectionLine(model) {
    const laneLines = Array.isArray(model?.laneLines) ? model.laneLines : [];
    const centerLane = laneLines[2];
    if (Array.isArray(centerLane?.x) && Array.isArray(centerLane?.z) && centerLane.x.length && centerLane.z.length) {
      return centerLane;
    }
    return model?.position || null;
  }

  function drawRadarTargets(radarState, model, calibTransform) {
    const showRadarInfo = finiteNumber(paramsState.ShowRadarInfo, 0);
    if (showRadarInfo <= 0) return;
    const projectionLine = getRadarProjectionLine(model);
    if (!projectionLine) return;
    const radarLatFactor = finiteNumber(paramsState.RadarLatFactor, 0) / 100.0;
    const isMetric = finiteNumber(paramsState.IsMetric, 1) !== 0;

    for (const radar of getRadarTracks(radarState)) {
      const dRel = finiteNumber(radar?.dRel, 0);
      if (!Number.isFinite(dRel) || dRel <= 2.5) continue;

      const yRel = finiteNumber(radar?.yRel, 0);
      const z = samplePathZ(projectionLine, dRel) - 0.61;
      const center = projectPointPrecise(calibTransform, dRel, -yRel, z);
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
          ? projectPointPrecise(calibTransform, futureDRel, -futureYRel, z)
          : null;
        if (future) {
          const vectorColor = vSigned >= 0 ? "rgba(35,213,93,0.94)" : "rgba(255,59,48,0.94)";
          drawPolyline([center, future], vectorColor, 3.0);
          drawPolygon(circlePolygon(future.x, future.y, 10, 12), vectorColor);
        }

        let badgeColor = "rgba(255,59,48,0.96)";
        if (!radarDetected) badgeColor = "rgba(61,123,255,0.96)";
        else if (Math.abs(modelProb - 0.01) < 1e-3) badgeColor = "rgba(35,213,93,0.96)";
        else if (vSigned > 0) badgeColor = "rgba(255,167,38,0.96)";

        const speedValue = vSigned * (isMetric ? 3.6 : 2.2369363);
        drawRadarSpeedBadge({ x: center.x, y: center.y }, speedValue.toFixed(0), badgeColor);

        if (showRadarInfo >= 2) {
          drawCanvasOutlinedText(finiteNumber(radar?.yRel, 0).toFixed(1), center.x, center.y - 40, {
            fontSize: 30,
            fontWeight: 900,
            strokeWidth: 3.8,
          });
          const distanceValue = isMetric ? dRel : dRel * 0.621371;
          drawCanvasOutlinedText(distanceValue.toFixed(1), center.x, center.y + 30, {
            fontSize: 30,
            fontWeight: 900,
            strokeWidth: 3.8,
          });
        }
      } else if (showRadarInfo >= 3) {
        drawCanvasOutlinedText("*", center.x, center.y, {
          fontSize: 40,
          fontWeight: 900,
          strokeWidth: 4.2,
        });
      }
    }
  }

  function drawRadarLeadBoxes(model, overlayState, hudState, calibTransform, videoWidth, videoHeight, stageWidth, stageHeight, transform) {
    const radarState = overlayState?.radarState || {};
    const modelPath = model?.position || null;
    const leadState = getLeadStateText(overlayState, hudState);
    const roadCameraState = overlayState?.roadCameraState || null;
    const longPlan = hudState?.longitudinalPlan || {};
    const leadTwoStatus = finiteNumber(longPlan?.longitudinalPlanSource, 0) === LONG_PLAN_SOURCE_LEAD1 ? 2 : 1;

    syncLeadRenderState(videoWidth, videoHeight, model?.frameId, roadCameraState?.frameId);

    const leadOneBox = projectLeadBox(
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
    const nowMs = performance.now();

    let primaryStatusAnchorBox = null;
    if (leadOneBox) {
      const isLeadScc = leadOneBox.radarTrackId < 1;
      // C3 exact colors: COLOR_RED(255,0,0), COLOR_ORANGE(255,175,3), COLOR_BLUE(0,0,255)
      const strokeColor = !leadOneBox.radarDetected
        ? "rgba(0,0,255,0.96)"
        : (isLeadScc ? "rgba(255,0,0,0.96)" : "rgba(255,175,3,0.96)");
      eraseLeadBoxOcclusion(leadOneBox, true);
      drawLeadBoxCard(leadOneBox, strokeColor, "rgba(0,0,0,0.20)", true);

      // Update hold state for persistence across brief status flickers
      leadHoldState.lastValidMs = nowMs;
      leadHoldState.box = leadOneBox;
      leadHoldState.strokeColor = strokeColor;
      leadHoldState.isLeadScc = isLeadScc;

      // C3-style distance badges: radar (red/orange bg) + vision (blue bg) side by side below box
      if (leadState?.showDistanceBadge !== false) {
        const radarDist = Boolean(radarState?.leadOne?.radar) ? Math.max(0, finiteNumber(radarState?.leadOne?.dRel, 0)) : 0;
        const visionDist = getPrimaryVisionDistance(model);
        leadHoldState.radarDist = radarDist;
        leadHoldState.visionDist = visionDist;
        if (radarDist > 0 || visionDist > 0) {
          const badgeTextColor = leadState?.xState === 0 ? "#ffffff" : (leadState?.xState === 1 ? "#b0b0b0" : "#23d55d");
          leadHoldState.badgeTextColor = badgeTextColor;
          drawLeadDistanceBadgesC3(leadOneBox, radarDist, visionDist, isLeadScc, badgeTextColor, videoWidth, videoHeight, stageWidth, stageHeight, transform);
        }
      }

      if (leadState?.text) {
        drawLeadStateBadge(leadOneBox, leadState.text, leadState.xState, videoWidth, videoHeight, stageWidth, stageHeight, transform);
      }
      primaryStatusAnchorBox = leadOneBox;
    } else if (leadHoldState.box && (nowMs - leadHoldState.lastValidMs) < LEAD_HOLD_MS) {
      // Status flickered false briefly — hold last valid box (C3's SubMaster doesn't flicker)
      const held = leadHoldState;
      eraseLeadBoxOcclusion(held.box, true);
      drawLeadBoxCard(held.box, held.strokeColor, "rgba(0,0,0,0.20)", true);
      if (leadState?.showDistanceBadge !== false && (held.radarDist > 0 || held.visionDist > 0)) {
        drawLeadDistanceBadgesC3(held.box, held.radarDist, held.visionDist, held.isLeadScc, held.badgeTextColor, videoWidth, videoHeight, stageWidth, stageHeight, transform);
      }
      if (leadState?.text) {
        drawLeadStateBadge(held.box, leadState.text, leadState.xState, videoWidth, videoHeight, stageWidth, stageHeight, transform);
      }
      primaryStatusAnchorBox = held.box;
    } else {
      leadHoldState.box = null;
    }

    // LeadTwo: same logic as C3 (radar && dRel > leadOne.dRel + 3)
    const leadTwo = radarState?.leadTwo;
    const validLeadTwo = Boolean(leadTwo?.status) &&
      Boolean(leadTwo?.radar) &&
      finiteNumber(leadTwo?.dRel, 0) > (finiteNumber(radarState?.leadOne?.dRel, 0) + 3);
    if (validLeadTwo) {
      const leadTwoBox = projectLeadTwoBox(leadTwo, modelPath, calibTransform, videoWidth, videoHeight, stageWidth, stageHeight, transform);
      if (leadTwoBox) {
        eraseLeadBoxOcclusion(leadTwoBox, false);
        drawLeadBoxCard(
          leadTwoBox,
          "rgba(218,111,37,0.96)",  // C3 COLOR_OCHRE(218,111,37)
          leadTwoStatus === 2 ? "rgba(255,0,0,0.20)" : "rgba(0,0,0,0.20)",  // C3 COLOR_RED_ALPHA(50)
          false,
        );
      }
    } else {
      resetLeadEmaSlot(1);
      resetLeadTwoEma();
    }
    drawPathStatusText(modelPath, hudState, calibTransform, videoWidth, videoHeight, primaryStatusAnchorBox);
    drawRadarTargets(radarState, model, calibTransform);
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
    context.fillStyle = fillStyle;
    const cachedPath = getCachedRoundedRectPath(width, height, radius);
    if (cachedPath) {
      context.save();
      context.translate(x, y);
      context.fill(cachedPath);
      context.restore();
      return;
    }
    roundedRectPath(context, x, y, width, height, radius);
    context.fill();
  }

  function strokeRoundedRect(context, x, y, width, height, radius, strokeStyle, lineWidth = 1) {
    context.strokeStyle = strokeStyle;
    context.lineWidth = lineWidth;
    const cachedPath = getCachedRoundedRectPath(width, height, radius);
    if (cachedPath) {
      context.save();
      context.translate(x, y);
      context.stroke(cachedPath);
      context.restore();
      return;
    }
    roundedRectPath(context, x, y, width, height, radius);
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

  function formatRtcPerfLabel() {
    const perf = window.CarrotRtcPerf || null;
    if (!perf?.active) return "";
    if (perf?.connectionState === "connecting" || perf?.iceConnectionState === "checking") return "RECONN";
    if (perf?.error) return "";

    const network = perf.network || {};
    const inbound = perf.inbound || {};
    const video = perf.video || {};
    const resolutionLabel = String(network.resolutionLabel || "").trim();
    const bitrateMbps = Number.isFinite(Number(network.bitrateMbps)) ? Number(network.bitrateMbps) : null;
    const fps = Number.isFinite(Number(inbound.framesPerSecond)) ? Number(inbound.framesPerSecond) : null;
    const rttMs = Number.isFinite(Number(network.rttMs)) ? Number(network.rttMs) : null;
    const hasFreeze = Number.isFinite(Number(inbound.freezeCount)) && Number(inbound.freezeCount) > 0 &&
      Number.isFinite(Number(video.readyState)) && Number(video.readyState) < 3;

    if (!resolutionLabel && bitrateMbps == null && fps == null && rttMs == null) {
      return hasFreeze ? "STALL" : "";
    }

    const bitrateLabel = bitrateMbps != null
      ? `${bitrateMbps >= 10 ? bitrateMbps.toFixed(0) : bitrateMbps.toFixed(1)}m`
      : "-m";
    const fpsLabel = fps != null ? `${Math.round(fps)}fps` : "-fps";
    const rttLabel = rttMs != null ? `${Math.round(rttMs)}ms` : "-ms";
    return [resolutionLabel || "-p", bitrateLabel, fpsLabel, rttLabel].join(" ");
  }

  function drawHudRightCenterPerfText(stageWidth, stageHeight, viewportRect, pathMode) {
    const label = shortText(formatRtcPerfLabel(), 48);
    if (!label) return;

    const exactC3Mode = stageWidth >= 1280 && stageHeight >= 720;
    const baseScale = Math.min(stageWidth / 1920, stageHeight / 1080);
    const scale = clamp(baseScale, 0.48, 1.0);
    const edgeInsetX = exactC3Mode ? 1.5 : clamp(2.0 * scale, 1.0, 2.5);
    const edgeInsetTop = exactC3Mode ? 28.0 : clamp(30.0 * scale, 12.0, 30.0);
    const maxWidth = Math.max(180.0, viewportRect.width * 0.42);
    const fontSize = fitSingleLineHudFontSize(
      label,
      exactC3Mode ? 18.0 : clamp(18.0 * scale, 8.0, 18.0),
      maxWidth,
      6.0,
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

  function drawStageEdgeFades(stageWidth, stageHeight) {
    const topHeight = clamp(stageHeight * 0.16, 72, 148);
    const bottomHeight = clamp(stageHeight * 0.24, 104, 212);

    hudCtx.save();
    hudCtx.globalCompositeOperation = "source-over";

    const topGradientKey = `hud-top:${Math.round(stageWidth)}x${Math.round(stageHeight)}:${Math.round(topHeight)}`;
    const topGradient = getCachedHudGradient(topGradientKey, () => {
      const gradient = hudCtx.createLinearGradient(0, 0, 0, topHeight);
      gradient.addColorStop(0.0, "rgba(0, 0, 0, 0.64)");
      gradient.addColorStop(0.42, "rgba(0, 0, 0, 0.30)");
      gradient.addColorStop(1.0, "rgba(0, 0, 0, 0.00)");
      return gradient;
    });
    hudCtx.fillStyle = topGradient;
    hudCtx.fillRect(0, 0, stageWidth, topHeight);

    const bottomStart = Math.max(0, stageHeight - bottomHeight);
    const bottomGradientKey = `hud-bottom:${Math.round(stageWidth)}x${Math.round(stageHeight)}:${Math.round(bottomStart)}:${Math.round(bottomHeight)}`;
    const bottomGradient = getCachedHudGradient(bottomGradientKey, () => {
      const gradient = hudCtx.createLinearGradient(0, bottomStart, 0, stageHeight);
      gradient.addColorStop(0.0, "rgba(0, 0, 0, 0.00)");
      gradient.addColorStop(0.42, "rgba(0, 0, 0, 0.28)");
      gradient.addColorStop(1.0, "rgba(0, 0, 0, 0.74)");
      return gradient;
    });
    hudCtx.fillStyle = bottomGradient;
    hudCtx.fillRect(0, bottomStart, stageWidth, bottomHeight);

    hudCtx.restore();
  }

  function drawHudTopLeftText(stageWidth, stageHeight, viewportRect, text, pathMode) {
    if (window.matchMedia("(orientation: portrait)").matches) return;
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
      x: viewportRect.left + edgeInsetX,
      y: viewportRect.top + edgeInsetTop,
      color: `rgba(244, 244, 244, ${alpha.toFixed(3)})`,
      strokeColor: `rgba(0, 0, 0, ${clamp(alpha + 0.08, 0.0, 1.0).toFixed(3)})`,
      strokeWidth: clamp(4.2 * scale, 2.8, 5.4),
      fontSize,
      fontWeight: 900,
      alignX: "left",
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

  function drawHudBottomLeftText(stageWidth, stageHeight, viewportRect, text, pathMode) {
    const label = shortText(text, 160);
    if (!label) return;
    const exactC3Mode = stageWidth >= 1280 && stageHeight >= 720;
    const baseScale = Math.min(stageWidth / 1920, stageHeight / 1080);
    const scale = clamp(baseScale, 0.48, 1.0);
    const edgeInsetX = exactC3Mode ? 1.5 : clamp(2.0 * scale, 1.0, 2.5);
    const bottomInset = exactC3Mode ? 1.5 : clamp(2.0 * scale, 1.0, 2.5);
    const maxWidth = Math.max(120.0, viewportRect.width * 0.42);
    const fontSize = fitSingleLineHudFontSize(
      label,
      exactC3Mode ? 24.0 : clamp(24.0 * scale, 7.0, 24.0),
      maxWidth,
      4.5,
      900,
    );
    const alpha = getHudLabelAlpha(pathMode, Math.PI);

    drawOutlinedHudText({
      text: label,
      x: viewportRect.left + edgeInsetX,
      y: viewportRect.bottom - bottomInset,
      color: `rgba(236, 236, 236, ${alpha.toFixed(3)})`,
      strokeColor: `rgba(0, 0, 0, ${clamp(alpha + 0.08, 0.0, 1.0).toFixed(3)})`,
      strokeWidth: clamp(4.0 * scale, 2.8, 5.2),
      fontSize,
      fontWeight: 900,
      alignX: "left",
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
          title: "2.Speed/Accel(Y:speed_0, G:v_ego, O:a_ego)",
          values: [speedTarget, vEgo, aEgo],
        };
      case 3:
        return {
          mode,
          title: "3.Model(Y:pos_32, G:vel_32, O:vel_0)",
          values: [modelPos32, modelVel32, modelVel0],
        };
      case 4:
        return {
          mode,
          title: "4.Lead(Y:accel, G:a_lead, O:v_rel)",
          values: [accelTarget, finiteNumber(radarLead?.aLeadK, 0), finiteNumber(radarLead?.vRel, 0)],
        };
      case 5:
        return {
          mode,
          title: "5.Lead(Y:a_ego, G:a_lead, O:j_lead)",
          values: [aEgo, finiteNumber(radarLead?.aLead, 0), finiteNumber(radarLead?.jLead, 0)],
        };
      case 6:
        return {
          mode,
          title: "6.Steer(Y:actual, G:desire, O:output)",
          values: [
            finiteNumber(controlsState?.actualLateralAccel, 0) * 10.0,
            finiteNumber(controlsState?.desiredLateralAccel, 0) * 10.0,
            finiteNumber(controlsState?.lateralOutput, 0) * 10.0,
          ],
        };
      case 7:
        return {
          mode,
          title: "7.SteerA(Y:Actual, G:Target, O:Offset*10)",
          values: [
            finiteNumber(carState?.steeringAngleDeg, 0),
            finiteNumber(actuators?.steeringAngleDeg, 0),
            finiteNumber(liveParameters?.angleOffsetDeg, 0) * 10.0,
          ],
        };
      case 8:
        return {
          mode,
          title: "8.SteerA(Y:Actual, G:Target, O:Offset*10)",
          values: [
            finiteNumber(actuators?.curvature, 0) * 10000,
            finiteNumber(actuators?.curvature, 0) * 10000,
            finiteNumber(actuators?.curvature, 0) * 10000,
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

  /* ── Phase 1-4: ring buffer for plot history ── */
  const _plotRing = [
    new Float64Array(PLOT_MAX_POINTS),
    new Float64Array(PLOT_MAX_POINTS),
    new Float64Array(PLOT_MAX_POINTS),
  ];
  let _plotRingHead = 0;
  let _plotRingSize = 0;

  function _plotRingPush(values) {
    const writeIdx = (_plotRingHead + _plotRingSize) % PLOT_MAX_POINTS;
    for (let i = 0; i < 3; i += 1) {
      _plotRing[i][writeIdx] = finiteNumber(values[i], 0);
    }
    if (_plotRingSize < PLOT_MAX_POINTS) _plotRingSize++;
    else _plotRingHead = (_plotRingHead + 1) % PLOT_MAX_POINTS;
  }

  function _plotRingGet(series, idx) {
    return _plotRing[series][(_plotRingHead + idx) % PLOT_MAX_POINTS];
  }

  function _plotRingReset() {
    _plotRingHead = 0;
    _plotRingSize = 0;
  }

  function updatePlotHistory(plotData) {
    if (!plotData) {
      lastPlotMode = -1;
      _plotRingReset();
      return;
    }

    if (plotData.mode !== lastPlotMode) {
      lastPlotMode = plotData.mode;
      _plotRingReset();
    }

    _plotRingPush(plotData.values);
  }

  function getPlotBounds() {
    let min = Number.POSITIVE_INFINITY;
    let max = Number.NEGATIVE_INFINITY;
    for (let seriesIndex = 0; seriesIndex < 3; seriesIndex += 1) {
      for (let i = 0; i < _plotRingSize; i += 1) {
        const value = _plotRingGet(seriesIndex, i);
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
    if (stageHeight > stageWidth) return;
    const viewportWidth = finiteNumber(viewportRect?.width, stageWidth);
    const viewportHeight = finiteNumber(viewportRect?.height, stageHeight);

    const plotScale = Math.min(
      viewportWidth / BASE_CAMERA.width,
      viewportHeight / BASE_CAMERA.height,
    );
    const plotX = finiteNumber(viewportRect?.left, 0) + 22.0 * plotScale;
    const plotY = finiteNumber(viewportRect?.top, 0) + 40.0 * plotScale;
    const plotWidth = 1000.0 * plotScale;
    const plotHeight = 300.0 * plotScale;
    const plotDx = 2.0 * plotScale;  // scale with plot area to match C3 density
    const size = Math.min(_plotRingSize, PLOT_MAX_POINTS);
    if (size < 2) return;
    const bounds = getPlotBounds();
    const range = Math.max(bounds.max - bounds.min, 1);
    const visibleSize = Math.min(size, Math.max(2, Math.floor(plotWidth / Math.max(plotDx, 0.001))));
    const latestPlotX = plotX + plotWidth;
    const latestLabelX = latestPlotX + 50.0 * plotScale;
    const titleX = plotX + 8.0 * plotScale;
    const titleY = plotY + plotHeight + 18.0 * plotScale;

    hudCtx.save();

    for (let seriesIndex = 0; seriesIndex < 3; seriesIndex += 1) {
      hudCtx.beginPath();
      let latestPoint = null;
      for (let i = 0; i < visibleSize; i += 1) {
        const currentValue = _plotRingGet(seriesIndex, size - 1 - i);
        const x = latestPlotX - i * plotDx;
        const y = plotY + plotHeight - ((currentValue - bounds.min) / range) * plotHeight;
        if (i === 0) {
          hudCtx.moveTo(x, y);
          latestPoint = { x, y, value: currentValue };
        } else {
          hudCtx.lineTo(x, y);
        }
      }
      hudCtx.lineWidth = Math.max(1.6, 3.0 * plotScale);
      hudCtx.strokeStyle = PLOT_SERIES[seriesIndex].color;
      hudCtx.stroke();

      if (latestPoint) {
        const labelY = latestPoint.y + (seriesIndex > 0 ? 40.0 * plotScale : 0);
        drawCanvasOutlinedText(latestPoint.value.toFixed(2), latestLabelX, labelY, {
          fontSize: clamp(Math.round(34.0 * plotScale), 15, 34),
          fontWeight: 900,
          strokeWidth: Math.max(1.3, 3.2 * plotScale),
          fillStyle: PLOT_SERIES[seriesIndex].color,
          align: "left",
          canvasCtx: hudCtx,
        });
      }
    }

    drawCanvasOutlinedText(plotData.title, titleX, titleY, {
      fontSize: clamp(Math.round(19.0 * plotScale), 12, 19),
      fontWeight: 800,
      strokeWidth: Math.max(1.4, 3.6 * plotScale),
      align: "left",
      canvasCtx: hudCtx,
    });

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

  function renderActiveFrame(options = {}) {
    refreshParams();
    const stageWidth = Math.max(1, stageEl.clientWidth);
    const stageHeight = Math.max(1, stageEl.clientHeight);
    const forceAll = Boolean(options.force || _forceNextRender);
    const rawOverlayState = window.CarrotOverlayState || {};
    const rawHudState = window.CarrotHudState || {};
    const runtimeState = mergeRuntimeState(rawHudState, rawOverlayState);
    let overlayState = runtimeState.overlayState;
    const hudState = runtimeState.hudState;
    const brokerServices = runtimeState.brokerServices;

    renderOnroadAlert(stageWidth, stageHeight, hudState?.selfdriveState);

    if (!window.CARROT_VISION_ACTIVE) {
      if (forceAll || _lastOverlaySig !== "vision-disabled" || _lastHudSig !== "vision-disabled") {
        _lastOverlaySig = "vision-disabled";
        _lastHudSig = "vision-disabled";
        _lastPlotInputSig = "off";
        setStageLoading(false);
        setStageReady(false);
        clearOverlay(canvasEl.width || 1, canvasEl.height || 1);
        clearHud(hudCanvasEl.width || 1, hudCanvasEl.height || 1);
        setStatus(window.CARROT_VISION_AVAILABLE === false
          ? (window.CARROT_VISION_DISABLED_MESSAGE || getUIText("disable_dm_inactive", "DisableDM is inactive."))
          : getUIText("start_vision_hint", "Tap the start button to enable drive vision."));
        setMeta("");
        setDebug("");
      }
      return;
    }

    const hasStream = syncSourceStream();
    if (!hasStream || !videoEl.videoWidth || !videoEl.videoHeight) {
      _lastOverlaySig = "";
      _lastHudSig = "";
      setStageLoading(true, getUIText("connecting", "Connecting..."));
      setStageReady(false);
      clearOverlay(canvasEl.width || 1, canvasEl.height || 1);
      clearHud(hudCanvasEl.width || 1, hudCanvasEl.height || 1);
      setStatus(getUIText("waiting_road_stream", "Waiting road camera stream..."));
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
    // Use raw radar state directly — no interpolation (matches C3/CarrotLink).
    // C3 reads SubMaster every frame; CarrotLink reads snapshot directly.
    // Position smoothing is handled in projectLeadBox() via EMA.
    const model = overlayState.modelV2 || null;
    const liveCalibration = overlayState.liveCalibration || null;
    const roadCameraState = overlayState.roadCameraState || null;
    const selectedPath = getSelectedPath(overlayState, hudState);
    const pathStyle = getPathStyle(overlayState, hudState);
    const plotData = buildPlotData(overlayState, hudState);
    const showLaneInfo = finiteNumber(paramsState.ShowLaneInfo, defaultParams.ShowLaneInfo);
    const debugText = firstNonEmptyText(
      brokerServices?.carrotMan?.stockDebugTopRightText,
      overlayState?.carrotMan?.stockDebugTopRightText,
      formatDebugText(overlayState),
    );
    const overlaySig = overlayDataSignature(hudState, overlayState, selectedPath, pathStyle, showLaneInfo);
    // C3 pushes plot data EVERY frame unconditionally (drawPlot.draw→updatePlotQueue).
    // Web must do the same for identical density — no signature gating.
    updatePlotHistory(plotData);
    const nextPlotSig = plotInputSignature(plotData);
    const plotChanged = forceAll || Boolean(options.hudDirty) || nextPlotSig !== _lastPlotInputSig;
    if (plotChanged) {
      _lastPlotInputSig = nextPlotSig;
    }
    const hudSig = hudDataSignature(hudState, overlayState, plotData, selectedPath, debugText);
    if ((!overlayInfoState.lastSignature || !overlayInfoState.carLabel || !overlayInfoState.branchLabel) && !overlayInfoState.loading) {
      refreshOverlayInfo().catch(() => {});
    }
    const overlayDirty = forceAll || Boolean(options.overlayDirty) || overlaySig !== _lastOverlaySig;
    const hudDirty = forceAll || overlayDirty || Boolean(options.hudDirty) || plotChanged || hudSig !== _lastHudSig;
    if (!overlayDirty && !hudDirty) return;
    _lastOverlaySig = overlaySig;
    _lastHudSig = hudSig;
    _forceNextRender = false;

    const calibration = getCalibrationMatrix(liveCalibration);
    const transform = getStageTransform(videoWidth, videoHeight, stageWidth, stageHeight, calibration);
    const viewportRect = getHudViewportRect(videoWidth, videoHeight, stageWidth, stageHeight, transform);
    const laneCount = Array.isArray(model?.laneLines) ? model.laneLines.length : 0;
    const edgeCount = Array.isArray(model?.roadEdges) ? model.roadEdges.length : 0;
    const leadCount = Array.isArray(model?.leadsV3) ? model.leadsV3.length : 0;
    const rpyText = formatRpyTriplet(liveCalibration);
    const modeLabel = getDisplayModeLabel(transform.displayMode || DISPLAY_MODES[1]);
    const laneLabel = laneModeLabel(hudState);

    applyStageTransform(transform);
    setStageReady(true);
    applyCarrotHudLayout(viewportRect);
    setStageLoading(false);

    if (overlayDirty) {
      resetFrameProjectionCache();
      clearOverlay(videoWidth, videoHeight);
      if (model) {
        if (showLaneInfo >= 1) drawLaneLines(model, overlayState, hudState, transform.calibTransform);
        if (showLaneInfo > 1) drawRoadEdges(model, overlayState, transform.calibTransform);
        if (showLaneInfo >= 0) drawPath(selectedPath.pathData, model, overlayState, transform.calibTransform, videoHeight, pathStyle);
        drawBlindspotBarriers(model?.position, overlayState, hudState, transform.calibTransform);
        drawRadarLeadBoxes(model, overlayState, hudState, transform.calibTransform, videoWidth, videoHeight, stageWidth, stageHeight, transform);
        drawProjectedTfMarker(model?.position, hudState?.longitudinalPlan, transform.calibTransform, videoWidth, videoHeight);
      }
    }

    if (hudDirty) {
      clearHud(stageWidth, stageHeight);
      drawStageEdgeFades(stageWidth, stageHeight);
      drawPlot(stageWidth, stageHeight, viewportRect, plotData);
      setDebug(debugText);
      drawHudTopLeftText(stageWidth, stageHeight, viewportRect, overlayInfoState.carLabel, pathStyle.mode);
      drawHudTopRightText(stageWidth, stageHeight, viewportRect, lastDebug, pathStyle.mode);
      drawHudRightCenterPerfText(stageWidth, stageHeight, viewportRect, pathStyle.mode);
      drawHudBottomLeftText(stageWidth, stageHeight, viewportRect, overlayInfoState.branchLabel, pathStyle.mode);
      drawHudBottomText(stageWidth, stageHeight, viewportRect, selectedPath.latDebugText, hudState, pathStyle.mode);
    }

    if (!model) {
      setStatus(`road ${videoWidth}x${videoHeight} · ${getUIText("waiting_model", "waiting modelV2...")} · ${laneLabel}`);
    } else {
      setStatus(`road ${videoWidth}x${videoHeight} · model ${model.frameId ?? "-"} · ${laneLabel} · ${modeLabel}`);
    }

    setMeta(
      `road:${roadCameraState?.frameId ?? "-"} model:${model?.frameId ?? "-"} path:${selectedPath.pathSource}/${pathStyle.mode}:${pathStyle.colorIndex} width:${finiteNumber(paramsState.ShowPathWidth, 100)} laneInfo:${showLaneInfo} lane:${laneCount} edge:${edgeCount} lead:${leadCount} plot:${finiteNumber(paramsState.ShowPlotMode, 0)} rpy:${rpyText} h:${finiteNumber(liveCalibration?.height?.[0], 0).toFixed(2)}`,
    );
    if (!hudDirty) setDebug(debugText);
  }

  function cancelScheduledRender() {
    if (_renderRafId != null) {
      window.cancelAnimationFrame(_renderRafId);
      _renderRafId = null;
    }
    if (_renderTimerId != null) {
      window.clearTimeout(_renderTimerId);
      _renderTimerId = null;
    }
    if (_renderVideoFrameId != null) {
      try {
        if (typeof videoEl.cancelVideoFrameCallback === "function") {
          videoEl.cancelVideoFrameCallback(_renderVideoFrameId);
        }
      } catch {}
      _renderVideoFrameId = null;
    }
  }

  function mergePendingRenderState(options = {}) {
    const force = Boolean(options.force);
    const overlayDirty = force || options.overlayDirty === true;
    const hudDirty = force || options.hudDirty === true;
    _pendingRenderState.force = _pendingRenderState.force || force;
    _pendingRenderState.overlayDirty = _pendingRenderState.overlayDirty || overlayDirty;
    _pendingRenderState.hudDirty = _pendingRenderState.hudDirty || hudDirty;
  }

  function clearPendingRenderState() {
    _pendingRenderState.force = false;
    _pendingRenderState.overlayDirty = false;
    _pendingRenderState.hudDirty = false;
  }

  function isStageVisible() {
    return isActive() && !document.hidden;
  }

  function flushScheduledRender() {
    _renderRafId = null;
    _renderTimerId = null;
    _renderVideoFrameId = null;
    if (!isStageVisible()) {
      if (!isActive()) resetCarrotHudLayout();
      return;
    }

    const pending = { ..._pendingRenderState };
    clearPendingRenderState();
    _lastRenderTime = performance.now();
    renderActiveFrame(pending);

    if (_pendingRenderState.force || _pendingRenderState.overlayDirty || _pendingRenderState.hudDirty) {
      scheduleRender();
    }
  }

  function canUseVideoFrameScheduling() {
    return (
      _pendingRenderState.overlayDirty &&
      !_pendingRenderState.force &&
      typeof videoEl.requestVideoFrameCallback === "function" &&
      window.CARROT_VISION_ACTIVE &&
      !videoEl.paused &&
      videoEl.readyState >= 2
    );
  }

  function scheduleRender(options = {}) {
    mergePendingRenderState(options);
    if (!isStageVisible()) {
      cancelScheduledRender();
      if (!isActive()) resetCarrotHudLayout();
      return;
    }

    if (_renderRafId != null || _renderTimerId != null || _renderVideoFrameId != null) return;
    const elapsed = performance.now() - _lastRenderTime;
    const waitMs = Math.max(0, RENDER_INTERVAL_MS - elapsed);
    if (waitMs > 0) {
      _renderTimerId = window.setTimeout(() => {
        _renderTimerId = null;
        scheduleRender();
      }, waitMs);
      return;
    }
    if (canUseVideoFrameScheduling()) {
      _renderVideoFrameId = videoEl.requestVideoFrameCallback(() => {
        _renderVideoFrameId = null;
        flushScheduledRender();
      });
      return;
    }
    _renderRafId = window.requestAnimationFrame(flushScheduledRender);
  }

  function requestRender(options = {}) {
    const hasOverlayDirty = Object.prototype.hasOwnProperty.call(options, "overlayDirty");
    const hasHudDirty = Object.prototype.hasOwnProperty.call(options, "hudDirty");
    const normalized = {
      force: Boolean(options.force),
      overlayDirty: Boolean(options.force || (hasOverlayDirty ? options.overlayDirty : true)),
      hudDirty: Boolean(options.force || (hasHudDirty ? options.hudDirty : true)),
    };
    scheduleRender(normalized);
  }

  function refresh() {
    transformSignature = "";
    overlaySizeSignature = "";
    hudSizeSignature = "";
    _lastOverlaySig = "";
    _lastHudSig = "";
    _lastPlotInputSig = "";
    _forceNextRender = true;
    _gradientCache.clear();
    _hudGradientCache.clear();
    _rgbaCache.clear();
    _mergeRuntimeCache.refs = null;
    _mergeRuntimeCache.result = null;
  }

  if (displayModeButton) {
    displayModeButton.addEventListener("click", () => {
      const nextIndex = (displayModeIndex + 1) % DISPLAY_MODES.length;
      setDisplayModeIndex(nextIndex);
      requestRender({ force: true, overlayDirty: true, hudDirty: true });
    });
  }

  function shouldIgnoreStageFullscreenToggle(target) {
    if (!(target instanceof Element)) return false;
    if (target.closest("button, a, input, textarea, select, label")) return true;
    if (target.closest(".carrot-stage__controls")) return true;
    if (target.closest(".vision-start-overlay")) return true;
    return false;
  }

  async function handleStageFullscreenToggle(event) {
    if (!window.CARROT_VISION_ACTIVE) return;
    if (shouldIgnoreStageFullscreenToggle(event?.target)) return;
    if (typeof window.ToggleCarrotFullscreen !== "function") return;
    await window.ToggleCarrotFullscreen({ quiet: false }).catch(() => {});
  }

  function requestFullRender() {
    refresh();
    requestRender({ force: true, overlayDirty: true, hudDirty: true });
  }

  function handleLifecycleChange() {
    if (isStageVisible()) {
      if (window.CARROT_VISION_ACTIVE) {
        setStageLoading(true, getUIText("connecting", "Connecting..."));
      }
      refreshOverlayInfo().catch(() => {});
      requestFullRender();
      return;
    }
    cancelScheduledRender();
    setStageLoading(false);
    if (!isActive()) resetCarrotHudLayout();
  }

  stageEl.addEventListener("click", handleStageFullscreenToggle);
  window.addEventListener("resize", requestFullRender);
  window.addEventListener("orientationchange", requestFullRender);
  document.addEventListener("fullscreenchange", requestFullRender);
  document.addEventListener("webkitfullscreenchange", requestFullRender);
  window.addEventListener("carrot:render-request", (event) => requestRender(event.detail || {}));
  window.addEventListener("carrot:pagechange", handleLifecycleChange);
  window.addEventListener("carrot:visionchange", handleLifecycleChange);
  document.addEventListener("visibilitychange", handleLifecycleChange);
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", requestFullRender, { passive: true });
    window.visualViewport.addEventListener("scroll", requestFullRender, { passive: true });
  }
  const renderVideoTargets = sourceVideoEl === videoEl ? [videoEl] : [sourceVideoEl, videoEl];
  ["loadedmetadata", "loadeddata", "playing", "resize", "emptied"].forEach((eventName) => {
    renderVideoTargets.forEach((target) => target.addEventListener(eventName, requestFullRender));
  });

  try {
    const stored = Number(localStorage.getItem(DISPLAY_MODE_STORAGE_KEY));
    if (Number.isFinite(stored)) {
      displayModeIndex = clamp(stored, 0, DISPLAY_MODES.length - 1);
    }
  } catch {}

  syncDisplayModeButtons();
  refreshParams(true);
  refreshOverlayInfo(true).catch(() => {});
  requestRender({ force: true, overlayDirty: true, hudDirty: true });

  return {
    refresh,
    requestRender,
    setDisplayModeIndex,
  };
})();
