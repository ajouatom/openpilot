/* ============================================================================
 * CARROT VISION GRAPHICS - DO NOT CHANGE CASUALLY
 *
 * This is the driving view the user actually looks at. Its smoothness was tuned
 * against real device behaviour and several "obvious" simplifications have
 * already been tried and reverted. If you are here while working on AR, replay
 * or any other feature, prefer adding your own layer over editing this one.
 *
 * Invariants. Breaking any of these brings back stutter, judder or heat:
 *
 *  1. One overlay update per presented video frame. requestVideoFrameCallback
 *     is taken first and unthrottled; the interval constant only paces the
 *     fallback. Never put a timer between a video frame and its overlay.
 *  2. Live and replay share one scheduler, one filter and one cadence. They are
 *     the same user experience; do not special-case one of them.
 *  3. Temporal smoothing is a time constant, never a per-call alpha. Render
 *     spacing moves constantly, and a fixed alpha makes the response wander.
 *  4. Geometry stays on the GPU: fills, strokes and dashes all land in one
 *     batched draw call. Moving any of them back to Canvas2D reintroduces a
 *     full-surface raster and its clear every frame.
 *  5. The 2D overlay is cleared by dirty rectangle, never skipped wholesale.
 *     The layer is NOT empty on the GPU path - lead badges and every text label
 *     still draw there via drawCanvasOutlinedText / fillRoundedRect /
 *     strokeRoundedRect. If you add anything that can put pixels on `ctx`, it
 *     MUST report its bounds via markOverlayDirty*, or fall back to
 *     markOverlayDirtyFull() when bounds are unknown. A periodic full clear
 *     bounds any mistake; do not remove it. Two earlier attempts shipped bugs:
 *     a plain "did anything draw?" flag produced accumulating ghosting, and
 *     hiding the layer made the lead box and badges vanish outright.
 *  6. Vertex pools are reused. Do not rebuild per-frame arrays.
 *
 * Already tried and rejected:
 *  - Overlay projection/triangulation in a worker: the extra hop makes geometry
 *    trail the video by a frame (see OFFSCREEN_WORKER_ENABLED).
 *  - Interpolating geometry between 20 Hz samples: the video is 20 fps, so the
 *    overlay slides against a still image.
 *  - Uniform Catmull-Rom resampling: overshoots on unevenly spaced projected
 *    points and draws streaks across the frame. Centripetal only.
 *
 * Measured state: worst curve kink 8.1deg -> 1.6deg, geometry fully GPU-batched,
 * per-frame 2D clear removed.
 * ==========================================================================*/

"use strict";

window.HomeDrive = (() => {
  const stageEl = document.getElementById("carrotStage");
  const replayViewportEl = document.getElementById("carrotReplayViewport");
  const videoEl = document.getElementById("carrotRoadVideo");
  const videoHoldEl = document.getElementById("carrotLastFrameCanvas");
  const performanceCanvasEl = document.getElementById("carrotPerformanceCanvas");
  const canvasEl = document.getElementById("carrotOverlayCanvas");
  const hudCanvasEl = document.getElementById("carrotHudCanvas");
  const onroadAlertEl = document.getElementById("carrotOnroadAlert");
  const onroadAlertBoxEl = document.getElementById("carrotOnroadAlertBox");
  const onroadAlertText1El = document.getElementById("carrotOnroadAlertText1");
  const onroadAlertText2El = document.getElementById("carrotOnroadAlertText2");
  const stageLoadingEl = document.getElementById("carrotStageLoading");
  const stageLoadingTextEl = document.getElementById("carrotStageLoadingText");
  const stageLoadingDetailEl = document.getElementById("carrotStageLoadingDetail");
  const stateSurfaceApi = globalThis.CarrotUI?.stateSurface;
  const stageOwnershipSurface = stateSurfaceApi?.create?.({
    host: stageLoadingEl,
    className: "carrot-stage__ownershipNotice",
    featureLabel: () => getUIText("web_drive_layout_content_vision", "Carrot Vision"),
  });
  const visionHudContent = window.DriveVisionHudContent;
  const driveHudCardEl = visionHudContent?.root || null;
  const sourceVideoEl = videoEl;
  const displayModeButton = document.getElementById("btnDisplayModeCycle");

  if (!stageEl || !videoEl || !canvasEl || !hudCanvasEl
      || !stageLoadingEl || !stageOwnershipSurface || !visionHudContent) {
    return {};
  }

  const ctx = canvasEl.getContext("2d");
  const hudCtx = hudCanvasEl.getContext("2d");
  if (!ctx || !hudCtx) {
    return {};
  }
  const performanceRenderer = window.CarrotVisionWebGL2?.getRenderer?.(performanceCanvasEl) || null;
  const replayRenderBridge = window.DriveVisionReplayRenderBridge?.create?.({
    stage: stageEl,
    replayViewport: replayViewportEl,
    video: videoEl,
    getReplay: () => window.CarrotVisionReplay,
  }) || null;
  const visionViewport = (window.DriveVisionViewport || window.DriveVisionHudLayout)?.create?.({
    stage: stageEl,
    card: driveHudCardEl,
  }) || null;
  const hudCanvas = window.DriveVisionHudCanvas?.create?.({
    stage: stageEl,
    context: hudCtx,
  }) || null;
  const hudModel = window.DriveVisionHudModel || null;
  const roadOverlayPolicy = window.DriveVisionRoadOverlayPolicy?.create?.({ hudModel }) || null;
  const roadOverlayLeadModel = window.DriveVisionRoadOverlayLeadModel?.create?.({
    text: (key, fallback) => getUIText(key, fallback),
  }) || null;
  const rtcPerfHud = window.DriveVisionRtcPerfHud?.create?.({
    root: driveHudCardEl,
    model: hudModel,
    isActive,
    isReplayActive: replayRenderBridge?.isActive,
    showToast(message, options) {
      if (typeof showAppToast === "function") showAppToast(message, options);
    },
  }) || null;
  if (!replayRenderBridge || !visionViewport || !hudCanvas || !hudModel || !roadOverlayPolicy || !roadOverlayLeadModel || !rtcPerfHud) return {};
  let performanceGeometryActive = false;

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
  // Road-camera intrinsics. Native openpilot selects these from
  // DEVICE_CAMERAS[(deviceType, sensor)]; the web renderer follows that same
  // rule so C3/C3X/C4 share one projection path.
  const BASE_CAMERA = {
    width: 1928,
    height: 1208,
    focalX: 2648,
    focalY: 2648,
  };
  // Mirror of openpilot common/transformations/camera.py DEVICE_CAMERAS[*].fcam.
  const ROAD_CAMERA_PROFILES = {
    ar_ox: { width: 1928, height: 1208, focal: 2648.0 },
    os04c10: { width: 1344, height: 760, focal: 1141.5 },
  };
  let _roadCameraDeviceType = "";
  let _roadCameraSensor = "";
  let _roadCameraProfileKey = "ar_ox";
  let _roadCameraProfileResolved = false;
  let _lastValidCalibrationMatrix = null;
  let _lastValidCalibrationSignature = "";
  const DISPLAY_MODES = [
    { key: "fit", labelKey: "display_fit", fallbackLabel: "Fit" },
    { key: "normal", labelKey: "display_normal", fallbackLabel: "Normal" },
    { key: "crop", labelKey: "display_crop", fallbackLabel: "Crop" },
  ];
  const DISPLAY_MODE_SETTING_KEY = "vision_display_mode";
  const HUD_TEXT_FONT = "system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif";
  const PHONE_PORTRAIT_DPR_CAP = 1.0;
  const MOBILE_DPR_CAP = 1.25;
  const DESKTOP_DPR_CAP = 1.5;
  const PERFORMANCE_RENDER_DPR_CAP = 3.0;
  /* Carrot Vision motion policy — keep this in mind for any future overlay work.
   *
   * The overlay must feel continuous with the video, not merely correct. Three
   * rules produced the current behaviour and breaking any one of them brings
   * back the steppy, stiff look:
   *
   *   1. One overlay update per presented video frame. When
   *      requestVideoFrameCallback is available it is taken unthrottled; the
   *      interval below only paces the fallback path. Never insert a timer
   *      between the video frame and its overlay - setTimeout lands between
   *      composited frames and adds judder on top of the source cadence.
   *   2. Temporal smoothing is a time constant, never a per-call alpha. The
   *      render interval moves constantly (20 Hz data, this cap, scheduler
   *      jitter); a fixed alpha makes the effective response wander and that
   *      inconsistency reads worse than plain lag. See road_overlay_projection
   *      temporalAlpha() and the *_SMOOTH_TAU_MS constants.
   *   3. Live and replay share one filter and one cadence. They are the same
   *      user experience and must not diverge.
   *
   * Also: prefer the WebGL2 geometry path. Rasterising large polygons and
   * gradients on Canvas2D is the known thermal cost on device. And do not move
   * overlay projection/triangulation into a worker - that was tried and the
   * extra message hop makes the geometry trail the video by a frame
   * (see OFFSCREEN_WORKER_ENABLED in vision_webgl2.js).
   */
  const RENDER_INTERVAL_MS = 33;  // fallback pacing only; ~30fps (C3 source: 20Hz/50ms)
  const CAMERA_FRAME_RECHECK_MS = 250;
  const MIN_ROAD_VIDEO_WIDTH = 320;
  const MIN_ROAD_VIDEO_HEIGHT = 180;
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
  const PATH_Z_OFFSET = 1.22;
  const MIN_DRAW_DISTANCE = 10;
  const MAX_DRAW_DISTANCE = 100;
  const RADAR_INTERPOLATION_MIN_MS = 16;
  const RADAR_INTERPOLATION_DEFAULT_MS = 50;
  const RADAR_INTERPOLATION_MAX_MS = 120;
  const RADAR_INTERPOLATION_LEAD_MS = 12;
  const ALERT_STATUS_NORMAL = 0;
  const ALERT_STATUS_USER_PROMPT = 1;
  const ALERT_STATUS_CRITICAL = 2;
  const ALERT_SIZE_NONE = 0;
  const ALERT_SIZE_SMALL = 1;
  const ALERT_SIZE_MID = 2;
  const ALERT_SIZE_FULL = 3;
  const ONROAD_ALERT_SCALE = 1.5;
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
  };

  function isMetricDisplay() {
    return finiteNumber(paramsState.IsMetric, defaultParams.IsMetric) !== 0;
  }

  function displayDistanceMeters(distanceMeters) {
    const distance = Number(distanceMeters);
    if (!Number.isFinite(distance)) return NaN;
    return isMetricDisplay() ? distance : distance * 3.28084;
  }

  let paramsState = { ...defaultParams };
  function displayModeIndexForKey(value) {
    const key = String(value || "").trim().toLowerCase();
    const index = DISPLAY_MODES.findIndex((mode) => mode.key === key);
    return index >= 0 ? index : 1;
  }

  function readServerDisplayModeIndex() {
    const key = typeof window.getWebSettingByKey === "function"
      ? window.getWebSettingByKey(DISPLAY_MODE_SETTING_KEY, "normal")
      : window.CarrotWebSettingsState?.[DISPLAY_MODE_SETTING_KEY];
    return displayModeIndexForKey(key);
  }

  let displayModeIndex = readServerDisplayModeIndex();
  let overlaySizeSignature = "";
  let hudSizeSignature = "";
  let transformSignature = "";
  let lastPlotMode = -1;
  let radarInterpolationState = {
    signature: "",
    previous: null,
    current: null,
    previousAtMs: 0,
    currentAtMs: 0,
  };

  /* ── EMA state for lead box smoothing ──
   * C3 uses alpha=0.85 at a stable 20Hz UI loop. The web uses a time-based
   * EMA: wall time for live video and media time for recorded replay.      */
  const LEAD_EMA_ALPHA = 0.85;
  const C3_FRAME_MS = 50;  // C3 UI loop interval (~20Hz)
  let leadEmaState = [
    { fx: NaN, fy: NaN, fw: NaN, trackId: -1, lastMs: 0 },  // slot 0: leadOne
    { fx: NaN, fy: NaN, fw: NaN, trackId: -1, lastMs: 0 },  // slot 1: leadTwo
  ];
  let leadTwoEmaState = { xl: NaN, xr: NaN, y: NaN, trackId: -1, lastMs: 0 };
  // Lead flicker hold and source-rewind state live in the renderer.
  let roadOverlayLeadRenderer = null;

  /* ── Phase 1-2: dirty check ── */
  let _lastOverlaySig = "";
  let _lastHudSig = "";
  let _lastPlotInputSig = "";
  let _lastAlertSig = "";
  let _forceNextRender = true;
  let _lastRenderTime = 0;
  let _renderRafId = null;
  let _renderVideoFrameId = null;
  let _cameraFrameRecheckId = null;
  let _roadCameraStreamState = {
    stream: null,
    decodedFramesAtBind: null,
    currentTimeAtBind: 0,
    firstRenderableSeen: false,
  };
  let _pendingRenderState = {
    force: true,
    overlayDirty: true,
    hudDirty: true,
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
      liveCalibration?.calStatus ?? "-",
      _roadCameraProfileKey,
      finiteNumber(liveCalibration?.height?.[0], 0).toFixed(2),
      Boolean(controlsState?.activeLaneLine) ? 1 : 0,
      Boolean(controlsState?.enabled) ? 1 : 0,
      finiteNumber(carState?.useLaneLineSpeed, 0).toFixed(2),
      Boolean(carState?.brakeLights) ? 1 : 0,
      Boolean(carState?.leftBlindspot) ? 1 : 0,
      Boolean(carState?.rightBlindspot) ? 1 : 0,
      finiteNumber(carState?.vEgo, 0).toFixed(2),
      overlayState?.lateralPlan?.laneChangeState ?? "-",
      overlayState?.lateralPlan?.laneChangeDirection ?? "-",
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
      paramsState.IsMetric,
    ].join("|");
  }

  function hudDataSignature(hudState, overlayState, plotData) {
    const carState = hudState?.carState;
    const carrotMan = hudState?.carrotMan;
    const longPlan = hudState?.longitudinalPlan;
    const selfdriveState = hudState?.selfdriveState;
    const rtcPerfText = hudModel.formatRtcPerfLabel(window.CarrotRtcPerf);
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
      rtcPerfText,
      plotInputSignature(plotData),
      paramsState.ShowPlotMode,
      paramsState.IsMetric,
    ].join("|");
  }

  /* ── Phase 1-3: gradient cache ── */
  const _gradientCache = new Map();
  const GRADIENT_CACHE_MAX = 16;
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

  function readRpyTriplet(liveCalibration) {
    const source = Array.isArray(liveCalibration?.rpyCalib) ? liveCalibration.rpyCalib : null;
    if (!source) return null;
    const roll = Number(source[0]);
    const pitch = Number(source[1]);
    const yaw = Number(source[2]);
    if (!Number.isFinite(roll) || !Number.isFinite(pitch) || !Number.isFinite(yaw)) return null;
    return [roll, pitch, yaw];
  }

  function normalizeEnumName(value, numericNames = null) {
    if (value == null) return "";
    const text = String(value).trim();
    if (!text) return "";
    const numeric = Number(value);
    if (numericNames && Number.isInteger(numeric) && numericNames[numeric]) {
      return numericNames[numeric];
    }
    const normalized = text.toLowerCase();
    const tail = normalized.split(".").pop() || normalized;
    return tail.replace(/[^a-z0-9_]/g, "");
  }

  function isCalibrationComplete(liveCalibration) {
    const status = normalizeEnumName(liveCalibration?.calStatus, {
      0: "uncalibrated",
      1: "calibrated",
      2: "invalid",
      3: "recalibrating",
    });
    return status === "calibrated";
  }

  function firstFinite(values, fallback = 0) {
    if (!Array.isArray(values)) return fallback;
    for (const value of values) {
      const num = Number(value);
      if (Number.isFinite(num)) return num;
    }
    return fallback;
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
    // Match native onroad: only completed calibration may replace the last
    // known-good transform. Invalid/recalibrating values must not move graphics.
    if (!isCalibrationComplete(liveCalibration)) return _lastValidCalibrationMatrix;
    const rpy = readRpyTriplet(liveCalibration);
    if (!rpy) return _lastValidCalibrationMatrix;
    const signature = rpy.map((value) => value.toFixed(8)).join("|");
    if (_lastValidCalibrationMatrix && signature === _lastValidCalibrationSignature) {
      return _lastValidCalibrationMatrix;
    }
    _lastValidCalibrationSignature = signature;
    _lastValidCalibrationMatrix = mat3Multiply(VIEW_FROM_DEVICE, rotFromEuler(rpy[0], rpy[1], rpy[2]));
    return _lastValidCalibrationMatrix;
  }

  function selectRoadCameraProfile(deviceType, sensor) {
    const deviceKey = normalizeEnumName(deviceType, {
      0: "unknown", 1: "neo", 2: "chffrandroid", 3: "chffrios",
      4: "tici", 5: "pc", 6: "tizi", 7: "mici",
    });
    const sensorKey = normalizeEnumName(sensor, {
      0: "unknown", 1: "ar0231", 2: "ox03c10", 3: "os04c10",
    });

    if (sensorKey === "os04c10") return { key: "os04c10", deviceKey, sensorKey };
    if (sensorKey === "ar0231" || sensorKey === "ox03c10") return { key: "ar_ox", deviceKey, sensorKey };
    // Only tici has a native "unknown sensor" fallback. C3X/C4 must wait for
    // their actual sensor instead of briefly drawing with a guessed camera.
    if (deviceKey === "tici") return { key: "ar_ox", deviceKey, sensorKey: sensorKey || "unknown" };
    return null;
  }

  function applyRoadCameraProfile(deviceType, sensor) {
    const selected = selectRoadCameraProfile(deviceType, sensor);
    if (!selected) return _roadCameraProfileResolved;
    _roadCameraProfileResolved = true;
    _roadCameraDeviceType = selected.deviceKey;
    _roadCameraSensor = selected.sensorKey;
    if (selected.key === _roadCameraProfileKey) return true;

    const cfg = ROAD_CAMERA_PROFILES[selected.key];
    if (!cfg) return false;
    _roadCameraProfileKey = selected.key;
    if (BASE_CAMERA.width === cfg.width && BASE_CAMERA.height === cfg.height &&
        BASE_CAMERA.focalX === cfg.focal && BASE_CAMERA.focalY === cfg.focal) {
      return true;
    }
    BASE_CAMERA.width = cfg.width;
    BASE_CAMERA.height = cfg.height;
    BASE_CAMERA.focalX = cfg.focal;
    BASE_CAMERA.focalY = cfg.focal;
    roadOverlayProjection.resetFrame(performance.now());
    transformSignature = "";
    _forceNextRender = true;
    try {
      console.log("[carrot-vision] road camera profile:", {
        deviceType: _roadCameraDeviceType,
        sensor: _roadCameraSensor,
        profile: _roadCameraProfileKey,
        config: cfg,
      });
    } catch (_) {}
    return true;
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

  function getRenderViewportOrigin(renderViewport) {
    if (!renderViewport || renderViewport === stageEl) return { left: 0, top: 0 };
    const stageRect = stageEl.getBoundingClientRect?.();
    const viewportRect = renderViewport.getBoundingClientRect?.();
    if (!stageRect || !viewportRect) {
      return {
        left: Math.max(0, Number(renderViewport.offsetLeft) || 0),
        top: Math.max(0, Number(renderViewport.offsetTop) || 0),
      };
    }
    // DOMRect includes any outer workspace scaling. Convert its delta back to
    // stage-local CSS pixels before positioning the AR worker surface.
    const scaleX = stageEl.clientWidth > 0 && stageRect.width > 0
      ? stageRect.width / stageEl.clientWidth
      : 1;
    const scaleY = stageEl.clientHeight > 0 && stageRect.height > 0
      ? stageRect.height / stageEl.clientHeight
      : 1;
    return {
      left: (viewportRect.left - stageRect.left) / Math.max(scaleX, 0.001),
      top: (viewportRect.top - stageRect.top) / Math.max(scaleY, 0.001),
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

  const roadOverlayProjection = window.DriveVisionRoadOverlayProjection?.create?.({
    projectPoint,
    projectPointPrecise,
    isRecordedReplayActive: replayRenderBridge.isActive,
    maxDrawDistance: MAX_DRAW_DISTANCE,
    isFullDetailActive: () => performanceGeometryActive,
  }) || null;
  if (!roadOverlayProjection) return {};

  function getLeadTemporalNowMs() {
    return replayRenderBridge.temporalNowMs(performance.now());
  }

  function getHeldLeadBox(nowMs = getLeadTemporalNowMs()) {
    return roadOverlayLeadRenderer?.getHeldBox(nowMs) || null;
  }

  function getPrimaryLeadDistance(overlayState = null, nowMs = getLeadTemporalNowMs()) {
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

  /* Dirty-rectangle bookkeeping for the 2D overlay.
   *
   * Only lead badges and text labels still draw here; everything geometric is
   * batched on the GPU. Erasing the whole video-resolution surface to make room
   * for a few small badges is almost all of the cost of this layer.
   *
   * Correctness rule: every function that can put pixels on `ctx` must report
   * its bounds. There are exactly five such entry points and they all live in
   * this file. Anything that cannot compute bounds reports a full-surface
   * repaint instead of guessing.
   *
   * Safety net: a full clear is forced periodically regardless. If a bound is
   * ever wrong or a new draw path is added without reporting, the artefact is
   * bounded to that interval rather than accumulating forever - which is the
   * failure that a plain "did anything draw?" flag shipped.
   */
  const OVERLAY_FULL_CLEAR_INTERVAL = 30;
  const OVERLAY_DIRTY_PAD = 3;
  let overlayDirtyRect = null;
  let overlayDirtyFull = false;
  let overlayFramesSinceFullClear = 0;

  function markOverlayDirtyFull() {
    overlayDirtyFull = true;
  }

  function markOverlayDirtyBox(minX, minY, maxX, maxY) {
    if (![minX, minY, maxX, maxY].every((value) => Number.isFinite(value))) {
      markOverlayDirtyFull();
      return;
    }
    if (!overlayDirtyRect) {
      overlayDirtyRect = { minX, minY, maxX, maxY };
      return;
    }
    if (minX < overlayDirtyRect.minX) overlayDirtyRect.minX = minX;
    if (minY < overlayDirtyRect.minY) overlayDirtyRect.minY = minY;
    if (maxX > overlayDirtyRect.maxX) overlayDirtyRect.maxX = maxX;
    if (maxY > overlayDirtyRect.maxY) overlayDirtyRect.maxY = maxY;
  }

  function markOverlayDirtyPoints(points, pad = 0) {
    if (!Array.isArray(points) || !points.length) return;
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const point of points) {
      const x = Number(point?.x);
      const y = Number(point?.y);
      if (!Number.isFinite(x) || !Number.isFinite(y)) { markOverlayDirtyFull(); return; }
      if (x < minX) minX = x;
      if (y < minY) minY = y;
      if (x > maxX) maxX = x;
      if (y > maxY) maxY = y;
    }
    markOverlayDirtyBox(minX - pad, minY - pad, maxX + pad, maxY + pad);
  }

  function drawPolygon(points, fillStyle, strokeStyle = "", lineWidth = 1) {
    if (!Array.isArray(points) || points.length < 3) return;
    markOverlayDirtyPoints(points, (Number(lineWidth) || 1) / 2 + OVERLAY_DIRTY_PAD);
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

  /* Keep the whole geometry pass on the GPU. Leaving strokes on Canvas2D meant a
   * full-surface 2D raster stayed alive every frame purely for outlines, which
   * is the CPU/thermal cost the WebGL2 path exists to remove. The stroke lands
   * in the same batch as the fill, so this adds no draw call. */
  /* Split a polyline into its dash "on" spans so a dashed line can be stroked as
   * geometry. Canvas2D does this internally; on the GPU path we have to do it
   * ourselves, and it is the last thing keeping a 2D raster alive per frame. */
  function dashSpans(points, dashPattern, dashOffset) {
    const pattern = Array.isArray(dashPattern) ? dashPattern.filter((n) => Number(n) > 0) : [];
    if (!pattern.length) return [points];
    const cycle = pattern.reduce((sum, value) => sum + value, 0);
    if (!(cycle > 0)) return [points];

    const spans = [];
    let current = [];
    let index = 0;
    let remaining = pattern[0];
    let drawing = true;
    // A negative offset animates the dash forward; normalise then consume it.
    let offset = ((Number(dashOffset) || 0) % cycle + cycle) % cycle;
    while (offset > 0) {
      const step = Math.min(offset, remaining);
      remaining -= step;
      offset -= step;
      if (remaining <= 1e-6) {
        index = (index + 1) % pattern.length;
        remaining = pattern[index];
        drawing = !drawing;
      }
    }

    for (let i = 0; i < points.length - 1; i += 1) {
      const a = points[i];
      const b = points[i + 1];
      let segment = Math.hypot(b.x - a.x, b.y - a.y);
      if (!(segment > 0)) continue;
      let t = 0;
      if (drawing && !current.length) current.push(a);
      while (segment - t > 1e-6) {
        const step = Math.min(remaining, segment - t);
        const from = t / segment;
        t += step;
        remaining -= step;
        const to = t / segment;
        const point = { x: a.x + (b.x - a.x) * to, y: a.y + (b.y - a.y) * to };
        if (drawing) {
          if (!current.length) current.push({ x: a.x + (b.x - a.x) * from, y: a.y + (b.y - a.y) * from });
          current.push(point);
        }
        if (remaining <= 1e-6) {
          if (drawing && current.length > 1) spans.push(current);
          current = [];
          index = (index + 1) % pattern.length;
          remaining = pattern[index];
          drawing = !drawing;
        }
      }
    }
    if (current.length > 1) spans.push(current);
    return spans;
  }

  function drawGeometryPolyline(points, strokeStyle, lineWidth, dashPattern = [], dashOffset = 0) {
    if (
      performanceGeometryActive
      && performanceRenderer
      && typeof performanceRenderer.drawStroke === "function"
      && Array.isArray(points)
      && points.length >= 2
    ) {
      for (const span of dashSpans(points, dashPattern, dashOffset)) {
        performanceRenderer.drawStroke(span, strokeStyle, lineWidth, false);
      }
      return;
    }
    drawPolyline(points, strokeStyle, lineWidth, dashPattern, dashOffset);
  }

  function drawGeometryPolygon(points, fillStyle, strokeStyle = "", lineWidth = 1) {
    if (performanceGeometryActive && performanceRenderer && (fillStyle || strokeStyle)) {
      if (fillStyle) performanceRenderer.drawPolygon(points, fillStyle);
      if (strokeStyle) {
        if (typeof performanceRenderer.drawStroke === "function") {
          performanceRenderer.drawStroke(points, strokeStyle, lineWidth);
        } else {
          drawPolygon(points, "", strokeStyle, lineWidth);
        }
      }
      return;
    }
    drawPolygon(points, fillStyle, strokeStyle, lineWidth);
  }

  function drawPolyline(points, strokeStyle, lineWidth, dashPattern = [], dashOffset = 0) {
    if (!Array.isArray(points) || points.length < 2) return;
    markOverlayDirtyPoints(points, (Number(lineWidth) || 1) / 2 + OVERLAY_DIRTY_PAD);
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

  function getCarrotVisionState() {
    return window.CarrotVisionState || {};
  }

  function isCarrotVisionActive() {
    const state = getCarrotVisionState();
    return Boolean(state.active ?? window.CARROT_VISION_ACTIVE);
  }

  function getCarrotVisionStatusText(fallback = "") {
    const state = getCarrotVisionState();
    return String(state.statusText || fallback || "");
  }

  function getCarrotVisionDetailText() {
    const state = getCarrotVisionState();
    return String(state.detailText || "");
  }

  function setCarrotVisionRenderPhase(phase, detail = {}) {
    if (replayRenderBridge.reportRenderable(phase === "ready")) return;
    // Single phase owner: home_drive is the authority on whether a real camera
    // frame is on screen, but it no longer writes the phase directly. It
    // reports renderability to vision_rtc, which owns the live/first-frame
    // transitions (removes the old home_drive/vision_rtc phase race).
    const rtc = window.CarrotVisionRtc;
    if (rtc && typeof rtc.reportCameraRenderable === "function") {
      if (phase === "ready") { rtc.reportCameraRenderable(true); return; }
      if (phase === "first-frame-waiting") { rtc.reportCameraRenderable(false); return; }
    }
    // Fallback (other phases, or vision_rtc not loaded yet): set directly.
    if (typeof window.CarrotVisionSetPhase !== "function") return;
    window.CarrotVisionSetPhase(phase, {
      source: "home_drive",
      updateRtcStatus: false,
      render: false,
      ...detail,
    });
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
    const alertScale = ONROAD_ALERT_SCALE;
    const alertPx = (value) => `${value * alertScale}px`;

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
      alertScale,
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
    onroadAlertEl.style.setProperty("--carrot-alert-pad-min", alertPx(4));
    onroadAlertEl.style.setProperty("--carrot-alert-pad-fluid", `${1.4 * alertScale}vw`);
    onroadAlertEl.style.setProperty("--carrot-alert-pad-max", alertPx(12));
    onroadAlertEl.style.setProperty("--carrot-alert-shadow-y-lg", alertPx(8));
    onroadAlertEl.style.setProperty("--carrot-alert-shadow-blur-lg", alertPx(24));
    onroadAlertEl.style.setProperty("--carrot-alert-shadow-y-sm", alertPx(3));
    onroadAlertEl.style.setProperty("--carrot-alert-shadow-blur-sm", alertPx(10));
    onroadAlertEl.style.setProperty("--carrot-alert-outline-strong-pos", alertPx(2.25));
    onroadAlertEl.style.setProperty("--carrot-alert-outline-strong-neg", alertPx(-2.25));
    onroadAlertEl.style.setProperty("--carrot-alert-outline-soft-pos", alertPx(1.8));
    onroadAlertEl.style.setProperty("--carrot-alert-outline-soft-neg", alertPx(-1.8));
    onroadAlertEl.style.setProperty("--carrot-alert-stroke-primary", alertPx(0.55));
    onroadAlertEl.style.setProperty("--carrot-alert-stroke-secondary", alertPx(0.48));
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

  function setDisplayModeIndex(nextIndex, options = {}) {
    displayModeIndex = clamp(nextIndex, 0, DISPLAY_MODES.length - 1);
    if (options.persist !== false) {
      const mode = DISPLAY_MODES[displayModeIndex] || DISPLAY_MODES[1];
      const request = window.setWebSettingByKey?.(DISPLAY_MODE_SETTING_KEY, mode.key);
      request?.catch?.(() => {});
    }
    transformSignature = "";
    syncDisplayModeButtons();
  }

  function syncDisplayModeFromServer(event) {
    const keys = Array.isArray(event?.detail?.keys) ? event.detail.keys : [];
    if (keys.length && !keys.includes(DISPLAY_MODE_SETTING_KEY)) return;
    const nextIndex = readServerDisplayModeIndex();
    if (nextIndex === displayModeIndex) return;
    setDisplayModeIndex(nextIndex, { persist: false });
    requestRender({ force: true, overlayDirty: true, hudDirty: true });
  }

  function getDisplayModeLabel(mode) {
    if (!mode) return getUIText("display_normal", "Normal");
    return getUIText(mode.labelKey, mode.fallbackLabel || mode.key || "Normal");
  }

  function syncSourceStream() {
    if (replayRenderBridge.isActive()) {
      const sourceKey = replayRenderBridge.videoSourceKey();
      if (_roadCameraStreamState.stream !== sourceKey) {
        _roadCameraStreamState = {
          stream: sourceKey,
          decodedFramesAtBind: sourceKey ? getDecodedVideoFrameCount(videoEl) : null,
          currentTimeAtBind: sourceKey ? Number(videoEl.currentTime || 0) : 0,
          firstRenderableSeen: false,
        };
        cancelCameraFrameRecheck();
      }
      return Boolean(sourceKey);
    }

    const stream = sourceVideoEl.srcObject || null;
    if (sourceVideoEl !== videoEl && videoEl.srcObject !== stream) {
      videoEl.srcObject = stream;
    }

    const activeStream = (sourceVideoEl === videoEl ? sourceVideoEl : videoEl).srcObject || stream;
    if (_roadCameraStreamState.stream !== activeStream) {
      _roadCameraStreamState = {
        stream: activeStream,
        decodedFramesAtBind: activeStream ? getDecodedVideoFrameCount(videoEl) : null,
        currentTimeAtBind: activeStream ? Number(videoEl.currentTime || 0) : 0,
        firstRenderableSeen: false,
      };
      cancelCameraFrameRecheck();
    }

    const hasStream = Boolean(activeStream);
    if (hasStream && videoEl.paused) {
      videoEl.play().catch(() => {});
    }
    return hasStream;
  }

  function hasLiveVideoTrack(video) {
    const stream = video?.srcObject;
    if (!stream || stream.active === false) return false;
    if (typeof stream.getVideoTracks !== "function") return true;
    const tracks = stream.getVideoTracks();
    if (!tracks.length) return false;
    return tracks.some((track) => track && track.readyState !== "ended" && track.muted !== true);
  }

  function getDecodedVideoFrameCount(video) {
    try {
      if (typeof video?.getVideoPlaybackQuality === "function") {
        const quality = video.getVideoPlaybackQuality();
        const total = Number(quality?.totalVideoFrames);
        if (Number.isFinite(total)) return total;
      }
    } catch {}
    const webkitCount = Number(video?.webkitDecodedFrameCount);
    return Number.isFinite(webkitCount) ? webkitCount : null;
  }

  function isRoadCameraFrameRenderable(video) {
    const recordedReplay = replayRenderBridge.isActive();
    if (!video) return false;
    if (!recordedReplay && (!video.srcObject || !hasLiveVideoTrack(video))) return false;
    const videoWidth = Number(video.videoWidth || 0);
    const videoHeight = Number(video.videoHeight || 0);
    if (videoWidth < MIN_ROAD_VIDEO_WIDTH || videoHeight < MIN_ROAD_VIDEO_HEIGHT) return false;
    if (Number(video.readyState || 0) < 2) return false;
    if (recordedReplay) {
      _roadCameraStreamState.firstRenderableSeen = true;
      return true;
    }
    if (_roadCameraStreamState.firstRenderableSeen) return true;

    const decodedFrames = getDecodedVideoFrameCount(video);
    const baselineFrames = _roadCameraStreamState.decodedFramesAtBind;
    if (decodedFrames != null && (baselineFrames == null ? decodedFrames > 0 : decodedFrames > baselineFrames)) {
      _roadCameraStreamState.firstRenderableSeen = true;
      return true;
    }

    const currentTime = Number(video.currentTime || 0);
    const baselineTime = Number(_roadCameraStreamState.currentTimeAtBind || 0);
    if (Number.isFinite(currentTime) && currentTime > baselineTime + 0.05 && video.paused !== true) {
      _roadCameraStreamState.firstRenderableSeen = true;
      return true;
    }
    if (decodedFrames == null && Number(video.readyState || 0) >= 3 && video.paused !== true) {
      _roadCameraStreamState.firstRenderableSeen = true;
      return true;
    }
    return false;
  }

  function scheduleCameraFrameRecheck() {
    if (_cameraFrameRecheckId != null || !isStageVisible() || !isCarrotVisionActive()) return;
    _cameraFrameRecheckId = window.setTimeout(() => {
      _cameraFrameRecheckId = null;
      requestRender({ force: true, overlayDirty: true, hudDirty: true });
    }, CAMERA_FRAME_RECHECK_MS);
  }

  function cancelCameraFrameRecheck() {
    if (_cameraFrameRecheckId == null) return;
    window.clearTimeout(_cameraFrameRecheckId);
    _cameraFrameRecheckId = null;
  }

  let _lastStageReady = null;
  function setStageReady(ready) {
    const r = Boolean(ready);
    if (_lastStageReady === r) return;
    _lastStageReady = r;
    stageEl.classList.toggle("is-stream-ready", r);
  }

  let _lastStageLoading = null;
  let _lastStageLoadingText = "";
  let _lastStageLoadingDetail = "";

  function setStageLoading(loading, text = getUIText("connecting", "Connecting..."), detail = "") {
    const l = Boolean(loading);
    const controlState = l ? String(getCarrotVisionState().controlState || "") : "";
    const ownershipBlocked = controlState === "blocked";
    if (_lastStageLoading !== l) {
      _lastStageLoading = l;
      stageEl.classList.toggle("is-loading", l);
      visionHudContent.setLoading(l);
      if (stageLoadingEl) stageLoadingEl.hidden = !l;
    }
    if (stageLoadingEl) stageLoadingEl.dataset.controlState = controlState;
    if (stageLoadingEl) stageLoadingEl.setAttribute("aria-busy", String(l && !ownershipBlocked));
    if (!l) {
      stageOwnershipSurface.hide();
      return;
    }
    if (ownershipBlocked) {
      stageOwnershipSurface.show(stateSurfaceApi.STATE.OWNERSHIP, {
        tone: stateSurfaceApi.TONE.NEUTRAL,
        title: getUIText("drive_stream_busy", "Active on another device"),
        actions: [
          {
            id: "takeover",
            label: getUIText("drive_stream_use_here", "Use here"),
            tone: stateSurfaceApi.ACTION_KIND.PRIMARY,
            onActivate: () => window.CarrotVisionRtc?.takeOwnership?.("busy action"),
          },
          {
            id: "stop",
            label: getUIText("vision_stop", "Stop"),
            tone: stateSurfaceApi.ACTION_KIND.SECONDARY,
            onActivate: () => window.CarrotVisionStop?.("ownership notice stop"),
          },
        ],
      });
    } else {
      stageOwnershipSurface.hide();
    }
    if (stageLoadingTextEl && _lastStageLoadingText !== text) {
      _lastStageLoadingText = text;
      stageLoadingTextEl.textContent = text;
    }
    const detailText = l ? String(detail || "") : "";
    if (stageLoadingDetailEl && _lastStageLoadingDetail !== detailText) {
      _lastStageLoadingDetail = detailText;
      stageLoadingDetailEl.textContent = detailText;
      stageLoadingDetailEl.hidden = !detailText;
    }
  }

  function getCanvasDpr() {
    const rawDpr = window.devicePixelRatio || 1;
    const portrait = !window.CarrotLayout?.isWide?.();
    const shortSide = Math.min(window.innerWidth || 0, window.innerHeight || 0);
    if (portrait && shortSide > 0 && shortSide <= 540) {
      return Math.min(rawDpr, PHONE_PORTRAIT_DPR_CAP);
    }
    const cap = shortSide >= 960 ? DESKTOP_DPR_CAP : MOBILE_DPR_CAP;
    return Math.min(rawDpr, cap);
  }

  function prefersPerformanceRenderer() {
    return Boolean(performanceCanvasEl && performanceRenderer);
  }

  function getPerformanceCanvasDpr(baseDpr) {
    if (!prefersPerformanceRenderer()) return baseDpr;
    // This scales only the backing pixels. Geometry keeps using the original
    // videoWidth/videoHeight coordinate space passed to beginFrame().
    const rawDpr = Math.max(1, Number(window.devicePixelRatio) || 1);
    return Math.max(baseDpr, Math.min(rawDpr, PERFORMANCE_RENDER_DPR_CAP));
  }

  function beginGeometryFrame(videoWidth, videoHeight) {
    performanceGeometryActive = Boolean(
      prefersPerformanceRenderer()
      && performanceCanvasEl
      && performanceRenderer?.beginFrame?.(videoWidth, videoHeight)
    );
    stageEl.classList.toggle("is-performance-renderer", performanceGeometryActive);
    if (!performanceGeometryActive) performanceRenderer?.clear?.();
  }

  function endGeometryFrame() {
    if (performanceGeometryActive) performanceRenderer?.endFrame?.();
  }

  function syncCanvasSize(videoWidth, videoHeight, stageWidth, stageHeight) {
    const dpr = getCanvasDpr();
    const performanceDpr = getPerformanceCanvasDpr(dpr);
    // Performance mode is a hybrid renderer: WebGL2 owns fills while the 2D
    // overlay owns their strokes and animated center lines. Rasterizing those
    // stacked layers at the same DPR prevents sub-pixel edge separation.
    const geometryDpr = prefersPerformanceRenderer() ? performanceDpr : dpr;

    const nextOverlaySignature = `${videoWidth}x${videoHeight}@${geometryDpr.toFixed(2)}|p${performanceDpr.toFixed(2)}`;
    if (overlaySizeSignature !== nextOverlaySignature) {
      overlaySizeSignature = nextOverlaySignature;
      roadOverlayProjection.resetTemporal();
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
      canvasEl.width = Math.max(1, Math.round(videoWidth * geometryDpr));
      canvasEl.height = Math.max(1, Math.round(videoHeight * geometryDpr));
      ctx.setTransform(geometryDpr, 0, 0, geometryDpr, 0, 0);
      if (performanceCanvasEl) {
        performanceCanvasEl.style.width = `${videoWidth}px`;
        performanceCanvasEl.style.height = `${videoHeight}px`;
        const performanceWidth = Math.max(1, Math.round(videoWidth * performanceDpr));
        const performanceHeight = Math.max(1, Math.round(videoHeight * performanceDpr));
        if (performanceRenderer?.ownsOffscreenCanvas?.()) {
          performanceRenderer.resize(performanceWidth, performanceHeight);
        } else {
          performanceCanvasEl.width = performanceWidth;
          performanceCanvasEl.height = performanceHeight;
        }
      }
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
    roadOverlayProjection.resetTemporal();
    const cssMatrix = `matrix(${transform.scale}, 0, 0, ${transform.scale}, ${transform.tx}, ${transform.ty})`;
    videoEl.style.transform = cssMatrix;
    if (videoHoldEl) videoHoldEl.style.transform = cssMatrix;
    if (performanceCanvasEl) performanceCanvasEl.style.transform = cssMatrix;
    canvasEl.style.transform = cssMatrix;
  }

  function clearOverlay(videoWidth, videoHeight, options = {}) {
    const forceFull = options.full === true
      || overlayDirtyFull
      || overlayFramesSinceFullClear >= OVERLAY_FULL_CLEAR_INTERVAL;
    if (forceFull) {
      ctx.clearRect(0, 0, videoWidth, videoHeight);
      overlayFramesSinceFullClear = 0;
    } else if (overlayDirtyRect) {
      const { minX, minY, maxX, maxY } = overlayDirtyRect;
      ctx.clearRect(minX, minY, maxX - minX, maxY - minY);
      overlayFramesSinceFullClear += 1;
    } else {
      // Nothing reached this layer last frame, so there is nothing to erase.
      overlayFramesSinceFullClear += 1;
    }
    overlayDirtyRect = null;
    overlayDirtyFull = false;
    if (options.clearPerformance) performanceRenderer?.clear?.();
  }

  const roadOverlayGeometryRenderer = window.DriveVisionRoadOverlayGeometryRenderer?.create?.({
    context: ctx,
    getParams: () => paramsState,
    isPerformanceActive: () => performanceGeometryActive,
    pathZOffset: PATH_Z_OFFSET,
    maxDrawDistance: MAX_DRAW_DISTANCE,
    paletteColor,
    rgba,
    getCachedGradient,
    geometry: {
      drawPolygon: drawGeometryPolygon,
      drawPolyline: drawGeometryPolyline,
      buildRibbon: roadOverlayProjection.buildRibbon,
      smoothRibbon: roadOverlayProjection.smoothRibbon,
      buildBandPolygon: roadOverlayProjection.buildBandPolygon,
      getSceneMaxDistance,
      getPathMaxDistance,
      getPathLengthIndex: roadOverlayProjection.getPathLengthIndex,
    },
  }) || null;
  if (!roadOverlayGeometryRenderer) return {};

  function resetLeadEmaSlot(slot) {
    if (slot !== 0 && slot !== 1) return;
    leadEmaState[slot] = { fx: NaN, fy: NaN, fw: NaN, trackId: -1, lastMs: 0 };
  }

  function resetLeadTwoEma() {
    leadTwoEmaState = { xl: NaN, xr: NaN, y: NaN, trackId: -1, lastMs: 0 };
  }

  function resetLeadTemporalState() {
    resetLeadEmaSlot(0);
    resetLeadEmaSlot(1);
    resetLeadTwoEma();
    roadOverlayLeadRenderer?.reset();
  }

  function resetVisualTemporalState() {
    roadOverlayProjection.resetTemporal();
    resetLeadTemporalState();
  }

  function resetReplayProjectionState() {
    // A replay/seek can switch device camera profiles and calibration. Never
    // draw the new segment using a previous segment's last-known-good matrix.
    _roadCameraDeviceType = "";
    _roadCameraSensor = "";
    _roadCameraProfileResolved = false;
    _lastValidCalibrationMatrix = null;
    _lastValidCalibrationSignature = "";
    transformSignature = "";
    roadOverlayProjection.resetFrame(performance.now());
    _forceNextRender = true;
  }

  function resetReplayVisualState() {
    resetVisualTemporalState();
    resetReplayProjectionState();
  }

  function getLeadBadgeOffsets(videoWidth, videoHeight) {
    const uiScale = getLeadUiScale(videoWidth, videoHeight);
    return {
      dx: 80 * uiScale,
      rectTopOffset: 25 * uiScale,
      textBaselineOffset: 60 * uiScale,
      badgeHeight: Math.max(42 * uiScale, 20),
      fontSize: Math.max(40 * uiScale, 18),
      radius: Math.max(12 * uiScale, 7),
      strokeWidth: Math.max(4 * uiScale, 2.2),
    };
  }

  function getLeadUiScale(videoWidth, videoHeight) {
    const scaleX = videoWidth / BASE_CAMERA.width;
    const scaleY = videoHeight / BASE_CAMERA.height;
    return clamp(Math.min(scaleX, scaleY), 0.45, 1.0);
  }

  function getLeadProjectionScale(videoWidth, videoHeight) {
    // Geometry must follow the encoded camera resolution exactly. qcamera is
    // 526x330, well below the 0.45 readability floor used for text/icons; using
    // that UI floor for geometry inflates an 800px native box to 360px instead
    // of about 218px and lets the enlarged box escape the replay viewport.
    const scaleX = videoWidth / BASE_CAMERA.width;
    const scaleY = videoHeight / BASE_CAMERA.height;
    return clamp(Math.min(scaleX, scaleY), 0.05, 1.0);
  }

  function getLeadBoxClampMargins(videoWidth, videoHeight, stageWidth = videoWidth, stageHeight = videoHeight, transform = null, options = {}) {
    const visibleRect = getVisibleSourceRect(videoWidth, videoHeight, stageWidth, stageHeight, transform);
    const uiScale = getLeadUiScale(videoWidth, videoHeight);
    // C3 fixed margins scaled to the encoded source resolution. Without this,
    // 964x604 streams keep 1928x1208 margins and force lead UI into the center.
    const topReserve = Math.max(200.0 * uiScale, 96.0);
    const bottomReserve = Math.max(80.0 * uiScale, 42.0);
    const sideReserve = Math.max(350.0 * uiScale, 120.0);
    const topMargin = Math.max(topReserve, visibleRect.top + 6 * uiScale);

    // C3 base: maxCenterY = fb_h - 80
    let maxCenterY = videoHeight - bottomReserve;

    // In crop/fit modes, also keep badges inside visible area
    const offsets = getLeadBadgeOffsets(videoWidth, videoHeight);
    let badgeReserve = 0;
    if (options.includeDistanceBadge !== false) {
      badgeReserve = Math.max(badgeReserve, offsets.rectTopOffset + offsets.badgeHeight + 8 * uiScale);
    }
    if (options.includeStateText) {
      badgeReserve = Math.max(badgeReserve, offsets.textBaselineOffset + Math.max(offsets.fontSize * 0.28, 8 * uiScale));
    }
    maxCenterY = Math.min(maxCenterY, visibleRect.bottom - Math.max(badgeReserve, bottomReserve));
    maxCenterY = Math.max(topMargin, maxCenterY);

    // Preserve the native side reserve when space allows, but keep a usable
    // center lane in aggressively cropped/portrait replay viewports.
    const sideInset = Math.min(sideReserve, Math.max(0, visibleRect.width * 0.28));
    const minCenterX = visibleRect.left + sideInset;
    const maxCenterX = visibleRect.right - sideInset;

    return {
      marginX: sideReserve,
      minCenterX,
      maxCenterX: Math.max(minCenterX, maxCenterX),
      topMargin,
      bottomMargin: Math.max(bottomReserve, videoHeight - maxCenterY),
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
    const z = roadOverlayProjection.samplePathZ(modelPath, distance) + PATH_Z_OFFSET;
    const left = projectPointPrecise(calibTransform, distance, yCenter - 1.2, z);
    const right = projectPointPrecise(calibTransform, distance, yCenter + 1.2, z);
    if (!left || !right) return null;

    const rawWidth = Math.abs(right.x - left.x);
    if (!Number.isFinite(rawWidth) || rawWidth <= 1) return null;

    const rawCenterX = (left.x + right.x) * 0.5;
    const rawCenterY = (left.y + right.y) * 0.5;
    const { minCenterX, maxCenterX, topMargin, maxCenterY, visibleRect } = getLeadBoxClampMargins(
      videoWidth,
      videoHeight,
      stageWidth,
      stageHeight,
      transform,
      options,
    );

    // Match CarrotLink's adaptive bottom margin while keeping carrot.cc clamp policy.
    const _path_x = clamp(rawCenterX, minCenterX, maxCenterX);
    const _path_y = clamp(rawCenterY, topMargin, maxCenterY);
    const uiScale = getLeadUiScale(videoWidth, videoHeight);
    const projectionScale = getLeadProjectionScale(videoWidth, videoHeight);
    const sidePad = Math.max(10 * uiScale, 5);
    const horizontalWidthLimit = Math.max(
      1,
      2 * Math.max(0, Math.min(_path_x - visibleRect.left, visibleRect.right - _path_x) - sidePad),
    );
    const verticalWidthLimit = Math.max(1, (maxCenterY - visibleRect.top) / 0.8);
    const maxWidth = Math.max(1, Math.min(800 * projectionScale, horizontalWidthLimit, verticalWidthLimit));
    const minWidth = Math.min(120 * projectionScale, maxWidth);
    const _path_width = clamp(rawWidth, minWidth, maxWidth);

    // ── Step 2: Time-based EMA on clamped values ──
    // C3 uses alpha=0.85 at stable 20Hz. Live rendering follows wall time;
    // replay follows media time so pause, seek and playback speed stay aligned.
    const ema = leadEmaState[slot] || { fx: NaN, fy: NaN, fw: NaN, trackId: -1, lastMs: 0 };
    const currentTrackId = (lead.radarTrackId != null) ? finiteNumber(lead.radarTrackId, -1) : -1;
    const nowMs = getLeadTemporalNowMs();
    const dt = ema.lastMs > 0 ? clamp(nowMs - ema.lastMs, 0, 500) : C3_FRAME_MS;
    const alpha = dt > 0 ? Math.pow(LEAD_EMA_ALPHA, dt / C3_FRAME_MS) : 1;
    const hasPrev = ema.trackId === currentTrackId
      && Number.isFinite(ema.fx)
      && Number.isFinite(ema.fy)
      && Number.isFinite(ema.fw);
    const path_fx = hasPrev ? (ema.fx * alpha + _path_x * (1 - alpha)) : _path_x;
    const path_fy = hasPrev ? (ema.fy * alpha + _path_y * (1 - alpha)) : _path_y;
    const path_fw = hasPrev ? (ema.fw * alpha + _path_width * (1 - alpha)) : _path_width;

    // ── Step 3: Build box from smoothed values ──
    const width = Math.max(Math.trunc(clamp(path_fw, minWidth, maxWidth)), 1);
    const height = Math.max(Math.trunc(width * 0.8), Math.round(12 * uiScale));
    const safeMinX = visibleRect.left + sidePad + width * 0.5;
    const safeMaxX = visibleRect.right - sidePad - width * 0.5;
    const path_x = Math.trunc(safeMaxX >= safeMinX
      ? clamp(path_fx, safeMinX, safeMaxX)
      : visibleRect.left + visibleRect.width * 0.5);
    const safeMinY = Math.min(maxCenterY, Math.max(topMargin, visibleRect.top + height));
    const path_y = Math.trunc(clamp(path_fy, safeMinY, maxCenterY));
    leadEmaState[slot] = { fx: path_x, fy: path_y, fw: width, trackId: currentTrackId, lastMs: nowMs };
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
      videoWidth,
      videoHeight,
    };
  }

  function projectLeadTwoBox(lead, modelPath, calibTransform, videoWidth, videoHeight, stageWidth = videoWidth, stageHeight = videoHeight, transform = null) {
    if (!lead?.status || !lead?.radar) return null;
    const distance = finiteNumber(lead.dRel, NaN);
    if (!Number.isFinite(distance) || distance <= 0) return null;

    const yCenter = -finiteNumber(lead.yRel, 0);
    const z = roadOverlayProjection.samplePathZ(modelPath, distance) + PATH_Z_OFFSET;
    const left = projectPointPrecise(calibTransform, distance, yCenter - 1.2, z);
    const right = projectPointPrecise(calibTransform, distance, yCenter + 1.2, z);
    if (!left || !right) return null;

    const prev = leadTwoEmaState;
    const currentTrackId = (lead.radarTrackId != null) ? finiteNumber(lead.radarTrackId, -1) : -1;
    const hasPrev = prev.trackId === currentTrackId
      && Number.isFinite(prev.xl)
      && Number.isFinite(prev.xr)
      && Number.isFinite(prev.y);
    // C3 lead_two uses alpha=0.8 at 20Hz; apply same time-compensation
    const nowMs2 = getLeadTemporalNowMs();
    const dt2 = prev.lastMs > 0 ? clamp(nowMs2 - prev.lastMs, 0, 500) : C3_FRAME_MS;
    const a2 = dt2 > 0 ? Math.pow(0.8, dt2 / C3_FRAME_MS) : 1;
    const xl = hasPrev ? (prev.xl * a2 + left.x * (1 - a2)) : left.x;
    const xr = hasPrev ? (prev.xr * a2 + right.x * (1 - a2)) : right.x;
    const y = hasPrev ? (prev.y * a2 + left.y * (1 - a2)) : left.y;
    leadTwoEmaState = { xl, xr, y, trackId: currentTrackId, lastMs: nowMs2 };

    const { minCenterX, maxCenterX, topMargin, maxCenterY, visibleRect } = getLeadBoxClampMargins(
      videoWidth,
      videoHeight,
      stageWidth,
      stageHeight,
      transform,
      { includeDistanceBadge: false, includeStateText: false },
    );
    const clampedCenterX = clamp((xl + xr) * 0.5, minCenterX, maxCenterX);
    const rawWidth = Math.max(xr - xl, 1);
    const uiScale = getLeadUiScale(videoWidth, videoHeight);
    const projectionScale = getLeadProjectionScale(videoWidth, videoHeight);
    const sidePad = Math.max(10 * uiScale, 5);
    const horizontalWidthLimit = Math.max(
      1,
      2 * Math.max(0, Math.min(clampedCenterX - visibleRect.left, visibleRect.right - clampedCenterX) - sidePad),
    );
    const verticalWidthLimit = Math.max(1, (maxCenterY - visibleRect.top) / 0.8);
    const maxWidth = Math.max(1, Math.min(800 * projectionScale, horizontalWidthLimit, verticalWidthLimit));
    const minWidth = Math.min(120 * projectionScale, maxWidth);
    const width = Math.max(Math.trunc(clamp(rawWidth, minWidth, maxWidth)), 1);
    const height = Math.max(Math.trunc(width * 0.8), Math.round(12 * uiScale));
    const safeMinX = visibleRect.left + sidePad + width * 0.5;
    const safeMaxX = visibleRect.right - sidePad - width * 0.5;
    const centerX = safeMaxX >= safeMinX
      ? clamp(clampedCenterX, safeMinX, safeMaxX)
      : visibleRect.left + visibleRect.width * 0.5;
    const yInt = Math.trunc(clamp(y, Math.min(maxCenterY, Math.max(topMargin, visibleRect.top + height)), maxCenterY));
    const xlInt = Math.trunc(centerX - width * 0.5);
    return {
      rect: {
        x: xlInt - sidePad,
        y: yInt - height,
        width: width + sidePad * 2,
        height,
      },
      dRel: distance,
      videoWidth,
      videoHeight,
    };
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
    if (canvasCtx === ctx) {
      /* Bounds before the glyphs are measured, so be generous: the outline
       * stroke, alignment and baseline all move the ink around the anchor. An
       * over-estimate only clears a few extra pixels; an under-estimate leaves
       * a trail. */
      const width = getCachedTextWidth(canvasCtx, `${fontWeight} ${fontSize}px ${HUD_TEXT_FONT}`, String(text));
      const pad = strokeWidth + OVERLAY_DIRTY_PAD;
      const left = align === "left" ? x : align === "right" ? x - width : x - width / 2;
      const top = baseline === "top" ? y : baseline === "bottom" ? y - fontSize : y - fontSize;
      markOverlayDirtyBox(left - pad, top - pad, left + width + pad, top + fontSize * 2 + pad);
    }
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

  const roadOverlayAuxRenderer = window.DriveVisionRoadOverlayAuxRenderer?.create?.({
    getParams: () => paramsState,
    pathZOffset: PATH_Z_OFFSET,
    geometry: {
      buildVerticalRibbon: roadOverlayProjection.buildVerticalRibbon,
      drawPolygon: drawGeometryPolygon,
      samplePathY: roadOverlayProjection.samplePathY,
      interpolate: roadOverlayProjection.interpolate,
      projectPoint,
      drawPolyline: drawGeometryPolyline,
    },
    ui: {
      getScale: getLeadUiScale,
      displayDistance: displayDistanceMeters,
      clampTextAnchor,
      drawText: drawCanvasOutlinedText,
    },
  }) || null;
  if (!roadOverlayAuxRenderer) return {};

  function projectPathEndAnchorBox(modelPath, calibTransform, videoWidth, videoHeight) {
    const xs = Array.isArray(modelPath?.x) ? modelPath.x : [];
    const ys = Array.isArray(modelPath?.y) ? modelPath.y : [];
    const zs = Array.isArray(modelPath?.z) ? modelPath.z : [];
    if (!xs.length || !ys.length || !zs.length) return null;

    const tailDistance = clamp(finiteNumber(xs[xs.length - 1], 0), MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);
    const idx = roadOverlayProjection.getPathLengthIndex(modelPath, tailDistance);
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
    const uiScale = getLeadUiScale(videoWidth, videoHeight);
    const width = clamp(rawWidth, 120 * uiScale, 800 * uiScale);
    const centerX = clamp(rawCenterX, marginX, Math.max(marginX, videoWidth - marginX));
    const centerY = clamp(rawCenterY, topMargin, Math.max(topMargin, videoHeight - bottomMargin));
    return {
      centerX: Math.trunc(centerX),
      centerY: Math.trunc(centerY),
      width: Math.trunc(width),
    };
  }

  roadOverlayLeadRenderer = window.DriveVisionRoadOverlayLeadRenderer?.create?.({
    context: ctx,
    model: roadOverlayLeadModel,
    getParams: () => paramsState,
    isMetricDisplay,
    displayDistance: displayDistanceMeters,
    baseCamera: BASE_CAMERA,
    fontFamily: HUD_TEXT_FONT,
    projection: {
      projectLeadBox,
      projectLeadTwoBox,
      projectPathEndAnchorBox,
      samplePathZ: roadOverlayProjection.samplePathZ,
      projectPointPrecise,
    },
    geometry: {
      drawPolyline: drawGeometryPolyline,
      drawPolygon: drawGeometryPolygon,
      circlePolygon: roadOverlayProjection.circlePolygon,
    },
    ui: {
      getScale: getLeadUiScale,
      getBadgeOffsets: getLeadBadgeOffsets,
      getVisibleRect: getVisibleSourceRect,
      fillRoundedRect,
      strokeRoundedRect,
      drawText: drawCanvasOutlinedText,
    },
    temporal: {
      now: getLeadTemporalNowMs,
      resetLeadSlot: resetLeadEmaSlot,
      resetLeadTwo: resetLeadTwoEma,
    },
  }) || null;
  if (!roadOverlayLeadRenderer) return {};

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
    if (context === ctx) {
      markOverlayDirtyBox(x - OVERLAY_DIRTY_PAD, y - OVERLAY_DIRTY_PAD,
        x + width + OVERLAY_DIRTY_PAD, y + height + OVERLAY_DIRTY_PAD);
    }
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
    if (context === ctx) {
      const pad = (Number(lineWidth) || 1) / 2 + OVERLAY_DIRTY_PAD;
      markOverlayDirtyBox(x - pad, y - pad, x + width + pad, y + height + pad);
    }
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
    return document.body?.dataset?.page === "carrot"
      && window.CarrotVisionContentRuntime?.isActive?.() !== false;
  }

  function renderActiveFrame(options = {}) {
    refreshParams();
    const renderViewport = replayRenderBridge.getRenderViewport();
    const stageWidth = Math.max(1, renderViewport.clientWidth);
    const stageHeight = Math.max(1, renderViewport.clientHeight);
    const viewportOrigin = getRenderViewportOrigin(renderViewport);
    const forceAll = Boolean(options.force || _forceNextRender);
    const rawOverlayState = window.CarrotOverlayState || {};
    const rawHudState = window.CarrotHudState || {};
    const runtimeState = window.CarrotDriveLiveStateProvider?.snapshot?.() || {
      hudState: rawHudState,
      overlayState: rawOverlayState,
    };
    let overlayState = runtimeState.overlayState;
    const synchronizedModel = window.CarrotVisionFrameSync?.selectModel?.();
    if (synchronizedModel && synchronizedModel !== overlayState?.modelV2) {
      overlayState = { ...overlayState, modelV2: synchronizedModel };
    }
    const hudState = runtimeState.hudState;
    if (!isCarrotVisionActive()) {
      if (forceAll || _lastOverlaySig !== "vision-disabled" || _lastHudSig !== "vision-disabled") {
        _lastOverlaySig = "vision-disabled";
        _lastHudSig = "vision-disabled";
        _lastPlotInputSig = "off";
        cancelCameraFrameRecheck();
        roadOverlayProjection.resetTemporal();
        hideOnroadAlert();
        setStageLoading(false);
        setStageReady(false);
        clearOverlay(canvasEl.width || 1, canvasEl.height || 1, { clearPerformance: true, full: true });
        hudCanvas.clear(hudCanvasEl.width || 1, hudCanvasEl.height || 1);
        rtcPerfHud.sync();
      }
      return;
    }

    const hasStream = syncSourceStream();
    const roadFrameRenderable = isRoadCameraFrameRenderable(videoEl);
    const holdReplayFrame = Boolean(
      hasStream
      && !roadFrameRenderable
      && replayRenderBridge.shouldHoldFrameDuringSeek()
    );
    if (holdReplayFrame) {
      setStageLoading(false);
      setStageReady(true);
      scheduleCameraFrameRecheck();
      return;
    }
    if (!hasStream || !roadFrameRenderable) {
      if (hasStream) {
        setCarrotVisionRenderPhase("first-frame-waiting", { reason: "camera stream waiting first frame" });
      }
      _lastOverlaySig = "";
      _lastHudSig = "";
      roadOverlayProjection.resetTemporal();
      hideOnroadAlert();
      setStageLoading(true, getCarrotVisionStatusText(getUIText("connecting", "Connecting...")), getCarrotVisionDetailText());
      setStageReady(false);
      clearOverlay(canvasEl.width || 1, canvasEl.height || 1, { clearPerformance: true, full: true });
      hudCanvas.clear(hudCanvasEl.width || 1, hudCanvasEl.height || 1);
      visionViewport.apply({
        renderViewport,
        renderWidth: stageWidth,
        renderHeight: stageHeight,
        viewportRect: {
          left: 0,
          top: 0,
          right: stageWidth,
          bottom: stageHeight,
        },
      });
      rtcPerfHud.sync();
      scheduleCameraFrameRecheck();
      return;
    }
    cancelCameraFrameRecheck();

    const videoWidth = videoEl.videoWidth;
    const videoHeight = videoEl.videoHeight;
    syncCanvasSize(videoWidth, videoHeight, stageWidth, stageHeight);
    renderOnroadAlert(stageWidth, stageHeight, hudState?.selfdriveState);
    // Use raw radar state directly — no interpolation (matches C3/CarrotLink).
    // C3 reads SubMaster every frame; CarrotLink reads snapshot directly.
    // Position smoothing is handled in projectLeadBox() via EMA.
    const model = overlayState.modelV2 || null;
    const liveCalibration = overlayState.liveCalibration || null;
    const roadCameraState = overlayState.roadCameraState || null;
    const cameraProfileReady = applyRoadCameraProfile(hudState?.deviceState?.deviceType, roadCameraState?.sensor);
    const { selectedPath, pathStyle } = roadOverlayPolicy.resolve(overlayState, hudState, paramsState);
    const plotData = buildPlotData(overlayState, hudState);
    const showLaneInfo = finiteNumber(paramsState.ShowLaneInfo, defaultParams.ShowLaneInfo);
    const overlaySig = overlayDataSignature(hudState, overlayState, selectedPath, pathStyle, showLaneInfo);
    // C3 pushes plot data EVERY frame unconditionally (drawPlot.draw→updatePlotQueue).
    // Web must do the same for identical density — no signature gating.
    updatePlotHistory(plotData);
    const nextPlotSig = plotInputSignature(plotData);
    const plotChanged = forceAll || Boolean(options.hudDirty) || nextPlotSig !== _lastPlotInputSig;
    if (plotChanged) {
      _lastPlotInputSig = nextPlotSig;
    }
    const hudSig = hudDataSignature(hudState, overlayState, plotData);
    const overlayDirty = forceAll || Boolean(options.overlayDirty) || overlaySig !== _lastOverlaySig;
    const hudDirty = forceAll || Boolean(options.hudDirty) || plotChanged || hudSig !== _lastHudSig;
    if (!overlayDirty && !hudDirty) return;
    if (overlayDirty) _lastOverlaySig = overlaySig;
    if (hudDirty) _lastHudSig = hudSig;
    _forceNextRender = false;

    const calibration = getCalibrationMatrix(liveCalibration);
    const projectionReady = Boolean(cameraProfileReady && calibration);
    const transform = getStageTransform(videoWidth, videoHeight, stageWidth, stageHeight, calibration || VIEW_FROM_DEVICE);
    applyStageTransform(transform);
    // AR 오버레이는 독립 canvas 를 쓰지만 좌표계는 반드시 같아야 한다. 스스로
    // 다시 계산하면 표지가 차선/경로 오버레이와 미세하게 어긋난다. 읽기 전용
    // 스냅샷만 노출하고 렌더 경로는 건드리지 않는다.
    window.CarrotVisionStageTransform = Object.freeze({
      calibTransform: transform.calibTransform,
      scale: transform.scale,
      tx: transform.tx,
      ty: transform.ty,
      videoWidth,
      videoHeight,
      stageWidth,
      stageHeight,
      viewportLeft: viewportOrigin.left,
      viewportTop: viewportOrigin.top,
      projectionReady,
      updatedAt: performance.now(),
    });
    const viewport = visionViewport.apply({
      renderViewport,
      renderWidth: stageWidth,
      renderHeight: stageHeight,
      sourceWidth: videoWidth,
      sourceHeight: videoHeight,
      transform,
    });
    const viewportRect = viewport.local;
    setStageReady(true);
    setCarrotVisionRenderPhase("ready", { reason: "camera frame renderable" });
    // The <video> may hold a renderable-but-stale last frame during a reconnect.
    // vision_rtc only promotes to controlState "live" when the connection is
    // genuinely up, so key the loading panel on that: live -> hide it; otherwise
    // keep a "reconnecting/connecting" message over the frozen frame instead of
    // a silent blank.
    const carrotLive = replayRenderBridge.isActive()
      ? replayRenderBridge.isReady()
      : (getCarrotVisionState().controlState || "") === "live";
    setStageLoading(!carrotLive, getCarrotVisionStatusText(getUIText("connecting", "Connecting...")), getCarrotVisionDetailText());

    if (overlayDirty) {
      roadOverlayProjection.resetFrame(performance.now());
      clearOverlay(videoWidth, videoHeight);
      beginGeometryFrame(videoWidth, videoHeight);
      if (model && projectionReady) {
        if (showLaneInfo >= 1) {
          roadOverlayGeometryRenderer.drawLaneLines(model, overlayState, hudState, transform.calibTransform);
        }
        if (showLaneInfo > 1) {
          roadOverlayGeometryRenderer.drawRoadEdges(model, overlayState, transform.calibTransform);
        }
        if (showLaneInfo >= 0) {
          roadOverlayGeometryRenderer.drawPath(
            selectedPath.pathData,
            model,
            overlayState,
            transform.calibTransform,
            videoHeight,
            pathStyle,
          );
        }
        roadOverlayAuxRenderer.drawBlindspotBarriers(
          model?.position,
          overlayState,
          hudState,
          transform.calibTransform,
        );
        roadOverlayLeadRenderer.draw(
          model,
          overlayState,
          hudState,
          transform.calibTransform,
          videoWidth,
          videoHeight,
          stageWidth,
          stageHeight,
          transform,
        );
        roadOverlayAuxRenderer.drawProjectedTfMarker(
          model?.position,
          hudState?.longitudinalPlan,
          transform.calibTransform,
          videoWidth,
          videoHeight,
        );
      }
      endGeometryFrame();
    }

    if (hudDirty) {
      hudCanvas.clear(stageWidth, stageHeight);
      drawPlot(stageWidth, stageHeight, viewportRect, plotData);
      rtcPerfHud.sync();
    }
  }

  function cancelScheduledRender() {
    cancelCameraFrameRecheck();
    if (_renderRafId != null) {
      window.cancelAnimationFrame(_renderRafId);
      _renderRafId = null;
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
    _renderVideoFrameId = null;
    if (!isStageVisible()) {
      if (!isActive()) visionViewport.reset();
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

  /* Replay is deliberately included here: live and replay are the same user
   * experience and must share one cadence. The callback below already skips the
   * live-only presented-frame reporting when the replay bridge is active, and a
   * paused or unready video still falls through to the display-clock path, so a
   * seek or scrub keeps updating. A forced render (activation, resize) is the
   * one case that must not wait for the next decoded frame. */
  function canUseVideoFrameScheduling() {
    return (
      _pendingRenderState.overlayDirty &&
      !_pendingRenderState.force &&
      typeof videoEl.requestVideoFrameCallback === "function" &&
      isCarrotVisionActive() &&
      !videoEl.paused &&
      videoEl.readyState >= 2
    );
  }

  function scheduleRender(options = {}) {
    mergePendingRenderState(options);
    if (!isStageVisible()) {
      cancelScheduledRender();
      if (!isActive()) visionViewport.reset();
      return;
    }

    if (_renderRafId != null || _renderVideoFrameId != null) return;

    // The video frame callback is already paced by the stream, so the interval
    // throttle can only push the overlay off the frame it belongs to. Take this
    // path first and unthrottled: one overlay update per presented video frame
    // is exactly the cadence the user should see.
    if (canUseVideoFrameScheduling()) {
      _renderVideoFrameId = videoEl.requestVideoFrameCallback((_now, metadata) => {
        _renderVideoFrameId = null;
        if (!replayRenderBridge.isActive()) {
          window.CarrotVisionFrameSync?.notePresentedVideoFrame?.(metadata);
          window.CarrotVisionRtc?.reportPresentedFrame?.();
        }
        flushScheduledRender();
      });
      return;
    }

    // Otherwise pace on the display clock. A timer lands between composited
    // frames and adds visible judder on top of the source cadence, so hold the
    // interval by re-arming rAF instead of sleeping on setTimeout.
    _renderRafId = window.requestAnimationFrame(() => {
      _renderRafId = null;
      if (performance.now() - _lastRenderTime < RENDER_INTERVAL_MS) {
        scheduleRender();
        return;
      }
      flushScheduledRender();
    });
  }

  function requestRender(options = {}) {
    const hasOverlayDirty = Object.prototype.hasOwnProperty.call(options, "overlayDirty");
    const hasHudDirty = Object.prototype.hasOwnProperty.call(options, "hudDirty");
    const normalized = {
      force: Boolean(options.force),
      overlayDirty: Boolean(options.force || (hasOverlayDirty ? options.overlayDirty : true)),
      hudDirty: Boolean(options.force || (hasHudDirty ? options.hudDirty : true)),
    };
    if (stageEl.closest(".drive-workspace")?.classList.contains("is-drive-workspace-resizing")) {
      mergePendingRenderState(normalized);
      cancelScheduledRender();
      return;
    }
    scheduleRender(normalized);
  }

  const replayRenderController = window.DriveVisionReplayRenderController?.create?.({
    isReplayActive: replayRenderBridge.isActive,
    isStageVisible,
    cancelScheduledRender,
    resetFrameTemporalState: resetVisualTemporalState,
    resetReplayState: resetReplayVisualState,
    mergePendingRenderState,
    flushScheduledRender,
    notifyPresentedFrame: (metadata) => (
      window.CarrotVisionFrameSync?.noteReplayPresentedFrame?.(metadata)
    ),
  }) || null;
  if (!replayRenderController) return {};

  function refresh() {
    transformSignature = "";
    overlaySizeSignature = "";
    hudSizeSignature = "";
    _lastOverlaySig = "";
    _lastHudSig = "";
    _lastPlotInputSig = "";
    _forceNextRender = true;
    _gradientCache.clear();
    hudCanvas.reset();
    _rgbaCache.clear();
    resetVisualTemporalState();
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
    if (target.closest(".c-state-surface")) return true;
    return false;
  }
  window.addEventListener("carrot:websettingschange", syncDisplayModeFromServer);

  async function handleStageFullscreenToggle(event) {
    if (!isCarrotVisionActive()) return;
    if (replayRenderBridge.isActive()) return;
    if (shouldIgnoreStageFullscreenToggle(event?.target)) return;
    if (typeof window.ToggleCarrotFullscreen !== "function") return;
    await window.ToggleCarrotFullscreen({ quiet: false }).catch(() => {});
  }

  function requestFullRender() {
    if (stageEl.closest(".drive-workspace")?.classList.contains("is-drive-workspace-resizing")) {
      mergePendingRenderState({ force: true, overlayDirty: true, hudDirty: true });
      cancelScheduledRender();
      return;
    }
    rtcPerfHud.layoutChanged();
    refresh();
    requestRender({ force: true, overlayDirty: true, hudDirty: true });
  }

  function handleLifecycleChange() {
    if (!isActive()) rtcPerfHud.close();
    if (isStageVisible()) {
      if (isCarrotVisionActive()) {
        const live = (getCarrotVisionState().controlState || "") === "live";
        setStageLoading(!live, getCarrotVisionStatusText(getUIText("connecting", "Connecting...")), getCarrotVisionDetailText());
      }
      requestFullRender();
      return;
    }
    cancelScheduledRender();
    setStageLoading(false);
    if (!isActive()) visionViewport.reset();
  }

  stageEl.addEventListener("click", handleStageFullscreenToggle);
  window.addEventListener("resize", requestFullRender);
  window.addEventListener("orientationchange", requestFullRender);
  document.addEventListener("fullscreenchange", requestFullRender);
  document.addEventListener("webkitfullscreenchange", requestFullRender);
  window.addEventListener("carrot:render-request", (event) => requestRender(event.detail || {}));
  window.addEventListener("carrot:pagechange", handleLifecycleChange);
  window.addEventListener("carrot:visionchange", handleLifecycleChange);
  window.addEventListener("carrot:visionstatechange", handleLifecycleChange);
  window.addEventListener("carrot:visioncontentchange", handleLifecycleChange);
  window.addEventListener("drive:workspacelayoutchange", requestFullRender);
  window.addEventListener("drive:workspaceresizestart", cancelScheduledRender);
  window.addEventListener("drive:workspaceresizeend", requestFullRender);
  document.addEventListener("visibilitychange", handleLifecycleChange);
  if (typeof ResizeObserver === "function") {
    const stageResizeObserver = new ResizeObserver(requestFullRender);
    stageResizeObserver.observe(stageEl);
    if (replayViewportEl) stageResizeObserver.observe(replayViewportEl);
  }
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", requestFullRender, { passive: true });
    window.visualViewport.addEventListener("scroll", requestFullRender, { passive: true });
  }
  const renderVideoTargets = sourceVideoEl === videoEl ? [videoEl] : [sourceVideoEl, videoEl];
  ["loadedmetadata", "loadeddata", "playing", "resize", "emptied"].forEach((eventName) => {
    renderVideoTargets.forEach((target) => target.addEventListener(eventName, requestFullRender));
  });

  syncDisplayModeButtons();
  rtcPerfHud.sync();
  refreshParams(true);
  requestRender({ force: true, overlayDirty: true, hudDirty: true });

  return {
    refresh,
    requestRender,
    resize: requestFullRender,
    renderReplayVideoFrame: replayRenderController.renderVideoFrame,
    resetReplayTemporalState: replayRenderController.resetTemporalState,
    renderText: syncDisplayModeButtons,
    getDisplayModeIndex: () => displayModeIndex,
    setDisplayModeIndex,
  };
})();

if (typeof window.HomeDrive?.refresh === "function") {
  window.dispatchEvent(new CustomEvent("carrot:driveplatformready"));
}
