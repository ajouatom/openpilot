export const ONNX_VISION_STATE = Object.freeze({
  DISABLED: "disabled",
  STARTING: "starting",
  UNAVAILABLE: "unavailable",
  WAITING: "waiting",
  READY: "ready",
  ATTENTION: "attention",
});

function object(value) {
  return value && typeof value === "object" ? value : {};
}

function number(value) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function confidence(value) {
  const parsed = number(value);
  return parsed === null ? 0 : Math.min(Math.max(parsed, 0), 1);
}

function laneLine(value) {
  if (value === 1) return "solid";
  if (value === 0) return "dashed";
  return "unknown";
}

function emptyStatus(state, error = "") {
  return Object.freeze({
    state,
    error: String(error || ""),
    raw: Object.freeze({}),
    service: null,
    lane: Object.freeze({
      cameraReady: false,
      cameraError: "",
      cameraAgeSeconds: null,
      left: Object.freeze({ line: "unknown", confidence: 0 }),
      right: Object.freeze({ line: "unknown", confidence: 0 }),
      fresh: false,
      candidates: 0,
      fps: null,
      latencyMs: null,
      threadCpuMs: null,
      count: 0,
      lastAgeSeconds: null,
    }),
    bsd: Object.freeze({
      cameraReady: false,
      cameraError: "",
      cameraAgeSeconds: null,
      active: false,
      side: "",
      configured: false,
      configuredSides: Object.freeze([]),
      gateReason: "",
      laneWidth: null,
      left: Object.freeze({ valid: false, active: false, confidence: 0 }),
      right: Object.freeze({ valid: false, active: false, confidence: 0 }),
      fps: null,
      latencyMs: null,
      threadCpuMs: null,
      count: 0,
      lastAgeSeconds: null,
    }),
  });
}

export function normalizeOnnxVisionStatus(payload, options = {}) {
  if (options.enabled !== true) return emptyStatus(ONNX_VISION_STATE.DISABLED);
  if (options.loading === true) return emptyStatus(ONNX_VISION_STATE.STARTING);
  if (options.error) {
    return emptyStatus(
      ONNX_VISION_STATE.UNAVAILABLE,
      options.error?.message || options.error?.code || String(options.error),
    );
  }

  const status = object(payload);
  const model = object(status.model);
  const camera = object(status.camera);
  const lanePayload = object(status.lane);
  const laneResult = object(lanePayload.result);
  const laneInference = object(lanePayload.inference);
  const bsdInference = object(status.inference);
  const gate = object(status.gate);
  const sides = object(status.vehicleSide || status.imageSide);
  const leftBsd = object(sides.left);
  const rightBsd = object(sides.right);
  const configuredSides = Array.isArray(status.configuredSides)
    ? status.configuredSides.map((side) => String(side)).filter((side) => side === "left" || side === "right")
    : [];
  const laneFresh = lanePayload.resultFresh === true;
  const laneModelReady = lanePayload.loaded === true;
  // Older services have no independent BSD switch.
  const bsdEnabled = status.bsdEnabled !== false;
  const bsdModelReady = model.loaded === true;
  const laneCameraReady = lanePayload.cameraAvailable === true;
  const bsdCameraReady = camera.available === true;

  const lane = Object.freeze({
    cameraReady: laneCameraReady,
    cameraError: String(lanePayload.cameraError || ""),
    cameraAgeSeconds: number(lanePayload.lastFrameAgeSeconds),
    left: Object.freeze({
      line: laneFresh ? laneLine(laneResult.leftLine) : "unknown",
      confidence: laneFresh ? confidence(laneResult.leftConf) : 0,
    }),
    right: Object.freeze({
      line: laneFresh ? laneLine(laneResult.rightLine) : "unknown",
      confidence: laneFresh ? confidence(laneResult.rightConf) : 0,
    }),
    fresh: laneFresh,
    candidates: Math.max(0, Math.round(number(laneResult.candidatesCount) || 0)),
    fps: number(laneInference.fps),
    latencyMs: number(laneInference.latencyMs),
    threadCpuMs: number(laneInference.threadCpuMs),
    count: Math.max(0, Math.round(number(laneInference.count) || 0)),
    lastAgeSeconds: number(laneInference.lastAgeSeconds),
  });

  const bsd = Object.freeze({
    enabled: bsdEnabled,
    cameraReady: bsdCameraReady,
    cameraError: String(camera.error || ""),
    cameraAgeSeconds: number(camera.lastFrameAgeSeconds),
    active: bsdEnabled && gate.active === true,
    side: gate.side === "left" || gate.side === "right" ? gate.side : "",
    configured: configuredSides.length > 0,
    configuredSides: Object.freeze(configuredSides),
    gateReason: String(gate.reason || ""),
    laneWidth: number(gate.laneWidth),
    left: Object.freeze({
      valid: bsdEnabled && leftBsd.valid === true,
      active: bsdEnabled && leftBsd.active === true,
      confidence: bsdEnabled ? confidence(leftBsd.confidence) : 0,
    }),
    right: Object.freeze({
      valid: bsdEnabled && rightBsd.valid === true,
      active: bsdEnabled && rightBsd.active === true,
      confidence: bsdEnabled ? confidence(rightBsd.confidence) : 0,
    }),
    fps: number(bsdInference.fps),
    latencyMs: number(bsdInference.latencyMs),
    threadCpuMs: number(bsdInference.threadCpuMs),
    count: Math.max(0, Math.round(number(bsdInference.count) || 0)),
    lastAgeSeconds: number(bsdInference.lastAgeSeconds),
  });

  const modelsReady = laneModelReady && (!bsdEnabled || bsdModelReady);
  const state = !modelsReady
    ? ONNX_VISION_STATE.ATTENTION
    : !(laneCameraReady && (!bsdEnabled || bsdCameraReady))
      ? ONNX_VISION_STATE.WAITING
      : ONNX_VISION_STATE.READY;

  return Object.freeze({
    state,
    error: "",
    raw: status,
    service: Object.freeze({
      modelsReady,
      bsdModelReady,
      laneModelReady,
      bsdModelPath: String(model.path || ""),
      bsdModelError: String(model.error || ""),
      laneModelError: String(lanePayload.error || ""),
    }),
    lane,
    bsd,
  });
}
