/* Carrot Vision compact HUD/overlay runtime.
 * Owns one compact websocket, latest display state, and HUD normalization.
 */
var setCarrotVisionState = window.CarrotVisionSetState;

let LAST_HUD_PAYLOAD_SIGNATURE = "";
const COMPACT_FIRST_DATA_TIMEOUT_MS = 8000;

const RAW_HUD_SERVICES = window.CarrotVisionCompact?.HUD_SERVICES || [];
const RAW_HUD_STATE = Object.create(null);
window.CarrotHudState = RAW_HUD_STATE;
const RAW_OVERLAY_SERVICES = window.CarrotVisionCompact?.OVERLAY_SERVICES || [];
const RAW_OVERLAY_STATE = Object.create(null);
window.CarrotOverlayState = RAW_OVERLAY_STATE;
const RAW_STATE_UPDATED_AT = Object.create(null);
window.CarrotStateUpdatedAt = RAW_STATE_UPDATED_AT;
const RAW_STATE_RECEIVED_AT_MONOTONIC = Object.create(null);
window.CarrotStateReceivedAtMonotonic = RAW_STATE_RECEIVED_AT_MONOTONIC;
const RAW_OVERLAY_HUD_ONLY_SERVICES = new Set(["roadCameraState", "liveDelay", "liveTorqueParameters", "liveParameters"]);
const COMPACT_STATE_MODE = "carrot-state-v1";
let COMPACT_OVERLAY_WS = null;
let COMPACT_OVERLAY_RETRY_T = null;
let COMPACT_OVERLAY_READY = false;
let COMPACT_STATE_SERVICE_SIGNATURE = "";
let COMPACT_OVERLAY_REQUESTED = false;
let COMPACT_FIRST_DATA_T = null;
const MODEL_FRAME_HISTORY = [];
const MODEL_FRAME_HISTORY_LIMIT = 64;
const RTP_FRAME_HISTORY = new Map();
const RTP_FRAME_HISTORY_LIMIT = 96;
let PRESENTED_VIDEO_FRAME_ID = null;
let LAST_PRESENTED_RTP_TIMESTAMP = null;

function rememberModelFrame(model) {
  const frameId = Number(model?.frameId);
  if (!Number.isFinite(frameId)) return;
  const previous = MODEL_FRAME_HISTORY[MODEL_FRAME_HISTORY.length - 1];
  if (Number(previous?.frameId) === frameId) {
    MODEL_FRAME_HISTORY[MODEL_FRAME_HISTORY.length - 1] = model;
  } else {
    MODEL_FRAME_HISTORY.push(model);
    if (MODEL_FRAME_HISTORY.length > MODEL_FRAME_HISTORY_LIMIT) MODEL_FRAME_HISTORY.shift();
  }
}

function notePresentedVideoFrame(metadata) {
  const rawRtpTimestamp = Number(metadata?.rtpTimestamp);
  const rtpTimestamp = Number.isFinite(rawRtpTimestamp) ? (rawRtpTimestamp >>> 0) : null;
  LAST_PRESENTED_RTP_TIMESTAMP = rtpTimestamp;
  if (rtpTimestamp != null && RTP_FRAME_HISTORY.has(rtpTimestamp)) {
    PRESENTED_VIDEO_FRAME_ID = RTP_FRAME_HISTORY.get(rtpTimestamp);
    return;
  }

  // requestVideoFrameCallback.rtpTimestamp is optional. mediaTime remains a
  // guarded compatibility fallback for browsers which do not expose it.
  const mediaTime = Number(metadata?.mediaTime);
  const roadFrameId = Number(RAW_OVERLAY_STATE?.roadCameraState?.frameId);
  const inferred = Math.round(mediaTime / 0.05);
  PRESENTED_VIDEO_FRAME_ID = Number.isFinite(inferred)
    && Number.isFinite(roadFrameId)
    && Math.abs(roadFrameId - inferred) <= 120
    ? inferred
    : null;
}

function noteRtpFrameMapping(rtpTimestampValue, frameIdValue) {
  const rtpTimestamp = Number(rtpTimestampValue);
  const frameId = Number(frameIdValue);
  if (!Number.isFinite(rtpTimestamp) || !Number.isFinite(frameId)) return;

  const normalizedTimestamp = rtpTimestamp >>> 0;
  RTP_FRAME_HISTORY.set(normalizedTimestamp, frameId >>> 0);
  while (RTP_FRAME_HISTORY.size > RTP_FRAME_HISTORY_LIMIT) {
    RTP_FRAME_HISTORY.delete(RTP_FRAME_HISTORY.keys().next().value);
  }

  if (LAST_PRESENTED_RTP_TIMESTAMP === normalizedTimestamp) {
    PRESENTED_VIDEO_FRAME_ID = frameId >>> 0;
    emitCarrotRenderRequest({ overlayDirty: true, hudDirty: false });
  }
}

function selectSynchronizedModel() {
  if (!MODEL_FRAME_HISTORY.length) return RAW_OVERLAY_STATE?.modelV2 || null;
  if (!Number.isFinite(PRESENTED_VIDEO_FRAME_ID)) {
    return MODEL_FRAME_HISTORY[MODEL_FRAME_HISTORY.length - 1];
  }
  const target = PRESENTED_VIDEO_FRAME_ID;

  let selected = null;
  for (let index = MODEL_FRAME_HISTORY.length - 1; index >= 0; index -= 1) {
    const candidate = MODEL_FRAME_HISTORY[index];
    if (Number(candidate?.frameId) <= target) {
      selected = candidate;
      break;
    }
  }
  return selected || MODEL_FRAME_HISTORY[0];
}

function resetFrameSynchronization() {
  MODEL_FRAME_HISTORY.length = 0;
  RTP_FRAME_HISTORY.clear();
  PRESENTED_VIDEO_FRAME_ID = null;
  LAST_PRESENTED_RTP_TIMESTAMP = null;
}

window.CarrotVisionFrameSync = {
  notePresentedVideoFrame,
  noteRtpFrameMapping,
  reset: resetFrameSynchronization,
  selectModel: selectSynchronizedModel,
};

function clearCompactFirstDataTimer() {
  if (COMPACT_FIRST_DATA_T != null) {
    clearTimeout(COMPACT_FIRST_DATA_T);
    COMPACT_FIRST_DATA_T = null;
  }
}

function compactStateActive() {
  return COMPACT_OVERLAY_READY;
}

function compactDesiredServices() {
  const services = [];
  if (shouldRunCarrotHudData()) services.push(...RAW_HUD_SERVICES);
  if (
    (COMPACT_OVERLAY_REQUESTED && shouldRunCarrotVisionRealtime())
    || isCarrotDriveDataRequested("overlay")
  ) services.push(...RAW_OVERLAY_SERVICES);
  return Array.from(new Set(services));
}

function publishCompactState() {
  window.CarrotVisionCompactStateActive = compactStateActive();
}

function buildCompactStateUrl(services) {
  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  const params = new URLSearchParams();
  params.set("services", services.join(","));
  return `${wsProto}://${location.host}/ws/compact_state?${params.toString()}`;
}

function applyCompactFrame(service, decoded, options = {}) {
  if (!decoded || typeof decoded !== "object") return false;
  let applied = false;
  let hudDirty = false;
  let overlayDirty = false;
  RAW_STATE_UPDATED_AT[service] = Date.now();
  if (RAW_HUD_SERVICES.includes(service)) {
    RAW_HUD_STATE[service] = decoded;
    window.CarrotHudState = RAW_HUD_STATE;
    if (options.updateVisionState !== false) {
      setCarrotVisionState({ raw: { hud: "ready" } }, { reason: `compact hud ${service}`, silent: true });
    }
    if (options.markHud !== false) _hudMarkDirty();
    hudDirty = true;
    applied = true;
  }
  if (RAW_OVERLAY_SERVICES.includes(service)) {
    if (service === "modelV2") rememberModelFrame(decoded);
    RAW_OVERLAY_STATE[service] = decoded;
    window.CarrotOverlayState = RAW_OVERLAY_STATE;
    if (options.updateVisionState !== false) {
      setCarrotVisionState({ raw: { overlay: "ready" } }, { reason: `compact overlay ${service}`, silent: true });
    }
    overlayDirty = !RAW_OVERLAY_HUD_ONLY_SERVICES.has(service);
    applied = true;
  }
  if (applied) {
    try {
      const providerSnapshot = window.CarrotDriveLiveStateProvider?.noteServiceReceived?.(service);
      const receivedAt = Number(providerSnapshot?.receivedAtMonotonic?.[service]);
      if (Number.isFinite(receivedAt)) RAW_STATE_RECEIVED_AT_MONOTONIC[service] = receivedAt;
    } catch (error) {
      console.warn("[compact state] provider receipt rejected", error);
    }
  }
  if (applied && options.render !== false) {
    emitCarrotRenderRequest({
      hudDirty,
      overlayDirty,
    });
  }
  return applied;
}

function applyCompactFrames(frames, options = {}) {
  const list = Array.isArray(frames) ? frames : [];
  let applied = 0;
  let hudReady = false;
  let overlayReady = false;
  let overlayDirty = false;

  for (const frame of list) {
    const service = frame?.service;
    const isHud = RAW_HUD_SERVICES.includes(service);
    const isOverlay = RAW_OVERLAY_SERVICES.includes(service);
    if (!isHud && !isOverlay) continue;
    if (!applyCompactFrame(service, frame.decoded, {
      render: false,
      markHud: false,
      updateVisionState: false,
    })) continue;
    applied += 1;
    hudReady = hudReady || isHud;
    overlayReady = overlayReady || isOverlay;
    overlayDirty = overlayDirty || (isOverlay && !RAW_OVERLAY_HUD_ONLY_SERVICES.has(service));
  }

  if (hudReady) {
    if (options.flushHud === true) {
      _hudRenderDirty = false;
      drivingHudUpdateFromRawState();
    } else {
      _hudMarkDirty();
    }
  }
  if (hudReady || overlayReady) {
    setCarrotVisionState({
      raw: {
        ...(hudReady ? { hud: "ready" } : {}),
        ...(overlayReady ? { overlay: "ready" } : {}),
      },
    }, { reason: options.reason || "compact batch", silent: true });
  }
  if (applied && options.render !== false) {
    emitCarrotRenderRequest({ hudDirty: hudReady, overlayDirty });
  }
  return applied;
}

function clearCompactState(options = {}) {
  for (const key of Object.keys(RAW_HUD_STATE)) delete RAW_HUD_STATE[key];
  for (const key of Object.keys(RAW_OVERLAY_STATE)) delete RAW_OVERLAY_STATE[key];
  for (const key of Object.keys(RAW_STATE_UPDATED_AT)) delete RAW_STATE_UPDATED_AT[key];
  for (const key of Object.keys(RAW_STATE_RECEIVED_AT_MONOTONIC)) delete RAW_STATE_RECEIVED_AT_MONOTONIC[key];
  try {
    window.CarrotDriveLiveStateProvider?.noteServiceReceived?.("*", { clear: true });
  } catch (error) {
    console.warn("[compact state] provider clear rejected", error);
  }
  LAST_HUD_PAYLOAD_SIGNATURE = "";
  resetFrameSynchronization();
  setCarrotVisionState({ raw: { hud: "idle", overlay: "idle" } }, {
    reason: options.reason || "compact state cleared",
    silent: true,
  });
  if (options.render !== false) {
    emitCarrotRenderRequest({ force: true, hudDirty: true, overlayDirty: true });
  }
}

function drivingHudUpdateFromCarPayload(j) {
  if (!j) return;

  const runtimeIsMetric = window.CarrotLiveRuntimeState?.runtime?.params?.IsMetric;
  const isMetric = j.isMetric != null
    ? Boolean(Number(j.isMetric))
    : (runtimeIsMetric == null ? true : Boolean(Number(runtimeIsMetric)));
  const vEgoKph = (typeof j.vEgo === "number" && isFinite(j.vEgo)) ? j.vEgo * 3.6 : null;
  const payload = {
    isMetric,
    cpuTempC: j.cpuTempC,
    memPct: j.memPct,
    diskPct: j.diskPct,
    voltageV: j.voltageV,
    vEgoKph,
    vSetKph: j.vSetKph,
    temp: j.temp,
    redDot: j.redDot,
    tlight: j.tlight,
    tfGap: j.tfGap,
    tfBars: j.tfBars,
    gear: j.gear,
    gearStep: j.gearStep,
    lfaActive: j.lfaActive,
    steeringAngleDeg: j.steeringAngleDeg,
    aEgo: j.aEgo,
    steerOutput: j.steerOutput,
    leftBlinker: j.leftBlinker,
    rightBlinker: j.rightBlinker,
    fuelGauge: j.fuelGauge,
    ureaGauge: j.ureaGauge,
    tpms: j.tpms,
    trafficState: j.trafficState,
    gpsOk: j.gpsOk,
    driveMode: j.driveMode,
    speedLimitKph: j.speedLimitKph,
    speedLimitOver: j.speedLimitOver,
    speedLimitBlink: j.speedLimitBlink,
    apm: j.apm,
  };

  const miniHudPayload = window.CarrotMiniHudModel?.build?.(
    payload,
    RAW_HUD_STATE,
    window.CarrotLiveRuntimeState,
  );
  if (miniHudPayload) window.CarrotMiniHud?.update?.(miniHudPayload);

  const payloadSignature = [
    typeof LANG === "string" ? LANG : "",
    payload.isMetric ? 1 : 0,
    payload.cpuTempC ?? "-",
    payload.memPct ?? "-",
    payload.diskPct ?? "-",
    payload.voltageV ?? "-",
    payload.vEgoKph ?? "-",
    payload.vSetKph ?? "-",
    payload.temp?.source ?? "-",
    payload.temp?.speed ?? "-",
    payload.temp?.is_decel ? 1 : 0,
    payload.redDot ?? "-",
    payload.tlight ?? "-",
    payload.tfGap ?? "-",
    payload.tfBars ?? "-",
    payload.gear ?? "-",
    payload.gearStep ?? "-",
    payload.lfaActive ? 1 : 0,
    Math.round(Number(payload.steeringAngleDeg) || 0), // 1° 단위로만 갱신
    Math.round((Number(payload.aEgo) || 0) * 20),      // 0.05 단위
    Math.round((Number(payload.steerOutput) || 0) * 50), // 0.02 단위
    payload.leftBlinker ? 1 : 0,
    payload.rightBlinker ? 1 : 0,
    Math.round((Number(payload.fuelGauge) || 0) * 100),
    Math.round((Number(payload.ureaGauge) || 0) * 100),
    Math.round(Number(payload.tpms?.fl) || 0),
    Math.round(Number(payload.tpms?.fr) || 0),
    Math.round(Number(payload.tpms?.rl) || 0),
    Math.round(Number(payload.tpms?.rr) || 0),
    payload.trafficState ?? "-",
    payload.gpsOk ?? "-",
    payload.driveMode?.name ?? "-",
    payload.driveMode?.kind ?? "-",
    payload.speedLimitKph ?? "-",
    payload.speedLimitOver ? 1 : 0,
    payload.speedLimitBlink ? 1 : 0,
    payload.apm ?? "-",
  ].join("|");
  if (payloadSignature === LAST_HUD_PAYLOAD_SIGNATURE) return;
  LAST_HUD_PAYLOAD_SIGNATURE = payloadSignature;
  // Both live and replay publish the same normalized payload to independent
  // presentation sinks. Neither sink wraps or owns the other one's lifecycle.
  try { window.CarrotHudOverlay?.update?.(payload); } catch {}
  try { window.DriveVisionHudContent?.update?.(payload); } catch {}
}

function averageFiniteMetric(values) {
  const samples = Array.isArray(values)
    ? values.map((value) => Number(value)).filter((value) => Number.isFinite(value))
    : [];
  if (!samples.length) return null;
  const total = samples.reduce((sum, value) => sum + value, 0);
  return total / samples.length;
}

function pickFiniteMetric(...values) {
  for (const value of values) {
    if (value === null || value === undefined || value === "") continue;
    const numeric = Number(value);
    if (Number.isFinite(numeric)) return numeric;
  }
  return null;
}

function deriveNormalizedHudDeviceMetrics(rawHudState) {
  const replayActive = Boolean(window.CarrotVisionReplay?.isActive?.());
  const liveServices = replayActive ? {} : window.CarrotLiveRuntimeState?.services;
  const liveDeviceState = liveServices?.deviceState && typeof liveServices.deviceState === "object"
    ? liveServices.deviceState
    : {};
  const livePeripheralState = liveServices?.peripheralState && typeof liveServices.peripheralState === "object"
    ? liveServices.peripheralState
    : {};
  const rawDeviceState = rawHudState?.deviceState && typeof rawHudState.deviceState === "object"
    ? rawHudState.deviceState
    : {};
  const rawPeripheralState = rawHudState?.peripheralState && typeof rawHudState.peripheralState === "object"
    ? rawHudState.peripheralState
    : {};

  const cpuTempC = pickFiniteMetric(
    averageFiniteMetric(liveDeviceState.cpuTempC),
    averageFiniteMetric(rawDeviceState.cpuTempC),
  );
  const memPct = pickFiniteMetric(
    liveDeviceState.memoryUsagePercent,
    rawDeviceState.memoryUsagePercent,
  );
  const freeSpacePct = pickFiniteMetric(
    liveDeviceState.freeSpacePercent,
    rawDeviceState.freeSpacePercent,
  );
  const diskPct = Number.isFinite(freeSpacePct)
    ? Math.min(100, Math.max(0, 100 - freeSpacePct))
    : null;
  const voltageMv = pickFiniteMetric(
    livePeripheralState.voltage,
    rawPeripheralState.voltage,
  );
  const voltageV = Number.isFinite(voltageMv) ? (voltageMv / 1000.0) : null;

  return { cpuTempC, memPct, diskPct, voltageV };
}

function deriveNormalizedHudVehicleMetrics(rawHudState) {
  const replayActive = Boolean(window.CarrotVisionReplay?.isActive?.());
  const liveServices = replayActive ? {} : window.CarrotLiveRuntimeState?.services;
  const liveCarState = liveServices?.carState && typeof liveServices.carState === "object"
    ? liveServices.carState
    : {};
  const rawCarState = rawHudState?.carState && typeof rawHudState.carState === "object"
    ? rawHudState.carState
    : {};

  const gearStep = pickFiniteMetric(
    rawCarState.gearStep,
    liveCarState.gearStep,
  );

  return {
    gearStep: Number.isFinite(gearStep) && gearStep > 0 ? Math.round(gearStep) : null,
  };
}

function compactHudDriveMode(state) {
  const mode = Number(state?.longitudinalPlan?.myDrivingMode);
  if (mode === 1) return { name: "Eco", kind: "eco" };
  if (mode === 2) return { name: "Safe", kind: "safe" };
  if (mode === 4) return { name: "Sport", kind: "sport" };
  return { name: "Normal", kind: "normal" };
}

function compactHudGap(state) {
  const personality = Number(state?.selfdriveState?.personality);
  return Number.isFinite(personality) ? personality + 1 : null;
}

function compactHudSpeedLimitKph(state) {
  const xSpdLimit = Number(state?.carrotMan?.xSpdLimit);
  const xSpdType = Number(state?.carrotMan?.xSpdType);
  if (Number.isFinite(xSpdLimit) && xSpdLimit > 0 && xSpdType !== 22) return xSpdLimit;
  const roadLimit = Number(state?.carrotMan?.nRoadLimitSpeed);
  return Number.isFinite(roadLimit) && roadLimit > 0 ? roadLimit : null;
}

function compactHudTemp(state) {
  const desiredSpeed = Number(state?.carrotMan?.desiredSpeed);
  const vCruise = Number(state?.carState?.vCruiseCluster ?? state?.controlsState?.vCruiseCluster);
  if (!Number.isFinite(desiredSpeed)) return null;
  return {
    speed: desiredSpeed,
    source: Number.isFinite(vCruise) && desiredSpeed >= vCruise ? (state?.carrotMan?.desiredSource || "") : "",
    is_decel: Number.isFinite(vCruise) ? desiredSpeed < vCruise : false,
  };
}

function compactHudGear(state) {
  const raw = String(state?.carState?.gearShifter || "").trim().toLowerCase();
  const labels = { park: "P", reverse: "R", neutral: "N", drive: "D", sport: "S", low: "L" };
  if (!raw) return null;
  return labels[raw] || (raw.length > 2 ? raw.slice(0, 2).toUpperCase() : raw.toUpperCase());
}

function deriveCompactHudPayload(state) {
  const deviceState = state?.deviceState || {};
  const peripheralState = state?.peripheralState || {};
  const carState = state?.carState || {};
  const carrotMan = state?.carrotMan || {};
  const controlsState = state?.controlsState || {};
  const cpuTemps = Array.isArray(deviceState?.cpuTempC)
    ? deviceState.cpuTempC.filter((value) => Number.isFinite(Number(value))).map(Number)
    : [];
  const cpuTempC = cpuTemps.length ? Math.max(...cpuTemps) : null;
  const voltageMv = Number(peripheralState?.voltage);
  const vEgo = Number(carState?.vEgoCluster ?? carState?.vEgo);
  const speedLimitKph = compactHudSpeedLimitKph(state);
  const tfGap = compactHudGap(state);
  const gearStep = String(carState?.gearShifter || "").toLowerCase() === "drive"
    && Number.isFinite(Number(carState?.gearStep)) && Number(carState.gearStep) > 0
    ? Math.round(Number(carState.gearStep))
    : null;
  const trafficState = Number(carrotMan?.trafficState ?? state?.longitudinalPlan?.trafficState);
  const vehiclePayload = window.CarrotHudDataBridge?.deriveVehicleHudPayload?.(state) || {};
  return {
    cpuTempC,
    memPct: Number(deviceState?.memoryUsagePercent),
    diskPct: null,
    voltageV: Number.isFinite(voltageMv) ? voltageMv / 1000.0 : null,
    vEgo,
    vSetKph: Number(carState?.vCruiseCluster ?? controlsState?.vCruiseCluster),
    temp: compactHudTemp(state),
    redDot: false,
    tlight: trafficState === 1 ? "red" : (trafficState === 2 ? "green" : "off"),
    tfGap,
    tfBars: tfGap,
    gear: vehiclePayload.gear ?? compactHudGear(state),
    gearStep: vehiclePayload.gearStep ?? gearStep,
    lfaActive: vehiclePayload.lfaActive,
    steeringAngleDeg: vehiclePayload.steeringAngleDeg,
    aEgo: vehiclePayload.aEgo,
    steerOutput: vehiclePayload.steerOutput,
    leftBlinker: vehiclePayload.leftBlinker,
    rightBlinker: vehiclePayload.rightBlinker,
    fuelGauge: vehiclePayload.fuelGauge,
    ureaGauge: vehiclePayload.ureaGauge,
    tpms: vehiclePayload.tpms,
    gpsOk: state?.gpsLocationExternal?.latitude != null || state?.gpsLocationExternal?.longitude != null,
    driveMode: compactHudDriveMode(state),
    speedLimitKph,
    speedLimitOver: Number.isFinite(vEgo) && Number.isFinite(speedLimitKph) ? (vEgo * 3.6) > speedLimitKph : false,
    speedLimitBlink: Number.isFinite(Number(carrotMan?.xSpdLimit)) && Number(carrotMan.xSpdLimit) > 0
      && Number(carrotMan?.xSpdType) !== 22 && Number(carrotMan?.xSpdType) !== 4,
    apm: Number(carrotMan?.activeCarrot) >= 2 ? "APN" : (Number(carrotMan?.activeCarrot) >= 1 ? "APM" : ""),
  };
}

function drivingHudUpdateFromRawState() {
  const payload = deriveCompactHudPayload(RAW_HUD_STATE);
  Object.assign(payload, deriveNormalizedHudDeviceMetrics(RAW_HUD_STATE));
  Object.assign(payload, deriveNormalizedHudVehicleMetrics(RAW_HUD_STATE));
  drivingHudUpdateFromCarPayload(payload);
}

let _hudRenderDirty = false;
let _hudRafId = null;

function _hudMarkDirty() {
  _hudRenderDirty = true;
  _hudScheduleRender();
}

function shouldRenderCarrotHudState() {
  return shouldRunCarrotHudRealtime() || Boolean(window.CarrotVisionReplay?.isActive?.());
}

function _hudScheduleRender() {
  if (_hudRafId != null || !_hudRenderDirty) return;
  if (!shouldRenderCarrotHudState()) return;
  _hudRafId = requestAnimationFrame(() => {
    _hudRafId = null;
    if (!_hudRenderDirty || !shouldRenderCarrotHudState()) return;
    _hudRenderDirty = false;
    drivingHudUpdateFromRawState();
  });
}

function _hudStopRenderLoop() {
  if (_hudRafId != null) {
    cancelAnimationFrame(_hudRafId);
    _hudRafId = null;
  }
  _hudRenderDirty = false;
}

function compactOverlayScheduleReconnect(ms = 1000) {
  if (!compactDesiredServices().length || COMPACT_OVERLAY_RETRY_T) return;
  COMPACT_OVERLAY_RETRY_T = setTimeout(() => {
    COMPACT_OVERLAY_RETRY_T = null;
    if (compactDesiredServices().length) compactOverlayConnect();
  }, ms);
}

function compactOverlayConnect() {
  const services = compactDesiredServices();
  if (!services.length) return;
  const serviceSignature = services.join(",");
  if (COMPACT_OVERLAY_WS && (COMPACT_OVERLAY_WS.readyState === WebSocket.OPEN || COMPACT_OVERLAY_WS.readyState === WebSocket.CONNECTING)) {
    if (COMPACT_STATE_SERVICE_SIGNATURE === serviceSignature) return;
    const previous = COMPACT_OVERLAY_WS;
    COMPACT_OVERLAY_WS = null;
    COMPACT_OVERLAY_READY = false;
    COMPACT_STATE_SERVICE_SIGNATURE = "";
    clearCompactFirstDataTimer();
    try { previous.close(1000, "compact_services_changed"); } catch {}
  }

  let firstDataReceived = false;
  const ws = new WebSocket(buildCompactStateUrl(services));
  ws.binaryType = "arraybuffer";
  COMPACT_OVERLAY_WS = ws;
  COMPACT_STATE_SERVICE_SIGNATURE = serviceSignature;
  clearCompactFirstDataTimer();
  COMPACT_FIRST_DATA_T = setTimeout(() => {
    COMPACT_FIRST_DATA_T = null;
    if (COMPACT_OVERLAY_WS === ws && !firstDataReceived) {
      try { ws.close(4000, "compact_first_data_timeout"); } catch {}
    }
  }, COMPACT_FIRST_DATA_TIMEOUT_MS);

  ws.onmessage = async (event) => {
    try {
      if (typeof event.data === "string") {
        const hello = JSON.parse(event.data);
        if (hello?.mode !== COMPACT_STATE_MODE) {
          try { ws.close(1002, "compact_bad_hello"); } catch {}
        }
        return;
      }
      const data = event.data instanceof Blob ? await event.data.arrayBuffer() : event.data;
      const frames = window.CarrotVisionCompact?.decodeFrames?.(data)
        || [window.CarrotVisionCompact?.decodeFrame?.(data)].filter(Boolean);
      for (const frame of frames) {
        if (!frame || (!RAW_HUD_SERVICES.includes(frame.service) && !RAW_OVERLAY_SERVICES.includes(frame.service))) continue;
        applyCompactFrame(frame.service, frame.decoded);
      }
      if (!frames.length) return;
      firstDataReceived = true;
      clearCompactFirstDataTimer();
      COMPACT_OVERLAY_READY = true;
      publishCompactState();
    } catch (error) {
      console.warn("[compact overlay] bad message", error);
    }
  };

  ws.onclose = () => {
    clearCompactFirstDataTimer();
    if (COMPACT_OVERLAY_WS !== ws) return;
    COMPACT_OVERLAY_WS = null;
    COMPACT_OVERLAY_READY = false;
    COMPACT_STATE_SERVICE_SIGNATURE = "";
    publishCompactState();
    if (!compactDesiredServices().length) return;
    compactOverlayScheduleReconnect(1000);
  };
}

function rawHudConnectAll() {
  if (!shouldRunCarrotDriveDataRealtime()) return;
  setCarrotVisionState({ raw: { hud: "connecting" } }, { reason: "raw hud connecting" });
  compactOverlayConnect();
  _hudMarkDirty();
}

function rawOverlayConnectAll() {
  if (!shouldRunCarrotVisionRealtime()) return;
  COMPACT_OVERLAY_REQUESTED = true;
  setCarrotVisionState({ raw: { overlay: "connecting" } }, { reason: "raw overlay connecting" });
  compactOverlayConnect();
}

async function rawHudDisconnectAll() {
  setCarrotVisionState({ raw: { hud: "idle" } }, { reason: "raw hud idle" });
  _hudStopRenderLoop();
  if (COMPACT_OVERLAY_RETRY_T) {
    clearTimeout(COMPACT_OVERLAY_RETRY_T);
    COMPACT_OVERLAY_RETRY_T = null;
  }
  clearCompactFirstDataTimer();
  const compactWs = COMPACT_OVERLAY_WS;
  COMPACT_OVERLAY_WS = null;
  COMPACT_OVERLAY_READY = false;
  COMPACT_STATE_SERVICE_SIGNATURE = "";
  publishCompactState();
  try { compactWs?.close?.(); } catch {}
  if (compactDesiredServices().length) compactOverlayConnect();
}

async function rawOverlayDisconnectAll() {
  const overlayWasRequested = COMPACT_OVERLAY_REQUESTED;
  COMPACT_OVERLAY_REQUESTED = false;
  resetFrameSynchronization();
  setCarrotVisionState({ raw: { overlay: "idle" } }, { reason: "raw overlay idle" });
  // The compact socket is shared with the HUD. Lifecycle synchronization can
  // request an overlay disconnect before the overlay was ever enabled; in that
  // case closing a HUD-only CONNECTING socket only creates a browser warning
  // and an unnecessary reconnect.
  if (!overlayWasRequested) return;
  if (COMPACT_OVERLAY_RETRY_T) {
    clearTimeout(COMPACT_OVERLAY_RETRY_T);
    COMPACT_OVERLAY_RETRY_T = null;
  }
  clearCompactFirstDataTimer();
  const compactWs = COMPACT_OVERLAY_WS;
  COMPACT_OVERLAY_WS = null;
  COMPACT_OVERLAY_READY = false;
  COMPACT_STATE_SERVICE_SIGNATURE = "";
  publishCompactState();
  try { compactWs?.close?.(); } catch {}
  if (compactDesiredServices().length) compactOverlayConnect();
}

function syncCompactDriveDataActivity() {
  if (compactDesiredServices().length) {
    compactOverlayConnect();
    return;
  }
  rawHudDisconnectAll();
}

window.CarrotVisionRaw = {
  applyCompactFrame,
  applyCompactFrames,
  clearState: clearCompactState,
  connectHud: rawHudConnectAll,
  connectOverlay: rawOverlayConnectAll,
  disconnectHud: rawHudDisconnectAll,
  disconnectOverlay: rawOverlayDisconnectAll,
  hasCompactState: compactStateActive,
  markHudDirty: _hudMarkDirty,
  scheduleHudRender: _hudScheduleRender,
  syncDataActivity: syncCompactDriveDataActivity,
  stopHudRenderLoop: _hudStopRenderLoop,
  updateHudFromCarPayload: drivingHudUpdateFromCarPayload,
  updateHudFromRawState: drivingHudUpdateFromRawState,
};
Object.assign(window, {
  drivingHudUpdateFromCarPayload,
  drivingHudUpdateFromRawState,
  rawHudConnectAll,
  rawHudDisconnectAll,
  rawOverlayConnectAll,
  rawOverlayDisconnectAll,
  _hudMarkDirty,
  _hudScheduleRender,
  _hudStopRenderLoop,
});
