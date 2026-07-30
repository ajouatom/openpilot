/* Carrot Vision compact HUD/overlay runtime.
 * Owns one compact websocket, latest display state, and HUD normalization.
 */
var setCarrotVisionState = window.CarrotVisionSetState;

let LAST_HUD_PAYLOAD_SIGNATURE = "";
const COMPACT_FIRST_DATA_TIMEOUT_MS = 8000;
let HUD_CRUISE_OVERRIDE_HOLD = null;

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
// Subscribed only while something holds a "tracks" lease, but stored and
// routed exactly like an overlay service once it arrives.
const RAW_TRACK_SERVICES = window.CarrotVisionCompact?.TRACK_SERVICES || [];
// AR 앵커 입력. "ar" 임대가 있을 때만 구독한다.
const RAW_AR_SERVICES = window.CarrotVisionCompact?.AR_SERVICES || [];
// vision_compact.js 는 번들이 아니라 index.html 의 수동 ?v= 로만 캐시가 갱신된다.
// 버전을 안 올리면 옛 파일이 로드되어 이 목록이 조용히 비고, cameraOdometry 가
// 오버레이 상태에 영영 도착하지 않는다(원인 찾기 매우 어려움). 시끄럽게 알린다.
if (window.CarrotVisionCompact && !RAW_AR_SERVICES.length) {
  console.warn("[carrot] vision_compact.js 가 구버전입니다 — index.html 의 ?v= 를 올리세요. AR 입력이 비활성화됩니다.");
}
const RAW_OVERLAY_HUD_ONLY_SERVICES = new Set([
  "roadCameraState", "liveDelay", "liveTorqueParameters", "liveParameters",
  // Drive Insights reads this through the live state provider; the camera
  // overlay never draws it, so it must not force an overlay repaint.
  "liveTracks",
  // AR 오버레이가 자체 스케줄로 그리므로 카메라 오버레이를 다시 칠하지 않는다.
  "cameraOdometry",
  "livePose",
  "carrotNavi",
]);

function isOverlayStateService(service) {
  return RAW_OVERLAY_SERVICES.includes(service)
    || RAW_TRACK_SERVICES.includes(service)
    || RAW_AR_SERVICES.includes(service);
}
const COMPACT_STATE_MODE = "carrot-state-v1";
let COMPACT_OVERLAY_WS = null;
let COMPACT_OVERLAY_RETRY_T = null;
let COMPACT_OVERLAY_READY = false;
let COMPACT_STATE_SERVICE_SIGNATURE = "";
let COMPACT_OVERLAY_REQUESTED = false;
let COMPACT_FIRST_DATA_T = null;
const MODEL_FRAME_HISTORY = [];
const MODEL_FRAME_HISTORY_LIMIT = 64;
const ROAD_FRAME_HISTORY = [];
const ROAD_FRAME_HISTORY_LIMIT = 96;
const RTP_FRAME_HISTORY = new Map();
const RTP_FRAME_HISTORY_LIMIT = 96;
let PRESENTED_VIDEO_FRAME_ID = null;
let LAST_PRESENTED_RTP_TIMESTAMP = null;

function publishPresentedFrame(detail = {}) {
  const frame = presentedRoadFrame(PRESENTED_VIDEO_FRAME_ID);
  return window.DriveVisionPresentedFrames?.publish?.({
    ...detail,
    frameId: Number.isFinite(PRESENTED_VIDEO_FRAME_ID) ? PRESENTED_VIDEO_FRAME_ID : null,
    cameraTimestampEof: frame?.timestampEof ?? null,
    clockMappingConfidence: frame?.confidence ?? "unmapped",
  }) || null;
}

function subscribePresentedFrame(listener) {
  const unsubscribe = window.DriveVisionPresentedFrames?.subscribe?.(listener);
  return typeof unsubscribe === "function" ? unsubscribe : () => false;
}

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
  } else {
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
  publishPresentedFrame({ source: "live", metadata: metadata || null });
}

function rememberRoadCameraFrame(cameraState) {
  const frameId = Number(cameraState?.frameId);
  const timestampEof = Number(cameraState?.timestampEof);
  if (!Number.isFinite(frameId) || !Number.isFinite(timestampEof) || timestampEof <= 0) return;
  const sample = Object.freeze({ frameId, timestampEof });
  const previous = ROAD_FRAME_HISTORY[ROAD_FRAME_HISTORY.length - 1];
  if (previous?.frameId === frameId) ROAD_FRAME_HISTORY[ROAD_FRAME_HISTORY.length - 1] = sample;
  else {
    ROAD_FRAME_HISTORY.push(sample);
    if (ROAD_FRAME_HISTORY.length > ROAD_FRAME_HISTORY_LIMIT) ROAD_FRAME_HISTORY.shift();
  }
}

function presentedRoadFrame(frameIdValue) {
  const frameId = Number(frameIdValue);
  if (!Number.isFinite(frameId) || !ROAD_FRAME_HISTORY.length) return null;
  let nearest = null;
  for (let index = ROAD_FRAME_HISTORY.length - 1; index >= 0; index -= 1) {
    const candidate = ROAD_FRAME_HISTORY[index];
    if (candidate.frameId === frameId) {
      return Object.freeze({ ...candidate, confidence: "exact-frame" });
    }
    if (!nearest || Math.abs(candidate.frameId - frameId) < Math.abs(nearest.frameId - frameId)) {
      nearest = candidate;
    }
  }
  const frameGap = frameId - nearest.frameId;
  if (Math.abs(frameGap) > 3) return null;
  return Object.freeze({
    frameId,
    timestampEof: nearest.timestampEof + frameGap * 50_000_000,
    confidence: "estimated-frame",
  });
}

function noteReplayPresentedFrame(metadata = {}) {
  const roadFrameId = Number(RAW_OVERLAY_STATE?.roadCameraState?.frameId);
  PRESENTED_VIDEO_FRAME_ID = Number.isFinite(roadFrameId) ? roadFrameId : null;
  LAST_PRESENTED_RTP_TIMESTAMP = null;
  return publishPresentedFrame({ source: "replay", metadata });
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
    publishPresentedFrame({ source: "live", reason: "late-rtp-mapping" });
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
  ROAD_FRAME_HISTORY.length = 0;
  RTP_FRAME_HISTORY.clear();
  PRESENTED_VIDEO_FRAME_ID = null;
  LAST_PRESENTED_RTP_TIMESTAMP = null;
}

window.CarrotVisionFrameSync = {
  notePresentedVideoFrame,
  noteReplayPresentedFrame,
  noteRtpFrameMapping,
  subscribePresented: subscribePresentedFrame,
  reset: resetFrameSynchronization,
  selectModel: selectSynchronizedModel,
  status() {
    return Object.freeze({
      presentedVideoFrameId: PRESENTED_VIDEO_FRAME_ID,
      lastPresentedRtpTimestamp: LAST_PRESENTED_RTP_TIMESTAMP,
      channel: window.DriveVisionPresentedFrames?.status?.() || null,
    });
  },
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

function recordedReplayActive() {
  return Boolean(window.CarrotVisionReplay?.isActive?.());
}

function compactDesiredServices() {
  // Recorded replay owns the whole compact-state timeline. Keeping the live
  // socket open here lets current-device carrotNavi/cameraOdometry samples
  // overwrite recorded samples between replay records, which presents as AR
  // blinking and impossible clock-age jumps.
  if (recordedReplayActive()) return [];
  const services = [];
  if (shouldRunCarrotHudData()) services.push(...RAW_HUD_SERVICES);
  if (
    (COMPACT_OVERLAY_REQUESTED && shouldRunCarrotVisionRealtime())
    || isCarrotDriveDataRequested("overlay")
  ) services.push(...RAW_OVERLAY_SERVICES);
  if (isCarrotDriveDataRequested("tracks")) services.push(...RAW_TRACK_SERVICES);
  if (isCarrotDriveDataRequested("ar")) services.push(...RAW_AR_SERVICES);
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
  if (isOverlayStateService(service)) {
    if (service === "modelV2") rememberModelFrame(decoded);
    if (service === "roadCameraState") rememberRoadCameraFrame(decoded);
    RAW_OVERLAY_STATE[service] = decoded;
    window.CarrotOverlayState = RAW_OVERLAY_STATE;
    if (options.updateVisionState !== false) {
      setCarrotVisionState({ raw: { overlay: "ready" } }, { reason: `compact overlay ${service}`, silent: true });
    }
    overlayDirty = !RAW_OVERLAY_HUD_ONLY_SERVICES.has(service);
    applied = true;
  }
  // Vision owns the latency-sensitive presentation request. Schedule it before
  // notifying auxiliary data consumers so an insights subscriber can never
  // delay the camera overlay's place in the browser task queue.
  if (applied && options.render !== false && (hudDirty || overlayDirty)) {
    emitCarrotRenderRequest({
      hudDirty,
      overlayDirty,
    });
  }
  if (applied && options.notifyProvider !== false) {
    try {
      const providerSnapshot = window.CarrotDriveLiveStateProvider?.noteServiceReceived?.(service);
      const receivedAt = Number(providerSnapshot?.receivedAtMonotonic?.[service]);
      if (Number.isFinite(receivedAt)) RAW_STATE_RECEIVED_AT_MONOTONIC[service] = receivedAt;
    } catch (error) {
      console.warn("[compact state] provider receipt rejected", error);
    }
  }
  return applied;
}

function applyCompactFrames(frames, options = {}) {
  const list = Array.isArray(frames) ? frames : [];
  let applied = 0;
  let hudReady = false;
  let overlayReady = false;
  let overlayDirty = false;
  const appliedServices = [];

  for (const frame of list) {
    const service = frame?.service;
    const isHud = RAW_HUD_SERVICES.includes(service);
    const isOverlay = isOverlayStateService(service);
    if (!isHud && !isOverlay) continue;
    if (!applyCompactFrame(service, frame.decoded, {
      render: false,
      markHud: false,
      updateVisionState: false,
      notifyProvider: false,
    })) continue;
    applied += 1;
    appliedServices.push(service);
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
  if (applied && options.render !== false && (hudReady || overlayDirty)) {
    emitCarrotRenderRequest({ hudDirty: hudReady, overlayDirty });
  }
  // A decoded websocket/replay batch is one coherent state transition. Publish
  // it once instead of synchronously repainting Drive Insights per service.
  if (appliedServices.length && options.notifyProvider !== false) {
    try {
      const provider = window.CarrotDriveLiveStateProvider;
      let providerSnapshot = null;
      if (typeof provider?.noteServicesReceived === "function") {
        providerSnapshot = provider.noteServicesReceived(appliedServices);
      } else {
        for (const service of appliedServices) {
          providerSnapshot = provider?.noteServiceReceived?.(service) || providerSnapshot;
        }
      }
      for (const service of appliedServices) {
        const receivedAt = Number(providerSnapshot?.receivedAtMonotonic?.[service]);
        if (Number.isFinite(receivedAt)) RAW_STATE_RECEIVED_AT_MONOTONIC[service] = receivedAt;
      }
    } catch (error) {
      console.warn("[compact state] provider batch receipt rejected", error);
    }
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
  const basePayload = {
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
    laneModeRequested: j.laneModeRequested,
    laneModePlanned: j.laneModePlanned,
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
  const payload = window.CarrotHudDataBridge?.withVehicleHudFields?.(basePayload, j) || {
    ...basePayload,
    evActive: j.evActive === true,
    activeLaneLine: j.activeLaneLine == null ? null : j.activeLaneLine === true,
    laneModeRequested: j.laneModeRequested == null ? null : j.laneModeRequested === true,
    laneModePlanned: j.laneModePlanned == null ? null : j.laneModePlanned === true,
    laneModePresentation: j.laneModePresentation ?? null,
    cruiseOverride: j.cruiseOverride && typeof j.cruiseOverride === "object"
      ? j.cruiseOverride
      : null,
  };
  payload.cruiseOverride = stabilizedHudCruiseOverride(payload.cruiseOverride, payload);

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
    window.CarrotHudDataBridge?.vehicleHudSignature?.(payload) || [
      payload.evActive ? 1 : 0,
      payload.activeLaneLine == null ? "-" : (payload.activeLaneLine ? 1 : 0),
      payload.cruiseOverride?.kph ?? "-",
      payload.cruiseOverride?.label ?? "-",
      payload.cruiseOverride?.mode ?? "-",
    ].join(":"),
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
  // DriveVisionHudContent owns the visible HUD lifecycle. The direct overlay
  // fallback is only for a partial/failed platform bootstrap.
  try {
    if (window.DriveVisionHudContent?.update) window.DriveVisionHudContent.update(payload);
    else window.CarrotHudOverlay?.update?.(payload);
  } catch {}
}

function hudCruiseOverrideClock() {
  const replay = window.CarrotVisionReplay?.status?.();
  const replayTime = Number(replay?.currentTime);
  if (replay?.active === true && Number.isFinite(replayTime)) {
    return {
      clockMs: replayTime * 1000,
      clockKey: `replay:${replay.route || ""}:${replay.segment || ""}`,
    };
  }
  const clockMs = Number(window.performance?.now?.());
  return {
    clockMs: Number.isFinite(clockMs) ? clockMs : Date.now(),
    clockKey: "live",
  };
}

function hudCruiseDisplayVisible(payload) {
  const hasRawCruiseState = Object.prototype.hasOwnProperty.call(RAW_HUD_STATE, "carState")
    || Object.prototype.hasOwnProperty.call(RAW_HUD_STATE, "controlsState")
    || Object.prototype.hasOwnProperty.call(RAW_HUD_STATE, "selfdriveState");
  const sharedGate = window.CarrotHudDataBridge?.isCruiseDisplayVisible;
  if (hasRawCruiseState && typeof sharedGate === "function") {
    return sharedGate(RAW_HUD_STATE);
  }
  const setSpeed = Number(payload?.vSetKph);
  return Number.isFinite(setSpeed) && setSpeed > 0 && setSpeed < 250;
}

function stabilizedHudCruiseOverride(value, payload) {
  if (!HUD_CRUISE_OVERRIDE_HOLD) {
    HUD_CRUISE_OVERRIDE_HOLD = window.CarrotHudDataBridge?.createCruiseOverrideHold?.() || null;
  }
  if (!HUD_CRUISE_OVERRIDE_HOLD) return value;
  return HUD_CRUISE_OVERRIDE_HOLD.update(value, {
    ...hudCruiseOverrideClock(),
    active: hudCruiseDisplayVisible(payload),
  });
}

function averageFiniteMetric(values) {
  const samples = Array.isArray(values)
    ? values.map((value) => Number(value)).filter((value) => Number.isFinite(value))
    : [];
  if (!samples.length) return null;
  const total = samples.reduce((sum, value) => sum + value, 0);
  return total / samples.length;
}

function maxFiniteMetric(values) {
  const samples = Array.isArray(values)
    ? values.map((value) => Number(value)).filter((value) => Number.isFinite(value))
    : [];
  return samples.length ? Math.max(...samples) : null;
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
  // Hottest core, matching the native cluster and openpilot HUD. Kept separate
  // from the averaged cpuTempC so each surface keeps the value it was built on.
  const cpuTempMaxC = pickFiniteMetric(
    maxFiniteMetric(liveDeviceState.cpuTempC),
    maxFiniteMetric(rawDeviceState.cpuTempC),
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

  return { cpuTempC, cpuTempMaxC, memPct, diskPct, voltageV };
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
  const vCruise = compactHudCruiseKph(state);
  if (!Number.isFinite(desiredSpeed)) return null;
  return {
    speed: desiredSpeed,
    source: Number.isFinite(vCruise) && desiredSpeed >= vCruise ? (state?.carrotMan?.desiredSource || "") : "",
    is_decel: Number.isFinite(vCruise) ? desiredSpeed < vCruise : false,
  };
}

function compactHudCruiseKph(state) {
  const sharedResolver = window.CarrotHudDataBridge?.resolveCruiseKph;
  if (typeof sharedResolver === "function") return sharedResolver(state);
  for (const value of [
    state?.carState?.vCruiseCluster,
    state?.carState?.vCruise,
    state?.controlsState?.vCruiseCluster,
    state?.controlsState?.vCruise,
  ]) {
    const number = Number(value);
    if (Number.isFinite(number) && number > 0 && number < 250) return number;
  }
  return null;
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
  const vehiclePayload = window.CarrotHudDataBridge?.deriveVehicleHudPayload?.(state) || {};
  const trafficState = Number(vehiclePayload.trafficState);
  const cruiseKph = compactHudCruiseKph(state);
  const payload = {
    cpuTempC,
    memPct: Number(deviceState?.memoryUsagePercent),
    diskPct: null,
    voltageV: Number.isFinite(voltageMv) ? voltageMv / 1000.0 : null,
    vEgo,
    vSetKph: cruiseKph,
    temp: compactHudTemp(state),
    redDot: false,
    // 미니 HUD용 문자열. 신규 오버레이용 numeric trafficState 등
    // 클러스터 전용 필드는 아래 withVehicleHudFields 가 vehiclePayload 로부터 일괄 채운다.
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

  // 클러스터 전용 필드(evActive/activeLaneLine/cruiseOverride/trafficState/drivingMode)를
  // 한 곳에서 채워 "한 필드만 누락" 회귀를 원천 차단한다(이전 trafficState 누락처럼).
  // 브리지 미로드 시(부팅 극초기) 동등한 폴백으로 채운다.
  const withVehicleHudFields = window.CarrotHudDataBridge?.withVehicleHudFields;
  if (typeof withVehicleHudFields === "function") return withVehicleHudFields(payload, vehiclePayload);
  return {
    ...payload,
    evActive: vehiclePayload.evActive === true,
    activeLaneLine: vehiclePayload.activeLaneLine == null ? null : vehiclePayload.activeLaneLine === true,
    laneModeRequested: vehiclePayload.laneModeRequested ?? null,
    laneModePlanned: vehiclePayload.laneModePlanned ?? null,
    laneModePresentation: vehiclePayload.laneModePresentation ?? null,
    cruiseOverride: vehiclePayload.cruiseOverride ?? null,
    trafficState: Number.isFinite(trafficState) ? trafficState : 0,
    drivingMode: vehiclePayload.drivingMode ?? null,
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
      // A Blob conversion may complete after replay has started and the socket
      // has been closed. Reject both sides of that async boundary so no late
      // live sample can leak into recorded state.
      if (recordedReplayActive()) return;
      if (typeof event.data === "string") {
        const hello = JSON.parse(event.data);
        if (hello?.mode !== COMPACT_STATE_MODE) {
          try { ws.close(1002, "compact_bad_hello"); } catch {}
        }
        return;
      }
      const data = event.data instanceof Blob ? await event.data.arrayBuffer() : event.data;
      if (recordedReplayActive()) return;
      const frames = window.CarrotVisionCompact?.decodeFrames?.(data)
        || [window.CarrotVisionCompact?.decodeFrame?.(data)].filter(Boolean);
      const applied = applyCompactFrames(frames, { reason: "compact websocket batch" });
      if (!applied) return;
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
