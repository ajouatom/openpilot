/* Carrot Vision raw HUD/overlay runtime.
 * Owns raw-capnp websocket streams, multiplex fallback, decode worker, and HUD normalization.
 */
var setCarrotVisionState = window.CarrotVisionSetState;

let LAST_HUD_PAYLOAD_SIGNATURE = "";
const RAW_WS_HELLO_TIMEOUT_MS = 3500;
const RAW_WS_FIRST_DATA_TIMEOUT_MS = 6500;
const RAW_WS_WATCHDOG_T = new WeakMap();

const RAW_HUD_LOG_PREFIX = "[raw hud]";
const RAW_HUD_MUX_LOG_PREFIX = "[raw hud mux]";
const RAW_HUD_SERVICES = window.CarrotRawCapnp?.HUD_SERVICES || [];
const RAW_HUD_WS = Object.create(null);
const RAW_HUD_RETRY_T = Object.create(null);
const RAW_HUD_STATE = Object.create(null);
window.CarrotHudState = RAW_HUD_STATE;
const RAW_HUD_OVERLAY_DIRTY_SERVICES = new Set(["carState", "controlsState"]);
const RAW_OVERLAY_LOG_PREFIX = "[raw overlay]";
const RAW_OVERLAY_MUX_LOG_PREFIX = "[raw overlay mux]";
const RAW_OVERLAY_SERVICES = window.CarrotRawCapnp?.OVERLAY_SERVICES || [];
const RAW_OVERLAY_WS = Object.create(null);
const RAW_OVERLAY_RETRY_T = Object.create(null);
const RAW_OVERLAY_STATE = Object.create(null);
window.CarrotOverlayState = RAW_OVERLAY_STATE;
const RAW_OVERLAY_HUD_ONLY_SERVICES = new Set(["roadCameraState", "liveDelay", "liveTorqueParameters", "liveParameters"]);
const RAW_MULTIPLEX_MODE = "raw-capnp-multiplex-relay";
const RAW_TEXT_DECODER = typeof TextDecoder !== "undefined" ? new TextDecoder() : null;
let RAW_HUD_MUX_WS = null;
let RAW_HUD_MUX_RETRY_T = null;
let RAW_HUD_MUX_DISABLED = false;
let RAW_OVERLAY_MUX_WS = null;
let RAW_OVERLAY_MUX_RETRY_T = null;
let RAW_OVERLAY_MUX_DISABLED = false;
const RAW_DECODE_WORKER_URL = "/js/raw_capnp_worker.js";
let RAW_DECODE_WORKER = null;
let RAW_DECODE_WORKER_FAILED = false;
let RAW_DECODE_REQ_ID = 0;
const RAW_DECODE_PENDING = new Map();
const RAW_DECODE_BACKPRESSURE = {
  hud: Object.create(null),
  overlay: Object.create(null),
};
const RAW_DECODE_EPOCH = {
  hud: 0,
  overlay: 0,
};

function rawClearWsWatchdog(ws) {
  const timer = RAW_WS_WATCHDOG_T.get(ws);
  if (!timer) return;
  clearTimeout(timer);
  RAW_WS_WATCHDOG_T.delete(ws);
}

function rawArmWsWatchdog(ws, label, ms, reason, shouldClose) {
  rawClearWsWatchdog(ws);
  const timer = setTimeout(() => {
    RAW_WS_WATCHDOG_T.delete(ws);
    let close = true;
    try {
      close = typeof shouldClose === "function" ? Boolean(shouldClose()) : true;
    } catch {
      close = true;
    }
    if (!close) return;
    console.warn("[raw watchdog]", label, reason);
    try { ws.close(4000, reason); } catch {}
  }, ms);
  RAW_WS_WATCHDOG_T.set(ws, timer);
}

function rawHasState(state) {
  return Boolean(state && Object.keys(state).length > 0);
}

function rejectPendingRawDecodeRequests(message) {
  for (const [requestId, pending] of RAW_DECODE_PENDING.entries()) {
    RAW_DECODE_PENDING.delete(requestId);
    pending.reject(new Error(message));
  }
}

function handleRawDecodeWorkerMessage(event) {
  const payload = event?.data || {};
  const requestId = Number(payload.requestId);
  const pending = RAW_DECODE_PENDING.get(requestId);
  if (!pending) return;
  RAW_DECODE_PENDING.delete(requestId);
  if (!payload.ok) {
    pending.reject(new Error(payload.error || "raw decode failed"));
    return;
  }
  pending.resolve(payload.decoded);
}

function handleRawDecodeWorkerFailure(error) {
  console.log("[raw decode] worker disabled", error);
  RAW_DECODE_WORKER_FAILED = true;
  if (RAW_DECODE_WORKER) {
    try { RAW_DECODE_WORKER.terminate(); } catch {}
    RAW_DECODE_WORKER = null;
  }
  rejectPendingRawDecodeRequests(error?.message || "raw decode worker failed");
}

function ensureRawDecodeWorker() {
  if (RAW_DECODE_WORKER || RAW_DECODE_WORKER_FAILED || typeof Worker === "undefined") return RAW_DECODE_WORKER;
  try {
    const worker = new Worker(RAW_DECODE_WORKER_URL);
    worker.onmessage = handleRawDecodeWorkerMessage;
    worker.onerror = handleRawDecodeWorkerFailure;
    worker.onmessageerror = handleRawDecodeWorkerFailure;
    RAW_DECODE_WORKER = worker;
  } catch (error) {
    RAW_DECODE_WORKER_FAILED = true;
    console.log("[raw decode] worker unavailable", error);
  }
  return RAW_DECODE_WORKER;
}

async function normalizeRawMessageBuffer(data) {
  if (data instanceof ArrayBuffer) return data;
  if (ArrayBuffer.isView(data)) {
    return data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength);
  }
  if (data instanceof Blob) return data.arrayBuffer();
  throw new Error(`unsupported raw payload: ${Object.prototype.toString.call(data)}`);
}

function decodeRawMessageSync(streamKind, service, buffer) {
  if (streamKind === "hud") {
    return window.CarrotRawCapnp?.decodeHudEvent?.(service, buffer);
  }
  if (streamKind === "overlay") {
    return window.CarrotRawCapnp?.decodeOverlayEvent?.(service, buffer);
  }
  return null;
}

async function decodeRawMessage(streamKind, service, data) {
  const buffer = await normalizeRawMessageBuffer(data);
  const worker = ensureRawDecodeWorker();
  if (!worker) {
    return decodeRawMessageSync(streamKind, service, buffer);
  }

  return new Promise((resolve, reject) => {
    const requestId = ++RAW_DECODE_REQ_ID;
    RAW_DECODE_PENDING.set(requestId, { resolve, reject });
    try {
      worker.postMessage({ requestId, streamKind, service, buffer }, [buffer]);
    } catch (error) {
      RAW_DECODE_PENDING.delete(requestId);
      reject(error);
    }
  });
}

function getRawDecodeEpoch(streamKind) {
  return Number(RAW_DECODE_EPOCH[streamKind] || 0);
}

function invalidateRawDecodeState(streamKind) {
  RAW_DECODE_EPOCH[streamKind] = getRawDecodeEpoch(streamKind) + 1;
  const nextEpoch = RAW_DECODE_EPOCH[streamKind];
  const bucket = RAW_DECODE_BACKPRESSURE[streamKind];
  Object.values(bucket).forEach((state) => {
    state.pendingData = null;
    state.pendingEpoch = nextEpoch;
    state.pendingVersion = Number(state.pendingVersion || 0) + 1;
  });
}

function getRawDecodeServiceState(streamKind, service) {
  const bucket = RAW_DECODE_BACKPRESSURE[streamKind];
  if (!bucket[service]) {
    bucket[service] = {
      inFlight: false,
      pendingData: null,
      pendingEpoch: getRawDecodeEpoch(streamKind),
      pendingVersion: 0,
    };
  }
  return bucket[service];
}

function shouldApplyDecodedStream(streamKind) {
  return streamKind === "hud" ? shouldRunCarrotHudRealtime() : shouldRunCarrotVisionRealtime();
}

async function flushRawDecodeQueue(streamKind, service, applyDecoded) {
  const state = getRawDecodeServiceState(streamKind, service);
  if (state.inFlight) return;
  state.inFlight = true;
  try {
    while (state.pendingData != null) {
      const nextData = state.pendingData;
      const pendingEpoch = state.pendingEpoch;
      const pendingVersion = state.pendingVersion;
      state.pendingData = null;

      let decoded = null;
      try {
        decoded = await decodeRawMessage(streamKind, service, nextData);
      } catch (error) {
        console.log(`[raw decode] ${streamKind}/${service} failed`, error);
        continue;
      }

      if (!decoded) continue;
      if (!shouldApplyDecodedStream(streamKind)) continue;
      if (pendingEpoch !== getRawDecodeEpoch(streamKind)) continue;
      if (state.pendingData != null || state.pendingEpoch !== pendingEpoch || state.pendingVersion !== pendingVersion) continue;
      applyDecoded(decoded);
    }
  } finally {
    state.inFlight = false;
    if (state.pendingData != null) {
      flushRawDecodeQueue(streamKind, service, applyDecoded).catch((error) => {
        console.log(`[raw decode] ${streamKind}/${service} queue restart failed`, error);
      });
    }
  }
}

function queueRawDecodedMessage(streamKind, service, data, applyDecoded) {
  const state = getRawDecodeServiceState(streamKind, service);
  state.pendingData = data;
  state.pendingEpoch = getRawDecodeEpoch(streamKind);
  state.pendingVersion = Number(state.pendingVersion || 0) + 1;
  flushRawDecodeQueue(streamKind, service, applyDecoded).catch((error) => {
    console.log(`[raw decode] ${streamKind}/${service} queue failed`, error);
  });
}

function buildRawMultiplexUrl(services) {
  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  const params = new URLSearchParams();
  params.set("services", services.join(","));
  return `${wsProto}://${location.host}/ws/raw_multiplex?${params.toString()}`;
}

async function parseRawMultiplexFrame(data) {
  const buffer = await normalizeRawMessageBuffer(data);
  const bytes = new Uint8Array(buffer);
  if (bytes.length < 2) {
    throw new Error("bad raw multiplex frame");
  }
  const serviceNameLength = bytes[0];
  const serviceNameEnd = 1 + serviceNameLength;
  if (bytes.length <= serviceNameEnd) {
    throw new Error("truncated raw multiplex frame");
  }
  const service = RAW_TEXT_DECODER
    ? RAW_TEXT_DECODER.decode(bytes.subarray(1, serviceNameEnd))
    : String.fromCharCode(...bytes.subarray(1, serviceNameEnd));
  return {
    service,
    payload: buffer.slice(serviceNameEnd),
  };
}

function handleHudDecodedMessage(service, data) {
  queueRawDecodedMessage("hud", service, data, (decoded) => {
    RAW_HUD_STATE[service] = decoded;
    window.CarrotHudState = RAW_HUD_STATE;
    setCarrotVisionState({ raw: { hud: "ready" } }, { reason: `raw hud ${service}`, silent: true });
    _hudMarkDirty();
    emitCarrotRenderRequest({
      hudDirty: true,
      overlayDirty: RAW_HUD_OVERLAY_DIRTY_SERVICES.has(service),
    });
  });
}

function handleOverlayDecodedMessage(service, data) {
  queueRawDecodedMessage("overlay", service, data, (decoded) => {
    RAW_OVERLAY_STATE[service] = decoded;
    window.CarrotOverlayState = RAW_OVERLAY_STATE;
    setCarrotVisionState({ raw: { overlay: "ready" } }, { reason: `raw overlay ${service}`, silent: true });
    emitCarrotRenderRequest({
      hudDirty: true,
      overlayDirty: !RAW_OVERLAY_HUD_ONLY_SERVICES.has(service),
    });
  });
}

function drivingHudUpdateFromCarPayload(j) {
  if (!window.DrivingHud || !j) return;

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
    gpsOk: j.gpsOk,
    driveMode: j.driveMode,
    speedLimitKph: j.speedLimitKph,
    speedLimitOver: j.speedLimitOver,
    speedLimitBlink: j.speedLimitBlink,
    apm: j.apm,
  };

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
  window.DrivingHud.update(payload);
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
  const liveServices = window.CarrotLiveRuntimeState?.services;
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
  const freeSpacePct = pickFiniteMetric(liveDeviceState.freeSpacePercent);
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
  const liveServices = window.CarrotLiveRuntimeState?.services;
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

function drivingHudUpdateFromRawState() {
  const payload = window.CarrotRawCapnp?.deriveHudPayload?.(RAW_HUD_STATE);
  if (!payload) return;
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

function _hudScheduleRender() {
  if (_hudRafId != null || !_hudRenderDirty) return;
  if (!shouldRunCarrotHudRealtime()) return;
  _hudRafId = requestAnimationFrame(() => {
    _hudRafId = null;
    if (!_hudRenderDirty || !shouldRunCarrotHudRealtime()) return;
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

function rawHudScheduleReconnect(service, ms = 1000) {
  if (!shouldRunCarrotHudRealtime()) return;
  if (RAW_HUD_RETRY_T[service]) return;
  RAW_HUD_RETRY_T[service] = setTimeout(() => {
    RAW_HUD_RETRY_T[service] = null;
    if (!shouldRunCarrotHudRealtime()) return;
    rawHudConnectService(service);
  }, ms);
}

function rawHudScheduleMultiplexReconnect(ms = 1000) {
  if (!shouldRunCarrotHudRealtime() || RAW_HUD_MUX_DISABLED) return;
  if (RAW_HUD_MUX_RETRY_T) return;
  RAW_HUD_MUX_RETRY_T = setTimeout(() => {
    RAW_HUD_MUX_RETRY_T = null;
    if (!shouldRunCarrotHudRealtime() || RAW_HUD_MUX_DISABLED) return;
    rawHudConnectMultiplex();
  }, ms);
}

function rawHudConnectService(service) {
  if (!shouldRunCarrotHudRealtime()) return;
  const current = RAW_HUD_WS[service];
  if (current && (current.readyState === WebSocket.OPEN || current.readyState === WebSocket.CONNECTING)) return;

  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  const ws = new WebSocket(`${wsProto}://${location.host}/ws/raw/${service}`);
  ws.binaryType = "arraybuffer";
  RAW_HUD_WS[service] = ws;

  ws.onopen = () => {
    console.log(RAW_HUD_LOG_PREFIX, service, "open");
  };

  ws.onmessage = async (ev) => {
    try {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        console.log(RAW_HUD_LOG_PREFIX, service, "hello", hello);
        return;
      }
      handleHudDecodedMessage(service, ev.data);
    } catch (e) {
      console.log(RAW_HUD_LOG_PREFIX, service, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_HUD_LOG_PREFIX, service, "error", e);
  };

  ws.onclose = () => {
    console.log(RAW_HUD_LOG_PREFIX, service, "close -> reconnect");
    if (RAW_HUD_WS[service] !== ws) return;
    RAW_HUD_WS[service] = null;
    rawHudScheduleReconnect(service, 1000);
  };
}

function rawHudConnectAllLegacy() {
  for (const service of RAW_HUD_SERVICES) {
    rawHudConnectService(service);
  }
  _hudMarkDirty();
}

function rawHudConnectMultiplex() {
  if (!shouldRunCarrotHudRealtime() || RAW_HUD_MUX_DISABLED) return;
  const current = RAW_HUD_MUX_WS;
  if (current && (current.readyState === WebSocket.OPEN || current.readyState === WebSocket.CONNECTING)) return;

  let helloReceived = false;
  let firstDataReceived = false;
  const ws = new WebSocket(buildRawMultiplexUrl(RAW_HUD_SERVICES));
  ws.binaryType = "arraybuffer";
  RAW_HUD_MUX_WS = ws;
  rawArmWsWatchdog(ws, "hud mux hello", RAW_WS_HELLO_TIMEOUT_MS, "raw_hello_timeout", () => (
    shouldRunCarrotHudRealtime() && RAW_HUD_MUX_WS === ws && !helloReceived
  ));

  ws.onopen = () => {
    console.log(RAW_HUD_MUX_LOG_PREFIX, "open");
  };

  ws.onmessage = async (ev) => {
    try {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        console.log(RAW_HUD_MUX_LOG_PREFIX, "hello", hello);
        helloReceived = hello?.mode === RAW_MULTIPLEX_MODE;
        rawClearWsWatchdog(ws);
        if (helloReceived) {
          rawArmWsWatchdog(ws, "hud mux first-data", RAW_WS_FIRST_DATA_TIMEOUT_MS, "raw_first_data_timeout", () => (
            shouldRunCarrotHudRealtime() && RAW_HUD_MUX_WS === ws && !firstDataReceived && !rawHasState(RAW_HUD_STATE)
          ));
        }
        return;
      }
      firstDataReceived = true;
      rawClearWsWatchdog(ws);
      const frame = await parseRawMultiplexFrame(ev.data);
      if (!RAW_HUD_SERVICES.includes(frame.service)) return;
      handleHudDecodedMessage(frame.service, frame.payload);
    } catch (e) {
      console.log(RAW_HUD_MUX_LOG_PREFIX, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_HUD_MUX_LOG_PREFIX, "error", e);
  };

  ws.onclose = () => {
    rawClearWsWatchdog(ws);
    console.log(RAW_HUD_MUX_LOG_PREFIX, "close");
    if (RAW_HUD_MUX_WS !== ws) return;
    RAW_HUD_MUX_WS = null;
    if (!shouldRunCarrotHudRealtime()) return;
    if (!helloReceived) {
      RAW_HUD_MUX_DISABLED = true;
      rawHudConnectAllLegacy();
      return;
    }
    rawHudScheduleMultiplexReconnect(1000);
  };
}

function rawHudConnectAll() {
  invalidateRawDecodeState("hud");
  setCarrotVisionState({ raw: { hud: "connecting" } }, { reason: "raw hud connecting" });
  if (!RAW_HUD_MUX_DISABLED) {
    rawHudConnectMultiplex();
    if (RAW_HUD_MUX_WS) {
      _hudMarkDirty();
      return;
    }
  }
  rawHudConnectAllLegacy();
}

function rawOverlayScheduleReconnect(service, ms = 1000) {
  if (!shouldRunCarrotVisionRealtime()) return;
  if (RAW_OVERLAY_RETRY_T[service]) return;
  RAW_OVERLAY_RETRY_T[service] = setTimeout(() => {
    RAW_OVERLAY_RETRY_T[service] = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    rawOverlayConnectService(service);
  }, ms);
}

function rawOverlayScheduleMultiplexReconnect(ms = 1000) {
  if (!shouldRunCarrotVisionRealtime() || RAW_OVERLAY_MUX_DISABLED) return;
  if (RAW_OVERLAY_MUX_RETRY_T) return;
  RAW_OVERLAY_MUX_RETRY_T = setTimeout(() => {
    RAW_OVERLAY_MUX_RETRY_T = null;
    if (!shouldRunCarrotVisionRealtime() || RAW_OVERLAY_MUX_DISABLED) return;
    rawOverlayConnectMultiplex();
  }, ms);
}

function rawOverlayConnectService(service) {
  if (!shouldRunCarrotVisionRealtime()) return;
  const current = RAW_OVERLAY_WS[service];
  if (current && (current.readyState === WebSocket.OPEN || current.readyState === WebSocket.CONNECTING)) return;

  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  const ws = new WebSocket(`${wsProto}://${location.host}/ws/raw/${service}`);
  ws.binaryType = "arraybuffer";
  RAW_OVERLAY_WS[service] = ws;

  ws.onopen = () => {
    console.log(RAW_OVERLAY_LOG_PREFIX, service, "open");
  };

  ws.onmessage = async (ev) => {
    try {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        console.log(RAW_OVERLAY_LOG_PREFIX, service, "hello", hello);
        return;
      }
      handleOverlayDecodedMessage(service, ev.data);
    } catch (e) {
      console.log(RAW_OVERLAY_LOG_PREFIX, service, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_OVERLAY_LOG_PREFIX, service, "error", e);
  };

  ws.onclose = () => {
    console.log(RAW_OVERLAY_LOG_PREFIX, service, "close -> reconnect");
    if (RAW_OVERLAY_WS[service] !== ws) return;
    RAW_OVERLAY_WS[service] = null;
    rawOverlayScheduleReconnect(service, 1000);
  };
}

function rawOverlayConnectAllLegacy() {
  for (const service of RAW_OVERLAY_SERVICES) {
    rawOverlayConnectService(service);
  }
}

function rawOverlayConnectMultiplex() {
  if (!shouldRunCarrotVisionRealtime() || RAW_OVERLAY_MUX_DISABLED) return;
  const current = RAW_OVERLAY_MUX_WS;
  if (current && (current.readyState === WebSocket.OPEN || current.readyState === WebSocket.CONNECTING)) return;

  let helloReceived = false;
  let firstDataReceived = false;
  const ws = new WebSocket(buildRawMultiplexUrl(RAW_OVERLAY_SERVICES));
  ws.binaryType = "arraybuffer";
  RAW_OVERLAY_MUX_WS = ws;
  rawArmWsWatchdog(ws, "overlay mux hello", RAW_WS_HELLO_TIMEOUT_MS, "raw_hello_timeout", () => (
    shouldRunCarrotVisionRealtime() && RAW_OVERLAY_MUX_WS === ws && !helloReceived
  ));

  ws.onopen = () => {
    console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "open");
  };

  ws.onmessage = async (ev) => {
    try {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "hello", hello);
        helloReceived = hello?.mode === RAW_MULTIPLEX_MODE;
        rawClearWsWatchdog(ws);
        if (helloReceived) {
          rawArmWsWatchdog(ws, "overlay mux first-data", RAW_WS_FIRST_DATA_TIMEOUT_MS, "raw_first_data_timeout", () => (
            shouldRunCarrotVisionRealtime() && RAW_OVERLAY_MUX_WS === ws && !firstDataReceived && !rawHasState(RAW_OVERLAY_STATE)
          ));
        }
        return;
      }
      firstDataReceived = true;
      rawClearWsWatchdog(ws);
      const frame = await parseRawMultiplexFrame(ev.data);
      if (!RAW_OVERLAY_SERVICES.includes(frame.service)) return;
      handleOverlayDecodedMessage(frame.service, frame.payload);
    } catch (e) {
      console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "error", e);
  };

  ws.onclose = () => {
    rawClearWsWatchdog(ws);
    console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "close");
    if (RAW_OVERLAY_MUX_WS !== ws) return;
    RAW_OVERLAY_MUX_WS = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    if (!helloReceived) {
      RAW_OVERLAY_MUX_DISABLED = true;
      rawOverlayConnectAllLegacy();
      return;
    }
    rawOverlayScheduleMultiplexReconnect(1000);
  };
}

function rawOverlayConnectAll() {
  invalidateRawDecodeState("overlay");
  setCarrotVisionState({ raw: { overlay: "connecting" } }, { reason: "raw overlay connecting" });
  if (!RAW_OVERLAY_MUX_DISABLED) {
    rawOverlayConnectMultiplex();
    if (RAW_OVERLAY_MUX_WS) return;
  }
  rawOverlayConnectAllLegacy();
}

async function rawHudDisconnectAll() {
  invalidateRawDecodeState("hud");
  setCarrotVisionState({ raw: { hud: "idle" } }, { reason: "raw hud idle" });
  _hudStopRenderLoop();
  if (RAW_HUD_MUX_RETRY_T) {
    clearTimeout(RAW_HUD_MUX_RETRY_T);
    RAW_HUD_MUX_RETRY_T = null;
  }
  if (RAW_HUD_MUX_WS) rawClearWsWatchdog(RAW_HUD_MUX_WS);
  try { if (RAW_HUD_MUX_WS) RAW_HUD_MUX_WS.close(); } catch {}
  RAW_HUD_MUX_WS = null;
  for (const service of RAW_HUD_SERVICES) {
    if (RAW_HUD_RETRY_T[service]) {
      clearTimeout(RAW_HUD_RETRY_T[service]);
      RAW_HUD_RETRY_T[service] = null;
    }
    if (RAW_HUD_WS[service]) rawClearWsWatchdog(RAW_HUD_WS[service]);
    try { if (RAW_HUD_WS[service]) RAW_HUD_WS[service].close(); } catch {}
    RAW_HUD_WS[service] = null;
  }
}

async function rawOverlayDisconnectAll() {
  invalidateRawDecodeState("overlay");
  setCarrotVisionState({ raw: { overlay: "idle" } }, { reason: "raw overlay idle" });
  if (RAW_OVERLAY_MUX_RETRY_T) {
    clearTimeout(RAW_OVERLAY_MUX_RETRY_T);
    RAW_OVERLAY_MUX_RETRY_T = null;
  }
  if (RAW_OVERLAY_MUX_WS) rawClearWsWatchdog(RAW_OVERLAY_MUX_WS);
  try { if (RAW_OVERLAY_MUX_WS) RAW_OVERLAY_MUX_WS.close(); } catch {}
  RAW_OVERLAY_MUX_WS = null;
  for (const service of RAW_OVERLAY_SERVICES) {
    if (RAW_OVERLAY_RETRY_T[service]) {
      clearTimeout(RAW_OVERLAY_RETRY_T[service]);
      RAW_OVERLAY_RETRY_T[service] = null;
    }
    if (RAW_OVERLAY_WS[service]) rawClearWsWatchdog(RAW_OVERLAY_WS[service]);
    try { if (RAW_OVERLAY_WS[service]) RAW_OVERLAY_WS[service].close(); } catch {}
    RAW_OVERLAY_WS[service] = null;
  }
}

window.CarrotVisionRaw = {
  connectHud: rawHudConnectAll,
  connectOverlay: rawOverlayConnectAll,
  disconnectHud: rawHudDisconnectAll,
  disconnectOverlay: rawOverlayDisconnectAll,
  ensureDecodeWorker: ensureRawDecodeWorker,
  markHudDirty: _hudMarkDirty,
  scheduleHudRender: _hudScheduleRender,
  stopHudRenderLoop: _hudStopRenderLoop,
  updateHudFromCarPayload: drivingHudUpdateFromCarPayload,
  updateHudFromRawState: drivingHudUpdateFromRawState,
};
Object.assign(window, {
  drivingHudUpdateFromCarPayload,
  drivingHudUpdateFromRawState,
  ensureRawDecodeWorker,
  rawHudConnectAll,
  rawHudDisconnectAll,
  rawOverlayConnectAll,
  rawOverlayDisconnectAll,
  _hudMarkDirty,
  _hudScheduleRender,
  _hudStopRenderLoop,
});
