/* ---------- Home Runtime State ---------- */
function setHomeServerState(summary, detail = summary, tone = "idle") {
  void summary;
  void detail;
  void tone;
}

function summarizeLiveRuntimeState(payload, errorMessage = "") {
  if (!payload?.ok) {
    const summary = errorMessage || getUIText("reconnecting", "Reconnecting...");
    return {
      summary,
      detail: JSON.stringify({ ok: false, error: errorMessage || "live_runtime unavailable" }, null, 2),
      tone: "error",
    };
  }

  const ageMs = Number.isFinite(Number(payload.snapshotAgeMs)) ? Number(payload.snapshotAgeMs) : null;
  const servicesCount = payload.services && typeof payload.services === "object" ? Object.keys(payload.services).length : 0;
  const paramsCount = payload.runtime?.params && typeof payload.runtime.params === "object" ? Object.keys(payload.runtime.params).length : 0;
  const summary = ageMs != null && ageMs > 1500
    ? `${getUIText("reconnecting", "Reconnecting...")} ${Math.round(ageMs)}ms`
    : getUIText("connected", "Connected");
  const tone = ageMs != null && ageMs > 1500 ? "error" : "connected";
  const detail = JSON.stringify({
    ok: true,
    snapshotAgeMs: ageMs,
    servicesCount,
    paramsCount,
    meta: payload.meta || {},
  }, null, 2);

  return { summary, detail, tone };
}

const CARROT_LIVE_RUNTIME_STATE = {
  ok: false,
  meta: {},
  runtime: { params: {} },
  services: {},
  snapshotAgeMs: null,
  fetchedAtMs: 0,
  dataSignature: "",
};
window.CarrotLiveRuntimeState = CARROT_LIVE_RUNTIME_STATE;

let LIVE_RUNTIME_FETCH_T = null;
let LIVE_RUNTIME_FETCH_IN_FLIGHT = null;
let LIVE_RUNTIME_POLL_ACTIVE = false;
let LAST_HUD_PAYLOAD_SIGNATURE = "";
const RTC_STATS_POLL_MS = 2000;
const RTC_PERF_STATE = {
  active: false,
  collectedAtMs: 0,
  connectionState: "idle",
  iceConnectionState: "new",
  codec: "",
  inbound: null,
  video: null,
  error: "",
};
window.CarrotRtcPerf = RTC_PERF_STATE;
let RTC_STATS_T = null;

function isCarrotPageVisible() {
  return document.body?.dataset?.page === "carrot" && !document.hidden;
}

function shouldRunCarrotHudRealtime() {
  return isCarrotPageVisible();
}

function shouldRunCarrotVisionRealtime() {
  return isCarrotPageVisible() && window.CARROT_VISION_ACTIVE;
}

function emitCarrotRenderRequest(detail = {}) {
  window.dispatchEvent(new CustomEvent("carrot:render-request", { detail }));
}

function emitCarrotVisionChange(active) {
  window.dispatchEvent(new CustomEvent("carrot:visionchange", { detail: { active: Boolean(active) } }));
}

function getLiveRuntimeDataSignature(payload) {
  try {
    return JSON.stringify({
      meta: payload?.meta || {},
      runtime: payload?.runtime || { params: {} },
      services: payload?.services || {},
    });
  } catch {
    return "";
  }
}

async function fetchLiveRuntimeState(force = false) {
  if (LIVE_RUNTIME_FETCH_IN_FLIGHT) return LIVE_RUNTIME_FETCH_IN_FLIGHT;

  LIVE_RUNTIME_FETCH_IN_FLIGHT = (async () => {
    let shouldNotifyRender = Boolean(force);
    try {
      const suffix = force ? "?force=1" : "";
      const response = await fetch(`/api/live_runtime${suffix}`, { cache: "no-store" });
      const payload = await response.json();
      if (!payload?.ok) throw new Error(payload?.error || "live_runtime failed");

      const wasOk = CARROT_LIVE_RUNTIME_STATE.ok;
      const nextDataSignature = getLiveRuntimeDataSignature(payload);
      const dataChanged = nextDataSignature !== CARROT_LIVE_RUNTIME_STATE.dataSignature;

      CARROT_LIVE_RUNTIME_STATE.ok = true;
      if (dataChanged) {
        CARROT_LIVE_RUNTIME_STATE.meta = payload.meta || {};
        CARROT_LIVE_RUNTIME_STATE.runtime = payload.runtime || { params: {} };
        CARROT_LIVE_RUNTIME_STATE.services = payload.services || {};
        CARROT_LIVE_RUNTIME_STATE.dataSignature = nextDataSignature;
      }
      CARROT_LIVE_RUNTIME_STATE.snapshotAgeMs = Number.isFinite(Number(payload.snapshotAgeMs)) ? Number(payload.snapshotAgeMs) : null;
      CARROT_LIVE_RUNTIME_STATE.fetchedAtMs = Date.now();
      window.CarrotLiveRuntimeState = CARROT_LIVE_RUNTIME_STATE;
      const serverState = summarizeLiveRuntimeState(payload);
      setHomeServerState(serverState.summary, serverState.detail, serverState.tone);
      shouldNotifyRender = shouldNotifyRender || dataChanged || !wasOk;
	    } catch (_error) {
      const wasOk = CARROT_LIVE_RUNTIME_STATE.ok;
      CARROT_LIVE_RUNTIME_STATE.ok = false;
      CARROT_LIVE_RUNTIME_STATE.fetchedAtMs = Date.now();
      window.CarrotLiveRuntimeState = CARROT_LIVE_RUNTIME_STATE;
	      const serverState = summarizeLiveRuntimeState(null, _error?.message || "");
	      setHomeServerState(serverState.summary, serverState.detail, serverState.tone);
      shouldNotifyRender = shouldNotifyRender || wasOk;
	    } finally {
      if (shouldNotifyRender) {
        emitCarrotRenderRequest({ force: false, overlayDirty: true, hudDirty: true });
      }
	      LIVE_RUNTIME_FETCH_IN_FLIGHT = null;
	    }
  })();

  return LIVE_RUNTIME_FETCH_IN_FLIGHT;
}

function getLiveRuntimePollMs() {
  return shouldRunCarrotHudRealtime() ? 1000 : 3000;
}

function scheduleLiveRuntimeStateFetch(ms = getLiveRuntimePollMs()) {
  if (!LIVE_RUNTIME_POLL_ACTIVE) return;
  if (LIVE_RUNTIME_FETCH_T) clearTimeout(LIVE_RUNTIME_FETCH_T);
  LIVE_RUNTIME_FETCH_T = setTimeout(async () => {
    LIVE_RUNTIME_FETCH_T = null;
    if (!LIVE_RUNTIME_POLL_ACTIVE) return;
    await fetchLiveRuntimeState(false).catch(() => {});
    scheduleLiveRuntimeStateFetch(getLiveRuntimePollMs());
  }, ms);
}

function stopLiveRuntimeStateFetch() {
  LIVE_RUNTIME_POLL_ACTIVE = false;
  if (LIVE_RUNTIME_FETCH_T) {
    clearTimeout(LIVE_RUNTIME_FETCH_T);
    LIVE_RUNTIME_FETCH_T = null;
  }
}

function startLiveRuntimeStateFetch(force = false, ms = getLiveRuntimePollMs()) {
  LIVE_RUNTIME_POLL_ACTIVE = true;
  if (force) fetchLiveRuntimeState(true).catch(() => {});
  scheduleLiveRuntimeStateFetch(ms);
}

function resetRtcPerfState() {
  RTC_PERF_STATE.active = false;
  RTC_PERF_STATE.collectedAtMs = Date.now();
  RTC_PERF_STATE.connectionState = "idle";
  RTC_PERF_STATE.iceConnectionState = "new";
  RTC_PERF_STATE.codec = "";
  RTC_PERF_STATE.inbound = null;
  RTC_PERF_STATE.video = null;
  RTC_PERF_STATE.error = "";
  window.CarrotRtcPerf = RTC_PERF_STATE;
}

function stopRtcPerfPolling() {
  if (RTC_STATS_T) {
    clearTimeout(RTC_STATS_T);
    RTC_STATS_T = null;
  }
}

function readRtcVideoPlaybackQuality(video) {
  if (!video || typeof video.getVideoPlaybackQuality !== "function") return null;
  const quality = video.getVideoPlaybackQuality();
  if (!quality) return null;
  return {
    totalVideoFrames: Number(quality.totalVideoFrames ?? 0),
    droppedVideoFrames: Number(quality.droppedVideoFrames ?? 0),
    corruptedVideoFrames: Number(quality.corruptedVideoFrames ?? 0),
    creationTime: Number(quality.creationTime ?? 0),
    width: Number(video.videoWidth || 0),
    height: Number(video.videoHeight || 0),
    readyState: Number(video.readyState || 0),
    currentTime: Number(video.currentTime || 0),
  };
}

function extractRtcInboundVideoStats(statsReport, statsMap) {
  if (!statsReport) return { inbound: null, codec: "" };
  const codecReport = statsReport.codecId ? statsMap.get(statsReport.codecId) : null;
  return {
    codec: codecReport?.mimeType || codecReport?.id || "",
    inbound: {
      framesDecoded: Number(statsReport.framesDecoded ?? 0),
      framesDropped: Number(statsReport.framesDropped ?? 0),
      framesPerSecond: Number(statsReport.framesPerSecond ?? 0),
      frameWidth: Number(statsReport.frameWidth ?? 0),
      frameHeight: Number(statsReport.frameHeight ?? 0),
      jitter: Number(statsReport.jitter ?? 0),
      bytesReceived: Number(statsReport.bytesReceived ?? 0),
      packetsLost: Number(statsReport.packetsLost ?? 0),
      decoderImplementation: statsReport.decoderImplementation || "",
      freezeCount: Number(statsReport.freezeCount ?? 0),
      totalFreezesDuration: Number(statsReport.totalFreezesDuration ?? 0),
    },
  };
}

async function collectRtcPerfStats() {
  const pc = RTC_PC;
  if (!pc) return;

  try {
    const stats = await pc.getStats(null);
    let inboundVideoReport = null;
    stats.forEach((report) => {
      if (inboundVideoReport) return;
      if (report?.type === "inbound-rtp" && report.kind === "video" && !report.isRemote) {
        inboundVideoReport = report;
      }
    });

    const video = document.getElementById("rtcVideo");
    const inbound = extractRtcInboundVideoStats(inboundVideoReport, stats);
    RTC_PERF_STATE.active = shouldRunCarrotVisionRealtime();
    RTC_PERF_STATE.collectedAtMs = Date.now();
    RTC_PERF_STATE.connectionState = pc.connectionState || "unknown";
    RTC_PERF_STATE.iceConnectionState = pc.iceConnectionState || "unknown";
    RTC_PERF_STATE.codec = inbound.codec;
    RTC_PERF_STATE.inbound = inbound.inbound;
    RTC_PERF_STATE.video = readRtcVideoPlaybackQuality(video);
    RTC_PERF_STATE.error = "";
    window.CarrotRtcPerf = RTC_PERF_STATE;
  } catch (error) {
    RTC_PERF_STATE.active = shouldRunCarrotVisionRealtime();
    RTC_PERF_STATE.collectedAtMs = Date.now();
    RTC_PERF_STATE.connectionState = pc.connectionState || "unknown";
    RTC_PERF_STATE.iceConnectionState = pc.iceConnectionState || "unknown";
    RTC_PERF_STATE.error = error?.message || String(error);
    window.CarrotRtcPerf = RTC_PERF_STATE;
  }
}

function scheduleRtcPerfPolling(ms = RTC_STATS_POLL_MS) {
  if (RTC_STATS_T || !shouldRunCarrotVisionRealtime()) return;
  RTC_STATS_T = setTimeout(async () => {
    RTC_STATS_T = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    await collectRtcPerfStats().catch(() => {});
    scheduleRtcPerfPolling(RTC_STATS_POLL_MS);
  }, ms);
}

function startRtcPerfPolling(force = false) {
  if (force) collectRtcPerfStats().catch(() => {});
  scheduleRtcPerfPolling(force ? 500 : RTC_STATS_POLL_MS);
}


// ===== WebRTC (auto) =====
let RTC_PC = null;
let RTC_RETRY_T = null;
let RTC_WAIT_TRACK_T = null;
let RTC_FAIL_COUNT = 0;
function rtcHasLiveTrack() {
  const video = document.getElementById("rtcVideo");
  return Boolean(video && video.srcObject);
}

function rtcStatusSet(s) {
  const el = document.getElementById("rtcStatus");
  if (el) el.textContent = String(s);
}

function rtcCancelRetry() {
  if (RTC_RETRY_T) {
    clearTimeout(RTC_RETRY_T);
    RTC_RETRY_T = null;
  }
}

async function rtcDisconnect() {
  rtcCancelRetry();
  rtcDisarmTrackTimeout();
  stopRtcPerfPolling();
  try { if (RTC_PC) RTC_PC.close(); } catch {}
  RTC_PC = null;
  resetRtcPerfState();

  const video = document.getElementById("rtcVideo");
  if (video) {
    video.srcObject = null;
    video.style.display = "none";
  }
}

function rtcScheduleRetry(ms = 2000) {
  if (!shouldRunCarrotVisionRealtime()) return;
  rtcCancelRetry();
  const backoff = Math.min(ms * Math.pow(1.5, RTC_FAIL_COUNT), 30000);
  RTC_FAIL_COUNT = Math.min(RTC_FAIL_COUNT + 1, 20);
  RTC_RETRY_T = setTimeout(async () => {
    RTC_RETRY_T = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    await rtcConnectOnce().catch(() => {});
  }, backoff);
}

function rtcArmTrackTimeout(ms = 5000) {
  if (RTC_WAIT_TRACK_T) clearTimeout(RTC_WAIT_TRACK_T);
  RTC_WAIT_TRACK_T = setTimeout(async () => {
    RTC_WAIT_TRACK_T = null;
    rtcStatusSet("no track, retry...");
    await rtcDisconnect();
    rtcScheduleRetry(2000);
  }, ms);
}

function rtcDisarmTrackTimeout() {
  if (RTC_WAIT_TRACK_T) {
    clearTimeout(RTC_WAIT_TRACK_T);
    RTC_WAIT_TRACK_T = null;
  }
}

async function waitIceComplete(pc, timeoutMs = 8000) {
  if (pc.iceGatheringState === "complete") return;
  await new Promise((resolve) => {
    const t = setTimeout(resolve, timeoutMs);
    function onchg() {
      if (pc.iceGatheringState === "complete") {
        pc.removeEventListener("icegatheringstatechange", onchg);
        clearTimeout(t);
        resolve();
      }
    }
    pc.addEventListener("icegatheringstatechange", onchg);
  });
}

let _rtcConnecting = false;

async function rtcConnectOnce() {
  if (!shouldRunCarrotVisionRealtime()) return;
  if (_rtcConnecting) return;
  if (RTC_PC && (RTC_PC.connectionState === "connected" || RTC_PC.connectionState === "connecting")) return;

  _rtcConnecting = true;
  try {
    await rtcDisconnect();
    rtcStatusSet("connecting...");

    const pc = new RTCPeerConnection({
      iceServers: [],
      sdpSemantics: "unified-plan",
      iceCandidatePoolSize: 1,
    });
    RTC_PC = pc;
    startRtcPerfPolling(true);

    const video = document.getElementById("rtcVideo");
    if (video) {
      video.muted = true;
      video.playsInline = true;
    }

    pc.addTransceiver("video", { direction: "recvonly" });

    pc.ontrack = async (ev) => {
      const videoEl = document.getElementById("rtcVideo");
      if (!videoEl) return;

      let stream = ev.streams && ev.streams[0];
      if (!stream) {
        stream = new MediaStream([ev.track]);
      }

      videoEl.srcObject = stream;
      videoEl.style.display = "block";
      try { await videoEl.play(); } catch (e) { console.log("[RTC] play() failed", e); }
      rtcStatusSet("track: " + ev.track.kind);
      rtcDisarmTrackTimeout();
      RTC_FAIL_COUNT = 0;
      collectRtcPerfStats().catch(() => {});
    };

    pc.onconnectionstatechange = () => {
      const state = pc.connectionState;
      rtcStatusSet("conn: " + state);
      if (state === "connected") RTC_FAIL_COUNT = 0;
      collectRtcPerfStats().catch(() => {});
      if (state === "failed" || state === "disconnected" || state === "closed") {
        rtcDisconnect();
        rtcScheduleRetry(2000);
      }
    };

    pc.oniceconnectionstatechange = () => {
      const state = pc.iceConnectionState;
      rtcStatusSet("ice: " + state);
      collectRtcPerfStats().catch(() => {});
      if (state === "failed" || state === "disconnected" || state === "closed") {
        rtcDisconnect();
        rtcScheduleRetry(2000);
      }
    };

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    await waitIceComplete(pc, 8000);

    const body = {
      sdp: pc.localDescription.sdp,
      cameras: ["road"],
      bridge_services_in: [],
      bridge_services_out: [],
    };

    const response = await fetch("/stream", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });

    if (!response.ok) {
      const text = await response.text().catch(() => "");
      throw new Error("stream http " + response.status + " " + text);
    }

    const answer = await response.json();
    if (!answer || !answer.sdp) throw new Error("bad answer");

    await pc.setRemoteDescription({ type: answer.type || "answer", sdp: answer.sdp });
    rtcStatusSet("connected (waiting track...)");
    rtcArmTrackTimeout(6000);
  } catch (e) {
    rtcStatusSet("error: " + e.message);
    await rtcDisconnect();
    rtcScheduleRetry(2000);
  } finally {
    _rtcConnecting = false;
  }
}

async function waitServerReady(timeoutMs = 8000) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    try {
      const r = await fetch("/api/settings", { cache: "no-store" });
      if (r.ok) return true;
    } catch {}
    await new Promise(res => setTimeout(res, 300));
  }
  return false;
}

window.CARROT_VISION_ACTIVE = false;

window.CarrotVisionStart = async function() {
  if (window.CARROT_VISION_ACTIVE) return;
  window.CARROT_VISION_ACTIVE = true;
  emitCarrotVisionChange(true);
  
  const btn = document.getElementById("visionStartOverlay");
  if (btn) btn.style.display = "none";
  
  rtcStatusSet("waiting server...");
  await waitServerReady(8000);
  syncCarrotRealtimeLifecycle(true);
};

function rtcInitAuto() {
  const btn = document.getElementById("btnStartVision");
  if (btn) btn.onclick = window.CarrotVisionStart;
}

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
const RAW_DECODE_SEQ = {
  hud: Object.create(null),
  overlay: Object.create(null),
};

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

function nextRawDecodeSeq(streamKind, service) {
  const bucket = RAW_DECODE_SEQ[streamKind];
  const next = Number(bucket[service] || 0) + 1;
  bucket[service] = next;
  return next;
}

function isLatestRawDecodeSeq(streamKind, service, seq) {
  return RAW_DECODE_SEQ[streamKind]?.[service] === seq;
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

async function handleHudDecodedMessage(service, data) {
  const seq = nextRawDecodeSeq("hud", service);
  const decoded = await decodeRawMessage("hud", service, data);
  if (!decoded) return;
  if (!isLatestRawDecodeSeq("hud", service, seq)) return;
  RAW_HUD_STATE[service] = decoded;
  window.CarrotHudState = RAW_HUD_STATE;
  _hudMarkDirty();
  emitCarrotRenderRequest({
    hudDirty: true,
    overlayDirty: RAW_HUD_OVERLAY_DIRTY_SERVICES.has(service),
  });
}

async function handleOverlayDecodedMessage(service, data) {
  const seq = nextRawDecodeSeq("overlay", service);
  const decoded = await decodeRawMessage("overlay", service, data);
  if (!decoded) return;
  if (!isLatestRawDecodeSeq("overlay", service, seq)) return;
  RAW_OVERLAY_STATE[service] = decoded;
  window.CarrotOverlayState = RAW_OVERLAY_STATE;
  emitCarrotRenderRequest({
    hudDirty: true,
    overlayDirty: !RAW_OVERLAY_HUD_ONLY_SERVICES.has(service),
  });
}

function drivingHudUpdateFromCarPayload(j) {
  if (!window.DrivingHud || !j) return;

  const vEgoKph = (typeof j.vEgo === "number" && isFinite(j.vEgo)) ? j.vEgo * 3.6 : null;
  const payload = {
    cpuTempC: j.cpuTempC,
    memPct: j.memPct,
    diskPct: j.diskPct,
    diskLabel: j.diskLabel,
    vEgoKph,
    vSetKph: j.vSetKph,
    temp: j.temp,
    redDot: j.redDot,
    tlight: j.tlight,
    tfGap: j.tfGap,
    tfBars: j.tfBars,
    gear: j.gear,
    gpsOk: j.gpsOk,
    driveMode: j.driveMode,
    speedLimitKph: j.speedLimitKph,
    speedLimitOver: j.speedLimitOver,
    speedLimitBlink: j.speedLimitBlink,
    apm: j.apm,
  };

  const payloadSignature = [
    typeof LANG === "string" ? LANG : "",
    payload.cpuTempC ?? "-",
    payload.memPct ?? "-",
    payload.diskPct ?? "-",
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

function drivingHudUpdateFromRawState() {
  const payload = window.CarrotRawCapnp?.deriveHudPayload?.(RAW_HUD_STATE);
  if (!payload) return;
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
      await handleHudDecodedMessage(service, ev.data);
    } catch (e) {
      console.log(RAW_HUD_LOG_PREFIX, service, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_HUD_LOG_PREFIX, service, "error", e);
  };

  ws.onclose = () => {
    console.log(RAW_HUD_LOG_PREFIX, service, "close -> reconnect");
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
  const ws = new WebSocket(buildRawMultiplexUrl(RAW_HUD_SERVICES));
  ws.binaryType = "arraybuffer";
  RAW_HUD_MUX_WS = ws;

  ws.onopen = () => {
    console.log(RAW_HUD_MUX_LOG_PREFIX, "open");
  };

  ws.onmessage = async (ev) => {
    try {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        console.log(RAW_HUD_MUX_LOG_PREFIX, "hello", hello);
        helloReceived = hello?.mode === RAW_MULTIPLEX_MODE;
        return;
      }
      const frame = await parseRawMultiplexFrame(ev.data);
      if (!RAW_HUD_SERVICES.includes(frame.service)) return;
      await handleHudDecodedMessage(frame.service, frame.payload);
    } catch (e) {
      console.log(RAW_HUD_MUX_LOG_PREFIX, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_HUD_MUX_LOG_PREFIX, "error", e);
  };

  ws.onclose = () => {
    console.log(RAW_HUD_MUX_LOG_PREFIX, "close");
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
      await handleOverlayDecodedMessage(service, ev.data);
    } catch (e) {
      console.log(RAW_OVERLAY_LOG_PREFIX, service, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_OVERLAY_LOG_PREFIX, service, "error", e);
  };

  ws.onclose = () => {
    console.log(RAW_OVERLAY_LOG_PREFIX, service, "close -> reconnect");
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
  const ws = new WebSocket(buildRawMultiplexUrl(RAW_OVERLAY_SERVICES));
  ws.binaryType = "arraybuffer";
  RAW_OVERLAY_MUX_WS = ws;

  ws.onopen = () => {
    console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "open");
  };

  ws.onmessage = async (ev) => {
    try {
      if (typeof ev.data === "string") {
        const hello = JSON.parse(ev.data);
        console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "hello", hello);
        helloReceived = hello?.mode === RAW_MULTIPLEX_MODE;
        return;
      }
      const frame = await parseRawMultiplexFrame(ev.data);
      if (!RAW_OVERLAY_SERVICES.includes(frame.service)) return;
      await handleOverlayDecodedMessage(frame.service, frame.payload);
    } catch (e) {
      console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "bad msg", e);
    }
  };

  ws.onerror = (e) => {
    console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "error", e);
  };

  ws.onclose = () => {
    console.log(RAW_OVERLAY_MUX_LOG_PREFIX, "close");
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
  if (!RAW_OVERLAY_MUX_DISABLED) {
    rawOverlayConnectMultiplex();
    if (RAW_OVERLAY_MUX_WS) return;
  }
  rawOverlayConnectAllLegacy();
}

async function rawHudDisconnectAll() {
  _hudStopRenderLoop();
  if (RAW_HUD_MUX_RETRY_T) {
    clearTimeout(RAW_HUD_MUX_RETRY_T);
    RAW_HUD_MUX_RETRY_T = null;
  }
  try { if (RAW_HUD_MUX_WS) RAW_HUD_MUX_WS.close(); } catch {}
  RAW_HUD_MUX_WS = null;
  for (const service of RAW_HUD_SERVICES) {
    if (RAW_HUD_RETRY_T[service]) {
      clearTimeout(RAW_HUD_RETRY_T[service]);
      RAW_HUD_RETRY_T[service] = null;
    }
    try { if (RAW_HUD_WS[service]) RAW_HUD_WS[service].close(); } catch {}
    RAW_HUD_WS[service] = null;
  }
}

async function rawOverlayDisconnectAll() {
  if (RAW_OVERLAY_MUX_RETRY_T) {
    clearTimeout(RAW_OVERLAY_MUX_RETRY_T);
    RAW_OVERLAY_MUX_RETRY_T = null;
  }
  try { if (RAW_OVERLAY_MUX_WS) RAW_OVERLAY_MUX_WS.close(); } catch {}
  RAW_OVERLAY_MUX_WS = null;
  for (const service of RAW_OVERLAY_SERVICES) {
    if (RAW_OVERLAY_RETRY_T[service]) {
      clearTimeout(RAW_OVERLAY_RETRY_T[service]);
      RAW_OVERLAY_RETRY_T[service] = null;
    }
    try { if (RAW_OVERLAY_WS[service]) RAW_OVERLAY_WS[service].close(); } catch {}
    RAW_OVERLAY_WS[service] = null;
  }
}

async function syncServerTimeOnConnect() {
  try {
    const timezone = Intl.DateTimeFormat().resolvedOptions().timeZone || "UTC";
    const body = {
      epoch_ms: Date.now(),
      client_iso: new Date().toISOString(),
      timezone,
      debug: true, // server side debug print enabled
    };

    console.log("[time_sync] request", body);
    const r = await fetch("/api/time_sync", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const j = await r.json().catch(() => ({}));
    console.log("[time_sync] response", j);
    return j;
  } catch (e) {
    console.log("[time_sync] failed", e);
    return { ok: false, error: String(e) };
  }
}

function startAll() {
  renderUIText();
  showPage("carrot", false);
  console.log("[time_sync] syncing server time on page load");
  syncServerTimeOnConnect().catch(() => {});
  rtcInitAuto();
  ensureRawDecodeWorker();

  if (window.DrivingHud) {
    window.DrivingHud.init();
    _hudMarkDirty();
  }
  syncCarrotRealtimeLifecycle(false);
}

let _carrotHudRealtimeActive = false;
let _carrotVisionRealtimeActive = false;

function syncCarrotRealtimeLifecycle(forceFetch = false) {
  const nextHudActive = shouldRunCarrotHudRealtime();
  const nextVisionActive = shouldRunCarrotVisionRealtime();

  if (nextHudActive === _carrotHudRealtimeActive && nextVisionActive === _carrotVisionRealtimeActive && !forceFetch) {
    if (nextVisionActive && !_rtcConnecting && !rtcHasLiveTrack()) {
      rtcConnectOnce().catch(() => {});
    }
    if (nextHudActive) _hudScheduleRender();
    return;
  }

  _carrotHudRealtimeActive = nextHudActive;
  _carrotVisionRealtimeActive = nextVisionActive;

  if (nextHudActive) {
    console.log("[perf] carrot hud realtime -> active");
    ensureRawDecodeWorker();
    rawHudConnectAll();
    startLiveRuntimeStateFetch(forceFetch, 100);
    _hudMarkDirty();
  } else {
    console.log("[perf] carrot hud realtime -> idle");
    _hudStopRenderLoop();
    stopLiveRuntimeStateFetch();
    rawHudDisconnectAll();
  }

  if (nextVisionActive) {
    console.log("[perf] carrot vision realtime -> active");
    ensureRawDecodeWorker();
    rawOverlayConnectAll();
    startRtcPerfPolling(true);
    if (!_rtcConnecting && !rtcHasLiveTrack()) {
      rtcCancelRetry();
      RTC_FAIL_COUNT = 0;
      rtcConnectOnce().catch(() => {});
    }
  } else {
    console.log("[perf] carrot vision realtime -> idle");
    stopRtcPerfPolling();
    rawOverlayDisconnectAll();
    rtcDisconnect().catch(() => {});
  }

  emitCarrotRenderRequest({ force: true, overlayDirty: true, hudDirty: true });
}

document.addEventListener("visibilitychange", () => syncCarrotRealtimeLifecycle(false));
window.addEventListener("carrot:pagechange", () => syncCarrotRealtimeLifecycle(false));



if (document.readyState === "loading") {
  window.addEventListener("DOMContentLoaded", startAll);
} else {
  startAll();
}
