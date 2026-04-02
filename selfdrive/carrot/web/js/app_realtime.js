/* ---------- Home Runtime State ---------- */
function setHomeServerState(summary, detail = summary, tone = "idle") {
  if (typeof setServerStateStatus === "function") {
    setServerStateStatus(summary, detail, tone);
    return;
  }

  const box = document.getElementById("stateBox");
  if (box) box.textContent = String(detail || summary || "");
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
};
window.CarrotLiveRuntimeState = CARROT_LIVE_RUNTIME_STATE;

let LIVE_RUNTIME_FETCH_T = null;
let LIVE_RUNTIME_FETCH_IN_FLIGHT = null;

async function fetchLiveRuntimeState(force = false) {
  if (LIVE_RUNTIME_FETCH_IN_FLIGHT) return LIVE_RUNTIME_FETCH_IN_FLIGHT;

  LIVE_RUNTIME_FETCH_IN_FLIGHT = (async () => {
    try {
      const suffix = force ? "?force=1" : "";
      const response = await fetch(`/api/live_runtime${suffix}`, { cache: "no-store" });
      const payload = await response.json();
      if (!payload?.ok) throw new Error(payload?.error || "live_runtime failed");

      CARROT_LIVE_RUNTIME_STATE.ok = true;
      CARROT_LIVE_RUNTIME_STATE.meta = payload.meta || {};
      CARROT_LIVE_RUNTIME_STATE.runtime = payload.runtime || { params: {} };
      CARROT_LIVE_RUNTIME_STATE.services = payload.services || {};
      CARROT_LIVE_RUNTIME_STATE.snapshotAgeMs = Number.isFinite(Number(payload.snapshotAgeMs)) ? Number(payload.snapshotAgeMs) : null;
      CARROT_LIVE_RUNTIME_STATE.fetchedAtMs = Date.now();
      window.CarrotLiveRuntimeState = CARROT_LIVE_RUNTIME_STATE;
      const serverState = summarizeLiveRuntimeState(payload);
      setHomeServerState(serverState.summary, serverState.detail, serverState.tone);
    } catch (_error) {
      CARROT_LIVE_RUNTIME_STATE.ok = false;
      CARROT_LIVE_RUNTIME_STATE.fetchedAtMs = Date.now();
      window.CarrotLiveRuntimeState = CARROT_LIVE_RUNTIME_STATE;
      const serverState = summarizeLiveRuntimeState(null, _error?.message || "");
      setHomeServerState(serverState.summary, serverState.detail, serverState.tone);
    } finally {
      LIVE_RUNTIME_FETCH_IN_FLIGHT = null;
    }
  })();

  return LIVE_RUNTIME_FETCH_IN_FLIGHT;
}

function getLiveRuntimePollMs() {
  const carrotVisible = document.body?.dataset?.page === "carrot" && !document.hidden;
  return carrotVisible ? 350 : 1500;
}

function scheduleLiveRuntimeStateFetch(ms = getLiveRuntimePollMs()) {
  if (LIVE_RUNTIME_FETCH_T) clearTimeout(LIVE_RUNTIME_FETCH_T);
  LIVE_RUNTIME_FETCH_T = setTimeout(async () => {
    LIVE_RUNTIME_FETCH_T = null;
    await fetchLiveRuntimeState(false).catch(() => {});
    scheduleLiveRuntimeStateFetch(getLiveRuntimePollMs());
  }, ms);
}

fetchLiveRuntimeState(true).catch(() => {});
scheduleLiveRuntimeStateFetch(1500);


// ===== WebRTC (auto) =====
let RTC_PC = null;
let RTC_RETRY_T = null;
let RTC_WAIT_TRACK_T = null;
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
  try { if (RTC_PC) RTC_PC.close(); } catch {}
  RTC_PC = null;

  const video = document.getElementById("rtcVideo");
  if (video) {
    video.srcObject = null;
    video.style.display = "none";
  }
}

function rtcScheduleRetry(ms = 2000) {
  rtcCancelRetry();
  RTC_RETRY_T = setTimeout(async () => {
    RTC_RETRY_T = null;
    await rtcConnectOnce().catch(() => {});
  }, ms);
}

function rtcArmTrackTimeout(ms = 5000) {
  if (RTC_WAIT_TRACK_T) clearTimeout(RTC_WAIT_TRACK_T);
  RTC_WAIT_TRACK_T = setTimeout(async () => {
    RTC_WAIT_TRACK_T = null;
    rtcStatusSet("no track, retry...");
    await rtcDisconnect();
    rtcScheduleRetry(1000);
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

async function rtcConnectOnce() {
  if (RTC_PC && (RTC_PC.connectionState === "connected" || RTC_PC.connectionState === "connecting")) return;

  try {
    await rtcDisconnect();
    rtcStatusSet("connecting...");

    const pc = new RTCPeerConnection({
      iceServers: [],
      sdpSemantics: "unified-plan",
      iceCandidatePoolSize: 1,
    });
    RTC_PC = pc;

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
    };

    pc.onconnectionstatechange = () => {
      const state = pc.connectionState;
      console.log("[RTC] connectionState:", state);
      rtcStatusSet("conn: " + state);
      if (state === "failed" || state === "disconnected" || state === "closed") {
        rtcDisconnect();
        rtcScheduleRetry(2000);
      }
    };

    pc.oniceconnectionstatechange = () => {
      const state = pc.iceConnectionState;
      console.log("[RTC] iceConnectionState:", state);
      rtcStatusSet("ice: " + state);
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
    throw e;
  }
}

async function waitServerReady(timeoutMs = 8000) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    try {
      // ���� ����ִ����� Ȯ�� (������ API)
      const r = await fetch("/api/settings", { cache: "no-store" });
      if (r.ok) return true;
    } catch {}
    await new Promise(res => setTimeout(res, 300));
  }
  return false;
}

function rtcInitAuto() {
  (async () => {
    rtcStatusSet("waiting server...");
    await waitServerReady(8000);   // �����ص� ��� ����
    await rtcConnectOnce().catch(() => {});
  })();

  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) rtcConnectOnce().catch(() => {});
  });
}

const RAW_HUD_LOG_PREFIX = "[raw hud]";
const RAW_HUD_SERVICES = window.CarrotRawCapnp?.HUD_SERVICES || [];
const RAW_HUD_WS = Object.create(null);
const RAW_HUD_RETRY_T = Object.create(null);
const RAW_HUD_STATE = Object.create(null);
window.CarrotHudState = RAW_HUD_STATE;
const RAW_OVERLAY_LOG_PREFIX = "[raw overlay]";
const RAW_OVERLAY_SERVICES = window.CarrotRawCapnp?.OVERLAY_SERVICES || [];
const RAW_OVERLAY_WS = Object.create(null);
const RAW_OVERLAY_RETRY_T = Object.create(null);
const RAW_OVERLAY_STATE = Object.create(null);
window.CarrotOverlayState = RAW_OVERLAY_STATE;

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
    apm: j.apm,
  };

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
}

function _hudRenderLoop() {
  _hudRafId = requestAnimationFrame(_hudRenderLoop);
  if (!_hudRenderDirty) return;
  _hudRenderDirty = false;
  drivingHudUpdateFromRawState();
}

function _hudStartRenderLoop() {
  if (_hudRafId != null) return;
  _hudRafId = requestAnimationFrame(_hudRenderLoop);
}

function _hudStopRenderLoop() {
  if (_hudRafId != null) {
    cancelAnimationFrame(_hudRafId);
    _hudRafId = null;
  }
  _hudRenderDirty = false;
}

function rawHudScheduleReconnect(service, ms = 1000) {
  if (RAW_HUD_RETRY_T[service]) return;
  RAW_HUD_RETRY_T[service] = setTimeout(() => {
    RAW_HUD_RETRY_T[service] = null;
    rawHudConnectService(service);
  }, ms);
}
function rawHudConnectService(service) {
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
      const buffer = ev.data instanceof Blob ? await ev.data.arrayBuffer() : ev.data;
      const decoded = window.CarrotRawCapnp?.decodeHudEvent?.(service, buffer);
      if (!decoded) return;
      RAW_HUD_STATE[service] = decoded;
      window.CarrotHudState = RAW_HUD_STATE;
      _hudMarkDirty();
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

function rawHudConnectAll() {
  for (const service of RAW_HUD_SERVICES) {
    rawHudConnectService(service);
  }
  _hudStartRenderLoop();
}

function rawOverlayScheduleReconnect(service, ms = 1000) {
  if (RAW_OVERLAY_RETRY_T[service]) return;
  RAW_OVERLAY_RETRY_T[service] = setTimeout(() => {
    RAW_OVERLAY_RETRY_T[service] = null;
    rawOverlayConnectService(service);
  }, ms);
}

function rawOverlayConnectService(service) {
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
      const buffer = ev.data instanceof Blob ? await ev.data.arrayBuffer() : ev.data;
      const decoded = window.CarrotRawCapnp?.decodeOverlayEvent?.(service, buffer);
      if (!decoded) return;
      RAW_OVERLAY_STATE[service] = decoded;
      window.CarrotOverlayState = RAW_OVERLAY_STATE;
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

function rawOverlayConnectAll() {
  for (const service of RAW_OVERLAY_SERVICES) {
    rawOverlayConnectService(service);
  }
}

async function rawHudDisconnectAll() {
  _hudStopRenderLoop();
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
  showPage("home", false);
  console.log("[time_sync] syncing server time on page load");
  syncServerTimeOnConnect().catch(() => {});
  rtcInitAuto();
  updateQuickLink().catch(() => {});

  if (window.DrivingHud) {
    window.DrivingHud.init();
    _hudMarkDirty();
  }

  rawHudConnectAll();
  rawOverlayConnectAll();
}



if (document.readyState === "loading") {
  window.addEventListener("DOMContentLoaded", startAll);
} else {
  startAll();
}
