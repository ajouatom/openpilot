/* ---------- Home WS state ---------- */
function setHomeServerState(summary, detail = summary, tone = "idle") {
  if (typeof setServerStateStatus === "function") {
    setServerStateStatus(summary, detail, tone);
    return;
  }

  const box = document.getElementById("stateBox");
  if (box) box.textContent = String(detail || summary || "");
}

function summarizeServerStatePayload(payload) {
  const statusText = [
    payload?.status,
    payload?.state,
    payload?.message,
    payload?.result,
    payload?.summary,
    payload?.text,
  ].find((value) => value != null && String(value).trim() !== "");

  const hasError = Boolean(payload?.error || payload?.error_code || payload?.ok === false);
  let summary = statusText ? String(statusText) : (hasError ? getUIText("error", "Error") : getUIText("connected", "Connected"));
  if (summary.length > 72) summary = `${summary.slice(0, 69)}...`;

  return {
    summary,
    tone: hasError ? "error" : "connected",
  };
}

function wsConnect() {
  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  const ws = new WebSocket(wsProto + "://" + location.host + "/ws/state");
  ws.onopen = () => {
    const connected = getUIText("connected", "Connected");
    setHomeServerState(connected, connected, "connected");
  };
  ws.onmessage = (ev) => {
    try {
      const j = JSON.parse(ev.data);
      const { summary, tone } = summarizeServerStatePayload(j);
      setHomeServerState(summary, JSON.stringify(j, null, 2), tone);
    } catch (e) {
      const text = String(ev.data || "").trim() || getUIText("connected", "Connected");
      setHomeServerState(text, text, "connected");
    }
  };
  ws.onerror = () => {
    const error = getUIText("error", "Error");
    setHomeServerState(error, error, "error");
  };
  ws.onclose = () => {
    const reconnecting = getUIText("reconnecting", "Reconnecting...");
    setHomeServerState(reconnecting, reconnecting, "error");
    setTimeout(wsConnect, 1000);
  };
}
wsConnect();


// ===== WebRTC (auto) =====
let RTC_PC = null;
let RTC_RETRY_T = null;
const RTC_DEV_PREVIEW_ENABLED = location.hostname === "localhost" || location.hostname === "127.0.0.1";
const RTC_DEV_PREVIEW_KEY = "carrot.rtc.devPreview";
let RTC_DEV_PREVIEW = false;

function rtcHasLiveTrack() {
  const v = document.getElementById("rtcVideo");
  return Boolean(v && v.srcObject);
}

function rtcSyncDevToggleUi() {
  const tools = document.getElementById("rtcDevTools");
  const btn = document.getElementById("btnRtcDevToggle");
  if (!tools || !btn) return;

  tools.hidden = !RTC_DEV_PREVIEW_ENABLED;
  btn.classList.toggle("is-on", RTC_DEV_PREVIEW);
  btn.setAttribute("aria-pressed", RTC_DEV_PREVIEW ? "true" : "false");
  btn.textContent = RTC_DEV_PREVIEW ? "RTC Preview ON" : "RTC Preview";
}

function rtcApplyDevPreview() {
  const rtcCard = document.getElementById("rtcCard");
  const rtcWrap = document.getElementById("rtcWrap");
  const v = document.getElementById("rtcVideo");
  if (!rtcCard || !rtcWrap || !v) return;

  const showPreview = RTC_DEV_PREVIEW_ENABLED && RTC_DEV_PREVIEW && !rtcHasLiveTrack();
  rtcWrap.classList.toggle("rtcWrap--dev-preview", showPreview);
  v.classList.toggle("rtcVideo--dev-preview", showPreview);

  if (showPreview) {
    rtcCard.style.display = "block";
    v.style.display = "block";
    rtcStatusSet("dev preview");
  } else if (!rtcHasLiveTrack()) {
    v.style.display = "none";
    rtcCard.style.display = "none";
  }

  hudAutoDock();
  rtcSyncDevToggleUi();
}

function rtcSetDevPreview(next) {
  RTC_DEV_PREVIEW = Boolean(next && RTC_DEV_PREVIEW_ENABLED);
  try {
    if (RTC_DEV_PREVIEW) localStorage.setItem(RTC_DEV_PREVIEW_KEY, "1");
    else localStorage.removeItem(RTC_DEV_PREVIEW_KEY);
  } catch {}
  rtcApplyDevPreview();
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
  rtcCancelRetry(); // �߰�
  try { if (RTC_PC) RTC_PC.close(); } catch {}
  RTC_PC = null;
  const v = document.getElementById("rtcVideo");
  if (v) { v.srcObject = null; v.style.display = "none"; }
  const rtcCard = document.getElementById("rtcCard");
  rtcCard.style.display = "none";
  rtcApplyDevPreview();

  // HUD auto dock handled by hudAutoDock()
  //await carWsDisconnect();
}

function rtcScheduleRetry(ms = 2000) {
  rtcCancelRetry(); // �׻� ���� ��´�
  RTC_RETRY_T = setTimeout(async () => {
    RTC_RETRY_T = null;
    await rtcConnectOnce().catch(() => {});
  }, ms);
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

let RTC_WAIT_TRACK_T = null;

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

async function rtcConnectOnce() {
  if (RTC_PC && (RTC_PC.connectionState === "connected" || RTC_PC.connectionState === "connecting")) return;

  try {
    await rtcDisconnect();
    rtcStatusSet("connecting...");

    const pc = new RTCPeerConnection({
      iceServers: [],
      sdpSemantics: "unified-plan",
      iceCandidatePoolSize: 1
    });
    RTC_PC = pc;

    const v = document.getElementById("rtcVideo");
    if (v) { v.muted = true; v.playsInline = true; }

    const dbg = (...a) => console.log("[RTC]", ...a);

    pc.addTransceiver("video", { direction: "recvonly" });

    pc.ontrack = async (ev) => {
      const rtcCard = document.getElementById("rtcCard");
      const v = document.getElementById("rtcVideo");
      if (!v) return;

      let stream = ev.streams && ev.streams[0];
      if (!stream) {
        stream = new MediaStream([ev.track]);
      }

      v.srcObject = stream;
      v.classList.remove("rtcVideo--dev-preview");
      const rtcWrap = document.getElementById("rtcWrap");
      if (rtcWrap) rtcWrap.classList.remove("rtcWrap--dev-preview");
      v.style.display = "block";
      rtcCard.style.display = "block";
      try { await v.play(); } catch(e) { console.log("[RTC] play() failed", e); }
      rtcStatusSet("track: " + ev.track.kind);
      rtcDisarmTrackTimeout();

      hudAutoDock();
      carWsConnect();
    };

    pc.onconnectionstatechange = () => {
      const st = pc.connectionState;
      dbg("connectionState:", st);
      rtcStatusSet("conn: " + st);
      if (st === "failed" || st === "disconnected" || st === "closed") {
        rtcDisconnect();
        rtcScheduleRetry(2000);
      }
    };

    pc.oniceconnectionstatechange = () => {
      const st = pc.iceConnectionState;
      dbg("iceConnectionState:", st);
      rtcStatusSet("ice: " + st);
      if (st === "failed" || st === "disconnected" || st === "closed") {
        rtcDisconnect();
        rtcScheduleRetry(2000);
      }
    };

    // offer
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);

    await waitIceComplete(pc, 8000);

    const url = "/stream";
    const body = {
      sdp: pc.localDescription.sdp,
      cameras: ["road"],
      bridge_services_in: [],
      bridge_services_out: [],
    };

    const r = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });

    if (!r.ok) {
      const t = await r.text().catch(() => "");
      throw new Error("stream http " + r.status + " " + t);
    }

    const ans = await r.json();
    if (!ans || !ans.sdp) throw new Error("bad answer");

    await pc.setRemoteDescription({ type: ans.type || "answer", sdp: ans.sdp });

    rtcStatusSet("connected (waiting track...)");
    rtcArmTrackTimeout(6000);

  } catch (e) {
    rtcStatusSet("error: " + e.message);
    await rtcDisconnect();        //  ���� �� ������ ����
    rtcScheduleRetry(2000);       //  ���⼭�� ������ ��õ�
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
const btnRtcFs = document.getElementById("btnRtcFs");
const rtcVideoEl = document.getElementById("rtcVideo");
const rtcWrap = document.getElementById("rtcWrap");
const btnRtcDevToggle = document.getElementById("btnRtcDevToggle");

// ���� ����ó������ ȣ��ǵ���: ��ư Ŭ�� / ���� �� �̺�Ʈ������ ����
async function rtcToggleFullscreen() {
  const target = rtcWrap || rtcVideoEl;

  // �̹� Ǯ��ũ���̸� ����
  const fsEl = document.fullscreenElement || document.webkitFullscreenElement;
  if (fsEl) {
    if (document.exitFullscreen) await document.exitFullscreen().catch(()=>{});
    else if (document.webkitExitFullscreen) document.webkitExitFullscreen();
    return;
  }

  // 1) ǥ�� Fullscreen API (��κ��� ũ��/�ȵ�/����ũž)
  if (target.requestFullscreen) {
    await target.requestFullscreen().catch(()=>{});
    return;
  }

  // 2) Safari (�Ϻδ� webkitRequestFullscreen)
  if (target.webkitRequestFullscreen) {
    target.webkitRequestFullscreen();
    return;
  }

  // 3) iOS Safari: video ���� ��üȭ�� (���� �� ����)
  //    (����: iOS�� inline ���/��å ������ �� ����� �� ������)
  if (target.webkitEnterFullscreen) {
    target.webkitEnterFullscreen();
    return;
  }

  showAppToast(UI_STRINGS[LANG].fullscreen_not_supported || "Fullscreen not supported on this browser.", {
    tone: "error",
  });
}

// ��ư
if (btnRtcFs) btnRtcFs.onclick = rtcToggleFullscreen;

// ���� ��(���ϸ�)
if (rtcVideoEl) {
  rtcVideoEl.style.cursor = "pointer";
  rtcVideoEl.addEventListener("click", rtcToggleFullscreen);
}

if (btnRtcDevToggle) {
  btnRtcDevToggle.onclick = () => rtcSetDevPreview(!RTC_DEV_PREVIEW);
}

let CAR_WS = null;
let CAR_WS_RETRY_T = null;

function carWsScheduleReconnect(ms = 1000) {
  if (CAR_WS_RETRY_T) return;
  CAR_WS_RETRY_T = setTimeout(() => {
    CAR_WS_RETRY_T = null;
    carWsConnect();
  }, ms);
}

// ===== Driving HUD docking (card <-> WebRTC overlay) =====
function hudDock(mode /* "card"|"top"|"bl" */) {
  const hudRoot = document.getElementById("hudRoot");
  const card = document.getElementById("driveHudCard");
  const host = document.getElementById("hudOverlayHost");
  if (!hudRoot || !card || !host) return;

  host.classList.remove("dock_top","dock_bl");
  host.style.display = "none";

  if (mode === "top" || mode === "bl") {
    host.classList.add(mode === "bl" ? "dock_bl" : "dock_top");
    host.style.display = "";
    if (hudRoot.parentElement !== host) host.appendChild(hudRoot);
    card.style.display = "none";
  } else {
    if (hudRoot.parentElement !== card) card.appendChild(hudRoot);
    card.style.display = "";
  }
}

function hudAutoDock() {
  const rtcVideo = document.getElementById("rtcVideo");
  const rtcCard = document.getElementById("rtcCard");
  const host = document.getElementById("hudOverlayHost");
  const rtcWrapEl = document.getElementById("rtcWrap");
  if (!rtcVideo || !rtcCard || !host || !rtcWrapEl) return;

  const videoVisible = rtcCard.style.display !== "none" && rtcVideo.style.display !== "none";
  if (!videoVisible) { hudDock("card"); return; }

  const fsEl = document.fullscreenElement || document.webkitFullscreenElement;
  const fs = Boolean(
    fsEl && (
      fsEl === rtcWrapEl ||
      fsEl === rtcVideo ||
      (typeof fsEl.contains === "function" && (fsEl.contains(rtcVideo) || fsEl.contains(host)))
    )
  );
  const landscape = window.innerWidth >= window.innerHeight;

  if (fs && landscape) hudDock("bl");
  else hudDock("top");
}

function drivingHudUpdateFromCarPayload(j) {
  if (!window.DrivingHud) {
    console.log("[HUD] update none");
    return;
  }

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
function carWsConnect() {
  // �̹� ��������� �н�
  if (CAR_WS && (CAR_WS.readyState === WebSocket.OPEN || CAR_WS.readyState === WebSocket.CONNECTING)) return;

  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  CAR_WS = new WebSocket(wsProto + "://" + location.host + "/ws/carstate");

  CAR_WS.onopen = () => {
    console.log("[CAR_WS] open");
  };

  CAR_WS.onmessage = (ev) => {
    try {
      const j = JSON.parse(ev.data);
      // console.log("[CAR_WS] msg keys:", Object.keys(j || {}));
      // console.log("[CAR_WS] vEgo:", j?.vEgo, "type:", typeof j?.vEgo);
      drivingHudUpdateFromCarPayload(j);
      hudAutoDock();
    } catch (e) {
      console.log("[CAR_WS] bad msg", e, ev.data);
    }
  };

  CAR_WS.onerror = (e) => {
    console.log("[CAR_WS] error", e);
  };

  CAR_WS.onclose = () => {
    console.log("[CAR_WS] close -> reconnect");
    CAR_WS = null;
    carWsScheduleReconnect(1000);
  };
}

async function carWsDisconnect() {
  if (CAR_WS_RETRY_T) { clearTimeout(CAR_WS_RETRY_T); CAR_WS_RETRY_T = null; }
  try { if (CAR_WS) CAR_WS.close(); } catch {}
  CAR_WS = null;
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
  try {
    RTC_DEV_PREVIEW = RTC_DEV_PREVIEW_ENABLED && localStorage.getItem(RTC_DEV_PREVIEW_KEY) === "1";
  } catch {
    RTC_DEV_PREVIEW = false;
  }
  rtcSyncDevToggleUi();
  rtcApplyDevPreview();

  renderUIText();
  showPage("home", false);
  console.log("[time_sync] syncing server time on page load");
  syncServerTimeOnConnect().catch(() => {});
  rtcInitAuto();
  updateQuickLink().catch(() => {});

  if (window.DrivingHud) {
    window.DrivingHud.init();
  }

  // start car telemetry WS (10Hz)
  carWsConnect();

  // keep HUD dock state in sync
  window.addEventListener("resize", hudAutoDock);
  document.addEventListener("fullscreenchange", hudAutoDock);
  document.addEventListener("webkitfullscreenchange", hudAutoDock);
  setInterval(hudAutoDock, 800);
}



if (document.readyState === "loading") {
  window.addEventListener("DOMContentLoaded", startAll);
} else {
  startAll();
}
