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
const CARROT_VISION_REQUIRED_LIVE_SERVICES = Object.freeze(["roadCameraState", "modelV2"]);
var CARROT_VISION_PHASE = window.CarrotVisionPhase;
var CARROT_VISION_STATE = window.CarrotVisionState;
var isCarrotVisionActive = window.isCarrotVisionActive;
var setCarrotVisionPhase = window.CarrotVisionSetPhase;
var setCarrotVisionState = window.CarrotVisionSetState;
var setCarrotVisionAvailable = window.CarrotVisionSetAvailable;
var setCarrotVisionActive = window.CarrotVisionSetActive;

function isCarrotPageActive() {
  return document.body?.dataset?.page === "carrot";
}

function isCarrotPageVisible() {
  return isCarrotPageActive() && !document.hidden;
}

function shouldRunCarrotHudRealtime() {
  return isCarrotPageActive();
}

function getCarrotVisionRealtimeBlockReason() {
  if (!isCarrotPageActive() || !isCarrotVisionActive()) return "";

  const runtimeState = window.CarrotLiveRuntimeState || CARROT_LIVE_RUNTIME_STATE;
  if (!runtimeState?.ok) return "runtime-unavailable";

  const runtime = runtimeState.runtime && typeof runtimeState.runtime === "object" ? runtimeState.runtime : {};
  const serviceAlive = runtime.serviceAlive && typeof runtime.serviceAlive === "object" ? runtime.serviceAlive : null;
  if (!serviceAlive) return "runtime-unavailable";

  const missing = CARROT_VISION_REQUIRED_LIVE_SERVICES.filter((service) => serviceAlive[service] !== true);
  return missing.length ? "services-missing" : "";
}

function isCarrotVisionRealtimeSourceReady() {
  return !getCarrotVisionRealtimeBlockReason();
}

function getCarrotVisionRuntimeWaitDetail(reason = getCarrotVisionRealtimeBlockReason()) {
  if (reason === "services-missing") {
    return getUIText("vision_step_waiting_car", "Waiting for car camera services.");
  }
  return getUIText("vision_step_waiting_runtime", "Waiting for car runtime connection.");
}

function shouldRunCarrotVisionRealtime() {
  return isCarrotPageActive() && isCarrotVisionActive() && isCarrotVisionRealtimeSourceReady();
}

function isLandscapeOrientation() {
  if (typeof window.matchMedia === "function") {
    try {
      return window.matchMedia("(orientation: landscape)").matches;
    } catch {}
  }
  return Number(window.innerWidth || 0) > Number(window.innerHeight || 0);
}

function isFullscreenActive() {
  return Boolean(
    document.fullscreenElement ||
    document.webkitFullscreenElement ||
    document.msFullscreenElement
  );
}

async function requestCarrotFullscreen(options = {}) {
  const quiet = Boolean(options.quiet);
  if (!isLandscapeOrientation()) return false;
  if (isFullscreenActive()) return true;

  const root = document.documentElement || document.body;
  if (!root) return false;

  const request =
    root.requestFullscreen ||
    root.webkitRequestFullscreen ||
    root.msRequestFullscreen;
  if (typeof request !== "function") {
    if (!quiet && typeof showAppToast === "function") {
      showAppToast(getUIText("fullscreen_not_supported", "Fullscreen not supported on this browser."), { tone: "error" });
    }
    return false;
  }

  try {
    const result = request.call(root);
    if (result && typeof result.then === "function") {
      await result;
    }
    return true;
  } catch (error) {
    console.log("[fullscreen] request failed", error);
    return false;
  }
}

async function exitCarrotFullscreen(options = {}) {
  const quiet = Boolean(options.quiet);
  if (!isFullscreenActive()) return true;

  const exit =
    document.exitFullscreen ||
    document.webkitExitFullscreen ||
    document.msExitFullscreen;
  if (typeof exit !== "function") {
    if (!quiet && typeof showAppToast === "function") {
      showAppToast(getUIText("fullscreen_not_supported", "Fullscreen not supported on this browser."), { tone: "error" });
    }
    return false;
  }

  try {
    const result = exit.call(document);
    if (result && typeof result.then === "function") {
      await result;
    }
    return true;
  } catch (error) {
    console.log("[fullscreen] exit failed", error);
    return false;
  }
}

async function toggleCarrotFullscreen(options = {}) {
  if (isFullscreenActive()) {
    return exitCarrotFullscreen(options);
  }
  return requestCarrotFullscreen(options);
}

function shouldKeepCarrotFullscreen() {
  return document.body?.dataset?.page === "carrot" && isCarrotVisionActive();
}

async function syncCarrotFullscreenLifecycle() {
  if (shouldKeepCarrotFullscreen()) return;
  await exitCarrotFullscreen({ quiet: true }).catch(() => {});
}

window.RequestCarrotFullscreen = requestCarrotFullscreen;
window.ExitCarrotFullscreen = exitCarrotFullscreen;
window.ToggleCarrotFullscreen = toggleCarrotFullscreen;

window.addEventListener("carrot:pagechange", () => {
  void syncCarrotFullscreenLifecycle();
});
window.addEventListener("carrot:visionchange", () => {
  void syncCarrotFullscreenLifecycle();
});

function emitCarrotRenderRequest(detail = {}) {
  window.dispatchEvent(new CustomEvent("carrot:render-request", { detail }));
}

function requestCarrotVisionRender(detail = {}) {
  emitCarrotRenderRequest({ force: true, overlayDirty: true, hudDirty: true, ...detail });
}
window.requestCarrotVisionRender = requestCarrotVisionRender;

function emitCarrotVisionChange(active) {
  window.dispatchEvent(new CustomEvent("carrot:visionchange", { detail: { active: Boolean(active) } }));
}
window.emitCarrotVisionChange = emitCarrotVisionChange;

function maybeRequestCarrotFullscreenOnPageChange(detail = {}) {
  if (String(detail?.page || "") !== "carrot") return;
  if (!isCarrotVisionActive()) return;
  requestCarrotFullscreen({ quiet: true }).catch(() => {});
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
        _hudMarkDirty();
        emitCarrotRenderRequest({ force: false, overlayDirty: true, hudDirty: true });
      }
      if (typeof syncCarrotRealtimeLifecycle === "function") {
        syncCarrotRealtimeLifecycle(false);
      }
	      LIVE_RUNTIME_FETCH_IN_FLIGHT = null;
	    }
  })();

  return LIVE_RUNTIME_FETCH_IN_FLIGHT;
}

function getLiveRuntimePollMs() {
  return isCarrotPageVisible() ? 1000 : 3000;
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

setCarrotVisionAvailable(false, {
  disabledMessage: getUIText("vision_unavailable_hint", "Available when DisableDM is 2."),
  reason: "init",
  updateRtcStatus: false,
  render: false,
});
let carrotVisionBadgeHintTimer = null;

function showCarrotVisionBadgeHint(el) {
  if (!el) return;
  el.classList.add("is-tooltip-visible");
  if (carrotVisionBadgeHintTimer) clearTimeout(carrotVisionBadgeHintTimer);
  carrotVisionBadgeHintTimer = window.setTimeout(() => {
    el.classList.remove("is-tooltip-visible");
    if (document.activeElement === el) el.blur();
    carrotVisionBadgeHintTimer = null;
  }, 1400);
}

async function fetchDisableDmValue() {
  const r = await fetch("/api/params_bulk?names=DisableDM", { cache: "no-store" });
  const j = await r.json().catch(() => ({}));
  if (!r.ok || !j.ok) throw new Error(j.error || `HTTP ${r.status}`);
  const raw = j.values?.DisableDM;
  const value = Number.parseInt(String(raw ?? "0"), 10);
  return Number.isFinite(value) ? value : 0;
}

function updateCarrotVisionAvailabilityUi(available, message = window.CARROT_VISION_DISABLED_MESSAGE) {
  const nextAvailable = Boolean(available);
  const wasActive = isCarrotVisionActive();
  const button = document.getElementById("btnStartVision");
  const messageEl = document.getElementById("visionDisabledMessage");
  const unavailableHint = getUIText("vision_unavailable_hint", "Available when DisableDM is 2.");
  const disabledMessage = nextAvailable ? "" : (message || unavailableHint);
  setCarrotVisionAvailable(nextAvailable, {
    disabledMessage,
    reason: "availability ui",
    updateRtcStatus: false,
  });
  if (button) {
    button.disabled = !nextAvailable;
    button.setAttribute("aria-disabled", nextAvailable ? "false" : "true");
    button.title = nextAvailable ? "" : unavailableHint;
  }
  if (messageEl) {
    messageEl.hidden = Boolean(nextAvailable);
    messageEl.replaceChildren();
    if (!nextAvailable) {
      const hint = unavailableHint;
      messageEl.textContent = "!";
      messageEl.dataset.tooltip = hint;
      messageEl.title = hint;
      messageEl.setAttribute("aria-label", hint);
      messageEl.setAttribute("role", "button");
      messageEl.tabIndex = 0;
      messageEl.onclick = () => showCarrotVisionBadgeHint(messageEl);
      messageEl.ontouchstart = () => showCarrotVisionBadgeHint(messageEl);
      messageEl.onkeydown = (event) => {
        if (event.key === "Enter" || event.key === " ") {
          event.preventDefault();
          showCarrotVisionBadgeHint(messageEl);
        }
      };
    } else {
      messageEl.classList.remove("is-tooltip-visible");
      messageEl.removeAttribute("data-tooltip");
      messageEl.removeAttribute("title");
      messageEl.removeAttribute("aria-label");
      messageEl.removeAttribute("role");
      messageEl.removeAttribute("tabindex");
      messageEl.onclick = null;
      messageEl.ontouchstart = null;
      messageEl.onkeydown = null;
      if (carrotVisionBadgeHintTimer) {
        clearTimeout(carrotVisionBadgeHintTimer);
        carrotVisionBadgeHintTimer = null;
      }
    }
  }
  if (nextAvailable) {
    if (!isCarrotVisionActive()) {
      setCarrotVisionPhase(CARROT_VISION_PHASE.INACTIVE, {
        reason: "vision available",
        updateRtcStatus: false,
      });
      rtcStatusSet(getUIText("start_vision_hint", "Tap the start button to enable drive vision."));
    }
  } else {
    if (wasActive) {
      syncCarrotRealtimeLifecycle(true);
    }
    rtcStatusSet(disabledMessage);
  }
}

async function syncCarrotVisionAvailability() {
  try {
    const disableDm = await fetchDisableDmValue();
    const available = disableDm === 2;
    updateCarrotVisionAvailabilityUi(available);
    return available;
  } catch (e) {
    updateCarrotVisionAvailabilityUi(false, getUIText("disable_dm_check_failed", "Could not check DisableDM status."));
    return false;
  }
}

async function reconnectCarrotVisionRealtime(reason = "manual reconnect") {
  if (!isCarrotVisionActive()) return;
  console.warn("[vision] reconnect requested", reason);
  rtcStatusSet(getUIText("reconnecting", "Reconnecting..."));
  setCarrotVisionPhase(CARROT_VISION_PHASE.RECOVERING, {
    reason,
    updateRtcStatus: false,
  });
  rtcCaptureVideoHoldFrame();
  stopCarrotVisionHealthWatch();
  await rawOverlayDisconnectAll().catch(() => {});
  await rtcDisconnect({ keepVideo: true }).catch(() => {});
  _carrotVisionRealtimeActive = false;
  syncCarrotRealtimeLifecycle(true);
  startCarrotVisionHealthWatch();
  requestCarrotVisionRender();
}

window.CarrotVisionReconnect = reconnectCarrotVisionRealtime;

window.CarrotVisionStart = async function() {
  if (isCarrotVisionActive()) {
    await reconnectCarrotVisionRealtime("start button while active");
    return;
  }
  if (!CARROT_VISION_STATE.available && !(await syncCarrotVisionAvailability())) {
    if (typeof showAppToast === "function") showAppToast(window.CARROT_VISION_DISABLED_MESSAGE, { tone: "error" });
    return;
  }
  requestCarrotFullscreen({ quiet: false }).catch(() => {});
  setCarrotVisionActive(true, {
    phase: CARROT_VISION_PHASE.STARTING,
    reason: "user start",
  });
  
  const btn = document.getElementById("visionStartOverlay");
  if (btn) btn.style.display = "none";

  rtcStatusSet(getUIText("connecting", "Connecting..."));
  syncCarrotRealtimeLifecycle(true);
};

function rtcInitAuto() {
  const btn = document.getElementById("btnStartVision");
  if (btn) btn.onclick = window.CarrotVisionStart;
  syncCarrotVisionAvailability().catch(() => {});
  rtcBindVideoEvents();
}

window.addEventListener("carrot:paramchange", (ev) => {
  if (ev?.detail?.name !== "DisableDM") return;
  syncCarrotVisionAvailability().catch(() => {});
});

window.addEventListener("carrot:pagechange", (ev) => {
  if (ev?.detail?.page !== "carrot") return;
  syncCarrotVisionAvailability().catch(() => {});
});

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

async function startAll() {
  if (typeof syncWebLanguageFromDeviceDefault === "function") {
    await syncWebLanguageFromDeviceDefault();
  }
  renderUIText();
  if (typeof window.bootstrapWebStartPage === "function") {
    window.bootstrapWebStartPage("realtime");
  }
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
let _carrotVisionRealtimeBlockReason = "";

function syncCarrotRealtimeLifecycle(forceFetch = false) {
  const nextHudActive = shouldRunCarrotHudRealtime();
  const nextVisionWanted = isCarrotPageActive() && isCarrotVisionActive();
  const nextVisionBlockReason = getCarrotVisionRealtimeBlockReason();
  const nextVisionActive = nextVisionWanted && !nextVisionBlockReason;

  if (
    nextHudActive === _carrotHudRealtimeActive &&
    nextVisionActive === _carrotVisionRealtimeActive &&
    nextVisionBlockReason === _carrotVisionRealtimeBlockReason &&
    !forceFetch
  ) {
    if (nextVisionActive) startCarrotVisionHealthWatch();
    else stopCarrotVisionHealthWatch();
    if (nextVisionActive && rtcShouldConnect()) {
      rtcConnectOnce().catch(() => {});
    }
    if (nextVisionActive) scheduleRtcPerfPolling();
    if (nextHudActive) _hudScheduleRender();
    return;
  }

  _carrotHudRealtimeActive = nextHudActive;
  _carrotVisionRealtimeActive = nextVisionActive;
  _carrotVisionRealtimeBlockReason = nextVisionBlockReason;

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
    startCarrotVisionHealthWatch();
    setCarrotVisionPhase(CARROT_VISION_PHASE.STARTING, {
      reason: "vision lifecycle active",
      updateRtcStatus: false,
    });
    requestCarrotFullscreen({ quiet: true }).catch(() => {});
    ensureRawDecodeWorker();
    rawOverlayConnectAll();
    startRtcPerfPolling(true);
    if (rtcShouldConnect()) {
      rtcCancelRetry();
      rtcResetFailCount();
      rtcConnectOnce().catch(() => {});
    }
  } else {
    if (nextVisionWanted && nextVisionBlockReason) {
      console.log("[perf] carrot vision realtime -> waiting", nextVisionBlockReason);
      setCarrotVisionPhase(CARROT_VISION_PHASE.STARTING, {
        reason: nextVisionBlockReason,
        statusText: getUIText("connecting", "Connecting..."),
        detailText: getCarrotVisionRuntimeWaitDetail(nextVisionBlockReason),
        updateRtcStatus: false,
      });
      rtcCancelRetry();
      rtcCancelRecovery();
      rtcDisarmTrackTimeout();
      rtcCancelResumeCheck();
    } else {
      console.log("[perf] carrot vision realtime -> idle");
      setCarrotVisionPhase(CARROT_VISION_STATE.available ? CARROT_VISION_PHASE.INACTIVE : CARROT_VISION_PHASE.UNAVAILABLE, {
        reason: "vision lifecycle idle",
        updateRtcStatus: false,
        render: false,
      });
    }
    stopCarrotVisionHealthWatch();
    stopRtcPerfPolling();
    rawOverlayDisconnectAll();
    rtcDisconnect().catch(() => {});
  }

  emitCarrotRenderRequest({ force: true, overlayDirty: true, hudDirty: true });
}

document.addEventListener("visibilitychange", () => {
  rtcHandleVisibilityChange();
  syncCarrotRealtimeLifecycle(false);
});

window.addEventListener("offline", () => {
  rtcStatusSet("offline");
});

window.addEventListener("online", () => {
  syncCarrotRealtimeLifecycle(false);
  rtcScheduleResumeIfConnected("network resumed");
});
window.addEventListener("pagehide", rtcExitPictureInPicture);
window.addEventListener("carrot:pagechange", (event) => {
  maybeRequestCarrotFullscreenOnPageChange(event?.detail || {});
  syncCarrotRealtimeLifecycle(false);
});



if (document.readyState === "loading") {
  window.addEventListener("DOMContentLoaded", startAll);
} else {
  startAll();
}
