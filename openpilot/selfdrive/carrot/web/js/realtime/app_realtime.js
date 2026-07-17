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
let CARROT_VISION_TEST_FETCH_T = null;
let CARROT_VISION_TEST_FETCH_IN_FLIGHT = null;
const CARROT_VISION_TEST_STATE = {
  active: false,
  status: "stopped",
  runnerAlive: false,
  children: {},
  vipcStreams: [],
  portOpen: false,
  error: "",
  fetchedAtMs: 0,
};
window.CarrotVisionTestState = CARROT_VISION_TEST_STATE;
const CARROT_DEVICE_RUNTIME_STATE = {
  disableDm: null,
  clusterHud: null,
  isOffroad: null,
  isOnroad: null,
  fetchedAtMs: 0,
};
window.CarrotDeviceRuntimeState = CARROT_DEVICE_RUNTIME_STATE;
const CARROT_VISION_CONTENT_STATE = {
  active: true,
  keepWarm: false,
};
var CARROT_VISION_PHASE = window.CarrotVisionPhase;
var CARROT_VISION_CONTROL = window.CarrotVisionControl;
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

function isCarrotRecordedReplayActive() {
  return Boolean(window.CarrotVisionReplay?.isActive?.());
}

function isCarrotVisionContentActive() {
  return CARROT_VISION_CONTENT_STATE.active;
}

function isCarrotVisionContentRuntimeWanted() {
  return CARROT_VISION_CONTENT_STATE.active || CARROT_VISION_CONTENT_STATE.keepWarm;
}

function shouldRunCarrotHudRealtime() {
  if (isCarrotRecordedReplayActive()) return false;
  return !document.hidden && (
    (isCarrotPageActive() && isCarrotVisionContentRuntimeWanted())
    || Boolean(window.CarrotMiniHudMode?.isActive?.())
  );
}

function isCarrotVisionTestActive() {
  return Boolean(window.CarrotVisionTestState?.active);
}
window.isCarrotVisionTestActive = isCarrotVisionTestActive;

function activateCarrotVisionForRunningTest() {
  if (
    !isCarrotPageActive()
    || !isCarrotVisionTestActive()
    || isCarrotVisionActive()
  ) return false;
  setCarrotVisionActive(true, {
    phase: CARROT_VISION_PHASE.STARTING,
    reason: "camera test active",
    updateRtcStatus: false,
  });
  syncCarrotVisionStartOverlay();
  return true;
}

function normalizeRuntimeBool(value) {
  if (typeof value === "boolean") return value;
  if (typeof value === "number") return value !== 0;
  const text = String(value ?? "").trim().toLowerCase();
  if (["1", "true", "yes", "on"].includes(text)) return true;
  if (["0", "false", "no", "off", ""].includes(text)) return false;
  return null;
}

function updateCarrotDeviceRuntimeState(patch = {}) {
  const previous = JSON.stringify({
    disableDm: CARROT_DEVICE_RUNTIME_STATE.disableDm,
    clusterHud: CARROT_DEVICE_RUNTIME_STATE.clusterHud,
    isOffroad: CARROT_DEVICE_RUNTIME_STATE.isOffroad,
    isOnroad: CARROT_DEVICE_RUNTIME_STATE.isOnroad,
  });
  if (Object.prototype.hasOwnProperty.call(patch, "disableDm")) {
    const value = Number.parseInt(String(patch.disableDm ?? ""), 10);
    CARROT_DEVICE_RUNTIME_STATE.disableDm = Number.isFinite(value) ? value : null;
  }
  if (Object.prototype.hasOwnProperty.call(patch, "clusterHud")) {
    const value = Number.parseInt(String(patch.clusterHud ?? ""), 10);
    CARROT_DEVICE_RUNTIME_STATE.clusterHud = Number.isFinite(value) ? value : null;
  }
  if (Object.prototype.hasOwnProperty.call(patch, "isOffroad")) {
    CARROT_DEVICE_RUNTIME_STATE.isOffroad = normalizeRuntimeBool(patch.isOffroad);
  }
  if (Object.prototype.hasOwnProperty.call(patch, "isOnroad")) {
    CARROT_DEVICE_RUNTIME_STATE.isOnroad = normalizeRuntimeBool(patch.isOnroad);
  }
  CARROT_DEVICE_RUNTIME_STATE.fetchedAtMs = Date.now();
  window.CarrotDeviceRuntimeState = CARROT_DEVICE_RUNTIME_STATE;
  return previous !== JSON.stringify({
    disableDm: CARROT_DEVICE_RUNTIME_STATE.disableDm,
    clusterHud: CARROT_DEVICE_RUNTIME_STATE.clusterHud,
    isOffroad: CARROT_DEVICE_RUNTIME_STATE.isOffroad,
    isOnroad: CARROT_DEVICE_RUNTIME_STATE.isOnroad,
  });
}

let _carrotVisionEnvironmentSignature = "";
function syncCarrotVisionEnvironmentState(reason = "environment update") {
  const environment = {
    disableDm: CARROT_DEVICE_RUNTIME_STATE.disableDm,
    clusterHud: CARROT_DEVICE_RUNTIME_STATE.clusterHud,
    isOffroad: CARROT_DEVICE_RUNTIME_STATE.isOffroad,
    isOnroad: CARROT_DEVICE_RUNTIME_STATE.isOnroad,
    testActive: CARROT_VISION_TEST_STATE.active,
    testStatus: CARROT_VISION_TEST_STATE.status,
    testError: CARROT_VISION_TEST_STATE.error,
    testChildren: CARROT_VISION_TEST_STATE.children,
    testPortOpen: CARROT_VISION_TEST_STATE.portOpen,
  };
  const signature = JSON.stringify(environment);
  if (_carrotVisionEnvironmentSignature === signature) return false;
  _carrotVisionEnvironmentSignature = signature;
  setCarrotVisionState({ environment }, { reason, silent: true });
  if (!isCarrotRecordedReplayActive()) {
    setCarrotVisionPhase(CARROT_VISION_STATE.phase, {
      reason,
      updateRtcStatus: false,
    });
  }
  return true;
}

function isCarrotVisionParked() {
  return CARROT_DEVICE_RUNTIME_STATE.isOffroad === true && CARROT_DEVICE_RUNTIME_STATE.isOnroad !== true;
}

function getCarrotVisionRealtimeBlockReason() {
  if (!isCarrotPageActive() || !isCarrotVisionActive()) return "";
  if (isCarrotRecordedReplayActive()) return "recorded-replay";
  if (window.CarrotMiniHudMode?.isActive?.()) return "mini-hud";
  if (isCarrotVisionParked() && !isCarrotVisionTestActive()) {
    if (CARROT_VISION_TEST_STATE.status === "starting") return "camera-test-starting";
    return "vehicle-parked";
  }
  return "";
}

function isCarrotVisionRealtimeSourceReady() {
  return !getCarrotVisionRealtimeBlockReason();
}

function getCarrotVisionRuntimeWaitDetail(reason = getCarrotVisionRealtimeBlockReason()) {
  if (reason === "recorded-replay") {
    return getUIText("replay_loading_detail", "Matching the saved road video with driving information.");
  }
  if (reason === "camera-test-starting") {
    return getUIText("vision_step_starting", "Preparing the device camera.");
  }
  if (reason === "vehicle-parked") {
    return getUIText("vision_parked_detail", "The camera will connect automatically when driving starts.");
  }
  if (reason === "services-missing") {
    return getUIText("vision_step_waiting_car", "Waiting for car camera services.");
  }
  return getUIText("vision_step_waiting_runtime", "Waiting for car runtime connection.");
}

function getCarrotVisionRuntimeWaitStatus(reason = getCarrotVisionRealtimeBlockReason()) {
  if (reason === "recorded-replay") {
    return getUIText("replay_loading", "Preparing drive replay...");
  }
  if (reason === "camera-test-starting") {
    return getUIText("vision_test_starting_title", "Preparing camera check");
  }
  if (reason === "vehicle-parked") {
    return getUIText("vision_parked_title", "Vehicle is parked");
  }
  return getUIText("connecting", "Connecting...");
}

function shouldRunCarrotVisionRealtime() {
  return isCarrotPageVisible()
    && isCarrotVisionContentRuntimeWanted()
    && isCarrotVisionActive()
    && isCarrotVisionRealtimeSourceReady();
}

function getCarrotVisionContentRuntimeStatus() {
  return {
    active: CARROT_VISION_CONTENT_STATE.active,
    keepWarm: CARROT_VISION_CONTENT_STATE.keepWarm,
    userActive: Boolean(isCarrotVisionActive()),
    pageActive: isCarrotPageActive(),
  };
}

function setCarrotVisionContentActive(value, options = {}) {
  const nextActive = Boolean(value);
  const nextKeepWarm = !nextActive && Boolean(options.keepWarm) && isCarrotVisionActive();
  const changed = CARROT_VISION_CONTENT_STATE.active !== nextActive
    || CARROT_VISION_CONTENT_STATE.keepWarm !== nextKeepWarm;
  if (!changed) return false;

  CARROT_VISION_CONTENT_STATE.active = nextActive;
  CARROT_VISION_CONTENT_STATE.keepWarm = nextKeepWarm;
  syncCarrotRealtimeLifecycle(true);
  if (nextActive) requestCarrotVisionRender({ reason: options.reason || "content active" });
  window.dispatchEvent(new CustomEvent("carrot:visioncontentchange", {
    detail: {
      ...getCarrotVisionContentRuntimeStatus(),
      reason: String(options.reason || "content state change"),
    },
  }));
  return true;
}

window.CarrotVisionContentRuntime = Object.freeze({
  isActive: isCarrotVisionContentActive,
  isRuntimeWanted: isCarrotVisionContentRuntimeWanted,
  setActive: setCarrotVisionContentActive,
  status: getCarrotVisionContentRuntimeStatus,
});

function isLandscapeOrientation() {
  if (window.CarrotLayout?.isWide) return window.CarrotLayout.isWide();
  if (typeof window.matchMedia === "function") {
    try {
      return window.matchMedia("(min-aspect-ratio: 13/10), (horizontal-viewport-segments: 2), (vertical-viewport-segments: 2), (min-width: 640px) and (min-height: 650px)").matches;
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

function isCarrotVisionDefaultFullscreenEnabled() {
  const settings = window.CarrotWebSettingsState || {};
  const fallback = true;
  const value = Object.prototype.hasOwnProperty.call(settings, "vision_fullscreen_default")
    ? settings.vision_fullscreen_default
    : fallback;
  if (typeof value === "string") return ["1", "true", "yes", "on"].includes(value.trim().toLowerCase());
  return Boolean(value);
}

function requestCarrotVisionDefaultFullscreen(options = {}) {
  if (!isCarrotVisionDefaultFullscreenEnabled()) return Promise.resolve(false);
  return requestCarrotFullscreen(options);
}

function shouldKeepCarrotFullscreen() {
  return document.body?.dataset?.page === "carrot" && isCarrotVisionActive() && !window.CarrotMiniHudMode?.isActive?.();
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
window.addEventListener("carrot:minihudchange", () => {
  void syncCarrotFullscreenLifecycle();
  syncCarrotRealtimeLifecycle(true);
});
window.addEventListener("carrot:websettingschange", (event) => {
  if (event?.detail?.key !== "vision_fullscreen_default") return;
  if (isCarrotVisionDefaultFullscreenEnabled()) return;
  void exitCarrotFullscreen({ quiet: true }).catch(() => {});
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
  requestCarrotVisionDefaultFullscreen({ quiet: true }).catch(() => {});
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
  if (!isCarrotPageVisible()) return 15000;
  return 5000;
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

async function fetchCarrotVisionTestState() {
  if (CARROT_VISION_TEST_FETCH_IN_FLIGHT) return CARROT_VISION_TEST_FETCH_IN_FLIGHT;

  CARROT_VISION_TEST_FETCH_IN_FLIGHT = (async () => {
    const wasActive = CARROT_VISION_TEST_STATE.active;
    let runtimeChanged = false;
    try {
      const response = await fetch("/api/vision_test/status", { cache: "no-store" });
      const payload = await response.json();
      if (!payload?.ok) throw new Error(payload?.error || "vision_test status failed");
      CARROT_VISION_TEST_STATE.active = payload.status === "running" && payload.runner_alive === true;
      CARROT_VISION_TEST_STATE.status = String(payload.status || "stopped");
      CARROT_VISION_TEST_STATE.runnerAlive = payload.runner_alive === true;
      CARROT_VISION_TEST_STATE.children = payload.children && typeof payload.children === "object" ? payload.children : {};
      CARROT_VISION_TEST_STATE.vipcStreams = Array.isArray(payload.vipc_streams) ? payload.vipc_streams : [];
      CARROT_VISION_TEST_STATE.portOpen = payload.webrtcd_port_open === true;
      CARROT_VISION_TEST_STATE.error = String(payload.error || "");
      CARROT_VISION_TEST_STATE.fetchedAtMs = Date.now();
      const device = payload.device && typeof payload.device === "object" ? payload.device : {};
      runtimeChanged = updateCarrotDeviceRuntimeState({
        disableDm: device.disable_dm,
        isOffroad: device.is_offroad,
        isOnroad: device.is_onroad,
      });
      window.CarrotVisionTestState = CARROT_VISION_TEST_STATE;
    } catch {
      CARROT_VISION_TEST_STATE.active = false;
      CARROT_VISION_TEST_STATE.status = "unavailable";
      CARROT_VISION_TEST_STATE.runnerAlive = false;
      CARROT_VISION_TEST_STATE.children = {};
      CARROT_VISION_TEST_STATE.vipcStreams = [];
      CARROT_VISION_TEST_STATE.portOpen = false;
      CARROT_VISION_TEST_STATE.fetchedAtMs = Date.now();
    } finally {
      CARROT_VISION_TEST_FETCH_IN_FLIGHT = null;
    }

    const environmentChanged = syncCarrotVisionEnvironmentState("vision test status");
    const visionActivated = activateCarrotVisionForRunningTest();
    if (environmentChanged) {
      requestCarrotVisionRender({ reason: "vision test status" });
    }
    if (wasActive !== CARROT_VISION_TEST_STATE.active || runtimeChanged || visionActivated) {
      window.dispatchEvent(new CustomEvent("carrot:visiontestchange", {
        detail: { ...CARROT_VISION_TEST_STATE },
      }));
      syncCarrotVisionAvailability().catch(() => {});
      syncCarrotRealtimeLifecycle(true);
    }
    return CARROT_VISION_TEST_STATE;
  })();

  return CARROT_VISION_TEST_FETCH_IN_FLIGHT;
}

function scheduleCarrotVisionTestStateFetch(ms = 1500) {
  if (CARROT_VISION_TEST_FETCH_T) clearTimeout(CARROT_VISION_TEST_FETCH_T);
  CARROT_VISION_TEST_FETCH_T = setTimeout(async () => {
    CARROT_VISION_TEST_FETCH_T = null;
    await fetchCarrotVisionTestState().catch(() => {});
    scheduleCarrotVisionTestStateFetch(document.hidden ? 4000 : 1500);
  }, ms);
}

function startCarrotVisionTestStateFetch() {
  fetchCarrotVisionTestState().catch(() => {});
  scheduleCarrotVisionTestStateFetch(1500);
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

async function fetchCarrotDeviceRuntimeState() {
  const r = await fetch("/api/params_bulk?names=DisableDM,ClusterHud,IsOffroad,IsOnroad", { cache: "no-store" });
  const j = await r.json().catch(() => ({}));
  if (!r.ok || !j.ok) throw new Error(j.error || `HTTP ${r.status}`);
  const changed = updateCarrotDeviceRuntimeState({
    disableDm: j.values?.DisableDM,
    clusterHud: j.values?.ClusterHud,
    isOffroad: j.values?.IsOffroad,
    isOnroad: j.values?.IsOnroad,
  });
  syncCarrotVisionEnvironmentState("device runtime");
  return { ...CARROT_DEVICE_RUNTIME_STATE, changed };
}

function updateCarrotVisionAvailabilityUi(available, message = window.CARROT_VISION_DISABLED_MESSAGE) {
  const nextAvailable = Boolean(available);
  const wasActive = isCarrotVisionActive();
  const button = document.getElementById("btnStartVision");
  const messageEl = document.getElementById("visionDisabledMessage");
  const defaultUnavailableHint = getUIText("vision_unavailable_hint", "Available when DisableDM is 2.");
  const disabledMessage = nextAvailable ? "" : (message || defaultUnavailableHint);
  const unavailableHint = disabledMessage || defaultUnavailableHint;
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
    const runtime = await fetchCarrotDeviceRuntimeState();
    if (isCarrotRecordedReplayActive()) return true;
    const clusterHudActive = Number(runtime.clusterHud || 0) > 0;
    const available = !clusterHudActive && (isCarrotVisionTestActive() || runtime.disableDm === 2);
    const unavailableMessage = clusterHudActive
      ? getUIText("vision_unavailable_cluster_hud", "Carrot Vision is unavailable while Cluster HUD is enabled.")
      : undefined;
    updateCarrotVisionAvailabilityUi(available, unavailableMessage);
    if (runtime.changed) syncCarrotRealtimeLifecycle(true);
    return available;
  } catch (e) {
    if (isCarrotVisionTestActive()) {
      updateCarrotVisionAvailabilityUi(true);
      return true;
    }
    updateCarrotVisionAvailabilityUi(false, getUIText("disable_dm_check_failed", "Could not check DisableDM status."));
    return false;
  }
}
window.CarrotVisionSyncAvailability = syncCarrotVisionAvailability;

function syncCarrotVisionStartOverlay() {
  const overlay = document.getElementById("visionStartOverlay");
  if (!overlay) return;
  overlay.hidden = isCarrotVisionActive();
  overlay.style.removeProperty("display");
}

async function stopCarrotVisionRealtime(reason = "user stop") {
  if (isCarrotRecordedReplayActive()) {
    await window.CarrotVisionReplay?.stop?.({ returnToLogs: true, reason });
    return;
  }
  if (!isCarrotVisionActive()) return;
  console.warn("[vision] stop requested", reason);
  setCarrotVisionActive(false, {
    phase: CARROT_VISION_PHASE.INACTIVE,
    reason,
    statusText: getUIText("start_vision_hint", "Tap the start button to enable drive vision."),
    updateRtcStatus: false,
  });
  syncCarrotRealtimeLifecycle(true);
  await exitCarrotFullscreen({ quiet: true }).catch(() => {});
  syncCarrotVisionStartOverlay();
  rtcStatusSet(getUIText("start_vision_hint", "Tap the start button to enable drive vision."));
  requestCarrotVisionRender({ reason });
}

window.CarrotVisionStop = stopCarrotVisionRealtime;

window.CarrotVisionStart = async function() {
  if (isCarrotVisionActive()) {
    syncCarrotVisionStartOverlay();
    return;
  }
  if (!CARROT_VISION_STATE.available && !(await syncCarrotVisionAvailability())) {
    if (typeof showAppToast === "function") showAppToast(window.CARROT_VISION_DISABLED_MESSAGE, { tone: "error" });
    return;
  }
  requestCarrotVisionDefaultFullscreen({ quiet: false }).catch(() => {});
  setCarrotVisionActive(true, {
    phase: CARROT_VISION_PHASE.STARTING,
    reason: "user start",
  });
  
  syncCarrotVisionStartOverlay();

  rtcStatusSet(getUIText("connecting", "Connecting..."));
  syncCarrotRealtimeLifecycle(true);
};

function rtcInitAuto() {
  const btn = document.getElementById("btnStartVision");
  if (btn) btn.onclick = window.CarrotVisionStart;
  const stopButton = document.getElementById("btnVisionStop");
  let actionBusy = false;
  const runLoadingAction = async (action) => {
    if (actionBusy) return;
    actionBusy = true;
    if (stopButton) stopButton.disabled = true;
    try {
      await action();
    } finally {
      actionBusy = false;
      if (stopButton) stopButton.disabled = false;
    }
  };
  if (stopButton) {
    stopButton.onclick = () => runLoadingAction(
      () => stopCarrotVisionRealtime("loading panel stop"),
    );
  }
  syncCarrotVisionStartOverlay();
  syncCarrotVisionAvailability().catch(() => {});
  rtcBindVideoEvents();
}

window.addEventListener("carrot:paramchange", (ev) => {
  if (!["DisableDM", "ClusterHud", "IsOffroad", "IsOnroad"].includes(ev?.detail?.name)) return;
  syncCarrotVisionAvailability().catch(() => {});
});

window.addEventListener("carrot:visionchange", syncCarrotVisionStartOverlay);
window.addEventListener("carrot:visionstatechange", syncCarrotVisionStartOverlay);

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
  startCarrotVisionTestStateFetch();

  _hudMarkDirty();
  syncCarrotRealtimeLifecycle(false);
}

let _carrotHudRealtimeActive = false;
let _carrotVisionRealtimeActive = false;
let _carrotVisionRealtimeBlockReason = "";
let _carrotVisionPageReturnConnectT = null;

function cancelCarrotVisionPageReturnConnect() {
  if (_carrotVisionPageReturnConnectT != null) {
    window.clearTimeout(_carrotVisionPageReturnConnectT);
    _carrotVisionPageReturnConnectT = null;
  }
}

function recordCarrotVisionLifecycleEvent(event, detail = {}) {
  try {
    window.dispatchEvent(new CustomEvent("carrot:visionlifecycle", { detail: { event, ...detail, ts: Date.now() } }));
  } catch (_) {}
}

function canReuseCarrotVisionConnection() {
  try {
    return Boolean(window.CarrotVisionRtc?.canResumeWithoutReconnect?.());
  } catch {
    return false;
  }
}

function scheduleCarrotVisionPageReturnConnect(reason = "page return") {
  cancelCarrotVisionPageReturnConnect();
  if (!shouldRunCarrotVisionRealtime()) return;

  if (canReuseCarrotVisionConnection()) {
    recordCarrotVisionLifecycleEvent("page_return_reconnect_skipped", { reason, reused: true });
    rtcScheduleResumeIfConnected(reason);
    startCarrotVisionHealthWatch();
    startRtcPerfPolling(true);
    requestCarrotVisionRender();
    return;
  }

  const shouldConnect = typeof window.CarrotVisionRtc?.shouldConnect === "function"
    ? window.CarrotVisionRtc.shouldConnect()
    : rtcShouldConnect();
  if (!shouldConnect) {
    recordCarrotVisionLifecycleEvent("page_return_reconnect_skipped", { reason, reused: false, busy: true });
    rtcScheduleResumeIfConnected(reason);
    startCarrotVisionHealthWatch();
    startRtcPerfPolling(true);
    requestCarrotVisionRender();
    return;
  }

  recordCarrotVisionLifecycleEvent("page_return_reconnect_scheduled", { reason });
  _carrotVisionPageReturnConnectT = window.setTimeout(async () => {
    _carrotVisionPageReturnConnectT = null;
    if (!shouldRunCarrotVisionRealtime()) return;
    recordCarrotVisionLifecycleEvent("page_return_reconnect_start", { reason });
    rtcCancelRetry();
    rtcCancelRecovery();
    rtcDisarmTrackTimeout();
    rtcDisarmFirstFrameTimeout();
    stopCarrotVisionHealthWatch();
    await rawOverlayDisconnectAll().catch(() => {});
    await rtcDisconnect({ keepVideo: true }).catch(() => {});
    startCarrotVisionHealthWatch();
    await rtcConnectOnce({ force: true }).catch(() => {});
  }, 200);
}

function syncCarrotRealtimeLifecycle(forceFetch = false) {
  const nextHudActive = shouldRunCarrotHudRealtime();
  const nextVisionWanted = isCarrotPageVisible()
    && isCarrotVisionContentRuntimeWanted()
    && isCarrotVisionActive();
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
    recordCarrotVisionLifecycleEvent("vision_realtime_active", {
      forceFetch: Boolean(forceFetch),
      page: document.body?.dataset?.page || "",
    });
    startCarrotVisionHealthWatch();
    setCarrotVisionPhase(CARROT_VISION_PHASE.STARTING, {
      reason: "vision lifecycle active",
      updateRtcStatus: false,
    });
    requestCarrotVisionDefaultFullscreen({ quiet: true }).catch(() => {});
    // Staged startup: keep the shared compact WS on HUD fields here.
    // Expanding it to the overlay services (including modelV2) competes with the
    // WebRTC first-frame/keyframe for the same link and viewer CPU. Overlay
    // data does not affect reaching "ready" (that comes from the camera video
    // frame), so we defer it until the first frame renders — see the
    // carrot:visionstatechange listener below. This gives the keyframe a clean
    // window so first-frame-waiting resolves fast.
    startRtcPerfPolling(true);
    if (rtcShouldConnect()) {
      rtcCancelRetry();
      rtcResetFailCount();
      if (forceFetch) scheduleCarrotVisionPageReturnConnect("vision lifecycle active");
      else rtcConnectOnce().catch(() => {});
    }
  } else {
    cancelCarrotVisionPageReturnConnect();
    if (nextVisionWanted && nextVisionBlockReason) {
      console.log("[perf] carrot vision realtime -> waiting", nextVisionBlockReason);
      if (nextVisionBlockReason !== "recorded-replay") {
        setCarrotVisionPhase(CARROT_VISION_PHASE.STARTING, {
          reason: nextVisionBlockReason,
          statusText: getCarrotVisionRuntimeWaitStatus(nextVisionBlockReason),
          detailText: getCarrotVisionRuntimeWaitDetail(nextVisionBlockReason),
          updateRtcStatus: false,
        });
      }
      rtcCancelRetry();
      rtcCancelRecovery();
      rtcDisarmTrackTimeout();
      rtcCancelResumeCheck();
    } else {
      console.log("[perf] carrot vision realtime -> idle");
      recordCarrotVisionLifecycleEvent("vision_realtime_idle", {
        wanted: Boolean(nextVisionWanted),
        blockReason: nextVisionBlockReason || "",
        page: document.body?.dataset?.page || "",
      });
      setCarrotVisionPhase(CARROT_VISION_STATE.available ? CARROT_VISION_PHASE.INACTIVE : CARROT_VISION_PHASE.UNAVAILABLE, {
        reason: "vision lifecycle idle",
        updateRtcStatus: false,
        render: false,
      });
    }
    stopCarrotVisionHealthWatch();
    stopRtcPerfPolling();
    rawOverlayDisconnectAll();
    rtcDisconnect({ keepVideo: true }).catch(() => {});
  }

  emitCarrotRenderRequest({ force: true, overlayDirty: true, hudDirty: true });
}

document.addEventListener("visibilitychange", () => {
  recordCarrotVisionLifecycleEvent("visibility_change", {
    state: document.visibilityState,
    page: document.body?.dataset?.page || "",
    visionActive: Boolean(isCarrotVisionActive()),
  });
  rtcHandleVisibilityChange();
  syncCarrotRealtimeLifecycle(false);
});

window.addEventListener("offline", () => {
  recordCarrotVisionLifecycleEvent("network_offline", {
    page: document.body?.dataset?.page || "",
    visionActive: Boolean(isCarrotVisionActive()),
  });
  rtcStatusSet("offline");
});

window.addEventListener("online", () => {
  recordCarrotVisionLifecycleEvent("network_online", {
    page: document.body?.dataset?.page || "",
    visionActive: Boolean(isCarrotVisionActive()),
  });
  syncCarrotRealtimeLifecycle(false);
  rtcScheduleResumeIfConnected("network resumed");
});

function handleCarrotVisionPageSuspend(eventName, detail = {}) {
  recordCarrotVisionLifecycleEvent(eventName, {
    page: document.body?.dataset?.page || "",
    visibility: document.visibilityState,
    visionActive: Boolean(isCarrotVisionActive()),
    ...detail,
  });
  cancelCarrotVisionPageReturnConnect();
  rtcExitPictureInPicture();
  if (!isCarrotVisionActive()) return;
  rtcCaptureVideoHoldFrame();
  stopCarrotVisionHealthWatch();
  rawOverlayDisconnectAll().catch(() => {});
  if (isCarrotPageActive()) {
    rtcDisconnect({ keepVideo: true }).catch(() => {});
  }
}

function handleCarrotVisionPageResume(eventName, detail = {}) {
  recordCarrotVisionLifecycleEvent(eventName, {
    page: document.body?.dataset?.page || "",
    visibility: document.visibilityState,
    visionActive: Boolean(isCarrotVisionActive()),
    ...detail,
  });
  if (!isCarrotPageActive() || !isCarrotVisionActive()) return;
  syncCarrotVisionAvailability().catch(() => {});
  fetchLiveRuntimeState(true).catch(() => {});
  syncCarrotRealtimeLifecycle(true);
  scheduleCarrotVisionPageReturnConnect(eventName);
}

window.addEventListener("pagehide", (event) => {
  handleCarrotVisionPageSuspend("pagehide", { persisted: Boolean(event?.persisted) });
});
window.addEventListener("pageshow", (event) => {
  handleCarrotVisionPageResume("pageshow", { persisted: Boolean(event?.persisted) });
});
window.addEventListener("focus", () => {
  handleCarrotVisionPageResume("window_focus");
});
window.addEventListener("blur", () => {
  recordCarrotVisionLifecycleEvent("window_blur", {
    page: document.body?.dataset?.page || "",
    visionActive: Boolean(isCarrotVisionActive()),
  });
});
document.addEventListener("freeze", () => {
  handleCarrotVisionPageSuspend("page_freeze", { persisted: true });
});
document.addEventListener("resume", () => {
  handleCarrotVisionPageResume("page_resume", { persisted: true });
});

window.addEventListener("carrot:pagechange", (event) => {
  maybeRequestCarrotFullscreenOnPageChange(event?.detail || {});
  const page = event?.detail?.page || "";
  recordCarrotVisionLifecycleEvent("page_change", {
    page,
    visionActive: Boolean(isCarrotVisionActive()),
  });
  if (page === "carrot" && isCarrotVisionActive()) {
    scheduleCarrotVisionPageReturnConnect("page changed to carrot");
  } else {
    cancelCarrotVisionPageReturnConnect();
  }
  syncCarrotRealtimeLifecycle(true);
});

// Staged overlay gate, driven by vision phase:
//   - Expand the shared compact WS to overlay fields only once the camera video
//     has produced its first renderable frame
//     (phase === "ready"). Until then the WebRTC first-frame/keyframe owns the
//     link + viewer CPU, so first-frame-waiting resolves fast.
//   - When phase leaves "ready" for an active (re)connection state, contract the
//     compact WS to HUD fields so the reconnect's keyframe gets a clean window. With
//     the tolerant freeze watchdog, transient stalls hold at phase "ready"
//     (no churn here); only genuine reconnects leave "ready", which is exactly
//     when we want the link freed.
// The _overlayStaged flag is REQUIRED: connectOverlay()/disconnectOverlay()
// themselves publish a (non-silent) vision state change, so acting on every
// event without an edge guard would re-enter this listener infinitely.
let _overlayStaged = false;
window.addEventListener("carrot:visionstatechange", (event) => {
  const state = event?.detail?.state || window.CarrotVisionState;
  if (!state || !state.active) {
    _overlayStaged = false;
    return;
  }
  const isReady = state.controlState === CARROT_VISION_CONTROL.LIVE
    && !isCarrotVisionTestActive()
    && !isCarrotRecordedReplayActive();
  if (isReady && !_overlayStaged) {
    _overlayStaged = true;
    window.CarrotVisionRaw?.connectOverlay?.();
  } else if (!isReady && _overlayStaged) {
    _overlayStaged = false;
    window.CarrotVisionRaw?.disconnectOverlay?.();
  }
});

window.addEventListener("carrot:visiontestchange", () => {
  if (!isCarrotVisionTestActive()) return;
  _overlayStaged = false;
  window.CarrotVisionRaw?.disconnectOverlay?.();
});



if (document.readyState === "loading") {
  window.addEventListener("DOMContentLoaded", startAll);
} else {
  startAll();
}
