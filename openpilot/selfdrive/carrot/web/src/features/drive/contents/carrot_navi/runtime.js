import { createOverlay } from "./overlay.js";
import { createOverlayStore } from "./overlay_store.js";
import { createLifecycle } from "./lifecycle.js";
import { createSplit } from "./split.js";
import { createCompositor } from "./compositor.js";
import { createMse, DEFAULT_MIME } from "./mse.js";
import { createTransport } from "./transport.js";
import { createHealth } from "./health.js";

const runtimeSingletons = new WeakMap();
const EMPTY_RUNTIME = Object.freeze({});

export function createCarrotNaviRuntime(options = {}) {
  const target = options.target || globalThis;
  const documentRoot = options.document || target.document;
  const workspace = options.workspace || documentRoot?.getElementById("carrotDriveWorkspace");
  const stage = options.stage || documentRoot?.getElementById("carrotStage");
  const pane = options.root || options.pane || documentRoot?.getElementById("carrotNaviPane");
  const divider = options.divider || documentRoot?.getElementById("carrotNaviDivider");
  const canvas = options.canvas || documentRoot?.getElementById("carrotNaviCanvas");
  const video = options.video || documentRoot?.getElementById("carrotNaviVideo");
  const overlayCanvas = options.overlayCanvas || documentRoot?.getElementById("carrotNaviOverlayCanvas");
  const statusEl = options.statusElement || documentRoot?.getElementById("carrotNaviStatus");
  const layoutSpec = options.layoutSpec || target.CarrotDriveLayoutSpec;
  const workspaceRuntime = options.workspaceRuntime || target.DriveWorkspaceRuntime;
  const workspaceApi = options.workspaceApi || target.DriveWorkspace;
  if (!workspace || !stage || !pane || !divider || !canvas || !video || !overlayCanvas || !layoutSpec) return EMPTY_RUNTIME;

  const overlay = (options.createOverlay || createOverlay)(overlayCanvas);
  const overlayStore = (options.createOverlayStore || createOverlayStore)(overlay);
  const lifecycle = (options.createLifecycle || createLifecycle)({ target });
  const split = (options.createSplit || createSplit)({
    target,
    workspaceRuntime,
    workspace,
    stage,
    pane,
    divider,
    overlay,
  });
  if (!overlay || !overlayStore || !lifecycle || !split) return EMPTY_RUNTIME;

  const MIN_LANDSCAPE_WORKSPACE_WIDTH = 540;
  const MIN_LANDSCAPE_WORKSPACE_HEIGHT = 240;
  const MIN_PORTRAIT_WORKSPACE_WIDTH = 300;
  const MIN_PORTRAIT_WORKSPACE_HEIGHT = 480;
  const MSE_STALL_RECOVERY_MS = 3000;
  const MSE_RECOVERY_COOLDOWN_MS = 5000;
  const CLIENT_DIAGNOSTIC_INTERVAL_MS = 2000;
  const STREAM_GATE_POLL_MS = 1000;
  const PIPELINE_VERSION = 8;

  let environmentActive = false;
  let layoutVisible = false;
  let lastMseRecoveryAt = 0;
  let mediaRecoveryTimer = 0;
  let pendingRecoveryReason = "";
  let mediaPipeline = "";
  let decodeError = "";
  let clientDiagnosticPending = false;
  let streamGateAllowed = false;
  let streamGateKnown = false;
  let streamGatePending = false;
  let streamGateCheckedAt = 0;
  let contentActive = options.contentActive === undefined ? true : Boolean(options.contentActive);
  let contentKeepWarm = false;

  const compositor = (options.createCompositor || createCompositor)({
    target,
    pane,
    canvas,
    video,
    overlayCanvas,
    overlayStore,
    onFrame: handlePresentedFrame,
  });
  const mse = (options.createMse || createMse)(video, {
    target,
    onFrame: () => compositor?.render(),
    onError: handleDecodeError,
  });
  if (!compositor || !mse) return EMPTY_RUNTIME;

  const transport = (options.createTransport || createTransport)({
    target,
    resolveAssetUrl: options.resolveAssetUrl,
    onState: handleNavigationState,
    onStateDisconnected: handleStateDisconnected,
    onWorkerMessage: handleWorkerMessage,
    onError: handleDecodeError,
    onConnectionChange: handleConnectionChange,
    onOwnershipBusy: handleOwnershipBusy,
  });
  if (!transport) return EMPTY_RUNTIME;
  const health = (options.createHealth || createHealth)({
    transport,
    mse,
    compositor,
    now: () => target.performance.now(),
  });
  if (!health) return EMPTY_RUNTIME;
  const introApi = options.introApi || target.DriveContentIntro;
  const intro = introApi?.create?.({
    host: pane,
    className: "carrot-navi-pane__intro",
    feature: {
      id: layoutSpec.CONTENT.NAVIGATION,
      label: () => uiText("web_drive_layout_content_navigation", "Carrot Navi"),
    },
  });
  const introStatus = introApi?.STATUS;
  const stateSurfaceApi = options.stateSurfaceApi || target.CarrotUI?.stateSurface;
  const ownershipSurface = stateSurfaceApi?.create?.({
    host: pane,
    className: "carrot-navi-pane__ownershipNotice",
    featureLabel: () => uiText("web_drive_layout_content_navigation", "Carrot Navi"),
  });
  if (!intro || !introStatus || !ownershipSurface) return EMPTY_RUNTIME;

function decoderMode() {
  return mse.supported() ? "mse" : "";
}

function viewportSize() {
  const viewport = target.visualViewport;
  return {
    width: Math.round(viewport?.width || target.innerWidth || 0),
    height: Math.round(viewport?.height || target.innerHeight || 0),
  };
}

function workspaceOrientation(viewport = viewportSize()) {
  const shared = workspaceRuntime?.viewportOrientation?.();
  if (shared) return shared;
  return viewport.height >= viewport.width
    ? workspaceApi.ORIENTATION.VERTICAL
    : workspaceApi.ORIENTATION.HORIZONTAL;
}

async function refreshStreamGate(force = false) {
  const now = target.performance.now();
  if (streamGatePending || (!force && streamGateKnown && now - streamGateCheckedAt < STREAM_GATE_POLL_MS)) return;
  streamGatePending = true;
  streamGateCheckedAt = now;
  let allowed = false;
  try {
    const response = await target.fetch("/api/carrot_navi/status", { cache: "no-store" });
    if (response.ok) {
      const status = await response.json();
      allowed = status?.streamAllowed === true;
    }
  } catch (_) {
  } finally {
    streamGatePending = false;
    const changed = !streamGateKnown || streamGateAllowed !== allowed;
    streamGateKnown = true;
    streamGateAllowed = allowed;
    if (changed) evaluateEnvironment();
  }
}

function isWorkspaceLayoutVisible() {
  return documentRoot.body.dataset.page === "carrot"
    && !documentRoot.hidden
    && !stage.classList.contains("is-replay");
}

function isEnvironmentEligible() {
  const orientation = workspaceOrientation();
  const workspaceRect = workspace.getBoundingClientRect();
  const workspaceWidth = workspaceRect.width || workspace.clientWidth || 0;
  const workspaceHeight = workspaceRect.height || workspace.clientHeight || 0;
  const sizeEligible = orientation === workspaceApi.ORIENTATION.VERTICAL
    ? workspaceWidth >= MIN_PORTRAIT_WORKSPACE_WIDTH && workspaceHeight >= MIN_PORTRAIT_WORKSPACE_HEIGHT
    : workspaceWidth >= MIN_LANDSCAPE_WORKSPACE_WIDTH && workspaceHeight >= MIN_LANDSCAPE_WORKSPACE_HEIGHT;
  const localEligible = contentActive
    && layoutVisible
    && navigationPaneVisible()
    && sizeEligible
    && typeof target.Worker === "function"
    && Boolean(decoderMode());
  if (localEligible) void refreshStreamGate();
  return localEligible && streamGateKnown && streamGateAllowed;
}

function canRetainRuntime() {
  return (contentActive || contentKeepWarm)
    && streamGateKnown
    && streamGateAllowed
    && typeof target.Worker === "function"
    && Boolean(decoderMode());
}

function uiText(key, fallback) {
  return typeof target.getUIText === "function" ? target.getUIText(key, fallback) : fallback;
}

function navigationPaneVisible(state = split.snapshot()) {
  if (!state.active) return false;
  if (state.mode === layoutSpec.MODE.SPLIT) return true;
  const area1IsNavigation = state.area1Content === layoutSpec.CONTENT.NAVIGATION;
  return state.mode === layoutSpec.MODE.AREA_1 ? area1IsNavigation : !area1IsNavigation;
}

function syncIntro() {
  const splitState = split.snapshot();
  const presentation = compositor.snapshot();
  const transportState = transport.snapshot();
  if (!navigationPaneVisible(splitState) || presentation.hasFrame) {
    intro.hide();
    ownershipSurface.hide();
    return false;
  }

  if (transportState.ownershipBlocked) {
    intro.hide();
    ownershipSurface.show(stateSurfaceApi.STATE.OWNERSHIP, {
      tone: stateSurfaceApi.TONE.NEUTRAL,
      title: uiText("drive_stream_busy", "Active on another device"),
      actions: [{
        id: "takeover",
        label: uiText("drive_stream_use_here", "Use here"),
        tone: stateSurfaceApi.ACTION_KIND.PRIMARY,
        onActivate: () => requestOwnershipTakeover("busy action"),
      }],
    });
    return true;
  }

  ownershipSurface.hide();

  let status;
  if (typeof target.Worker !== "function" || !decoderMode()) {
    status = {
      kind: introStatus.UNSUPPORTED,
      message: uiText("drive_content_status_unsupported", "Not supported"),
      description: uiText("carrot_navi_intro_unsupported_detail", "This browser cannot start the navigation video pipeline."),
    };
  } else if (streamGateKnown && !streamGateAllowed) {
    status = {
      kind: introStatus.UNAVAILABLE,
      message: uiText("drive_content_status_unavailable", "Unavailable"),
      description: uiText("carrot_navi_intro_unavailable_detail", "The stream remains stopped for the current device state."),
    };
  } else if (decodeError) {
    status = {
      kind: introStatus.RECOVERING,
      message: uiText("drive_content_status_recovering", "Recovering"),
      description: uiText("carrot_navi_intro_recovering_detail", "The screen will return automatically after recovery."),
    };
  } else {
    status = {
      kind: introStatus.PREPARING,
      message: uiText("drive_content_status_preparing", "Preparing"),
      description: uiText("carrot_navi_intro_preparing_detail", "The navigation screen will appear here automatically when it is ready."),
    };
  }
  intro.setStatus(status);
  return true;
}

function showStatus(mode, fallbackText) {
  if (!statusEl || !navigationPaneVisible()) return;
  const keys = {
    waiting: ["carrot_navi_waiting_video", "지도 영상 대기 중"],
    disconnected: ["carrot_navi_disconnected", "내비 연결 끊김 · 재연결 중"],
    stalled: ["carrot_navi_map_stalled", "지도 영상 멈춤 · 복구 중"],
  };
  const [key, defaultText] = keys[mode] || ["", fallbackText || "내비 상태 확인 중"];
  statusEl.textContent = fallbackText || (typeof target.getUIText === "function" ? target.getUIText(key, defaultText) : defaultText);
  statusEl.dataset.tone = mode === "stalled" ? "warning" : mode;
  statusEl.hidden = false;
}

function updateRuntimeStatus(now = target.performance.now(), introVisible = false) {
  if (!statusEl || !navigationPaneVisible() || introVisible) {
    if (statusEl) statusEl.hidden = true;
    return;
  }
  const source = health.sourceSnapshot(now);
  const presentation = compositor.snapshot();
  if (source.seen && !source.connected) {
    showStatus("disconnected");
  } else if (presentation.hasFrame && !compositor.fresh(health.thresholds.mapStaleMs, now)) {
    showStatus("stalled");
  } else if (!presentation.hasFrame || decodeError) {
    showStatus("waiting", decodeError ? "지도 디코더 복구 중" : "");
  } else {
    statusEl.hidden = true;
    delete statusEl.dataset.tone;
  }
}

function handleDecodeError(error) {
  decodeError = String(error?.message || error || "Carrot Navi decode error");
  if (environmentActive) lifecycle.transition("recovering", decodeError);
  showStatus("waiting", "지도 디코더 복구 중");
  requestMediaRecovery(decodeError, 120);
  void reportClientDiagnostic();
}

function handlePresentedFrame() {
  target.clearTimeout(mediaRecoveryTimer);
  mediaRecoveryTimer = 0;
  pendingRecoveryReason = "";
  decodeError = "";
  lifecycle.transition("live", "frame presented");
  if (statusEl) statusEl.hidden = true;
  evaluateSplit();
}

function handleNavigationState(state) {
  health.recordState(state);
  overlayStore.setState(state);
  evaluateSplit();
}

function handleStateDisconnected() {
  health.recordDisconnect();
  evaluateSplit();
}

function handleOwnershipBusy(code) {
  target.clearTimeout(mediaRecoveryTimer);
  mediaRecoveryTimer = 0;
  pendingRecoveryReason = "";
  mse.destroy();
  compositor.invalidate();
  overlayStore.reset();
  resetSourceState();
  decodeError = "";
  lifecycle.transition("busy", String(code || "carrot_navi_busy"));
  evaluateSplit();
  void reportClientDiagnostic();
}

function handleConnectionChange(state) {
  if (!environmentActive) return;
  if (state.ownershipBlocked) {
    lifecycle.transition("busy", state.ownershipCode || "carrot_navi_busy");
    evaluateSplit();
    return;
  }
  const phase = lifecycle.snapshot().phase;
  if (!state.mediaOpen && phase === "live") lifecycle.transition("recovering", "media socket closed");
  else if ((state.mediaOpen || state.stateOpen) && phase === "idle") lifecycle.transition("connecting", "socket connected");
}

function handleWorkerMessage(message = {}) {
  if (message.type === "overlay-image" && message.bitmap) {
    overlayStore.present(message.name, message.bitmap, message.sequence, message.sourceTimestampMillis);
    return;
  }
  if (message.type === "overlay-bytes") {
    void overlayStore.decode(message);
    return;
  }
  if (message.type === "overlay-clear") {
    overlayStore.clear(message.name, message.sequence, message.sourceTimestampMillis);
    return;
  }
  if (message.type === "overlay-reset") {
    overlayStore.resetImages();
    return;
  }
  if (message.type === "overlay-error") return;
  if (message.type === "pipeline") {
    mediaPipeline = String(message.mode || "");
    return;
  }
  if (message.type === "mse-init" && message.segment instanceof ArrayBuffer) {
    lifecycle.transition("buffering", "MSE initialization");
    mse.start(message.segment, String(message.mime || DEFAULT_MIME));
    return;
  }
  if (message.type === "mse-segment" && message.segment instanceof ArrayBuffer) {
    if (lifecycle.snapshot().phase !== "live") lifecycle.transition("buffering", "MSE segment received");
    mse.append(message.segment, message);
    return;
  }
  if (message.type === "clear") {
    compositor.invalidate({ preserveFrame: true });
    mse.markClear(message.reason);
    lifecycle.transition("recovering", String(message.reason || "media cleared"));
    evaluateSplit();
    void reportClientDiagnostic();
    return;
  }
  if (message.type === "error") handleDecodeError(message.message || "decode error");
}

function evaluateSplit() {
  const now = target.performance.now();
  updateRuntimeStatus(now, syncIntro());
}

function requestMediaRecovery(reason, minimumDelayMs = 0) {
  if (!environmentActive || transport.snapshot().ownershipBlocked) return;
  pendingRecoveryReason = String(reason || "Carrot Navi media recovery").slice(0, 128);
  lifecycle.transition("recovering", pendingRecoveryReason);
  if (mediaRecoveryTimer) return;
  const now = target.performance.now();
  const cooldownRemaining = lastMseRecoveryAt > 0
    ? Math.max(0, (lastMseRecoveryAt + MSE_RECOVERY_COOLDOWN_MS) - now)
    : 0;
  const delay = Math.max(0, Number(minimumDelayMs) || 0, cooldownRemaining);
  mediaRecoveryTimer = target.setTimeout(() => {
    mediaRecoveryTimer = 0;
    if (!environmentActive) return;
    lastMseRecoveryAt = target.performance.now();
    decodeError = pendingRecoveryReason || "Carrot Navi media recovery";
    pendingRecoveryReason = "";
    mse.destroy();
    transport.recoverMedia();
    showStatus("waiting");
    void reportClientDiagnostic();
  }, delay);
}

function recoverMseIfStalled(now = target.performance.now()) {
  const presentation = compositor.snapshot();
  if (!environmentActive || transport.snapshot().ownershipBlocked
      || !presentation.hasFrame || compositor.fresh(health.thresholds.mapStaleMs, now)) return;
  if (!health.mediaFresh(now)
      || presentation.lastFrameAt <= 0
      || now - presentation.lastFrameAt < MSE_STALL_RECOVERY_MS
      || mediaRecoveryTimer) return;
  requestMediaRecovery("Carrot Navi MSE playback recovery");
}

function clientDiagnosticPayload() {
  const now = target.performance.now();
  const splitState = split.snapshot();
  const transportState = transport.snapshot();
  const decoderState = mse.snapshot();
  const presentation = compositor.snapshot();
  return {
    decoderMode: decoderMode(),
    mediaPipeline,
    pipelineVersion: PIPELINE_VERSION,
    secureContext: target.isSecureContext,
    presentationMode: "back-buffered-canvas",
    lifecycle: lifecycle.snapshot(),
    environmentActive,
    splitOrientation: splitState.orientation,
    splitActive: splitState.active,
    splitMode: splitState.mode,
    splitArea1Content: splitState.area1Content,
    health: health.snapshot(decodeError, now),
    sourceState: health.sourceSnapshot(now),
    overlay: overlay.snapshot(),
    mediaSocketState: transportState.mediaSocketState,
    ownership: {
      blocked: transportState.ownershipBlocked,
      code: transportState.ownershipCode,
      takeoverArmed: transportState.takeoverArmed,
    },
    decodeError: decodeError || decoderState.error,
    userAgent: String(target.navigator.userAgent || "").slice(0, 256),
    media: decoderState.media,
    mse: {
      ...decoderState.mse,
      hasFrame: presentation.hasFrame,
      videoVisible: false,
      canvasVisible: presentation.canvasVisible,
    },
  };
}

async function reportClientDiagnostic() {
  if ((!environmentActive && !decodeError) || clientDiagnosticPending) return;
  clientDiagnosticPending = true;
  try {
    await target.fetch("/api/carrot_navi/client_diagnostic", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(clientDiagnosticPayload()),
      cache: "no-store",
      keepalive: true,
    });
  } catch (_) {
  } finally {
    clientDiagnosticPending = false;
  }
}

function resetSourceState() {
  health.reset();
  mediaPipeline = "";
}

function startRuntime() {
  if (environmentActive) return;
  environmentActive = true;
  compositor.setActive(true);
  lifecycle.transition("connecting", "environment eligible");
  transport.start();
}

function stopRuntime(reason = "environment inactive") {
  environmentActive = false;
  target.clearTimeout(mediaRecoveryTimer);
  mediaRecoveryTimer = 0;
  pendingRecoveryReason = "";
  lastMseRecoveryAt = 0;
  transport.stop();
  mse.destroy();
  compositor.setActive(false);
  compositor.invalidate();
  overlayStore.reset();
  resetSourceState();
  decodeError = "";
  lifecycle.transition("idle", reason);
}

function armOwnershipTakeover(reason = "user foreground entry") {
  transport.requestOwnershipTakeover();
  if (environmentActive) lifecycle.transition("connecting", String(reason || "user foreground entry"));
}

function requestOwnershipTakeover(reason = "user foreground entry") {
  armOwnershipTakeover(reason);
  evaluateEnvironment();
}

function setContentActive(value, options = {}) {
  const nextActive = Boolean(value);
  const nextKeepWarm = !nextActive && environmentActive && Boolean(options.keepWarm);
  const changed = contentActive !== nextActive || contentKeepWarm !== nextKeepWarm;
  const shouldTakeover = nextActive && !contentActive;
  contentActive = nextActive;
  contentKeepWarm = nextKeepWarm;
  if (shouldTakeover) armOwnershipTakeover(options.reason);
  evaluateEnvironment();
  return changed;
}

function resizeContent() {
  if (!contentActive && !contentKeepWarm) return false;
  const rendered = compositor.render();
  overlay.requestRender();
  evaluateSplit();
  return rendered;
}

function evaluateEnvironment() {
  layoutVisible = isWorkspaceLayoutVisible();
  const runtimeEligible = isEnvironmentEligible();
  if (runtimeEligible) {
    startRuntime();
    transport.start();
  } else if (environmentActive) {
    if (!canRetainRuntime()) stopRuntime("hard environment exit");
    else transport.start();
  }
  evaluateSplit();
}

for (const eventName of [
  "resize",
  "orientationchange",
  "carrot:viewportlayout",
  "carrot:pagechange",
]) target.addEventListener(eventName, evaluateEnvironment, { passive: true });
target.addEventListener("carrot:websettingschange", (event) => {
  const key = event?.detail?.key;
  const keys = Array.isArray(event?.detail?.keys) ? event.detail.keys : [key];
  if (keys.some((entry) => layoutSpec.settingKeys.includes(entry))) {
    evaluateEnvironment();
  }
});
documentRoot.addEventListener("visibilitychange", evaluateEnvironment);
target.addEventListener("carrot:languagechange", evaluateSplit);
if (target.visualViewport) target.visualViewport.addEventListener("resize", evaluateEnvironment, { passive: true });
if (typeof target.ResizeObserver === "function") new target.ResizeObserver(evaluateEnvironment).observe(workspace);
new target.MutationObserver(evaluateEnvironment).observe(stage, { attributes: true, attributeFilter: ["class"] });
target.setInterval(() => {
  evaluateEnvironment();
  evaluateSplit();
  recoverMseIfStalled();
}, 500);
target.setInterval(() => void reportClientDiagnostic(), CLIENT_DIAGNOSTIC_INTERVAL_MS);
evaluateEnvironment();

return Object.freeze({
  refresh: evaluateEnvironment,
  requestOwnershipTakeover,
  setContentActive,
  resize: resizeContent,
  status() {
    const splitState = split.snapshot();
    const diagnostic = clientDiagnosticPayload();
    return {
      enabled: true,
      contentActive,
      contentKeepWarm,
      streamGateAllowed,
      streamGateKnown,
      streamGatePending,
      environmentActive,
      layoutVisible,
      splitOrientation: splitState.orientation,
      splitActive: splitState.active,
      splitRatio: splitState.ratio,
      splitMode: splitState.mode,
      splitArea1Content: splitState.area1Content,
      intro: intro.snapshot(),
      ownershipNotice: ownershipSurface.snapshot(),
      mapReady: health.mapReady(),
      phase: lifecycle.snapshot(),
      health: diagnostic.health,
      sourceState: diagnostic.sourceState,
      overlay: diagnostic.overlay,
      decoderMode: diagnostic.decoderMode,
      mediaPipeline,
      pipelineVersion: PIPELINE_VERSION,
      secureContext: target.isSecureContext,
      decodeError: diagnostic.decodeError,
      ownership: diagnostic.ownership,
      media: diagnostic.media,
      mse: diagnostic.mse,
    };
  },
});

}

export function getCarrotNaviRuntime(options = {}) {
  const target = options.target || globalThis;
  const existing = runtimeSingletons.get(target);
  if (existing) return existing;
  const runtime = createCarrotNaviRuntime(options);
  if (runtime && typeof runtime.setContentActive === "function") runtimeSingletons.set(target, runtime);
  return runtime;
}

export function installCarrotNaviRuntimeGlobal(target = globalThis, options = {}) {
  const runtime = getCarrotNaviRuntime({ ...options, target });
  target.CarrotNaviWeb = runtime;
  return runtime;
}
