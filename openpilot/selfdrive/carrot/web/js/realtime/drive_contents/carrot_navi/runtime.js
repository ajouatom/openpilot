"use strict";

window.CarrotNaviWeb = (() => {
  const workspace = document.getElementById("carrotDriveWorkspace");
  const stage = document.getElementById("carrotStage");
  const pane = document.getElementById("carrotNaviPane");
  const divider = document.getElementById("carrotNaviDivider");
  const canvas = document.getElementById("carrotNaviCanvas");
  const video = document.getElementById("carrotNaviVideo");
  const overlayCanvas = document.getElementById("carrotNaviOverlayCanvas");
  const statusEl = document.getElementById("carrotNaviStatus");
  const layoutSpec = globalThis.CarrotDriveLayoutSpec;
  if (!workspace || !stage || !pane || !divider || !canvas || !video || !overlayCanvas || !layoutSpec) return {};

  const overlay = globalThis.CarrotNaviOverlay?.create?.(overlayCanvas);
  const overlayStore = globalThis.CarrotNaviOverlayStore?.create?.(overlay);
  const lifecycle = globalThis.CarrotNaviLifecycle?.create?.();
  const split = globalThis.CarrotNaviSplit?.create?.({
    workspace,
    stage,
    pane,
    divider,
    overlay,
  });
  if (!overlay || !overlayStore || !lifecycle || !split) return {};

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
  let contentActive = true;
  let contentKeepWarm = false;

  const compositor = globalThis.CarrotNaviCompositor?.create?.({
    pane,
    canvas,
    video,
    overlayCanvas,
    overlayStore,
    onFrame: handlePresentedFrame,
  });
  const mse = globalThis.CarrotNaviMse?.create?.(video, {
    onFrame: () => compositor?.render(),
    onError: handleDecodeError,
  });
  if (!compositor || !mse) return {};

  const transport = globalThis.CarrotNaviTransport?.create?.({
    workerUrl: "/js/realtime/drive_contents/carrot_navi/worker.js?v=2607-09",
    onState: handleNavigationState,
    onStateDisconnected: handleStateDisconnected,
    onWorkerMessage: handleWorkerMessage,
    onError: handleDecodeError,
    onConnectionChange: handleConnectionChange,
  });
  if (!transport) return {};
  const health = globalThis.CarrotNaviHealth?.create?.({ transport, mse, compositor });
  if (!health) return {};
  const intro = globalThis.DriveContentIntro?.create?.({
    host: pane,
    className: "carrot-navi-pane__intro",
  });
  if (!intro) return {};

  function featureEnabled() {
    return window.CarrotWebSettingsState?.carrot_navi_enabled !== false;
  }

  function decoderMode() {
    return mse.supported() ? "mse" : "";
  }

  function viewportSize() {
    const viewport = window.visualViewport;
    return {
      width: Math.round(viewport?.width || window.innerWidth || 0),
      height: Math.round(viewport?.height || window.innerHeight || 0),
    };
  }

  function workspaceOrientation(viewport = viewportSize()) {
    const shared = globalThis.DriveWorkspaceRuntime?.viewportOrientation?.();
    if (shared) return shared;
    return viewport.height >= viewport.width
      ? globalThis.DriveWorkspace.ORIENTATION.VERTICAL
      : globalThis.DriveWorkspace.ORIENTATION.HORIZONTAL;
  }

  async function refreshStreamGate(force = false) {
    const now = performance.now();
    if (streamGatePending || (!force && streamGateKnown && now - streamGateCheckedAt < STREAM_GATE_POLL_MS)) return;
    streamGatePending = true;
    streamGateCheckedAt = now;
    let allowed = false;
    try {
      const response = await fetch("/api/carrot_navi/status", { cache: "no-store" });
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
    return document.body.dataset.page === "carrot"
      && !document.hidden
      && !stage.classList.contains("is-replay");
  }

  function isEnvironmentEligible() {
    const orientation = workspaceOrientation();
    const workspaceRect = workspace.getBoundingClientRect();
    const workspaceWidth = workspaceRect.width || workspace.clientWidth || 0;
    const workspaceHeight = workspaceRect.height || workspace.clientHeight || 0;
    const sizeEligible = orientation === globalThis.DriveWorkspace.ORIENTATION.VERTICAL
      ? workspaceWidth >= MIN_PORTRAIT_WORKSPACE_WIDTH && workspaceHeight >= MIN_PORTRAIT_WORKSPACE_HEIGHT
      : workspaceWidth >= MIN_LANDSCAPE_WORKSPACE_WIDTH && workspaceHeight >= MIN_LANDSCAPE_WORKSPACE_HEIGHT;
    const localEligible = contentActive
      && layoutVisible
      && navigationPaneVisible()
      && featureEnabled()
      && sizeEligible
      && typeof Worker === "function"
      && Boolean(decoderMode());
    if (localEligible) void refreshStreamGate();
    return localEligible && streamGateKnown && streamGateAllowed;
  }

  function canRetainRuntime() {
    return (contentActive || contentKeepWarm)
      && featureEnabled()
      && streamGateKnown
      && streamGateAllowed
      && typeof Worker === "function"
      && Boolean(decoderMode());
  }

  function uiText(key, fallback) {
    return typeof getUIText === "function" ? getUIText(key, fallback) : fallback;
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
    if (!navigationPaneVisible(splitState) || presentation.hasFrame) {
      intro.hide();
      return false;
    }

    const label = uiText("web_drive_layout_content_navigation", "Carrot Navi");
    let view;
    if (!featureEnabled()) {
      view = {
        state: "disabled",
        title: uiText("carrot_navi_intro_disabled", "Carrot Navi is turned off"),
        detail: uiText("carrot_navi_intro_disabled_detail", "This area stays in place and will show navigation when enabled in Web settings."),
      };
    } else if (typeof Worker !== "function" || !decoderMode()) {
      view = {
        state: "unavailable",
        title: uiText("carrot_navi_intro_unsupported", "Navigation display is not supported"),
        detail: uiText("carrot_navi_intro_unsupported_detail", "This browser cannot start the navigation video pipeline."),
      };
    } else if (streamGateKnown && !streamGateAllowed) {
      view = {
        state: "unavailable",
        title: uiText("carrot_navi_intro_unavailable", "Carrot Navi is unavailable"),
        detail: uiText("carrot_navi_intro_unavailable_detail", "The stream remains stopped for the current device state."),
      };
    } else if (decodeError) {
      view = {
        state: "recovering",
        title: uiText("carrot_navi_intro_recovering", "Recovering the navigation screen"),
        detail: uiText("carrot_navi_intro_recovering_detail", "The screen will return automatically after recovery."),
        busy: true,
      };
    } else {
      view = {
        state: "connecting",
        title: uiText("carrot_navi_intro_connecting", "Connecting to Carrot Navi"),
        detail: uiText("carrot_navi_intro_connecting_detail", "The navigation screen will appear here when the stream is ready."),
        busy: true,
      };
    }
    intro.show({ label, ...view });
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
    statusEl.textContent = fallbackText || (typeof getUIText === "function" ? getUIText(key, defaultText) : defaultText);
    statusEl.dataset.tone = mode === "stalled" ? "warning" : mode;
    statusEl.hidden = false;
  }

  function updateRuntimeStatus(now = performance.now(), introVisible = false) {
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
    globalThis.clearTimeout(mediaRecoveryTimer);
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

  function handleConnectionChange(state) {
    if (!environmentActive) return;
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
      mse.start(message.segment, String(message.mime || globalThis.CarrotNaviMse.DEFAULT_MIME));
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
    const now = performance.now();
    updateRuntimeStatus(now, syncIntro());
  }

  function requestMediaRecovery(reason, minimumDelayMs = 0) {
    if (!environmentActive) return;
    pendingRecoveryReason = String(reason || "Carrot Navi media recovery").slice(0, 128);
    lifecycle.transition("recovering", pendingRecoveryReason);
    if (mediaRecoveryTimer) return;
    const now = performance.now();
    const cooldownRemaining = lastMseRecoveryAt > 0
      ? Math.max(0, (lastMseRecoveryAt + MSE_RECOVERY_COOLDOWN_MS) - now)
      : 0;
    const delay = Math.max(0, Number(minimumDelayMs) || 0, cooldownRemaining);
    mediaRecoveryTimer = globalThis.setTimeout(() => {
      mediaRecoveryTimer = 0;
      if (!environmentActive) return;
      lastMseRecoveryAt = performance.now();
      decodeError = pendingRecoveryReason || "Carrot Navi media recovery";
      pendingRecoveryReason = "";
      mse.destroy();
      transport.recoverMedia();
      showStatus("waiting");
      void reportClientDiagnostic();
    }, delay);
  }

  function recoverMseIfStalled(now = performance.now()) {
    const presentation = compositor.snapshot();
    if (!environmentActive || !presentation.hasFrame || compositor.fresh(health.thresholds.mapStaleMs, now)) return;
    if (!health.mediaFresh(now)
        || presentation.lastFrameAt <= 0
        || now - presentation.lastFrameAt < MSE_STALL_RECOVERY_MS
        || mediaRecoveryTimer) return;
    requestMediaRecovery("Carrot Navi MSE playback recovery");
  }

  function clientDiagnosticPayload() {
    const now = performance.now();
    const splitState = split.snapshot();
    const transportState = transport.snapshot();
    const decoderState = mse.snapshot();
    const presentation = compositor.snapshot();
    return {
      decoderMode: decoderMode(),
      mediaPipeline,
      pipelineVersion: PIPELINE_VERSION,
      secureContext: window.isSecureContext,
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
      decodeError: decodeError || decoderState.error,
      userAgent: String(navigator.userAgent || "").slice(0, 256),
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
      await fetch("/api/carrot_navi/client_diagnostic", {
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
    globalThis.clearTimeout(mediaRecoveryTimer);
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

  function setContentActive(value, options = {}) {
    const nextActive = Boolean(value);
    const nextKeepWarm = !nextActive && environmentActive && Boolean(options.keepWarm);
    const changed = contentActive !== nextActive || contentKeepWarm !== nextKeepWarm;
    contentActive = nextActive;
    contentKeepWarm = nextKeepWarm;
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
  ]) window.addEventListener(eventName, evaluateEnvironment, { passive: true });
  window.addEventListener("carrot:websettingschange", (event) => {
    const key = event?.detail?.key;
    const keys = Array.isArray(event?.detail?.keys) ? event.detail.keys : [key];
    if (keys.includes("carrot_navi_enabled") || keys.some((entry) => layoutSpec.settingKeys.includes(entry))) {
      evaluateEnvironment();
    }
  });
  document.addEventListener("visibilitychange", evaluateEnvironment);
  window.addEventListener("carrot:languagechange", evaluateSplit);
  if (window.visualViewport) window.visualViewport.addEventListener("resize", evaluateEnvironment, { passive: true });
  if (typeof ResizeObserver === "function") new ResizeObserver(evaluateEnvironment).observe(workspace);
  new MutationObserver(evaluateEnvironment).observe(stage, { attributes: true, attributeFilter: ["class"] });
  window.setInterval(() => {
    evaluateEnvironment();
    evaluateSplit();
    recoverMseIfStalled();
  }, 500);
  window.setInterval(() => void reportClientDiagnostic(), CLIENT_DIAGNOSTIC_INTERVAL_MS);
  evaluateEnvironment();

  return Object.freeze({
    refresh: evaluateEnvironment,
    setContentActive,
    resize: resizeContent,
    status() {
      const splitState = split.snapshot();
      const diagnostic = clientDiagnosticPayload();
      return {
        enabled: featureEnabled(),
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
        mapReady: health.mapReady(),
        phase: lifecycle.snapshot(),
        health: diagnostic.health,
        sourceState: diagnostic.sourceState,
        overlay: diagnostic.overlay,
        decoderMode: diagnostic.decoderMode,
        mediaPipeline,
        pipelineVersion: PIPELINE_VERSION,
        secureContext: window.isSecureContext,
        decodeError: diagnostic.decodeError,
        media: diagnostic.media,
        mse: diagnostic.mse,
      };
    },
  });
})();
