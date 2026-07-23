"use strict";

window.CarrotVisionReplay = window.CarrotVisionReplay || (() => {
  const REPLAY_HUD_SETTING_KEY = "replay_hud_visible";
  const stageEl = document.getElementById("carrotStage");
  const videoEl = document.getElementById("carrotRoadVideo");
  const controlsEl = document.getElementById("carrotReplayControls");
  const closeButton = document.getElementById("btnCarrotReplayClose");
  const playButton = document.getElementById("btnCarrotReplayPlay");
  const speedButton = document.getElementById("btnCarrotReplaySpeed");
  const seekEl = document.getElementById("carrotReplaySeek");
  const currentTimeEl = document.getElementById("carrotReplayCurrentTime");
  const durationEl = document.getElementById("carrotReplayDuration");
  const filmstripEl = document.getElementById("carrotReplayFilmstrip");
  const thumbnailEl = document.getElementById("carrotReplayThumbnail");
  const scrubPreviewEl = document.getElementById("carrotReplayScrubPreview");
  const scrubImageEl = document.getElementById("carrotReplayScrubImage");
  const scrubTimeEl = document.getElementById("carrotReplayScrubTime");
  const scrubEventInfoEl = document.getElementById("carrotReplayScrubEventInfo");
  const scrubEventIconEl = document.getElementById("carrotReplayScrubEventIcon");
  const scrubEventTitleEl = document.getElementById("carrotReplayScrubEventTitle");
  const scrubEventMetaEl = document.getElementById("carrotReplayScrubEventMeta");
  const scrubEventsEl = document.getElementById("carrotReplayScrubEvents");
  const scrubEventsCanvasEl = document.getElementById("carrotReplayScrubEventsCanvas");
  const titleEl = document.getElementById("carrotReplayTitle");
  const statusEl = document.getElementById("carrotReplayStatus");
  const hudControlEl = document.getElementById("carrotReplayHudControl");
  const hudToggleEl = document.getElementById("carrotReplayHudToggle");
  const hudToggleLabelEl = document.getElementById("carrotReplayHudToggleLabel");
  const segmentPickerEl = document.getElementById("carrotReplaySegmentPicker");
  const segmentSelectEl = document.getElementById("carrotReplaySegmentSelect");
  const segmentNavEl = document.getElementById("carrotReplaySegmentNav");
  const previousSegmentButton = document.getElementById("btnCarrotReplayPrevious");
  const nextSegmentButton = document.getElementById("btnCarrotReplayNext");
  const insights = window.CarrotReplayInsights;

  const SEEK_FRAME_HOLD_MS = 350;
  const STAGE_SCRUB_THRESHOLD_PX = 8;
  const STAGE_SCRUB_SPAN_SECONDS = 20;
  const REPLAY_SPEEDS = Object.freeze([0.5, 1, 2]);
  let transport = null;
  let controlsHideTimer = null;
  let suppressStageClickUntilMs = 0;
  const stageScrub = {
    pointerId: null,
    startX: 0,
    startY: 0,
    startTime: 0,
    pendingTime: null,
    frameId: null,
    active: false,
    cancelled: false,
    resumeAfter: false,
  };

  const state = {
    active: false,
    ready: false,
    loading: false,
    token: 0,
    requestToken: 0,
    previousVisionActive: false,
    route: "",
    segment: "",
    segments: [],
    title: "",
    titleForSegment: null,
    manifest: null,
    records: [],
    nextRecord: 0,
    lastTimeMs: -1,
    sourceKey: "",
    previewSource: null,
    previewsRequested: false,
    scrubbing: false,
    seekHoldUntilMs: 0,
    hasRenderableFrame: false,
    videoFrameId: null,
    rafId: null,
    abortController: null,
    clientSession: null,
    hudVisible: typeof window.getWebSettingByKey === "function"
      ? Boolean(window.getWebSettingByKey(REPLAY_HUD_SETTING_KEY, false))
      : Boolean(window.CarrotWebSettingsState?.[REPLAY_HUD_SETTING_KEY]),
    timelineReady: false,
  };

  function text(key, fallback, vars = null) {
    return typeof getUIText === "function" ? getUIText(key, fallback, vars) : fallback;
  }

  function replayDuration() {
    const mediaDuration = Number(videoEl?.duration);
    if (Number.isFinite(mediaDuration) && mediaDuration > 0) return mediaDuration;
    return Math.max(0, Number(state.manifest?.durationMs || 0) / 1000);
  }

  function replayStatusText() {
    if (state.loading) return text("replay_loading", "Preparing drive replay...");
    if (videoEl?.ended) return text("replay_finished", "Replay finished");
    if (videoEl?.paused) return text("replay_paused", "Replay paused");
    return text("replay_playing", "Playing drive record");
  }

  function replayDetailText() {
    return text("replay_detail", "Saved road video and driving information are shown together.");
  }

  function setVisionPhase(phase, statusText = replayStatusText(), detailText = replayDetailText()) {
    if (!state.active || typeof window.CarrotVisionSetPhase !== "function") return;
    window.CarrotVisionSetPhase(phase, {
      reason: "recorded-replay",
      statusText,
      detailText,
      updateRtcStatus: false,
    });
  }

  function normalizeSegments(segments, activeSegment) {
    const normalized = [];
    const seen = new Set();
    for (const value of Array.isArray(segments) ? segments : []) {
      const segment = String(value || "").trim();
      if (!segment || seen.has(segment)) continue;
      seen.add(segment);
      normalized.push(segment);
    }
    if (activeSegment && !seen.has(activeSegment)) normalized.push(activeSegment);
    return normalized;
  }

  function replayTitleForSegment(segment) {
    if (typeof state.titleForSegment === "function") {
      try {
        const resolved = String(state.titleForSegment(segment) || "").trim();
        if (resolved) return resolved;
      } catch {}
    }
    if (segment === state.segment && state.title) return state.title;
    return String(segment || "");
  }

  function syncSegmentPicker(index, total) {
    if (!segmentSelectEl) return;
    const expectedSegments = state.active ? state.segments : [];
    const expectedLabels = expectedSegments.map((segment) => replayTitleForSegment(segment));
    const currentOptions = Array.from(segmentSelectEl.options);
    const optionsChanged = currentOptions.length !== expectedSegments.length
      || currentOptions.some((option, optionIndex) => (
        option.value !== expectedSegments[optionIndex]
        || option.textContent !== expectedLabels[optionIndex]
      ));
    if (optionsChanged) {
      segmentSelectEl.replaceChildren(...expectedSegments.map((segment, optionIndex) => {
        const option = document.createElement("option");
        option.value = segment;
        option.textContent = expectedLabels[optionIndex];
        return option;
      }));
    }
    const showPicker = state.active && total > 0;
    if (segmentPickerEl) segmentPickerEl.hidden = !showPicker;
    segmentSelectEl.disabled = state.loading || total <= 1;
    if (index >= 0) segmentSelectEl.value = state.segment;
    const label = text("replay_select_segment", "Select replay segment");
    segmentSelectEl.setAttribute("aria-label", label);
    segmentSelectEl.title = label;
  }

  function syncSegmentNavigation() {
    const index = state.segments.indexOf(state.segment);
    const total = state.segments.length;
    const hasNavigation = state.active && index >= 0 && total > 1;
    if (titleEl && state.active) titleEl.textContent = replayTitleForSegment(state.segment);
    syncSegmentPicker(index, total);
    if (segmentNavEl) segmentNavEl.hidden = !state.active;
    if (previousSegmentButton) {
      const label = text("replay_previous_segment", "Previous segment");
      previousSegmentButton.disabled = state.loading || index <= 0;
      previousSegmentButton.hidden = !hasNavigation;
      previousSegmentButton.setAttribute("aria-label", label);
      previousSegmentButton.title = label;
    }
    if (nextSegmentButton) {
      const label = text("replay_next_segment", "Next segment");
      nextSegmentButton.disabled = state.loading || index < 0 || index >= total - 1;
      nextSegmentButton.hidden = !hasNavigation;
      nextSegmentButton.setAttribute("aria-label", label);
      nextSegmentButton.title = label;
    }
  }

  function syncTransport() {
    stageEl?.classList.toggle("is-replay-loading", state.active && state.loading);
    syncSegmentNavigation();
    transport?.sync();
    insights?.syncTime?.(Number(videoEl?.currentTime || 0), replayDuration());
  }

  function setReplayHudVisible(visible, options = {}) {
    state.hudVisible = Boolean(visible);
    if (hudToggleEl) hudToggleEl.checked = state.hudVisible;
    hudControlEl?.classList.toggle("is-active", state.hudVisible);
    stageEl?.classList.toggle("is-replay-hud-visible", state.hudVisible);
    document.body?.classList.toggle("carrot-replay-hud-visible", state.active && state.hudVisible);
    if (state.active) window.requestCarrotVisionRender?.({ reason: "replay HUD visibility changed" });
    if (options.persist === true) {
      const request = window.setWebSettingByKey?.(REPLAY_HUD_SETTING_KEY, state.hudVisible);
      request?.catch?.(() => {});
    }
  }

  function syncReplayHudFromServer(event) {
    const keys = Array.isArray(event?.detail?.keys) ? event.detail.keys : [];
    if (keys.length && !keys.includes(REPLAY_HUD_SETTING_KEY)) return;
    const visible = typeof window.getWebSettingByKey === "function"
      ? window.getWebSettingByKey(REPLAY_HUD_SETTING_KEY, false)
      : window.CarrotWebSettingsState?.[REPLAY_HUD_SETTING_KEY];
    setReplayHudVisible(Boolean(visible));
  }

  function syncReplayLabels() {
    transport?.syncLabels();
    playButton?.setAttribute("aria-keyshortcuts", "Space K");
    seekEl?.setAttribute("aria-keyshortcuts", "ArrowLeft ArrowRight J L Home End");
    speedButton?.setAttribute("aria-keyshortcuts", "Shift+, Shift+.");
    previousSegmentButton?.setAttribute("aria-keyshortcuts", "Shift+P");
    nextSegmentButton?.setAttribute("aria-keyshortcuts", "Shift+N");
    if (hudToggleLabelEl) hudToggleLabelEl.textContent = text("replay_show_hud", "HUD");
    hudToggleEl?.setAttribute("aria-label", text("replay_show_hud", "HUD"));
    syncSegmentNavigation();
    if (state.ready) setVisionPhase(window.CarrotVisionPhase?.READY || "ready");
  }

  function updateSegments(segments, route = "") {
    if (!state.active || (route && route !== state.route)) return false;
    const nextSegments = normalizeSegments(segments, state.segment);
    if (nextSegments.length === state.segments.length
      && nextSegments.every((segment, index) => segment === state.segments[index])) return false;
    state.segments = nextSegments;
    syncSegmentNavigation();
    return true;
  }

  function setReplayUiVisible(visible) {
    const show = Boolean(visible);
    let workspaceManaged = false;
    if (show && typeof window.DriveWorkspaceRuntime?.setReplayPresentation === "function") {
      workspaceManaged = window.DriveWorkspaceRuntime.setReplayPresentation(true) === true;
    }
    if (controlsEl) controlsEl.hidden = !show;
    if (hudControlEl) hudControlEl.hidden = !show;
    if (speedButton) speedButton.hidden = !show;
    stageEl?.classList.toggle("is-replay", show);
    stageEl?.classList.toggle("is-replay-loading", show && state.loading);
    stageEl?.classList.remove("is-replay-controls-hidden");
    document.body?.classList.toggle("carrot-replay-active", show);
    document.body?.classList.toggle("carrot-replay-hud-visible", show && state.hudVisible);
    insights?.setActive?.(show && state.timelineReady);
    syncSegmentNavigation();
    if (!show && typeof window.DriveWorkspaceRuntime?.setReplayPresentation === "function") {
      window.DriveWorkspaceRuntime.setReplayPresentation(false);
      workspaceManaged = true;
    }
    if (!show && controlsHideTimer != null) {
      window.clearTimeout(controlsHideTimer);
      controlsHideTimer = null;
    }
    return workspaceManaged;
  }

  function loadReplayInsights(options, signal) {
    if (typeof insights?.load !== "function") return Promise.resolve();
    return new Promise((resolve, reject) => {
      let settled = false;
      const finish = () => {
        if (settled) return;
        settled = true;
        signal?.removeEventListener?.("abort", finish);
        resolve();
      };
      if (signal?.aborted) {
        finish();
        return;
      }
      signal?.addEventListener?.("abort", finish, { once: true });
      try {
        insights.load({ ...options, onReady: finish });
      } catch (error) {
        settled = true;
        signal?.removeEventListener?.("abort", finish);
        reject(error);
      }
    });
  }

  function scheduleControlsHide() {
    if (controlsHideTimer != null) window.clearTimeout(controlsHideTimer);
    controlsHideTimer = null;
    if (!state.active || state.loading || !videoEl || videoEl.ended || insights?.hasOpenPopover?.()) return;
    controlsHideTimer = window.setTimeout(() => {
      controlsHideTimer = null;
      if (!state.active || videoEl.ended || insights?.hasOpenPopover?.()) return;
      stageEl?.classList.add("is-replay-controls-hidden");
    }, window.CarrotMediaTransport?.autoHideDelay?.() || 1400);
  }

  function revealReplayControls() {
    if (!state.active) return;
    stageEl?.classList.remove("is-replay-controls-hidden");
    scheduleControlsHide();
  }

  function hideReplayControls() {
    if (!state.active || state.loading) return;
    if (controlsHideTimer != null) window.clearTimeout(controlsHideTimer);
    controlsHideTimer = null;
    stageEl?.classList.add("is-replay-controls-hidden");
  }

  function toggleReplayControls() {
    if (stageEl?.classList.contains("is-replay-controls-hidden")) revealReplayControls();
    else hideReplayControls();
  }

  function seekReplayTo(seconds) {
    if (!state.active || state.loading || !state.sourceKey || !videoEl) return false;
    const target = Math.max(0, Math.min(replayDuration(), Number(seconds) || 0));
    beginScrub();
    videoEl.currentTime = target;
    applyAt(target * 1000, { reset: true });
    endScrub();
    syncTransport();
    revealReplayControls();
    return true;
  }

  async function navigateToSegment(segment) {
    if (!state.active || state.loading) return;
    const targetSegment = String(segment || "").trim();
    if (!targetSegment || targetSegment === state.segment || !state.segments.includes(targetSegment)) return;
    const segments = [...state.segments];
    const titleForSegment = state.titleForSegment;
    const route = state.route;
    await start({
      route,
      segment: targetSegment,
      segments,
      titleForSegment,
      title: replayTitleForSegment(targetSegment),
    });
  }

  async function navigateSegment(offset) {
    if (!state.active || state.loading) return;
    const currentIndex = state.segments.indexOf(state.segment);
    const targetIndex = currentIndex + Number(offset || 0);
    if (currentIndex < 0 || targetIndex < 0 || targetIndex >= state.segments.length) return;
    await navigateToSegment(state.segments[targetIndex]);
  }

  function replayShortcutBlocked(target) {
    if (!(target instanceof Element)) return false;
    return Boolean(target.closest(
      "input, textarea, select, button, a[href], summary, "
      + "[contenteditable]:not([contenteditable='false']), "
      + "[role='textbox'], [role='slider'], [role='tab'], [role='button'], "
      + ".carrot-replay-insights",
    ));
  }

  function toggleReplayPlayback() {
    if (!playButton || playButton.disabled) return false;
    playButton.click();
    return true;
  }

  function adjustReplayRate(direction) {
    const currentRate = Number(videoEl?.playbackRate) || 1;
    let currentIndex = REPLAY_SPEEDS.indexOf(currentRate);
    if (currentIndex < 0) {
      currentIndex = REPLAY_SPEEDS.reduce((bestIndex, rate, index) => (
        Math.abs(rate - currentRate) < Math.abs(REPLAY_SPEEDS[bestIndex] - currentRate) ? index : bestIndex
      ), 0);
    }
    const nextIndex = Math.max(0, Math.min(REPLAY_SPEEDS.length - 1, currentIndex + direction));
    if (nextIndex === currentIndex) return false;
    transport?.setRate?.(REPLAY_SPEEDS[nextIndex]);
    revealReplayControls();
    return true;
  }

  function handleReplayKeydown(event) {
    if (!state.active || state.loading || !state.sourceKey || document.hidden) return;
    if (document.body?.dataset?.page !== "carrot" || document.body?.classList.contains("dialog-open")) return;
    if (event.defaultPrevented || event.isComposing || event.ctrlKey || event.metaKey || event.altKey) return;
    if (replayShortcutBlocked(event.target)) return;

    let handled = false;
    if (event.code === "Space" || event.code === "KeyK") {
      if (event.repeat) return;
      handled = toggleReplayPlayback();
    } else if (event.code === "ArrowLeft") {
      seekReplayTo(Number(videoEl?.currentTime || 0) - 5);
      handled = true;
    } else if (event.code === "ArrowRight") {
      seekReplayTo(Number(videoEl?.currentTime || 0) + 5);
      handled = true;
    } else if (event.code === "KeyJ") {
      seekReplayTo(Number(videoEl?.currentTime || 0) - 10);
      handled = true;
    } else if (event.code === "KeyL") {
      seekReplayTo(Number(videoEl?.currentTime || 0) + 10);
      handled = true;
    } else if (event.code === "Home") {
      seekReplayTo(0);
      handled = true;
    } else if (event.code === "End") {
      seekReplayTo(replayDuration());
      handled = true;
    } else if (event.shiftKey && event.code === "Comma") {
      if (event.repeat) return;
      handled = adjustReplayRate(-1);
    } else if (event.shiftKey && event.code === "Period") {
      if (event.repeat) return;
      handled = adjustReplayRate(1);
    } else if (event.shiftKey && event.code === "KeyP") {
      if (event.repeat || previousSegmentButton?.disabled) return;
      handled = true;
      navigateSegment(-1).catch(() => syncSegmentNavigation());
    } else if (event.shiftKey && event.code === "KeyN") {
      if (event.repeat || nextSegmentButton?.disabled) return;
      handled = true;
      navigateSegment(1).catch(() => syncSegmentNavigation());
    }
    if (!handled) return;
    event.preventDefault();
    revealReplayControls();
  }

  function pauseReplayAtCurrentTime() {
    if (!state.active || state.loading || !state.sourceKey || !videoEl) return null;
    const currentSeconds = Math.max(0, Number(videoEl.currentTime || 0));
    videoEl.pause();
    syncTransport();
    revealReplayControls();
    return currentSeconds;
  }

  /* AR 자동 진단처럼 실제 replay 파이프라인을 순서대로 재생해야 하는 도구를 위한
   * 최소 제어면. DOM 버튼을 대신 클릭하지 않고, 시작 전 상태를 스냅샷으로 보관한
   * 뒤 수집 종료 시 그대로 복원한다. 일반 재생 UI는 이 API에 의존하지 않는다. */
  function diagnosticPlaybackSnapshot() {
    return Object.freeze({
      active: state.active,
      ready: state.ready,
      loading: state.loading,
      route: state.route || null,
      segment: state.segment || null,
      currentTime: Number(videoEl?.currentTime || 0),
      duration: replayDuration(),
      paused: videoEl?.paused !== false,
      ended: videoEl?.ended === true,
      playbackRate: Number(videoEl?.playbackRate || 1),
    });
  }

  function setDiagnosticPlaybackRate(value) {
    if (!videoEl) return false;
    const rate = Number(value);
    if (!Number.isFinite(rate) || rate <= 0) return false;
    if (typeof transport?.setRate === "function") transport.setRate(rate);
    else videoEl.playbackRate = rate;
    syncTransport();
    return true;
  }

  function playDiagnosticReplay() {
    if (!state.active || !state.ready || state.loading || !videoEl) {
      return Promise.reject(new Error("Replay is not ready for AR diagnostics"));
    }
    const result = videoEl.play?.();
    schedulePlaybackTick();
    syncTransport();
    revealReplayControls();
    return result && typeof result.then === "function" ? result : Promise.resolve();
  }

  async function restoreDiagnosticPlayback(snapshot = {}) {
    if (!videoEl || !state.active) return false;
    pauseReplayAtCurrentTime();
    setDiagnosticPlaybackRate(snapshot.playbackRate || 1);
    seekReplayTo(snapshot.currentTime || 0);
    if (snapshot.paused === false) await playDiagnosticReplay().catch(() => {});
    return true;
  }

  const diagnosticPlayback = Object.freeze({
    snapshot: diagnosticPlaybackSnapshot,
    pause: () => pauseReplayAtCurrentTime() !== null,
    seek: seekReplayTo,
    setRate: setDiagnosticPlaybackRate,
    play: playDiagnosticReplay,
    restore: restoreDiagnosticPlayback,
  });

  function maybeGenerateFilmstrip() {
    const previewSource = state.previewSource || (!state.manifest?.clientMode ? state.sourceKey : "");
    if (!state.active || state.previewsRequested || !previewSource) return;
    state.previewsRequested = true;
    const token = state.token;
    window.setTimeout(() => {
      if (!state.active || token !== state.token) return;
      transport?.generateFilmstrip?.(previewSource, { duration: replayDuration(), keepAlive: true });
    }, 450);
  }

  function cancelPlaybackTick() {
    if (state.videoFrameId != null && typeof videoEl?.cancelVideoFrameCallback === "function") {
      try { videoEl.cancelVideoFrameCallback(state.videoFrameId); } catch {}
    }
    if (state.rafId != null) window.cancelAnimationFrame(state.rafId);
    state.videoFrameId = null;
    state.rafId = null;
  }

  function decodeRecord(record) {
    if (!record.frames) {
      record.frames = window.CarrotVisionCompact?.decodeFrames?.(record.payload) || [];
    }
    return record.frames;
  }

  function presentAppliedState(result, options = {}) {
    // A native `seeking` event may arrive after the explicit seek already
    // rebuilt raw state. It still resets AR/overlay temporal state, so redraw
    // that same snapshot immediately instead of leaving a paused replay blank
    // until another compact record or video frame happens to arrive.
    const presentFrame = options.presentFrame === true;
    if (!result?.applied && !result?.reset && !result?.resetTemporal && !presentFrame) return false;
    const replayFacade = window.DriveVisionFacade?.replay;
    const canPresentFrame = typeof replayFacade?.renderVideoFrame === "function";
    if (!canPresentFrame && result.resetTemporal) replayFacade?.resetTemporalState?.();
    const rendered = replayFacade?.renderVideoFrame?.({
      force: Boolean(result.reset || result.resetTemporal),
      resetTemporal: Boolean(result.resetTemporal),
      overlayDirty: options.overlayDirty ?? Boolean(result.applied || result.reset || result.resetTemporal),
      hudDirty: options.hudDirty ?? Boolean(result.applied || result.reset || result.resetTemporal),
      mediaTime: Number.isFinite(Number(options.mediaTime)) ? Number(options.mediaTime) : null,
      reason: String(options.reason || "replay state applied"),
    });
    if (rendered) return true;
    window.requestCarrotVisionRender?.({
      force: Boolean(result.reset || result.resetTemporal),
      overlayDirty: true,
      hudDirty: true,
      reason: String(options.reason || "replay state applied"),
    });
    return false;
  }

  function applyAt(timeMs, options = {}) {
    if (!state.active || !state.records.length) return { applied: 0, reset: false, resetTemporal: false };
    const targetMs = Math.max(0, Number(timeMs) || 0);
    const reset = Boolean(options.reset || targetMs + 100 < state.lastTimeMs);
    const resetTemporal = Boolean(options.resetTemporal || reset);
    const render = options.render !== false;
    if (reset) {
      state.nextRecord = 0;
      window.CarrotVisionRaw?.clearState?.({ render: false, reason: "replay seek" });
    }

    const latestByService = new Map();
    while (state.nextRecord < state.records.length && state.records[state.nextRecord].timeMs <= targetMs) {
      const record = state.records[state.nextRecord];
      for (const frame of decodeRecord(record)) {
        if (!frame?.service) continue;
        latestByService.delete(frame.service);
        latestByService.set(frame.service, frame);
      }
      state.nextRecord += 1;
    }
    state.lastTimeMs = targetMs;
    let applied = 0;
    if (latestByService.size) {
      applied = Number(
        window.CarrotVisionRaw?.applyCompactFrames?.(Array.from(latestByService.values()), {
          reason: "recorded replay",
          // Replay presentation is synchronized below after the complete
          // batch has been applied. Per-service render requests would race
          // the visible video frame and bypass the shared presented channel.
          render: false,
          flushHud: true,
        }),
      ) || 0;
    }
    const result = { applied, reset, resetTemporal };
    if (render) {
      presentAppliedState(result, {
        mediaTime: targetMs / 1000,
        reason: options.reason || (reset ? "replay seek" : "replay state applied"),
      });
    }
    return result;
  }

  function playbackTick(_now, metadata = null) {
    state.videoFrameId = null;
    state.rafId = null;
    if (!state.active || !videoEl) return;
    const mediaTime = Number(metadata?.mediaTime);
    const result = applyAt(
      (Number.isFinite(mediaTime) ? mediaTime : Number(videoEl.currentTime || 0)) * 1000,
      { render: false },
    );
    // Every decoded video frame is a presentation event, even when no cereal
    // service changed at exactly that instant. AR follows this clock; gating it
    // on rlog mutations freezes the marker between sparse records and can leave
    // a nominal 60-second diagnostic with only a short prefix.
    const stateChanged = Boolean(result.applied || result.reset || result.resetTemporal);
    presentAppliedState(result, {
      presentFrame: true,
      overlayDirty: stateChanged,
      hudDirty: stateChanged,
      mediaTime: Number.isFinite(mediaTime) ? mediaTime : Number(videoEl.currentTime || 0),
      reason: "replay video frame",
    });
    syncTransport();
    schedulePlaybackTick();
  }

  function schedulePlaybackTick() {
    if (!state.active || !videoEl || videoEl.paused || videoEl.ended) return;
    if (state.videoFrameId != null || state.rafId != null) return;
    if (typeof videoEl.requestVideoFrameCallback === "function") {
      state.videoFrameId = videoEl.requestVideoFrameCallback(playbackTick);
    } else {
      state.rafId = window.requestAnimationFrame(playbackTick);
    }
  }

  function clearVideoSource() {
    if (!videoEl) return;
    try { videoEl.pause(); } catch {}
    videoEl.srcObject = null;
    videoEl.removeAttribute("src");
    try { videoEl.load(); } catch {}
  }

  function monotonicNowMs() {
    const now = Number(window.performance?.now?.());
    return Number.isFinite(now) ? now : Date.now();
  }

  function beginScrub() {
    if (!state.active) return;
    state.scrubbing = true;
    state.seekHoldUntilMs = Number.POSITIVE_INFINITY;
  }

  function endScrub() {
    if (!state.active) return;
    state.scrubbing = false;
    state.seekHoldUntilMs = monotonicNowMs() + SEEK_FRAME_HOLD_MS;
    window.requestCarrotVisionRender?.({ reason: "replay seek complete" });
  }

  function resetStageScrub() {
    if (stageScrub.frameId != null) window.cancelAnimationFrame(stageScrub.frameId);
    stageScrub.pointerId = null;
    stageScrub.startX = 0;
    stageScrub.startY = 0;
    stageScrub.startTime = 0;
    stageScrub.pendingTime = null;
    stageScrub.frameId = null;
    stageScrub.active = false;
    stageScrub.cancelled = false;
    stageScrub.resumeAfter = false;
    stageEl?.classList.remove("is-replay-scrubbing");
  }

  function applyStageScrubPosition() {
    stageScrub.frameId = null;
    const targetTime = Number(stageScrub.pendingTime);
    stageScrub.pendingTime = null;
    if (!state.active || !videoEl || !Number.isFinite(targetTime)) return;
    videoEl.currentTime = targetTime;
    applyAt(targetTime * 1000, { reset: true });
    syncTransport();
  }

  function queueStageScrubPosition(targetTime) {
    stageScrub.pendingTime = targetTime;
    if (stageScrub.frameId != null) return;
    stageScrub.frameId = window.requestAnimationFrame(applyStageScrubPosition);
  }

  function isStageScrubTarget(target) {
    if (!(target instanceof Element)) return false;
    return !target.closest(
      "button, input, select, a, .carrot-replay-controls__header, .carrot-replay-controls__hudToggle, "
      + ".carrot-replay-controls__transport, .carrot-replay-controls__segmentNav, "
      + ".carrot-stage__controls, .carrot-replay-insights",
    );
  }

  function handleStageScrubStart(event) {
    if (!state.active || state.loading || !state.sourceKey || !videoEl) return;
    if (event.isPrimary === false || (event.pointerType === "mouse" && event.button !== 0)) return;
    if (!isStageScrubTarget(event.target) || replayDuration() <= 0) return;
    resetStageScrub();
    stageScrub.pointerId = event.pointerId;
    stageScrub.startX = event.clientX;
    stageScrub.startY = event.clientY;
    stageScrub.startTime = Number(videoEl.currentTime || 0);
    try { stageEl?.setPointerCapture?.(event.pointerId); } catch {}
  }

  function handleStageScrubMove(event) {
    if (event.pointerId !== stageScrub.pointerId) {
      if (event.pointerType === "mouse") revealReplayControls();
      return;
    }
    if (stageScrub.cancelled) return;
    const deltaX = event.clientX - stageScrub.startX;
    const deltaY = event.clientY - stageScrub.startY;
    if (!stageScrub.active) {
      if (Math.abs(deltaY) > STAGE_SCRUB_THRESHOLD_PX && Math.abs(deltaY) >= Math.abs(deltaX)) {
        stageScrub.cancelled = true;
        return;
      }
      if (Math.abs(deltaX) < STAGE_SCRUB_THRESHOLD_PX || Math.abs(deltaX) <= Math.abs(deltaY)) return;
      stageScrub.active = true;
      stageScrub.resumeAfter = !videoEl.paused && !videoEl.ended;
      if (stageScrub.resumeAfter) videoEl.pause();
      beginScrub();
      stageEl?.classList.add("is-replay-scrubbing");
      revealReplayControls();
    }
    event.preventDefault();
    const width = Math.max(1, Number(stageEl?.clientWidth || 0));
    const duration = replayDuration();
    const scrubSpan = Math.min(duration, STAGE_SCRUB_SPAN_SECONDS);
    const targetTime = Math.max(0, Math.min(duration, stageScrub.startTime + (deltaX / width) * scrubSpan));
    queueStageScrubPosition(targetTime);
  }

  function finishStageScrub(event) {
    if (event?.pointerId !== stageScrub.pointerId) return;
    try {
      if (stageEl?.hasPointerCapture?.(event.pointerId)) stageEl.releasePointerCapture(event.pointerId);
    } catch {}
    if (stageScrub.frameId != null) {
      window.cancelAnimationFrame(stageScrub.frameId);
      stageScrub.frameId = null;
      applyStageScrubPosition();
    }
    const wasActive = stageScrub.active;
    const resumeAfter = stageScrub.resumeAfter;
    if (wasActive) {
      endScrub();
      suppressStageClickUntilMs = monotonicNowMs() + 420;
    }
    resetStageScrub();
    if (wasActive && resumeAfter && state.active && videoEl?.paused && !videoEl.ended) {
      videoEl.play().catch(() => syncTransport());
    }
  }

  function shouldHoldFrameDuringSeek() {
    return Boolean(
      state.active
      && state.hasRenderableFrame
      && (state.scrubbing || videoEl?.seeking || monotonicNowMs() < state.seekHoldUntilMs)
    );
  }

  function reportRenderable(renderable) {
    if (!state.active) return;
    const ready = Boolean(renderable);
    if (ready) state.hasRenderableFrame = true;
    if (state.ready === ready) return;
    state.ready = ready;
    if (ready) {
      setVisionPhase(window.CarrotVisionPhase?.READY || "ready");
    } else {
      setVisionPhase(
        window.CarrotVisionPhase?.FIRST_FRAME_WAITING || "first-frame-waiting",
        text("replay_waiting_video", "Waiting for the saved road video..."),
      );
    }
    syncTransport();
  }

  async function stop(options = {}) {
    const suppliedRequestToken = Number(options.requestToken);
    const hasSuppliedRequestToken = Number.isInteger(suppliedRequestToken) && suppliedRequestToken > 0;
    if (hasSuppliedRequestToken && suppliedRequestToken !== state.requestToken) return false;
    const requestToken = hasSuppliedRequestToken ? suppliedRequestToken : ++state.requestToken;
    const wasActive = state.active;
    const previousVisionActive = state.previousVisionActive;
    const preserveVisionSession = options.preserveVisionSession === true;
    let restorePreviousVision = previousVisionActive;
    state.token += 1;
    state.active = false;
    state.ready = false;
    state.loading = false;
    state.timelineReady = false;
    state.scrubbing = false;
    state.seekHoldUntilMs = 0;
    state.hasRenderableFrame = false;
    resetStageScrub();
    state.abortController?.abort();
    state.abortController = null;
    state.clientSession?.dispose?.();
    state.clientSession = null;
    cancelPlaybackTick();
    setReplayUiVisible(false);
    insights?.reset?.();

    if (options.returnToLogs !== false && typeof showPage === "function") {
      showPage("logs", true);
    }
    clearVideoSource();
    window.CarrotVisionRaw?.clearState?.({ reason: "replay stopped" });

    state.manifest = null;
    state.route = "";
    state.segment = "";
    state.segments = [];
    state.title = "";
    state.titleForSegment = null;
    state.records = [];
    state.nextRecord = 0;
    state.lastTimeMs = -1;
    state.sourceKey = "";
    state.previewSource = null;
    state.previewsRequested = false;
    if (titleEl) titleEl.textContent = "";
    syncSegmentNavigation();
    transport?.reset();

    if (preserveVisionSession) return true;

    if (wasActive && typeof window.CarrotVisionSyncAvailability === "function") {
      const liveAvailable = await window.CarrotVisionSyncAvailability().catch(() => Boolean(window.CarrotVisionState?.available));
      restorePreviousVision = previousVisionActive && liveAvailable;
    }
    if (requestToken !== state.requestToken) return false;
    if (wasActive && typeof window.CarrotVisionSetActive === "function") {
      window.CarrotVisionSetActive(restorePreviousVision, {
        phase: restorePreviousVision
          ? (window.CarrotVisionPhase?.STARTING || "starting")
          : (window.CarrotVisionState?.available
            ? (window.CarrotVisionPhase?.INACTIVE || "inactive")
            : (window.CarrotVisionPhase?.UNAVAILABLE || "unavailable")),
        reason: options.reason || "recorded replay stopped",
        updateRtcStatus: false,
      });
    }
    window.syncCarrotVisionStartOverlay?.();
    window.syncCarrotRealtimeLifecycle?.(true);
    return true;
  }

  async function start(options = {}) {
    const segment = String(options.segment || "").trim();
    const route = String(options.route || "").trim();
    const segments = normalizeSegments(options.segments, segment);
    const titleForSegment = typeof options.titleForSegment === "function" ? options.titleForSegment : null;
    const requestedTitle = String(options.title || "").trim();
    if (!segment) throw new Error(text("replay_segment_missing", "A drive segment was not selected."));
    if (!stageEl || !videoEl || !controlsEl || !transport) {
      throw new Error(text("replay_unavailable", "Drive replay is not available in this browser."));
    }
    const requestToken = ++state.requestToken;
    const previousVisionActive = state.active
      ? state.previousVisionActive
      : Boolean(window.isCarrotVisionActive?.());
    if (state.active) {
      await stop({
        returnToLogs: false,
        reason: "replace recorded replay",
        preserveVisionSession: true,
        requestToken,
      });
    }
    if (requestToken !== state.requestToken) return false;

    const token = ++state.token;
    state.active = true;
    state.ready = false;
    state.loading = true;
    state.timelineReady = false;
    state.scrubbing = false;
    state.seekHoldUntilMs = 0;
    state.hasRenderableFrame = false;
    state.previousVisionActive = previousVisionActive;
    state.route = route;
    state.segment = segment;
    state.segments = segments;
    state.title = requestedTitle || segment;
    state.titleForSegment = titleForSegment;
    state.manifest = null;
    state.records = [];
    state.nextRecord = 0;
    state.lastTimeMs = -1;
    state.sourceKey = "";
    state.previewSource = null;
    state.previewsRequested = false;
    state.abortController = new AbortController();
    const workspaceManaged = setReplayUiVisible(true);
    if (!workspaceManaged) {
      window.DriveContentVision?.activate?.({ reason: "recorded replay" });
    }
    syncTransport();

    window.CarrotVisionSetActive?.(true, {
      phase: window.CarrotVisionPhase?.STARTING || "starting",
      reason: "recorded-replay",
      statusText: text("replay_loading", "Preparing drive replay..."),
      detailText: text("replay_loading_detail", "Matching the saved road video with driving information."),
      updateRtcStatus: false,
    });
    if (typeof showPage === "function") showPage("carrot", true);
    window.syncCarrotVisionStartOverlay?.();
    window.syncCarrotRealtimeLifecycle?.(true);

    try {
      const clientProfile = window.CarrotReplayClient?.capabilities?.() || { client: false, mode: "unsupported" };
      if (!clientProfile.client) {
        throw new Error(text(
          "replay_client_processing_required",
          "This browser is too old to process replay data on this device.",
        ));
      }
      const clientMode = true;
      const sourcePromise = window.CarrotReplayClient.fetchSource(segment, state.abortController.signal);
      const startup = await Promise.all([
        window.CarrotVisionRtc?.disconnect?.({ keepVideo: false }),
        window.CarrotVisionRaw?.disconnectOverlay?.(),
        window.CarrotVisionRaw?.disconnectHud?.(),
        sourcePromise,
      ]);
      if (!state.active || token !== state.token) return;
      window.CarrotVisionRaw?.clearState?.({ render: false, reason: "replay starting" });

      const clientSource = startup[3];
      if (clientMode) {
        const clientRun = window.CarrotReplayClient.start(
          clientSource,
          videoEl,
          state.abortController.signal,
        );
        const videoCompletion = clientRun.video.complete;
        videoCompletion.catch(() => {});
        state.clientSession = clientRun.session;
        const [videoUrl, timeline] = await Promise.all([
          clientRun.video.urlPromise,
          clientRun.timelinePromise,
        ]);
        if (!state.active || token !== state.token) return;
        state.manifest = {
          ...clientSource,
          clientMode: true,
          clientProfile: clientRun.profile.mode,
          videoUrl,
          ...(timeline.metadata || {}),
          durationMs: timeline.durationMs,
        };
        state.sourceKey = String(videoUrl || "");
        if (!state.sourceKey) throw new Error(text("replay_video_missing", "The saved road video was not found."));
        transport.setFallbackThumbnail("");
        state.records = timeline.records;
        state.nextRecord = 0;
        state.lastTimeMs = -1;
        clientRun.video.createPreviewSource?.(state.manifest.durationMs / 1000).then((previewSource) => {
          if (!state.active || token !== state.token || !previewSource?.url) return;
          state.previewSource = previewSource;
          maybeGenerateFilmstrip();
        }).catch(() => {});
        await loadReplayInsights({
          manifest: state.manifest,
          records: state.records,
          durationMs: state.manifest.durationMs,
          routeId: state.route || state.segment || null,
          decodeRecord,
          onSeek: seekReplayTo,
          onPause: pauseReplayAtCurrentTime,
          onUpdate: syncSegmentNavigation,
        }, state.abortController.signal);
        if (!state.active || token !== state.token) return;
        state.timelineReady = true;
        videoCompletion.catch((error) => {
          if (!state.active || token !== state.token || error?.name === "AbortError") return;
          setVisionPhase(
            window.CarrotVisionPhase?.FAILED || "failed",
            text("replay_video_failed", "The saved road video could not be played."),
            String(error?.message || error || text("replay_video_failed_detail", "Close replay and check the saved video.")),
          );
        });
        state.loading = false;
        stageEl?.classList.remove("is-replay-loading");
        videoEl.srcObject = null;
        transport.setRate(1);
        videoEl.src = state.sourceKey;
        videoEl.load();
        applyAt(0, { reset: true });
        setVisionPhase(
          window.CarrotVisionPhase?.FIRST_FRAME_WAITING || "first-frame-waiting",
          text("replay_waiting_video", "Waiting for the saved road video..."),
        );
        window.requestCarrotVisionRender?.({ reason: "client replay timeline ready" });
        syncTransport();
        videoEl.play().catch(() => syncTransport());
        return true;
      }

      throw new Error(text("replay_client_processing_required", "This browser cannot process replay data on this device."));
    } catch (error) {
      if (error?.name === "AbortError" || token !== state.token) return;
      const message = String(error?.message || error || text("replay_failed", "Drive replay could not be started."));
      await stop({ returnToLogs: true, reason: "recorded replay failed", requestToken });
      if (typeof showAppToast === "function") {
        showAppToast(`${text("replay_failed", "Drive replay could not be started.")} ${message}`, {
          tone: "error",
          duration: 5200,
        });
      }
      return false;
    }
  }

  function handleVideoReady() {
    if (!state.active) return;
    transport?.captureFallbackFrame?.();
    maybeGenerateFilmstrip();
    applyAt(Number(videoEl.currentTime || 0) * 1000);
    window.requestCarrotVisionRender?.({ reason: "replay video ready" });
    schedulePlaybackTick();
    syncTransport();
  }

  function handlePlaybackStateChange(event) {
    if (!state.active) return;
    if (event?.type === "seeked" && !state.scrubbing) {
      state.seekHoldUntilMs = monotonicNowMs() + SEEK_FRAME_HOLD_MS;
    }
    applyAt(Number(videoEl.currentTime || 0) * 1000, {
      resetTemporal: event?.type === "seeking",
    });
    if (state.ready) setVisionPhase(window.CarrotVisionPhase?.READY || "ready");
    schedulePlaybackTick();
    syncTransport();
    if (videoEl.ended) {
      if (controlsHideTimer != null) window.clearTimeout(controlsHideTimer);
      controlsHideTimer = null;
      stageEl?.classList.remove("is-replay-controls-hidden");
    } else if (event?.type !== "timeupdate") {
      revealReplayControls();
    }
  }

  transport = window.CarrotMediaTransport?.create?.({
    media: videoEl,
    root: controlsEl,
    closeButton,
    playButton,
    speedButton,
    seek: seekEl,
    currentTime: currentTimeEl,
    duration: durationEl,
    status: statusEl,
    filmstrip: filmstripEl,
    fallbackThumbnail: thumbnailEl,
    scrubPreview: scrubPreviewEl,
    scrubImage: scrubImageEl,
    scrubTime: scrubTimeEl,
    scrubEventInfo: scrubEventInfoEl,
    scrubEventIcon: scrubEventIconEl,
    scrubEventTitle: scrubEventTitleEl,
    scrubEventMeta: scrubEventMetaEl,
    scrubEvents: scrubEventsEl,
    scrubEventsCanvas: scrubEventsCanvasEl,
    speeds: REPLAY_SPEEDS,
    text,
    getDuration: replayDuration,
    getStatus: replayStatusText,
    canInteract: () => state.active && !state.loading && Boolean(state.sourceKey),
    onInteract: revealReplayControls,
    onClose: () => stop({ returnToLogs: true, reason: "replay close" }),
    onSeek: () => {
      if (!state.active) return;
      applyAt(Number(videoEl.currentTime || 0) * 1000, { reset: true });
    },
    onScrubStart: beginScrub,
    onScrubEnd: endScrub,
    onTimelineCommit: (seconds, pointer) => insights?.selectTimelineAt?.(seconds, pointer),
    getTimelineEvents: (seconds, windowSeconds) => insights?.previewEventsAt?.(seconds, windowSeconds),
    getTimelineEvent: (seconds, toleranceSeconds) => insights?.previewEventAt?.(seconds, toleranceSeconds),
    onPreviewVisibilityChange: (visible) => insights?.setPreviewVisible?.(visible),
    onTimelineEventSelect: (eventId) => insights?.selectEventById?.(eventId),
    onTimelinePreviewSeek: (seconds) => seekReplayTo(seconds),
  });

  ["loadedmetadata", "loadeddata", "canplay"].forEach((eventName) => {
    videoEl?.addEventListener(eventName, handleVideoReady);
  });
  ["play", "pause", "ended", "ratechange", "timeupdate", "seeking", "seeked"].forEach((eventName) => {
    videoEl?.addEventListener(eventName, handlePlaybackStateChange);
  });
  videoEl?.addEventListener("error", () => {
    if (!state.active || state.loading) return;
    state.ready = false;
    setVisionPhase(
      window.CarrotVisionPhase?.FAILED || "failed",
      text("replay_video_failed", "The saved road video could not be played."),
      text("replay_video_failed_detail", "Close replay and check that this segment has qcamera video."),
    );
    syncTransport();
  });
  thumbnailEl?.addEventListener("load", () => thumbnailEl.classList.add("is-ready"));
  thumbnailEl?.addEventListener("error", () => thumbnailEl.classList.remove("is-ready"));
  previousSegmentButton?.addEventListener("click", () => navigateSegment(-1));
  nextSegmentButton?.addEventListener("click", () => navigateSegment(1));
  segmentSelectEl?.addEventListener("change", () => {
    const targetSegment = segmentSelectEl.value;
    navigateToSegment(targetSegment).catch(() => syncSegmentNavigation());
  });
  hudToggleEl?.addEventListener("change", () => {
    setReplayHudVisible(hudToggleEl.checked, { persist: true });
    revealReplayControls();
  });
  stageEl?.addEventListener("pointerdown", handleStageScrubStart, { passive: true });
  stageEl?.addEventListener("pointerenter", (event) => {
    if (event.pointerType === "mouse") revealReplayControls();
  }, { passive: true });
  stageEl?.addEventListener("pointermove", handleStageScrubMove, { passive: false });
  stageEl?.addEventListener("pointerup", finishStageScrub, { passive: true });
  stageEl?.addEventListener("pointercancel", finishStageScrub, { passive: true });
  stageEl?.addEventListener("lostpointercapture", finishStageScrub, { passive: true });
  document.addEventListener("keydown", handleReplayKeydown);
  stageEl?.addEventListener("click", (event) => {
    if (!state.active || !(event.target instanceof Element)) return;
    if (monotonicNowMs() < suppressStageClickUntilMs) {
      event.stopImmediatePropagation();
      return;
    }
    if (event.target.closest("button, input, select, .carrot-replay-controls__hudToggle, .carrot-replay-controls__transport, .carrot-stage__controls, .carrot-replay-insights")) {
      return;
    }
    event.stopImmediatePropagation();
    toggleReplayControls();
  }, true);
  window.addEventListener("carrot:replayinsightschange", revealReplayControls);
  window.addEventListener("carrot:pagechange", (event) => {
    if (!state.active || event?.detail?.page === "carrot") return;
    videoEl?.pause?.();
  });
  window.addEventListener("carrot:languagechange", syncReplayLabels);
  window.addEventListener("carrot:websettingschange", syncReplayHudFromServer);

  setReplayHudVisible(state.hudVisible);
  syncReplayLabels();
  return {
    isActive: () => state.active,
    isReady: () => state.active && state.ready,
    status: () => Object.freeze({
      active: state.active,
      ready: state.ready,
      loading: state.loading,
      route: state.route || null,
      segment: state.segment || null,
      currentTime: Number(videoEl?.currentTime || 0),
      duration: replayDuration(),
      paused: videoEl?.paused !== false,
      ended: videoEl?.ended === true,
      playbackRate: Number(videoEl?.playbackRate || 1),
      timelineReady: state.timelineReady,
      services: Object.freeze({ ...(state.manifest?.services || {}) }),
    }),
    diagnosticPlayback,
    reportRenderable,
    shouldHoldFrameDuringSeek,
    start,
    stop,
    updateSegments,
    videoSourceKey: () => state.active ? state.sourceKey : "",
  };
})();
