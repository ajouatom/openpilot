import { DriveVisionHudModel } from "./hud_model.js";

export const RTC_PERF_HOLD_MS = 600;
export const RTC_PERF_HOLD_MOVE_PX = 12;
export const RTC_PERF_SUMMARY_AUTO_CLOSE_MS = 8000;
export const RTC_PERF_LOG_RESULT_RESET_MS = 1400;
export const RTC_PERF_WIDE_LAYOUT_QUERY = "(min-aspect-ratio: 13/10), (horizontal-viewport-segments: 2), (vertical-viewport-segments: 2), (min-width: 640px) and (min-height: 650px)";

export function createRtcPerfHud(options = {}) {
  const model = options.model || DriveVisionHudModel;
  if (!model?.buildRtcPerf) return null;

  const runtimeWindow = options.window || globalThis.window || globalThis;
  const documentRoot = options.document || runtimeWindow.document || globalThis.document;
  if (!documentRoot) return null;

  const root = options.root || documentRoot;
  const query = (id) => root.querySelector?.(`#${id}`) || documentRoot.getElementById(id);
  const hudEl = query("carrotRtcPerfHud");
  const glanceEl = query("carrotRtcPerfGlance");
  const glanceTextEl = query("carrotRtcPerfGlanceText");
  const summaryEl = query("carrotRtcPerfSummary");
  const titleEl = query("carrotRtcPerfTitle");
  const videoEl = query("carrotRtcPerfVideo");
  const codecEl = query("carrotRtcPerfCodec");
  const bitrateEl = query("carrotRtcPerfBitrate");
  const rttEl = query("carrotRtcPerfRtt");
  const jitterEl = query("carrotRtcPerfJitter");
  const lossEl = query("carrotRtcPerfLoss");
  const freezeEl = query("carrotRtcPerfFreeze");
  const pathEl = query("carrotRtcPerfPath");
  const holdTargetEl = query("carrotRtcPerfHoldTarget");
  const closeButtonEl = query("carrotRtcPerfCloseBtn");
  const logButtonEl = query("carrotRtcPerfLogBtn");
  const isActive = typeof options.isActive === "function" ? options.isActive : () => true;
  const isReplayActive = typeof options.isReplayActive === "function" ? options.isReplayActive : () => false;
  const showToast = typeof options.showToast === "function" ? options.showToast : () => {};

  let summaryOpen = false;
  let summaryCloseTimer = null;
  let holdTimer = null;
  let holdPointerId = null;
  let holdStartX = 0;
  let holdStartY = 0;
  let logResetTimer = null;

  function getPerfModel() {
    return model.buildRtcPerf(runtimeWindow.CarrotRtcPerf);
  }

  function isSummaryAvailable() {
    return Boolean(runtimeWindow.CarrotLayout?.isWide?.() ?? runtimeWindow.matchMedia(RTC_PERF_WIDE_LAYOUT_QUERY).matches);
  }

  function clearSummaryCloseTimer() {
    if (summaryCloseTimer == null) return;
    runtimeWindow.clearTimeout(summaryCloseTimer);
    summaryCloseTimer = null;
  }

  function syncSummaryAutoClose(tone) {
    clearSummaryCloseTimer();
    if (!summaryOpen || tone !== "normal") return;
    summaryCloseTimer = runtimeWindow.setTimeout(() => {
      setOpen(false);
    }, RTC_PERF_SUMMARY_AUTO_CLOSE_MS);
  }

  function setOpen(open) {
    const perfModel = getPerfModel();
    const nextOpen = Boolean(open && perfModel.visible && isSummaryAvailable());
    summaryOpen = nextOpen;
    if (summaryEl) summaryEl.hidden = !nextOpen;
    if (!nextOpen) {
      clearSummaryCloseTimer();
      return;
    }
    syncSummaryAutoClose(perfModel.tone);
  }

  function close() {
    setOpen(false);
  }

  function sync() {
    if (!hudEl || !glanceEl || !glanceTextEl) return;
    if (isReplayActive()) {
      hudEl.hidden = true;
      close();
      return;
    }

    const perfModel = getPerfModel();
    hudEl.hidden = !perfModel.visible;
    glanceEl.dataset.tone = perfModel.tone;
    glanceTextEl.textContent = perfModel.glance || perfModel.title;

    if (!perfModel.visible || !isSummaryAvailable()) {
      close();
      return;
    }

    if (summaryEl) summaryEl.dataset.tone = perfModel.tone;
    if (titleEl) titleEl.textContent = perfModel.title;
    if (videoEl) videoEl.textContent = perfModel.video;
    if (codecEl) codecEl.textContent = perfModel.codec;
    if (bitrateEl) bitrateEl.textContent = perfModel.bitrate;
    if (rttEl) rttEl.textContent = perfModel.rtt;
    if (jitterEl) jitterEl.textContent = perfModel.jitter;
    if (lossEl) lossEl.textContent = perfModel.loss;
    if (freezeEl) freezeEl.textContent = perfModel.freeze;
    if (pathEl) pathEl.textContent = perfModel.path;
    if (summaryOpen && summaryCloseTimer == null) {
      syncSummaryAutoClose(perfModel.tone);
    } else if (summaryOpen && perfModel.tone !== "normal") {
      clearSummaryCloseTimer();
    }
  }

  function layoutChanged() {
    if (!isSummaryAvailable()) close();
  }

  function clearHold() {
    if (holdTimer != null) {
      runtimeWindow.clearTimeout(holdTimer);
      holdTimer = null;
    }
    holdPointerId = null;
    holdTargetEl?.classList.remove("is-holding");
  }

  function handleHoldPointerDown(event) {
    event.stopPropagation();
    clearHold();
    if (!isActive() || !isSummaryAvailable()) return;
    holdPointerId = event.pointerId;
    holdStartX = event.clientX;
    holdStartY = event.clientY;
    holdTargetEl.classList.add("is-holding");
    holdTargetEl.setPointerCapture?.(event.pointerId);
    holdTimer = runtimeWindow.setTimeout(() => {
      holdTimer = null;
      holdTargetEl.classList.remove("is-holding");
      setOpen(!summaryOpen);
    }, RTC_PERF_HOLD_MS);
  }

  function handleHoldPointerMove(event) {
    if (holdPointerId !== event.pointerId) return;
    if (Math.hypot(event.clientX - holdStartX, event.clientY - holdStartY) > RTC_PERF_HOLD_MOVE_PX) {
      clearHold();
    }
  }

  function handleHoldClick(event) {
    event.preventDefault();
    event.stopPropagation();
  }

  function handleCloseClick(event) {
    event.stopPropagation();
    close();
  }

  async function handleLogClick(event) {
    event.stopPropagation();
    const previousText = logButtonEl.textContent;
    logButtonEl.disabled = true;
    logButtonEl.textContent = "SEND";
    try {
      if (!runtimeWindow.CarrotVisionDiag?.uploadDiscord) throw new Error("diagnostic upload unavailable");
      await runtimeWindow.CarrotVisionDiag.uploadDiscord();
      logButtonEl.textContent = "SENT";
      showToast("Carrot Vision log sent to Discord", { duration: 2600 });
    } catch (error) {
      logButtonEl.textContent = "FAIL";
      showToast(`Discord upload failed: ${error?.message || error}`, { tone: "error", duration: 4200 });
    } finally {
      if (logResetTimer != null) runtimeWindow.clearTimeout(logResetTimer);
      logResetTimer = runtimeWindow.setTimeout(() => {
        logResetTimer = null;
        logButtonEl.disabled = false;
        logButtonEl.textContent = previousText;
      }, RTC_PERF_LOG_RESULT_RESET_MS);
    }
  }

  function handleDocumentPointerDown(event) {
    if (!summaryOpen) return;
    if (summaryEl?.contains(event.target) || holdTargetEl?.contains(event.target)) return;
    close();
  }

  function bind() {
    if (!holdTargetEl || !summaryEl) return;
    holdTargetEl.addEventListener("pointerdown", handleHoldPointerDown);
    holdTargetEl.addEventListener("pointermove", handleHoldPointerMove);
    ["pointerup", "pointercancel", "lostpointercapture"].forEach((eventName) => {
      holdTargetEl.addEventListener(eventName, clearHold);
    });
    holdTargetEl.addEventListener("click", handleHoldClick);
    closeButtonEl?.addEventListener("click", handleCloseClick);
    logButtonEl?.addEventListener("click", handleLogClick);
    documentRoot.addEventListener("pointerdown", handleDocumentPointerDown, true);
  }

  function destroy() {
    close();
    clearHold();
    if (logResetTimer != null) {
      runtimeWindow.clearTimeout(logResetTimer);
      logResetTimer = null;
    }
    if (!holdTargetEl || !summaryEl) return;
    holdTargetEl.removeEventListener("pointerdown", handleHoldPointerDown);
    holdTargetEl.removeEventListener("pointermove", handleHoldPointerMove);
    ["pointerup", "pointercancel", "lostpointercapture"].forEach((eventName) => {
      holdTargetEl.removeEventListener(eventName, clearHold);
    });
    holdTargetEl.removeEventListener("click", handleHoldClick);
    closeButtonEl?.removeEventListener("click", handleCloseClick);
    logButtonEl?.removeEventListener("click", handleLogClick);
    documentRoot.removeEventListener("pointerdown", handleDocumentPointerDown, true);
  }

  function status() {
    return Object.freeze({
      available: isSummaryAvailable(),
      visible: Boolean(hudEl && !hudEl.hidden),
      summaryOpen,
    });
  }

  bind();
  return Object.freeze({ sync, close, layoutChanged, destroy, status });
}

export const DriveVisionRtcPerfHud = Object.freeze({
  create: createRtcPerfHud,
});

export function installDriveVisionRtcPerfHudFacade(target = globalThis) {
  target.DriveVisionRtcPerfHud = DriveVisionRtcPerfHud;
  return DriveVisionRtcPerfHud;
}
