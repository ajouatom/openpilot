export function createReplayRenderController(options = {}) {
  const isReplayActive = options.isReplayActive;
  const isStageVisible = options.isStageVisible;
  const cancelScheduledRender = options.cancelScheduledRender;
  const resetFrameTemporalState = options.resetFrameTemporalState;
  const resetReplayState = options.resetReplayState;
  const mergePendingRenderState = options.mergePendingRenderState;
  const flushScheduledRender = options.flushScheduledRender;
  const notifyPresentedFrame = options.notifyPresentedFrame;

  if (
    typeof isReplayActive !== "function"
    || typeof isStageVisible !== "function"
    || typeof cancelScheduledRender !== "function"
    || typeof resetFrameTemporalState !== "function"
    || typeof resetReplayState !== "function"
    || typeof mergePendingRenderState !== "function"
    || typeof flushScheduledRender !== "function"
    || typeof notifyPresentedFrame !== "function"
  ) {
    return null;
  }

  function renderVideoFrame(renderOptions = {}) {
    if (!isReplayActive() || !isStageVisible()) return false;

    cancelScheduledRender();
    if (renderOptions.resetTemporal) resetFrameTemporalState();
    mergePendingRenderState({
      force: Boolean(renderOptions.force),
      overlayDirty: renderOptions.overlayDirty !== false,
      hudDirty: renderOptions.hudDirty !== false,
    });
    // Replay invokes this after applying the rlog samples for the presented
    // media frame, so render synchronously against that exact frame.
    flushScheduledRender();
    notifyPresentedFrame({
      source: "replay",
      mediaTime: Number.isFinite(Number(renderOptions.mediaTime))
        ? Number(renderOptions.mediaTime)
        : null,
      reason: String(renderOptions.reason || "replay video frame"),
    });
    return true;
  }

  function resetTemporalState() {
    resetReplayState();
  }

  return Object.freeze({
    renderVideoFrame,
    resetTemporalState,
  });
}

export const DriveVisionReplayRenderController = Object.freeze({
  create: createReplayRenderController,
});

export function installDriveVisionReplayRenderControllerFacade(target = globalThis) {
  target.DriveVisionReplayRenderController = DriveVisionReplayRenderController;
  return DriveVisionReplayRenderController;
}
