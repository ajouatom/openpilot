"use strict";

globalThis.DriveVisionReplayRenderController = (() => {
  function create(options = {}) {
    const isReplayActive = options.isReplayActive;
    const isStageVisible = options.isStageVisible;
    const cancelScheduledRender = options.cancelScheduledRender;
    const resetFrameTemporalState = options.resetFrameTemporalState;
    const resetReplayState = options.resetReplayState;
    const mergePendingRenderState = options.mergePendingRenderState;
    const flushScheduledRender = options.flushScheduledRender;

    if (
      typeof isReplayActive !== "function"
      || typeof isStageVisible !== "function"
      || typeof cancelScheduledRender !== "function"
      || typeof resetFrameTemporalState !== "function"
      || typeof resetReplayState !== "function"
      || typeof mergePendingRenderState !== "function"
      || typeof flushScheduledRender !== "function"
    ) {
      return null;
    }

    function renderVideoFrame(options = {}) {
      if (!isReplayActive() || !isStageVisible()) return false;

      cancelScheduledRender();
      if (options.resetTemporal) resetFrameTemporalState();
      mergePendingRenderState({
        force: Boolean(options.force),
        overlayDirty: options.overlayDirty !== false,
        hudDirty: options.hudDirty !== false,
      });
      // Replay invokes this after applying the rlog samples for the presented
      // media frame, so render synchronously against that exact frame.
      flushScheduledRender();
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

  return Object.freeze({ create });
})();
