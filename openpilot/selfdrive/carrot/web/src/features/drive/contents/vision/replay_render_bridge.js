export function createReplayRenderBridge(options = {}) {
  const stage = options.stage || null;
  const replayViewport = options.replayViewport || null;
  const video = options.video || null;
  const getReplay = typeof options.getReplay === "function"
    ? options.getReplay
    : () => globalThis.CarrotVisionReplay;

  if (!stage || !video) return null;

  function replay() {
    const value = getReplay();
    return value && typeof value === "object" ? value : null;
  }

  function isActive() {
    return Boolean(replay()?.isActive?.());
  }

  function isReady() {
    return Boolean(replay()?.isReady?.());
  }

  function reportRenderable(renderable) {
    if (!isActive()) return false;
    replay()?.reportRenderable?.(Boolean(renderable));
    return true;
  }

  function videoSourceKey() {
    if (!isActive()) return "";
    return String(replay()?.videoSourceKey?.() || "");
  }

  function shouldHoldFrameDuringSeek() {
    return Boolean(isActive() && replay()?.shouldHoldFrameDuringSeek?.());
  }

  function getRenderViewport() {
    return stage.classList.contains("is-replay") && replayViewport
      ? replayViewport
      : stage;
  }

  function temporalNowMs(fallbackNowMs) {
    if (isActive()) {
      const replaySeconds = Number(video.currentTime);
      if (Number.isFinite(replaySeconds) && replaySeconds >= 0) return replaySeconds * 1000;
    }
    const fallback = Number(fallbackNowMs);
    return Number.isFinite(fallback) ? fallback : 0;
  }

  return Object.freeze({
    isActive,
    isReady,
    reportRenderable,
    videoSourceKey,
    shouldHoldFrameDuringSeek,
    getRenderViewport,
    temporalNowMs,
  });
}

export const DriveVisionReplayRenderBridge = Object.freeze({
  create: createReplayRenderBridge,
});

export function installDriveVisionReplayRenderBridgeFacade(target = globalThis) {
  target.DriveVisionReplayRenderBridge = DriveVisionReplayRenderBridge;
  return DriveVisionReplayRenderBridge;
}
