export const COMPOSITOR_DPR_LIMIT = 2;

export function createCompositor(options = {}) {
  const target = options.target || globalThis;
  const pane = options.pane;
  const canvas = options.canvas;
  const video = options.video;
  const overlayCanvas = options.overlayCanvas;
  const overlayStore = options.overlayStore;
  const context = canvas?.getContext?.("2d", { alpha: false }) || null;
  const createStagingCanvas = options.createStagingCanvas || (() => {
    const element = target.document?.createElement?.("canvas");
    if (element) {
      element.width = 1;
      element.height = 1;
      return element;
    }
    if (typeof target.OffscreenCanvas === "function") return new target.OffscreenCanvas(1, 1);
    return null;
  });
  const stagingCanvas = createStagingCanvas();
  const stagingContext = stagingCanvas?.getContext?.("2d", { alpha: false }) || null;
  if (!pane || !canvas || !video || !overlayCanvas || !overlayStore || !context || !stagingCanvas || !stagingContext) return null;

  let active = false;
  let hasFrame = false;
  let lastFrameAt = 0;

  function render() {
    if (!active || video.readyState < 2 || video.videoWidth < 1 || video.videoHeight < 1) return false;
    const rect = pane.getBoundingClientRect();
    const dpr = Math.min(COMPOSITOR_DPR_LIMIT, Math.max(1, target.devicePixelRatio || 1));
    const displayWidth = rect.width > 0 ? rect.width : video.videoWidth;
    const displayHeight = rect.height > 0 ? rect.height : video.videoHeight;
    const targetWidth = Math.max(1, Math.round(displayWidth * dpr));
    const targetHeight = Math.max(1, Math.round(displayHeight * dpr));
    if (stagingCanvas.width !== targetWidth) stagingCanvas.width = targetWidth;
    if (stagingCanvas.height !== targetHeight) stagingCanvas.height = targetHeight;
    const scale = Math.max(targetWidth / video.videoWidth, targetHeight / video.videoHeight);
    const sourceWidth = targetWidth / scale;
    const sourceHeight = targetHeight / scale;

    try {
      overlayStore.flush();
      stagingContext.drawImage(
        video,
        Math.max(0, (video.videoWidth - sourceWidth) / 2),
        Math.max(0, (video.videoHeight - sourceHeight) / 2),
        sourceWidth,
        sourceHeight,
        0,
        0,
        targetWidth,
        targetHeight,
      );
      if (overlayCanvas.width > 1 && overlayCanvas.height > 1) {
        stagingContext.drawImage(
          overlayCanvas,
          0,
          0,
          overlayCanvas.width,
          overlayCanvas.height,
          0,
          0,
          targetWidth,
          targetHeight,
        );
      }
      if (canvas.width !== targetWidth) canvas.width = targetWidth;
      if (canvas.height !== targetHeight) canvas.height = targetHeight;
      context.drawImage(stagingCanvas, 0, 0, targetWidth, targetHeight);
    } catch (_) {
      return false;
    }

    hasFrame = true;
    lastFrameAt = target.performance.now();
    options.onFrame?.(lastFrameAt);
    return true;
  }

  function setActive(value) {
    active = Boolean(value);
    video.hidden = false;
    canvas.hidden = false;
  }

  function invalidate(optionsValue = {}) {
    lastFrameAt = 0;
    if (!optionsValue.preserveFrame) hasFrame = false;
  }

  function fresh(maxAgeMs, now = target.performance.now()) {
    return lastFrameAt > 0 && now - lastFrameAt <= Math.max(0, Number(maxAgeMs) || 0);
  }

  function snapshot() {
    return { active, hasFrame, lastFrameAt, canvasVisible: !canvas.hidden };
  }

  for (const eventName of ["loadeddata", "playing", "timeupdate"]) video.addEventListener(eventName, render);

  return Object.freeze({ render, setActive, invalidate, fresh, snapshot });
}

export const CarrotNaviCompositor = Object.freeze({ create: createCompositor });

export function installCarrotNaviCompositorGlobal(target = globalThis) {
  target.CarrotNaviCompositor = CarrotNaviCompositor;
  return CarrotNaviCompositor;
}
