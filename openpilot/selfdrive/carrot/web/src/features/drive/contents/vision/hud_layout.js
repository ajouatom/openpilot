function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function finiteNumber(value, fallback = 0) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : fallback;
}

function positiveNumber(value, fallback = 0) {
  const numeric = finiteNumber(value, fallback);
  return numeric > 0 ? numeric : fallback;
}

function freezeRect(left, top, right, bottom) {
  const width = Math.max(0, right - left);
  const height = Math.max(0, bottom - top);
  return Object.freeze({
    left,
    top,
    right,
    bottom,
    width,
    height,
    centerX: left + width / 2,
    centerY: top + height / 2,
  });
}

function fullRect(width, height) {
  return freezeRect(0, 0, Math.max(0, width), Math.max(0, height));
}

function clipRect(rect, width, height) {
  const boundsWidth = Math.max(0, width);
  const boundsHeight = Math.max(0, height);
  const rawLeft = finiteNumber(rect?.left ?? rect?.x, 0);
  const rawTop = finiteNumber(rect?.top ?? rect?.y, 0);
  const rawRight = finiteNumber(rect?.right, rawLeft + finiteNumber(rect?.width, boundsWidth));
  const rawBottom = finiteNumber(rect?.bottom, rawTop + finiteNumber(rect?.height, boundsHeight));
  const left = clamp(Math.min(rawLeft, rawRight), 0, boundsWidth);
  const right = clamp(Math.max(rawLeft, rawRight), 0, boundsWidth);
  const top = clamp(Math.min(rawTop, rawBottom), 0, boundsHeight);
  const bottom = clamp(Math.max(rawTop, rawBottom), 0, boundsHeight);
  return right - left >= 2 && bottom - top >= 2
    ? freezeRect(left, top, right, bottom)
    : fullRect(boundsWidth, boundsHeight);
}

function transformedContentRect(sourceWidth, sourceHeight, renderWidth, renderHeight, transform) {
  const width = positiveNumber(sourceWidth, renderWidth);
  const height = positiveNumber(sourceHeight, renderHeight);
  const scale = Math.max(positiveNumber(transform?.scale, 1), 0.01);
  const left = finiteNumber(transform?.tx, 0);
  const top = finiteNumber(transform?.ty, 0);
  return clipRect({
    left,
    top,
    right: left + width * scale,
    bottom: top + height * scale,
  }, renderWidth, renderHeight);
}

function relativeRenderOrigin(stage, renderViewport) {
  if (!renderViewport || renderViewport === stage) return Object.freeze({ left: 0, top: 0 });
  const stageRect = stage.getBoundingClientRect?.();
  const renderRect = renderViewport.getBoundingClientRect?.();
  if (stageRect && renderRect) {
    return Object.freeze({
      left: finiteNumber(renderRect.left, 0) - finiteNumber(stageRect.left, 0),
      top: finiteNumber(renderRect.top, 0) - finiteNumber(stageRect.top, 0),
    });
  }
  return Object.freeze({
    left: finiteNumber(renderViewport.offsetLeft, 0),
    top: finiteNumber(renderViewport.offsetTop, 0),
  });
}

function viewportSignature(viewport) {
  return [
    viewport.left,
    viewport.top,
    viewport.width,
    viewport.height,
    viewport.render.left,
    viewport.render.top,
    viewport.render.width,
    viewport.render.height,
    viewport.stageWidth,
    viewport.stageHeight,
  ].map((value) => Math.round(value * 100) / 100).join(",");
}

export function createDriveVisionViewport(options = {}) {
  const target = options.target || globalThis;
  const stage = options.stage;
  const card = options.card;
  if (!stage) return null;

  let lastHudLeft = "";
  let lastHudBottom = "";
  let lastViewportSignature = "";
  let currentViewport = null;

  function clearHudPosition() {
    if (card) {
      card.style.removeProperty("--carrot-hud-left");
      card.style.removeProperty("--carrot-hud-bottom");
    }
    lastHudLeft = "";
    lastHudBottom = "";
  }

  function clearViewportProperties() {
    stage.style.removeProperty("--carrot-viewport-left");
    stage.style.removeProperty("--carrot-viewport-top");
    stage.style.removeProperty("--carrot-viewport-width");
    stage.style.removeProperty("--carrot-viewport-height");
  }

  function reset() {
    clearHudPosition();
    clearViewportProperties();
    lastViewportSignature = "";
    currentViewport = null;
  }

  function resolve(input = {}) {
    const renderViewport = input.renderViewport || stage;
    const stageWidth = positiveNumber(input.stageWidth, positiveNumber(stage.clientWidth, 0));
    const stageHeight = positiveNumber(input.stageHeight, positiveNumber(stage.clientHeight, 0));
    const renderWidth = positiveNumber(input.renderWidth, positiveNumber(renderViewport?.clientWidth, stageWidth));
    const renderHeight = positiveNumber(input.renderHeight, positiveNumber(renderViewport?.clientHeight, stageHeight));
    if (!stageWidth || !stageHeight || !renderWidth || !renderHeight) return null;

    const local = input.viewportRect
      ? clipRect(input.viewportRect, renderWidth, renderHeight)
      : transformedContentRect(
        input.sourceWidth,
        input.sourceHeight,
        renderWidth,
        renderHeight,
        input.transform,
      );
    const origin = relativeRenderOrigin(stage, renderViewport);
    const render = freezeRect(
      origin.left,
      origin.top,
      origin.left + renderWidth,
      origin.top + renderHeight,
    );
    const stageRect = clipRect({
      left: origin.left + local.left,
      top: origin.top + local.top,
      right: origin.left + local.right,
      bottom: origin.top + local.bottom,
    }, stageWidth, stageHeight);

    return Object.freeze({
      ...stageRect,
      local,
      render,
      stageWidth,
      stageHeight,
      sourceWidth: positiveNumber(input.sourceWidth, renderWidth),
      sourceHeight: positiveNumber(input.sourceHeight, renderHeight),
    });
  }

  function apply(input = {}) {
    const viewport = resolve(input);
    if (!viewport) return null;
    currentViewport = viewport;

    const nextViewportSignature = viewportSignature(viewport);
    if (lastViewportSignature !== nextViewportSignature) {
      lastViewportSignature = nextViewportSignature;
      stage.style.setProperty("--carrot-viewport-left", `${Math.round(viewport.left)}px`);
      stage.style.setProperty("--carrot-viewport-top", `${Math.round(viewport.top)}px`);
      stage.style.setProperty("--carrot-viewport-width", `${Math.round(viewport.width)}px`);
      stage.style.setProperty("--carrot-viewport-height", `${Math.round(viewport.height)}px`);
      if (typeof target.dispatchEvent === "function" && typeof target.CustomEvent === "function") {
        target.dispatchEvent(new target.CustomEvent("carrot:viewportlayout", { detail: viewport }));
      }
    }

    if (card) {
      if (!target.CarrotLayout?.isWide?.()) {
        if (lastHudLeft !== "" || lastHudBottom !== "") clearHudPosition();
      } else {
        const left = `${Math.round(clamp(viewport.stageWidth * 0.028, 16, 28))}px`;
        const bottom = `${Math.round(clamp(viewport.stageHeight * 0.038, 20, 30))}px`;
        if (lastHudLeft !== left) {
          lastHudLeft = left;
          card.style.setProperty("--carrot-hud-left", left);
        }
        if (lastHudBottom !== bottom) {
          lastHudBottom = bottom;
          card.style.setProperty("--carrot-hud-bottom", bottom);
        }
      }
    }
    return viewport;
  }

  function current() {
    return currentViewport;
  }

  function status() {
    return Object.freeze({
      viewportSignature: lastViewportSignature,
      viewport: currentViewport,
      hudLeft: lastHudLeft,
      hudBottom: lastHudBottom,
    });
  }

  return Object.freeze({ apply, current, reset, resolve, status });
}

export const DriveVisionViewport = Object.freeze({ create: createDriveVisionViewport });
export const DriveVisionHudLayout = DriveVisionViewport;

export function installDriveVisionHudLayoutFacade(target = globalThis) {
  target.DriveVisionViewport = DriveVisionViewport;
  target.DriveVisionHudLayout = DriveVisionViewport;
  return DriveVisionViewport;
}
