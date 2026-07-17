"use strict";

globalThis.DriveVisionHudLayout = (() => {
  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  function finiteNumber(value, fallback = 0) {
    const numeric = Number(value);
    return Number.isFinite(numeric) ? numeric : fallback;
  }

  function create(options = {}) {
    const stage = options.stage;
    const card = options.card;
    if (!stage) return null;

    let lastHudLeft = "";
    let lastHudBottom = "";
    let lastViewportSignature = "";

    function clearHudPosition() {
      if (card) {
        card.style.removeProperty("--carrot-hud-left");
        card.style.removeProperty("--carrot-hud-bottom");
      }
      lastHudLeft = "";
      lastHudBottom = "";
    }

    function reset() {
      clearHudPosition();
      stage.style.removeProperty("--carrot-viewport-left");
      stage.style.removeProperty("--carrot-viewport-top");
      stage.style.removeProperty("--carrot-viewport-width");
      stage.style.removeProperty("--carrot-viewport-height");
      lastViewportSignature = "";
    }

    function apply(viewportRect) {
      if (!window.CarrotLayout?.isWide?.()) {
        if (lastHudLeft !== "" || lastHudBottom !== "") clearHudPosition();
        return false;
      }

      const stageWidth = stage.clientWidth || viewportRect?.width || 0;
      const stageHeight = stage.clientHeight || viewportRect?.height || 0;
      if (!stageWidth || !stageHeight) return false;
      const viewport = {
        left: Math.round(finiteNumber(viewportRect?.left, 0)),
        top: Math.round(finiteNumber(viewportRect?.top, 0)),
        width: Math.round(finiteNumber(viewportRect?.width, stageWidth)),
        height: Math.round(finiteNumber(viewportRect?.height, stageHeight)),
        stageWidth: Math.round(stageWidth),
        stageHeight: Math.round(stageHeight),
      };
      const viewportSignature = [
        viewport.left,
        viewport.top,
        viewport.width,
        viewport.height,
        viewport.stageWidth,
        viewport.stageHeight,
      ].join(",");
      let changed = false;
      if (lastViewportSignature !== viewportSignature) {
        lastViewportSignature = viewportSignature;
        stage.style.setProperty("--carrot-viewport-left", `${viewport.left}px`);
        stage.style.setProperty("--carrot-viewport-top", `${viewport.top}px`);
        stage.style.setProperty("--carrot-viewport-width", `${viewport.width}px`);
        stage.style.setProperty("--carrot-viewport-height", `${viewport.height}px`);
        window.dispatchEvent(new CustomEvent("carrot:viewportlayout", { detail: viewport }));
        changed = true;
      }

      if (!card) return changed;
      const left = `${Math.round(clamp(stageWidth * 0.028, 16, 28))}px`;
      const bottom = `${Math.round(clamp(stageHeight * 0.038, 20, 30))}px`;
      if (lastHudLeft !== left) {
        lastHudLeft = left;
        card.style.setProperty("--carrot-hud-left", left);
        changed = true;
      }
      if (lastHudBottom !== bottom) {
        lastHudBottom = bottom;
        card.style.setProperty("--carrot-hud-bottom", bottom);
        changed = true;
      }
      return changed;
    }

    function status() {
      return {
        viewportSignature: lastViewportSignature,
        hudLeft: lastHudLeft,
        hudBottom: lastHudBottom,
      };
    }

    return Object.freeze({ apply, reset, status });
  }

  return Object.freeze({ create });
})();
