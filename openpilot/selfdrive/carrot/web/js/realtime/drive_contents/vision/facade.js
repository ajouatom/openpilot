"use strict";

globalThis.DriveVisionFacade = (() => {
  const implementation = window.HomeDrive;
  const requiredMethods = Object.freeze([
    "refresh",
    "resize",
    "requestRender",
    "renderText",
    "renderReplayVideoFrame",
    "resetReplayTemporalState",
    "getDisplayModeIndex",
    "setDisplayModeIndex",
  ]);
  if (!implementation || requiredMethods.some((name) => typeof implementation[name] !== "function")) return null;

  const lifecycle = Object.freeze({
    refresh() {
      return implementation.refresh();
    },
    resize(rect) {
      return implementation.resize(rect);
    },
    renderText() {
      return implementation.renderText();
    },
  });

  const live = Object.freeze({
    requestRender(options = {}) {
      return implementation.requestRender(options);
    },
  });

  const replay = Object.freeze({
    renderVideoFrame(options = {}) {
      return implementation.renderReplayVideoFrame(options);
    },
    resetTemporalState() {
      return implementation.resetReplayTemporalState();
    },
  });

  const display = Object.freeze({
    getModeIndex() {
      return implementation.getDisplayModeIndex();
    },
    setModeIndex(index, options = {}) {
      return implementation.setDisplayModeIndex(index, options);
    },
  });

  function status() {
    return {
      ready: true,
      implementation: "HomeDrive",
      boundaries: ["lifecycle", "live", "replay", "display"],
    };
  }

  return Object.freeze({ lifecycle, live, replay, display, status });
})();
