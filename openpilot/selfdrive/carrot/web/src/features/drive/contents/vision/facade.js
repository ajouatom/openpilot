"use strict";

export const DRIVE_VISION_REQUIRED_METHODS = Object.freeze([
  "refresh",
  "resize",
  "requestRender",
  "renderText",
  "renderReplayVideoFrame",
  "resetReplayTemporalState",
  "getDisplayModeIndex",
  "setDisplayModeIndex",
]);

let facadeSingleton = null;
let facadeImplementation = null;

export let DriveVisionFacade = null;

export function createDriveVisionFacade(implementation) {
  if (
    !implementation
    || DRIVE_VISION_REQUIRED_METHODS.some((name) => typeof implementation[name] !== "function")
  ) return null;

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
}

export function getDriveVisionFacade() {
  return facadeSingleton;
}

export function installDriveVisionFacade(target = globalThis, options = {}) {
  const implementation = options.implementation ?? target?.HomeDrive;
  if (!implementation) return null;

  if (!facadeSingleton) {
    facadeSingleton = createDriveVisionFacade(implementation);
    if (!facadeSingleton) return null;
    facadeImplementation = implementation;
    DriveVisionFacade = facadeSingleton;
  } else if (implementation !== facadeImplementation) {
    throw new Error("DriveVisionFacade is an app-lifetime singleton");
  }

  target.DriveVisionFacade = facadeSingleton;
  return facadeSingleton;
}

export function finalizeDriveVisionFacade(target = globalThis, options = {}) {
  return installDriveVisionFacade(target, options);
}
