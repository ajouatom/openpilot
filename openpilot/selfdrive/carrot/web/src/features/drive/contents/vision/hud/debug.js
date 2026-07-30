"use strict";

function own(object, key) {
  return Object.prototype.hasOwnProperty.call(object || {}, key);
}

function selectedSourceState(state = {}) {
  const carState = state.carState || {};
  const controlsState = state.controlsState || {};
  const lateralPlan = state.lateralPlan || {};
  const longitudinalPlan = state.longitudinalPlan || {};
  const carrotMan = state.carrotMan || {};
  const selfdriveState = state.selfdriveState || {};
  const carControl = state.carControl || {};
  return {
    services: {
      carState: own(state, "carState"),
      controlsState: own(state, "controlsState"),
      lateralPlan: own(state, "lateralPlan"),
      longitudinalPlan: own(state, "longitudinalPlan"),
      carrotMan: own(state, "carrotMan"),
      selfdriveState: own(state, "selfdriveState"),
      carControl: own(state, "carControl"),
    },
    carState: {
      vEgoCluster: carState.vEgoCluster ?? null,
      vCruiseCluster: carState.vCruiseCluster ?? null,
      vCruise: carState.vCruise ?? null,
      evModeValid: carState.evModeValid ?? null,
      evModeActive: carState.evModeActive ?? null,
      gearShifter: carState.gearShifter ?? null,
      useLaneLineSpeed: carState.useLaneLineSpeed ?? null,
    },
    controlsState: {
      enabled: controlsState.enabled ?? null,
      vCruiseCluster: controlsState.vCruiseCluster ?? null,
      activeLaneLine: controlsState.activeLaneLine ?? null,
    },
    lateralPlan: {
      useLaneLines: lateralPlan.useLaneLines ?? null,
    },
    longitudinalPlan: {
      cruiseTarget: longitudinalPlan.cruiseTarget ?? null,
      trafficState: longitudinalPlan.trafficState ?? null,
    },
    carrotMan: {
      desiredSpeed: carrotMan.desiredSpeed ?? null,
      desiredSource: carrotMan.desiredSource ?? null,
      trafficState: carrotMan.trafficState ?? null,
      xSpdType: carrotMan.xSpdType ?? null,
      xSpdLimit: carrotMan.xSpdLimit ?? null,
      xSpdDist: carrotMan.xSpdDist ?? null,
      xSpdCountDown: carrotMan.xSpdCountDown ?? null,
    },
    selfdriveState: {
      enabled: selfdriveState.enabled ?? null,
    },
    carControl: {
      latActive: carControl.latActive ?? null,
    },
  };
}

function replayState(target) {
  const status = target.CarrotVisionReplay?.status?.() || {};
  return {
    active: status.active === true,
    ready: status.ready === true,
    loading: status.loading === true,
    route: status.route ?? null,
    segment: status.segment ?? null,
    currentTime: Number.isFinite(Number(status.currentTime)) ? Number(status.currentTime) : null,
    paused: status.paused !== false,
    services: Object.keys(status.services || {}).sort(),
  };
}

function nodeState(target, node, options = {}) {
  if (!node) return { present: false };
  let style = null;
  try {
    style = target.getComputedStyle?.(node) || target.document?.defaultView?.getComputedStyle?.(node) || null;
  } catch {}
  const rect = node.getBoundingClientRect?.();
  const result = {
    present: true,
    hidden: node.hidden === true,
    display: style?.display ?? null,
    visibility: style?.visibility ?? null,
    opacity: style?.opacity ?? null,
  };
  if (rect) {
    result.rect = {
      x: Math.round(Number(rect.x) || 0),
      y: Math.round(Number(rect.y) || 0),
      width: Math.round(Number(rect.width) || 0),
      height: Math.round(Number(rect.height) || 0),
    };
  }
  if (options.text === true) result.text = String(node.textContent || "").trim().slice(0, 80);
  return result;
}

export function createHudDebugFacade(target = globalThis, overlay = target.CarrotHudOverlay) {
  function snapshot() {
    const source = target.CarrotHudState || {};
    const root = overlay?.root || target.document?.querySelector?.("[data-carrot-hud='overlay']") || null;
    const stage = target.document?.getElementById?.("carrotStage") || null;
    const trafficLights = root?.querySelectorAll?.(".chud-speed-traffic") || [];
    return {
      version: 1,
      capturedAt: new Date().toISOString(),
      replay: {
        ...replayState(target),
        hudToggleOn: stage?.classList?.contains?.("is-replay-hud-visible") ?? null,
      },
      source: selectedSourceState(source),
      derived: target.CarrotHudDataBridge?.deriveVehicleHudPayload?.(source) || null,
      presentation: {
        overlay: overlay?.status?.() || null,
        content: target.DriveVisionHudContent?.status?.() || null,
      },
      dom: {
        root: nodeState(target, root),
        ev: nodeState(target, root?.querySelector?.(".chud-t-ev"), { text: true }),
        lane: nodeState(target, root?.querySelector?.(".chud-lfa-lane")),
        cruiseOverride: nodeState(target, root?.querySelector?.(".chud-t-override"), { text: true }),
        cruiseOverrideLabel: nodeState(target, root?.querySelector?.(".chud-t-override-label"), { text: true }),
        trafficRed: nodeState(target, trafficLights[0]),
        trafficGreen: nodeState(target, trafficLights[1]),
      },
    };
  }

  function text() {
    return JSON.stringify(snapshot(), null, 2);
  }

  async function copy() {
    const value = text();
    try {
      const clipboard = target.navigator?.clipboard;
      if (typeof clipboard?.writeText !== "function") throw new Error("Clipboard API unavailable");
      await clipboard.writeText(value);
      return { copied: true, length: value.length };
    } catch {
      target.console?.log?.(value);
      return { copied: false, length: value.length, text: value };
    }
  }

  return Object.freeze({ snapshot, text, copy });
}
