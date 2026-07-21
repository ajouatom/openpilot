import { finiteOrNull, normalizeDriveInsightsSnapshot } from "./schema.js";

const CONNECTION_STATES = Object.freeze(["idle", "connecting", "connected", "disconnected"]);

function requireProvider(provider) {
  if (!provider || typeof provider.snapshot !== "function" || typeof provider.subscribe !== "function") {
    throw new TypeError("Drive Insights live source requires a provider with snapshot/subscribe");
  }
  return provider;
}

function receivedAt(received, service) {
  const direct = finiteOrNull(received?.[service]);
  if (direct !== null) return direct;
  return finiteOrNull(received?.liveRuntime);
}

function serviceAges(providerSnapshot) {
  const now = finiteOrNull(providerSnapshot?.timestampMs);
  const received = providerSnapshot?.receivedAtMonotonic || {};
  const ages = {};
  for (const service of [
    "carState",
    "modelV2",
    "lateralPlan",
    "radarState",
    "liveTracks",
    "carrotMan",
    "navInstructionCarrot",
    "roadCameraState",
  ]) {
    const serviceReceivedAt = receivedAt(received, service);
    ages[service] = now !== null && serviceReceivedAt !== null && now >= serviceReceivedAt
      ? now - serviceReceivedAt
      : null;
  }
  return ages;
}

function liveContext(providerSnapshot) {
  const connectionState = CONNECTION_STATES.includes(providerSnapshot?.connectionState)
    ? providerSnapshot.connectionState
    : "idle";
  const routeId = typeof providerSnapshot?.routeId === "string" && providerSnapshot.routeId.trim()
    ? providerSnapshot.routeId.trim()
    : null;
  return Object.freeze({ routeId, connectionState });
}

export function normalizeDriveInsightsLiveSnapshot(providerSnapshot) {
  if (!providerSnapshot || typeof providerSnapshot !== "object") {
    throw new TypeError("Drive Insights live provider snapshot must be an object");
  }
  const context = liveContext(providerSnapshot);
  const received = providerSnapshot.receivedAtMonotonic || {};
  return normalizeDriveInsightsSnapshot({
    timestampMs: providerSnapshot.timestampMs,
    hudState: providerSnapshot.hudState,
    overlayState: providerSnapshot.overlayState,
    serviceAges: serviceAges(providerSnapshot),
    cameraFrameTimestampMs: receivedAt(received, "roadCameraState"),
    forceMissing: context.connectionState !== "connected",
  });
}

export function createDriveInsightsLiveSource({ provider } = {}) {
  const sourceProvider = requireProvider(provider);
  let providerSnapshot = sourceProvider.snapshot();
  let normalizedSnapshot = normalizeDriveInsightsLiveSnapshot(providerSnapshot);
  let normalizedProviderSnapshot = providerSnapshot;
  let context = liveContext(providerSnapshot);
  let destroyed = false;
  const listeners = new Set();
  const updateListeners = new Set();

  const stopProvider = sourceProvider.subscribe((nextProviderSnapshot) => {
    if (destroyed) return;
    providerSnapshot = nextProviderSnapshot && typeof nextProviderSnapshot === "object"
      ? nextProviderSnapshot
      : sourceProvider.snapshot();
    context = liveContext(providerSnapshot);
    for (const listener of [...updateListeners]) listener();
    if (listeners.size) {
      const next = snapshot();
      for (const listener of [...listeners]) listener(next);
    }
  });

  function snapshot() {
    if (normalizedProviderSnapshot !== providerSnapshot) {
      normalizedSnapshot = normalizeDriveInsightsLiveSnapshot(providerSnapshot);
      normalizedProviderSnapshot = providerSnapshot;
    }
    return normalizedSnapshot;
  }

  function subscribe(listener) {
    if (destroyed) throw new Error("Drive Insights live source is destroyed");
    if (typeof listener !== "function") throw new TypeError("Drive Insights live listener must be a function");
    listeners.add(listener);
    let active = true;
    return () => {
      if (!active) return false;
      active = false;
      return listeners.delete(listener);
    };
  }

  // Low-rate renderers use this dirty signal and normalize only the latest
  // provider snapshot when their independent presentation clock fires.
  function subscribeUpdates(listener) {
    if (destroyed) throw new Error("Drive Insights live source is destroyed");
    if (typeof listener !== "function") throw new TypeError("Drive Insights live update listener must be a function");
    updateListeners.add(listener);
    let active = true;
    return () => {
      if (!active) return false;
      active = false;
      return updateListeners.delete(listener);
    };
  }

  function sourceContext() {
    return context;
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    listeners.clear();
    updateListeners.clear();
    if (typeof stopProvider === "function") stopProvider();
    return true;
  }

  return Object.freeze({ snapshot, subscribe, subscribeUpdates, context: sourceContext, destroy });
}
