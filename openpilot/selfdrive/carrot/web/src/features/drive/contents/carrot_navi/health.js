export const DEFAULT_HEALTH_THRESHOLDS = Object.freeze({
  mapStaleMs: 1900,
  stateStaleMs: 3500,
  disconnectGraceMs: 1500,
  sourceSuspendGraceMs: 6000,
  mediaFreshMs: 1600,
});

export function createHealth(options = {}) {
  const transport = options.transport;
  const mse = options.mse;
  const compositor = options.compositor;
  if (!transport || !mse || !compositor) return null;

  const now = options.now || (() => globalThis.performance.now());
  const thresholds = Object.freeze({
    ...DEFAULT_HEALTH_THRESHOLDS,
    ...(options.thresholds || {}),
  });
  let sourceSeen = false;
  let sourceConnected = false;
  let lastSourceAt = 0;
  let disconnectedAt = 0;
  let routeActive = false;
  let sourceSuspendedAt = 0;

  function recordState(state, currentTime = now()) {
    const connected = state?.connected === true;
    const navigation = state?.navigationStatus;
    routeActive = Boolean(navigation?.guidanceActive || navigation?.routePresent);
    sourceSeen = true;
    lastSourceAt = currentTime;
    if (connected) {
      sourceConnected = true;
      disconnectedAt = 0;
      sourceSuspendedAt = 0;
    } else {
      if (sourceConnected || !disconnectedAt) disconnectedAt = currentTime;
      sourceConnected = false;
      if (routeActive && !sourceSuspendedAt) sourceSuspendedAt = currentTime;
      if (!routeActive) sourceSuspendedAt = 0;
    }
  }

  function recordDisconnect(currentTime = now()) {
    if (sourceConnected || !disconnectedAt) disconnectedAt = currentTime;
    sourceConnected = false;
    if (routeActive && !sourceSuspendedAt) sourceSuspendedAt = currentTime;
  }

  function reset() {
    sourceSeen = false;
    sourceConnected = false;
    lastSourceAt = 0;
    disconnectedAt = 0;
    routeActive = false;
    sourceSuspendedAt = 0;
  }

  function sourceSnapshot(currentTime = now()) {
    const ageMs = lastSourceAt > 0 ? Math.max(0, Math.round(currentTime - lastSourceAt)) : null;
    const disconnectedForMs = disconnectedAt > 0 ? Math.max(0, Math.round(currentTime - disconnectedAt)) : null;
    const suspendedForMs = sourceSuspendedAt > 0 ? Math.max(0, Math.round(currentTime - sourceSuspendedAt)) : null;
    const fresh = sourceSeen && ageMs !== null && ageMs <= thresholds.stateStaleMs;
    return {
      seen: sourceSeen,
      connected: sourceConnected,
      routeActive,
      suspended: !sourceConnected && routeActive,
      fresh,
      ageMs,
      disconnectedForMs,
      suspendedForMs,
    };
  }

  function mediaFresh(currentTime = now(), decoderState = mse.snapshot()) {
    const lastSegmentAt = Number(decoderState?.media?.lastSegmentAt) || 0;
    return lastSegmentAt > 0 && currentTime - lastSegmentAt <= thresholds.mediaFreshMs;
  }

  function sourceAvailable(currentTime = now()) {
    const source = sourceSnapshot(currentTime);
    if (!source.seen || !source.fresh) return false;
    if (source.connected) return true;
    return source.routeActive
      && source.suspendedForMs !== null
      && source.suspendedForMs <= thresholds.sourceSuspendGraceMs;
  }

  function mapReady(currentTime = now()) {
    const presentation = compositor.snapshot();
    if (!presentation.hasFrame || !compositor.fresh(thresholds.mapStaleMs, currentTime)) return false;
    const source = sourceSnapshot(currentTime);
    if (!source.seen) return true;
    if (source.connected) return source.fresh;
    return source.disconnectedForMs !== null && source.disconnectedForMs <= thresholds.disconnectGraceMs;
  }

  function snapshot(decodeError = "", currentTime = now()) {
    const source = sourceSnapshot(currentTime);
    const transportState = transport.snapshot();
    const decoderState = mse.snapshot();
    const presentationState = compositor.snapshot();
    return {
      ready: mapReady(currentTime),
      source: source.fresh && source.connected,
      transport: transportState.mediaOpen && mediaFresh(currentTime, decoderState),
      decoder: decoderState.mse.readyState === "open" && !decoderState.error && !decodeError,
      presentation: presentationState.hasFrame && compositor.fresh(thresholds.mapStaleMs, currentTime),
    };
  }

  return Object.freeze({
    recordState,
    recordDisconnect,
    reset,
    sourceSnapshot,
    mediaFresh,
    mapReady,
    sourceAvailable,
    snapshot,
    thresholds,
  });
}

export const CarrotNaviHealth = Object.freeze({
  DEFAULT_THRESHOLDS: DEFAULT_HEALTH_THRESHOLDS,
  create: createHealth,
});

export function installCarrotNaviHealthGlobal(target = globalThis) {
  target.CarrotNaviHealth = CarrotNaviHealth;
  return CarrotNaviHealth;
}
