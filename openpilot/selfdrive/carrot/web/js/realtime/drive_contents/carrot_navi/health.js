"use strict";

globalThis.CarrotNaviHealth = (() => {
  const DEFAULT_THRESHOLDS = Object.freeze({
    mapStaleMs: 1900,
    stateStaleMs: 3500,
    disconnectGraceMs: 1500,
    // The browser's media stall window is roughly three seconds. Keep the last
    // guided map for one additional recovery window, then release the split.
    sourceSuspendGraceMs: 6000,
    mediaFreshMs: 1600,
  });

  function create(options = {}) {
    const transport = options.transport;
    const mse = options.mse;
    const compositor = options.compositor;
    if (!transport || !mse || !compositor) return null;

    const thresholds = Object.freeze({
      ...DEFAULT_THRESHOLDS,
      ...(options.thresholds || {}),
    });
    let sourceSeen = false;
    let sourceConnected = false;
    let lastSourceAt = 0;
    let disconnectedAt = 0;
    let routeActive = false;
    let sourceSuspendedAt = 0;

    function recordState(state, now = performance.now()) {
      const connected = state?.connected === true;
      const navigation = state?.navigationStatus;
      routeActive = Boolean(navigation?.guidanceActive || navigation?.routePresent);
      sourceSeen = true;
      lastSourceAt = now;
      if (connected) {
        sourceConnected = true;
        disconnectedAt = 0;
        sourceSuspendedAt = 0;
      } else {
        if (sourceConnected || !disconnectedAt) disconnectedAt = now;
        sourceConnected = false;
        if (routeActive && !sourceSuspendedAt) sourceSuspendedAt = now;
        if (!routeActive) sourceSuspendedAt = 0;
      }
    }

    function recordDisconnect(now = performance.now()) {
      if (sourceConnected || !disconnectedAt) disconnectedAt = now;
      sourceConnected = false;
      if (routeActive && !sourceSuspendedAt) sourceSuspendedAt = now;
    }

    function reset() {
      sourceSeen = false;
      sourceConnected = false;
      lastSourceAt = 0;
      disconnectedAt = 0;
      routeActive = false;
      sourceSuspendedAt = 0;
    }

    function sourceSnapshot(now = performance.now()) {
      const ageMs = lastSourceAt > 0 ? Math.max(0, Math.round(now - lastSourceAt)) : null;
      const disconnectedForMs = disconnectedAt > 0 ? Math.max(0, Math.round(now - disconnectedAt)) : null;
      const suspendedForMs = sourceSuspendedAt > 0 ? Math.max(0, Math.round(now - sourceSuspendedAt)) : null;
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

    function mediaFresh(now = performance.now(), decoderState = mse.snapshot()) {
      const lastSegmentAt = Number(decoderState?.media?.lastSegmentAt) || 0;
      return lastSegmentAt > 0 && now - lastSegmentAt <= thresholds.mediaFreshMs;
    }

    function sourceAvailable(now = performance.now()) {
      const source = sourceSnapshot(now);
      if (!source.seen || !source.fresh) return false;
      if (source.connected) return true;
      return source.routeActive
        && source.suspendedForMs !== null
        && source.suspendedForMs <= thresholds.sourceSuspendGraceMs;
    }

    function mapReady(now = performance.now()) {
      const presentation = compositor.snapshot();
      if (!presentation.hasFrame || !compositor.fresh(thresholds.mapStaleMs, now)) return false;
      const source = sourceSnapshot(now);
      if (!source.seen) return true;
      if (source.connected) return source.fresh;
      return source.disconnectedForMs !== null && source.disconnectedForMs <= thresholds.disconnectGraceMs;
    }

    function snapshot(decodeError = "", now = performance.now()) {
      const source = sourceSnapshot(now);
      const transportState = transport.snapshot();
      const decoderState = mse.snapshot();
      const presentationState = compositor.snapshot();
      return {
        ready: mapReady(now),
        source: source.fresh && source.connected,
        transport: transportState.mediaOpen && mediaFresh(now, decoderState),
        decoder: decoderState.mse.readyState === "open" && !decoderState.error && !decodeError,
        presentation: presentationState.hasFrame && compositor.fresh(thresholds.mapStaleMs, now),
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

  return Object.freeze({ DEFAULT_THRESHOLDS, create });
})();
