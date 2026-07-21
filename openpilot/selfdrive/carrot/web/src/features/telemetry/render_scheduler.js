function finiteNonNegative(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) && number >= 0 ? number : fallback;
}

/**
 * Low-priority latest-only scheduler for auxiliary telemetry surfaces.
 *
 * Camera delivery and presentation stay on their own hot path. Auxiliary
 * canvases cross a timer boundary, collapse bursts to the newest state, and
 * render no faster than the cadence selected by their feature.
 */
export function createLatestOnlyRenderScheduler(options = {}) {
  const target = options.target || globalThis;
  const nowMs = options.nowMs || (() => target.performance?.now?.() ?? Date.now());
  const setTimeoutFn = options.setTimeout || target.setTimeout?.bind(target);
  const clearTimeoutFn = options.clearTimeout || target.clearTimeout?.bind(target);
  const requestIdleFn = options.requestIdleCallback || target.requestIdleCallback?.bind(target);
  const cancelIdleFn = options.cancelIdleCallback || target.cancelIdleCallback?.bind(target);
  const onFlush = options.onFlush;
  if (typeof setTimeoutFn !== "function" || typeof clearTimeoutFn !== "function"
    || typeof onFlush !== "function") return null;

  let timerId = null;
  let idleId = null;
  let scheduledAtMs = null;
  let lastFlushAtMs = null;
  let pending = false;
  let destroyed = false;
  let cadenceMs = Math.max(1, finiteNonNegative(options.cadenceMs, 100));
  const idleTimeoutMs = Math.max(1, finiteNonNegative(options.idleTimeoutMs, 12));
  const slowFlushThresholdMs = Math.max(1, finiteNonNegative(options.slowFlushThresholdMs, 8));
  let requests = 0;
  let flushes = 0;
  let lastFlushDurationMs = 0;
  let maxFlushDurationMs = 0;
  let slowFlushes = 0;

  function clearTimer() {
    if (timerId === null) return false;
    clearTimeoutFn(timerId);
    timerId = null;
    scheduledAtMs = null;
    return true;
  }

  function clearIdle() {
    if (idleId === null) return false;
    cancelIdleFn?.(idleId);
    idleId = null;
    scheduledAtMs = null;
    return true;
  }

  function run() {
    timerId = null;
    idleId = null;
    scheduledAtMs = null;
    if (destroyed || !pending) return false;
    pending = false;
    const startedAtMs = finiteNonNegative(nowMs());
    lastFlushAtMs = startedAtMs;
    flushes += 1;
    try {
      onFlush();
    } finally {
      lastFlushDurationMs = Math.max(0, finiteNonNegative(nowMs()) - startedAtMs);
      maxFlushDurationMs = Math.max(maxFlushDurationMs, lastFlushDurationMs);
      if (lastFlushDurationMs > slowFlushThresholdMs) slowFlushes += 1;
    }
    return true;
  }

  function yieldThenRun() {
    timerId = null;
    if (destroyed || !pending) {
      scheduledAtMs = null;
      return false;
    }
    // Auxiliary canvases yield once after their cadence timer. On browsers
    // with requestIdleCallback this lets camera/video presentation and the
    // road overlay finish first, while the short timeout keeps the visible
    // graph/forward refresh effectively unchanged.
    if (typeof requestIdleFn === "function" && typeof cancelIdleFn === "function") {
      idleId = requestIdleFn(run, { timeout: Math.min(cadenceMs, idleTimeoutMs) });
      return true;
    }
    return run();
  }

  function request(nextCadenceMs = cadenceMs, requestOptions = {}) {
    if (destroyed) return false;
    requests += 1;
    pending = true;
    cadenceMs = Math.max(1, finiteNonNegative(nextCadenceMs, cadenceMs));
    const current = finiteNonNegative(nowMs());
    const earliest = requestOptions.immediate === true || lastFlushAtMs === null
      ? current
      : Math.max(current, lastFlushAtMs + cadenceMs);
    if (idleId !== null) return true;
    if (timerId !== null && scheduledAtMs !== null && scheduledAtMs <= earliest) return true;
    clearTimer();
    scheduledAtMs = earliest;
    timerId = setTimeoutFn(yieldThenRun, Math.max(0, earliest - current));
    return true;
  }

  function cancel() {
    pending = false;
    const timerCleared = clearTimer();
    const idleCleared = clearIdle();
    return timerCleared || idleCleared;
  }

  function flush() {
    if (destroyed || !pending) return false;
    clearTimer();
    clearIdle();
    return run();
  }

  function status() {
    return Object.freeze({
      pending,
      scheduled: timerId !== null || idleId !== null,
      cadenceMs,
      requests,
      flushes,
      lastFlushAtMs,
      lastFlushDurationMs,
      maxFlushDurationMs,
      slowFlushes,
    });
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    cancel();
    return true;
  }

  return Object.freeze({ request, cancel, flush, status, destroy });
}
