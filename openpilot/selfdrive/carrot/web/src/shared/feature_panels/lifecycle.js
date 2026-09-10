/**
 * Owns resources created by a mounted feature panel.
 *
 * A panel receives this object from the settings extension registry. Nothing
 * starts automatically: adapters opt into timers or requests only while their
 * host is mounted. Destroying the panel aborts requests and clears every
 * registered cleanup exactly once.
 */
export function createFeaturePanelLifecycle() {
  const controller = new AbortController();
  const cleanups = new Set();
  let destroyed = false;

  function addCleanup(cleanup) {
    if (typeof cleanup !== "function") return () => {};
    if (destroyed) {
      cleanup();
      return () => {};
    }
    cleanups.add(cleanup);
    return () => cleanups.delete(cleanup);
  }

  function setIntervalWhileMounted(callback, intervalMs) {
    const timer = globalThis.setInterval(callback, intervalMs);
    addCleanup(() => globalThis.clearInterval(timer));
    return timer;
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    controller.abort();
    for (const cleanup of cleanups) cleanup();
    cleanups.clear();
    return true;
  }

  return Object.freeze({
    signal: controller.signal,
    addCleanup,
    setIntervalWhileMounted,
    destroy,
    get destroyed() { return destroyed; },
  });
}
