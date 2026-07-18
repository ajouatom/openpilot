export const SETTINGS_STORE_EVENT = "carrot:settings-store";

export function validateSettingsSnapshot(snapshot) {
  if (!snapshot || typeof snapshot !== "object" || !snapshot.settings) {
    throw new Error("Invalid settings snapshot");
  }
  return snapshot;
}

export function createSettingsStore(options) {
  const loadSnapshot = options?.loadSnapshot;
  const eventTarget = options?.eventTarget || null;
  const now = options?.now || (() => performance.now());
  if (typeof loadSnapshot !== "function") throw new TypeError("loadSnapshot must be a function");

  const listeners = new Set();
  const state = {
    status: "idle",
    snapshot: null,
    promise: null,
    error: null,
    requestedAt: 0,
    resolvedAt: 0,
  };

  function getState() {
    return Object.freeze({
      status: state.status,
      snapshot: state.snapshot,
      error: state.error,
      requestedAt: state.requestedAt,
      resolvedAt: state.resolvedAt,
    });
  }

  function emitStatus() {
    const detail = getState();
    listeners.forEach((listener) => {
      try {
        listener(detail);
      } catch (_) {
        // A UI subscriber must not be able to break snapshot delivery.
      }
    });

    const EventConstructor = eventTarget?.CustomEvent || globalThis.CustomEvent;
    if (typeof eventTarget?.dispatchEvent === "function" && typeof EventConstructor === "function") {
      eventTarget.dispatchEvent(new EventConstructor(SETTINGS_STORE_EVENT, { detail }));
    }
  }

  function load(options = {}) {
    const force = options.force === true;
    if (!force && state.snapshot) return Promise.resolve(state.snapshot);
    if (state.promise) return state.promise;

    state.status = "loading";
    state.error = null;
    state.requestedAt = now();

    const request = Promise.resolve()
      .then(loadSnapshot)
      .then(validateSettingsSnapshot)
      .then((snapshot) => {
        state.snapshot = snapshot;
        state.status = "ready";
        state.resolvedAt = now();
        emitStatus();
        return snapshot;
      })
      .catch((error) => {
        state.status = "error";
        state.error = error?.message || String(error || "unknown");
        emitStatus();
        throw error;
      })
      .finally(() => {
        state.promise = null;
      });

    state.promise = request;
    emitStatus();
    return request;
  }

  function preload() {
    return load().catch(() => null);
  }

  function peek() {
    return state.snapshot;
  }

  function subscribe(listener) {
    if (typeof listener !== "function") return () => {};
    listeners.add(listener);
    return () => listeners.delete(listener);
  }

  return Object.freeze({
    load,
    preload,
    peek,
    subscribe,
    getState,
    get status() { return state.status; },
    get requestedAt() { return state.requestedAt; },
    get resolvedAt() { return state.resolvedAt; },
  });
}
