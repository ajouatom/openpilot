"use strict";

export const VISION_NETWORK_RECOVERY_STATE = Object.freeze({
  DISABLED: "disabled",
  CHECKING: "checking",
  REACHABLE: "reachable",
  WAITING: "waiting",
});

const DEFAULT_BACKOFF_MS = Object.freeze([300, 600, 1200, 2400, 4000]);

function defaultSchedule(task, delayMs) {
  return setTimeout(task, delayMs);
}

function defaultCancel(timerId) {
  clearTimeout(timerId);
}

function normalizeDelay(value, fallback) {
  const number = Number(value);
  return Number.isFinite(number) && number >= 0 ? number : fallback;
}

function freezeSnapshot(state) {
  return Object.freeze({
    enabled: state.enabled,
    state: state.state,
    reachable: state.reachable,
    generation: state.generation,
    failureCount: state.failureCount,
    inFlight: state.inFlight,
    queueScheduled: state.queueScheduled,
    nextCheckAtMs: state.nextCheckAtMs,
    lastSuccessAtMs: state.lastSuccessAtMs,
    lastFailureAtMs: state.lastFailureAtMs,
    lastReason: state.lastReason,
    lastError: state.lastError,
    browserOnlineHint: state.browserOnlineHint,
    originClassification: state.originClassification,
  });
}

export function createVisionNetworkRecoveryController(options = {}) {
  if (typeof options.probe !== "function") {
    throw new TypeError("Carrot Vision network recovery requires a probe function");
  }

  const probe = options.probe;
  const schedule = typeof options.schedule === "function" ? options.schedule : defaultSchedule;
  const cancel = typeof options.cancel === "function" ? options.cancel : defaultCancel;
  const now = typeof options.now === "function" ? options.now : Date.now;
  const onState = typeof options.onState === "function" ? options.onState : null;
  const healthyIntervalMs = normalizeDelay(options.healthyIntervalMs, 12000);
  const probeTimeoutMs = normalizeDelay(options.probeTimeoutMs, 1500);
  const backoffMs = Array.isArray(options.backoffMs) && options.backoffMs.length
    ? Object.freeze(options.backoffMs.map((delay) => normalizeDelay(delay, 500)))
    : DEFAULT_BACKOFF_MS;
  const AbortControllerImpl = options.AbortControllerImpl || globalThis.AbortController;

  let enabled = false;
  let state = VISION_NETWORK_RECOVERY_STATE.DISABLED;
  let reachable = null;
  let generation = 0;
  let failureCount = 0;
  let queueTimer = null;
  let timeoutTimer = null;
  let inFlight = null;
  let probeAbortController = null;
  let nextCheckAtMs = null;
  let lastSuccessAtMs = null;
  let lastFailureAtMs = null;
  let lastReason = "";
  let lastError = "";
  let browserOnlineHint = null;
  let originClassification = "not-checked";

  function snapshot() {
    return freezeSnapshot({
      enabled,
      state,
      reachable,
      generation,
      failureCount,
      inFlight: Boolean(inFlight),
      queueScheduled: queueTimer !== null,
      nextCheckAtMs,
      lastSuccessAtMs,
      lastFailureAtMs,
      lastReason,
      lastError,
      browserOnlineHint,
      originClassification,
    });
  }

  function publish(nextState, detail = {}) {
    state = nextState;
    if (detail.reason) lastReason = String(detail.reason);
    const current = snapshot();
    onState?.(current, Object.freeze({ ...detail }));
    return current;
  }

  function clearQueue() {
    if (queueTimer !== null) cancel(queueTimer);
    queueTimer = null;
    nextCheckAtMs = null;
  }

  function clearProbeTimeout() {
    if (timeoutTimer !== null) cancel(timeoutTimer);
    timeoutTimer = null;
  }

  function scheduleCheck(delayMs, reason, publishWaiting = false) {
    if (!enabled) return false;
    clearQueue();
    const delay = Math.max(0, Number(delayMs) || 0);
    nextCheckAtMs = now() + delay;
    if (publishWaiting) publish(VISION_NETWORK_RECOVERY_STATE.WAITING, { reason, delayMs: delay });
    queueTimer = schedule(() => {
      queueTimer = null;
      nextCheckAtMs = null;
      void check(reason);
    }, delay);
    return true;
  }

  async function check(reason = "network check") {
    if (!enabled) return false;
    if (inFlight) return inFlight;

    clearQueue();
    const checkGeneration = ++generation;
    const retainedReachability = reachable === true;
    probeAbortController = typeof AbortControllerImpl === "function" ? new AbortControllerImpl() : null;
    const signal = probeAbortController?.signal;
    publish(VISION_NETWORK_RECOVERY_STATE.CHECKING, {
      reason,
      retainedReachability,
    });

    if (probeTimeoutMs > 0 && probeAbortController) {
      timeoutTimer = schedule(() => {
        timeoutTimer = null;
        try {
          probeAbortController?.abort(new DOMException("Server reachability probe timed out", "TimeoutError"));
        } catch {
          probeAbortController?.abort();
        }
      }, probeTimeoutMs);
    }

    const task = (async () => {
      try {
        const result = await probe(Object.freeze({
          signal,
          generation: checkGeneration,
          reason,
        }));
        if (!enabled || checkGeneration !== generation) return false;
        if (result === false || result?.ok === false) {
          throw new Error(result?.error || "Server reachability probe failed");
        }

        clearProbeTimeout();
        reachable = true;
        failureCount = 0;
        lastSuccessAtMs = now();
        lastError = "";
        originClassification = "current-origin-reachable";
        publish(VISION_NETWORK_RECOVERY_STATE.REACHABLE, { reason });
        scheduleCheck(healthyIntervalMs, "scheduled server reachability check");
        return true;
      } catch (error) {
        clearProbeTimeout();
        if (!enabled || checkGeneration !== generation || signal?.aborted && error?.name === "AbortError") {
          return false;
        }

        reachable = false;
        failureCount += 1;
        lastFailureAtMs = now();
        lastError = String(error?.message || error || "Server reachability probe failed");
        originClassification = failureCount >= 3
          ? "possible-origin-address-change-or-hotspot-route"
          : "current-origin-unreachable";
        const delay = backoffMs[Math.min(failureCount - 1, backoffMs.length - 1)];
        scheduleCheck(delay, reason, true);
        return false;
      } finally {
        if (checkGeneration === generation) {
          clearProbeTimeout();
          probeAbortController = null;
          inFlight = null;
        }
      }
    })();
    inFlight = task;
    return task;
  }

  function setEnabled(nextEnabled, detail = {}) {
    const next = Boolean(nextEnabled);
    if (next === enabled) {
      if (next && detail.force === true) void requestCheck(detail.reason || "forced network check");
      return snapshot();
    }

    enabled = next;
    if (!enabled) {
      generation += 1;
      clearQueue();
      clearProbeTimeout();
      probeAbortController?.abort();
      probeAbortController = null;
      inFlight = null;
      reachable = null;
      failureCount = 0;
      lastError = "";
      originClassification = "not-checked";
      return publish(VISION_NETWORK_RECOVERY_STATE.DISABLED, {
        reason: detail.reason || "network recovery disabled",
      });
    }

    reachable = false;
    failureCount = 0;
    originClassification = "not-checked";
    void check(detail.reason || "network recovery enabled");
    return snapshot();
  }

  function requestCheck(reason = "network hint") {
    if (!enabled) return Promise.resolve(false);
    clearQueue();
    return check(reason);
  }

  function noteBrowserHint(isOnline, reason = "browser network hint") {
    browserOnlineHint = typeof isOnline === "boolean" ? isOnline : null;
    return requestCheck(reason);
  }

  function destroy() {
    setEnabled(false, { reason: "network recovery destroyed" });
  }

  return Object.freeze({
    setEnabled,
    requestCheck,
    noteBrowserHint,
    reportTransportFailure: requestCheck,
    snapshot,
    destroy,
  });
}

const NETWORK_RECOVERY_API = Object.freeze({
  state: VISION_NETWORK_RECOVERY_STATE,
  create: createVisionNetworkRecoveryController,
});

export function installDriveVisionNetworkRecoveryFacade(target = globalThis) {
  if (!target) return NETWORK_RECOVERY_API;
  if (target.DriveVisionNetworkRecovery) return target.DriveVisionNetworkRecovery;
  target.DriveVisionNetworkRecovery = NETWORK_RECOVERY_API;
  return target.DriveVisionNetworkRecovery;
}

export const DriveVisionNetworkRecovery = NETWORK_RECOVERY_API;
