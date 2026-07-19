export const DRIVE_INSIGHTS_HISTORY_SOURCE = Object.freeze({
  LIVE: "live",
  REPLAY: "replay",
});

export const DRIVE_INSIGHTS_HISTORY_STATE = Object.freeze({
  FRESH: "fresh",
  STALE: "stale",
  MISSING: "missing",
});

export const DRIVE_INSIGHTS_HISTORY_DEFAULTS = Object.freeze({
  cadenceMs: 100,
  windowMs: 10_000,
  maxSamples: 200,
  liveStaleAfterMs: 2_000,
  replaySeekResetThresholdMs: 500,
});

const CONNECTED_STATES = new Set(["connected", "online", "open", "ready"]);
const SOURCE_VALUES = new Set(Object.values(DRIVE_INSIGHTS_HISTORY_SOURCE));

export class DriveInsightsHistoryError extends Error {
  constructor(code, message) {
    super(message);
    this.name = "DriveInsightsHistoryError";
    this.code = code;
  }
}

function fail(code, message) {
  throw new DriveInsightsHistoryError(code, message);
}

function hasOwn(value, key) {
  return Object.prototype.hasOwnProperty.call(value || {}, key);
}

function finiteTimestamp(value, label) {
  const timestamp = Number(value);
  if (!Number.isFinite(timestamp) || timestamp < 0) {
    fail("INVALID_TIMESTAMP", `${label} must be a finite, non-negative number`);
  }
  return timestamp;
}

function positiveNumber(value, label) {
  const number = Number(value);
  if (!Number.isFinite(number) || number <= 0) {
    fail("INVALID_HISTORY_OPTIONS", `${label} must be a finite number greater than zero`);
  }
  return number;
}

function positiveInteger(value, label) {
  const number = Number(value);
  if (!Number.isSafeInteger(number) || number <= 0) {
    fail("INVALID_HISTORY_OPTIONS", `${label} must be a positive safe integer`);
  }
  return number;
}

function normalizeOptions(options = {}) {
  const cadenceMs = positiveNumber(
    options.cadenceMs ?? DRIVE_INSIGHTS_HISTORY_DEFAULTS.cadenceMs,
    "cadenceMs",
  );
  const windowMs = positiveNumber(
    options.windowMs ?? DRIVE_INSIGHTS_HISTORY_DEFAULTS.windowMs,
    "windowMs",
  );
  const maxSamples = positiveInteger(
    options.maxSamples ?? DRIVE_INSIGHTS_HISTORY_DEFAULTS.maxSamples,
    "maxSamples",
  );
  const liveStaleAfterMs = positiveNumber(
    options.liveStaleAfterMs ?? DRIVE_INSIGHTS_HISTORY_DEFAULTS.liveStaleAfterMs,
    "liveStaleAfterMs",
  );
  const replaySeekResetThresholdMs = positiveNumber(
    options.replaySeekResetThresholdMs
      ?? DRIVE_INSIGHTS_HISTORY_DEFAULTS.replaySeekResetThresholdMs,
    "replaySeekResetThresholdMs",
  );
  return Object.freeze({
    cadenceMs,
    windowMs,
    maxSamples,
    liveStaleAfterMs,
    replaySeekResetThresholdMs,
  });
}

function normalizeSource(value) {
  const source = String(value || "").trim().toLowerCase();
  if (!SOURCE_VALUES.has(source)) {
    fail("INVALID_SOURCE", `unsupported Drive Insights history source: ${String(value)}`);
  }
  return source;
}

function normalizeRouteId(value) {
  if (value == null || value === "") return null;
  return String(value);
}

function isConnected(value) {
  if (typeof value === "boolean") return value;
  return CONNECTED_STATES.has(String(value || "").trim().toLowerCase());
}

function assertSample(sample, label = "sample") {
  if (!sample || typeof sample !== "object" || Array.isArray(sample)) {
    fail("INVALID_SAMPLE", `${label} must be a normalized Drive Insights snapshot`);
  }
  finiteTimestamp(sample.timestampMs, `${label}.timestampMs`);
  return sample;
}

function bucketFor(timestampMs, cadenceMs) {
  return Math.floor(timestampMs / cadenceMs);
}

function defaultNowMs() {
  if (typeof globalThis.performance?.now !== "function") {
    fail("MONOTONIC_CLOCK_UNAVAILABLE", "performance.now() is required for live history status");
  }
  return globalThis.performance.now();
}

export function createReplayHistoryTimestamps(currentTimestampMs, options = {}) {
  const config = normalizeOptions(options);
  const endTimestampMs = finiteTimestamp(currentTimestampMs, "currentTimestampMs");
  const earliestTimestampMs = Math.max(0, endTimestampMs - config.windowMs);
  const availableCount = Math.floor((endTimestampMs - earliestTimestampMs) / config.cadenceMs) + 1;
  const count = Math.min(config.maxSamples, availableCount);
  const timestamps = [];
  for (let index = count - 1; index >= 0; index -= 1) {
    timestamps.push(endTimestampMs - (index * config.cadenceMs));
  }
  return Object.freeze(timestamps);
}

export function createDriveInsightsHistory(options = {}) {
  const config = normalizeOptions(options);
  const nowMs = typeof options.nowMs === "function" ? options.nowMs : defaultNowMs;

  let source = null;
  let routeId = null;
  let routeSeen = false;
  let connectionState = null;
  let connectionSeen = false;
  let connectionId = null;
  let connectionIdSeen = false;
  let liveConnectedOnce = false;
  let samples = [];
  let anchorTimestampMs = null;
  let lastLiveTimestampMs = null;
  let lastLiveReceivedAtMonotonicMs = null;
  let lastReplayPositionMs = null;
  let generation = 0;
  let lastResetReason = "initial";

  function clearSamples(reason) {
    samples = [];
    anchorTimestampMs = null;
    lastLiveTimestampMs = null;
    lastLiveReceivedAtMonotonicMs = null;
    lastReplayPositionMs = null;
    generation += 1;
    lastResetReason = String(reason || "manual");
  }

  function resetSourceTracking(nextSource) {
    routeId = null;
    routeSeen = false;
    connectionState = null;
    connectionSeen = false;
    connectionId = null;
    connectionIdSeen = false;
    liveConnectedOnce = false;
    source = nextSource;
  }

  function applyContext(context = {}) {
    if (!context || typeof context !== "object" || Array.isArray(context)) {
      fail("INVALID_CONTEXT", "Drive Insights history context must be an object");
    }

    const nextSource = hasOwn(context, "source")
      ? normalizeSource(context.source)
      : source;
    if (nextSource === null) {
      fail("INVALID_SOURCE", "Drive Insights history source is required");
    }

    let resetReason = null;
    if (source !== null && source !== nextSource) resetReason = "source-change";

    if (resetReason) {
      clearSamples(resetReason);
      resetSourceTracking(nextSource);
    } else if (source === null) {
      resetSourceTracking(nextSource);
    }

    const nextRouteSeen = hasOwn(context, "routeId");
    const nextRouteId = nextRouteSeen ? normalizeRouteId(context.routeId) : routeId;
    if (!resetReason
      && source === DRIVE_INSIGHTS_HISTORY_SOURCE.REPLAY
      && routeSeen
      && nextRouteSeen
      && routeId !== nextRouteId) {
      resetReason = "replay-route-change";
      clearSamples(resetReason);
    }

    const nextConnectionSeen = hasOwn(context, "connectionState");
    const nextConnectionState = nextConnectionSeen ? context.connectionState : connectionState;
    const nextConnectionIdSeen = hasOwn(context, "connectionId");
    const nextConnectionId = nextConnectionIdSeen
      ? normalizeRouteId(context.connectionId)
      : connectionId;

    if (!resetReason && source === DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE) {
      const transitionedToConnected = connectionSeen
        && nextConnectionSeen
        && !isConnected(connectionState)
        && isConnected(nextConnectionState)
        && liveConnectedOnce;
      const changedConnectionId = connectionIdSeen
        && nextConnectionIdSeen
        && connectionId !== nextConnectionId;
      if (context.reconnected === true || transitionedToConnected || changedConnectionId) {
        resetReason = "live-reconnect";
        clearSamples(resetReason);
      }
    }

    if (source === DRIVE_INSIGHTS_HISTORY_SOURCE.REPLAY
      && hasOwn(context, "currentTimestampMs")) {
      const position = finiteTimestamp(context.currentTimestampMs, "currentTimestampMs");
      if (!resetReason
        && lastReplayPositionMs !== null
        && Math.abs(position - lastReplayPositionMs) > config.replaySeekResetThresholdMs) {
        resetReason = "replay-seek";
        clearSamples(resetReason);
      }
      lastReplayPositionMs = position;
    }

    if (nextRouteSeen) {
      routeId = nextRouteId;
      routeSeen = true;
    }
    if (nextConnectionSeen) {
      connectionState = nextConnectionState;
      connectionSeen = true;
      if (isConnected(nextConnectionState)) liveConnectedOnce = true;
    }
    if (nextConnectionIdSeen) {
      connectionId = nextConnectionId;
      connectionIdSeen = true;
    }

    return Object.freeze({ reset: resetReason !== null, reason: resetReason });
  }

  function trimLive() {
    if (anchorTimestampMs === null) return;
    const cutoff = Math.max(0, anchorTimestampMs - config.windowMs);
    samples = samples.filter((sample) => sample.timestampMs >= cutoff);
    if (samples.length > config.maxSamples) samples = samples.slice(-config.maxSamples);
  }

  function pushLive(sample, context = {}) {
    const normalizedSample = assertSample(sample);
    const contextResult = applyContext({ ...context, source: DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE });
    const timestampMs = finiteTimestamp(normalizedSample.timestampMs, "sample.timestampMs");
    const receivedAtMonotonicMs = hasOwn(context, "receivedAtMonotonicMs")
      ? finiteTimestamp(context.receivedAtMonotonicMs, "receivedAtMonotonicMs")
      : finiteTimestamp(nowMs(), "nowMs()");

    if (lastLiveTimestampMs !== null) {
      const previousBucket = bucketFor(lastLiveTimestampMs, config.cadenceMs);
      const nextBucket = bucketFor(timestampMs, config.cadenceMs);
      if (nextBucket < previousBucket) {
        fail("NON_MONOTONIC_LIVE_SAMPLE", "live samples must not move to an older cadence bucket");
      }
      if (nextBucket === previousBucket) samples[samples.length - 1] = normalizedSample;
      else samples.push(normalizedSample);
    } else {
      samples.push(normalizedSample);
    }

    lastLiveTimestampMs = timestampMs;
    lastLiveReceivedAtMonotonicMs = receivedAtMonotonicMs;
    anchorTimestampMs = timestampMs;
    trimLive();
    generation += 1;
    return Object.freeze({ ...contextResult, window: readWindow() });
  }

  function rebuildReplay(nextSamples, context = {}) {
    if (!Array.isArray(nextSamples)) {
      fail("INVALID_SAMPLE", "replay history samples must be an array");
    }
    if (!hasOwn(context, "currentTimestampMs")) {
      fail("INVALID_CONTEXT", "replay history requires currentTimestampMs");
    }
    const currentTimestampMs = finiteTimestamp(context.currentTimestampMs, "currentTimestampMs");
    const contextResult = applyContext({ ...context, source: DRIVE_INSIGHTS_HISTORY_SOURCE.REPLAY });
    const cutoff = Math.max(0, currentTimestampMs - config.windowMs);
    const bucketSamples = new Map();

    nextSamples.forEach((sample, index) => {
      const normalizedSample = assertSample(sample, `samples[${index}]`);
      const timestampMs = finiteTimestamp(normalizedSample.timestampMs, `samples[${index}].timestampMs`);
      if (timestampMs < cutoff || timestampMs > currentTimestampMs) return;
      bucketSamples.set(bucketFor(timestampMs, config.cadenceMs), normalizedSample);
    });

    samples = [...bucketSamples.entries()]
      .sort(([left], [right]) => left - right)
      .map(([, sample]) => sample)
      .slice(-config.maxSamples);
    anchorTimestampMs = currentTimestampMs;
    generation += 1;
    return Object.freeze({ ...contextResult, window: readWindow() });
  }

  function readWindow() {
    const endTimestampMs = anchorTimestampMs;
    const startTimestampMs = endTimestampMs === null
      ? null
      : Math.max(0, endTimestampMs - config.windowMs);
    return Object.freeze({
      startTimestampMs,
      endTimestampMs,
      cadenceMs: config.cadenceMs,
      samples: Object.freeze([...samples]),
    });
  }

  function status(statusOptions = {}) {
    if (!statusOptions || typeof statusOptions !== "object" || Array.isArray(statusOptions)) {
      fail("INVALID_CONTEXT", "Drive Insights history status options must be an object");
    }
    let state = samples.length
      ? DRIVE_INSIGHTS_HISTORY_STATE.FRESH
      : DRIVE_INSIGHTS_HISTORY_STATE.MISSING;
    let ageMs = null;

    if (source === DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE && samples.length) {
      const statusNowMs = hasOwn(statusOptions, "nowMonotonicMs")
        ? finiteTimestamp(statusOptions.nowMonotonicMs, "nowMonotonicMs")
        : finiteTimestamp(nowMs(), "nowMs()");
      const age = lastLiveReceivedAtMonotonicMs === null
        ? Number.NaN
        : statusNowMs - lastLiveReceivedAtMonotonicMs;
      if (Number.isFinite(age) && age >= 0) ageMs = age;
      const disconnected = connectionSeen && !isConnected(connectionState);
      if (disconnected || ageMs === null || ageMs >= config.liveStaleAfterMs) {
        state = DRIVE_INSIGHTS_HISTORY_STATE.STALE;
      }
    }

    return Object.freeze({
      source,
      routeId,
      connectionState,
      state,
      stale: state === DRIVE_INSIGHTS_HISTORY_STATE.STALE,
      ageMs,
      sampleCount: samples.length,
      generation,
      lastResetReason,
    });
  }

  function reset(reason = "manual") {
    clearSamples(reason);
    return readWindow();
  }

  return Object.freeze({
    config,
    setContext: applyContext,
    pushLive,
    rebuildReplay,
    replayTimestamps(currentTimestampMs) {
      return createReplayHistoryTimestamps(currentTimestampMs, config);
    },
    window: readWindow,
    status,
    reset,
  });
}

export const DriveInsightsHistory = Object.freeze({
  SOURCE: DRIVE_INSIGHTS_HISTORY_SOURCE,
  STATE: DRIVE_INSIGHTS_HISTORY_STATE,
  DEFAULTS: DRIVE_INSIGHTS_HISTORY_DEFAULTS,
  create: createDriveInsightsHistory,
  replayTimestamps: createReplayHistoryTimestamps,
});

export function installDriveInsightsHistoryFacade(target = globalThis) {
  if (!target || (typeof target !== "object" && typeof target !== "function")) {
    throw new TypeError("Drive Insights history facade target must be an object");
  }
  target.DriveInsightsHistory = DriveInsightsHistory;
  return DriveInsightsHistory;
}
