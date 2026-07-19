import { normalizeDriveInsightsSnapshot, requireTimelineTimestamp } from "./schema.js";

const INPUT_KEYS = Object.freeze(["currentTimestampMs", "decodeRecord", "records", "routeId"]);
const HUD_SERVICES = new Set([
  "carState",
  "controlsState",
  "deviceState",
  "peripheralState",
  "carrotMan",
  "selfdriveState",
  "gpsLocationExternal",
  "longitudinalPlan",
  "navInstructionCarrot",
]);
const OVERLAY_SERVICES = new Set([
  "modelV2",
  "liveCalibration",
  "roadCameraState",
  "lateralPlan",
  "radarState",
  "carControl",
  "liveDelay",
  "liveTorqueParameters",
  "liveParameters",
  "liveTracks",
]);

function validateInput(input) {
  if (!input || typeof input !== "object" || Array.isArray(input)) {
    throw new TypeError("Drive Insights replay source input must be an object");
  }
  const keys = Object.keys(input).sort();
  if (keys.length !== INPUT_KEYS.length || keys.some((key, index) => key !== INPUT_KEYS[index])) {
    throw new TypeError(`Drive Insights replay source input must contain exactly: ${INPUT_KEYS.join(", ")}`);
  }
  if (!Array.isArray(input.records)) throw new TypeError("Drive Insights replay records must be an array");
  if (typeof input.decodeRecord !== "function") throw new TypeError("Drive Insights replay decodeRecord must be a function");
  requireTimelineTimestamp(input.currentTimestampMs, "currentTimestampMs");
  if (input.routeId !== null && typeof input.routeId !== "string") {
    throw new TypeError("Drive Insights replay routeId must be a string or null");
  }
  return input;
}

function validateTimeline(records) {
  let previousTimeMs = -1;
  for (let recordIndex = 0; recordIndex < records.length; recordIndex += 1) {
    const timeMs = requireTimelineTimestamp(records[recordIndex]?.timeMs, `records[${recordIndex}].timeMs`);
    if (timeMs < previousTimeMs) throw new RangeError("Drive Insights replay records must be sorted by timeMs");
    previousTimeMs = timeMs;
  }
}

function buildSeries(records, decodeRecord) {
  const series = new Map();
  for (let recordIndex = 0; recordIndex < records.length; recordIndex += 1) {
    const record = records[recordIndex];
    const timeMs = requireTimelineTimestamp(record?.timeMs, `records[${recordIndex}].timeMs`);
    let frames;
    try {
      frames = decodeRecord(record);
    } catch {
      frames = [];
    }
    if (!Array.isArray(frames)) continue;
    for (const frame of frames) {
      const service = typeof frame?.service === "string" ? frame.service : "";
      if (!service || !frame?.decoded || typeof frame.decoded !== "object") continue;
      let samples = series.get(service);
      if (!samples) {
        samples = [];
        series.set(service, samples);
      }
      const sample = Object.freeze({ timeMs, decoded: frame.decoded });
      if (samples.at(-1)?.timeMs === timeMs) samples[samples.length - 1] = sample;
      else samples.push(sample);
    }
  }
  return series;
}

function nearestSample(samples, timestampMs) {
  if (!samples?.length) return null;
  let low = 0;
  let high = samples.length;
  while (low < high) {
    const middle = (low + high) >> 1;
    if (samples[middle].timeMs <= timestampMs) low = middle + 1;
    else high = middle;
  }
  const before = low > 0 ? samples[low - 1] : null;
  const after = low < samples.length ? samples[low] : null;
  if (!before) return after;
  if (!after) return before;
  return timestampMs - before.timeMs <= after.timeMs - timestampMs ? before : after;
}

function replayContext(routeId) {
  const normalizedRouteId = typeof routeId === "string" && routeId.trim() ? routeId.trim() : null;
  return Object.freeze({ routeId: normalizedRouteId });
}

export function createDriveInsightsReplaySource(input) {
  const options = validateInput(input);
  validateTimeline(options.records);
  const context = replayContext(options.routeId);
  let series = null;
  let currentSnapshot = null;

  function indexedSeries() {
    series ||= buildSeries(options.records, options.decodeRecord);
    return series;
  }

  function snapshotAt(timestampMs) {
    const currentTimestampMs = requireTimelineTimestamp(timestampMs, "currentTimestampMs");
    const hudState = {};
    const overlayState = {};
    const serviceAges = {};
    let cameraFrameTimestampMs = null;

    for (const [service, samples] of indexedSeries()) {
      const sample = nearestSample(samples, currentTimestampMs);
      if (!sample) continue;
      serviceAges[service] = Math.abs(currentTimestampMs - sample.timeMs);
      if (HUD_SERVICES.has(service)) hudState[service] = sample.decoded;
      if (OVERLAY_SERVICES.has(service)) overlayState[service] = sample.decoded;
      if (service === "roadCameraState") cameraFrameTimestampMs = sample.timeMs;
    }

    return normalizeDriveInsightsSnapshot({
      timestampMs: currentTimestampMs,
      hudState,
      overlayState,
      serviceAges,
      cameraFrameTimestampMs,
    });
  }

  function snapshot() {
    currentSnapshot ||= snapshotAt(options.currentTimestampMs);
    return currentSnapshot;
  }

  function snapshotsAt(timestamps) {
    if (!Array.isArray(timestamps)) throw new TypeError("Drive Insights replay timestamps must be an array");
    return Object.freeze(timestamps.map((timestampMs) => snapshotAt(timestampMs)));
  }

  function sourceContext() {
    return context;
  }

  return Object.freeze({ snapshot, snapshotAt, snapshotsAt, context: sourceContext });
}

export function normalizeDriveInsightsReplaySnapshot(input) {
  return createDriveInsightsReplaySource(input).snapshot();
}

export { nearestSample as nearestDriveInsightsReplaySample };
