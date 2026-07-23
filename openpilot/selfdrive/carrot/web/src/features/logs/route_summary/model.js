function number(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function fixed(value, digits = 1) {
  return number(value).toLocaleString(undefined, {
    minimumFractionDigits: digits,
    maximumFractionDigits: digits,
  });
}

export function formatSummaryDuration(seconds) {
  const total = Math.max(0, Math.round(number(seconds)));
  const hours = Math.floor(total / 3600);
  const minutes = Math.floor((total % 3600) / 60);
  const rest = total % 60;
  return `${String(hours).padStart(2, "0")}:${String(minutes).padStart(2, "0")}:${String(rest).padStart(2, "0")}`;
}

function formatClock(epochSeconds) {
  if (number(epochSeconds) <= 0) return "";
  return new Intl.DateTimeFormat(undefined, {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  }).format(new Date(number(epochSeconds) * 1000));
}

function formatDate(epochSeconds) {
  if (number(epochSeconds) <= 0) return "";
  return new Intl.DateTimeFormat(undefined, {
    year: "numeric",
    month: "2-digit",
    day: "2-digit",
  }).format(new Date(number(epochSeconds) * 1000));
}

function eventTimeLabel(item, firstMonoSec) {
  if (number(item?.wallTimeSec) > 0) return formatClock(item.wallTimeSec);
  const elapsed = Math.max(0, number(item?.monoSec) - number(firstMonoSec));
  const minutes = Math.floor(elapsed / 60);
  const seconds = Math.floor(elapsed % 60);
  return `+${String(minutes).padStart(2, "0")}:${String(seconds).padStart(2, "0")}`;
}

function normalizedEvent(source, firstMonoSec) {
  const items = Array.isArray(source?.items) ? source.items : [];
  return Object.freeze({
    count: Math.max(0, number(source?.count)),
    items: Object.freeze(items.map((item) => Object.freeze({
      time: eventTimeLabel(item, firstMonoSec),
      peak: item?.peak == null ? "" : fixed(item.peak, 2),
    }))),
  });
}

export function createRouteSummaryViewModel(result) {
  if (!result?.ok) throw new TypeError("Invalid route summary result");
  const time = result.time || {};
  const distance = result.distance || {};
  const extras = result.extras || {};
  const diagnostics = result.diagnostics || {};
  const totalSec = Math.max(0, number(time.totalSec));
  const autoSec = Math.max(0, number(time.autoEnabledSec));
  const manualSec = Math.max(0, number(time.manualSec));
  const ratio = (value) => totalSec > 0 ? Math.max(0, Math.min(100, value / totalSec * 100)) : 0;
  const startClock = formatClock(time.startWallSec);
  const endClock = formatClock(time.endWallSec);
  const date = formatDate(time.startWallSec);
  const firstMonoSec = number(time.firstMonoSec);
  const processed = Math.max(0, number(diagnostics.processedSegments));
  const requested = Math.max(processed, number(diagnostics.requestedSegments));
  const partial = Math.max(0, number(diagnostics.partialSegments));
  return Object.freeze({
    route: String(result.route || ""),
    hasData: Boolean(result.hasData),
    date,
    timeRange: startClock && endClock ? `${startClock}–${endClock}` : startClock,
    metrics: Object.freeze([
      Object.freeze({ key: "duration", value: formatSummaryDuration(totalSec), unit: "" }),
      Object.freeze({ key: "distance", value: fixed(number(distance.totalM) / 1000, 2), unit: "km" }),
      Object.freeze({ key: "averageSpeed", value: fixed(number(distance.averageSpeedMs) * 3.6, 1), unit: "km/h" }),
      Object.freeze({ key: "autoRatio", value: fixed(ratio(autoSec), 1), unit: "%" }),
    ]),
    distanceMetrics: Object.freeze([
      Object.freeze({ key: "totalDistance", value: fixed(number(distance.totalM) / 1000, 2), unit: "km" }),
      Object.freeze({ key: "autoDistance", value: fixed(number(distance.autoM) / 1000, 2), unit: "km" }),
      Object.freeze({ key: "manualDistance", value: fixed(number(distance.manualM) / 1000, 2), unit: "km" }),
      Object.freeze({ key: "averageSpeed", value: fixed(number(distance.averageSpeedMs) * 3.6, 1), unit: "km/h" }),
      Object.freeze({ key: "maxSpeed", value: fixed(number(distance.maxSpeedMs) * 3.6, 1), unit: "km/h" }),
    ]),
    composition: Object.freeze([
      Object.freeze({ key: "auto", seconds: autoSec, duration: formatSummaryDuration(autoSec), ratio: ratio(autoSec), emphasis: true }),
      Object.freeze({ key: "manual", seconds: manualSec, duration: formatSummaryDuration(manualSec), ratio: ratio(manualSec), emphasis: false }),
      Object.freeze({ key: "gas", seconds: number(time.manualGasSec), duration: formatSummaryDuration(time.manualGasSec), ratio: ratio(number(time.manualGasSec)), emphasis: false }),
      Object.freeze({ key: "brake", seconds: number(time.manualBrakeSec), duration: formatSummaryDuration(time.manualBrakeSec), ratio: ratio(number(time.manualBrakeSec)), emphasis: false }),
    ]),
    events: Object.freeze({
      hardAccel: normalizedEvent(result.events?.hardAccel, firstMonoSec),
      overAccel: normalizedEvent(result.events?.overAccel, firstMonoSec),
      hardDecel: normalizedEvent(result.events?.hardDecel, firstMonoSec),
      overDecel: normalizedEvent(result.events?.overDecel, firstMonoSec),
    }),
    details: Object.freeze([
      Object.freeze({ key: "stops", value: String(Math.max(0, number(extras.stopCount))), unit: "count" }),
      Object.freeze({ key: "disengagements", value: String(Math.max(0, number(extras.disengageCount))), unit: "count" }),
      Object.freeze({ key: "steerOverrides", value: String(Math.max(0, number(extras.steerOverrideCount))), unit: "count" }),
      Object.freeze({ key: "hardCorners", value: String(Math.max(0, number(extras.cornerCount))), unit: "count" }),
      Object.freeze({ key: "maxLateral", value: fixed(extras.maxLatAccel, 2), unit: "m/s²" }),
      Object.freeze({ key: "stopTime", value: formatSummaryDuration(time.stopSec), unit: "" }),
    ]),
    warnings: Object.freeze({
      fcw: Math.max(0, number(extras.warningCounts?.fcw)),
      ldw: Math.max(0, number(extras.warningCounts?.ldw)),
      driverDistracted: Math.max(0, number(extras.warningCounts?.driverDistracted)),
    }),
    metadata: Object.freeze({
      source: String(result.source || "-").toUpperCase(),
      processed,
      requested,
      partial,
      policyVersion: number(result.policyVersion, 1),
      elapsedMs: Math.max(0, number(diagnostics.elapsedMs)),
    }),
  });
}
