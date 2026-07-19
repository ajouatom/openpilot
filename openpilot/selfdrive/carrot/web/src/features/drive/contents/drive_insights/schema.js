import { normalizeTelemetryForwardSource } from "../../../telemetry/forward_sources.js";

const RADAR_TO_CAMERA_M = 1.52;

export const DRIVE_INSIGHTS_ENTITY_SOURCES = Object.freeze([
  "vision",
  "front",
  "scc",
  "corner",
  "fusion",
  "unknown",
]);

export const DRIVE_INSIGHTS_FRESHNESS_DOMAINS = Object.freeze([
  "ego",
  "path",
  "lanes",
  "leads",
  "radar",
  "navigation",
  "camera",
]);

export const DRIVE_INSIGHTS_FRESHNESS_LIMITS = Object.freeze({
  ego: Object.freeze({ freshMs: 500, staleMs: 2_000 }),
  path: Object.freeze({ freshMs: 750, staleMs: 3_000 }),
  lanes: Object.freeze({ freshMs: 750, staleMs: 3_000 }),
  leads: Object.freeze({ freshMs: 750, staleMs: 3_000 }),
  radar: Object.freeze({ freshMs: 750, staleMs: 3_000 }),
  navigation: Object.freeze({ freshMs: 5_000, staleMs: 15_000 }),
  camera: Object.freeze({ freshMs: 1_000, staleMs: 3_000 }),
});

function frozen(value) {
  if (!value || typeof value !== "object" || Object.isFrozen(value)) return value;
  for (const child of Object.values(value)) frozen(child);
  return Object.freeze(value);
}

export function finiteOrNull(value) {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

export function unitIntervalOrNull(value) {
  const number = finiteOrNull(value);
  return number !== null && number >= 0 && number <= 1 ? number : null;
}

export function requireTimelineTimestamp(value, label = "timestampMs") {
  const number = finiteOrNull(value);
  if (number === null || number < 0) {
    throw new TypeError(`${label} must be a finite non-negative number`);
  }
  return number;
}

export function freshnessFromAge(domain, ageMs, available = true) {
  const limits = DRIVE_INSIGHTS_FRESHNESS_LIMITS[domain];
  if (!limits) throw new RangeError(`Unknown Drive Insights freshness domain: ${domain}`);
  const age = finiteOrNull(ageMs);
  if (!available || age === null || age < 0 || age > limits.staleMs) {
    return Object.freeze({ state: "missing", ageMs: null });
  }
  return Object.freeze({
    state: age <= limits.freshMs ? "fresh" : "stale",
    ageMs: age,
  });
}

function firstFinite(...values) {
  for (const value of values) {
    const number = finiteOrNull(value);
    if (number !== null) return number;
  }
  return null;
}

function firstNonEmptyString(...values) {
  for (const value of values) {
    if (typeof value !== "string") continue;
    const text = value.trim();
    if (text) return text;
  }
  return null;
}

function arrayValue(value, index = 0) {
  return Array.isArray(value) ? value[index] : value;
}

function pointSeries(data, { invertY = true } = {}) {
  const xs = Array.isArray(data?.x) ? data.x : [];
  const ys = Array.isArray(data?.y) ? data.y : [];
  const zs = Array.isArray(data?.z) ? data.z : [];
  const points = [];
  const length = Math.min(xs.length, ys.length);
  for (let index = 0; index < length; index += 1) {
    const xM = finiteOrNull(xs[index]);
    const sourceY = finiteOrNull(ys[index]);
    if (xM === null || sourceY === null) continue;
    points.push(Object.freeze({
      xM,
      yM: invertY ? -sourceY : sourceY,
      zM: finiteOrNull(zs[index]),
    }));
  }
  return Object.freeze(points);
}

function laneSide(lane, index, count) {
  const explicit = String(lane?.side || "").toLowerCase();
  if (["left", "right", "unknown"].includes(explicit)) return explicit;
  if (count === 2) return index === 0 ? "left" : "right";
  if (count >= 4) return index < Math.ceil(count / 2) ? "left" : "right";
  if (count === 3) return index === 0 ? "left" : index === 2 ? "right" : "unknown";
  return "unknown";
}

function entityId(value, source, stableIndex) {
  if ((typeof value === "string" || typeof value === "number") && String(value).trim()) {
    return String(value).trim();
  }
  return `${source}:${stableIndex}`;
}

function entitySource(value, fallback = "unknown") {
  const source = normalizeTelemetryForwardSource(value, fallback);
  return DRIVE_INSIGHTS_ENTITY_SOURCES.includes(source) ? source : fallback;
}

function normalizeLanes(model) {
  const laneLines = Array.isArray(model?.laneLines) ? model.laneLines : [];
  const probabilities = Array.isArray(model?.laneLineProbs) ? model.laneLineProbs : [];
  return Object.freeze(laneLines.flatMap((lane, index) => {
    const points = pointSeries(lane);
    if (!points.length) return [];
    return [Object.freeze({
      side: laneSide(lane, index, laneLines.length),
      probability: unitIntervalOrNull(probabilities[index] ?? lane?.probability ?? lane?.prob),
      points,
    })];
  }));
}

function normalizeModelLeads(model, egoSpeedMps) {
  const leads = Array.isArray(model?.leadsV3) ? model.leadsV3 : [];
  const modelSpeedMps = firstFinite(arrayValue(model?.velocity?.x), egoSpeedMps);
  return Object.freeze(leads.flatMap((lead, index) => {
    const sourceX = finiteOrNull(arrayValue(lead?.x));
    const sourceY = finiteOrNull(arrayValue(lead?.y));
    if (sourceX === null || sourceY === null) return [];
    const absoluteSpeed = finiteOrNull(arrayValue(lead?.v));
    return [Object.freeze({
      id: entityId(lead?.id ?? lead?.trackId, "vision", index),
      source: entitySource(lead?.source, "vision"),
      xM: sourceX - RADAR_TO_CAMERA_M,
      yM: -sourceY,
      relativeSpeedMps: absoluteSpeed !== null && modelSpeedMps !== null
        ? absoluteSpeed - modelSpeedMps
        : null,
      confidence: unitIntervalOrNull(lead?.confidence ?? lead?.probability ?? lead?.prob),
    })];
  }));
}

const RADAR_LEAD_KEYS = Object.freeze(["leadOne", "leadTwo", "leadLeft", "leadRight"]);
const RADAR_LIST_KEYS = Object.freeze([
  "leadsLeft",
  "leadsCenter",
  "leadsRight",
  "leadsLeft2",
  "leadsRight2",
  "leadsCutIn",
]);

function unpackLiveTracks(value) {
  if (Array.isArray(value)) return value;
  if (Array.isArray(value?.points)) return value.points;
  if (!Array.isArray(value?.tracks)) return [];
  const stride = Math.max(8, Math.trunc(finiteOrNull(value.stride) ?? 8));
  const tracks = [];
  for (let index = 0; index + 7 < value.tracks.length; index += stride) {
    tracks.push({
      trackId: value.tracks[index],
      dRel: value.tracks[index + 1],
      yRel: value.tracks[index + 2],
      vRel: value.tracks[index + 3],
      measured: value.tracks[index + 6],
      radarSource: value.tracks[index + 7],
    });
  }
  return tracks;
}

function normalizeRadar(overlayState) {
  const radarState = overlayState?.radarState || {};
  const candidates = [];
  for (const key of RADAR_LEAD_KEYS) {
    const lead = radarState?.[key];
    if (lead && lead.status !== false) candidates.push({ value: lead, origin: "radarState" });
  }
  for (const key of RADAR_LIST_KEYS) {
    for (const lead of Array.isArray(radarState?.[key]) ? radarState[key] : []) {
      if (lead && lead.status !== false) candidates.push({ value: lead, origin: "radarState" });
    }
  }
  for (const track of unpackLiveTracks(overlayState?.liveTracks)) {
    candidates.push({ value: track, origin: "liveTracks" });
  }

  const seen = new Set();
  const radar = [];
  for (const candidate of candidates) {
    const value = candidate.value;
    const xM = finiteOrNull(value?.xM ?? value?.dRel);
    const yM = finiteOrNull(value?.yM ?? value?.yRel);
    if (xM === null || yM === null) continue;
    const measured = typeof value?.measured === "boolean" ? value.measured : Boolean(value?.radar);
    const fallbackSource = candidate.origin === "liveTracks" ? "front" : "fusion";
    const source = entitySource(value?.entitySource ?? value?.radarSource, fallbackSource);
    const sourceId = value?.id ?? value?.trackId ?? value?.radarTrackId;
    const stableIndex = radar.length;
    const id = entityId(sourceId, source, stableIndex);
    const dedupeKey = sourceId !== undefined && sourceId !== null && String(sourceId).trim()
      ? `${source}:${String(sourceId)}`
      : `${candidate.origin}:${stableIndex}`;
    if (seen.has(dedupeKey)) continue;
    seen.add(dedupeKey);
    radar.push(Object.freeze({
      id,
      source,
      xM,
      yM,
      relativeSpeedMps: finiteOrNull(value?.relativeSpeedMps ?? value?.vRel),
      measured,
    }));
  }
  return Object.freeze(radar);
}

function nonNegativeOrNull(value) {
  const number = finiteOrNull(value);
  return number !== null && number >= 0 ? number : null;
}

function normalizeNavigation(hudState, overlayState) {
  const direct = hudState?.navigation || overlayState?.navigation || hudState?.navInstructionCarrot;
  const carrotMan = hudState?.carrotMan || overlayState?.carrotMan || {};
  const directDistance = nonNegativeOrNull(direct?.distanceM);
  const rawDistance = firstFinite(carrotMan?.xDistToTurn, carrotMan?.xSpdDist);
  const distanceM = directDistance ?? (rawDistance !== null && rawDistance >= 0 ? rawDistance : null);
  const directLimit = nonNegativeOrNull(direct?.speedLimitMps);
  const rawLimitKph = firstFinite(carrotMan?.xSpdLimit, carrotMan?.nRoadLimitSpeed);
  const speedLimitMps = directLimit ?? (rawLimitKph !== null && rawLimitKph > 0 ? rawLimitKph / 3.6 : null);
  const maneuverText = firstNonEmptyString(
    direct?.maneuverType,
    direct?.turnType,
    carrotMan?.atcType,
    carrotMan?.szTBTMainText,
  );
  const maneuverCode = finiteOrNull(carrotMan?.xTurnInfo);
  const maneuverType = maneuverText || (maneuverCode !== null && maneuverCode !== 0 ? String(maneuverCode) : null);
  const present = (distanceM !== null && distanceM > 0)
    || speedLimitMps !== null
    || maneuverType !== null
    || firstNonEmptyString(direct?.mainText, carrotMan?.naviPaths) !== null;
  if (!present) return null;
  return Object.freeze({ distanceM, speedLimitMps, maneuverType });
}

function selectPath(hudState, overlayState) {
  const model = overlayState?.modelV2 || {};
  const lateral = overlayState?.lateralPlan || {};
  const laneMode = Boolean(hudState?.controlsState?.activeLaneLine)
    || (finiteOrNull(hudState?.carState?.useLaneLineSpeed) ?? 0) > 0;
  const lateralPoints = pointSeries(lateral?.position);
  if (laneMode && lateralPoints.length > 2) {
    return { points: lateralPoints, service: "lateralPlan" };
  }
  return { points: pointSeries(model?.position), service: "modelV2" };
}

function domainAge(serviceAges, ...services) {
  const ages = services
    .map((service) => finiteOrNull(serviceAges?.[service]))
    .filter((age) => age !== null && age >= 0);
  return ages.length ? Math.max(...ages) : null;
}

function hasSource(value) {
  return Boolean(value && typeof value === "object");
}

export function normalizeDriveInsightsSnapshot({
  timestampMs,
  hudState = {},
  overlayState = {},
  serviceAges = {},
  cameraFrameTimestampMs = null,
  forceMissing = false,
} = {}) {
  const timelineTimestamp = requireTimelineTimestamp(timestampMs);
  const carState = hudState?.carState || {};
  const ego = Object.freeze({
    speedMps: finiteOrNull(carState?.vEgo),
    accelMps2: finiteOrNull(carState?.aEgo),
    steeringAngleDeg: finiteOrNull(carState?.steeringAngleDeg),
  });
  const selectedPath = selectPath(hudState, overlayState);
  const path = selectedPath.points;
  const lanes = normalizeLanes(overlayState?.modelV2);
  const leads = normalizeModelLeads(overlayState?.modelV2, ego.speedMps);
  const radar = normalizeRadar(overlayState);
  const navigation = normalizeNavigation(hudState, overlayState);
  const frameTimestamp = nonNegativeOrNull(cameraFrameTimestampMs);
  const cameraAvailable = Boolean(overlayState?.roadCameraState) && frameTimestamp !== null;
  const camera = Object.freeze({
    available: cameraAvailable,
    frameTimestampMs: cameraAvailable ? frameTimestamp : null,
  });

  // Empty arrays and null navigation are valid source results. Freshness
  // describes the source receipt, not whether that receipt contained an
  // object to draw.
  const availability = {
    ego: hasSource(hudState?.carState),
    path: hasSource(overlayState?.[selectedPath.service]),
    lanes: hasSource(overlayState?.modelV2),
    leads: hasSource(overlayState?.modelV2),
    radar: hasSource(overlayState?.radarState) || hasSource(overlayState?.liveTracks),
    navigation: hasSource(hudState?.navigation)
      || hasSource(overlayState?.navigation)
      || hasSource(hudState?.navInstructionCarrot)
      || hasSource(hudState?.carrotMan)
      || hasSource(overlayState?.carrotMan),
    camera: camera.available,
  };
  const ages = {
    ego: domainAge(serviceAges, "carState"),
    path: domainAge(serviceAges, selectedPath.service),
    lanes: domainAge(serviceAges, "modelV2"),
    leads: domainAge(serviceAges, "modelV2"),
    radar: domainAge(serviceAges, ...(overlayState?.liveTracks ? ["radarState", "liveTracks"] : ["radarState"])),
    navigation: domainAge(serviceAges, "carrotMan", "navInstructionCarrot"),
    camera: domainAge(serviceAges, "roadCameraState"),
  };
  const freshness = {};
  for (const domain of DRIVE_INSIGHTS_FRESHNESS_DOMAINS) {
    freshness[domain] = freshnessFromAge(domain, ages[domain], availability[domain] && !forceMissing);
  }

  return frozen({
    timestampMs: timelineTimestamp,
    ego,
    path,
    lanes,
    leads,
    radar,
    navigation,
    camera,
    freshness,
  });
}

export function assertDriveInsightsSnapshot(snapshot) {
  const expected = ["timestampMs", "ego", "path", "lanes", "leads", "radar", "navigation", "camera", "freshness"];
  if (!snapshot || typeof snapshot !== "object" || Object.keys(snapshot).join("|") !== expected.join("|")) {
    throw new TypeError("Drive Insights snapshot has an invalid top-level shape");
  }
  requireTimelineTimestamp(snapshot.timestampMs);
  for (const domain of DRIVE_INSIGHTS_FRESHNESS_DOMAINS) {
    const entry = snapshot.freshness?.[domain];
    if (!entry || !["fresh", "stale", "missing"].includes(entry.state)) {
      throw new TypeError(`Drive Insights freshness.${domain} is invalid`);
    }
  }
  return snapshot;
}
