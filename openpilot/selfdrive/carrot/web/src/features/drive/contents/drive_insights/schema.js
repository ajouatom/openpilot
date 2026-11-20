import { normalizeTelemetryForwardSource } from "../../../telemetry/forward_sources.js";

const RADAR_TO_CAMERA_M = 1.52;
// Matches the replay forward view, so the same drive shows the same marks.
const MODEL_LEAD_MIN_CONFIDENCE = 0.5;

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
    const confidence = unitIntervalOrNull(lead?.confidence ?? lead?.probability ?? lead?.prob);
    // leadsV3 always carries its full slot count, so unconfident entries are
    // padding rather than targets. Drawing them puts ghosts on the view.
    if (confidence !== null && confidence < MODEL_LEAD_MIN_CONFIDENCE) return [];
    const absoluteSpeed = finiteOrNull(arrayValue(lead?.v));
    return [Object.freeze({
      id: entityId(lead?.id ?? lead?.trackId, "vision", index),
      source: entitySource(lead?.source, "vision"),
      xM: sourceX - RADAR_TO_CAMERA_M,
      yM: -sourceY,
      relativeSpeedMps: absoluteSpeed !== null && modelSpeedMps !== null
        ? absoluteSpeed - modelSpeedMps
        : null,
      confidence,
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

// Mode-5 liveTracks points carry the same raw 0x238 values as their display
// point; the two services may be one 50 ms scan apart, hence the range margin.
const CLASSIC_MATCH_MAX_DX_M = 1.0;
const CLASSIC_MATCH_MAX_DY_M = 0.4;
const CLASSIC_MAX_OBJECT_ID = 63;

// Label: the radar's object ID plus the object's ground speed in km/h
// (ego speed + vRel; the compact wire has no vLead).
function classicObjectLabel(track, egoSpeedMps) {
  const objectId = finiteOrNull(track?.trackId);
  const idText = objectId !== null && objectId >= 0 && objectId <= CLASSIC_MAX_OBJECT_ID
    ? String(Math.trunc(objectId))
    : null;
  const vLead = finiteOrNull(track?.vLead);
  const vRel = finiteOrNull(track?.vRel);
  const speed = vLead ?? (vRel !== null && egoSpeedMps !== null ? egoSpeedMps + vRel : null);
  if (speed === null) return idText;
  const kmh = Math.round(speed * 3.6);
  return `${idText ?? "-"} ${Object.is(kmh, -0) ? 0 : kmh}`;
}

function radarTrackKey(value) {
  const trackId = finiteOrNull(
    typeof value === "number" ? value : Number.parseFloat(String(value ?? "")),
  );
  return trackId !== null && trackId >= 0 ? String(trackId) : null;
}

// radarState leads are fused products of the very tracks liveTracks reports, so
// a lead and its originating track describe one vehicle. Match them on track id
// the way the replay view does; otherwise the same car draws twice - once as a
// "fusion" lead and once under its real radar source.
function normalizeRadar(overlayState, egoSpeedMps = null) {
  const radarState = overlayState?.radarState || {};
  const leads = [];
  const appendLead = (lead) => {
    if (!lead || lead.status === false) return;
    const xM = finiteOrNull(lead?.xM ?? lead?.dRel);
    const yM = finiteOrNull(lead?.yM ?? lead?.yRel);
    if (xM === null || yM === null) return;
    leads.push({ value: lead, xM, yM, trackKey: radarTrackKey(lead?.radarTrackId) });
  };
  for (const key of RADAR_LEAD_KEYS) appendLead(radarState?.[key]);
  for (const key of RADAR_LIST_KEYS) {
    for (const lead of Array.isArray(radarState?.[key]) ? radarState[key] : []) appendLead(lead);
  }

  const leadTrackKeys = new Set(leads.map(({ trackKey }) => trackKey).filter(Boolean));
  const matchedTrackKeys = new Set();
  const seen = new Set();
  const radar = [];

  const push = (entry) => {
    if (seen.has(entry.dedupeKey)) return;
    seen.add(entry.dedupeKey);
    radar.push(Object.freeze({
      id: entry.id,
      source: entry.source,
      xM: entry.xM,
      yM: entry.yM,
      relativeSpeedMps: entry.relativeSpeedMps,
      measured: entry.measured,
      selected: entry.selected,
      ...(entry.label ? { label: entry.label } : {}),
    }));
  };

  // 0x238 계열 점의 trackId는 레이더 자체 객체 ID(0-63)이다. K7 EnableRadarTracks=1의
  // liveTracks 점은 같은 스캔의 같은 좌표로 나오므로, 그 점에 객체 ID를 붙이고
  // 표시 전용 점은 한 번만 그린다.
  const classicTracks = [];
  for (const track of unpackLiveTracks(overlayState?.classicRadarTracks)) {
    const xM = finiteOrNull(track?.xM ?? track?.dRel);
    const yM = finiteOrNull(track?.yM ?? track?.yRel);
    if (xM === null || yM === null) continue;
    classicTracks.push({ track, xM, yM, used: false, label: classicObjectLabel(track, egoSpeedMps) });
  }
  const takeClassicLabel = (xM, yM) => {
    let best = null;
    for (const classic of classicTracks) {
      if (classic.used) continue;
      const dx = Math.abs(classic.xM - xM);
      const dy = Math.abs(classic.yM - yM);
      if (dx > CLASSIC_MATCH_MAX_DX_M || dy > CLASSIC_MATCH_MAX_DY_M) continue;
      if (!best || dx + dy < best.cost) best = { classic, cost: dx + dy };
    }
    if (!best) return null;
    best.classic.used = true;
    return best.classic.label;
  };

  for (const track of unpackLiveTracks(overlayState?.liveTracks)) {
    const xM = finiteOrNull(track?.xM ?? track?.dRel);
    const yM = finiteOrNull(track?.yM ?? track?.yRel);
    if (xM === null || yM === null) continue;
    const trackKey = radarTrackKey(track?.trackId);
    const selected = trackKey !== null && leadTrackKeys.has(trackKey);
    if (selected) matchedTrackKeys.add(trackKey);
    const source = entitySource(track?.entitySource ?? track?.radarSource, "front");
    const stableIndex = radar.length;
    push({
      id: entityId(track?.id ?? track?.trackId, source, stableIndex),
      source,
      xM,
      yM,
      relativeSpeedMps: finiteOrNull(track?.relativeSpeedMps ?? track?.vRel),
      measured: typeof track?.measured === "boolean" ? track.measured : Boolean(track?.radar),
      selected,
      label: source === "front" ? takeClassicLabel(xM, yM) : null,
      dedupeKey: trackKey !== null ? `track:${trackKey}` : `liveTracks:${stableIndex}`,
    });
  }

  // 제어용 liveTracks와 겹치지 않는 0x238 점은 표시 전용 점으로 추가한다.
  for (const classic of classicTracks) {
    if (classic.used) continue;
    const { track, xM, yM } = classic;
    const trackId = track?.id ?? track?.trackId ?? radar.length;
    push({
      id: `classic238:${trackId}`,
      source: "front",
      xM,
      yM,
      relativeSpeedMps: finiteOrNull(track?.relativeSpeedMps ?? track?.vRel),
      measured: typeof track?.measured === "boolean" ? track.measured : true,
      selected: false,
      label: classic.label,
      dedupeKey: `classic238:${trackId}`,
    });
  }

  // Only leads with no matching track need their own mark; a matched lead was
  // already drawn above and keeps its real radar source colour.
  for (const lead of leads) {
    if (lead.trackKey !== null && matchedTrackKeys.has(lead.trackKey)) continue;
    const value = lead.value;
    const source = entitySource(value?.entitySource ?? value?.radarSource, "fusion");
    const stableIndex = radar.length;
    push({
      id: entityId(value?.id ?? value?.radarTrackId, source, stableIndex),
      source,
      xM: lead.xM,
      yM: lead.yM,
      relativeSpeedMps: finiteOrNull(value?.relativeSpeedMps ?? value?.vRel),
      measured: typeof value?.measured === "boolean" ? value.measured : Boolean(value?.radar),
      selected: true,
      dedupeKey: lead.trackKey !== null
        ? `lead:${lead.trackKey}`
        : `lead:${lead.xM.toFixed(1)}:${lead.yM.toFixed(1)}`,
    });
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
  const radar = normalizeRadar(overlayState, ego.speedMps);
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
    radar: hasSource(overlayState?.radarState)
      || hasSource(overlayState?.liveTracks)
      || hasSource(overlayState?.classicRadarTracks),
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
    radar: domainAge(serviceAges, "radarState", "liveTracks", "classicRadarTracks"),
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
