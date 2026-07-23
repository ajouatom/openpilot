/* AR marker identity and lifecycle channels.
 *
 * Navi publishes snapshots, not durable event IDs. Distance and label text can
 * change on every publish, so neither may own a Three object or a world anchor.
 * This module provides the one shared identity contract used by composition,
 * anchor storage, and rendering.
 */

const PRIMARY_GUIDANCE = "guidance:primary";
const NEXT_GUIDANCE = "guidance:next";
const COMMIT_GUIDANCE = "guidance:commit";
const METERS_PER_LATITUDE_DEGREE = 111_320;

export const AR_MARKER_IDENTITY_LIMITS = Object.freeze({
  pointToleranceM: 12,
  pointClusterM: 8,
  forwardResetM: 35,
  promotionDistanceToleranceM: 120,
  missingGraceTicks: 4,
});

function finite(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function sourceOf(marker) {
  return String(marker?.source || "unknown");
}

function descriptorOf(marker) {
  return marker?.descriptor || marker || {};
}

function kindOf(marker) {
  return String(marker?.kind || descriptorOf(marker).kind || "unknown");
}

function pointMeters(point) {
  const latitude = finite(point?.latitude);
  const longitude = finite(point?.longitude);
  if (point?.pointValid !== true || latitude === null || longitude === null) return null;
  const latitudeRadians = latitude * Math.PI / 180;
  return Object.freeze({
    north: latitude * METERS_PER_LATITUDE_DEGREE,
    east: longitude * METERS_PER_LATITUDE_DEGREE * Math.cos(latitudeRadians),
  });
}

function pointIdentity(point, clusterM = AR_MARKER_IDENTITY_LIMITS.pointClusterM) {
  const meters = pointMeters(point);
  if (!meters) return "";
  const size = Math.max(1, finite(clusterM) || AR_MARKER_IDENTITY_LIMITS.pointClusterM);
  return `${Math.round(meters.north / size)},${Math.round(meters.east / size)}`;
}

function pointDistanceM(a, b) {
  if (!a || !b) return null;
  return Math.hypot(a.north - b.north, a.east - b.east);
}

function semanticOf(marker) {
  const descriptor = descriptorOf(marker);
  return String(
    marker?.turn?.code
    ?? descriptor?.turnCode
    ?? marker?.sdiCode
    ?? marker?.sdiFamily
    ?? descriptor?.sdiFamily
    ?? marker?.imageCode
    ?? descriptor?.imageCode
    ?? marker?.speedLimitKph
    ?? descriptor?.speedLimitKph
    ?? marker?.limitKph
    ?? descriptor?.limitKph
    ?? marker?.turnCode
    ?? "",
  );
}

function providedEventId(marker) {
  for (const value of [marker?.stableEventId, marker?.navigationEventId, marker?.eventId]) {
    const text = String(value ?? "").trim();
    if (text) return text;
  }
  return "";
}

function isGuidanceSource(source) {
  return source === "guidanceCurrent" || source === "guidanceNext";
}

function identityCandidate(marker) {
  const source = sourceOf(marker);
  return Object.freeze({
    source,
    slot: markerLifecycleSlot(marker),
    kind: kindOf(marker),
    semantic: semanticOf(marker),
    providedId: providedEventId(marker),
    point: pointMeters(marker?.point),
    distanceM: finite(marker?.distanceM),
    guidance: isGuidanceSource(source),
  });
}

/** One visual slot can contain only one event at a time. */
export function markerLifecycleSlot(marker = {}) {
  const source = sourceOf(marker);
  const kind = kindOf(marker);
  if (source === "calibrationProbe") return PRIMARY_GUIDANCE;
  if (source === "guidanceCurrent") {
    return kind === "commit_arrow" ? COMMIT_GUIDANCE : PRIMARY_GUIDANCE;
  }
  if (source === "guidanceNext") return NEXT_GUIDANCE;
  return `${source}:${kind}`;
}

/**
 * Stable identity for one semantic Navi event.
 *
 * Distance, phase, and label are intentionally excluded. They are mutable
 * presentation data and must repaint the existing object instead of creating
 * another anchor. A valid Navi point differentiates consecutive events with
 * the same turn/code whenever that information is available.
 */
export function markerIdentity(marker = {}) {
  const source = sourceOf(marker);
  const point = pointIdentity(marker?.point);
  const semantic = semanticOf(marker);
  // TMap commonly publishes one maneuver first as guidanceNext and then
  // promotes that exact coordinate to guidanceCurrent. Source is presentation
  // priority, not event identity: keeping it in the key recreates the Three
  // object and world anchor at the hand-off, which looks like a visible jump.
  if (point && (source === "guidanceCurrent" || source === "guidanceNext")) {
    return ["guidance:event", kindOf(marker), point, String(semantic)].join("|");
  }
  return [
    markerLifecycleSlot(marker),
    source,
    kindOf(marker),
    point,
    String(semantic),
  ].join("|");
}

function matchScore(record, candidate, limits) {
  if (record.kind !== candidate.kind) return null;
  if (record.providedId || candidate.providedId) {
    return record.providedId && record.providedId === candidate.providedId ? -1_000 : null;
  }
  const guidancePair = record.guidance && candidate.guidance;
  if (!guidancePair && record.slot !== candidate.slot) return null;

  const pointDeltaM = pointDistanceM(record.point, candidate.point);
  if (pointDeltaM !== null) {
    if (pointDeltaM > limits.pointToleranceM) return null;
    // A physical maneuver point owns the occurrence. A refined turn code or
    // current/next promotion only repaints that same world object.
    const semanticPenalty = record.semantic === candidate.semantic ? 0 : 4;
    const sourcePenalty = record.source === candidate.source ? 0 : 1;
    return pointDeltaM + semanticPenalty + sourcePenalty;
  }

  if (record.semantic !== candidate.semantic) return null;
  const distanceDelta = record.distanceM === null || candidate.distanceM === null
    ? 0
    : candidate.distanceM - record.distanceM;
  if (record.slot === candidate.slot) {
    if (distanceDelta > limits.forwardResetM) return null;
    return 100 + Math.abs(distanceDelta);
  }
  // Coordinate-less guidance can move from next to current. Match only the
  // promotion direction and only while its remaining-distance estimate is
  // still close enough to be the same upcoming maneuver.
  if (
    guidancePair
    && record.source === "guidanceNext"
    && candidate.source === "guidanceCurrent"
    && Math.abs(distanceDelta) <= limits.promotionDistanceToleranceM
  ) return 200 + Math.abs(distanceDelta);
  return null;
}

/**
 * Stateful occurrence matcher for Navi snapshots that do not contain durable
 * per-event IDs. The returned marker ID never contains distance, text, phase,
 * opacity, or raw centimetre-sensitive coordinates.
 */
export function createMarkerIdentityTracker(options = {}) {
  const limits = Object.freeze({
    ...AR_MARKER_IDENTITY_LIMITS,
    ...(options.limits || {}),
  });
  let sessionId = null;
  let serial = 0;
  let tick = 0;
  let records = new Map();

  function reset(nextSessionId = null) {
    sessionId = nextSessionId === null ? null : String(nextSessionId || "legacy");
    serial = 0;
    tick = 0;
    records = new Map();
  }

  function assign(markers = [], context = {}) {
    const nextSessionId = String(context.sessionId || "legacy");
    if (sessionId !== nextSessionId) reset(nextSessionId);
    tick += 1;
    const used = new Set();
    const ids = [];

    for (const marker of Array.isArray(markers) ? markers : []) {
      const candidate = identityCandidate(marker);
      let best = null;
      let bestScore = Infinity;
      for (const record of records.values()) {
        if (used.has(record.id)) continue;
        const score = matchScore(record, candidate, limits);
        if (score === null || score >= bestScore) continue;
        best = record;
        bestScore = score;
      }

      const deterministicProvidedId = candidate.providedId
        ? `ar-event|${nextSessionId}|${candidate.kind}|provided:${candidate.providedId}`
        : "";
      const id = best?.id || deterministicProvidedId
        || `ar-event|${nextSessionId}|${candidate.kind}|${++serial}`;
      records.set(id, Object.freeze({ ...candidate, id, seenTick: tick }));
      used.add(id);
      ids.push(id);
    }

    for (const [id, record] of records) {
      if (tick - record.seenTick > limits.missingGraceTicks) records.delete(id);
    }
    return Object.freeze(ids);
  }

  function status() {
    return Object.freeze({ sessionId, occurrenceCount: serial, tracked: records.size, tick, limits });
  }

  return Object.freeze({ assign, reset, status, limits });
}
