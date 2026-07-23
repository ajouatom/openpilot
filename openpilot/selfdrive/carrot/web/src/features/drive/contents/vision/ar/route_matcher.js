/* Ordered TMap route matching for AR world anchors.
 *
 * A nearest-segment-only match is unsafe around divided roads, parallel
 * carriageways and intersections. This module scores the vehicle against the
 * ordered route with heading, route progress and the previous accepted match.
 * It never changes the route geometry: it only selects the segment from which
 * routeDistanceAnchor() may walk forward.
 */

import { enuOffset, toVehicleFrame } from "./geo.js";

export const AR_ROUTE_MATCH_LIMITS = Object.freeze({
  // Reject crossing/opposite branches before scoring distance and progress.
  // A matcher miss does not hide the marker: geo.js keeps the nearest-segment
  // placement fallback, so this gate can stay strict without deleting guidance.
  maxHeadingErrorRad: 75 * Math.PI / 180,
  defaultStartSnapM: 45,
  minDynamicStartSnapM: 18,
  maxDynamicStartSnapM: 60,
  positionSigmaSnapGain: 2,
  minProgressWindowM: 220,
  progressWindowRatio: 0.35,
  maxBackwardM: 12,
  progressResetM: 50,
  headingPenaltyM: 14,
  progressPenaltyM: 12,
  continuityPenaltyRatio: 0.08,
  branchSwitchHysteresisM: 8,
  adjacentSegmentGrace: 1,
});

function finite(value, fallback = null) {
  if (value === null || value === undefined || value === "") return fallback;
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function wrapAngleRad(value) {
  return Math.atan2(Math.sin(value), Math.cos(value));
}

function localRoutePoints(vehicle, polyline) {
  if (!vehicle || vehicle.present === false || !Array.isArray(polyline)) return [];
  const points = [];
  for (const point of polyline) {
    const offset = enuOffset(
      vehicle.latitude,
      vehicle.longitude,
      point?.latitude,
      point?.longitude,
    );
    const local = offset && toVehicleFrame(offset.east, offset.north, vehicle.headingDeg);
    if (local) points.push(local);
  }
  return points;
}

function cumulativeRouteLengths(points) {
  const cumulative = [0];
  for (let index = 1; index < points.length; index += 1) {
    cumulative.push(cumulative[index - 1] + Math.hypot(
      points[index].x - points[index - 1].x,
      points[index].y - points[index - 1].y,
    ));
  }
  return cumulative;
}

function routeProgress(route, routeLengthM, limits) {
  const totalDistanceM = finite(route?.totalDistanceM);
  const remainingDistanceM = finite(route?.remainingDistanceM);
  const explicitMovedDistanceM = finite(route?.movedDistanceM);
  if (!(totalDistanceM > 0) || !(routeLengthM > 0)) return null;

  // movedDistanceM is not required by the compact AR contract: when omitted,
  // total - remaining gives the same monotonic route progress. A zero-filled
  // missing remaining value is not treated as arrival.
  let movedDistanceM = explicitMovedDistanceM;
  if (movedDistanceM === null && remainingDistanceM > 0) {
    movedDistanceM = totalDistanceM - remainingDistanceM;
  }
  if (movedDistanceM === null || movedDistanceM < 0) return null;
  const ratio = clamp(movedDistanceM / totalDistanceM, 0, 1);
  return Object.freeze({
    movedDistanceM,
    expectedAlongTrackM: routeLengthM * ratio,
    windowM: Math.max(limits.minProgressWindowM, routeLengthM * limits.progressWindowRatio),
  });
}

function routeSignature(route, points, sessionId) {
  const first = route?.polyline?.[0] || {};
  const middle = route?.polyline?.[Math.floor((route?.polyline?.length || 1) / 2)] || {};
  const last = route?.polyline?.at?.(-1) || {};
  const rounded = (value) => {
    const number = finite(value);
    // Roughly 10m buckets keep ordinary route-coordinate jitter from looking
    // like a reroute while still detecting a materially different branch.
    return number === null ? "?" : number.toFixed(4);
  };
  const geometry = [
    points.length,
    `${rounded(first.latitude)},${rounded(first.longitude)}`,
    `${rounded(middle.latitude)},${rounded(middle.longitude)}`,
    `${rounded(last.latitude)},${rounded(last.longitude)}`,
  ].join(":");
  return sessionId ? `session:${sessionId}:${geometry}` : `legacy:${geometry}`;
}

function startSnapLimit(positionSigmaM, limits) {
  const sigma = finite(positionSigmaM);
  if (sigma === null || sigma < 0) return limits.defaultStartSnapM;
  return clamp(
    limits.minDynamicStartSnapM + sigma * limits.positionSigmaSnapGain,
    limits.minDynamicStartSnapM,
    limits.maxDynamicStartSnapM,
  );
}

/**
 * Select the ordered route segment under the vehicle.
 *
 * The result uses `index` as the segment end index, matching geo.js. A caller
 * may pass the previous accepted result to reject backward branch jumps.
 */
export function matchRoutePosition(vehicle, route, options = {}) {
  const limits = Object.freeze({ ...AR_ROUTE_MATCH_LIMITS, ...(options.limits || {}) });
  const polyline = route?.polyline;
  const points = localRoutePoints(vehicle, polyline);
  if (points.length < 2) return null;
  const cumulative = cumulativeRouteLengths(points);
  const routeLengthM = cumulative.at(-1);
  if (!(routeLengthM > 0.01)) return null;

  const progress = routeProgress(route, routeLengthM, limits);
  const previous = options.previous || null;
  const snapLimitM = startSnapLimit(options.positionSigmaM, limits);
  const candidates = [];
  for (let index = 1; index < points.length; index += 1) {
    const a = points[index - 1];
    const b = points[index];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const length2 = dx * dx + dy * dy;
    if (!(length2 > 0.01)) continue;
    const lengthM = Math.sqrt(length2);
    const headingRad = Math.atan2(dy, dx);
    const headingErrorRad = Math.abs(wrapAngleRad(headingRad));
    if (headingErrorRad > limits.maxHeadingErrorRad) continue;

    const t = clamp(-(a.x * dx + a.y * dy) / length2, 0, 1);
    const x = a.x + dx * t;
    const y = a.y + dy * t;
    const snapDistanceM = Math.hypot(x, y);
    if (snapDistanceM > snapLimitM) continue;
    const alongTrackM = cumulative[index - 1] + lengthM * t;
    if (previous && alongTrackM < previous.alongTrackM - limits.maxBackwardM) continue;

    const progressErrorM = progress
      ? Math.abs(alongTrackM - progress.expectedAlongTrackM)
      : null;
    if (progress && progressErrorM > progress.windowM) continue;
    const predictedAlongTrackM = previous
      ? previous.alongTrackM + Math.max(
        0,
        (progress?.expectedAlongTrackM ?? previous.progressExpectedAlongTrackM ?? previous.alongTrackM)
          - (previous.progressExpectedAlongTrackM ?? previous.alongTrackM),
      )
      : null;
    const continuityErrorM = predictedAlongTrackM === null
      ? 0
      : Math.abs(alongTrackM - predictedAlongTrackM);
    const score = snapDistanceM
      + (headingErrorRad / limits.maxHeadingErrorRad) * limits.headingPenaltyM
      + (progress ? progressErrorM / progress.windowM * limits.progressPenaltyM : 0)
      + continuityErrorM * limits.continuityPenaltyRatio;
    candidates.push({
      index,
      t,
      x,
      y,
      snapDistanceM,
      alongTrackM,
      headingRad,
      headingErrorRad,
      progressErrorM,
      score,
    });
  }
  if (!candidates.length) return null;
  candidates.sort((left, right) => left.score - right.score || left.index - right.index);
  let selected = candidates[0];

  // Do not jump to a distant parallel/intersection branch for a marginal score
  // improvement while the previous ordered segment (or its neighbour) remains
  // a valid match.
  if (previous && Math.abs(selected.index - previous.index) > limits.adjacentSegmentGrace) {
    const incumbent = candidates
      .filter((candidate) => Math.abs(candidate.index - previous.index) <= limits.adjacentSegmentGrace)
      .sort((left, right) => left.score - right.score)[0];
    if (incumbent && incumbent.score <= selected.score + limits.branchSwitchHysteresisM) {
      selected = incumbent;
    }
  }

  return Object.freeze({
    ...selected,
    routeLengthM,
    routePointCount: points.length,
    maxStartSnapM: snapLimitM,
    progressExpectedAlongTrackM: progress?.expectedAlongTrackM ?? null,
    progressMovedDistanceM: progress?.movedDistanceM ?? null,
    routeSignature: routeSignature(route, points, options.sessionId),
  });
}

export function createRouteMatcher(options = {}) {
  const limits = Object.freeze({ ...AR_ROUTE_MATCH_LIMITS, ...(options.limits || {}) });
  let previous = null;
  let sessionId = null;
  let accepted = 0;
  let rejected = 0;

  function reset(nextSessionId = null) {
    previous = null;
    sessionId = nextSessionId === null ? null : String(nextSessionId || "legacy");
  }

  function match(vehicle, route, context = {}) {
    const nextSessionId = String(context.sessionId || "legacy");
    if (sessionId !== nextSessionId) reset(nextSessionId);
    let candidate = matchRoutePosition(vehicle, route, {
      limits,
      positionSigmaM: context.positionSigmaM,
      previous,
      sessionId: nextSessionId,
    });
    // A reroute may keep the same navigation session while replacing the
    // ordered polyline. Re-evaluate once without stale segment indices.
    if (previous && (!candidate || candidate.routeSignature !== previous.routeSignature)) {
      const rerouteCandidate = matchRoutePosition(vehicle, route, {
        limits,
        positionSigmaM: context.positionSigmaM,
        sessionId: nextSessionId,
      });
      if (rerouteCandidate?.routeSignature !== previous.routeSignature) {
        candidate = rerouteCandidate;
      }
    }
    if (!candidate) {
      rejected += 1;
      return null;
    }
    if (previous
        && candidate.progressExpectedAlongTrackM !== null
        && previous.progressExpectedAlongTrackM !== null
        && candidate.progressExpectedAlongTrackM
          < previous.progressExpectedAlongTrackM - limits.progressResetM) {
      previous = matchRoutePosition(vehicle, route, {
        limits,
        positionSigmaM: context.positionSigmaM,
        sessionId: nextSessionId,
      });
    } else {
      previous = candidate;
    }
    if (previous) accepted += 1;
    else rejected += 1;
    return previous;
  }

  function status() {
    return Object.freeze({
      sessionId,
      accepted,
      rejected,
      match: previous,
      limits,
    });
  }

  return Object.freeze({ match, reset, status, limits });
}
