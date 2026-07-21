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

function pointIdentity(point) {
  const latitude = finite(point?.latitude);
  const longitude = finite(point?.longitude);
  if (point?.pointValid !== true || latitude === null || longitude === null) return "";
  return `${latitude.toFixed(7)},${longitude.toFixed(7)}`;
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
  const descriptor = descriptorOf(marker);
  const source = sourceOf(marker);
  const point = pointIdentity(marker?.point);
  const semantic = marker?.turn?.code
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
    ?? "";
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
