import { TURN_DIRECTION, TURN_FAMILY, classifyTurnType } from "../drive/contents/vision/ar/tmap_catalog.js";

const INTERNAL_NAV_EVENT_TYPES = new Set([
  "navi_connected",
  "navi_disconnected",
  "navigation_session_changed",
  "navigation_active",
  "navigation_ended",
  "navigation_maneuver_current_cleared",
  "navigation_maneuver_next_cleared",
  "lane_guidance_hidden",
  "speed_alert_cleared",
  "traffic_signal_hidden",
  "crossroad_guidance_hidden",
]);

const MANEUVER_META = Object.freeze({
  [TURN_DIRECTION.LEFT]: ["replay_event_maneuver_left", "Turn left"],
  [TURN_DIRECTION.SLIGHT_LEFT]: ["replay_event_maneuver_left", "Turn left"],
  [TURN_DIRECTION.SHARP_LEFT]: ["replay_event_maneuver_left", "Turn left"],
  [TURN_DIRECTION.RIGHT]: ["replay_event_maneuver_right", "Turn right"],
  [TURN_DIRECTION.SLIGHT_RIGHT]: ["replay_event_maneuver_right", "Turn right"],
  [TURN_DIRECTION.SHARP_RIGHT]: ["replay_event_maneuver_right", "Turn right"],
  [TURN_DIRECTION.UTURN]: ["replay_event_maneuver_uturn", "Make a U-turn"],
  [TURN_DIRECTION.STRAIGHT]: ["replay_event_maneuver_straight", "Continue straight"],
});

export function isUserVisibleReplayEvent(event) {
  return !INTERNAL_NAV_EVENT_TYPES.has(String(event?.type || ""));
}

export function replayEventSourceSuffix(sourceTag) {
  if (sourceTag === "CarrotNavi") return "Navi";
  if (sourceTag === "CarrotMan") return "Man";
  return "";
}

export function replayEventDisplayTitle(title, sourceTag) {
  const normalizedTitle = String(title || "").trim();
  const suffix = replayEventSourceSuffix(sourceTag);
  return suffix ? `${normalizedTitle} (${suffix})` : normalizedTitle;
}

export function navigationManeuverLabel(event, translate) {
  const type = String(event?.type || "");
  if (!["navigation_maneuver", "navigation_maneuver_current", "navigation_maneuver_next", "navigation_approach"].includes(type)) {
    return "";
  }
  const turnType = event?.params?.turnType ?? event?.params?.turnInfo;
  const maneuver = classifyTurnType(turnType);
  let metadata = MANEUVER_META[maneuver.direction];
  if (maneuver.family === TURN_FAMILY.ARRIVE) {
    metadata = ["replay_event_maneuver_arrive", "Arrive at destination"];
  }
  if (!metadata || maneuver.family === TURN_FAMILY.UNKNOWN || maneuver.family === TURN_FAMILY.NOTIFICATION) return "";
  const text = typeof translate === "function" ? translate : (_key, fallback) => fallback;
  const label = text(metadata[0], metadata[1]);
  if (type === "navigation_maneuver_next") {
    return text("replay_event_maneuver_next_value", "Next: {maneuver}", { maneuver: label });
  }
  if (type === "navigation_approach") {
    const distance = Math.max(0, Math.round(Number(event?.params?.thresholdM || event?.params?.distanceM) || 0));
    return distance > 0
      ? text("replay_event_maneuver_approach_value", "{maneuver} in {distance} m", { maneuver: label, distance })
      : label;
  }
  return label;
}
