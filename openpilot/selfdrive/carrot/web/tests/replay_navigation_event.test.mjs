import assert from "node:assert/strict";
import test from "node:test";

import {
  isUserVisibleReplayEvent,
  navigationManeuverLabel,
  replayEventDisplayTitle,
} from "../src/features/replay/navigation_event.js";

function translate(_key, fallback, values = {}) {
  return String(fallback).replace(/\{(\w+)\}/g, (_match, key) => String(values[key] ?? ""));
}

test("replay navigation hides transport lifecycle noise but keeps driver guidance", () => {
  assert.equal(isUserVisibleReplayEvent({ type: "navigation_session_changed" }), false);
  assert.equal(isUserVisibleReplayEvent({ type: "navi_connected" }), false);
  assert.equal(isUserVisibleReplayEvent({ type: "navigation_active" }), false);
  assert.equal(isUserVisibleReplayEvent({ type: "navigation_off_route" }), true);
  assert.equal(isUserVisibleReplayEvent({ type: "navigation_route_recovered" }), true);
  assert.equal(isUserVisibleReplayEvent({ type: "navigation_maneuver_current" }), true);
  assert.equal(isUserVisibleReplayEvent({ type: "lane_guidance_changed" }), true);
  assert.equal(isUserVisibleReplayEvent({ type: "speed_alert_shown" }), true);
});

test("turn codes become ordinary driver-facing maneuver labels", () => {
  assert.equal(navigationManeuverLabel({
    type: "navigation_maneuver_current",
    params: { turnType: 12 },
  }, translate), "Turn left");
  assert.equal(navigationManeuverLabel({
    type: "navigation_maneuver_next",
    params: { turnType: 13 },
  }, translate), "Next: Turn right");
  assert.equal(navigationManeuverLabel({
    type: "navigation_approach",
    params: { turnType: 13, thresholdM: 200 },
  }, translate), "Turn right in 200 m");
  assert.equal(navigationManeuverLabel({
    type: "navigation_maneuver_current",
    params: { turnType: 51 },
  }, translate), "");
});

test("navigation source is a short parenthesized suffix after the title", () => {
  assert.equal(replayEventDisplayTitle("Turn right", "CarrotNavi"), "Turn right (Navi)");
  assert.equal(replayEventDisplayTitle("Turn left", "CarrotMan"), "Turn left (Man)");
  assert.equal(replayEventDisplayTitle("Lane guidance", ""), "Lane guidance");
});
