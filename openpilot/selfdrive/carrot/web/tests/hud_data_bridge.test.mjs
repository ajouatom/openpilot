import assert from "node:assert/strict";
import test from "node:test";

import {
  deriveVehicleHudPayload,
  deriveCruiseOverride,
} from "../src/features/drive/contents/vision/hud/data_bridge.js";

// Cluster parity: carState.evModeValid & evModeActive drive the green EV telltale.
test("EV telltale requires both valid and active", () => {
  assert.equal(deriveVehicleHudPayload({ carState: { evModeValid: true, evModeActive: true } }).evActive, true);
  assert.equal(deriveVehicleHudPayload({ carState: { evModeValid: false, evModeActive: true } }).evActive, false);
  assert.equal(deriveVehicleHudPayload({ carState: { evModeValid: true, evModeActive: false } }).evActive, false);
  assert.equal(deriveVehicleHudPayload({}).evActive, false);
});

// Cluster parity: controlsState.activeLaneLine drives the green lane wings; unknown stays null.
test("active lane line is tri-state (true/false/null)", () => {
  assert.equal(deriveVehicleHudPayload({ controlsState: { activeLaneLine: true } }).activeLaneLine, true);
  assert.equal(deriveVehicleHudPayload({ controlsState: { activeLaneLine: false } }).activeLaneLine, false);
  assert.equal(deriveVehicleHudPayload({}).activeLaneLine, null);
});

test("cruise override is null when cruise is off or sentinel", () => {
  assert.equal(deriveCruiseOverride({ carState: { vCruiseCluster: 0 }, carrotMan: { desiredSpeed: 40 } }), null);
  assert.equal(deriveCruiseOverride({ carState: { vCruiseCluster: 255 }, carrotMan: { desiredSpeed: 40 } }), null);
  assert.equal(deriveCruiseOverride({ carrotMan: { desiredSpeed: 40 } }), null);
});

test("decel override (orange, mode 2) maps the source label", () => {
  const out = deriveCruiseOverride({
    carState: { vCruiseCluster: 88 },
    carrotMan: { desiredSpeed: 77, desiredSource: "cam" },
  });
  assert.deepEqual(out, { kph: 77, label: "cam:n", mode: 2 });
});

test("decel override falls back to 'apply' and truncates unknown sources", () => {
  assert.equal(deriveCruiseOverride({ carState: { vCruiseCluster: 88 }, carrotMan: { desiredSpeed: 70 } }).label, "apply");
  assert.equal(
    deriveCruiseOverride({ carState: { vCruiseCluster: 88 }, carrotMan: { desiredSpeed: 70, desiredSource: "unknownsource" } }).label,
    "unknowns", // cluster: normalized[:8]
  );
  // Already-suffixed sources pass through verbatim.
  assert.equal(
    deriveCruiseOverride({ carState: { vCruiseCluster: 88 }, carrotMan: { desiredSpeed: 70, desiredSource: "route:v" } }).label,
    "route:v",
  );
});

test("eco override (green, mode 1) wins when cruiseTarget exceeds the set speed", () => {
  const out = deriveCruiseOverride({
    carState: { vCruiseCluster: 60 },
    longitudinalPlan: { cruiseTarget: 72 },
    carrotMan: { desiredSpeed: 50 },
  });
  assert.deepEqual(out, { kph: 72, label: "eco", mode: 1 });
});

test("no override when desiredSpeed is not below the set speed", () => {
  assert.equal(deriveCruiseOverride({ carState: { vCruiseCluster: 88 }, carrotMan: { desiredSpeed: 88 } }), null);
  assert.equal(deriveCruiseOverride({ carState: { vCruiseCluster: 88 }, carrotMan: { desiredSpeed: 95 } }), null);
});
