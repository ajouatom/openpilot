import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

import {
  createCruiseOverrideHold,
  deriveVehicleHudPayload,
  deriveCruiseOverride,
  isCruiseDisplayVisible,
  resolveLaneModeState,
  resolveCruiseKph,
  resolveTrafficState,
  vehicleHudSignature,
  withVehicleHudFields,
} from "../src/features/drive/contents/vision/hud/data_bridge.js";

const rawSource = readFileSync(new URL("../js/realtime/vision_raw.js", import.meta.url), "utf8");

// Cluster parity: carState.evModeValid & evModeActive drive the green EV telltale.
test("EV telltale requires both valid and active", () => {
  assert.equal(deriveVehicleHudPayload({ carState: { evModeValid: true, evModeActive: true } }).evActive, true);
  assert.equal(deriveVehicleHudPayload({ carState: { evModeValid: false, evModeActive: true } }).evActive, false);
  assert.equal(deriveVehicleHudPayload({ carState: { evModeValid: true, evModeActive: false } }).evActive, false);
  assert.equal(deriveVehicleHudPayload({}).evActive, false);
});

test("lane mode separates user request, planner result and final control", () => {
  assert.deepEqual(resolveLaneModeState({
    carState: { useLaneLineSpeed: 50 },
    lateralPlan: { useLaneLines: false },
    controlsState: { activeLaneLine: false },
  }), {
    requested: true,
    planned: false,
    controlled: false,
    pathActive: false,
    presentation: "armed",
  });
  assert.deepEqual(resolveLaneModeState({
    carState: { useLaneLineSpeed: 50 },
    lateralPlan: { useLaneLines: true },
    controlsState: { activeLaneLine: true },
  }), {
    requested: true,
    planned: true,
    controlled: true,
    pathActive: true,
    presentation: "active",
  });
  assert.equal(deriveVehicleHudPayload({}).activeLaneLine, null);
});

test("manual laneless request wins presentation while stale services settle", () => {
  const laneMode = resolveLaneModeState({
    carState: { useLaneLineSpeed: 0 },
    lateralPlan: { useLaneLines: true },
    controlsState: { activeLaneLine: true },
  });
  assert.equal(laneMode.presentation, "laneless");
  assert.equal(laneMode.pathActive, true, "road geometry keeps following the current planner sample");
});

test("traffic light keeps an active planner state when carrotMan carries zero", () => {
  assert.equal(resolveTrafficState({
    longitudinalPlan: { trafficState: 1 },
    carrotMan: { trafficState: 0 },
  }), 1);
  assert.equal(resolveTrafficState({
    longitudinalPlan: { trafficState: 2 },
    carrotMan: { trafficState: 0 },
  }), 2);
});

test("traffic light falls back to carrotMan and rejects unknown states", () => {
  assert.equal(resolveTrafficState({
    longitudinalPlan: { trafficState: 0 },
    carrotMan: { trafficState: 1 },
  }), 1);
  assert.equal(resolveTrafficState({ carrotMan: { trafficState: 2 } }), 2);
  assert.equal(resolveTrafficState({ longitudinalPlan: { trafficState: 9 } }), 0);
  assert.equal(deriveVehicleHudPayload({
    longitudinalPlan: { trafficState: 2 },
    carrotMan: { trafficState: 0 },
  }).trafficState, 2);
});

test("LFA activity follows carControl lateral actuation before engagement fallbacks", () => {
  assert.equal(deriveVehicleHudPayload({
    carControl: { latActive: false },
    selfdriveState: { enabled: true },
  }).lfaActive, false);
  assert.equal(deriveVehicleHudPayload({
    carControl: { latActive: true },
    selfdriveState: { enabled: false },
  }).lfaActive, true);
  assert.equal(deriveVehicleHudPayload({ selfdriveState: { enabled: true } }).lfaActive, true);
});

test("set speed skips zero and sentinel values before using cluster fallbacks", () => {
  assert.equal(resolveCruiseKph({
    carState: { vCruiseCluster: 0 },
    controlsState: { vCruiseCluster: 88 },
  }), 88);
  assert.equal(resolveCruiseKph({
    carState: { vCruiseCluster: 255, vCruise: 77 },
  }), 77);
  assert.equal(resolveCruiseKph({
    carState: { cruiseState: { available: true, speedCluster: 20 } },
  }), 72);
  assert.equal(resolveCruiseKph({
    carState: { cruiseState: { available: false, speedCluster: 20 } },
  }), null);
  assert.equal(resolveCruiseKph({
    carState: { vCruiseCluster: 88, cruiseState: { available: false } },
  }), null);
});

test("cruise display gate mirrors cluster enabled and paused states", () => {
  assert.equal(isCruiseDisplayVisible({
    carState: { vCruiseCluster: 88 },
    selfdriveState: { enabled: false },
  }), false);
  assert.equal(isCruiseDisplayVisible({
    carState: { vCruiseCluster: 88 },
    selfdriveState: { enabled: true },
  }), true);
  assert.equal(isCruiseDisplayVisible({ carState: { vCruiseCluster: 88 } }), true);
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

test("orange override survives a short live or replay sample gap", () => {
  const hold = createCruiseOverrideHold();
  const orange = { kph: 70, label: "cam:n", mode: 2 };
  assert.deepEqual(hold.update(orange, { clockMs: 1000, clockKey: "replay:a", active: true }), orange);
  assert.deepEqual(hold.update(null, { clockMs: 3499, clockKey: "replay:a", active: true }), orange);
  assert.equal(hold.update(null, { clockMs: 3501, clockKey: "replay:a", active: true }), null);
});

test("orange override hold resets on seek, source clock change, or cruise off", () => {
  const orange = { kph: 70, label: "cam:n", mode: 2 };

  const seek = createCruiseOverrideHold();
  seek.update(orange, { clockMs: 5000, clockKey: "replay:a", active: true });
  assert.equal(seek.update(null, { clockMs: 1000, clockKey: "replay:a", active: true }), null);

  const segment = createCruiseOverrideHold();
  segment.update(orange, { clockMs: 1000, clockKey: "replay:a", active: true });
  assert.equal(segment.update(null, { clockMs: 1100, clockKey: "replay:b", active: true }), null);

  const cruise = createCruiseOverrideHold();
  cruise.update(orange, { clockMs: 1000, clockKey: "live", active: true });
  assert.equal(cruise.update(null, { clockMs: 1100, clockKey: "live", active: false }), null);
});

test("eco override replaces orange immediately and is never held", () => {
  const hold = createCruiseOverrideHold();
  const orange = { kph: 70, label: "cam:n", mode: 2 };
  const eco = { kph: 95, label: "eco", mode: 1 };
  hold.update(orange, { clockMs: 1000, clockKey: "live", active: true });
  assert.deepEqual(hold.update(eco, { clockMs: 1100, clockKey: "live", active: true }), eco);
  assert.equal(hold.update(null, { clockMs: 1200, clockKey: "live", active: true }), null);
});

test("all carrot/MICI deceleration sources keep their cluster display origin", () => {
  const cases = {
    cam: "cam:n",
    section: "section:n",
    bump: "bump:n",
    police: "police:n",
    waze: "waze:n",
    road: "road:n",
    atc: "turn:n",
    atc2: "turn:n",
    hda: "cam:v",
    route: "route:v",
    gas: "gas:v",
    vturn: "turn:c",
    model: "turn:c",
  };
  for (const [source, label] of Object.entries(cases)) {
    assert.equal(deriveCruiseOverride({
      carState: { vCruiseCluster: 88 },
      carrotMan: { desiredSpeed: 70, desiredSource: source },
    }).label, label, source);
  }
});

test("cruise override is hidden while cluster cruise display is off", () => {
  assert.equal(deriveCruiseOverride({
    carState: { vCruiseCluster: 88 },
    selfdriveState: { enabled: false },
    carrotMan: { desiredSpeed: 70, desiredSource: "cam" },
  }), null);
});

test("final presentation payload retains cluster-only fields", () => {
  const payload = withVehicleHudFields(
    { vEgoKph: 52, gear: "D" },
    {
      evActive: true,
      activeLaneLine: false,
      laneModeRequested: true,
      laneModePlanned: false,
      cruiseOverride: { kph: 77, label: "cam:n", mode: 2 },
    },
  );
  assert.deepEqual(payload, {
    vEgoKph: 52,
    gear: "D",
    evActive: true,
    activeLaneLine: false,
    laneModeRequested: true,
    laneModePlanned: false,
    laneModePresentation: "armed",
    cruiseOverride: { kph: 77, label: "cam:n", mode: 2 },
    trafficState: 0,
    drivingMode: null,
  });
});

test("cluster-only changes produce distinct presentation signatures", () => {
  const base = { evActive: false, activeLaneLine: null, cruiseOverride: null };
  assert.notEqual(vehicleHudSignature(base), vehicleHudSignature({ ...base, evActive: true }));
  assert.notEqual(vehicleHudSignature(base), vehicleHudSignature({ ...base, activeLaneLine: false }));
  assert.notEqual(vehicleHudSignature(base), vehicleHudSignature({ ...base, laneModeRequested: true }));
  assert.notEqual(vehicleHudSignature(base), vehicleHudSignature({ ...base, laneModePlanned: true }));
  assert.notEqual(
    vehicleHudSignature(base),
    vehicleHudSignature({ ...base, cruiseOverride: { kph: 77, label: "cam:n", mode: 2 } }),
  );
  assert.notEqual(
    vehicleHudSignature(base),
    vehicleHudSignature({ ...base, drivingMode: 2 }),
  );
});

test("classic live and replay glue uses the shared fields and one lifecycle sink", () => {
  assert.match(rawSource, /CarrotHudDataBridge\?\.withVehicleHudFields\?\.\(basePayload,\s*j\)/);
  assert.match(rawSource, /CarrotHudDataBridge\?\.vehicleHudSignature\?\.\(payload\)/);
  assert.match(rawSource, /createCruiseOverrideHold\?\.\(\)/);
  assert.match(rawSource, /const trafficState = Number\(vehiclePayload\.trafficState\)/);
  assert.match(rawSource, /payload\.cruiseOverride = stabilizedHudCruiseOverride\(payload\.cruiseOverride,\s*payload\)/);
  assert.match(
    rawSource,
    /if \(window\.DriveVisionHudContent\?\.update\) window\.DriveVisionHudContent\.update\(payload\);[\s\S]*else window\.CarrotHudOverlay\?\.update\?\.\(payload\);/,
  );
});
