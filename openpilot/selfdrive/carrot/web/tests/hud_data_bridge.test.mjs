import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

import {
  deriveVehicleHudPayload,
  deriveCruiseOverride,
  deriveSdiAlert,
  isCruiseDisplayVisible,
  resolveCruiseKph,
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

// Cluster parity: controlsState.activeLaneLine drives the green lane wings; unknown stays null.
test("active lane line is tri-state (true/false/null)", () => {
  assert.equal(deriveVehicleHudPayload({ controlsState: { activeLaneLine: true } }).activeLaneLine, true);
  assert.equal(deriveVehicleHudPayload({ controlsState: { activeLaneLine: false } }).activeLaneLine, false);
  assert.equal(deriveVehicleHudPayload({}).activeLaneLine, null);
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

test("SDI alert remains available before an orange cruise override begins", () => {
  const state = {
    carState: { vCruiseCluster: 80 },
    carrotMan: {
      desiredSpeed: 200,
      desiredSource: "road",
      xSpdType: 1,
      xSpdLimit: 60,
      xSpdDist: 420,
      xSpdCountDown: 8,
      szSdiDescr: "Speed camera",
    },
  };
  assert.equal(deriveCruiseOverride(state), null);
  assert.deepEqual(deriveSdiAlert(state), {
    type: 1,
    family: "camera",
    label: "Speed camera",
    speedLimitKph: 60,
    distanceM: 420,
    countdownS: 8,
  });
  assert.deepEqual(deriveVehicleHudPayload(state).sdiAlert, deriveSdiAlert(state));
});

test("SDI families and sentinel values are normalized", () => {
  for (const [type, family, label] of [
    [4, "section", "SECTION"],
    [22, "bump", "BUMP"],
    [100, "police", "POLICE"],
    [101, "waze", "WAZE"],
  ]) {
    assert.equal(deriveSdiAlert({
      carrotMan: { xSpdType: type, xSpdLimit: 50, xSpdDist: 200, xSpdCountDown: 100 },
    }).family, family);
    assert.equal(deriveSdiAlert({
      carrotMan: { xSpdType: type, xSpdLimit: 50, xSpdDist: 200, xSpdCountDown: 100 },
    }).label, label);
  }
  assert.equal(deriveSdiAlert({ carrotMan: { xSpdType: -1, xSpdLimit: 60 } }), null);
  assert.equal(deriveSdiAlert({ carrotMan: { xSpdType: 1, xSpdLimit: 0, xSpdDist: -1 } }), null);
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
      cruiseOverride: { kph: 77, label: "cam:n", mode: 2 },
    },
  );
  assert.deepEqual(payload, {
    vEgoKph: 52,
    gear: "D",
    evActive: true,
    activeLaneLine: false,
    cruiseOverride: { kph: 77, label: "cam:n", mode: 2 },
    sdiAlert: null,
  });
});

test("cluster-only changes produce distinct presentation signatures", () => {
  const base = { evActive: false, activeLaneLine: null, cruiseOverride: null, sdiAlert: null };
  assert.notEqual(vehicleHudSignature(base), vehicleHudSignature({ ...base, evActive: true }));
  assert.notEqual(vehicleHudSignature(base), vehicleHudSignature({ ...base, activeLaneLine: false }));
  assert.notEqual(
    vehicleHudSignature(base),
    vehicleHudSignature({ ...base, cruiseOverride: { kph: 77, label: "cam:n", mode: 2 } }),
  );
  assert.notEqual(
    vehicleHudSignature(base),
    vehicleHudSignature({
      ...base,
      sdiAlert: { type: 1, family: "camera", label: "CAM", speedLimitKph: 60, distanceM: 420 },
    }),
  );
});

test("classic live and replay glue uses the shared fields and one lifecycle sink", () => {
  assert.match(rawSource, /CarrotHudDataBridge\?\.withVehicleHudFields\?\.\(basePayload,\s*j\)/);
  assert.match(rawSource, /CarrotHudDataBridge\?\.vehicleHudSignature\?\.\(payload\)/);
  assert.match(
    rawSource,
    /if \(window\.DriveVisionHudContent\?\.update\) window\.DriveVisionHudContent\.update\(payload\);[\s\S]*else window\.CarrotHudOverlay\?\.update\?\.\(payload\);/,
  );
});
