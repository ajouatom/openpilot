import assert from "node:assert/strict";
import test from "node:test";

import { createHudDebugFacade } from "../src/features/drive/contents/vision/hud/debug.js";

function diagnosticNode(text = "") {
  return {
    hidden: false,
    textContent: text,
    getBoundingClientRect: () => ({ x: 10, y: 20, width: 30, height: 40 }),
  };
}

test("HUD debug snapshot stays compact and separates source, derivation and display", () => {
  const nodes = {
    ".chud-t-ev": diagnosticNode("EV"),
    ".chud-lfa-lane": diagnosticNode(),
    ".chud-t-override": diagnosticNode("77"),
    ".chud-t-override-label": diagnosticNode("cam:n"),
  };
  const root = {
    ...diagnosticNode(),
    querySelector: (selector) => nodes[selector] || null,
  };
  const target = {
    CarrotHudState: {
      carState: {
        vEgoCluster: 52,
        vCruiseCluster: 88,
        evModeValid: true,
        evModeActive: true,
      },
      controlsState: { enabled: true, activeLaneLine: true },
      carrotMan: { desiredSpeed: 77, desiredSource: "cam" },
    },
    CarrotHudDataBridge: {
      deriveVehicleHudPayload: () => ({
        evActive: true,
        activeLaneLine: true,
        cruiseOverride: { kph: 77, label: "cam:n", mode: 2 },
      }),
    },
    CarrotVisionReplay: {
      status: () => ({
        active: true,
        ready: true,
        currentTime: 12.3,
        services: { carState: {}, controlsState: {}, carrotMan: {} },
      }),
    },
    DriveVisionHudContent: { status: () => ({ active: true }) },
    document: {
      getElementById: () => ({ classList: { contains: () => true } }),
    },
    getComputedStyle: () => ({ display: "block", visibility: "visible", opacity: "1" }),
  };
  const overlay = {
    root,
    status: () => ({
      active: true,
      visible: true,
      data: {
        evActive: true,
        activeLaneLine: true,
        cruiseOverride: { kph: 77, label: "cam:n", mode: 2 },
      },
    }),
  };

  const report = createHudDebugFacade(target, overlay).snapshot();
  assert.equal(report.replay.currentTime, 12.3);
  assert.deepEqual(report.replay.services, ["carState", "carrotMan", "controlsState"]);
  assert.equal(report.source.carState.evModeActive, true);
  assert.equal(report.derived.cruiseOverride.kph, 77);
  assert.equal(report.presentation.overlay.data.activeLaneLine, true);
  assert.equal(report.dom.cruiseOverride.text, "77");
  assert.ok(JSON.stringify(report).length < 5000);
});
