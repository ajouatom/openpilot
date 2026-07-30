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
    querySelectorAll: (selector) => (
      selector === ".chud-speed-traffic"
        ? [diagnosticNode(), { ...diagnosticNode(), hidden: true }]
        : []
    ),
  };
  const target = {
    CarrotHudState: {
      carState: {
        vEgoCluster: 52,
        vCruiseCluster: 88,
        evModeValid: true,
        evModeActive: true,
        useLaneLineSpeed: 50,
      },
      controlsState: { enabled: true, vCruiseCluster: 87, activeLaneLine: true },
      lateralPlan: { useLaneLines: true },
      longitudinalPlan: { trafficState: 2 },
      carControl: { latActive: true },
      carrotMan: {
        desiredSpeed: 77,
        desiredSource: "cam",
        trafficState: 0,
        xSpdType: 1,
        xSpdLimit: 60,
        xSpdDist: 420,
        xSpdCountDown: 8,
      },
    },
    CarrotHudDataBridge: {
      deriveVehicleHudPayload: () => ({
        evActive: true,
        activeLaneLine: true,
        trafficState: 2,
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
  assert.equal(report.source.carState.useLaneLineSpeed, 50);
  assert.equal(report.source.controlsState.vCruiseCluster, 87);
  assert.equal(report.source.lateralPlan.useLaneLines, true);
  assert.equal(report.source.carControl.latActive, true);
  assert.equal(report.source.carrotMan.xSpdDist, 420);
  assert.equal(report.source.carrotMan.trafficState, 0);
  assert.equal(report.source.longitudinalPlan.trafficState, 2);
  assert.equal(report.derived.trafficState, 2);
  assert.equal(report.derived.cruiseOverride.kph, 77);
  assert.equal(report.presentation.overlay.data.activeLaneLine, true);
  assert.equal(report.dom.cruiseOverride.text, "77");
  assert.equal(report.dom.trafficRed.present, true);
  assert.equal(report.dom.trafficGreen.hidden, true);
  assert.ok(JSON.stringify(report).length < 5000);
});
