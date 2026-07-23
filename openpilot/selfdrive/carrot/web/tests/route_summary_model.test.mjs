import assert from "node:assert/strict";
import test from "node:test";

import { createRouteSummaryViewModel, formatSummaryDuration } from "../src/features/logs/route_summary/model.js";
import { compositionDistributionSegments } from "../src/features/logs/route_summary/components.js";

test("route summary model formats a numeric worker result without presentation markup", () => {
  const model = createRouteSummaryViewModel({
    ok: true,
    route: "route-a",
    source: "rlog",
    hasData: true,
    policyVersion: 2,
    time: {
      totalSec: 600,
      autoEnabledSec: 450,
      manualSec: 150,
      manualGasSec: 30,
      manualBrakeSec: 15,
      firstMonoSec: 10,
      startWallSec: 1_720_000_000,
      endWallSec: 1_720_000_600,
    },
    distance: { totalM: 12500, autoM: 10000, manualM: 2500, averageSpeedMs: 12.5, maxSpeedMs: 25 },
    events: { hardAccel: { count: 1, items: [{ monoSec: 25, peak: 3.2 }] } },
    extras: { stopCount: 2, warningCounts: { fcw: 1 } },
    diagnostics: { requestedSegments: 3, processedSegments: 2, partialSegments: 1, elapsedMs: 50 },
  });
  assert.equal(model.route, "route-a");
  assert.ok(model.date);
  assert.match(model.timeRange, /\d{2}:\d{2}:\d{2}.+\d{2}:\d{2}:\d{2}/);
  assert.equal("subtitle" in model, false);
  assert.equal(model.metrics[0].value, "00:10:00");
  assert.equal(model.metrics[1].unit, "km");
  assert.equal(model.distanceMetrics[0].value, "12.50");
  assert.equal(model.distanceMetrics[1].value, "10.00");
  assert.equal(model.distanceMetrics[2].value, "2.50");
  assert.equal(model.distanceMetrics[4].value, "90.0");
  assert.equal(model.composition[0].ratio, 75);
  assert.equal(model.events.hardAccel.items[0].time, "+00:15");
  assert.deepEqual(model.metadata, {
    source: "RLOG",
    processed: 2,
    requested: 3,
    partial: 1,
    policyVersion: 2,
    elapsedMs: 50,
  });
  assert.equal(Object.isFrozen(model), true);
});

test("duration formatting is bounded and stable", () => {
  assert.equal(formatSummaryDuration(-10), "00:00:00");
  assert.equal(formatSummaryDuration(3661.4), "01:01:01");
});

test("single composition bar nests pedal time inside the manual interval", () => {
  assert.deepEqual(compositionDistributionSegments([
    { key: "auto", seconds: 60 },
    { key: "manual", seconds: 40 },
    { key: "gas", seconds: 10 },
    { key: "brake", seconds: 5 },
  ]), { auto: 60, manual: 40, gas: 10, brake: 5, other: 25 });

  assert.deepEqual(compositionDistributionSegments([
    { key: "auto", seconds: 60 },
    { key: "manual", seconds: 40 },
    { key: "gas", seconds: 20 },
    { key: "brake", seconds: 30 },
  ]), { auto: 60, manual: 40, gas: 16, brake: 24, other: 0 });
});
