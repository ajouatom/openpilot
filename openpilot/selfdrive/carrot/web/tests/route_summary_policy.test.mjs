import assert from "node:assert/strict";
import test from "node:test";

import { createRouteSummaryAccumulator } from "../src/features/logs/route_summary/policy.js";

function event(service, seconds, decoded) {
  return { service, logMonoTime: seconds * 1e9, decoded };
}

test("route summary policy aggregates driving data and event edges", () => {
  const summary = createRouteSummaryAccumulator({ route: "route-a", source: "rlog" });
  summary.startSegment();
  summary.ingest(event("initData", 1, { wallTimeNanos: 1700000000 * 1e9 }));
  summary.ingest(event("selfdriveState", 1.1, { enabled: true, active: true }));
  summary.ingest(event("onroadEvents", 1.15, [{ name: 65 }]));
  summary.ingest(event("onroadEvents", 1.16, [{ name: 65 }]));
  summary.ingest(event("carState", 1.2, {
    vEgo: 10,
    aEgo: 3,
    yawRate: 0.4,
    gearShifter: "drive",
  }));
  summary.ingest(event("carState", 1.4, {
    vEgo: 10,
    aEgo: 3,
    yawRate: 0.4,
    gearShifter: "drive",
  }));
  summary.ingest(event("selfdriveState", 1.5, { enabled: false, active: false }));
  summary.ingest(event("carState", 1.6, {
    vEgo: 5,
    aEgo: 0,
    yawRate: 0,
    gearShifter: "drive",
  }));

  const result = summary.finish({ requestedSegments: 1, processedSegments: 1 });
  assert.equal(result.ok, true);
  assert.equal(result.route, "route-a");
  assert.equal(result.source, "rlog");
  assert.equal(result.hasData, true);
  assert.ok(Math.abs(result.time.totalSec - 0.4) < 1e-6);
  assert.ok(Math.abs(result.distance.totalM - 3) < 1e-6);
  assert.ok(Math.abs(result.distance.autoM - 2) < 1e-6);
  assert.equal(result.events.hardAccel.count, 1);
  assert.equal(result.extras.cornerCount, 1);
  assert.equal(result.extras.disengageCount, 1);
  assert.equal(result.extras.warningCounts.fcw, 1);
});

test("segment boundaries never integrate the gap between log files", () => {
  const summary = createRouteSummaryAccumulator({ source: "qlog" });
  summary.startSegment();
  summary.ingest(event("carState", 10, { vEgo: 4, aEgo: 0, gearShifter: "drive" }));
  summary.ingest(event("carState", 10.5, { vEgo: 4, aEgo: 0, gearShifter: "drive" }));
  summary.startSegment();
  summary.ingest(event("carState", 100, { vEgo: 4, aEgo: 0, gearShifter: "drive" }));
  const result = summary.finish({ requestedSegments: 2, processedSegments: 2 });
  assert.ok(Math.abs(result.time.totalSec - 0.5) < 1e-6);
  assert.ok(Math.abs(result.distance.totalM - 2) < 1e-6);
});
