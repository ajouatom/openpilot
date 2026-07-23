import assert from "node:assert/strict";
import test from "node:test";

import {
  clearRouteSummaryCache,
  getCachedRouteSummary,
  routeSummarySourceFingerprint,
  setCachedRouteSummary,
} from "../src/features/logs/route_summary/cache.js";

function source(route, overrides = {}) {
  return {
    route,
    schemaVersion: 2,
    segmentCount: 1,
    skippedSegments: 0,
    segments: [{
      segment: `${route}--0`,
      index: 0,
      kind: "qlog",
      name: "qlog.zst",
      size: 100,
      modifiedMs: 1_000,
      compression: "zstd",
      ...overrides,
    }],
  };
}

test("route summary cache reuses only an identical source manifest", () => {
  clearRouteSummaryCache();
  const original = source("route-a");
  const result = { ok: true, route: "route-a" };
  assert.equal(setCachedRouteSummary(original, result), true);
  assert.equal(getCachedRouteSummary(source("route-a")), result);
  assert.equal(getCachedRouteSummary(source("route-a", { size: 101 })), null);
  assert.equal(getCachedRouteSummary(source("route-a", { modifiedMs: 1_001 })), null);
  assert.notEqual(
    routeSummarySourceFingerprint(source("route-a")),
    routeSummarySourceFingerprint(source("route-a", { kind: "rlog", name: "rlog.zst" })),
  );
});

test("route summary cache is bounded and refreshes recent entries", () => {
  clearRouteSummaryCache();
  for (let index = 0; index < 6; index += 1) {
    setCachedRouteSummary(source(`route-${index}`), { ok: true, index });
  }
  assert.equal(getCachedRouteSummary(source("route-0"))?.index, 0);
  setCachedRouteSummary(source("route-6"), { ok: true, index: 6 });
  assert.equal(getCachedRouteSummary(source("route-1")), null);
  assert.equal(getCachedRouteSummary(source("route-0"))?.index, 0);
  assert.equal(getCachedRouteSummary(source("route-6"))?.index, 6);
});

test("route summary cache rejects incomplete keys and unsuccessful results", () => {
  clearRouteSummaryCache();
  assert.equal(routeSummarySourceFingerprint({ route: "empty", segments: [] }), "");
  assert.equal(setCachedRouteSummary(source("route-a"), { ok: false }), false);
  assert.equal(getCachedRouteSummary(source("route-a")), null);
});
