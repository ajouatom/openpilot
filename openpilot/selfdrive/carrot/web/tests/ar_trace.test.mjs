import assert from "node:assert/strict";
import test from "node:test";

import { createArTrace } from "../src/features/drive/contents/vision/ar/trace.js";

test("AR trace is disabled by default and stores no payload references", () => {
  const trace = createArTrace({ now: () => 10 });
  const payload = { nested: { value: 1 } };

  assert.equal(trace.record("frame", payload), null);
  assert.deepEqual(trace.status(), { enabled: false, capacity: 240, size: 0, dropped: 0 });

  trace.setEnabled(true);
  const entry = trace.record("frame", payload);
  payload.nested.value = 9;

  assert.equal(entry.atMs, 10);
  assert.equal(trace.snapshot().entries[0].payload.nested.value, 1);
  assert.equal(Object.isFrozen(trace.snapshot().entries[0].payload.nested), true);
});

test("AR trace is a bounded ring and reports discarded records", () => {
  const trace = createArTrace({ enabled: true, capacity: 2, now: () => 20 });
  trace.record("first", { frame: 1 });
  trace.record("second", { frame: 2 });
  trace.record("third", { frame: 3 });

  const snapshot = trace.snapshot();
  assert.deepEqual(snapshot.entries.map((entry) => entry.kind), ["second", "third"]);
  assert.deepEqual(snapshot.entries.map((entry) => entry.sequence), [2, 3]);
  assert.equal(snapshot.dropped, 1);
  assert.equal(snapshot.size, 2);

  assert.equal(trace.clear(), true);
  assert.deepEqual(trace.status(), { enabled: true, capacity: 2, size: 0, dropped: 0 });
});

test("AR trace enable and disable are explicit diagnostic operations", () => {
  const trace = createArTrace({ enabled: true });
  trace.record("active");
  assert.equal(trace.setEnabled(false), false);
  assert.equal(trace.record("ignored"), null);
  assert.equal(trace.snapshot().entries.length, 1);
  assert.equal(trace.setEnabled(true, { clear: true }), true);
  assert.equal(trace.snapshot().entries.length, 0);
});
