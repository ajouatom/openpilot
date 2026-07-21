import assert from "node:assert/strict";
import test from "node:test";

import { createSettingsEntryController } from "../src/features/settings/entry_controller.js";

const CATALOG = Object.freeze({ groups: [] });

function createController(overrides = {}) {
  return createSettingsEntryController({
    commitCatalog: (catalog) => catalog,
    ...overrides,
  });
}

test("an already prepared catalog is served from memory without any load", async () => {
  let loads = 0;
  const controller = createController({
    getPreparedCatalog: () => CATALOG,
    snapshotStore: { load: () => { loads += 1; return null; } },
  });

  assert.deepEqual(await controller.prepare(), { catalog: CATALOG, source: "memory" });
  assert.equal(loads, 0);
});

test("force bypasses the prepared catalog and goes back to the snapshot", async () => {
  const controller = createController({
    getPreparedCatalog: () => CATALOG,
    snapshotStore: { load: () => ({ settings: CATALOG }) },
  });

  const result = await controller.prepare({ force: true });
  assert.equal(result.source, "snapshot");
});

test("a snapshot response is committed together with its snapshot", async () => {
  const committed = [];
  const snapshot = { settings: CATALOG, values: { A: 1 } };
  const controller = createController({
    snapshotStore: { load: () => snapshot },
    commitCatalog: (catalog, fullSnapshot) => { committed.push([catalog, fullSnapshot]); return catalog; },
  });

  const result = await controller.prepare();
  assert.equal(result.source, "snapshot");
  assert.equal(result.catalog, CATALOG);
  assert.deepEqual(committed, [[CATALOG, snapshot]]);
});

test("a failing snapshot endpoint falls through to the legacy catalog", async () => {
  let auxiliaryCalls = 0;
  const controller = createController({
    snapshotStore: { load: () => { throw new Error("404"); } },
    loadLegacyCatalog: () => CATALOG,
    loadLegacyAuxiliary: () => { auxiliaryCalls += 1; return Promise.resolve(); },
  });

  const result = await controller.prepare();
  assert.equal(result.source, "legacy");
  assert.equal(result.catalog, CATALOG);
  assert.equal(auxiliaryCalls, 1);
});

test("a snapshot without settings also falls through to legacy", async () => {
  const controller = createController({
    snapshotStore: { load: () => ({ values: {} }) },
    loadLegacyCatalog: () => CATALOG,
  });

  assert.equal((await controller.prepare()).source, "legacy");
});

test("a failing auxiliary load does not block the legacy catalog", async () => {
  const controller = createController({
    snapshotStore: { load: () => null },
    loadLegacyCatalog: () => CATALOG,
    loadLegacyAuxiliary: () => Promise.reject(new Error("aux down")),
  });

  assert.equal((await controller.prepare()).source, "legacy");
});

test("a synchronously throwing auxiliary loader is contained", async () => {
  const controller = createController({
    snapshotStore: { load: () => null },
    loadLegacyCatalog: () => CATALOG,
    loadLegacyAuxiliary: () => { throw new Error("aux exploded"); },
  });

  assert.equal((await controller.prepare()).source, "legacy");
});

test("preparation fails loudly when no catalog source is available", async () => {
  const controller = createController({ snapshotStore: { load: () => null } });
  await assert.rejects(() => controller.prepare(), /Legacy settings catalog loader is unavailable/);
});

test("a commit that yields no catalog is reported as a preparation failure", async () => {
  const controller = createController({
    snapshotStore: { load: () => ({ settings: CATALOG }) },
    commitCatalog: () => null,
  });

  await assert.rejects(() => controller.prepare(), /Settings catalog preparation failed/);
});

test("concurrent prepare calls share one run and the flag clears afterwards", async () => {
  let calls = 0;
  let release;
  const gate = new Promise((resolve) => { release = resolve; });
  const controller = createController({
    snapshotStore: {
      load: async () => { calls += 1; await gate; return { settings: CATALOG }; },
    },
  });

  const first = controller.prepare();
  const second = controller.prepare();
  assert.equal(controller.pending, true);

  release();
  await Promise.all([first, second]);

  assert.equal(calls, 1);
  assert.equal(controller.pending, false);
});

test("the pending flag clears after a rejection so a retry is possible", async () => {
  let attempt = 0;
  const controller = createController({
    snapshotStore: { load: () => null },
    loadLegacyCatalog: () => {
      attempt += 1;
      if (attempt === 1) throw new Error("network");
      return CATALOG;
    },
  });

  await assert.rejects(() => controller.prepare(), /network/);
  assert.equal(controller.pending, false);
  assert.equal((await controller.prepare()).source, "legacy");
});

test("primeSnapshot commits directly and ignores a snapshot without settings", () => {
  const controller = createController();
  assert.equal(controller.primeSnapshot({ settings: CATALOG }), CATALOG);
  assert.equal(controller.primeSnapshot({}), null);
  assert.equal(controller.primeSnapshot(null), null);
});
