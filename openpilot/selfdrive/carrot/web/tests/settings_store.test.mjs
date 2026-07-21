import assert from "node:assert/strict";
import test from "node:test";

import {
  SETTINGS_STORE_EVENT,
  createSettingsStore,
  validateSettingsSnapshot,
} from "../src/features/settings/store.js";

const SNAPSHOT = Object.freeze({ settings: { groups: [] }, values: {} });

function createEventTarget() {
  const events = [];
  return {
    events,
    CustomEvent: class {
      constructor(type, init) {
        this.type = type;
        this.detail = init?.detail;
      }
    },
    dispatchEvent(event) { events.push(event); },
  };
}

test("validateSettingsSnapshot requires a settings payload", () => {
  assert.equal(validateSettingsSnapshot(SNAPSHOT), SNAPSHOT);
  assert.throws(() => validateSettingsSnapshot(null), /Invalid settings snapshot/);
  assert.throws(() => validateSettingsSnapshot({}), /Invalid settings snapshot/);
  assert.throws(() => validateSettingsSnapshot("x"), /Invalid settings snapshot/);
});

test("the store requires a loader function", () => {
  assert.throws(() => createSettingsStore({}), TypeError);
  assert.throws(() => createSettingsStore({ loadSnapshot: "nope" }), TypeError);
});

test("a successful load moves idle -> loading -> ready", async () => {
  const seen = [];
  const store = createSettingsStore({
    loadSnapshot: () => SNAPSHOT,
    now: () => 42,
  });
  store.subscribe((state) => seen.push(state.status));

  assert.equal(store.status, "idle");
  const snapshot = await store.load();

  assert.equal(snapshot, SNAPSHOT);
  assert.equal(store.status, "ready");
  assert.equal(store.peek(), SNAPSHOT);
  assert.deepEqual(seen, ["loading", "ready"]);
  assert.equal(store.requestedAt, 42);
  assert.equal(store.resolvedAt, 42);
});

test("a cached snapshot is returned without calling the loader again", async () => {
  let calls = 0;
  const store = createSettingsStore({
    loadSnapshot: () => { calls += 1; return SNAPSHOT; },
  });

  await store.load();
  await store.load();
  assert.equal(calls, 1);

  await store.load({ force: true });
  assert.equal(calls, 2, "force must bypass the cached snapshot");
});

test("concurrent loads share a single in-flight request", async () => {
  let calls = 0;
  let release;
  const gate = new Promise((resolve) => { release = resolve; });
  const store = createSettingsStore({
    loadSnapshot: async () => { calls += 1; await gate; return SNAPSHOT; },
  });

  const first = store.load();
  const second = store.load();
  release();
  await Promise.all([first, second]);

  assert.equal(calls, 1);
});

test("a failed load surfaces the error and stays retryable", async () => {
  let attempt = 0;
  const store = createSettingsStore({
    loadSnapshot: () => {
      attempt += 1;
      if (attempt === 1) throw new Error("boom");
      return SNAPSHOT;
    },
  });

  await assert.rejects(() => store.load(), /boom/);
  assert.equal(store.status, "error");
  assert.equal(store.getState().error, "boom");

  await store.load();
  assert.equal(store.status, "ready", "a later attempt must be able to recover");
});

test("an invalid snapshot is rejected rather than cached", async () => {
  const store = createSettingsStore({ loadSnapshot: () => ({ nope: true }) });
  await assert.rejects(() => store.load(), /Invalid settings snapshot/);
  assert.equal(store.peek(), null);
});

test("preload swallows the failure so callers never see an unhandled rejection", async () => {
  const store = createSettingsStore({ loadSnapshot: () => { throw new Error("boom"); } });
  assert.equal(await store.preload(), null);
  assert.equal(store.status, "error");
});

test("a throwing subscriber cannot break snapshot delivery", async () => {
  const store = createSettingsStore({ loadSnapshot: () => SNAPSHOT });
  const delivered = [];
  store.subscribe(() => { throw new Error("subscriber blew up"); });
  store.subscribe((state) => delivered.push(state.status));

  assert.equal(await store.load(), SNAPSHOT);
  assert.deepEqual(delivered, ["loading", "ready"]);
});

test("unsubscribe stops further notifications", async () => {
  const store = createSettingsStore({ loadSnapshot: () => SNAPSHOT });
  const seen = [];
  const unsubscribe = store.subscribe((state) => seen.push(state.status));

  unsubscribe();
  await store.load();
  assert.deepEqual(seen, []);
  assert.equal(typeof store.subscribe("not a function"), "function");
});

test("status changes are dispatched on the event target", async () => {
  const eventTarget = createEventTarget();
  const store = createSettingsStore({ loadSnapshot: () => SNAPSHOT, eventTarget });

  await store.load();
  assert.deepEqual(eventTarget.events.map((event) => event.type), [
    SETTINGS_STORE_EVENT,
    SETTINGS_STORE_EVENT,
  ]);
  assert.equal(eventTarget.events.at(-1).detail.status, "ready");
});

test("getState returns a frozen snapshot of the current status", async () => {
  const store = createSettingsStore({ loadSnapshot: () => SNAPSHOT });
  await store.load();
  const state = store.getState();

  assert.equal(Object.isFrozen(state), true);
  assert.equal(state.snapshot, SNAPSHOT);
  assert.equal(state.error, null);
});
