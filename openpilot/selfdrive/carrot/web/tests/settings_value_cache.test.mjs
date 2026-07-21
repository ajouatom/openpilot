import assert from "node:assert/strict";
import test from "node:test";

import {
  DEFAULT_SETTING_VALUES_TTL_MS,
  createSettingValueCache,
} from "../src/features/settings/value_cache.js";

// isFresh() requires loadedAt > 0, so the fake clock never starts at zero.
function createClock(start = 1_000) {
  let current = start;
  return {
    now: () => current,
    advance(ms) { current += ms; },
  };
}

test("peekValues reports incomplete until every name is cached", () => {
  const cache = createSettingValueCache();
  assert.deepEqual(cache.peekValues(["A", "B"]), { values: {}, complete: false });

  cache.setValue("A", 1);
  const partial = cache.peekValues(["A", "B"]);
  assert.equal(partial.complete, false);
  assert.equal(partial.values.A, 1);

  cache.setValue("B", 2);
  assert.deepEqual(cache.peekValues(["A", "B"]), { values: { A: 1, B: 2 }, complete: true });
});

test("peekValues on an empty name list is never complete", () => {
  const cache = createSettingValueCache();
  assert.equal(cache.peekValues([]).complete, false);
  assert.equal(cache.peekValues(null).complete, false);
});

test("loadGroup only fetches the names that are missing or stale", async () => {
  const clock = createClock();
  const cache = createSettingValueCache({ now: clock.now });
  const requested = [];
  const fetchMissing = (names) => {
    requested.push([...names]);
    return Object.fromEntries(names.map((name) => [name, `${name}-fetched`]));
  };

  cache.setValue("A", "a-cached");
  const values = await cache.loadGroup("group", { names: ["A", "B"], fetchMissing });

  assert.deepEqual(requested, [["B"]]);
  assert.deepEqual(values, { A: "a-cached", B: "B-fetched" });
});

test("loadGroup serves a fresh group without touching the network", async () => {
  const clock = createClock();
  const cache = createSettingValueCache({ now: clock.now });
  let calls = 0;
  const fetchMissing = (names) => {
    calls += 1;
    return Object.fromEntries(names.map((name) => [name, 1]));
  };

  await cache.loadGroup("group", { names: ["A"], fetchMissing });
  await cache.loadGroup("group", { names: ["A"], fetchMissing });
  assert.equal(calls, 1);

  clock.advance(DEFAULT_SETTING_VALUES_TTL_MS + 1);
  await cache.loadGroup("group", { names: ["A"], fetchMissing });
  assert.equal(calls, 2, "an expired TTL must trigger exactly one refetch");
});

test("force bypasses both the group cache and the per-name cache", async () => {
  const cache = createSettingValueCache();
  let calls = 0;
  const fetchMissing = (names) => {
    calls += 1;
    return Object.fromEntries(names.map((name) => [name, calls]));
  };

  await cache.loadGroup("group", { names: ["A"], fetchMissing });
  const forced = await cache.loadGroup("group", { names: ["A"], fetchMissing, force: true });

  assert.equal(calls, 2);
  assert.deepEqual(forced, { A: 2 });
});

test("concurrent loadGroup calls share one in-flight request", async () => {
  const cache = createSettingValueCache();
  let calls = 0;
  let release;
  const gate = new Promise((resolve) => { release = resolve; });
  const fetchMissing = async (names) => {
    calls += 1;
    await gate;
    return Object.fromEntries(names.map((name) => [name, "v"]));
  };

  const first = cache.loadGroup("group", { names: ["A"], fetchMissing });
  const second = cache.loadGroup("group", { names: ["A"], fetchMissing });
  release();
  await Promise.all([first, second]);

  assert.equal(calls, 1);
});

// This is the guard that keeps a slow background read from clobbering a value
// the user just committed. Without it the UI silently reverts to the old value.
test("a write during an in-flight fetch is not overwritten by the stale response", async () => {
  const cache = createSettingValueCache();
  let release;
  const gate = new Promise((resolve) => { release = resolve; });
  const fetchMissing = async () => {
    await gate;
    return { ApplyModelSpeed: 0 };
  };

  const pending = cache.loadGroup("DRIVING", { names: ["ApplyModelSpeed"], fetchMissing });
  cache.setValue("ApplyModelSpeed", -1, "DRIVING");
  release();
  await pending;

  assert.equal(
    cache.peekValues(["ApplyModelSpeed"]).values.ApplyModelSpeed,
    -1,
    "the committed value must survive a stale in-flight response",
  );
});

test("clear() discards the result of a request that was already in flight", async () => {
  const cache = createSettingValueCache();
  let release;
  const gate = new Promise((resolve) => { release = resolve; });
  const fetchMissing = async () => {
    await gate;
    return { A: "stale" };
  };

  const pending = cache.loadGroup("group", { names: ["A"], fetchMissing });
  cache.clear();
  release();
  await pending;

  assert.equal(cache.peekValues(["A"]).complete, false);
});

test("invalidateGroup drops the group snapshot but keeps individual values", async () => {
  const cache = createSettingValueCache();
  let calls = 0;
  const fetchMissing = (names) => {
    calls += 1;
    return Object.fromEntries(names.map((name) => [name, "v"]));
  };

  await cache.loadGroup("group", { names: ["A"], fetchMissing });
  cache.invalidateGroup("group");

  assert.equal(cache.peekValues(["A"]).complete, true, "per-name cache survives invalidation");
  await cache.loadGroup("group", { names: ["A"], fetchMissing });
  assert.equal(calls, 1, "a fresh per-name value still satisfies the reload");
});

test("applyValues updates every cached group that already tracks the name", async () => {
  const cache = createSettingValueCache();
  const fetchMissing = (names) => Object.fromEntries(names.map((name) => [name, "old"]));

  await cache.loadGroup("left", { names: ["Shared"], fetchMissing });
  await cache.loadGroup("right", { names: ["Shared"], fetchMissing });
  cache.applyValues({ Shared: "new", Unknown: "ignored" });

  const left = await cache.loadGroup("left", { names: ["Shared"], fetchMissing });
  const right = await cache.loadGroup("right", { names: ["Shared"], fetchMissing });
  assert.equal(left.Shared, "new");
  assert.equal(right.Shared, "new");
});

test("loadGroup rejects when values are missing and no fetcher is supplied", async () => {
  const cache = createSettingValueCache();
  await assert.rejects(
    () => cache.loadGroup("group", { names: ["A"] }),
    /fetchMissing must be a function/,
  );
});

test("loadGroup with no names primes an empty group instead of fetching", async () => {
  const cache = createSettingValueCache();
  assert.deepEqual(await cache.loadGroup("group", { names: [] }), {});
  assert.deepEqual(await cache.loadGroup("", { names: ["A"] }), {});
});
