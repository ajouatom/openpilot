import assert from "node:assert/strict";
import test from "node:test";

import {
  createSettingsAuxState,
  normalizeSettingFavoriteNames,
  normalizeSettingPopularValues,
  normalizeSettingProfiles,
} from "../src/features/settings/aux_state.js";

test("favorite names are trimmed, de-duplicated and order preserving", () => {
  assert.deepEqual(
    normalizeSettingFavoriteNames(["  A  ", "B", "A", "", null, "C"]),
    ["A", "B", "C"],
  );
  assert.deepEqual(normalizeSettingFavoriteNames(null), []);
  assert.deepEqual(normalizeSettingFavoriteNames("A"), []);
});

test("profiles missing id, name or values are discarded", () => {
  const profiles = normalizeSettingProfiles([
    { id: "p1", name: "고속도로", values: { A: 1 } },
    { id: "p2", name: "no values" },
    { name: "no id", values: {} },
    null,
  ]);
  assert.deepEqual(profiles.map((profile) => profile.id), ["p1"]);
});

test("profile values and meta are copied so callers cannot mutate the source", () => {
  const source = { id: "p1", name: "n", values: { A: 1 }, meta: { note: "x" } };
  const [profile] = normalizeSettingProfiles([source]);

  profile.values.A = 999;
  profile.meta.note = "changed";
  assert.equal(source.values.A, 1);
  assert.equal(source.meta.note, "x");
});

test("popular payloads normalize their car key, timestamp and value map", () => {
  assert.deepEqual(
    normalizeSettingPopularValues({ car_key: "Casper EV", fetched_at: "1700", popular_values: { A: {} } }),
    { carKey: "Casper EV", fetchedAt: 1700, values: { A: {} } },
  );
  assert.deepEqual(
    normalizeSettingPopularValues({ popular_values: ["not an object map"] }),
    { carKey: "", fetchedAt: 0, values: {} },
  );
  assert.deepEqual(normalizeSettingPopularValues(null), { carKey: "", fetchedAt: 0, values: {} });
});

test("hydrateSnapshot fills every auxiliary resource in one pass", () => {
  const aux = createSettingsAuxState();
  aux.hydrateSnapshot({
    favorites: ["A", "A", "B"],
    profiles: [{ id: "p1", name: "n", values: {} }],
    popular: { car_key: "Casper EV", fetched_at: 5, popular_values: { A: { count: 3 } } },
  });

  assert.deepEqual(aux.favorites.get(), ["A", "B"]);
  assert.equal(aux.profiles.get().length, 1);
  assert.equal(aux.popular.getState().carKey, "Casper EV");
  assert.deepEqual(aux.popular.getValue("A"), { count: 3 });
  assert.equal(aux.favorites.loaded, true);
});

test("getValue only returns object entries", () => {
  const aux = createSettingsAuxState();
  aux.popular.replace({ popular_values: { A: { count: 1 }, B: "scalar" } });

  assert.deepEqual(aux.popular.getValue("A"), { count: 1 });
  assert.equal(aux.popular.getValue("B"), null);
  assert.equal(aux.popular.getValue("missing"), null);
});

test("a loaded resource is served from memory until forced", async () => {
  let calls = 0;
  const aux = createSettingsAuxState({
    loadFavorites: () => { calls += 1; return { favorites: [`A${calls}`] }; },
  });

  assert.deepEqual(await aux.favorites.load(), ["A1"]);
  assert.deepEqual(await aux.favorites.load(), ["A1"]);
  assert.equal(calls, 1);

  assert.deepEqual(await aux.favorites.load(true), ["A2"]);
  assert.equal(calls, 2);
});

test("concurrent loads of the same resource share one request", async () => {
  let calls = 0;
  let release;
  const gate = new Promise((resolve) => { release = resolve; });
  const aux = createSettingsAuxState({
    loadProfiles: async () => {
      calls += 1;
      await gate;
      return { profiles: [{ id: "p1", name: "n", values: {} }] };
    },
  });

  const first = aux.profiles.load();
  const second = aux.profiles.load();
  release();
  await Promise.all([first, second]);

  assert.equal(calls, 1);
});

test("a first-load failure falls back to an empty resource", async () => {
  const aux = createSettingsAuxState({
    loadFavorites: () => { throw new Error("offline"); },
  });

  assert.deepEqual(await aux.favorites.load(), []);
  assert.equal(aux.favorites.loaded, true);
});

test("a refresh failure keeps the previously loaded value", async () => {
  let calls = 0;
  const aux = createSettingsAuxState({
    loadFavorites: () => {
      calls += 1;
      if (calls === 1) return { favorites: ["A"] };
      throw new Error("offline");
    },
  });

  await aux.favorites.load();
  assert.deepEqual(await aux.favorites.load(true), ["A"], "a failed refresh must not wipe the list");
});

test("a missing loader is treated as a load failure, not a crash", async () => {
  const aux = createSettingsAuxState();
  assert.deepEqual(await aux.favorites.load(), []);
});

test("a snapshot arriving mid-flight wins over the in-flight response", async () => {
  let release;
  const gate = new Promise((resolve) => { release = resolve; });
  const aux = createSettingsAuxState({
    loadFavorites: async () => { await gate; return { favorites: ["stale"] }; },
  });

  const pending = aux.favorites.load();
  aux.favorites.replace(["fresh"]);
  release();
  await pending;

  assert.deepEqual(aux.favorites.get(), ["fresh"]);
});
