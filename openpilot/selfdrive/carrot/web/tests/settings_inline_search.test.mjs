import assert from "node:assert/strict";
import test from "node:test";
import { createSettingsDerivedModel, SETTING_DERIVED_IDS } from "../src/features/settings/derived_model.js";
import { createSettingsGroupRenderPlan, createSettingsItemLayoutPlan } from "../src/features/settings/dom_renderer.js";
import { createSettingValueCache } from "../src/features/settings/value_cache.js";

const catalog = {
  groups: [{ group: "SPEED", ko: "속도 제어" }, { group: "FOLLOW", ko: "차간거리" }],
  // Deliberately different object order: the menu determines result order.
  items_by_group: {
    FOLLOW: [{ name: "Gap", title: "차간거리", descr: "부드러운 감속" }],
    SPEED: [
      { name: "Decel", title: "감속 조절", etitle: "Deceleration", min: 0, max: 10 },
      { name: "TfMode", title: "TF 모드" },
      { name: "Child", title: "세부 조절", detail_parent: "TfMode" },
      { name: "Section", __section: { ko: "곡선 구간", en: "Curve" } },
    ],
  },
};
const model = () => createSettingsDerivedModel({ catalog, language: "ko", favorites: ["Decel"], profiles: [{ id: "p", values: { Decel: 2 } }] });
const names = (result) => result.entries.map(({ item }) => item.name);

test("inline search matches Korean titles/descriptions in menu order and returns original live controls", () => {
  const result = model().searchItemEntries("  감속  ");
  assert.deepEqual(names(result), ["Decel", "Gap"]);
  assert.equal(result.total, 2, "favorites and profiles do not duplicate live parameters");
  assert.equal(result.entries[0].item, catalog.items_by_group.SPEED[0]);
  assert.equal(result.entries[0].group, "SPEED");
});

test("TF search ignores case and includes detail controls through parent context", () => {
  assert.deepEqual(names(model().searchItemEntries("TF")), ["TfMode", "Child"]);
  assert.deepEqual(names(model().searchItemEntries("tf")), ["TfMode", "Child"]);
  assert.deepEqual(names(model().searchItemEntries("세부")), ["Child"]);
});

test("search covers names, translated descriptions, group and section labels", () => {
  assert.deepEqual(names(model().searchItemEntries("deceleration")), ["Decel"]);
  assert.deepEqual(names(model().searchItemEntries("곡선")), ["Section"]);
  assert.deepEqual(names(model().searchItemEntries("차간거리")), ["Gap"]);
  assert.deepEqual(names(model().searchItemEntries("감속".normalize("NFD"))), ["Decel", "Gap"]);
});

test("results stop at 20 in catalog order while counting all unique matches", () => {
  const items = Array.from({ length: 32 }, (_, i) => ({ name: `TF${i}` }));
  const large = createSettingsDerivedModel({ catalog: {
    groups: [{ group: "A" }, { group: "B" }],
    items_by_group: { A: items, B: [items[0]] },
  } });
  const result = large.searchItemEntries("TF");
  assert.equal(result.total, 32);
  assert.deepEqual(names(result), items.slice(0, 20).map(({ name }) => name));
});

test("empty and unmatched queries never turn into an all-settings list", () => {
  for (const query of ["", "  ", "unknown", "<script>"]) {
    assert.deepEqual(model().searchItemEntries(query), { entries: [], total: 0 });
  }
});

test("search slot is distinct from navigation buttons and can reuse DOM across selection", () => {
  const groups = [{ group: SETTING_DERIVED_IDS.favoritesGroup }, { group: SETTING_DERIVED_IDS.searchGroup }, { group: "SPEED" }];
  const plan = (currentGroup) => createSettingsGroupRenderPlan({ groups, ids: SETTING_DERIVED_IDS, currentGroup });
  assert.equal(plan("SPEED").entries[1].kind, "search");
  assert.equal(plan("SPEED").signature, plan(SETTING_DERIVED_IDS.searchGroup).signature);
});

test("result cards retain their source grouping", () => {
  const rows = createSettingsItemLayoutPlan({
    entries: model().searchItemEntries("감속").entries,
    searchMode: true,
    getItemContextLabel: (group) => group,
  }).rows;
  assert.deepEqual(rows.map(({ section }) => section.label), ["SPEED", "FOLLOW"]);
});

test("invalidating a changing search result set prevents reuse of an older pending load", async () => {
  const cache = createSettingValueCache();
  let resolveOld;
  const old = cache.loadGroup("search", { names: ["Decel"], fetchMissing: () => new Promise((resolve) => { resolveOld = resolve; }) });
  await Promise.resolve();
  cache.invalidateGroup("search");
  assert.deepEqual(await cache.loadGroup("search", { names: ["TfMode"], fetchMissing: async () => ({ TfMode: 3 }) }), { TfMode: 3 });
  resolveOld({ Decel: 1 });
  await old;
  assert.deepEqual(await cache.loadGroup("search", { names: ["TfMode"] }), { TfMode: 3 });
});
