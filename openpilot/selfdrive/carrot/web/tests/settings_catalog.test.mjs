import assert from "node:assert/strict";
import test from "node:test";

import {
  createSettingsCatalogState,
  normalizeSettingsCatalog,
} from "../src/features/settings/catalog.js";

// Mirrors the payload shape built by server/services/settings.py:
// categories carry the 대>중>소 tree, items_by_group carries the definitions.
function createCatalog() {
  return {
    categories: [
      {
        id: "DRIVING",
        ko: "주행 설정",
        en: "Driving",
        groups: [
          {
            id: "SPEED",
            ko: "속도제어",
            en: "Speed",
            count: 2,
            sections: [
              { id: "SPEED_CAMERA", ko: "과속카메라", en: "Speed camera", items: ["AutoNaviSpeedCtrlEnd"] },
              { id: "SPEED_LIMIT", ko: "제한속도", en: "Speed limit", items: ["ApplyModelSpeed", "Unknown"] },
            ],
          },
        ],
      },
    ],
    items_by_group: {
      LEGACY: [
        { name: "AutoNaviSpeedCtrlEnd", min: 3, max: 20, default: 6 },
        { name: "ApplyModelSpeed", min: -120, max: 120, default: 0 },
      ],
    },
  };
}

test("a catalog without categories is returned untouched", () => {
  const flat = { groups: [{ group: "LEGACY" }], items_by_group: { LEGACY: [] } };
  assert.equal(normalizeSettingsCatalog(flat), flat);
  assert.equal(normalizeSettingsCatalog(null), null);
  assert.deepEqual(normalizeSettingsCatalog({ categories: [] }), { categories: [] });
});

test("categories are flattened into a group list that carries its category id", () => {
  const catalog = normalizeSettingsCatalog(createCatalog());
  assert.deepEqual(catalog.groups, [
    { group: "SPEED", ko: "속도제어", en: "Speed", zh: undefined, count: 2, category: "DRIVING" },
  ]);
});

test("section items are resolved from the definition index and tagged with __section", () => {
  const catalog = normalizeSettingsCatalog(createCatalog());
  const items = catalog.items_by_group.SPEED;

  assert.equal(items.length, 2, "an item with no definition must be dropped");
  assert.deepEqual(items.map((item) => item.name), ["AutoNaviSpeedCtrlEnd", "ApplyModelSpeed"]);
  assert.equal(items[0].__section.id, "SPEED_CAMERA");
  assert.equal(items[1].__section.ko, "제한속도");
  assert.equal(items[1].min, -120, "the original definition fields are preserved");
});

test("normalizing replaces the legacy flat grouping", () => {
  const catalog = normalizeSettingsCatalog(createCatalog());
  assert.equal(catalog.items_by_group.LEGACY, undefined);
});

test("commit normalizes once and reports whether anything changed", () => {
  const state = createSettingsCatalogState();
  const raw = createCatalog();

  const first = state.commit(raw);
  assert.equal(first.changed, true);
  assert.equal(first.catalog.groups[0].group, "SPEED");
  assert.equal(state.peek(), first.catalog);

  const second = state.commit(first.catalog);
  assert.equal(second.changed, false, "the same catalog and snapshot must not re-normalize");
});

test("commit treats a new snapshot as a change even for the same catalog", () => {
  const state = createSettingsCatalogState();
  const catalog = normalizeSettingsCatalog(createCatalog());

  state.commit(catalog, { id: 1 });
  assert.equal(state.commit(catalog, { id: 2 }).changed, true);
});

test("commit rejects a non-object catalog", () => {
  const state = createSettingsCatalogState();
  assert.throws(() => state.commit(null), TypeError);
  assert.throws(() => state.commit("catalog"), TypeError);
  assert.equal(state.peek(), null);
});
