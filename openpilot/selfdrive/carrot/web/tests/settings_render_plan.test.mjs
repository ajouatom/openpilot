import assert from "node:assert/strict";
import test from "node:test";

import { SETTING_DERIVED_IDS } from "../src/features/settings/derived_model.js";
import {
  createSettingsGroupRenderPlan,
  createSettingsItemLayoutPlan,
} from "../src/features/settings/dom_renderer.js";

const IDS = SETTING_DERIVED_IDS;

function groupPlan(groups, currentGroup = "") {
  return createSettingsGroupRenderPlan({
    groups,
    currentGroup,
    ids: IDS,
    getGroupLabel: (group) => `label:${group}`,
    profilesLabel: "프로필",
  });
}

test("plain groups retain internal counts without displaying them in the title", () => {
  const plan = groupPlan([{ group: "SPEED", count: 3 }], "SPEED");
  const [entry] = plan.entries;

  assert.equal(entry.kind, "group");
  assert.equal(entry.group, "SPEED");
  assert.equal(entry.label, "label:SPEED");
  assert.equal(entry.count, 3);
  assert.equal(entry.title, "label:SPEED");
  assert.equal(entry.active, true);
  assert.equal(entry.favorite, false);
  assert.equal(entry.profile, false);
});

test("a group without a usable count omits the count from the title", () => {
  const [entry] = groupPlan([{ group: "SPEED", count: "many" }]).entries;
  assert.equal(entry.count, null);
  assert.equal(entry.title, "label:SPEED");
});

test("favorites and profile groups are flagged for their own styling", () => {
  const plan = groupPlan([
    { group: IDS.favoritesGroup, count: 2 },
    { group: `${IDS.profileGroupPrefix}p1`, count: 1 },
  ]);

  assert.equal(plan.entries[0].favorite, true);
  assert.equal(plan.entries[1].profile, true);
});

test("category and profile dividers become divider entries", () => {
  const plan = groupPlan([
    { group: `${IDS.categoryDividerPrefix}DRIVING`, label: "주행 설정" },
    { group: IDS.profilesDivider },
  ]);

  assert.equal(plan.entries[0].kind, "divider");
  assert.equal(plan.entries[0].categoryDivider, true);
  assert.equal(plan.entries[0].label, "주행 설정");
  assert.equal(plan.entries[1].kind, "divider");
  assert.equal(plan.entries[1].categoryDivider, false);
  assert.equal(plan.entries[1].label, "프로필", "a divider with no label falls back to the profiles label");
});

// The signature drives DOM reuse. It must ignore selection so that changing
// groups only updates state instead of rebuilding every button.
test("the signature ignores the active group but tracks label and count", () => {
  const groups = [{ group: "SPEED", count: 3 }, { group: "STEER", count: 1 }];
  assert.equal(groupPlan(groups, "SPEED").signature, groupPlan(groups, "STEER").signature);
  assert.notEqual(
    groupPlan(groups).signature,
    groupPlan([{ group: "SPEED", count: 4 }, { group: "STEER", count: 1 }]).signature,
  );
});

test("group plans are frozen and tolerate an empty input", () => {
  const plan = groupPlan([]);
  assert.equal(Object.isFrozen(plan), true);
  assert.equal(Object.isFrozen(plan.entries), true);
  assert.deepEqual(plan.entries, []);
  assert.equal(createSettingsGroupRenderPlan().entries.length, 0);
});

function itemPlan(options) {
  return createSettingsItemLayoutPlan({
    getSectionLabel: (section) => section?.ko || "",
    getGroupLabel: (group) => `label:${group}`,
    ...options,
  });
}

test("a category section header is emitted only when the section id changes", () => {
  const entries = [
    { group: "SPEED", item: { name: "A", __section: { id: "S1", ko: "과속카메라" } } },
    { group: "SPEED", item: { name: "B", __section: { id: "S1", ko: "과속카메라" } } },
    { group: "SPEED", item: { name: "C", __section: { id: "S2", ko: "제한속도" } } },
  ];
  const rows = itemPlan({ entries, group: "SPEED" }).rows;

  assert.equal(rows[0].section.kind, "category");
  assert.equal(rows[0].section.label, "과속카메라");
  assert.equal(rows[1].section, null, "a repeated section must not repeat its header");
  assert.equal(rows[2].section.key, "S2");
});

test("detail and favorite modes replace the section header with their own kind", () => {
  const entries = [{ group: "SPEED", item: { name: "A", __section: { id: "S1", ko: "x" } } }];

  assert.equal(itemPlan({ entries, group: "SPEED", detailMode: true }).rows[0].section.kind, "detail");
  assert.equal(itemPlan({ entries, group: "SPEED", favoriteMode: true }).rows[0].section.kind, "favorites");
});

test("profile rows are sectioned by their originating group with counts", () => {
  const profile = { id: "p1", name: "고속도로" };
  const entries = [
    { group: "SPEED", item: { name: "A" } },
    { group: "SPEED", item: { name: "B" } },
    { group: "STEER", item: { name: "C" } },
  ];
  const rows = itemPlan({ entries, group: "p1", profile }).rows;

  assert.equal(rows[0].section.kind, "profile");
  assert.equal(rows[0].section.key, "p1:SPEED");
  assert.equal(rows[0].section.label, "label:SPEED");
  assert.equal(rows[0].section.count, 2);
  assert.equal(rows[1].section, null);
  assert.equal(rows[2].section.key, "p1:STEER");
  assert.equal(rows[2].section.count, 1);
});

test("profile section expansion is delegated to the caller", () => {
  const profile = { id: "p1", name: "n" };
  const entries = [{ group: "SPEED", item: { name: "A" } }];
  const collapsed = itemPlan({
    entries,
    group: "p1",
    profile,
    getProfileSectionExpanded: () => false,
  }).rows[0].section;

  assert.equal(collapsed.expanded, false);
  assert.equal(itemPlan({ entries, group: "p1", profile }).rows[0].section.expanded, true);
});

test("item rows keep their origin group and are frozen", () => {
  const entries = [{ group: "SPEED", item: { name: "A" } }, { item: { name: "B" } }];
  const plan = itemPlan({ entries, group: "FALLBACK" });

  assert.equal(plan.rows[0].originGroup, "SPEED");
  assert.equal(plan.rows[1].originGroup, "FALLBACK", "a row without a group inherits the rendered group");
  assert.equal(plan.rows[0].index, 0);
  assert.equal(Object.isFrozen(plan.rows[0]), true);
  assert.deepEqual(createSettingsItemLayoutPlan().rows, []);
});
