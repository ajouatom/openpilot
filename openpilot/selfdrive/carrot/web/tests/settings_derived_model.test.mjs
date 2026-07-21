import assert from "node:assert/strict";
import test from "node:test";

import {
  SETTING_DERIVED_IDS,
  createSettingsDerivedModel,
  createSettingsDerivedModelMemo,
  localizedSettingItemText,
  localizedSettingNodeLabel,
} from "../src/features/settings/derived_model.js";

const CATALOG = Object.freeze({
  categories: [
    {
      id: "DRIVING",
      ko: "주행 설정",
      en: "Driving",
      groups: [
        { id: "SPEED", ko: "속도제어", en: "Speed", count: 2 },
        { id: "STEER", ko: "조향 설정", en: "Steering", count: 1 },
      ],
    },
  ],
  groups: [
    { group: "SPEED", ko: "속도제어", en: "Speed", count: 2 },
    { group: "STEER", ko: "조향 설정", en: "Steering", count: 1 },
  ],
  items_by_group: {
    SPEED: [
      { name: "ApplyModelSpeed", title: "모델 주행속도", etitle: "Model speed", __section: { ko: "제한속도", en: "Speed limit" } },
      { name: "CruiseSpeed1", title: "크루즈속도1", etitle: "Cruise speed 1" },
    ],
    STEER: [
      { name: "SteerActuatorDelay", title: "조향 지연", etitle: "Steer delay" },
    ],
  },
});

const PROFILES = Object.freeze([
  { id: "p1", name: "고속도로", values: { SteerActuatorDelay: 30, ApplyModelSpeed: 0 } },
]);

function createModel(overrides = {}) {
  return createSettingsDerivedModel({
    catalog: CATALOG,
    favorites: ["ApplyModelSpeed", "Missing"],
    profiles: PROFILES,
    language: "ko",
    labels: { favorites: "즐겨찾기", profiles: "프로필" },
    ...overrides,
  });
}

test("localizedSettingNodeLabel falls back across languages", () => {
  assert.equal(localizedSettingNodeLabel({ ko: "가", en: "A", zh: "甲" }, "ko"), "가");
  assert.equal(localizedSettingNodeLabel({ en: "A", zh: "甲" }, "ko"), "A");
  assert.equal(localizedSettingNodeLabel({ zh: "甲" }, "en"), "甲");
  assert.equal(localizedSettingNodeLabel(null, "ko"), "");
});

test("localizedSettingItemText prefers the language-specific key", () => {
  const item = { title: "제목", etitle: "Title", ctitle: "标题" };
  assert.equal(localizedSettingItemText(item, "title", "etitle", "", "ko"), "제목");
  assert.equal(localizedSettingItemText(item, "title", "etitle", "", "en"), "Title");
  assert.equal(localizedSettingItemText(item, "title", "etitle", "", "zh"), "标题");
  assert.equal(localizedSettingItemText(null, "title", "etitle", "fallback", "ko"), "fallback");
});

test("favorites resolve to real items and silently drop unknown names", () => {
  const model = createModel();
  assert.deepEqual(model.getValidFavoriteNames(), ["ApplyModelSpeed"]);
  assert.equal(model.getFavoriteEntries()[0].group, "SPEED");
});

test("findItemByName locates an item together with its owning group", () => {
  const model = createModel();
  assert.deepEqual(model.findItemByName("SteerActuatorDelay").group, "STEER");
  assert.equal(model.findItemByName("  ApplyModelSpeed  ").item.name, "ApplyModelSpeed");
  assert.equal(model.findItemByName("Nope"), null);
});

test("profile entries are ordered by catalog position, not by object key order", () => {
  const model = createModel();
  const names = model.getProfileEntries(PROFILES[0]).map((entry) => entry.item.name);
  assert.deepEqual(names, ["ApplyModelSpeed", "SteerActuatorDelay"]);
});

test("the display list opens with favorites and inserts category dividers", () => {
  const model = createModel();
  const display = model.getGroupsForDisplay();

  assert.equal(display[0].group, SETTING_DERIVED_IDS.favoritesGroup);
  assert.equal(display[0].count, 1);
  assert.equal(display[1].group, `${SETTING_DERIVED_IDS.categoryDividerPrefix}DRIVING`);
  assert.equal(display[1].divider, true);
  assert.equal(display[1].label, "주행 설정");
  assert.deepEqual(display.slice(2, 4).map((entry) => entry.group), ["SPEED", "STEER"]);
});

test("profiles are appended behind their own divider", () => {
  const model = createModel();
  const display = model.getGroupsForDisplay();
  const divider = display.find((entry) => entry.group === SETTING_DERIVED_IDS.profilesDivider);
  const profileGroup = display.at(-1);

  assert.equal(divider.label, "프로필");
  assert.equal(profileGroup.group, model.profileGroup("p1"));
  assert.equal(profileGroup.label, "고속도로");
  assert.equal(profileGroup.count, 2);
});

test("a catalog without categories falls back to the flat group list", () => {
  const model = createModel({ catalog: { groups: CATALOG.groups, items_by_group: CATALOG.items_by_group } });
  const display = model.getGroupsForDisplay();
  assert.deepEqual(display.slice(1, 3).map((entry) => entry.group), ["SPEED", "STEER"]);
});

test("profile group ids round-trip through the derived prefix", () => {
  const model = createModel();
  const group = model.profileGroup("p1");

  assert.equal(model.isProfileGroup(group), true);
  assert.equal(model.isProfileGroup("SPEED"), false);
  assert.equal(model.profileIdFromGroup(group), "p1");
  assert.equal(model.profileIdFromGroup("SPEED"), "");
  assert.equal(model.getProfileByGroup(group).name, "고속도로");
  assert.equal(model.getProfileById("nope"), null);
});

test("getItemEntriesForGroup dispatches between favorites, profiles and plain groups", () => {
  const model = createModel();
  assert.deepEqual(
    model.getItemEntriesForGroup(SETTING_DERIVED_IDS.favoritesGroup).map((entry) => entry.item.name),
    ["ApplyModelSpeed"],
  );
  assert.equal(model.getItemEntriesForGroup(model.profileGroup("p1")).length, 2);
  assert.equal(model.getItemEntriesForGroup("SPEED").length, 2);
  assert.deepEqual(model.getItemEntriesForGroup("NOPE"), []);
});

test("group labels honour the active language and virtual groups", () => {
  assert.equal(createModel().getGroupLabel("SPEED"), "속도제어");
  assert.equal(createModel({ language: "en" }).getGroupLabel("SPEED"), "Speed");
  assert.equal(createModel().getGroupLabel(SETTING_DERIVED_IDS.favoritesGroup), "즐겨찾기");
  assert.equal(createModel().getGroupLabel("UNKNOWN"), "UNKNOWN");
});

test("the item context label appends the section when it adds information", () => {
  const model = createModel();
  const withSection = model.findItemByName("ApplyModelSpeed").item;
  const withoutSection = model.findItemByName("CruiseSpeed1").item;

  assert.equal(model.getItemContextLabel("SPEED", withSection), "속도제어 > 제한속도");
  assert.equal(model.getItemContextLabel("SPEED", withoutSection), "속도제어");
});

test("search entries cover catalog items and profile items with a lowercase haystack", () => {
  const model = createModel();
  const entries = model.buildSearchEntries({ carrot: "당근파일럿", profile: "프로필" });

  assert.equal(entries.length, 5, "3 catalog items + 2 profile items");
  const carrot = entries.find((entry) => entry.source === "carrot" && entry.name === "ApplyModelSpeed");
  assert.equal(carrot.groupLabel, "속도제어");
  assert.equal(carrot.title, "모델 주행속도");
  assert.equal(carrot.haystack, carrot.haystack.toLowerCase());

  const profile = entries.find((entry) => entry.source === "profile");
  assert.equal(profile.profileId, "p1");
  assert.equal(profile.group, model.profileGroup("p1"));
  assert.equal(profile.originalGroup, "SPEED");
  assert.equal(profile.contextLabel.startsWith("고속도로 / "), true);
});

test("the memo reuses the model until an input identity changes", () => {
  const memo = createSettingsDerivedModelMemo();
  const favorites = [];
  const options = { catalog: CATALOG, favorites, profiles: PROFILES, language: "ko" };

  const first = memo.get(options);
  assert.equal(memo.get({ ...options }), first, "same identities reuse the model");
  assert.notEqual(memo.get({ ...options, language: "en" }), first);
  assert.notEqual(memo.get({ ...options, favorites: [] }), first, "a new array identity invalidates");

  memo.clear();
  assert.notEqual(memo.get(options), first);
});
