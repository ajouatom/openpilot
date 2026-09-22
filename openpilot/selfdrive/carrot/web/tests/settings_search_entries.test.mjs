import assert from "node:assert/strict";
import test from "node:test";

import { createSettingsDerivedModel } from "../src/features/settings/derived_model.js";
import {
  collectSettingSearchMatches,
  DEFAULT_SEARCH_RESULT_LIMIT,
  filterSettingSearchEntries,
  isProfileSearchScope,
  normalizeSearchScope,
} from "../src/features/settings/search/entries.js";

const entries = [
  { source: "carrot", group: "SPEED", name: "Decel", haystack: "감속 조절 decel deceleration 속도" },
  { source: "carrot", group: "FOLLOW", name: "Gap", haystack: "차간거리 gap follow" },
  { source: "profile", profileId: "p1", group: "profile:p1:SPEED", name: "Decel", haystack: "감속 조절 decel profile" },
  { source: "profile", profileId: "p2", group: "profile:p2:SPEED", name: "Decel", haystack: "감속 조절 decel profile" },
];

const names = (result) => result.map(({ name }) => name);

test("empty or whitespace-only queries never return the whole catalog", () => {
  assert.deepEqual(filterSettingSearchEntries(entries, { query: "" }), []);
  assert.deepEqual(filterSettingSearchEntries(entries, { query: "   " }), []);
  assert.deepEqual(filterSettingSearchEntries(entries, { query: null }), []);
});

test("matching is trimmed, case-insensitive and keeps catalog order", () => {
  assert.deepEqual(names(filterSettingSearchEntries(entries, { query: "  DECEL " })), ["Decel", "Decel", "Decel"]);
  assert.deepEqual(names(filterSettingSearchEntries(entries, { query: "gap" })), ["Gap"]);
});

test("the default result limit stays at the shared 36", () => {
  const many = Array.from({ length: 40 }, (_, index) => ({
    source: "carrot",
    group: "SPEED",
    name: `TF${index}`,
    haystack: `tf ${index}`,
  }));
  assert.equal(filterSettingSearchEntries(many, { query: "tf" }).length, DEFAULT_SEARCH_RESULT_LIMIT);
  assert.equal(filterSettingSearchEntries(many, { query: "tf", limit: 5 }).length, 5);
  assert.equal(filterSettingSearchEntries(many, { query: "tf", limit: 0 }).length, 0);
});

test("the collector reports the full total while limiting the returned entries", () => {
  const many = Array.from({ length: 40 }, (_, index) => ({
    source: "carrot",
    group: "SPEED",
    name: `TF${index}`,
    haystack: `tf ${index}`,
  }));
  const collected = collectSettingSearchMatches(many, { query: "tf" });
  assert.equal(collected.entries.length, DEFAULT_SEARCH_RESULT_LIMIT);
  assert.equal(collected.total, 40);

  const limited = collectSettingSearchMatches(many, { query: "tf", limit: 5 });
  assert.equal(limited.entries.length, 5);
  assert.equal(limited.total, 40);

  const zero = collectSettingSearchMatches(many, { query: "tf", limit: 0 });
  assert.deepEqual(zero.entries, []);
  assert.equal(zero.total, 40);

  assert.deepEqual(collectSettingSearchMatches(many, { query: "" }), { entries: [], total: 0 });
});

test("the collector counts only the scoped matches in its total", () => {
  const collected = collectSettingSearchMatches(entries, { query: "decel", scope: { type: "profile", profileId: "p1" } });
  assert.equal(collected.total, 1);
  assert.deepEqual(collected.entries.map(({ profileId }) => profileId), ["p1"]);
});

test("a profile scope only returns that profile's entries", () => {
  assert.deepEqual(
    filterSettingSearchEntries(entries, { query: "decel", scope: { type: "profile", profileId: "p1" } })
      .map(({ profileId }) => profileId),
    ["p1"],
  );
  assert.equal(filterSettingSearchEntries(entries, { query: "gap", scope: { type: "profile", profileId: "p1" } }).length, 0);
});

test("the all scope keeps every source", () => {
  assert.equal(filterSettingSearchEntries(entries, { query: "decel", scope: { type: "all" } }).length, 3);
});

test("the overlay index matches an NFD query against its NFC haystack", () => {
  const model = createSettingsDerivedModel({
    catalog: {
      groups: [{ group: "SPEED", ko: "속도 제어" }],
      items_by_group: { SPEED: [{ name: "Decel", title: "감속 조절", descr: "부드러운 감속" }] },
    },
    language: "ko",
  });
  const entries = model.buildSearchEntries({ carrot: "CarrotPilot", profile: "Profile" });

  assert.deepEqual(names(filterSettingSearchEntries(entries, { query: "감속".normalize("NFD") })), ["Decel"]);
  assert.deepEqual(names(filterSettingSearchEntries(entries, { query: "DECEL" })), ["Decel"]);
});

test("detail children are searchable through their parent's screen", () => {
  const model = createSettingsDerivedModel({
    catalog: {
      groups: [{ group: "VISION" }],
      items_by_group: {
        VISION: [
          { name: "ShareData", title: "Share data", descr: "Sharing options" },
          { name: "LaneThreshold", title: "Lane threshold", descr: "Lane detection cut-off", detail_parent: "ShareData" },
        ],
      },
    },
    language: "en",
  });
  const entries = model.buildSearchEntries({ carrot: "CarrotPilot", profile: "Profile" });

  const child = entries.find((entry) => entry.name === "LaneThreshold");
  assert.equal(child.detailParent, "ShareData");
  assert.deepEqual(names(filterSettingSearchEntries(entries, { query: "threshold" })), ["LaneThreshold"]);
  assert.deepEqual(names(filterSettingSearchEntries(entries, { query: "sharing" })), ["ShareData", "LaneThreshold"]);
});

test("entries without a haystack are skipped instead of throwing", () => {
  const odd = [{ source: "carrot", name: "Broken" }, { source: "carrot", name: "Ok", haystack: "ok" }];
  assert.deepEqual(names(filterSettingSearchEntries(odd, { query: "ok" })), ["Ok"]);
});

test("scope normalization rejects incomplete profile scopes", () => {
  assert.deepEqual(normalizeSearchScope(null), { type: "all", profileId: "" });
  assert.deepEqual(normalizeSearchScope({ type: "profile" }), { type: "all", profileId: "" });
  assert.deepEqual(normalizeSearchScope({ type: "profile", profileId: 7 }), { type: "profile", profileId: "7" });
  assert.equal(isProfileSearchScope({ type: "profile", profileId: "p1" }), true);
  assert.equal(isProfileSearchScope({ type: "profile", profileId: "" }), false);
  assert.equal(isProfileSearchScope({ type: "all", profileId: "p1" }), false);
});
