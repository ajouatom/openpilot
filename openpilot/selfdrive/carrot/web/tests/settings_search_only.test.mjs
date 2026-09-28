import assert from "node:assert/strict";
import test from "node:test";

import { normalizeSettingsCatalog } from "../src/features/settings/catalog.js";
import { createSettingsDerivedModel } from "../src/features/settings/derived_model.js";
import { collectSettingSearchMatches } from "../src/features/settings/search/entries.js";

function createModel() {
  const catalog = normalizeSettingsCatalog({
    categories: [{ id: "ROOT", en: "Settings", groups: [{
      id: "MONITORING", en: "Monitoring", count: 1,
      sections: [{ id: "DM", en: "Driver", items: ["DriverMonitoringMode", "DisableDM"] }],
    }] }],
    items_by_group: { START: [
      { name: "DriverMonitoringMode", title: "운전자 감시 모드", etitle: "Driver monitoring mode" },
      { name: "DisableDM", title: "운전자 감시 끄기", etitle: "Disable Driver Monitoring",
        descr: "숨겨진 감시 설정", search_only: true, control: "select", min: 0, max: 2, default: 0 },
    ] },
  });
  return createSettingsDerivedModel({ catalog, language: "ko" });
}

test("search-only settings survive catalog normalization without appearing in ordinary rows", () => {
  const model = createModel();
  assert.deepEqual(model.getItemEntriesForGroup("MONITORING").map(({ item }) => item.name), ["DriverMonitoringMode"]);
  assert.equal(model.getGroupsForDisplay().find(({ group }) => group === "MONITORING").count, 1);
  assert.equal(model.findItemByName("DisableDM").item.search_only, true);
});

test("both search surfaces hide partial names, titles, descriptions and group matches", () => {
  const model = createModel();
  const overlay = model.buildSearchEntries();
  for (const query of ["", "DM", "Disable", "감시", "숨겨진", "monitoring", "Driver", "운전자 감시 끄기", "DisableDM extra"]) {
    assert.equal(model.searchItemEntries(query).entries.some(({ item }) => item.name === "DisableDM"), false, query);
    assert.equal(collectSettingSearchMatches(overlay, { query }).entries.some(({ name }) => name === "DisableDM"), false, query);
  }
  assert.equal(model.searchItemEntries("감시").total, 1, "normal settings retain substring matching");
});

test("an exact name query reveals the original live control and its detail entry", () => {
  const model = createModel();
  const original = model.findItemByName("DisableDM").item;
  for (const query of ["DisableDM", "disabledm", "  DISABLEDM  "]) {
    const inline = model.searchItemEntries(query);
    assert.equal(inline.total, 1);
    assert.equal(inline.entries[0].item, original);
    assert.equal(inline.entries[0].item.control, "select");
    assert.deepEqual(collectSettingSearchMatches(model.buildSearchEntries(), { query }).entries.map(({ name }) => name), ["DisableDM"]);
  }
  assert.equal(model.getDetailEntries("MONITORING", "DisableDM")[0].item, original);
  assert.equal(model.searchItemEntries("DM").entries.some(({ item }) => item.name === "DisableDM"), false,
    "an earlier exact search does not reveal it to later partial queries");
});
