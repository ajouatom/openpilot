import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";

const root = new URL("../", import.meta.url);
const source = readFileSync(new URL("js/shared/ui/navigation.js", root), "utf8");
const settings = readFileSync(new URL("js/pages/setting.js", root), "utf8");

function functionSource(text, name) {
  const start = text.search(new RegExp(`^(?:async )?function ${name}\\(`, "m"));
  assert.notEqual(start, -1);
  const end = text.indexOf("\n}", start);
  return text.slice(start, end + 2);
}

function createView({ split = false, detail = null, group = "Cruise", previousPage = "tools", inlineSearch = false } = {}) {
  const calls = [];
  const history = {
    state: { page: "setting", screen: detail ? "detail" : "items", group },
    back() { calls.push(["browserBack", previousPage]); },
    replaceState(state) { this.state = state; },
  };
  const context = vm.createContext({
    CURRENT_PAGE: "setting", CURRENT_GROUP: group, CURRENT_SETTING_DETAIL: detail,
    SETTING_INLINE_SEARCH_GROUP: "search", itemsTitle: {}, history,
    window: { scrollTo() {} }, requestAnimationFrame: (fn) => fn(),
    shouldUseSettingSplitLayout: () => split, isCompactLandscapeMode: () => split,
    isCarrotSettingTabActive: () => true,
    closeSettingSearchPanel: (options) => calls.push(["closeSearch", options.syncHistory]),
    syncSettingViewportLayout: async () => calls.push(["splitRoot"]),
    showSettingScreen: (screen, push) => calls.push(["screen", screen, push]),
    setSettingItemsScrollTop: () => {},
    transitionSettingItemsContent: async (render, direction) => {
      calls.push(["transition", direction]);
      await render();
    },
    settingValueRepository: { invalidateGroup: (name) => calls.push(["invalidate", name]) },
    hasRenderedSettingItems: () => false, renderGroups: () => {},
    saveCurrentSettingScrollPosition: () => {},
    syncSettingGroupChrome: () => {
      if (group === "search") history.state.inlineSearchQuery = "StoppingAccel";
    },
    renderItems: async (name, options) => calls.push(["render", name, options.scrollMode]),
    ...(inlineSearch ? {
      showSettingInlineSearchResultsInGroups: async () => calls.push(["inlineSearchSurface"]),
    } : {}),
  });
  vm.runInContext([
    functionSource(source, "goToSettingParent"),
    functionSource(source, "resetSettingPageToRoot"),
    functionSource(settings, "activateSettingGroup"),
    source.match(/^if \(itemsTitle\) itemsTitle\.onclick = .*;$/m)[0],
  ].join("\n"), context);
  return { context, calls, history, click: () => context.itemsTitle.onclick() };
}

for (const previousPage of ["tools", "carrot", "terminal"]) {
  test(`group header returns to settings root even when browser history points to ${previousPage}`, async () => {
    const view = createView({ previousPage });
    await view.click();
    assert.ok(view.calls.some((call) => call[0] === "screen" && call[1] === "groups"));
    assert.equal(view.history.state.page, "setting");
    assert.equal(view.history.state.screen, "groups");
    assert.equal(view.calls.some((call) => call[0] === "browserBack"), false);
  });
}

for (const split of [false, true]) {
  test(`detail header restores its group and scroll position (split=${split})`, async () => {
    const view = createView({ split, detail: "StoppingAccel" });
    await view.click();
    assert.equal(view.context.CURRENT_SETTING_DETAIL, null);
    assert.equal(view.history.state.screen, "items");
    assert.equal(view.history.state.group, "Cruise");
    assert.ok(view.calls.some((call) => call.join() === "render,Cruise,restore"));
    assert.ok(view.calls.some((call) => call.join() === "transition,backward"));
    assert.equal(view.calls.some((call) => call[0] === "browserBack"), false);
  });
}

test("split group header stays on the already visible settings root", async () => {
  const view = createView({ split: true });
  await view.click();
  assert.ok(view.calls.some((call) => call[0] === "splitRoot"));
  assert.equal(view.history.state.page, "setting");
  assert.equal(view.calls.some((call) => call[0] === "browserBack"), false);
});

test("search detail returns to results with its query, then to the settings root", async () => {
  const view = createView({ group: "search", detail: "StoppingAccel" });
  await view.click();
  assert.ok(view.calls.some((call) => call.join() === "invalidate,search"));
  assert.equal(view.history.state.inlineSearchQuery, "StoppingAccel");
  await view.click();
  assert.equal(view.history.state.screen, "groups");
  assert.equal(view.calls.some((call) => call[0] === "browserBack"), false);
});

test("single-column search detail header returns to the in-place results", async () => {
  const view = createView({ group: "search", detail: "StoppingAccel", inlineSearch: true });
  await view.click();
  assert.ok(view.calls.some((call) => call.join() === "inlineSearchSurface"));
  assert.equal(view.calls.some((call) => call[0] === "transition"), false);
  assert.equal(view.calls.some((call) => call[0] === "render"), false);
  assert.equal(view.history.state.screen, "detail");
  assert.equal(view.history.state.group, "search");
});

test("split layout keeps the items flow for a search detail header", async () => {
  const view = createView({ split: true, group: "search", detail: "StoppingAccel", inlineSearch: true });
  await view.click();
  assert.equal(view.calls.some((call) => call.join() === "inlineSearchSurface"), false);
  assert.ok(view.calls.some((call) => call.join() === "transition,backward"));
});


test("ATC add-on lives in the original mode selector's details", async () => {
  const { createSettingsDerivedModel } = await import("../src/features/settings/derived_model.js");
  const catalog = JSON.parse(readFileSync(new URL("../../carrot_settings.json", root), "utf8"));
  const parent = catalog.params.find((item) => item.name === "AutoTurnControl");
  const child = catalog.params.find((item) => item.name === "AutoTurnControlRightLaneChange");
  const model = createSettingsDerivedModel({
    catalog: { groups: [], categories: [], items_by_group: { ATC: [parent, child] } },
    language: "ko", favorites: [], profiles: [],
  });
  assert.equal(parent.max, 3);
  assert.equal(child.default, 0);
  assert.equal(child.control, "toggle");
  assert.deepEqual(model.getItemEntriesForGroup("ATC").map((e) => e.item.name), [parent.name]);
  assert.deepEqual(model.getDetailEntries("ATC", parent.name).map((e) => e.item.name), [parent.name, child.name]);
});


test("live geometry is one directly adjustable setting with zero as vehicle default", async () => {
  const { createSettingsDerivedModel } = await import("../src/features/settings/derived_model.js");
  const catalog = JSON.parse(readFileSync(new URL("../../carrot_settings.json", root), "utf8"));
  assert.equal(catalog.params.some((item) => item.name === "CenterToFrontRatioEnabled"), false);
  const ratio = catalog.params.find((item) => item.name === "CenterToFrontRatio");
  const model = createSettingsDerivedModel({
    catalog: { groups: [], categories: [], items_by_group: { STEER: [ratio] } },
    language: "ko", favorites: [], profiles: [],
  });
  assert.equal(ratio.default, 0);
  assert.equal(ratio.display_unit, "percent");
  assert.equal(ratio.detail_parent, undefined);
  assert.deepEqual(model.getItemEntriesForGroup("STEER").map((e) => e.item.name), [ratio.name]);
});

test("geometry numeric entry and step buttons skip the unsupported gap in both directions", () => {
  const context = vm.createContext({});
  vm.runInContext(functionSource(settings, "normalizeSettingNumericValue"), context);
  const normalize = context.normalizeSettingNumericValue;
  const p = { min: 0, max: 69, nonzero_min: 31 };
  assert.equal(normalize(1, p, 0), 31);
  assert.equal(normalize(30, p, 31), 0);
  assert.equal(normalize(30, p), 31);
  assert.equal(normalize(0, p), 0);
  assert.equal(normalize(60, p), 60);
  assert.equal(normalize(100, p), 69);
  assert.equal(normalize(-10, p), 0);
  assert.equal(normalize("bad", p), null);
  assert.equal(normalize("", p), null);
  assert.equal(normalize(1, { min: 0, max: 300 }), 1);
});

test("geometry zero label shows vehicle default without a misleading percent suffix", () => {
  const context = vm.createContext({ LANG: "ko", getDeclaredSettingOptionLabel: () => null, getSettingDisplayUnit: () => "%" });
  vm.runInContext(functionSource(settings, "formatSettingDisplayValue"), context);
  const p = { name: "CenterToFrontRatio", zero_label: { ko: "0 · 차량 기본", en: "0 · Vehicle default" } };
  assert.equal(context.formatSettingDisplayValue(p, 0), "0 · 차량 기본");
  assert.equal(context.formatSettingDisplayValue(p, 60), "60%");
});
