// Single-column inline search keeps its results in place on the group screen
// (#settingInlineSearchResults); the split layout keeps them in the items pane.
// These tests pin the branch selection in applySettingInlineSearch().
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";

const root = new URL("../", import.meta.url);
const settings = readFileSync(new URL("js/pages/setting.js", root), "utf8");

function functionSource(text, name) {
  const start = text.search(new RegExp(`^(?:async )?function ${name}\\(`, "m"));
  assert.notEqual(start, -1);
  const end = text.indexOf("\n}", start);
  return text.slice(start, end + 2);
}

function createSearchView({ split = false, query = "", previousQuery = "", currentGroup = null, previousGroup = null } = {}) {
  const calls = [];
  const history = {
    state: { page: "setting", screen: "groups", group: null },
    pushState(state) { this.state = state; calls.push(["pushHistory"]); },
    replaceState(state) { this.state = state; calls.push(["replaceHistory"]); },
  };
  const context = vm.createContext({
    CURRENT_PAGE: "setting",
    CURRENT_GROUP: currentGroup,
    CURRENT_SETTING_DETAIL: null,
    SETTINGS: {},
    SETTING_INLINE_SEARCH_GROUP: "search",
    SETTING_FAVORITES_GROUP: "favorites",
    settingInlineSearchQuery: previousQuery,
    settingInlineSearchPreviousGroup: previousGroup,
    settingRenderToken: 0,
    settingInlineSearchView: {
      cancelPending() {},
      getQuery: () => query,
      isFocused: () => false,
      focus() {},
      setQuery() {},
    },
    history,
    requestAnimationFrame: (fn) => fn(),
    isCarrotSettingTabActive: () => true,
    isCompactLandscapeMode: () => split,
    isSettingInlineSearchSurfaceActive: () => calls.push(["surfaceActive"]),
    getSettingDerivedModel: () => ({ getGroupMeta: (group) => (group ? { group } : null) }),
    settingValueRepository: { invalidateGroup: (group) => calls.push(["invalidate", group]) },
    saveCurrentSettingScrollPosition: () => calls.push(["saveScroll"]),
    getSavedSettingScrollPosition: () => 0,
    setSettingItemsScrollTop: (top) => calls.push(["setScroll", top]),
    renderGroups: () => calls.push(["renderGroups"]),
    mountSettingInlineSearch: () => calls.push(["mount"]),
    renderItems: async (group, options) => calls.push(["renderItems", group, options?.container ? "container" : "default"]),
    exitSettingInlineSearchSurface: (options) => calls.push(["exit", options?.keepFocus === true]),
    activateSettingGroup: async (group) => calls.push(["activate", group]),
    showSettingScreen: (screen, push) => calls.push(["screen", screen, push]),
  });
  // The real surface function also takes ownership of the current group.
  context.showSettingInlineSearchResultsInGroups = async () => {
    context.CURRENT_GROUP = "search";
    context.CURRENT_SETTING_DETAIL = null;
    calls.push(["inPlace"]);
  };
  vm.runInContext(functionSource(settings, "applySettingInlineSearch"), context);
  return { context, calls, history };
}

test("single-column search enters the in-place surface without a screen switch", async () => {
  const { context, calls, history } = createSearchView({ query: "StoppingAccel" });
  await context.applySettingInlineSearch("StoppingAccel");
  assert.ok(calls.some((call) => call.join() === "inPlace"));
  assert.equal(calls.some((call) => call[0] === "screen"), false);
  assert.equal(calls.some((call) => call[0] === "activate"), false);
  assert.equal(calls.some((call) => call[0] === "renderGroups"), false, "no group-list rebuild while typing");
  assert.equal(context.CURRENT_GROUP, "search");
  assert.equal(history.state.screen, "items");
  assert.equal(history.state.group, "search");
});

test("split search keeps the results on the items screen", async () => {
  const { context, calls } = createSearchView({ split: true, query: "StoppingAccel" });
  await context.applySettingInlineSearch("StoppingAccel");
  assert.equal(calls.some((call) => call.join() === "inPlace"), false);
  assert.ok(calls.some((call) => call.join() === "screen,items,true"));
  assert.ok(calls.some((call) => call.join() === "renderItems,search,default"));
});

test("clearing the query in place restores the group list and history", async () => {
  const { context, calls, history } = createSearchView({
    currentGroup: "search",
    previousGroup: "Cruise",
    previousQuery: "StoppingAccel",
  });
  await context.applySettingInlineSearch("");
  assert.ok(calls.some((call) => call.join() === "exit,true"));
  assert.ok(calls.some((call) => call[0] === "renderGroups"));
  assert.equal(calls.some((call) => call[0] === "screen"), false);
  assert.equal(calls.some((call) => call[0] === "activate"), false);
  assert.equal(context.CURRENT_GROUP, "Cruise");
  assert.equal(history.state.screen, "groups");
});

test("clearing the query in the split layout restores the previous group pane", async () => {
  const { context, calls } = createSearchView({
    split: true,
    currentGroup: "search",
    previousGroup: "Cruise",
    previousQuery: "StoppingAccel",
  });
  await context.applySettingInlineSearch("");
  assert.ok(calls.some((call) => call.join() === "exit,true"));
  assert.ok(calls.some((call) => call.join() === "activate,Cruise"));
});

test("clearing without a previous group falls back to favorites in the split layout", async () => {
  const { context, calls } = createSearchView({
    split: true,
    currentGroup: "search",
    previousGroup: null,
    previousQuery: "StoppingAccel",
  });
  await context.applySettingInlineSearch("");
  assert.ok(calls.some((call) => call.join() === "activate,favorites"));
});
