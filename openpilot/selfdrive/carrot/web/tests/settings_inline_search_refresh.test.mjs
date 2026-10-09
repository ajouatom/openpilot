import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";

const settings = readFileSync(new URL("../js/pages/setting.js", import.meta.url), "utf8");
function functionSource(name) {
  const start = settings.search(new RegExp(`^(?:async )?function ${name}\\(`, "m"));
  assert.notEqual(start, -1);
  return settings.slice(start, settings.indexOf("\n}", start) + 2);
}

function createVisibleSearchSurface() {
  const calls = [];
  const box = { dataset: { renderedGroup: "search", renderedSearchQuery: "감속" }, childElementCount: 1 };
  const content = { classList: { contains: () => true, add() {} } };
  const elements = { carrotTabContent: content, settingInlineSearchResults: box,
    items: { dataset: {}, childElementCount: 0 } };
  const context = vm.createContext({
    CURRENT_PAGE: "setting", CURRENT_GROUP: "search", CURRENT_SETTING_DETAIL: null,
    SETTING_INLINE_SEARCH_GROUP: "search", settingInlineSearchQuery: "감속",
    screenGroups: { style: { display: "" }, classList: { contains: () => false } },
    document: { hidden: false, getElementById: (id) => elements[id] },
    isCompactLandscapeMode: () => false, isSettingItemsScreenActive: () => false,
    isCarrotSettingTabActive: () => true, getSettingProfileByGroup: () => null,
    mountSettingInlineSearch() {}, scheduleSettingLiveRefresh: (delay) => calls.push(delay),
    renderItems: async () => { throw new Error("cached rows should keep their DOM"); },
  });
  for (const name of ["isSettingInlineSearchSurfaceActive", "getSettingItemRenderContainer",
    "hasRenderedInlineSearchResults", "hasRenderedSettingItems", "shouldRefreshSettingValues",
    "getSettingInlineSearchResultsBox", "showSettingInlineSearchResultsInGroups"]) {
    vm.runInContext(functionSource(name), context);
  }
  return { context, calls, box };
}

test("visible inline results receive live device values without an items-screen render", () => {
  const { context, box } = createVisibleSearchSurface();
  assert.equal(context.shouldRefreshSettingValues(), true);
  box.dataset.renderedSearchQuery = "previous query";
  assert.equal(context.shouldRefreshSettingValues(), false);
  box.dataset.renderedSearchQuery = "감속";
  context.screenGroups.style.display = "none";
  assert.equal(context.shouldRefreshSettingValues(), false);
});

test("returning to cached search rows restarts immediate live value refresh", async () => {
  const { context, calls } = createVisibleSearchSurface();
  assert.equal(await context.showSettingInlineSearchResultsInGroups(), true);
  assert.deepEqual(calls, [0]);
});

test("restored values can rerender inline rows while the items screen is hidden", () => {
  const { context, box } = createVisibleSearchSurface();
  let onRestore;
  let rendered;
  Object.assign(context, {
    settingRestoreRefreshTimer: null,
    settingValueRepository: { applyValues() {} },
    applyRestoredSettingValuesToRenderedItems() {},
    getSettingGroupParamNames: () => ["StoppingAccel"],
    getSettingItemsScrollTop: () => 12,
    window: { addEventListener: (_name, handler) => { onRestore = handler; }, setTimeout: (fn) => { fn(); return 1; } },
    renderItems: async (_group, options) => { rendered = options; },
  });
  const start = settings.indexOf('window.addEventListener("carrot:paramsrestored"');
  const end = settings.indexOf('\nwindow.addEventListener("resize"', start);
  vm.runInContext(settings.slice(start, end), context);
  onRestore({ detail: { values: { StoppingAccel: 1 } } });
  assert.equal(rendered.container, box);
  assert.equal(rendered.allowHidden, true);
  assert.equal(rendered.scrollTop, 12);
});
