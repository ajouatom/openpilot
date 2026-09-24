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

function createView({ split = false, detail = null, group = "Cruise", previousPage = "tools" } = {}) {
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
