import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";

const root = new URL("../", import.meta.url);
const html = readFileSync(new URL("index.html", root), "utf8");
const source = readFileSync(new URL("js/shared/ui/viewport.js", root), "utf8");
const viewportMeta = html.match(/<meta name="viewport" content="([^"]+)"/)[1];
const keyboardPolicy = viewportMeta.match(/interactive-widget=([\w-]+)/)?.[1] || "resizes-visual";

// Model the documented browser viewport policies, then run the production
// metrics/resolver. HTTP phone connections need this path without VirtualKeyboard.
function createViewport(width, height, { overlay = false } = {}) {
  const tokens = new Map();
  const element = {
    clientWidth: width, clientHeight: height, dataset: {},
    style: { setProperty: (key, value) => tokens.set(key, value), removeProperty: (key) => tokens.delete(key) },
  };
  const vk = overlay ? { boundingRect: { height: 0 }, addEventListener() {} } : undefined;
  const window = {
    innerWidth: width, innerHeight: height,
    visualViewport: { width, height, offsetTop: 0, addEventListener() {} },
    addEventListener() {},
  };
  const context = vm.createContext({
    window, navigator: { virtualKeyboard: vk },
    document: { documentElement: element, querySelector: () => null, addEventListener() {} },
    requestAnimationFrame: () => 1,
  });
  vm.runInContext(source, context);
  function resize(w, h, keyboardHeight = 0) {
    window.innerWidth = element.clientWidth = window.visualViewport.width = w;
    const resizeLayout = !overlay && keyboardPolicy === "resizes-content";
    window.innerHeight = element.clientHeight = resizeLayout ? h - keyboardHeight : h;
    window.visualViewport.height = overlay ? h : h - keyboardHeight;
    if (vk) vk.boundingRect.height = keyboardHeight;
    window.CarrotViewport.updateMetrics();
  }
  return { window, element, tokens, resize };
}

test("portrait phone keyboard shrinks usable height without activating the left rail", () => {
  const view = createViewport(390, 844);
  assert.equal(view.window.CarrotLayout.isWide(), false);
  view.resize(390, 844, 584);
  assert.equal(view.window.CarrotLayout.isWide(), false);
  assert.equal(view.window.CarrotLayout.orientation(), "portrait");
  assert.equal(view.element.dataset.layout, "tall");
  assert.equal(view.element.dataset.kbOpen, "1");
  assert.equal(view.tokens.get("--app-vv-height"), "320px"); // existing minimum
  assert.equal(view.tokens.has("--kb-inset"), false); // no double subtraction
  view.resize(390, 844);
  assert.equal(view.element.dataset.kbOpen, undefined);
  assert.equal(view.tokens.get("--app-vv-height"), "844px");
});

test("real rotation still changes the layout, including while typing", () => {
  const view = createViewport(390, 844);
  view.resize(844, 390, 150);
  assert.equal(view.window.CarrotLayout.isWide(), true);
  assert.equal(view.window.CarrotLayout.orientation(), "landscape");
  view.resize(390, 844, 584);
  assert.equal(view.window.CarrotLayout.isWide(), false);
  assert.equal(view.window.CarrotLayout.orientation(), "portrait");
});

test("large portrait panels retain their wide layout when the keyboard opens", () => {
  const view = createViewport(700, 800);
  view.resize(700, 800, 500);
  assert.equal(view.window.CarrotLayout.isWide(), true);
  assert.equal(view.window.CarrotLayout.orientation(), "portrait");
});

test("secure-context overlay keyboard retains its existing inset handling", () => {
  const view = createViewport(390, 844, { overlay: true });
  view.resize(390, 844, 500);
  assert.equal(view.window.CarrotLayout.isWide(), false);
  assert.equal(view.tokens.get("--app-vv-height"), "844px");
  assert.equal(view.tokens.get("--kb-inset"), "500px");
  view.resize(390, 844);
  assert.equal(view.tokens.get("--kb-inset"), "0px");
});
