import assert from "node:assert/strict";
import test from "node:test";

import { createSettingInlineSearchView } from "../src/features/settings/search/inline.js";
import { createSettingSearchPanelView } from "../src/features/settings/search/panel.js";

function createElement(overrides = {}) {
  const listeners = new Map();
  const classes = new Set();
  const el = {
    hidden: false,
    value: "",
    textContent: "",
    attributes: {},
    children: [],
    parentElement: null,
    classList: {
      add: (...names) => names.forEach((name) => classes.add(name)),
      remove: (...names) => names.forEach((name) => classes.delete(name)),
      contains: (name) => classes.has(name),
      get value() {
        return [...classes].join(" ");
      },
    },
    appendChild(child) {
      child.parentElement = this;
      this.children.push(child);
      return child;
    },
    insertBefore(child) {
      child.parentElement = this;
      this.children.unshift(child);
      return child;
    },
    setAttribute(name, value) {
      this.attributes[name] = String(value);
    },
    removeAttribute(name) {
      delete this.attributes[name];
    },
    addEventListener(type, handler) {
      listeners.set(type, handler);
    },
    dispatch(type, event = {}) {
      listeners.get(type)?.(event);
    },
    focus() {
      this.focused = true;
    },
    select() {
      this.selected = true;
    },
    setSelectionRange() {},
    replaceChildren() {
      this.children = [];
    },
    querySelector() {
      return null;
    },
    ...overrides,
  };
  return el;
}

const tick = () => new Promise((resolve) => setTimeout(resolve, 0));

function inlineHarness(overrides = {}) {
  const ownerDocument = { activeElement: null };
  const form = createElement({ ownerDocument });
  const input = createElement({ ownerDocument });
  const clear = createElement();
  const label = createElement();
  const hint = createElement();
  const status = createElement();
  const applied = [];
  let statusText = "";
  const view = createSettingInlineSearchView({
    elements: { form, input, clear, label, hint, status },
    getLabels: () => ({ label: "설정 찾기", hint: "힌트", placeholder: "예: 감속", clear: "지우기" }),
    getStatusText: () => statusText,
    onApply: (query) => applied.push(query),
    debounceMs: 0,
    ...overrides,
  });
  view.bind();
  return { view, form, input, clear, label, hint, status, applied, setStatusText: (value) => { statusText = value; } };
}

test("inline search applies the trimmed query after the debounce", async () => {
  const { input, applied } = inlineHarness();
  input.value = "  감속  ";
  input.dispatch("input", {});
  assert.deepEqual(applied, []);
  await tick();
  assert.deepEqual(applied, ["감속"]);
});

test("inline search ignores composing input and applies on composition end", async () => {
  const { input, applied } = inlineHarness();
  input.value = "감";
  input.dispatch("input", { isComposing: true });
  await tick();
  assert.deepEqual(applied, []);
  input.value = "감속";
  input.dispatch("compositionend", {});
  await tick();
  assert.deepEqual(applied, ["감속"]);
});

test("inline search clear button resets the field, applies empty and refocuses", async () => {
  const { input, clear, applied } = inlineHarness();
  input.value = "감속";
  clear.dispatch("click", {});
  await tick();
  assert.equal(input.value, "");
  assert.deepEqual(applied, [""]);
  assert.equal(input.focused, true);
});

test("inline search chrome follows labels, clear state and status", () => {
  const { view, input, clear, label, hint, status, setStatusText } = inlineHarness();
  input.value = "감속";
  setStatusText("2개 중 2개");
  view.refresh();
  assert.equal(label.textContent, "설정 찾기");
  assert.equal(hint.textContent, "힌트");
  assert.equal(input.placeholder, "예: 감속");
  assert.equal(input.attributes["aria-label"], "설정 찾기");
  assert.equal(clear.hidden, false);
  assert.equal(clear.attributes["aria-label"], "지우기");
  assert.equal(status.textContent, "2개 중 2개");
});

test("inline search mount moves the form into the slot and keeps focus", () => {
  const { view, form, input } = inlineHarness();
  const slot = createElement();
  const ownerDocument = form.ownerDocument;
  ownerDocument.activeElement = input;
  input.selectionStart = 1;
  input.selectionEnd = 2;
  assert.equal(view.isFocused(), true);
  assert.equal(view.mount(slot), true);
  assert.equal(form.parentElement, slot);
  assert.equal(input.focused, true);
});

function panelHarness(overrides = {}) {
  const body = createElement();
  const ownerDocument = { body, activeElement: null };
  const panel = createElement({ ownerDocument });
  const backdrop = createElement({ ownerDocument });
  const form = createElement({ ownerDocument });
  const input = createElement({ ownerDocument });
  const results = createElement();
  const opens = [];
  const hiddens = [];
  const renders = [];
  const selected = [];
  const labels = {
    all: "모든 설정",
    placeholder: "설정 검색",
    profilePlaceholder: "프로필에서 검색",
    empty: "결과 없음",
    sourceCarrot: "당근파일럿",
    sourceProfile: "프로필",
  };
  const view = createSettingSearchPanelView({
    elements: { panel, backdrop, form, input, results },
    getEntries: () => [{ source: "carrot", haystack: "decel" }],
    getLabels: () => labels,
    getProfileName: () => "출퇴근",
    renderResults: (options) => {
      renders.push(options);
      return 1;
    },
    onSelect: (entry) => selected.push(entry),
    onOpenChange: (open) => opens.push(open),
    onHidden: () => hiddens.push(true),
    debounceMs: 0,
    closeDelayMs: 0,
    ...overrides,
  });
  view.bind();
  return { view, panel, backdrop, form, input, results, opens, hiddens, renders, selected, labels };
}

test("panel show/hide toggles visibility, chrome and the open callback", async () => {
  const { view, panel, backdrop, input, opens, hiddens } = panelHarness();
  view.setScope({ type: "profile", profileId: "p1" });
  input.value = "decel";
  view.show();
  assert.equal(view.isOpen(), true);
  assert.equal(panel.hidden, false);
  assert.equal(backdrop.hidden, false);
  assert.equal(panel.classList.contains("is-open"), true);
  assert.equal(backdrop.classList.contains("is-open"), true);
  assert.equal(input.placeholder, "프로필에서 검색");
  assert.equal(input.attributes["aria-label"], "출퇴근");
  assert.deepEqual(opens, [true]);

  view.hide();
  assert.equal(view.isOpen(), false);
  assert.equal(panel.classList.contains("is-open"), false);
  assert.equal(backdrop.classList.contains("is-open"), false);
  assert.equal(input.value, "");
  assert.equal(input.placeholder, "설정 검색");
  assert.equal(input.attributes["aria-label"], undefined);
  assert.deepEqual(view.getScope(), { type: "all", profileId: "" });
  assert.deepEqual(opens, [true, false]);

  // The DOM leaves the layout only after the close fade, then the hidden
  // callback lets the page release the modal body lock.
  assert.equal(panel.hidden, false);
  assert.deepEqual(hiddens, []);
  await tick();
  assert.equal(panel.hidden, true);
  assert.equal(backdrop.hidden, true);
  assert.deepEqual(hiddens, [true]);
});

test("reopening before the close delay cancels the pending hide", async () => {
  const { view, panel, hiddens } = panelHarness({ closeDelayMs: 20 });
  view.show();
  view.hide();
  view.show();
  await new Promise((resolve) => setTimeout(resolve, 40));
  assert.equal(view.isOpen(), true);
  assert.equal(panel.hidden, false);
  assert.equal(panel.classList.contains("is-open"), true);
  assert.deepEqual(hiddens, []);
});

test("panel renders the current query for its scope after the debounce", async () => {
  const { view, input, renders } = panelHarness();
  view.setScope({ type: "profile", profileId: "p1" });
  view.show();
  renders.length = 0;
  input.value = "decel";
  input.dispatch("input", {});
  await tick();
  assert.equal(renders.length, 1);
  assert.equal(renders[0].query, "decel");
  assert.deepEqual(renders[0].scope, { type: "profile", profileId: "p1" });
  assert.equal(typeof renders[0].highlight, "function");
});

test("panel ignores composing input and renders on composition end", async () => {
  const { view, input, renders } = panelHarness();
  view.show();
  renders.length = 0;

  input.value = "감";
  input.dispatch("input", { isComposing: true });
  await tick();
  assert.equal(renders.length, 0);

  input.value = "감속";
  input.dispatch("compositionend", {});
  await tick();
  assert.equal(renders.length, 1);
  assert.equal(renders[0].query, "감속");
});

test("panel scope normalization falls back to all settings", () => {
  const { view } = panelHarness();
  assert.deepEqual(view.setScope({ type: "profile", profileId: "" }), { type: "all", profileId: "" });
  assert.deepEqual(view.setScope({ type: "profile", profileId: 7 }), { type: "profile", profileId: "7" });
  assert.equal(view.scopeLabel(), "출퇴근");
  view.setScope(null);
  assert.equal(view.scopeLabel(), "모든 설정");
});
