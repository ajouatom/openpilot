import assert from "node:assert/strict";
import test from "node:test";

import {
  createSettingContextPanel,
  resolveSettingContextTabIndex,
  SETTING_CONTEXT_TABS,
} from "../src/features/settings/context_panel.js";

class FakeElement {
  constructor(tagName) {
    this.tagName = String(tagName).toUpperCase();
    this.children = [];
    this.attributes = new Map();
    this.listeners = new Map();
    this.className = "";
    this.dataset = {};
    this.hidden = false;
    this.open = false;
    this.tabIndex = 0;
    this.textContent = "";
  }

  appendChild(child) {
    this.children.push(child);
    child.parentElement = this;
    return child;
  }

  append(...children) {
    children.forEach((child) => this.appendChild(child));
  }

  prepend(child) {
    this.children.unshift(child);
    child.parentElement = this;
  }

  replaceChildren(...children) {
    this.children = [];
    this.append(...children);
  }

  setAttribute(name, value) {
    this.attributes.set(name, String(value));
    if (name === "open") this.open = true;
  }

  getAttribute(name) {
    return this.attributes.get(name) ?? null;
  }

  removeAttribute(name) {
    this.attributes.delete(name);
    if (name === "open") this.open = false;
  }

  addEventListener(type, listener) {
    if (!this.listeners.has(type)) this.listeners.set(type, []);
    this.listeners.get(type).push(listener);
  }

  dispatch(type, event = {}) {
    this.listeners.get(type)?.forEach((listener) => listener({
      preventDefault() {},
      target: this,
      ...event,
    }));
  }

  focus() {
    this.focused = true;
  }

  querySelector(selector) {
    if (!selector.startsWith(".")) return null;
    const className = selector.slice(1);
    const queue = [...this.children];
    while (queue.length) {
      const candidate = queue.shift();
      if (String(candidate.className).split(/\s+/).includes(className)) return candidate;
      queue.push(...(candidate.children || []));
    }
    return null;
  }
}

class FakeDocument {
  createElement(tagName) {
    return new FakeElement(tagName);
  }
}

function translated(key, fallback, params) {
  const labels = {
    setting_context_description: "설명",
    setting_context_popular: "인기값",
    setting_context_history: "이력",
  };
  let output = labels[key] || fallback;
  Object.entries(params || {}).forEach(([name, value]) => {
    output = output.replaceAll(`{${name}}`, String(value));
  });
  return output;
}

test("context panel keeps all three tabs with description selected by default", () => {
  const panel = createSettingContextPanel({
    document: new FakeDocument(),
    text: translated,
    locale: "ko",
  });

  assert.deepEqual(SETTING_CONTEXT_TABS, ["description", "popular", "history"]);
  assert.match(panel.root.children[0].className, /c-segmented-control/);
  assert.match(panel.root.children[0].children[0].className, /c-segmented-control__item/);
  assert.equal(panel.root.dataset.activePanel, "description");
  assert.equal(panel.getPanel("description").hidden, false);
  assert.equal(panel.getPanel("popular").hidden, true);
  assert.equal(panel.getPanel("history").hidden, true);

  panel.setEmpty("popular", "데이터 없음");
  panel.activate("popular");
  assert.equal(panel.root.dataset.activePanel, "popular");
  assert.equal(panel.getPanel("popular").hidden, false);
  assert.match(panel.getContent("popular").children[0].className, /--empty/);
});

test("keyboard tab movement wraps and supports Home and End", () => {
  assert.equal(resolveSettingContextTabIndex(0, "ArrowLeft"), 2);
  assert.equal(resolveSettingContextTabIndex(2, "ArrowRight"), 0);
  assert.equal(resolveSettingContextTabIndex(1, "Home"), 0);
  assert.equal(resolveSettingContextTabIndex(1, "End"), 2);
  assert.equal(resolveSettingContextTabIndex(1, "Enter"), 1);
});

test("history count changes without removing the history menu", () => {
  const panel = createSettingContextPanel({
    document: new FakeDocument(),
    text: translated,
    locale: "ko",
  });
  const history = new FakeElement("div");
  panel.setHistoryContent(history, null, { count: 2, summaryCount: 2 });

  const count = panel.root.children[0].children[2].querySelector(".setting-context__tab-count");
  assert.equal(count.textContent, "2");
  assert.equal(panel.getPanel("history").getAttribute("role"), "tabpanel");
});
