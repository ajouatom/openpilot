import assert from "node:assert/strict";
import test from "node:test";

import { SETTING_DERIVED_IDS } from "../src/features/settings/derived_model.js";
import { createSettingsGroupRenderPlan, renderSettingsGroupList } from "../src/features/settings/dom_renderer.js";

const IDS = SETTING_DERIVED_IDS;

function createFakeDom() {
  function createNode(tag) {
    return {
      tag,
      className: "",
      title: "",
      type: "",
      textContent: "",
      children: [],
      dataset: {},
      attributes: {},
      onclick: null,
      ownerDocument: null,
      style: {
        values: {},
        setProperty(name, value) {
          this.values[name] = String(value);
        },
        removeProperty(name) {
          delete this.values[name];
        },
      },
      appendChild(child) {
        child.parentElement = this;
        this.children.push(child);
        return child;
      },
      replaceChildren(...nodes) {
        this.children = [];
        for (const node of nodes) this.appendChild(node);
      },
      setAttribute(name, value) {
        this.attributes[name] = String(value);
      },
      removeAttribute(name) {
        delete this.attributes[name];
      },
    };
  }

  const documentRef = {
    createElement(tag) {
      const node = createNode(tag);
      node.ownerDocument = documentRef;
      return node;
    },
    createDocumentFragment() {
      return createNode("#fragment");
    },
  };
  const root = documentRef.createElement("div");
  root.replaceChildren = function replaceChildren(...nodes) {
    this.children = [];
    for (const node of nodes) {
      if (node?.tag === "#fragment") this.children.push(...node.children);
      else this.children.push(node);
    }
  };
  return { root, documentRef };
}

function planFor(currentGroup = "") {
  return createSettingsGroupRenderPlan({
    groups: [
      { group: IDS.favoritesGroup },
      { group: IDS.searchGroup },
      { group: `${IDS.categoryDividerPrefix}DRIVING`, label: "주행 제어" },
      { group: "SPEED" },
    ],
    currentGroup,
    ids: IDS,
    getGroupLabel: (group) => `label:${group}`,
    profilesLabel: "프로필",
  });
}

test("the search slot reveals with the group list and keeps its own class", () => {
  const { root } = createFakeDom();
  renderSettingsGroupList(root, planFor("SPEED"), { animate: true });

  assert.equal(root.children.length, 4);
  const slot = root.children[1];
  assert.equal(slot.className, "setting-inline-search-slot ui-stagger-item");
  assert.equal(slot.style.values["--i"], "1");
  assert.match(root.children[0].className, /ui-stagger-item/);
  assert.match(root.children[2].className, /ui-stagger-item/);
});

test("a reuse pass clears the stagger class and index from the slot", () => {
  const { root } = createFakeDom();
  const plan = planFor("SPEED");
  renderSettingsGroupList(root, plan, { animate: true });
  const slot = root.children[1];

  const result = renderSettingsGroupList(root, plan, { animate: false });
  assert.equal(result.reused, true);
  assert.equal(slot.className, "setting-inline-search-slot");
  assert.equal("--i" in slot.style.values, false);
});
