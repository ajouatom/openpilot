import assert from "node:assert/strict";
import test from "node:test";

import {
  SETTING_CONTROL_KINDS,
  createSettingControl,
} from "../src/ui/components/setting_row/control.js";

// Minimal stand-in for the pieces of the DOM this builder touches. Enough to
// assert the produced structure without pulling in a full DOM implementation.
function createDocument() {
  function createElement(tag) {
    return {
      tagName: tag.toUpperCase(),
      className: "",
      textContent: undefined,
      type: undefined,
      dataset: {},
      attributes: {},
      children: [],
      appendChild(child) { this.children.push(child); return child; },
      setAttribute(name, value) { this.attributes[name] = String(value); },
      getAttribute(name) { return this.attributes[name] ?? null; },
    };
  }
  return { createElement };
}

function build(kind, options = {}) {
  return createSettingControl({
    document: createDocument(),
    kind,
    label: "모델 주행속도",
    valueLabel: "Edit value",
    ...options,
  });
}

const classesOf = (element) => element.children.map((child) => child.className);

test("a document and a known kind are required", () => {
  assert.throws(() => createSettingControl({ kind: "toggle" }), TypeError);
  assert.throws(() => createSettingControl({ document: createDocument(), kind: "nope" }), TypeError);
  assert.deepEqual(SETTING_CONTROL_KINDS, ["toggle", "segmented", "select", "stepper", "slider"]);
});

test("the container class follows the kind, and the stepper reuses the value layout", () => {
  assert.equal(build("toggle").ctrl.className, "ctrl ctrl--toggle");
  assert.equal(build("segmented").ctrl.className, "ctrl ctrl--segmented");
  assert.equal(build("select").ctrl.className, "ctrl ctrl--select");
  assert.equal(build("slider").ctrl.className, "ctrl ctrl--slider");
  assert.equal(build("stepper").ctrl.className, "ctrl ctrl--value");
});

test("the value button is a compact surface only for the stepper", () => {
  assert.equal(build("toggle").value.className, "value-surface val");
  assert.equal(build("stepper").value.className, "value-surface val setting-value-compact");
  assert.equal(build("toggle").value.getAttribute("aria-label"), "Edit value");
  assert.equal(build("toggle").value.type, "button");
});

test("a toggle produces the shared switch primitive followed by the value", () => {
  const refs = build("toggle");
  assert.deepEqual(classesOf(refs.ctrl), ["c-switch", "value-surface val"]);

  const [switchLabel] = refs.ctrl.children;
  assert.deepEqual(classesOf(switchLabel), ["c-switch__input", "c-switch__track"]);
  assert.equal(refs.toggleInput.type, "checkbox");
  assert.equal(refs.toggleInput.getAttribute("aria-label"), "모델 주행속도");
});

test("segments carry their value, label and initial pressed state", () => {
  const refs = build("segmented", {
    optionValues: [0, 1, 2],
    optionLabel: (value) => `옵션${value}`,
  });

  assert.deepEqual(classesOf(refs.ctrl), ["setting-segments", "value-surface val"]);
  assert.equal(refs.segmentGroup, refs.ctrl.children[0]);
  assert.equal(refs.segmentButtons.length, 3);
  assert.deepEqual(refs.segmentButtons.map((b) => b.dataset.value), ["0", "1", "2"]);
  assert.deepEqual(refs.segmentButtons.map((b) => b.textContent), ["옵션0", "옵션1", "옵션2"]);
  assert.deepEqual(refs.segmentButtons.map((b) => b.getAttribute("aria-pressed")), ["false", "false", "false"]);
});

test("a segmented control with no options still builds an empty group", () => {
  const refs = build("segmented");
  assert.equal(refs.segmentButtons.length, 0);
  assert.deepEqual(classesOf(refs.ctrl), ["setting-segments", "value-surface val"]);
});

test("a select announces that it opens a dialog", () => {
  const refs = build("select");
  assert.deepEqual(classesOf(refs.ctrl), ["setting-select setting-select--button", "value-surface val"]);
  assert.equal(refs.selectInput.getAttribute("aria-haspopup"), "dialog");
  assert.equal(refs.selectInput.getAttribute("aria-label"), "모델 주행속도");
});

// The value sits between the buttons, which the stylesheet's grid relies on.
test("a stepper places the value between the two arrows", () => {
  const refs = build("stepper", { previousLabel: "이전 값", nextLabel: "다음 값" });

  assert.deepEqual(classesOf(refs.ctrl), [
    "smallBtn setting-value-arrow setting-value-arrow--prev",
    "value-surface val setting-value-compact",
    "smallBtn setting-value-arrow setting-value-arrow--next",
  ]);
  assert.equal(refs.minusButton.textContent, "-");
  assert.equal(refs.plusButton.textContent, "+");
  assert.equal(refs.minusButton.getAttribute("aria-label"), "이전 값");
  assert.equal(refs.plusButton.getAttribute("aria-label"), "다음 값");
});

test("a slider puts the track first, then minus, value and plus", () => {
  const refs = build("slider", { min: -120, max: 120, step: 1 });

  assert.deepEqual(classesOf(refs.ctrl), [
    "setting-slider",
    "smallBtn setting-step setting-step--minus",
    "value-surface val",
    "smallBtn setting-step setting-step--plus",
  ]);
  assert.equal(refs.sliderInput.type, "range");
  assert.equal(refs.sliderInput.min, "-120");
  assert.equal(refs.sliderInput.max, "120");
  assert.equal(refs.sliderInput.step, "1");
  assert.equal(refs.sliderInput.getAttribute("aria-label"), "모델 주행속도");
});

// The multiplier button is rendered into the row's footer, not the control.
test("both numeric kinds hand back a multiplier button that is not appended", () => {
  for (const kind of ["stepper", "slider"]) {
    const refs = build(kind);
    assert.equal(refs.unitButton.className, "setting-unit-cycle");
    assert.equal(refs.ctrl.children.includes(refs.unitButton), false);
  }
});

test("elements that do not apply to a kind come back as null", () => {
  const refs = build("toggle");
  assert.equal(refs.selectInput, null);
  assert.equal(refs.sliderInput, null);
  assert.equal(refs.minusButton, null);
  assert.equal(refs.plusButton, null);
  assert.equal(refs.unitButton, null);
  assert.equal(refs.segmentGroup, null);
  assert.deepEqual(refs.segmentButtons, []);
});
