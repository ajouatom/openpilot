import assert from "node:assert/strict";
import test from "node:test";

import {
  createNumericStepper,
  nextStepValue,
} from "../src/ui/components/numeric_stepper/numeric_stepper.js";
import { installNumericStepperFacade } from "../src/ui/components/numeric_stepper/facade.js";

function createFakeButton() {
  const handlers = new Map();
  return {
    addEventListener(type, handler) {
      if (!handlers.has(type)) handlers.set(type, new Set());
      handlers.get(type).add(handler);
    },
    removeEventListener(type, handler) { handlers.get(type)?.delete(handler); },
    classList: { add() {}, remove() {} },
    emit(type, event = {}) {
      const detail = { pointerId: 1, clientX: 0, clientY: 0, button: 0, stopPropagation() {}, ...event };
      [...(handlers.get(type) || [])].forEach((handler) => handler(detail));
    },
    tap() {
      this.emit("pointerdown", {});
      this.emit("pointerup", {});
    },
  };
}

test("a step moves by the supplied multiplier, not by one", () => {
  assert.equal(nextStepValue({ current: 50, sign: 1, step: 10, min: 0, max: 100 }), 60);
  assert.equal(nextStepValue({ current: 50, sign: -1, step: 10, min: 0, max: 100 }), 40);
  assert.equal(nextStepValue({ current: 50, sign: 1, step: 1, min: 0, max: 100 }), 51);
});

test("results are clamped to the parameter range", () => {
  assert.equal(nextStepValue({ current: 98, sign: 1, step: 5, min: 0, max: 100 }), 100);
  assert.equal(nextStepValue({ current: 2, sign: -1, step: 5, min: 0, max: 100 }), 0);
  assert.equal(nextStepValue({ current: 0, sign: -1, step: 1, min: -120, max: 120 }), -1);
});

test("whole-number parameters snap to integers, fractional steps do not", () => {
  assert.equal(nextStepValue({ current: 0.4, sign: 1, step: 1, min: 0, max: 10 }), 1);
  assert.equal(
    nextStepValue({ current: 1, sign: 1, step: 0.5, min: 0, max: 10 }),
    1.5,
    "a fractional step must keep its precision",
  );
  assert.equal(nextStepValue({ current: 1, sign: 1, step: 1, min: 0, max: 2.5 }), 2);
});

test("an unreadable current value falls back to the parameter default", () => {
  assert.equal(nextStepValue({ current: NaN, fallback: 6, sign: 1, step: 1, min: 3, max: 20 }), 7);
  assert.equal(nextStepValue({ current: "x", fallback: 6, sign: -1, step: 1, min: 3, max: 20 }), 5);
});

test("with neither a value nor a default the range minimum is used", () => {
  assert.equal(nextStepValue({ current: NaN, fallback: NaN, sign: 1, step: 1, min: 3, max: 20 }), 4);
});

test("a malformed step leaves the value untouched", () => {
  assert.equal(nextStepValue({ current: 42, sign: 1, step: NaN, min: 0, max: 100 }), 42);
  assert.equal(nextStepValue({ current: 42, sign: NaN, step: 1, min: 0, max: 100 }), 42);
});

test("the stepper requires a commit function", () => {
  assert.throws(() => createNumericStepper({}), TypeError);
});

test("tapping minus and plus commits the neighbouring values", async () => {
  const minusButton = createFakeButton();
  const plusButton = createFakeButton();
  const commits = [];
  let value = 50;

  createNumericStepper({
    minusButton,
    plusButton,
    getValue: () => value,
    getStep: () => 10,
    getRange: () => ({ min: 0, max: 100 }),
    commit: (next) => { commits.push(next); value = next; },
  });

  minusButton.tap();
  await Promise.resolve();
  plusButton.tap();
  await Promise.resolve();

  assert.deepEqual(commits, [40, 50]);
});

test("a press that turns into a scroll commits nothing", async () => {
  const minusButton = createFakeButton();
  const commits = [];

  createNumericStepper({
    minusButton,
    getValue: () => 0,
    getStep: () => 1,
    getRange: () => ({ min: -120, max: 120 }),
    commit: (next) => commits.push(next),
  });

  minusButton.emit("pointerdown", { clientX: 0, clientY: 0 });
  minusButton.emit("pointermove", { clientX: 0, clientY: 120 });
  minusButton.emit("pointerup", {});
  await Promise.resolve();

  assert.deepEqual(commits, [], "ApplyModelSpeed must not slip from 0 to -1 on a scroll");
});

test("the step size is read at press time so the multiplier button applies immediately", async () => {
  const plusButton = createFakeButton();
  const commits = [];
  let step = 1;

  createNumericStepper({
    plusButton,
    getValue: () => 0,
    getStep: () => step,
    getRange: () => ({ min: 0, max: 100 }),
    commit: (next) => commits.push(next),
  });

  plusButton.tap();
  await Promise.resolve();
  step = 10;
  plusButton.tap();
  await Promise.resolve();

  assert.deepEqual(commits, [1, 10]);
});

test("an in-flight commit blocks a second one from interleaving", async () => {
  const plusButton = createFakeButton();
  const commits = [];
  let release;
  const gate = new Promise((resolve) => { release = resolve; });

  createNumericStepper({
    plusButton,
    getValue: () => 0,
    getStep: () => 1,
    getRange: () => ({ min: 0, max: 100 }),
    commit: async (next) => { commits.push(next); await gate; },
  });

  plusButton.tap();
  plusButton.tap();
  release();
  await Promise.resolve();

  assert.deepEqual(commits, [1], "the second tap must be dropped while the first write is pending");
});

test("destroy stops the buttons from committing", async () => {
  const plusButton = createFakeButton();
  const commits = [];
  const stepper = createNumericStepper({
    plusButton,
    getValue: () => 0,
    getStep: () => 1,
    getRange: () => ({ min: 0, max: 100 }),
    commit: (next) => commits.push(next),
  });

  stepper.destroy();
  plusButton.tap();
  await Promise.resolve();

  assert.deepEqual(commits, []);
});

test("the facade publishes the stepper on the shared CarrotUI namespace", () => {
  const target = {};
  const api = installNumericStepperFacade(target);

  assert.equal(target.CarrotUI.numericStepper, api);
  assert.equal(target.CarrotNumericStepper, api);
  assert.equal(typeof api.create, "function");
  assert.equal(typeof api.createGesture, "function");
  assert.equal(api.nextStepValue, nextStepValue);
});

test("the facade preserves other components already on the namespace", () => {
  const existing = { segmentedControl: { create() {} } };
  const target = { CarrotUI: existing };
  installNumericStepperFacade(target);

  assert.equal(target.CarrotUI.segmentedControl, existing.segmentedControl);
  assert.equal(typeof target.CarrotUI.numericStepper.create, "function");
});

test("the facade rejects an unusable target", () => {
  assert.throws(() => installNumericStepperFacade(null), TypeError);
  assert.throws(() => installNumericStepperFacade({}, { createNumericStepper: "nope" }), TypeError);
});
