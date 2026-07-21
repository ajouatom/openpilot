import {
  COMMIT_GESTURE_HOLD_DELAY_MS,
  COMMIT_GESTURE_HOLD_INTERVAL_MS,
  createCommitGesture,
} from "../commit_gesture.js";

/**
 * Behaviour for the −/value/+ control and its slider.
 *
 * This module owns the gesture contract and the step arithmetic only. The
 * markup is still produced by the settings renderer, so the DOM and CSS are
 * untouched; moving the element creation in here is a later step.
 *
 * Step size intentionally comes from the caller: it is the per-parameter
 * multiplier behind the `x1` button, not the parameter's own `unit`.
 */

function clampValue(value, min, max) {
  let next = value;
  if (Number.isFinite(min)) next = Math.max(next, min);
  if (Number.isFinite(max)) next = Math.min(next, max);
  return next;
}

export function nextStepValue(options = {}) {
  const step = Number(options.step);
  const sign = Number(options.sign);
  const min = Number(options.min);
  const max = Number(options.max);

  let current = Number(options.current);
  if (!Number.isFinite(current)) current = Number(options.fallback);
  if (!Number.isFinite(current)) current = Number.isFinite(min) ? min : 0;
  if (!Number.isFinite(step) || !Number.isFinite(sign)) return current;

  let next = clampValue(current + sign * step, min, max);
  // Matches the long-standing renderer rule: only snap to whole numbers when
  // the parameter and the step are themselves whole numbers.
  if (Number.isInteger(min) && Number.isInteger(max) && Number.isInteger(step)) {
    next = Math.round(next);
  }
  return next;
}

export function createNumericStepper(options = {}) {
  const commit = options.commit;
  if (typeof commit !== "function") throw new TypeError("commit must be a function");

  const getValue = typeof options.getValue === "function" ? options.getValue : () => NaN;
  const getStep = typeof options.getStep === "function" ? options.getStep : () => 1;
  const getRange = typeof options.getRange === "function" ? options.getRange : () => ({});
  const getFallback = typeof options.getFallback === "function" ? options.getFallback : () => NaN;
  const holdDelayMs = Number.isFinite(options.holdDelayMs) ? options.holdDelayMs : COMMIT_GESTURE_HOLD_DELAY_MS;
  const holdIntervalMs = Number.isFinite(options.holdIntervalMs)
    ? options.holdIntervalMs
    : COMMIT_GESTURE_HOLD_INTERVAL_MS;

  // One in-flight write at a time, so a fast repeat cannot interleave commits
  // and read a value that has not been written back yet.
  let busy = false;

  async function applyDelta(sign) {
    if (busy) return;
    busy = true;
    try {
      const range = getRange() || {};
      const next = nextStepValue({
        current: getValue(),
        fallback: getFallback(),
        sign,
        step: getStep(),
        min: range.min,
        max: range.max,
      });
      await commit(next);
    } finally {
      busy = false;
    }
  }

  const gestures = [];
  function bindButton(element, sign) {
    if (!element) return;
    gestures.push(createCommitGesture(element, {
      onCommit: () => { applyDelta(sign); },
      holdDelayMs,
      holdIntervalMs,
      pressedClass: options.pressedClass,
      now: options.now,
      scheduler: options.scheduler,
    }));
  }

  bindButton(options.minusButton, -1);
  bindButton(options.plusButton, +1);

  return Object.freeze({
    applyDelta,
    destroy() {
      gestures.forEach((gesture) => gesture.destroy());
      gestures.length = 0;
    },
  });
}
