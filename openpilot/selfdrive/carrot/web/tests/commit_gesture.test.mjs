import assert from "node:assert/strict";
import test from "node:test";

import {
  COMMIT_GESTURE_HOLD_DELAY_MS,
  COMMIT_GESTURE_MOVE_TOLERANCE,
  createCommitGesture,
} from "../src/ui/components/commit_gesture.js";

function createFakeElement() {
  const handlers = new Map();
  const classes = new Set();
  const captured = new Set();
  return {
    handlers,
    classes,
    captured,
    addEventListener(type, handler) {
      if (!handlers.has(type)) handlers.set(type, new Set());
      handlers.get(type).add(handler);
    },
    removeEventListener(type, handler) {
      handlers.get(type)?.delete(handler);
    },
    setPointerCapture(id) { captured.add(id); },
    releasePointerCapture(id) { captured.delete(id); },
    classList: {
      add: (name) => classes.add(name),
      remove: (name) => classes.delete(name),
    },
    emit(type, event = {}) {
      const detail = { pointerId: 1, clientX: 0, clientY: 0, button: 0, ...event };
      detail.stopPropagation = detail.stopPropagation || (() => {});
      [...(handlers.get(type) || [])].forEach((handler) => handler(detail));
    },
  };
}

function createScheduler() {
  let id = 0;
  const timers = new Map();
  return {
    scheduler: {
      setTimeout(fn, delay) {
        id += 1;
        timers.set(id, { fn, delay });
        return id;
      },
      clearTimeout(timerId) { timers.delete(timerId); },
    },
    pending: () => timers.size,
    // Fires every timer currently scheduled, once.
    flush() {
      [...timers.entries()].forEach(([timerId, timer]) => {
        timers.delete(timerId);
        timer.fn();
      });
    },
  };
}

function createHarness(options = {}) {
  const element = createFakeElement();
  const clock = { value: 1_000 };
  const scheduler = createScheduler();
  const commits = [];
  const gesture = createCommitGesture(element, {
    onCommit: () => commits.push("commit"),
    onHoldStep: () => commits.push("hold"),
    now: () => clock.value,
    scheduler: scheduler.scheduler,
    ...options,
  });
  return { element, clock, scheduler, commits, gesture };
}

test("a target element and an onCommit callback are required", () => {
  assert.throws(() => createCommitGesture(null, { onCommit: () => {} }), TypeError);
  assert.throws(() => createCommitGesture(createFakeElement(), {}), TypeError);
});

// The regression this whole contract exists for.
test("pressing does not commit; only releasing does", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", { clientX: 50, clientY: 50 });
  assert.deepEqual(commits, [], "touching a control must never write a value");

  element.emit("pointerup", { clientX: 50, clientY: 50 });
  assert.deepEqual(commits, ["commit"]);
});

test("a scroll that starts on the control commits nothing", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", { clientX: 50, clientY: 50 });
  element.emit("pointermove", { clientX: 50, clientY: 50 + COMMIT_GESTURE_MOVE_TOLERANCE + 1 });
  element.emit("pointerup", { clientX: 50, clientY: 200 });

  assert.deepEqual(commits, []);
});

test("movement within the tolerance still counts as a tap", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", { clientX: 50, clientY: 50 });
  element.emit("pointermove", { clientX: 50 + COMMIT_GESTURE_MOVE_TOLERANCE, clientY: 50 });
  element.emit("pointerup", { clientX: 50 + COMMIT_GESTURE_MOVE_TOLERANCE, clientY: 50 });

  assert.deepEqual(commits, ["commit"]);
});

test("horizontal drift beyond the tolerance cancels too", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", { clientX: 50, clientY: 50 });
  element.emit("pointermove", { clientX: 50 + COMMIT_GESTURE_MOVE_TOLERANCE + 1, clientY: 50 });
  element.emit("pointerup", {});

  assert.deepEqual(commits, []);
});

// The browser fires pointercancel when it takes the gesture over to scroll.
test("pointercancel from the browser taking over the scroll commits nothing", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", { clientX: 50, clientY: 50 });
  element.emit("pointercancel", {});
  element.emit("pointerup", {});

  assert.deepEqual(commits, []);
});

test("losing the pointer capture cancels the press", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", {});
  element.emit("lostpointercapture", {});
  element.emit("pointerup", {});

  assert.deepEqual(commits, []);
});

test("a tap commits exactly once even though a click follows the release", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", {});
  element.emit("pointerup", {});
  element.emit("click", {});

  assert.deepEqual(commits, ["commit"], "the click echo must not double-commit");
});

test("a cancelled press also swallows the click echo", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", { clientX: 0, clientY: 0 });
  element.emit("pointermove", { clientX: 0, clientY: 99 });
  element.emit("pointerup", {});
  element.emit("click", {});

  assert.deepEqual(commits, []);
});

// Enter/Space on a button produce a click with no pointer sequence.
test("keyboard activation still commits", () => {
  const { element, commits } = createHarness();

  element.emit("click", {});
  assert.deepEqual(commits, ["commit"]);
});

test("a keyboard click after the suppression window has passed commits", () => {
  const { element, clock, commits } = createHarness();

  element.emit("pointerdown", {});
  element.emit("pointerup", {});
  clock.value += 5_000;
  element.emit("click", {});

  assert.deepEqual(commits, ["commit", "commit"]);
});

test("holding repeats and the release adds no extra step", () => {
  const { element, scheduler, commits } = createHarness({
    holdDelayMs: COMMIT_GESTURE_HOLD_DELAY_MS,
  });

  element.emit("pointerdown", {});
  scheduler.flush();            // hold delay elapses -> first repeat fires immediately
  scheduler.flush();            // one more repeat interval
  element.emit("pointerup", {});

  assert.deepEqual(commits, ["hold", "hold"], "a release after a hold must not commit again");
});

test("moving during a hold stops the repeat", () => {
  const { element, scheduler, commits } = createHarness({
    holdDelayMs: COMMIT_GESTURE_HOLD_DELAY_MS,
  });

  element.emit("pointerdown", { clientX: 0, clientY: 0 });
  scheduler.flush();
  element.emit("pointermove", { clientX: 0, clientY: 99 });
  scheduler.flush();

  assert.deepEqual(commits, ["hold"], "no further repeats after the gesture is cancelled");
  assert.equal(scheduler.pending(), 0);
});

test("hold repeat is off unless a delay is configured", () => {
  const { element, scheduler, commits } = createHarness();

  element.emit("pointerdown", {});
  assert.equal(scheduler.pending(), 0);
  scheduler.flush();
  element.emit("pointerup", {});

  assert.deepEqual(commits, ["commit"]);
});

test("non-primary buttons are ignored", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", { button: 2 });
  element.emit("pointerup", {});

  assert.deepEqual(commits, []);
});

test("events from a different pointer are ignored while one is pressed", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", { pointerId: 1, clientX: 0, clientY: 0 });
  element.emit("pointermove", { pointerId: 2, clientX: 0, clientY: 999 });
  element.emit("pointerup", { pointerId: 2 });
  assert.deepEqual(commits, [], "the second pointer must not commit the first press");

  element.emit("pointerup", { pointerId: 1 });
  assert.deepEqual(commits, ["commit"]);
});

test("the pressed class is applied while held and removed on release", () => {
  const { element } = createHarness({ pressedClass: "is-holding" });

  element.emit("pointerdown", {});
  assert.equal(element.classes.has("is-holding"), true);
  element.emit("pointerup", {});
  assert.equal(element.classes.has("is-holding"), false);
});

test("pointer capture is taken on press and released on the way out", () => {
  const { element } = createHarness();

  element.emit("pointerdown", { pointerId: 7 });
  assert.equal(element.captured.has(7), true);
  element.emit("pointerup", { pointerId: 7 });
  assert.equal(element.captured.has(7), false);
});

test("destroy detaches every listener and clears pending timers", () => {
  const { element, scheduler, gesture, commits } = createHarness({
    holdDelayMs: COMMIT_GESTURE_HOLD_DELAY_MS,
  });

  element.emit("pointerdown", {});
  assert.equal(gesture.pressed, true);
  gesture.destroy();

  assert.equal(gesture.pressed, false);
  assert.equal(scheduler.pending(), 0);
  element.emit("pointerup", {});
  element.emit("click", {});
  assert.deepEqual(commits, []);
});

test("a fresh press supersedes one that never ended", () => {
  const { element, commits } = createHarness();

  element.emit("pointerdown", { clientX: 0, clientY: 0 });
  element.emit("pointerdown", { clientX: 80, clientY: 80 });
  element.emit("pointerup", { clientX: 80, clientY: 80 });

  assert.deepEqual(commits, ["commit"]);
});
