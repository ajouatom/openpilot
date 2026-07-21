/**
 * The single input-confirmation contract shared by every value control.
 *
 * A press only records where it started. The value is committed on release,
 * and only when the pointer stayed within `moveTolerance` of that origin. A
 * scroll that happens to begin on a control therefore changes nothing: the
 * browser hands the gesture over with `pointercancel`, and any drag beyond the
 * tolerance cancels on its own.
 *
 * `pointerdown` is deliberately not cancelled. Calling preventDefault() there
 * is what used to stop the list from scrolling while the value had already
 * been written.
 */

export const COMMIT_GESTURE_MOVE_TOLERANCE = 10;
export const COMMIT_GESTURE_HOLD_DELAY_MS = 900;
export const COMMIT_GESTURE_HOLD_INTERVAL_MS = 160;
export const COMMIT_GESTURE_CLICK_SUPPRESS_MS = 700;

function requireElement(element) {
  if (!element || typeof element.addEventListener !== "function") {
    throw new TypeError("A commit gesture target element is required");
  }
  return element;
}

export function createCommitGesture(element, options = {}) {
  const target = requireElement(element);
  const onCommit = options.onCommit;
  if (typeof onCommit !== "function") throw new TypeError("onCommit must be a function");

  const onHoldStep = typeof options.onHoldStep === "function" ? options.onHoldStep : onCommit;
  const holdDelayMs = Number.isFinite(options.holdDelayMs) ? options.holdDelayMs : 0;
  const holdIntervalMs = Number.isFinite(options.holdIntervalMs)
    ? options.holdIntervalMs
    : COMMIT_GESTURE_HOLD_INTERVAL_MS;
  const moveTolerance = Number.isFinite(options.moveTolerance)
    ? options.moveTolerance
    : COMMIT_GESTURE_MOVE_TOLERANCE;
  const clickSuppressMs = Number.isFinite(options.clickSuppressMs)
    ? options.clickSuppressMs
    : COMMIT_GESTURE_CLICK_SUPPRESS_MS;
  const pressedClass = typeof options.pressedClass === "string" ? options.pressedClass : "is-holding";
  const now = typeof options.now === "function" ? options.now : () => Date.now();
  const scheduler = options.scheduler || globalThis;
  const setTimer = scheduler.setTimeout.bind(scheduler);
  const clearTimer = scheduler.clearTimeout.bind(scheduler);

  let press = null;
  let holdTimer = null;
  let repeatTimer = null;
  // A pointer sequence always ends in a click. Keyboard activation (Enter or
  // Space on a button) produces a click with no pointer sequence at all, so the
  // click handler stays as the accessible path and only ignores the echo.
  let suppressClickUntil = 0;

  function clearTimers() {
    if (holdTimer !== null) {
      clearTimer(holdTimer);
      holdTimer = null;
    }
    if (repeatTimer !== null) {
      clearTimer(repeatTimer);
      repeatTimer = null;
    }
  }

  function releaseCapture() {
    if (!press || press.pointerId === undefined || press.pointerId === null) return;
    if (typeof target.releasePointerCapture !== "function") return;
    try {
      target.releasePointerCapture(press.pointerId);
    } catch (_) {
      // The browser may have released the capture already.
    }
  }

  function endPress() {
    clearTimers();
    releaseCapture();
    target.classList?.remove?.(pressedClass);
    press = null;
  }

  function repeat() {
    if (!press) return;
    press.holdFired = true;
    onHoldStep();
    repeatTimer = setTimer(repeat, holdIntervalMs);
  }

  function beginHold() {
    holdTimer = null;
    if (!press) return;
    // The first repeat fires immediately so a hold reacts at the same moment
    // the user expects it to, instead of staying silent for the whole delay.
    repeat();
  }

  function onPointerDown(event) {
    if (event.button !== undefined && event.button !== 0) return;
    endPress();

    press = {
      pointerId: event.pointerId,
      startX: event.clientX,
      startY: event.clientY,
      holdFired: false,
      cancelled: false,
    };
    target.classList?.add?.(pressedClass);

    if (typeof target.setPointerCapture === "function" && event.pointerId !== undefined) {
      try {
        target.setPointerCapture(event.pointerId);
      } catch (_) {
        // Pointer capture is best effort; movement tracking still works without it.
      }
    }

    if (holdDelayMs > 0) holdTimer = setTimer(beginHold, holdDelayMs);
  }

  function onPointerMove(event) {
    if (!press || (event.pointerId !== undefined && event.pointerId !== press.pointerId)) return;
    const dx = Math.abs(event.clientX - press.startX);
    const dy = Math.abs(event.clientY - press.startY);
    if (dx <= moveTolerance && dy <= moveTolerance) return;

    // The press turned into a drag or a scroll: nothing is committed.
    press.cancelled = true;
    suppressClickUntil = now() + clickSuppressMs;
    endPress();
  }

  function onPointerUp(event) {
    if (!press || (event.pointerId !== undefined && event.pointerId !== press.pointerId)) return;
    const shouldCommit = !press.cancelled && !press.holdFired;
    suppressClickUntil = now() + clickSuppressMs;
    endPress();
    if (shouldCommit) onCommit();
  }

  function onPointerCancel() {
    if (!press) return;
    // Fired when the browser takes the gesture over for scrolling.
    press.cancelled = true;
    suppressClickUntil = now() + clickSuppressMs;
    endPress();
  }

  function onClick(event) {
    if (now() < suppressClickUntil) {
      suppressClickUntil = 0;
      return;
    }
    event?.stopPropagation?.();
    onCommit();
  }

  const listeners = [
    ["pointerdown", onPointerDown],
    ["pointermove", onPointerMove],
    ["pointerup", onPointerUp],
    ["pointercancel", onPointerCancel],
    ["lostpointercapture", onPointerCancel],
    ["click", onClick],
  ];
  listeners.forEach(([type, handler]) => target.addEventListener(type, handler));

  return Object.freeze({
    destroy() {
      endPress();
      suppressClickUntil = 0;
      listeners.forEach(([type, handler]) => target.removeEventListener?.(type, handler));
    },
    get pressed() { return press !== null; },
  });
}
