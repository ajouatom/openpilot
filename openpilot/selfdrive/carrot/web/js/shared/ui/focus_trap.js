"use strict";

((target) => {
  const createFocusTrap = target.CarrotUI?.dialog?.createFocusTrap;
  if (typeof createFocusTrap === "function") target.createFocusTrap = createFocusTrap;
})(globalThis);
