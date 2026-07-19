"use strict";

((target) => {
  const dialog = target.CarrotUI?.dialog;
  if (!dialog) return;
  for (const name of [
    "openAppDialog",
    "appAlert",
    "appConfirm",
    "appPrompt",
    "appForm",
    "syncModalBodyLock",
  ]) {
    if (typeof dialog[name] === "function") target[name] = dialog[name];
  }
})(globalThis);
