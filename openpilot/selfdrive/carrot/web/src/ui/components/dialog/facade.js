import { createDialogController } from "./dialog.js";
import { createFocusTrap } from "./focus_trap.js";

export const DIALOG_LEGACY_GLOBALS = Object.freeze([
  "openAppDialog",
  "openAppProgressDialog",
  "appAlert",
  "appConfirm",
  "appPrompt",
  "appForm",
  "syncModalBodyLock",
  "createFocusTrap",
]);

export function installDialogFacade(target = globalThis, documentRoot = target.document, environment = {}) {
  const createInjectedFocusTrap = (container, options = {}) => createFocusTrap(container, options, {
    document: documentRoot,
    window: target,
    requestAnimationFrame: environment.requestAnimationFrame,
    cancelAnimationFrame: environment.cancelAnimationFrame,
    getComputedStyle: environment.getComputedStyle,
  });
  const controller = createDialogController({
    ...environment,
    target,
    document: documentRoot,
    createFocusTrap: createInjectedFocusTrap,
  });
  const facade = Object.freeze({
    openAppDialog: controller.openAppDialog,
    openAppProgressDialog: controller.openAppProgressDialog,
    appAlert: controller.appAlert,
    appConfirm: controller.appConfirm,
    appPrompt: controller.appPrompt,
    appForm: controller.appForm,
    syncModalBodyLock: controller.syncModalBodyLock,
    createFocusTrap: createInjectedFocusTrap,
  });

  const currentNamespace = target.CarrotUI && typeof target.CarrotUI === "object"
    ? target.CarrotUI
    : null;
  const carrotUI = currentNamespace && Object.isExtensible(currentNamespace)
    ? currentNamespace
    : { ...(currentNamespace || {}) };
  carrotUI.dialog = facade;
  target.CarrotUI = carrotUI;
  for (const name of DIALOG_LEGACY_GLOBALS) target[name] = facade[name];
  return facade;
}
