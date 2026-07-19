import { createToastController } from "./toast.js";

const installedControllers = new WeakMap();

function isObject(value) {
  return (typeof value === "object" && value !== null) || typeof value === "function";
}

function attachNamespace(target, key, value) {
  const currentNamespace = isObject(target.CarrotUI) ? target.CarrotUI : null;
  const namespace = currentNamespace && Object.isExtensible(currentNamespace)
    ? currentNamespace
    : Object.assign({}, currentNamespace || {});
  namespace[key] = value;
  target.CarrotUI = namespace;
}

export function installToastFacade(target = globalThis, options = {}) {
  if (!isObject(target)) throw new TypeError("A facade target is required");
  const previousController = installedControllers.get(target);

  const documentRoot = options.documentRoot || target.document;
  const host = options.host || documentRoot?.getElementById?.("appToastHost") || null;
  const environment = {
    document: documentRoot,
    setTimeout: target.setTimeout?.bind?.(target),
    clearTimeout: target.clearTimeout?.bind?.(target),
    requestAnimationFrame: target.requestAnimationFrame?.bind?.(target),
    cancelAnimationFrame: target.cancelAnimationFrame?.bind?.(target),
    ...(options.environment || {}),
  };
  const controller = options.controller || createToastController({
    host,
    documentRoot,
    defaultDuration: options.defaultDuration,
    removeDelay: options.removeDelay,
    environment,
  });
  if (!controller || ["show", "dismiss", "destroy", "snapshot"].some((name) => typeof controller[name] !== "function")) {
    throw new TypeError("A toast controller is required");
  }
  const facade = Object.freeze({
    create: createToastController,
    createController: createToastController,
    show: controller.show,
    dismiss: controller.dismiss,
    destroy: controller.destroy,
    snapshot: controller.snapshot,
  });
  function showAppToast(message, toneOrOptions, duration) {
    if (toneOrOptions && typeof toneOrOptions === "object") {
      return controller.show(message, toneOrOptions);
    }
    const showOptions = {};
    if (toneOrOptions != null && toneOrOptions !== "") showOptions.tone = toneOrOptions;
    if (duration != null) showOptions.duration = duration;
    return controller.show(message, showOptions);
  }
  attachNamespace(target, "toast", facade);
  target.showAppToast = showAppToast;
  if (previousController && previousController !== controller) previousController.destroy();
  installedControllers.set(target, controller);
  return facade;
}
