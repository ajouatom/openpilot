import { createPopoverController } from "./popover.js";

function isObject(value) {
  return (typeof value === "object" && value !== null) || typeof value === "function";
}

export const popoverFacade = Object.freeze({
  create: createPopoverController,
  createController: createPopoverController,
  mount: createPopoverController,
});

export function installPopoverFacade(target = globalThis, options = {}) {
  if (!isObject(target)) throw new TypeError("A facade target is required");
  const create = options.createPopoverController || createPopoverController;
  if (typeof create !== "function") throw new TypeError("A popover controller factory is required");
  const facade = create === createPopoverController
    ? popoverFacade
    : Object.freeze({ create, createController: create, mount: create });
  const currentNamespace = isObject(target.CarrotUI) ? target.CarrotUI : null;
  const namespace = currentNamespace && Object.isExtensible(currentNamespace)
    ? currentNamespace
    : Object.assign({}, currentNamespace || {});
  namespace.popover = facade;
  target.CarrotUI = namespace;
  return facade;
}
