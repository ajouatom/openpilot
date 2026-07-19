import { createMenuController } from "./menu.js";

function isObject(value) {
  return (typeof value === "object" && value !== null) || typeof value === "function";
}

export const menuFacade = Object.freeze({
  create: createMenuController,
  createController: createMenuController,
  mount: createMenuController,
});

export function installMenuFacade(target = globalThis, options = {}) {
  if (!isObject(target)) throw new TypeError("A facade target is required");
  const create = options.createMenuController || createMenuController;
  if (typeof create !== "function") throw new TypeError("A menu controller factory is required");
  const facade = create === createMenuController
    ? menuFacade
    : Object.freeze({ create, createController: create, mount: create });
  const currentNamespace = isObject(target.CarrotUI) ? target.CarrotUI : null;
  const namespace = currentNamespace && Object.isExtensible(currentNamespace)
    ? currentNamespace
    : Object.assign({}, currentNamespace || {});
  namespace.menu = facade;
  target.CarrotUI = namespace;
  if (options.compatibility !== false) target.CarrotDropdownMenu = facade;
  return facade;
}
