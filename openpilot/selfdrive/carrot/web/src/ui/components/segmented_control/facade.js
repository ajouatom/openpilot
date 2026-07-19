import { createSegmentedControl } from "./segmented_control.js";

function isObject(value) {
  return (typeof value === "object" && value !== null) || typeof value === "function";
}

export function installSegmentedControlFacade(target = globalThis, environment = {}) {
  if (!isObject(target)) throw new TypeError("A facade target is required");

  const create = environment.createSegmentedControl || createSegmentedControl;
  if (typeof create !== "function") throw new TypeError("A segmented control factory is required");

  const api = Object.freeze({ create });
  const currentNamespace = isObject(target.CarrotUI) ? target.CarrotUI : null;
  const namespace = currentNamespace && Object.isExtensible(currentNamespace)
    ? currentNamespace
    : Object.assign({}, currentNamespace || {});
  namespace.segmentedControl = api;
  target.CarrotUI = namespace;
  target.CarrotSegmentedControl = api;
  return api;
}
