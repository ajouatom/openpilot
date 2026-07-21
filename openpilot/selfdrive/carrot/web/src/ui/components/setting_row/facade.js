import { SETTING_CONTROL_KINDS, createSettingControl } from "./control.js";

function isObject(value) {
  return (typeof value === "object" && value !== null) || typeof value === "function";
}

export function installSettingRowFacade(target = globalThis, environment = {}) {
  if (!isObject(target)) throw new TypeError("A facade target is required");

  const createControl = environment.createSettingControl || createSettingControl;
  if (typeof createControl !== "function") throw new TypeError("A setting control factory is required");

  const api = Object.freeze({ createControl, kinds: SETTING_CONTROL_KINDS });
  const currentNamespace = isObject(target.CarrotUI) ? target.CarrotUI : null;
  const namespace = currentNamespace && Object.isExtensible(currentNamespace)
    ? currentNamespace
    : Object.assign({}, currentNamespace || {});
  namespace.settingRow = api;
  target.CarrotUI = namespace;
  return api;
}
