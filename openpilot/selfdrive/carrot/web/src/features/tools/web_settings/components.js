"use strict";

// Registry for compound preference controls. Generic bool/enum rows remain in
// The generic renderer handles bool/enum rows; multi-key controls register here independently.
globalThis.WebSettingsComponents = (() => {
  const components = new Map();

  function register(name, definition) {
    const key = String(name || "").trim();
    if (!key || !definition || typeof definition.render !== "function") {
      throw new TypeError("WebSettingsComponents.register requires a renderable component");
    }
    if (components.has(key)) throw new Error(`Web settings component already registered: ${key}`);
    components.set(key, Object.freeze({ ...definition }));
  }

  function definition(name) {
    return components.get(String(name || "")) || null;
  }

  function isVisible(name, fields) {
    const component = definition(name);
    if (!component) return false;
    return (component.settingKeys || []).every((key) => Boolean(fields[key]));
  }

  function render(name, item) {
    return definition(name)?.render(item) || "";
  }

  function bind(root = document) {
    for (const component of components.values()) component.bind?.(root);
  }

  return Object.freeze({ register, isVisible, render, bind });
})();
