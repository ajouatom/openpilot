"use strict";

globalThis.DriveCarrotNaviFacade = (() => {
  const implementation = globalThis.CarrotNaviWeb;
  const requiredMethods = Object.freeze([
    "refresh",
    "setContentActive",
    "resize",
    "status",
  ]);

  if (!implementation || requiredMethods.some((name) => typeof implementation[name] !== "function")) return null;

  const lifecycle = Object.freeze({
    refresh() {
      return implementation.refresh();
    },
  });

  const content = Object.freeze({
    activate(context = {}) {
      return implementation.setContentActive(true, context);
    },
    deactivate(options = {}) {
      return implementation.setContentActive(false, options);
    },
    resize(rect) {
      return implementation.resize(rect);
    },
    status() {
      return implementation.status();
    },
  });

  function status() {
    return {
      ready: true,
      implementation: "CarrotNaviWeb",
      boundaries: ["lifecycle", "content"],
      runtime: implementation.status(),
    };
  }

  return Object.freeze({ lifecycle, content, status });
})();
