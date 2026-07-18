export const CARROT_NAVI_REQUIRED_METHODS = Object.freeze([
  "refresh",
  "requestOwnershipTakeover",
  "setContentActive",
  "resize",
  "status",
]);

export function createCarrotNaviFacade(implementation) {
  if (!implementation || CARROT_NAVI_REQUIRED_METHODS.some((name) => typeof implementation[name] !== "function")) return null;

  const lifecycle = Object.freeze({
    refresh() {
      return implementation.refresh();
    },
    requestOwnershipTakeover(reason) {
      return implementation.requestOwnershipTakeover(reason);
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
}

export function installCarrotNaviFacadeGlobal(target = globalThis, implementation = target.CarrotNaviWeb) {
  const facade = createCarrotNaviFacade(implementation);
  target.DriveCarrotNaviFacade = facade;
  return facade;
}
