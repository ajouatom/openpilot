function recordPresent(value) {
  return Boolean(value?.meta?.present);
}

function recordIdentity(value, fields) {
  if (!value || typeof value !== "object") return "";
  const meta = value.meta || {};
  const sequence = Math.max(0, Number(meta.sequence) || 0);
  const sourceTimestampMillis = Math.max(0, Number(meta.sourceTimestampMillis) || 0);
  return [sequence, sourceTimestampMillis, ...fields.map((field) => String(value[field] ?? ""))].join(":");
}

export function createFooterState() {
  let route = null;
  let routeIdentity = "";
  let vehicle = null;
  let vehicleIdentity = "";

  function navigationEnded(state) {
    const status = state?.navigationStatus;
    return recordPresent(status) && !status.guidanceActive && !status.routePresent;
  }

  function updateRoute(state) {
    const value = state?.route;
    const identity = recordIdentity(value, ["remainingDistanceM", "remainingTimeSec"]);
    if (navigationEnded(state)) {
      route = null;
      routeIdentity = "";
    } else if (recordPresent(value)) {
      if (!route || identity !== routeIdentity) {
        route = {
          meta: { ...(value.meta || {}) },
          remainingDistanceM: Math.max(0, Number(value.remainingDistanceM) || 0),
          remainingTimeSec: Math.max(0, Number(value.remainingTimeSec) || 0),
        };
        routeIdentity = identity;
      }
    }
  }

  function updateVehicle(state) {
    const value = state?.vehicle;
    const roadName = String(value?.roadName || "").trim();
    const identity = recordIdentity(value, ["roadName"]);
    if (navigationEnded(state)) {
      vehicle = null;
      vehicleIdentity = "";
    } else if (recordPresent(value) && roadName) {
      if (!vehicle || identity !== vehicleIdentity) {
        vehicle = {
          meta: { ...(value.meta || {}) },
          roadName,
        };
        vehicleIdentity = identity;
      }
    }
  }

  function snapshot() {
    return { route, vehicle };
  }

  function update(state) {
    updateRoute(state);
    updateVehicle(state);
    return snapshot();
  }

  function reset() {
    route = null;
    routeIdentity = "";
    vehicle = null;
    vehicleIdentity = "";
  }

  return Object.freeze({ update, reset, snapshot });
}

export const CarrotNaviFooterState = Object.freeze({ create: createFooterState });

export function installCarrotNaviFooterStateGlobal(target = globalThis) {
  target.CarrotNaviFooterState = CarrotNaviFooterState;
  return CarrotNaviFooterState;
}
