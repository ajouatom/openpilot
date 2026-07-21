const LIVE_STATE_EVENT = "carrot:drivestatechange";
const LIVE_RUNTIME_RECEIPT_KEY = "liveRuntime";

class LiveStateProviderError extends Error {
  constructor(code, message) {
    super(message);
    this.name = "LiveStateProviderError";
    this.code = code;
  }
}

function hasEnumerableKeys(value) {
  if (!value || typeof value !== "object") return false;
  for (const _key in value) {
    return true;
  }
  return false;
}

function mergeServiceState(rawState, liveState) {
  const raw = rawState && typeof rawState === "object" ? rawState : {};
  const live = liveState && typeof liveState === "object" ? liveState : {};
  if (raw === live) return raw;
  if (!hasEnumerableKeys(live)) return raw;
  if (!hasEnumerableKeys(raw)) return live;
  return { ...live, ...raw };
}

function mergeDefinedState(baseState, preferredState) {
  const base = baseState && typeof baseState === "object" ? baseState : {};
  if (!preferredState || typeof preferredState !== "object") return base;
  let merged = null;
  for (const [key, value] of Object.entries(preferredState)) {
    if (value === undefined || value === null) continue;
    if (!merged) merged = { ...base };
    merged[key] = value;
  }
  return merged || base;
}

function mergeRadarState(rawState, liveState) {
  const raw = rawState && typeof rawState === "object" ? rawState : {};
  const live = liveState && typeof liveState === "object" ? liveState : {};
  if (raw === live) return raw;
  if (!hasEnumerableKeys(live)) return raw;
  if (!hasEnumerableKeys(raw)) return live;

  return {
    ...live,
    ...raw,
    leadOne: mergeDefinedState(live.leadOne, raw.leadOne),
    leadTwo: mergeDefinedState(live.leadTwo, raw.leadTwo),
    leadLeft: mergeDefinedState(live.leadLeft, raw.leadLeft),
    leadRight: mergeDefinedState(live.leadRight, raw.leadRight),
    leadsLeft: Array.isArray(raw.leadsLeft) && raw.leadsLeft.length ? raw.leadsLeft : live.leadsLeft,
    leadsCenter: Array.isArray(raw.leadsCenter) && raw.leadsCenter.length ? raw.leadsCenter : live.leadsCenter,
    leadsRight: Array.isArray(raw.leadsRight) && raw.leadsRight.length ? raw.leadsRight : live.leadsRight,
  };
}

function runtimeCacheMatches(cache, rawHudState, rawOverlayState, liveServices) {
  const refs = cache?.refs;
  return Boolean(
    refs
    && refs.rawHudState === rawHudState
    && refs.rawOverlayState === rawOverlayState
    && refs.liveServices === liveServices
    && refs.rawCarState === rawHudState?.carState
    && refs.rawControlsState === rawHudState?.controlsState
    && refs.rawDeviceState === rawHudState?.deviceState
    && refs.rawPeripheralState === rawHudState?.peripheralState
    && refs.rawSelfdriveState === rawHudState?.selfdriveState
    && refs.rawGpsLocationExternal === rawHudState?.gpsLocationExternal
    && refs.rawLongitudinalPlan === rawHudState?.longitudinalPlan
    && refs.rawCarrotMan === rawHudState?.carrotMan
    && refs.rawNavInstructionCarrot === rawHudState?.navInstructionCarrot
    && refs.rawNavRoute === rawHudState?.navRoute
    && refs.rawModelV2 === rawOverlayState?.modelV2
    && refs.rawLiveCalibration === rawOverlayState?.liveCalibration
    && refs.rawRoadCameraState === rawOverlayState?.roadCameraState
    && refs.rawRadarState === rawOverlayState?.radarState
    && refs.rawLateralPlan === rawOverlayState?.lateralPlan
    && refs.rawCarControl === rawOverlayState?.carControl
    && refs.rawLiveDelay === rawOverlayState?.liveDelay
    && refs.rawLiveTorqueParameters === rawOverlayState?.liveTorqueParameters
    && refs.rawLiveParameters === rawOverlayState?.liveParameters
    && refs.rawLiveTracks === rawOverlayState?.liveTracks
    && refs.rawCameraOdometry === rawOverlayState?.cameraOdometry
    && refs.rawLivePose === rawOverlayState?.livePose
    && refs.rawCarrotNavi === rawOverlayState?.carrotNavi
    && refs.liveCarState === liveServices?.carState
    && refs.liveControlsState === liveServices?.controlsState
    && refs.liveDeviceState === liveServices?.deviceState
    && refs.liveSelfdriveState === liveServices?.selfdriveState
    && refs.liveLongitudinalPlan === liveServices?.longitudinalPlan
    && refs.liveCarrotMan === liveServices?.carrotMan
    && refs.liveNavInstructionCarrot === liveServices?.navInstructionCarrot
    && refs.liveNavRoute === liveServices?.navRoute
    && refs.liveLateralPlan === liveServices?.lateralPlan
    && refs.liveRadarState === liveServices?.radarState
    && refs.liveLiveCalibration === liveServices?.liveCalibration
    && refs.liveRoadCameraState === liveServices?.roadCameraState
  );
}

function captureRuntimeRefs(rawHudState, rawOverlayState, liveServices) {
  return {
    rawHudState,
    rawOverlayState,
    liveServices,
    rawCarState: rawHudState?.carState,
    rawControlsState: rawHudState?.controlsState,
    rawDeviceState: rawHudState?.deviceState,
    rawPeripheralState: rawHudState?.peripheralState,
    rawSelfdriveState: rawHudState?.selfdriveState,
    rawGpsLocationExternal: rawHudState?.gpsLocationExternal,
    rawLongitudinalPlan: rawHudState?.longitudinalPlan,
    rawCarrotMan: rawHudState?.carrotMan,
    rawNavInstructionCarrot: rawHudState?.navInstructionCarrot,
    rawNavRoute: rawHudState?.navRoute,
    rawModelV2: rawOverlayState?.modelV2,
    rawLiveCalibration: rawOverlayState?.liveCalibration,
    rawRoadCameraState: rawOverlayState?.roadCameraState,
    rawRadarState: rawOverlayState?.radarState,
    rawLateralPlan: rawOverlayState?.lateralPlan,
    rawCarControl: rawOverlayState?.carControl,
    rawLiveDelay: rawOverlayState?.liveDelay,
    rawLiveTorqueParameters: rawOverlayState?.liveTorqueParameters,
    rawLiveParameters: rawOverlayState?.liveParameters,
    rawLiveTracks: rawOverlayState?.liveTracks,
    rawCameraOdometry: rawOverlayState?.cameraOdometry,
    rawLivePose: rawOverlayState?.livePose,
    rawCarrotNavi: rawOverlayState?.carrotNavi,
    liveCarState: liveServices?.carState,
    liveControlsState: liveServices?.controlsState,
    liveDeviceState: liveServices?.deviceState,
    liveSelfdriveState: liveServices?.selfdriveState,
    liveLongitudinalPlan: liveServices?.longitudinalPlan,
    liveCarrotMan: liveServices?.carrotMan,
    liveNavInstructionCarrot: liveServices?.navInstructionCarrot,
    liveNavRoute: liveServices?.navRoute,
    liveLateralPlan: liveServices?.lateralPlan,
    liveRadarState: liveServices?.radarState,
    liveLiveCalibration: liveServices?.liveCalibration,
    liveRoadCameraState: liveServices?.roadCameraState,
  };
}

function mergeRuntimeState(rawHudState, rawOverlayState, liveServices, cache = null) {
  if (runtimeCacheMatches(cache, rawHudState, rawOverlayState, liveServices)) {
    return cache.result;
  }

  const radarState = mergeRadarState(rawOverlayState?.radarState, liveServices?.radarState);
  const liveCalibration = mergeDefinedState(liveServices?.liveCalibration, rawOverlayState?.liveCalibration);
  const roadCameraState = mergeDefinedState(liveServices?.roadCameraState, rawOverlayState?.roadCameraState);
  const mergedHudState = {
    ...rawHudState,
    carState: mergeServiceState(rawHudState?.carState, liveServices?.carState),
    controlsState: mergeServiceState(rawHudState?.controlsState, liveServices?.controlsState),
    deviceState: mergeServiceState(rawHudState?.deviceState, liveServices?.deviceState),
    selfdriveState: mergeServiceState(rawHudState?.selfdriveState, liveServices?.selfdriveState),
    longitudinalPlan: mergeServiceState(rawHudState?.longitudinalPlan, liveServices?.longitudinalPlan),
    carrotMan: mergeServiceState(rawHudState?.carrotMan, liveServices?.carrotMan),
    navInstructionCarrot: mergeServiceState(
      rawHudState?.navInstructionCarrot,
      liveServices?.navInstructionCarrot,
    ),
    navRoute: mergeServiceState(rawHudState?.navRoute, liveServices?.navRoute),
    lateralPlan: mergeServiceState(rawOverlayState?.lateralPlan, liveServices?.lateralPlan),
    radarState,
  };

  const mergedOverlayState = {
    ...rawOverlayState,
    liveCalibration,
    roadCameraState,
    radarState: mergedHudState.radarState,
    lateralPlan: mergedHudState.lateralPlan,
    carrotMan: mergedHudState.carrotMan,
  };

  const result = {
    brokerServices: liveServices,
    hudState: mergedHudState,
    overlayState: mergedOverlayState,
  };
  if (cache) {
    cache.refs = captureRuntimeRefs(rawHudState, rawOverlayState, liveServices);
    cache.result = result;
  }
  return result;
}

function defaultNowMs(target) {
  const performanceObject = target?.performance || globalThis.performance;
  if (!performanceObject || typeof performanceObject.now !== "function") {
    throw new LiveStateProviderError("CLOCK_UNAVAILABLE", "performance.now is required for live state");
  }
  return performanceObject.now.bind(performanceObject);
}

function explicitRouteId(liveRuntimeState) {
  const candidates = [
    liveRuntimeState?.routeId,
    liveRuntimeState?.meta?.routeId,
    liveRuntimeState?.runtime?.routeId,
    liveRuntimeState?.services?.navRoute?.routeId,
  ];
  for (const candidate of candidates) {
    const routeId = String(candidate ?? "").trim();
    if (routeId) return routeId;
  }
  return null;
}

function createLiveStateProvider(options = {}) {
  const target = options.target || globalThis;
  const clock = typeof options.nowMs === "function" ? options.nowMs : defaultNowMs(target);
  const getHudState = options.getHudState || (() => target.CarrotHudState || {});
  const getOverlayState = options.getOverlayState || (() => target.CarrotOverlayState || {});
  const getLiveRuntimeState = options.getLiveRuntimeState || (() => target.CarrotLiveRuntimeState || {});
  const isReplayActive = options.isReplayActive || (() => Boolean(target.CarrotVisionReplay?.isActive?.()));
  const getActivity = options.getActivity || (() => target.CarrotDriveDataActivity || null);
  const getCompactActive = options.getCompactActive || (() => Boolean(target.CarrotVisionCompactStateActive));
  const listeners = new Set();
  const receivedAt = Object.create(null);
  const mergeCache = { refs: null, result: null };
  let destroyed = false;
  let everReceived = false;
  let lastPublishTimestampMs = Number.NEGATIVE_INFINITY;

  function readClock() {
    const timestampMs = Number(clock());
    if (!Number.isFinite(timestampMs) || timestampMs < 0) {
      throw new LiveStateProviderError("INVALID_CLOCK", "live state clock must return a finite non-negative number");
    }
    if (timestampMs < lastPublishTimestampMs) {
      throw new LiveStateProviderError(
        "NON_MONOTONIC_CLOCK",
        `live state clock moved backward (${timestampMs} < ${lastPublishTimestampMs})`,
      );
    }
    return timestampMs;
  }

  function nowMs() {
    return Number(clock());
  }

  function connectionState(timestampMs, liveRuntimeState) {
    const receiptValues = Object.values(receivedAt).filter((value) => Number.isFinite(value));
    const receivedNow = receiptValues.some((value) => value === timestampMs);
    if (getCompactActive() || liveRuntimeState?.ok === true || receivedNow) return "connected";
    if (getActivity()?.isActive?.()) return "connecting";
    if (everReceived) return "disconnected";
    return "idle";
  }

  function buildSnapshot(timestampMs) {
    const rawHudState = getHudState() || {};
    const rawOverlayState = getOverlayState() || {};
    const liveRuntimeState = getLiveRuntimeState() || {};
    const liveServices = isReplayActive()
      ? {}
      : (liveRuntimeState.services && typeof liveRuntimeState.services === "object"
        ? liveRuntimeState.services
        : {});
    const merged = mergeRuntimeState(rawHudState, rawOverlayState, liveServices, mergeCache);
    const snapshot = {
      timestampMs,
      hudState: merged.hudState,
      overlayState: merged.overlayState,
      receivedAtMonotonic: Object.freeze({ ...receivedAt }),
      connectionState: connectionState(timestampMs, liveRuntimeState),
      routeId: explicitRouteId(liveRuntimeState),
    };
    // HomeDrive historically reads the unmerged live service object before
    // its merged fallback. Keep that private compatibility path without
    // widening the public six-field snapshot contract.
    Object.defineProperty(snapshot, "brokerServices", {
      value: merged.brokerServices,
      enumerable: false,
      configurable: false,
      writable: false,
    });
    return Object.freeze(snapshot);
  }

  let currentSnapshot = buildSnapshot(readClock());
  lastPublishTimestampMs = currentSnapshot.timestampMs;

  function assertAlive(operation) {
    if (destroyed) {
      throw new LiveStateProviderError("PROVIDER_DESTROYED", `live state provider cannot ${operation} after destroy`);
    }
  }

  function publish(timestampMs) {
    const nextSnapshot = buildSnapshot(timestampMs);
    lastPublishTimestampMs = timestampMs;
    currentSnapshot = nextSnapshot;
    for (const listener of Array.from(listeners)) {
      try {
        listener(currentSnapshot);
      } catch (error) {
        target?.console?.error?.("[drive live state] listener failed", error);
      }
    }
    if (typeof target?.dispatchEvent === "function" && typeof target?.CustomEvent === "function") {
      target.dispatchEvent(new target.CustomEvent(LIVE_STATE_EVENT, { detail: currentSnapshot }));
    }
    return currentSnapshot;
  }

  function noteServiceReceived(service, noteOptions = {}) {
    return noteServicesReceived([service], noteOptions);
  }

  function noteServicesReceived(services, noteOptions = {}) {
    assertAlive("note service receipt");
    const serviceNames = Array.from(new Set(
      (Array.isArray(services) ? services : [services])
        .map((service) => String(service || "").trim())
        .filter(Boolean),
    ));
    if (!serviceNames.length) {
      throw new LiveStateProviderError("INVALID_SERVICE", "live state service name is required");
    }
    const timestampMs = readClock();
    if (noteOptions?.clear === true) {
      if (serviceNames.includes("*")) {
        for (const key of Object.keys(receivedAt)) {
          if (key !== LIVE_RUNTIME_RECEIPT_KEY) delete receivedAt[key];
        }
      } else {
        for (const serviceName of serviceNames) delete receivedAt[serviceName];
      }
      mergeCache.refs = null;
      mergeCache.result = null;
    } else {
      for (const serviceName of serviceNames) receivedAt[serviceName] = timestampMs;
      everReceived = true;
    }
    return publish(timestampMs);
  }

  function noteLiveRuntimeReceived(noteOptions = {}) {
    assertAlive("note live runtime receipt");
    const timestampMs = readClock();
    const liveRuntimeState = getLiveRuntimeState() || {};
    if (noteOptions?.received !== false && liveRuntimeState.ok === true) {
      receivedAt[LIVE_RUNTIME_RECEIPT_KEY] = timestampMs;
      everReceived = true;
    }
    return publish(timestampMs);
  }

  function snapshot() {
    return currentSnapshot;
  }

  function subscribe(listener) {
    assertAlive("subscribe");
    if (typeof listener !== "function") {
      throw new LiveStateProviderError("INVALID_LISTENER", "live state listener must be a function");
    }
    listeners.add(listener);
    let subscribed = true;
    return () => {
      if (!subscribed) return false;
      subscribed = false;
      return listeners.delete(listener);
    };
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    listeners.clear();
    activityUnsubscribe?.();
    activityUnsubscribe = null;
    mergeCache.refs = null;
    mergeCache.result = null;
    return true;
  }

  let activityUnsubscribe = getActivity()?.subscribe?.(() => {
    if (destroyed) return;
    publish(readClock());
  }) || null;

  return Object.freeze({
    nowMs,
    noteServiceReceived,
    noteServicesReceived,
    noteLiveRuntimeReceived,
    snapshot,
    subscribe,
    destroy,
  });
}

function installDriveLiveStateProviderFacade(target = globalThis, options = {}) {
  const existing = target?.CarrotDriveLiveStateProvider;
  if (existing && typeof existing.snapshot === "function" && typeof existing.subscribe === "function") {
    return existing;
  }
  const provider = createLiveStateProvider({ ...options, target });
  target.CarrotDriveLiveStateProvider = provider;
  return provider;
}

export {
  LIVE_RUNTIME_RECEIPT_KEY,
  LIVE_STATE_EVENT,
  LiveStateProviderError,
  createLiveStateProvider,
  installDriveLiveStateProviderFacade,
  mergeDefinedState,
  mergeRadarState,
  mergeRuntimeState,
  mergeServiceState,
};
