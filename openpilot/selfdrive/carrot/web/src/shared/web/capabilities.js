"use strict";

function capabilitySpec(target = globalThis) {
  if (Array.isArray(target?.CarrotWebCapabilitiesSpec)) return target.CarrotWebCapabilitiesSpec;
  const bootstrap = target?.__CARROT_BOOTSTRAP__?.webCapabilitiesSpec;
  return Array.isArray(bootstrap) ? bootstrap : [];
}

function settingSpec(target = globalThis) {
  if (Array.isArray(target?.CarrotWebSettingsSpec)) return target.CarrotWebSettingsSpec;
  const bootstrap = target?.__CARROT_BOOTSTRAP__?.webSettingsSpec;
  return Array.isArray(bootstrap) ? bootstrap : [];
}

export function findWebCapabilitySpec(capabilityId, target = globalThis) {
  const id = String(capabilityId || "");
  return capabilitySpec(target).find((item) => item?.id === id) || null;
}

export function requiredWebCapability(settingKey, target = globalThis) {
  const key = String(settingKey || "");
  const field = settingSpec(target).find((item) => item?.key === key);
  return typeof field?.requiresCapability === "string" && field.requiresCapability
    ? field.requiresCapability
    : null;
}

export function readWebCapabilityEnabled(capabilityId, target = globalThis) {
  const id = String(capabilityId || "");
  const state = target?.CarrotWebCapabilitiesState;
  if (state && Object.prototype.hasOwnProperty.call(state, id)) {
    return Boolean(state[id]);
  }
  const bootstrap = target?.__CARROT_BOOTSTRAP__?.webCapabilities;
  if (bootstrap && Object.prototype.hasOwnProperty.call(bootstrap, id)) {
    return Boolean(bootstrap[id]);
  }
  const descriptor = findWebCapabilitySpec(id, target);
  const settingKey = descriptor?.settingKey;
  const settings = target?.CarrotWebSettingsState || target?.__CARROT_BOOTSTRAP__?.webSettings;
  return Boolean(settingKey && settings?.[settingKey]);
}

export function readWebSettingUnlocked(settingKey, target = globalThis) {
  const capabilityId = requiredWebCapability(settingKey, target);
  return !capabilityId || readWebCapabilityEnabled(capabilityId, target);
}

export function getWebSettingCapabilitySpec(settingKey, target = globalThis) {
  const capabilityId = requiredWebCapability(settingKey, target);
  return capabilityId ? findWebCapabilitySpec(capabilityId, target) : null;
}
