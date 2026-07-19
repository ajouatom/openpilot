"use strict";

// Web settings state + API. The backend spec (bootstrap.webSettingsSpec) is the
// single source of truth for each key's type/default; this module derives its
// defaults and normalization from it instead of re-declaring them. Server-backed
// values live on the device; last-page history stays browser-local by design.

const WEB_LAST_PAGE_KEY = "carrot_web_last_page";
const WEB_PRIMARY_PAGES = new Set(["carrot", "setting", "tools", "logs", "terminal"]);

// Spec from the backend bootstrap: [{ key, type: "bool"|"enum"|"str", default, choices? }].
// If it is somehow absent, state is still fully seeded from bootstrap.webSettings
// (the server always sends every key) and normalization degrades to identity.
const WEB_SETTINGS_SPEC = Array.isArray(window.__CARROT_BOOTSTRAP__?.webSettingsSpec)
  ? window.__CARROT_BOOTSTRAP__.webSettingsSpec
  : [];
const WEB_SPEC_BY_KEY = Object.create(null);
WEB_SETTINGS_SPEC.forEach((field) => { if (field && field.key) WEB_SPEC_BY_KEY[field.key] = field; });

const WEB_SETTING_DEFAULTS = Object.create(null);
WEB_SETTINGS_SPEC.forEach((field) => { if (field && field.key) WEB_SETTING_DEFAULTS[field.key] = field.default; });
window.CarrotWebSettingDefaults = WEB_SETTING_DEFAULTS;

function webSettingKnownKeys() {
  const keys = Object.keys(WEB_SETTING_DEFAULTS);
  return keys.length ? keys : Object.keys(webSettingsState);
}

function coerceWebBool(value) {
  if (typeof value === "string") return ["1", "true", "yes", "on"].includes(value.trim().toLowerCase());
  return Boolean(value);
}

function normalizeWebSettingValue(key, value) {
  const field = WEB_SPEC_BY_KEY[key];
  if (!field) return value;
  if (field.type === "bool") return coerceWebBool(value);
  if (field.type === "enum") {
    const candidate = String(value ?? "").trim().toLowerCase();
    return (field.choices || []).includes(candidate) ? candidate : field.default;
  }
  // "str"
  if (key === "web_language") {
    if (typeof normalizeLangCode === "function") return normalizeLangCode(value);
    const lang = String(value ?? "").trim().toLowerCase();
    return ["en", "ko", "zh"].includes(lang) ? lang : "";
  }
  if (key === "toss_upload_url") {
    let url = String(value ?? "").trim().replace(/\/+$/, "");
    if (url && !/^https?:\/\//i.test(url)) url = `https://${url}`;
    return url || field.default;
  }
  const str = String(value ?? "").trim();
  return str || field.default;
}

const webSettingsState = {};
window.CarrotWebSettingsState = webSettingsState;

let webSettingsLoaded = false;
let webSettingsLoadPromise = null;
const webSettingsGuards = new Set();

function registerWebSettingsGuard(guard) {
  if (typeof guard !== "function") throw new TypeError("web settings guard must be a function");
  webSettingsGuards.add(guard);
  return () => webSettingsGuards.delete(guard);
}

function validateWebSettingsChange(detail) {
  const frozen = Object.freeze({
    ...detail,
    keys: Object.freeze([...detail.keys]),
    updates: Object.freeze({ ...detail.updates }),
    previous: Object.freeze({ ...detail.previous }),
    settings: Object.freeze({ ...detail.settings }),
  });
  for (const guard of webSettingsGuards) {
    const result = guard(frozen);
    if (result && typeof result.then === "function") {
      throw new TypeError("web settings guards must complete synchronously");
    }
  }
}

function applyWebSettings(settings = {}) {
  webSettingKnownKeys().forEach((key) => {
    const value = Object.prototype.hasOwnProperty.call(settings, key)
      ? settings[key]
      : (Object.prototype.hasOwnProperty.call(webSettingsState, key) ? webSettingsState[key] : WEB_SETTING_DEFAULTS[key]);
    webSettingsState[key] = normalizeWebSettingValue(key, value);
  });
  return webSettingsState;
}

function changedWebSettingKeys(previous = {}) {
  return webSettingKnownKeys().filter((key) => (
    !Object.is(previous[key], webSettingsState[key])
  ));
}

function dispatchWebSettingsChange(keys, detail = {}) {
  const changedKeys = [...new Set((keys || []).filter((key) => typeof key === "string" && key))];
  if (!changedKeys.length) return false;
  window.dispatchEvent(new CustomEvent("carrot:websettingschange", {
    detail: {
      ...detail,
      key: changedKeys[0],
      keys: changedKeys,
      value: webSettingsState[changedKeys[0]],
      values: Object.fromEntries(changedKeys.map((key) => [key, webSettingsState[key]])),
      settings: { ...webSettingsState },
    },
  }));
  return true;
}

applyWebSettings(window.__CARROT_BOOTSTRAP__?.webSettings || {});

async function requestWebSettings(method = "GET", body = null) {
  const options = { method };
  if (body) {
    options.headers = { "Content-Type": "application/json" };
    options.body = JSON.stringify(body);
  }
  if (typeof requestJson === "function") {
    return requestJson("/api/web_settings", options);
  }
  const response = await fetch("/api/web_settings", options);
  const payload = await response.json();
  if (!response.ok || payload?.ok === false) {
    throw new Error(payload?.error || `HTTP ${response.status}`);
  }
  return payload;
}

async function loadWebSettings(force = false) {
  if (!force && webSettingsLoaded) return webSettingsState;
  if (webSettingsLoadPromise) return webSettingsLoadPromise;
  webSettingsLoadPromise = requestWebSettings("GET")
    .then((payload) => {
      const previous = { ...webSettingsState };
      webSettingsLoaded = true;
      const settings = applyWebSettings(payload?.settings || {});
      dispatchWebSettingsChange(changedWebSettingKeys(previous), {
        source: "load",
        pending: false,
      });
      return settings;
    })
    .finally(() => {
      webSettingsLoadPromise = null;
    });
  return webSettingsLoadPromise;
}

function getWebSettingByKey(key, fallback = undefined) {
  if (!Object.prototype.hasOwnProperty.call(webSettingsState, key) &&
      !Object.prototype.hasOwnProperty.call(WEB_SETTING_DEFAULTS, key)) {
    return fallback;
  }
  return webSettingsState[key] ?? fallback ?? WEB_SETTING_DEFAULTS[key];
}

async function setWebSettingsByKeys(updates = {}) {
  const normalizedUpdates = {};
  const previous = {};
  for (const [key, value] of Object.entries(updates || {})) {
    const known = Object.prototype.hasOwnProperty.call(webSettingsState, key) ||
      Object.prototype.hasOwnProperty.call(WEB_SETTING_DEFAULTS, key);
    if (!known) continue;
    previous[key] = webSettingsState[key];
    normalizedUpdates[key] = normalizeWebSettingValue(key, value);
  }
  const keys = Object.keys(normalizedUpdates);
  if (!keys.length) return {};
  const nextSettings = { ...webSettingsState, ...normalizedUpdates };
  const locallyChangedKeys = keys.filter((key) => !Object.is(previous[key], nextSettings[key]));
  if (locallyChangedKeys.length) {
    validateWebSettingsChange({
      source: "local",
      keys: locallyChangedKeys,
      updates: normalizedUpdates,
      previous,
      settings: nextSettings,
    });
  }
  Object.assign(webSettingsState, normalizedUpdates);
  dispatchWebSettingsChange(locallyChangedKeys, {
    source: "local",
    pending: true,
  });
  try {
    const payload = await requestWebSettings("POST", normalizedUpdates);
    const beforeServer = { ...webSettingsState };
    applyWebSettings(payload?.settings || normalizedUpdates);
    dispatchWebSettingsChange([...keys, ...changedWebSettingKeys(beforeServer)], {
      source: "server",
      pending: false,
    });
  } catch (err) {
    Object.assign(webSettingsState, previous);
    dispatchWebSettingsChange(locallyChangedKeys, {
      source: "rollback",
      pending: false,
    });
    throw err;
  }
  return Object.fromEntries(keys.map((key) => [key, webSettingsState[key]]));
}

async function setWebSettingByKey(key, value) {
  const settings = await setWebSettingsByKeys({ [key]: value });
  return settings[key];
}

function getWebStartPageSetting() {
  return normalizeWebSettingValue("start_page", webSettingsState.start_page);
}

function getWebStartPage() {
  const setting = getWebStartPageSetting();
  if (setting !== "last") return setting;
  try {
    const lastPage = localStorage.getItem(WEB_LAST_PAGE_KEY);
    return WEB_PRIMARY_PAGES.has(lastPage) ? lastPage : "carrot";
  } catch {
    return "carrot";
  }
}

function setWebStartPage(value) {
  return setWebSettingByKey("start_page", value);
}

function recordWebLastPage(page) {
  if (!WEB_PRIMARY_PAGES.has(page)) return;
  try {
    localStorage.setItem(WEB_LAST_PAGE_KEY, page);
  } catch {}
}

window.loadWebSettings = loadWebSettings;
window.getWebSettingByKey = getWebSettingByKey;
window.setWebSettingByKey = setWebSettingByKey;
window.setWebSettingsByKeys = setWebSettingsByKeys;
window.registerWebSettingsGuard = registerWebSettingsGuard;
window.getWebStartPage = getWebStartPage;
window.getWebStartPageSetting = getWebStartPageSetting;
window.setWebStartPage = setWebStartPage;
window.recordWebLastPage = recordWebLastPage;

export {
  WEB_SETTINGS_SPEC,
  WEB_SPEC_BY_KEY,
  WEB_SETTING_DEFAULTS,
  webSettingsState,
  webSettingKnownKeys,
  normalizeWebSettingValue,
  applyWebSettings,
  loadWebSettings,
  getWebSettingByKey,
  setWebSettingByKey,
  setWebSettingsByKeys,
  registerWebSettingsGuard,
  getWebStartPage,
  getWebStartPageSetting,
  setWebStartPage,
  recordWebLastPage,
};
