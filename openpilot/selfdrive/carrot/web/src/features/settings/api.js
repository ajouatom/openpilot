export const SETTINGS_SNAPSHOT_ENDPOINT = "/api/settings/snapshot";
export const SETTINGS_FAVORITES_ENDPOINT = "/api/setting_favorites";
export const SETTINGS_PROFILES_ENDPOINT = "/api/setting_profiles";
export const SETTINGS_POPULAR_VALUES_ENDPOINT = "/api/setting_popular_values";

function requestWithGlobalClient(global, endpoint) {
  if (typeof global?.getJson !== "function") {
    return Promise.reject(new Error("Settings API client is unavailable"));
  }
  return global.getJson(endpoint);
}

export function createSettingsSnapshotLoader(global) {
  return function loadSettingsSnapshot() {
    return requestWithGlobalClient(global, SETTINGS_SNAPSHOT_ENDPOINT);
  };
}

export function createSettingsAuxLoaders(global) {
  return Object.freeze({
    loadFavorites: () => requestWithGlobalClient(global, SETTINGS_FAVORITES_ENDPOINT),
    loadProfiles: () => requestWithGlobalClient(global, SETTINGS_PROFILES_ENDPOINT),
    loadPopular: () => requestWithGlobalClient(global, SETTINGS_POPULAR_VALUES_ENDPOINT),
  });
}
