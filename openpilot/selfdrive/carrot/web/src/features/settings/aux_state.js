export function normalizeSettingFavoriteNames(names) {
  const normalized = [];
  const seen = new Set();
  (Array.isArray(names) ? names : []).forEach((item) => {
    const name = String(item || "").trim();
    if (!name || seen.has(name)) return;
    seen.add(name);
    normalized.push(name);
  });
  return normalized;
}

export function normalizeSettingProfiles(profiles) {
  return (Array.isArray(profiles) ? profiles : [])
    .filter((profile) => profile && profile.id && profile.name && profile.values)
    .map((profile) => ({
      ...profile,
      values: { ...(profile.values || {}) },
      meta: { ...(profile.meta || {}) },
    }));
}

export function normalizeSettingPopularValues(payload) {
  const values = payload?.popular_values;
  return {
    carKey: String(payload?.car_key || ""),
    fetchedAt: Number(payload?.fetched_at || 0),
    values: values && typeof values === "object" && !Array.isArray(values) ? { ...values } : {},
  };
}

export function createSettingsAuxState(loaders = {}) {
  const favoritesState = { loaded: false, promise: null, version: 0, names: [] };
  const profilesState = { loaded: false, promise: null, version: 0, profiles: [] };
  const popularState = { loaded: false, promise: null, version: 0, carKey: "", fetchedAt: 0, values: {} };

  function loadResource({ state, force, loader, apply, current, fallback }) {
    if (!force && state.loaded) return Promise.resolve(current());
    if (state.promise) return state.promise;
    const hadPreparedValue = state.loaded;
    const requestVersion = state.version;

    const request = Promise.resolve()
      .then(() => {
        if (typeof loader !== "function") throw new Error("Settings resource loader is unavailable");
        return loader();
      })
      .then((payload) => state.version === requestVersion ? apply(payload) : current())
      .catch(() => (
        state.version !== requestVersion || state.loaded || hadPreparedValue
          ? current()
          : fallback()
      ))
      .finally(() => {
        if (state.promise === request) state.promise = null;
      });
    state.promise = request;
    return request;
  }

  function replaceFavorites(names) {
    favoritesState.names = normalizeSettingFavoriteNames(names);
    favoritesState.loaded = true;
    favoritesState.version += 1;
    return favoritesState.names;
  }

  function loadFavorites(force = false) {
    return loadResource({
      state: favoritesState,
      force,
      loader: loaders.loadFavorites,
      apply: (payload) => replaceFavorites(payload?.favorites || []),
      current: () => favoritesState.names,
      fallback: () => replaceFavorites([]),
    });
  }

  function replaceProfiles(profiles) {
    profilesState.profiles = normalizeSettingProfiles(profiles);
    profilesState.loaded = true;
    profilesState.version += 1;
    return profilesState.profiles;
  }

  function loadProfiles(force = false) {
    return loadResource({
      state: profilesState,
      force,
      loader: loaders.loadProfiles,
      apply: (payload) => replaceProfiles(payload?.profiles || []),
      current: () => profilesState.profiles,
      fallback: () => replaceProfiles([]),
    });
  }

  function replacePopular(payload) {
    const normalized = normalizeSettingPopularValues(payload);
    popularState.carKey = normalized.carKey;
    popularState.fetchedAt = normalized.fetchedAt;
    popularState.values = normalized.values;
    popularState.loaded = true;
    popularState.version += 1;
    return popularState.values;
  }

  function loadPopular(force = false) {
    return loadResource({
      state: popularState,
      force,
      loader: loaders.loadPopular,
      apply: replacePopular,
      current: () => popularState.values,
      fallback: () => replacePopular({}),
    });
  }

  function hydrateSnapshot(snapshot) {
    replaceFavorites(snapshot?.favorites || []);
    replaceProfiles(snapshot?.profiles || []);
    replacePopular(snapshot?.popular || {});
  }

  const favorites = Object.freeze({
    load: loadFavorites,
    replace: replaceFavorites,
    normalize: normalizeSettingFavoriteNames,
    get: () => favoritesState.names,
    get loaded() { return favoritesState.loaded; },
  });
  const profiles = Object.freeze({
    load: loadProfiles,
    replace: replaceProfiles,
    normalize: normalizeSettingProfiles,
    get: () => profilesState.profiles,
    get loaded() { return profilesState.loaded; },
  });
  const popular = Object.freeze({
    load: loadPopular,
    replace: replacePopular,
    getValue(name) {
      const entry = popularState.values?.[String(name || "")];
      return entry && typeof entry === "object" ? entry : null;
    },
    getState() {
      return {
        carKey: popularState.carKey,
        fetchedAt: popularState.fetchedAt,
        values: popularState.values,
      };
    },
    get loaded() { return popularState.loaded; },
  });

  return Object.freeze({ favorites, profiles, popular, hydrateSnapshot });
}
