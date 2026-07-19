export const DEFAULT_SETTING_VALUES_TTL_MS = 60_000;

export function createSettingValueCache(options = {}) {
  const now = options.now || (() => Date.now());
  const defaultTtlMs = Number.isFinite(options.defaultTtlMs)
    ? options.defaultTtlMs
    : DEFAULT_SETTING_VALUES_TTL_MS;
  const values = new Map();
  const groups = new Map();
  const pendingGroups = new Map();
  const groupVersions = new Map();
  let generation = 0;

  function isFresh(loadedAt, ttlMs) {
    return loadedAt > 0 && (now() - loadedAt) < ttlMs;
  }

  function clear() {
    generation += 1;
    values.clear();
    groups.clear();
    pendingGroups.clear();
    groupVersions.clear();
  }

  function invalidateGroup(group) {
    if (!group) return;
    groupVersions.set(group, (groupVersions.get(group) || 0) + 1);
    groups.delete(group);
    pendingGroups.delete(group);
  }

  function setValue(name, value, group = null) {
    if (!name) return;
    const loadedAt = now();
    values.set(name, { value, loadedAt });
    if (!group) return;

    if (pendingGroups.has(group)) {
      groupVersions.set(group, (groupVersions.get(group) || 0) + 1);
      pendingGroups.delete(group);
    }

    const cachedGroup = groups.get(group);
    if (!cachedGroup) return;
    cachedGroup.values[name] = value;
    cachedGroup.loadedAt = loadedAt;
  }

  function primeGroup(group, groupValues) {
    if (!group) return;
    const loadedAt = now();
    const snapshot = { values: { ...(groupValues || {}) }, loadedAt };
    groups.set(group, snapshot);
    Object.entries(snapshot.values).forEach(([name, value]) => {
      values.set(name, { value, loadedAt });
    });
  }

  function applyValues(nextValues) {
    if (!nextValues || typeof nextValues !== "object") return;
    const loadedAt = now();
    Object.entries(nextValues).forEach(([name, value]) => {
      values.set(name, { value, loadedAt });
    });

    for (const cachedGroup of groups.values()) {
      let changed = false;
      Object.entries(nextValues).forEach(([name, value]) => {
        if (!(name in cachedGroup.values)) return;
        cachedGroup.values[name] = value;
        changed = true;
      });
      if (changed) cachedGroup.loadedAt = loadedAt;
    }
  }

  function peekValues(names) {
    const requested = Array.isArray(names) ? names.filter(Boolean) : [];
    const snapshot = {};
    let complete = requested.length > 0;
    requested.forEach((name) => {
      const cachedValue = values.get(name);
      if (!cachedValue) {
        complete = false;
        return;
      }
      snapshot[name] = cachedValue.value;
    });
    return Object.freeze({ values: snapshot, complete });
  }

  function loadGroup(group, options = {}) {
    if (!group) return Promise.resolve({});
    const names = Array.isArray(options.names) ? options.names.filter(Boolean) : [];
    const force = options.force === true;
    const ttlMs = Number.isFinite(options.ttlMs) ? options.ttlMs : defaultTtlMs;
    const fetchMissing = options.fetchMissing;

    if (!names.length) {
      primeGroup(group, {});
      return Promise.resolve({});
    }

    if (force && pendingGroups.has(group)) {
      groupVersions.set(group, (groupVersions.get(group) || 0) + 1);
      pendingGroups.delete(group);
    }

    const cachedGroup = groups.get(group);
    if (!force && cachedGroup && isFresh(cachedGroup.loadedAt, ttlMs)) {
      return Promise.resolve({ ...cachedGroup.values });
    }
    if (!force && pendingGroups.has(group)) return pendingGroups.get(group);

    const assembled = {};
    const missing = [];
    names.forEach((name) => {
      const cachedValue = values.get(name);
      if (!force && cachedValue && isFresh(cachedValue.loadedAt, ttlMs)) {
        assembled[name] = cachedValue.value;
      } else {
        missing.push(name);
      }
    });

    if (!missing.length) {
      primeGroup(group, assembled);
      return Promise.resolve({ ...assembled });
    }
    if (typeof fetchMissing !== "function") {
      return Promise.reject(new TypeError("fetchMissing must be a function when values are missing"));
    }

    const requestGeneration = generation;
    const requestGroupVersion = groupVersions.get(group) || 0;
    const request = Promise.resolve()
      .then(() => fetchMissing(missing.slice()))
      .then((fetched) => {
        const nextValues = { ...assembled, ...(fetched || {}) };
        if (
          generation === requestGeneration &&
          (groupVersions.get(group) || 0) === requestGroupVersion
        ) {
          primeGroup(group, nextValues);
        }
        return { ...nextValues };
      })
      .finally(() => {
        if (pendingGroups.get(group) === request) pendingGroups.delete(group);
      });
    pendingGroups.set(group, request);
    return request;
  }

  return Object.freeze({
    clear,
    invalidateGroup,
    setValue,
    primeGroup,
    applyValues,
    peekValues,
    loadGroup,
    defaultTtlMs,
  });
}
