export function createSettingsEntryController(options = {}) {
  const snapshotStore = options.snapshotStore || null;
  const getPreparedCatalog = options.getPreparedCatalog;
  const loadLegacyCatalog = options.loadLegacyCatalog;
  const loadLegacyAuxiliary = options.loadLegacyAuxiliary;
  const commitCatalog = options.commitCatalog;
  let preparePromise = null;

  function requireCatalog(catalog) {
    if (!catalog || typeof catalog !== "object") {
      throw new Error("Settings catalog preparation failed");
    }
    return catalog;
  }

  function primeSnapshot(snapshot) {
    if (!snapshot?.settings || typeof commitCatalog !== "function") return null;
    return requireCatalog(commitCatalog(snapshot.settings, snapshot));
  }

  function prepare(options = {}) {
    const force = options.force === true;
    if (preparePromise) return preparePromise;

    const prepared = typeof getPreparedCatalog === "function" ? getPreparedCatalog() : null;
    if (!force && prepared) {
      return Promise.resolve({ catalog: prepared, source: "memory" });
    }

    const task = (async () => {
      let snapshot = null;
      if (typeof snapshotStore?.load === "function") {
        try {
          snapshot = await snapshotStore.load({ force });
        } catch (_) {
          // A partially upgraded or temporarily unavailable snapshot endpoint
          // falls through to the legacy catalog and auxiliary resources.
        }
      }

      if (snapshot?.settings) {
        return { catalog: primeSnapshot(snapshot), source: "snapshot" };
      }
      if (typeof loadLegacyCatalog !== "function") {
        throw new Error("Legacy settings catalog loader is unavailable");
      }

      let auxiliaryPromise = Promise.resolve();
      if (typeof loadLegacyAuxiliary === "function") {
        try {
          auxiliaryPromise = Promise.resolve(loadLegacyAuxiliary(force)).catch(() => null);
        } catch (_) {
          auxiliaryPromise = Promise.resolve();
        }
      }
      const legacyCatalog = await loadLegacyCatalog();
      const catalog = requireCatalog(commitCatalog(legacyCatalog, null));
      await auxiliaryPromise;
      return { catalog, source: "legacy" };
    })();

    const tracked = task.finally(() => {
      if (preparePromise === tracked) preparePromise = null;
    });
    preparePromise = tracked;
    return tracked;
  }

  return Object.freeze({
    prepare,
    primeSnapshot,
    get pending() { return Boolean(preparePromise); },
  });
}
