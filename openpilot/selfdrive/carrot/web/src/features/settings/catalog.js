export function normalizeSettingsCatalog(catalog) {
  if (!catalog || !Array.isArray(catalog.categories) || !catalog.categories.length) return catalog;

  const itemIndex = new Map();
  Object.values(catalog.items_by_group || {}).forEach((items) => {
    (items || []).forEach((item) => {
      if (item?.name) itemIndex.set(item.name, item);
    });
  });

  const groups = [];
  const itemsByGroup = {};
  catalog.categories.forEach((category) => {
    (category.groups || []).forEach((group) => {
      groups.push({
        group: group.id,
        ko: group.ko,
        en: group.en,
        zh: group.zh,
        count: group.count,
        category: category.id,
      });

      const items = [];
      (group.sections || []).forEach((section) => {
        const sectionLabel = {
          id: section.id,
          ko: section.ko,
          en: section.en,
          zh: section.zh,
        };
        (section.items || []).forEach((name) => {
          const item = itemIndex.get(name);
          if (item) items.push({ ...item, __section: sectionLabel });
        });
      });
      itemsByGroup[group.id] = items;
    });
  });

  catalog.groups = groups;
  catalog.items_by_group = itemsByGroup;
  return catalog;
}

export function createSettingsCatalogState() {
  let currentCatalog = null;
  let currentSnapshot = null;

  function commit(catalog, snapshot = null) {
    if (!catalog || typeof catalog !== "object") throw new TypeError("Settings catalog must be an object");
    if (currentCatalog === catalog && currentSnapshot === snapshot) {
      return { catalog: currentCatalog, changed: false };
    }

    currentCatalog = normalizeSettingsCatalog(catalog);
    currentSnapshot = snapshot;
    return { catalog: currentCatalog, changed: true };
  }

  function peek() {
    return currentCatalog;
  }

  return Object.freeze({ commit, peek });
}
