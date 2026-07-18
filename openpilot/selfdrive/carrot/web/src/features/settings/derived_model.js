export const SETTING_DERIVED_IDS = Object.freeze({
  favoritesGroup: "__setting_favorites__",
  profilesDivider: "__setting_profiles_divider__",
  profileGroupPrefix: "__setting_profile__:",
  categoryDividerPrefix: "__setting_category__:",
});

export function localizedSettingNodeLabel(node, language = "en") {
  if (!node) return "";
  if (language === "zh") return node.zh || node.en || node.ko || "";
  if (language === "ko") return node.ko || node.en || node.zh || "";
  return node.en || node.ko || node.zh || "";
}

export function localizedSettingItemText(item, keyKo, keyEn, fallback = "", language = "en") {
  if (!item) return fallback;
  if (language === "zh") return item[`c${keyEn.slice(1)}`] || item[keyEn] || item[keyKo] || fallback;
  if (language === "ko") return item[keyKo] || item[keyEn] || fallback;
  return item[keyEn] || item[keyKo] || fallback;
}

export function createSettingsDerivedModel(options = {}) {
  const catalog = options.catalog || null;
  const favorites = Array.isArray(options.favorites) ? options.favorites : [];
  const profiles = Array.isArray(options.profiles) ? options.profiles : [];
  const language = options.language || "en";
  const labels = {
    favorites: options.labels?.favorites || "Favorites",
    profiles: options.labels?.profiles || "Profiles",
  };
  const groups = catalog?.groups || [];
  const itemsByGroup = catalog?.items_by_group || {};
  const groupIndex = new Map();
  groups.forEach((entry, index) => {
    if (!groupIndex.has(entry?.group)) groupIndex.set(entry?.group, { entry, index });
  });
  const itemIndex = new Map();
  const itemOrder = new Map();
  Object.entries(itemsByGroup).forEach(([group, items]) => {
    const order = new Map();
    (items || []).forEach((item, index) => {
      if (!item?.name) return;
      if (!itemIndex.has(item.name)) itemIndex.set(item.name, { group, item });
      if (!order.has(item.name)) order.set(item.name, index);
    });
    itemOrder.set(group, order);
  });
  const profilesById = new Map();
  profiles.forEach((profile) => {
    const id = String(profile?.id || "");
    if (!profilesById.has(id)) profilesById.set(id, profile);
  });

  function profileGroup(profileId) {
    return SETTING_DERIVED_IDS.profileGroupPrefix + String(profileId || "");
  }

  function isProfileGroup(group) {
    return String(group || "").startsWith(SETTING_DERIVED_IDS.profileGroupPrefix);
  }

  function profileIdFromGroup(group) {
    return isProfileGroup(group)
      ? String(group).slice(SETTING_DERIVED_IDS.profileGroupPrefix.length)
      : "";
  }

  function getProfileById(profileId) {
    return profilesById.get(String(profileId || "")) || null;
  }

  function getProfileByGroup(group) {
    return getProfileById(profileIdFromGroup(group));
  }

  function findItemByName(name) {
    return itemIndex.get(String(name || "").trim()) || null;
  }

  function getFavoriteEntries() {
    return favorites.map(findItemByName).filter(Boolean);
  }

  function getProfileEntries(profile) {
    return Object.keys(profile?.values || {})
      .map(findItemByName)
      .filter(Boolean)
      .sort((left, right) => {
        const leftGroup = groupIndex.get(left.group)?.index ?? 9999;
        const rightGroup = groupIndex.get(right.group)?.index ?? 9999;
        if (leftGroup !== rightGroup) return leftGroup - rightGroup;

        const leftItem = itemOrder.get(left.group)?.get(left.item.name) ?? 9999;
        const rightItem = itemOrder.get(right.group)?.get(right.item.name) ?? 9999;
        if (leftItem !== rightItem) return leftItem - rightItem;
        return String(left.item.name).localeCompare(String(right.item.name));
      });
  }

  function getValidFavoriteNames() {
    return getFavoriteEntries().map((entry) => entry.item.name).filter(Boolean);
  }

  function getGroupsForDisplay() {
    const displayGroups = [{
      group: SETTING_DERIVED_IDS.favoritesGroup,
      count: getFavoriteEntries().length,
      virtual: true,
    }];
    const categories = catalog?.categories;
    if (Array.isArray(categories) && categories.length) {
      categories.forEach((category) => {
        displayGroups.push({
          group: SETTING_DERIVED_IDS.categoryDividerPrefix + (category.id || ""),
          label: localizedSettingNodeLabel(category, language),
          divider: true,
          virtual: true,
        });
        (category.groups || []).forEach((group) => {
          displayGroups.push({
            group: group.id,
            ko: group.ko,
            en: group.en,
            zh: group.zh,
            count: group.count,
            category: category.id,
          });
        });
      });
    } else {
      displayGroups.push(...groups);
    }

    if (profiles.length) {
      displayGroups.push({
        group: SETTING_DERIVED_IDS.profilesDivider,
        label: labels.profiles,
        divider: true,
        virtual: true,
      });
      profiles.forEach((profile) => {
        displayGroups.push({
          group: profileGroup(profile.id),
          count: getProfileEntries(profile).length,
          label: profile.name,
          profile,
          virtual: true,
        });
      });
    }
    return displayGroups;
  }

  function getItemEntriesForGroup(group) {
    if (group === SETTING_DERIVED_IDS.favoritesGroup) return getFavoriteEntries();
    const profile = getProfileByGroup(group);
    if (profile) return getProfileEntries(profile);
    return (itemsByGroup[group] || []).map((item) => ({ group, item }));
  }

  function getGroupMeta(group) {
    if (group === SETTING_DERIVED_IDS.favoritesGroup) {
      return { group, egroup: "Favorites", count: getFavoriteEntries().length, virtual: true };
    }
    const profile = getProfileByGroup(group);
    if (profile) {
      return {
        group,
        egroup: profile.name,
        count: getProfileEntries(profile).length,
        profile,
        virtual: true,
      };
    }
    return groupIndex.get(group)?.entry || null;
  }

  function getGroupLabel(group) {
    if (group === SETTING_DERIVED_IDS.favoritesGroup) return labels.favorites;
    const profile = getProfileByGroup(group);
    if (profile) return profile.name;
    const meta = getGroupMeta(group);
    if (!meta) return group;
    if (meta.ko || meta.en || meta.zh) return localizedSettingNodeLabel(meta, language);
    if (language === "zh") return meta.cgroup || meta.egroup || meta.group;
    if (language === "ko") return meta.group || meta.egroup || group;
    return meta.egroup || meta.group || group;
  }

  function getItemContextLabel(group, item) {
    const groupLabel = getGroupLabel(group);
    const sectionLabel = item?.__section
      ? localizedSettingNodeLabel(item.__section, language)
      : "";
    if (!sectionLabel || sectionLabel === groupLabel) return groupLabel;
    return `${groupLabel} > ${sectionLabel}`;
  }

  function makeSearchEntry({ source, profile = null, group, item, sourceLabels }) {
    const groupLabel = getGroupLabel(group);
    const contextGroupLabel = getItemContextLabel(group, item);
    const title = localizedSettingItemText(item, "title", "etitle", "", language);
    const descr = localizedSettingItemText(item, "descr", "edescr", "", language);
    const isProfile = source === "profile" && profile?.id;
    const profileName = isProfile ? String(profile.name || "") : "";
    const sourceLabel = isProfile ? sourceLabels.profile : sourceLabels.carrot;
    const contextLabel = isProfile ? `${profileName} / ${contextGroupLabel}` : contextGroupLabel;
    return {
      source: isProfile ? "profile" : "carrot",
      sourceLabel,
      profileId: isProfile ? profile.id : "",
      profileName,
      group: isProfile ? profileGroup(profile.id) : group,
      originalGroup: group,
      groupLabel,
      contextGroupLabel,
      contextLabel,
      name: item.name,
      title,
      descr,
      haystack: [sourceLabel, profileName, groupLabel, contextGroupLabel, item.name, title, descr]
        .join("\n")
        .toLowerCase(),
    };
  }

  function buildSearchEntries(sourceLabels = {}) {
    const resolvedLabels = {
      carrot: sourceLabels.carrot || "CarrotPilot",
      profile: sourceLabels.profile || "Profile",
    };
    const entries = [];
    groups.forEach((groupMeta) => {
      const group = groupMeta.group;
      (itemsByGroup[group] || []).forEach((item) => {
        entries.push(makeSearchEntry({ source: "carrot", group, item, sourceLabels: resolvedLabels }));
      });
    });
    profiles.forEach((profile) => {
      getProfileEntries(profile).forEach((entry) => {
        entries.push(makeSearchEntry({
          source: "profile",
          profile,
          group: entry.group,
          item: entry.item,
          sourceLabels: resolvedLabels,
        }));
      });
    });
    return entries;
  }

  return Object.freeze({
    profileGroup,
    isProfileGroup,
    profileIdFromGroup,
    getProfileById,
    getProfileByGroup,
    findItemByName,
    getFavoriteEntries,
    getProfileEntries,
    getValidFavoriteNames,
    getGroupsForDisplay,
    getItemEntriesForGroup,
    getGroupMeta,
    getGroupLabel,
    getItemContextLabel,
    buildSearchEntries,
  });
}

export function createSettingsDerivedModelMemo() {
  let cached = null;

  function get(options = {}) {
    const favoritesLabel = options.labels?.favorites || "Favorites";
    const profilesLabel = options.labels?.profiles || "Profiles";
    if (
      cached?.catalog === options.catalog &&
      cached.favorites === options.favorites &&
      cached.profiles === options.profiles &&
      cached.language === options.language &&
      cached.favoritesLabel === favoritesLabel &&
      cached.profilesLabel === profilesLabel
    ) {
      return cached.model;
    }

    const model = createSettingsDerivedModel(options);
    cached = {
      catalog: options.catalog,
      favorites: options.favorites,
      profiles: options.profiles,
      language: options.language,
      favoritesLabel,
      profilesLabel,
      model,
    };
    return model;
  }

  function clear() {
    cached = null;
  }

  return Object.freeze({ get, clear });
}
