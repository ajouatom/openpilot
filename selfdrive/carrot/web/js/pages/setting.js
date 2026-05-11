"use strict";

// Setting page — groups, items, value cache, search, subnav, screen layout.

let settingsLoadPromise = null;
let settingValueWarmupTimer = null;
let settingValueWarmupPromise = null;
let settingRestoreRefreshTimer = null;
const SETTING_VALUES_TTL_MS = 60000;
const settingValueCache = new Map();
const settingGroupValueCache = new Map();
const settingGroupValuePromises = new Map();

let settingSubnavSettleTimer = null;
let settingSubnavProgrammaticScroll = false;
let settingSubnavFocusTimer = null;

const SETTING_FAVORITES_GROUP = "__setting_favorites__";
const SETTING_PROFILES_DIVIDER = "__setting_profiles_divider__";
const SETTING_PROFILE_GROUP_PREFIX = "__setting_profile__:";
const SETTING_FAVORITES_LONG_PRESS_MS = 620;
const SETTING_FAVORITES_MOVE_TOLERANCE = 10;
const settingFavoritesState = {
  names: [],
  loaded: false,
  loadPromise: null,
};
const settingProfilesState = {
  profiles: [],
  loaded: false,
  loadPromise: null,
};
const settingProfileSectionExpandedState = new Map();

function isSettingFavoritesGroup(group) {
  return group === SETTING_FAVORITES_GROUP;
}

function isSettingProfilesDivider(entry) {
  return entry?.group === SETTING_PROFILES_DIVIDER || entry === SETTING_PROFILES_DIVIDER;
}

function settingProfileGroup(profileId) {
  return SETTING_PROFILE_GROUP_PREFIX + String(profileId || "");
}

function isSettingProfileGroup(group) {
  return String(group || "").startsWith(SETTING_PROFILE_GROUP_PREFIX);
}

function getSettingProfileIdFromGroup(group) {
  return isSettingProfileGroup(group) ? String(group).slice(SETTING_PROFILE_GROUP_PREFIX.length) : "";
}

function getSettingProfileById(profileId) {
  const id = String(profileId || "");
  return settingProfilesState.profiles.find((profile) => profile?.id === id) || null;
}

function getSettingProfileByGroup(group) {
  return getSettingProfileById(getSettingProfileIdFromGroup(group));
}

function normalizeSettingFavoriteNames(names) {
  const out = [];
  const seen = new Set();
  (Array.isArray(names) ? names : []).forEach((item) => {
    const name = String(item || "").trim();
    if (!name || seen.has(name)) return;
    seen.add(name);
    out.push(name);
  });
  return out;
}

function findSettingItemByName(name) {
  const target = String(name || "").trim();
  if (!target || !SETTINGS?.items_by_group) return null;

  for (const [group, list] of Object.entries(SETTINGS.items_by_group)) {
    const item = (list || []).find((entry) => entry?.name === target);
    if (item) return { group, item };
  }
  return null;
}

function getFavoriteSettingEntries() {
  return settingFavoritesState.names
    .map((name) => findSettingItemByName(name))
    .filter(Boolean);
}

function getSettingGroupOrderIndex(group) {
  const groups = SETTINGS?.groups || [];
  const index = groups.findIndex((entry) => entry?.group === group);
  return index >= 0 ? index : 9999;
}

function getSettingItemOrderIndex(group, name) {
  const list = SETTINGS?.items_by_group?.[group] || [];
  const index = list.findIndex((entry) => entry?.name === name);
  return index >= 0 ? index : 9999;
}

function getProfileSettingEntries(profile) {
  const values = profile?.values || {};
  return Object.keys(values)
    .map((name) => findSettingItemByName(name))
    .filter(Boolean)
    .sort((a, b) => {
      const groupDelta = getSettingGroupOrderIndex(a.group) - getSettingGroupOrderIndex(b.group);
      if (groupDelta) return groupDelta;
      const itemDelta = getSettingItemOrderIndex(a.group, a.item.name) - getSettingItemOrderIndex(b.group, b.item.name);
      if (itemDelta) return itemDelta;
      return String(a.item.name).localeCompare(String(b.item.name));
    });
}

function getValidSettingFavoriteNames() {
  return getFavoriteSettingEntries().map((entry) => entry.item.name).filter(Boolean);
}

function isSettingFavorite(name) {
  return settingFavoritesState.names.includes(String(name || "").trim());
}

function getSettingFavoritesLabel() {
  return getUIText("setting_favorites", "Favorites");
}

function getSettingProfilesLabel() {
  return getUIText("setting_profiles", "Profiles");
}

function getSettingGroupsForDisplay() {
  const groups = SETTINGS?.groups || [];
  const out = [
    {
      group: SETTING_FAVORITES_GROUP,
      count: getFavoriteSettingEntries().length,
      virtual: true,
    },
    ...groups,
  ];
  const profiles = settingProfilesState.profiles || [];
  if (profiles.length) {
    out.push({
      group: SETTING_PROFILES_DIVIDER,
      label: getSettingProfilesLabel(),
      divider: true,
      virtual: true,
    });
    profiles.forEach((profile) => {
      out.push({
        group: settingProfileGroup(profile.id),
        count: getProfileSettingEntries(profile).length,
        label: profile.name,
        profile,
        virtual: true,
      });
    });
  }
  return out;
}

function getSettingItemEntriesForGroup(group) {
  if (isSettingFavoritesGroup(group)) return getFavoriteSettingEntries();
  const profile = getSettingProfileByGroup(group);
  if (profile) return getProfileSettingEntries(profile);
  return (SETTINGS?.items_by_group?.[group] || []).map((item) => ({ group, item }));
}

async function loadSettingFavorites(force = false) {
  if (!force && settingFavoritesState.loaded) return settingFavoritesState.names;
  if (!force && settingFavoritesState.loadPromise) return settingFavoritesState.loadPromise;

  settingFavoritesState.loadPromise = getJson("/api/setting_favorites")
    .then((payload) => {
      settingFavoritesState.loaded = true;
      settingFavoritesState.names = normalizeSettingFavoriteNames(payload?.favorites || []);
      return settingFavoritesState.names;
    })
    .catch(() => {
      settingFavoritesState.loaded = true;
      settingFavoritesState.names = [];
      return settingFavoritesState.names;
    })
    .finally(() => {
      settingFavoritesState.loadPromise = null;
    });

  return settingFavoritesState.loadPromise;
}

function invalidateSettingFavoriteRenderState() {
  settingGroupValueCache.delete(SETTING_FAVORITES_GROUP);
  settingGroupValuePromises.delete(SETTING_FAVORITES_GROUP);
  const itemsBox = document.getElementById("items");
  if (itemsBox?.dataset.renderedGroup === SETTING_FAVORITES_GROUP) {
    delete itemsBox.dataset.renderedGroup;
  }
}

function renderSettingFavoriteMark(name) {
  const active = isSettingFavorite(name);
  return `
    <span class="setting-favorite-mark${active ? " is-active" : ""}" aria-hidden="true">
      <svg viewBox="0 0 24 24" focusable="false">
        <path d="M6 3.5h12a1 1 0 0 1 1 1v16l-7-4-7 4v-16a1 1 0 0 1 1-1z"/>
      </svg>
    </span>
  `;
}

function updateSettingFavoriteRowMarks(root = document.getElementById("items")) {
  if (!root) return;
  root.querySelectorAll(".setting[data-setting-name]").forEach((row) => {
    const active = isSettingFavorite(row.dataset.settingName);
    row.classList.toggle("is-favorite", active);
    const mark = row.querySelector(".setting-favorite-mark");
    if (mark) mark.classList.toggle("is-active", active);
  });
}

function refreshSettingFavoriteChrome(options = {}) {
  const animateGroups = options.animateGroups === true;
  renderGroups({ animateGroups });
  renderSettingSubnav();
  syncSettingGroupChrome(CURRENT_GROUP);
  updateSettingFavoriteRowMarks();
}

async function persistSettingFavorites(nextNames) {
  const payload = await postJson("/api/setting_favorites", {
    favorites: normalizeSettingFavoriteNames(nextNames),
  });
  settingFavoritesState.names = normalizeSettingFavoriteNames(payload?.favorites || nextNames);
  return settingFavoritesState.names;
}

async function toggleSettingFavorite(name) {
  const cleanName = String(name || "").trim();
  if (!cleanName || !findSettingItemByName(cleanName)) return;

  const previous = settingFavoritesState.names.slice();
  const exists = previous.includes(cleanName);
  const next = exists
    ? previous.filter((entry) => entry !== cleanName)
    : [...previous, cleanName];

  settingFavoritesState.names = normalizeSettingFavoriteNames(next);
  invalidateSettingFavoriteRenderState();
  refreshSettingFavoriteChrome({ animateGroups: false });

  if (isSettingFavoritesGroup(CURRENT_GROUP)) {
    const scrollTop = getSettingItemsScrollTop();
    renderItems(SETTING_FAVORITES_GROUP, {
      animateItems: false,
      scrollMode: "restore",
      scrollTop,
    }).catch(() => {});
  }

  try {
    await persistSettingFavorites(getValidSettingFavoriteNames());
    invalidateSettingFavoriteRenderState();
    refreshSettingFavoriteChrome({ animateGroups: false });
    if (navigator.vibrate) navigator.vibrate(12);
    showAppToast(exists
      ? getUIText("setting_favorite_removed", "Removed from favorites")
      : getUIText("setting_favorite_added", "Added to favorites"));
  } catch (e) {
    settingFavoritesState.names = previous;
    invalidateSettingFavoriteRenderState();
    refreshSettingFavoriteChrome({ animateGroups: false });
    if (isSettingFavoritesGroup(CURRENT_GROUP)) {
      renderItems(SETTING_FAVORITES_GROUP, { animateItems: false, scrollMode: "restore" }).catch(() => {});
    }
    showAppToast(e?.message || getUIText("setting_favorites_save_failed", "Failed to save favorites"), { tone: "error" });
  }
}

function getSettingGroupParamNames(group) {
  if (isSettingFavoritesGroup(group)) return getValidSettingFavoriteNames();
  const profile = getSettingProfileByGroup(group);
  if (profile) return getProfileSettingEntries(profile).map((entry) => entry.item.name).filter(Boolean);
  const list = SETTINGS?.items_by_group?.[group] || [];
  return list.map((item) => item.name).filter(Boolean);
}

function cacheSettingValue(name, value, group = null) {
  if (!name) return;
  const loadedAt = Date.now();
  settingValueCache.set(name, { value, loadedAt });
  if (!group) return;
  const cachedGroup = settingGroupValueCache.get(group);
  if (!cachedGroup) return;
  cachedGroup.values[name] = value;
  cachedGroup.loadedAt = loadedAt;
}

function primeSettingGroupValueCache(group, values) {
  if (!group) return;
  const loadedAt = Date.now();
  const snapshot = { values: { ...(values || {}) }, loadedAt };
  settingGroupValueCache.set(group, snapshot);
  Object.entries(snapshot.values).forEach(([name, value]) => {
    settingValueCache.set(name, { value, loadedAt });
  });
}

function applyRestoredSettingValuesToRenderedItems(values) {
  if (!values || typeof values !== "object") return false;
  let updated = false;
  document.querySelectorAll(".setting[data-setting-name]").forEach((row) => {
    const name = row.dataset.settingName;
    if (!name || !(name in values)) return;
    const valueButton = row.querySelector(".val");
    if (!valueButton) return;
    valueButton.textContent = String(values[name]);
    row.classList.add("is-restored-live");
    window.setTimeout(() => row.classList.remove("is-restored-live"), 900);
    updated = true;
  });
  return updated;
}

async function fetchSettingGroupValues(group, options = {}) {
  if (!group) return {};
  const profile = getSettingProfileByGroup(group);
  if (profile) return { ...(profile.values || {}) };
  const force = options.force === true;
  const ttlMs = Number.isFinite(options.ttlMs) ? options.ttlMs : SETTING_VALUES_TTL_MS;
  const names = getSettingGroupParamNames(group);
  if (!names.length) {
    primeSettingGroupValueCache(group, {});
    return {};
  }

  const cachedGroup = settingGroupValueCache.get(group);
  if (!force && cachedGroup && hasFreshPageData(cachedGroup.loadedAt, ttlMs)) {
    return { ...cachedGroup.values };
  }

  if (!force && settingGroupValuePromises.has(group)) {
    return settingGroupValuePromises.get(group);
  }

  const assembledValues = {};
  const missingNames = [];
  names.forEach((name) => {
    const cachedValue = settingValueCache.get(name);
    if (!force && cachedValue && hasFreshPageData(cachedValue.loadedAt, ttlMs)) {
      assembledValues[name] = cachedValue.value;
    } else {
      missingNames.push(name);
    }
  });

  if (!missingNames.length) {
    primeSettingGroupValueCache(group, assembledValues);
    return assembledValues;
  }

  const loadPromise = (async () => {
    const fetchedValues = await bulkGet(missingNames);
    const nextValues = { ...assembledValues, ...(fetchedValues || {}) };
    primeSettingGroupValueCache(group, nextValues);
    return { ...nextValues };
  })().finally(() => {
    settingGroupValuePromises.delete(group);
  });

  settingGroupValuePromises.set(group, loadPromise);
  return loadPromise;
}

async function warmupSettingGroupValues() {
  if (!SETTINGS?.groups?.length) return;
  const groups = SETTINGS.groups
    .map((entry) => entry.group)
    .filter(Boolean)
    .filter((group) => group !== CURRENT_GROUP);

  for (const group of groups) {
    try {
      await fetchSettingGroupValues(group, { ttlMs: SETTING_VALUES_TTL_MS });
    } catch {}
    await new Promise((resolve) => window.setTimeout(resolve, 24));
  }
}

function scheduleSettingGroupValueWarmup(delay = 220) {
  if (!SETTINGS?.groups?.length || settingValueWarmupTimer || settingValueWarmupPromise) return;
  settingValueWarmupTimer = window.setTimeout(() => {
    settingValueWarmupTimer = null;
    requestIdleTask(() => {
      settingValueWarmupPromise = warmupSettingGroupValues()
        .catch(() => {})
        .finally(() => {
          settingValueWarmupPromise = null;
        });
    }, 1200);
  }, Math.max(0, delay));
}

function isMissingCarSelectionLabel(label) {
  const text = String(label || "").trim();
  if (!text || text === "-") return true;
  return text.toLowerCase().includes("mock");
}

function isMissingCarSelectionValues(values) {
  const selected = String(values?.CarSelected3 || "").trim();
  if (!selected) return true;
  const carName = String(values?.CarName || "").trim();
  return isMissingCarSelectionLabel(selected) || (carName && carName.toLowerCase().includes("mock"));
}

function highlightSettingCarEntry() {
  if (!settingCarRow) return;
  settingCarRow.scrollIntoView({ behavior: "smooth", block: "center" });
  try {
    settingCarRow.focus({ preventScroll: true });
  } catch {
    settingCarRow.focus();
  }
  settingCarRow.classList.remove("is-attention");
  void settingCarRow.offsetWidth;
  settingCarRow.classList.add("is-attention");
  window.setTimeout(() => {
    settingCarRow.classList.remove("is-attention");
  }, 3600);
}

async function promptMissingCurrentCarSelection(values = null) {
  if (currentCarPromptActive) return false;
  try {
    if (sessionStorage.getItem(CURRENT_CAR_PROMPT_SESSION_KEY) === "1") return false;
  } catch {}

  let snapshot = values;
  if (!snapshot) {
    try {
      snapshot = await bulkGet(["CarSelected3", "CarName"]);
    } catch {
      return false;
    }
  }

  if (!isMissingCarSelectionValues(snapshot)) return false;

  currentCarPromptActive = true;
  try {
    sessionStorage.setItem(CURRENT_CAR_PROMPT_SESSION_KEY, "1");
  } catch {}

  try {
    await appAlert(getUIText("missing_car_select", "No car is selected.\nPlease select a car in settings first."), {
      title: getUIText("car_select", "Car Select"),
    });

    if (typeof showPage === "function") {
      showPage("setting", true, typeof getSwipeTransition === "function" ? getSwipeTransition(CURRENT_PAGE, "setting") : null);
    }
    if (typeof showSettingScreen === "function") {
      CURRENT_GROUP = null;
      showSettingScreen("groups", false);
    }
    window.setTimeout(highlightSettingCarEntry, 260);
  } finally {
    currentCarPromptActive = false;
  }
  return true;
}
async function loadSettings(options = {}) {
  const background = options.background === true;
  const force = options.force === true;
  const meta = document.getElementById("settingsMeta");

  if (SETTINGS && !force) {
    await loadSettingFavorites();
    await loadSettingProfiles();
    renderGroups({ animateGroups: false });
    renderSettingSubnav();
    syncSettingSearchFabState();
    if (!background && CURRENT_PAGE === "setting" && typeof syncSettingViewportLayout === "function") {
      await syncSettingViewportLayout({ animateChrome: false, animateItems: false });
    }
    return SETTINGS;
  }

  if (!force && settingsLoadPromise) return settingsLoadPromise;
  if (!background && meta) meta.textContent = getUIText("loading", "Loading...");

  settingsLoadPromise = (async () => {
    const j = await getJson("/api/settings");

    SETTINGS = j;
    UNIT_CYCLE = j.unit_cycle || UNIT_CYCLE;
    settingValueCache.clear();
    settingGroupValueCache.clear();
    settingGroupValuePromises.clear();
    await loadSettingFavorites(force);
    await loadSettingProfiles(force);
    rebuildSettingSearchEntries();

    if (meta) {
      meta.textContent = `path: ${j.path} | has_params: ${j.has_params} | type_api: ${j.has_param_type}`;
      if (!DEBUG_UI) {
        meta.style.display = "none";
      }
    }

    if (!DEBUG_UI) {
      const gm = document.getElementById("groupMeta");
      if (gm) gm.style.display = "none";
      const cm = document.getElementById("carMeta");
      if (cm) cm.style.display = "none";
    }

    renderGroups();
    renderSettingSubnav();
    syncSettingSearchFabState();
    scheduleSettingGroupValueWarmup(260);

    if (!background || CURRENT_PAGE === "setting") {
      CURRENT_GROUP = null;
      if (isCompactLandscapeMode()) {
        const initialGroup = getLandscapeDefaultSettingGroup();
        if (initialGroup) await activateSettingGroup(initialGroup, false);
        else showSettingScreen("groups", false);
      } else {
        showSettingScreen("groups", false);
      }
      if (settingSearchPanel && !settingSearchPanel.hidden) {
        renderSettingSearchResults(settingSearchInput?.value || "");
      }
    }

    return SETTINGS;
  })().catch((e) => {
    settingSearchEntries = [];
    if (!background && meta) meta.textContent = "Failed: " + (e?.message || "unknown");
    throw e;
  }).finally(() => {
    settingsLoadPromise = null;
  });

  return settingsLoadPromise;
}

let settingOverflowSyncRaf = 0;
let settingOverflowSyncTimer = 0;
let settingOverflowResizeObserver = null;

function measureSettingGroupButtonOverflow(button) {
  if (!button) return;
  const labelEl = button.querySelector(".setting-group-label");
  if (!labelEl) return;
  const buttonWidth = button.clientWidth || 0;
  if (buttonWidth <= 0) return;
  const shift = Math.min(0, buttonWidth - labelEl.scrollWidth - 8);
  button.style.setProperty("--setting-label-shift", `${shift}px`);
  button.classList.toggle("is-overflowing", shift < 0);
}

function syncSettingGroupLabelOverflow(root = document) {
  const scope = root && typeof root.querySelectorAll === "function" ? root : document;
  if (scope.matches?.("#groupList .groupBtn, #deviceGroupList .groupBtn")) {
    measureSettingGroupButtonOverflow(scope);
  }
  const selector = (scope.id === "groupList" || scope.id === "deviceGroupList")
    ? ".groupBtn"
    : "#groupList .groupBtn, #deviceGroupList .groupBtn";
  scope.querySelectorAll(selector).forEach(measureSettingGroupButtonOverflow);
}

function syncSettingOverflow(root = document) {
  syncSettingMarqueeOverflow(root);
  syncSettingGroupLabelOverflow(root);
}

function scheduleSettingOverflowSync(root = document, delayMs = 0) {
  if (settingOverflowSyncRaf) cancelAnimationFrame(settingOverflowSyncRaf);
  if (settingOverflowSyncTimer) {
    window.clearTimeout(settingOverflowSyncTimer);
    settingOverflowSyncTimer = 0;
  }

  const run = () => {
    settingOverflowSyncRaf = requestAnimationFrame(() => {
      settingOverflowSyncRaf = 0;
      syncSettingOverflow(root);
    });
  };

  if (delayMs > 0) {
    settingOverflowSyncTimer = window.setTimeout(() => {
      settingOverflowSyncTimer = 0;
      run();
    }, delayMs);
  } else {
    run();
  }
}

function initSettingOverflowObservers() {
  if (settingOverflowResizeObserver || typeof ResizeObserver !== "function") return;
  settingOverflowResizeObserver = new ResizeObserver(() => scheduleSettingOverflowSync(document));
  [
    "settingScreenHost",
    "settingScreenGroups",
    "settingScreenItems",
    "groupList",
    "deviceGroupList",
    "items",
    "deviceItems",
  ].forEach((id) => {
    const el = document.getElementById(id);
    if (el) settingOverflowResizeObserver.observe(el);
  });
}

function renderGroups(options = {}) {
  const box = document.getElementById("groupList");
  const animateGroups = options.animateGroups !== false;
  const groups = getSettingGroupsForDisplay();
  const signature = groups.map((g) => isSettingProfilesDivider(g) ? SETTING_PROFILES_DIVIDER : `${g.group}:${g.count ?? ""}:${g.label || ""}`).join("|");

  function setGroupButtonLabel(button, label, count) {
    const text = Number.isFinite(Number(count)) ? `${label} (${count})` : label;
    button.title = text;
    button.innerHTML = `<span class="setting-group-label">${escapeHtml(text)}</span>`;
    requestAnimationFrame(() => measureSettingGroupButtonOverflow(button));
  }

  if (!animateGroups && box.dataset.groupsSignature === signature && box.children.length === groups.length) {
    Array.from(box.children).forEach((button, index) => {
      const g = groups[index];
      if (isSettingProfilesDivider(g)) {
        button.className = "setting-profile-divider";
        button.innerHTML = `<span></span><strong>${escapeHtml(g.label || getSettingProfilesLabel())}</strong><span></span>`;
        button.removeAttribute("data-group");
        button.onclick = null;
        return;
      }
      const label = getSettingGroupLabel(g.group);
      button.className = "btn groupBtn";
      if (isSettingFavoritesGroup(g.group)) button.classList.add("groupBtn--favorites");
      if (isSettingProfileGroup(g.group)) button.classList.add("groupBtn--profile");
      if (g.group === CURRENT_GROUP) button.classList.add("active");
      button.dataset.group = g.group;
      setGroupButtonLabel(button, label, g.count);
      button.onclick = () => selectGroup(g.group);
    });
    scheduleSettingOverflowSync(box);
    return;
  }

  box.innerHTML = "";
  box.dataset.groupsSignature = signature;

  groups.forEach(g => {
    if (isSettingProfilesDivider(g)) {
      const divider = document.createElement("div");
      divider.className = animateGroups ? "setting-profile-divider ui-stagger-item" : "setting-profile-divider";
      if (animateGroups) divider.style.setProperty("--i", String(box.children.length));
      divider.innerHTML = `<span></span><strong>${escapeHtml(g.label || getSettingProfilesLabel())}</strong><span></span>`;
      box.appendChild(divider);
      return;
    }

    const label = getSettingGroupLabel(g.group);

    const b = document.createElement("button");
    b.className = animateGroups ? "btn groupBtn ui-stagger-item" : "btn groupBtn";
    if (isSettingFavoritesGroup(g.group)) b.classList.add("groupBtn--favorites");
    if (isSettingProfileGroup(g.group)) b.classList.add("groupBtn--profile");
    if (animateGroups) b.style.setProperty("--i", String(box.children.length));
    if (g.group === CURRENT_GROUP) b.classList.add("active");
    b.dataset.group = g.group;
    setGroupButtonLabel(b, label, g.count);
    b.onclick = () => selectGroup(g.group);
    box.appendChild(b);
  });
  scheduleSettingOverflowSync(box);
}

function getSettingGroupMeta(group) {
  if (isSettingFavoritesGroup(group)) {
    return {
      group,
      egroup: "Favorites",
      count: getFavoriteSettingEntries().length,
      virtual: true,
    };
  }
  const profile = getSettingProfileByGroup(group);
  if (profile) {
    return {
      group,
      egroup: profile.name,
      count: getProfileSettingEntries(profile).length,
      profile,
      virtual: true,
    };
  }
  const groups = SETTINGS?.groups || [];
  return groups.find((entry) => entry.group === group) || null;
}

function getSettingGroupLabel(group) {
  if (isSettingFavoritesGroup(group)) return getSettingFavoritesLabel();
  const profile = getSettingProfileByGroup(group);
  if (profile) return profile.name;
  const meta = getSettingGroupMeta(group);
  if (!meta) return group;
  if (LANG === "zh") return meta.cgroup || meta.egroup || meta.group;
  if (LANG === "ko") return meta.group || meta.egroup || group;
  return meta.egroup || meta.group || group;
}

const SETTING_SUBNAV_PAGE_STEP = 1;
let settingGroupTransitionLock = false;
let settingRenderToken = 0;
let pendingSettingFocus = null;
let settingFocusClearTimer = null;
let settingSearchDebounceTimer = null;
let settingSearchEntries = [];
let settingSearchScope = { type: "all", profileId: "" };
const settingPageRoot = document.getElementById("pageSetting");
let settingFabMenuOpen = false;

function isCompactLandscapeMode() {
  return window.matchMedia("(orientation: landscape)").matches;
}

function isFixedPortraitSettingSubnavMode() {
  return window.matchMedia("(max-width: 640px) and (orientation: portrait)").matches;
}

function syncSettingSubnavFixedOffset() {
  if (!settingSubnavWrap || !screenItems) return;

  const shouldFix =
    CURRENT_PAGE === "setting" &&
    isFixedPortraitSettingSubnavMode() &&
    screenItems.style.display !== "none" &&
    settingSubnavWrap.style.display !== "none" &&
    !settingPageRoot?.classList.contains("setting-profile-active");

  if (!shouldFix) {
    document.documentElement.style.removeProperty("--setting-fixed-subnav-height");
    return;
  }

  const height = Math.ceil(settingSubnavWrap.getBoundingClientRect().height || settingSubnavWrap.offsetHeight || 0);
  if (height > 0) {
    document.documentElement.style.setProperty("--setting-fixed-subnav-height", `${height}px`);
  }
}

function getLandscapeDefaultSettingGroup() {
  const groups = SETTINGS?.groups || [];
  if (!groups.length) return null;

  const match = groups.find((entry) => {
    const raw = String(entry.group || "").trim().toLowerCase();
    const label = String(getSettingGroupLabel(entry.group) || "").trim().toLowerCase();
    return raw === "시작" || raw === "start" || label === "시작" || label === "start";
  });

  return match?.group || CURRENT_GROUP || groups[0]?.group || null;
}

function syncSettingSearchFabState() {
  const isOpen = Boolean(settingSearchPanel && !settingSearchPanel.hidden);
  if (settingPageRoot) settingPageRoot.classList.toggle("setting-search-open", isOpen);
  if (btnSettingSearch) {
    btnSettingSearch.classList.toggle("active", isOpen || settingFabMenuOpen);
    btnSettingSearch.setAttribute("aria-expanded", settingFabMenuOpen ? "true" : "false");
  }
}

function normalizeSettingProfiles(profiles) {
  return (Array.isArray(profiles) ? profiles : [])
    .filter((profile) => profile && profile.id && profile.name && profile.values)
    .map((profile) => ({
      ...profile,
      values: { ...(profile.values || {}) },
      meta: { ...(profile.meta || {}) },
    }));
}

async function loadSettingProfiles(force = false) {
  if (!force && settingProfilesState.loaded) return settingProfilesState.profiles;
  if (!force && settingProfilesState.loadPromise) return settingProfilesState.loadPromise;

  settingProfilesState.loadPromise = getJson("/api/setting_profiles")
    .then((payload) => {
      settingProfilesState.loaded = true;
      settingProfilesState.profiles = normalizeSettingProfiles(payload?.profiles || []);
      return settingProfilesState.profiles;
    })
    .catch(() => {
      settingProfilesState.loaded = true;
      settingProfilesState.profiles = [];
      return settingProfilesState.profiles;
    })
    .finally(() => {
      settingProfilesState.loadPromise = null;
    });

  return settingProfilesState.loadPromise;
}

function updateSettingProfilesFromPayload(payload) {
  if (!payload || !Array.isArray(payload.profiles)) return;
  settingProfilesState.loaded = true;
  settingProfilesState.profiles = normalizeSettingProfiles(payload.profiles);
}

function formatSettingProfileDate(value) {
  const raw = String(value || "").trim();
  if (!raw) return "";
  const date = new Date(raw);
  if (Number.isNaN(date.getTime())) return raw;
  try {
    return date.toLocaleString(LANG === "ko" ? "ko-KR" : undefined, {
      year: "numeric",
      month: "2-digit",
      day: "2-digit",
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      hour12: false,
    });
  } catch {
    return raw;
  }
}

function settingProfileMetaRows(profile) {
  const meta = profile?.meta || {};
  const rows = [];
  if (profile?.created_at) {
    rows.push([getUIText("setting_profile_created", "Created"), settingsDiffEscape(formatSettingProfileDate(profile.created_at))]);
  }
  if (meta.branch) rows.push([getUIText("branch", "Branch"), settingsDiffEscape(meta.branch)]);
  if (meta.commit) {
    const commitText = meta.commit_short || String(meta.commit).slice(0, 7);
    const commitValue = meta.commit_url
      ? `<a href="${settingsDiffEscape(meta.commit_url)}" target="_blank" rel="noopener">${settingsDiffEscape(commitText)}</a>`
      : settingsDiffEscape(commitText);
    rows.push([getUIText("commit", "Commit"), commitValue]);
  }
  return rows;
}

async function openSettingProfileInfo(profile) {
  const rows = settingProfileMetaRows(profile);
  const messageHtml = rows.length
    ? `<div class="setting-profile-info">${rows.map(([label, value]) => `
        <div class="setting-profile-panel__metaRow">
          <span>${settingsDiffEscape(label)}</span>
          <strong>${value}</strong>
        </div>
      `).join("")}</div>`
    : `<div class="setting-profile-info setting-profile-info--empty">${settingsDiffEscape(getUIText("setting_profile_info_empty", "No profile metadata"))}</div>`;
  await openAppDialog({
    mode: "alert",
    title: getUIText("setting_profile_info", "Profile Info"),
    html: true,
    messageHtml,
    confirmLabel: getUIText("ok", "OK"),
  });
}

async function saveSettingProfile(profileId, updates) {
  const payload = await postJson("/api/setting_profiles/update", { id: profileId, ...(updates || {}) });
  updateSettingProfilesFromPayload(payload);
  return payload.profile || getSettingProfileById(profileId);
}

async function createSettingProfileFromCurrent() {
  closeSettingFabMenu();
  const name = await appPrompt(getUIText("setting_profile_create_prompt", "Enter a profile name."), {
    title: getUIText("setting_profile_create_title", "Add Profile"),
    placeholder: getUIText("setting_profile_name", "Profile name"),
  });
  if (!name || !String(name).trim()) return;

  try {
    const payload = await postJson("/api/setting_profiles", { name: String(name).trim() });
    updateSettingProfilesFromPayload(payload);
    const profile = payload.profile;
    renderGroups({ animateGroups: false });
    renderSettingSubnav();
    if (profile?.id) {
      await selectGroup(settingProfileGroup(profile.id));
      showAppToast(getUIText("setting_profile_saved", "Profile saved"));
    }
  } catch (e) {
    showAppToast(e?.message || getUIText("setting_profile_save_failed", "Failed to save profile"), { tone: "error" });
  }
}

function setSettingProfileDialogClass(enabled) {
  if (typeof appDialog !== "undefined" && appDialog) {
    appDialog.classList.toggle("app-dialog--settings-diff", Boolean(enabled));
  }
}

async function applySettingProfile(profile) {
  if (!profile?.id) return;
  let preview = null;
  try {
    const payload = await postJson("/api/setting_profiles/preview", { id: profile.id, values: profile.values || {} });
    preview = payload.preview;
  } catch (e) {
    showAppToast(e?.message || getUIText("setting_profile_apply_failed", "Failed to preview profile"), { tone: "error" });
    return;
  }

  const selected = typeof getSettingsDiffSelectedCount === "function" ? getSettingsDiffSelectedCount(preview) : 0;
  const html = `
    <div class="setting-profile-apply">
      <div class="setting-profile-apply__title">${settingsDiffEscape(profile.name)}</div>
      ${typeof renderSettingsDiffHtml === "function" ? renderSettingsDiffHtml(preview, {
        nextLabel: getUIText("setting_profile_value", "Profile"),
      }) : ""}
    </div>
  `;
  const promise = openAppDialog({
    mode: selected > 0 ? "confirm" : "alert",
    title: getUIText("setting_profile_apply_title", "Apply Profile"),
    html: true,
    messageHtml: html,
    confirmLabel: getUIText("apply", "Apply"),
    cancelLabel: getUIText("cancel", "Cancel"),
  });
  setSettingProfileDialogClass(true);
  const ok = await promise.finally(() => setSettingProfileDialogClass(false));
  if (selected <= 0 || !ok) return;

  try {
    const result = await postJson("/api/setting_profiles/apply", { id: profile.id, values: profile.values || {} });
    const failed = new Set((result.result?.fails || []).map((entry) => String(entry?.key || "")).filter(Boolean));
    const restoredValues = {};
    (result.preview?.entries || []).forEach((entry) => {
      if (!entry?.apply || failed.has(String(entry.key))) return;
      restoredValues[entry.key] = entry.value;
    });
    if (Object.keys(restoredValues).length) {
      window.dispatchEvent(new CustomEvent("carrot:paramsrestored", {
        detail: { source: "setting_profile", values: restoredValues },
      }));
      Object.entries(restoredValues).forEach(([name, value]) => {
        window.dispatchEvent(new CustomEvent("carrot:paramchange", {
          detail: { name, value, source: "setting_profile" },
        }));
      });
    }
    showAppToast(getUIText("setting_profile_apply_done", "Profile applied"));
  } catch (e) {
    showAppToast(e?.message || getUIText("setting_profile_apply_failed", "Failed to apply profile"), { tone: "error" });
  }
}

async function deleteSettingProfile(profile) {
  if (!profile?.id) return;
  const ok = await appConfirm(getUIText("setting_profile_delete_confirm", "Delete this profile?\n{name}", { name: profile.name }), {
    title: getUIText("setting_profile_delete", "Delete Profile"),
    confirmLabel: getUIText("delete", "Delete"),
  });
  if (!ok) return;

  try {
    const payload = await postJson("/api/setting_profiles/delete", { id: profile.id });
    updateSettingProfilesFromPayload(payload);
    CURRENT_GROUP = null;
    renderGroups({ animateGroups: false });
    renderSettingSubnav();
    showSettingScreen("groups", false);
    showAppToast(getUIText("setting_profile_deleted", "Profile deleted"));
  } catch (e) {
    showAppToast(e?.message || getUIText("setting_profile_save_failed", "Failed to save profile"), { tone: "error" });
  }
}

function closeSettingProfileActionMenus(exceptPanel = null) {
  document.querySelectorAll(".setting-profile-action-menu.is-open").forEach((menu) => {
    if (exceptPanel && menu === exceptPanel) return;
    menu.classList.remove("is-open");
    const button = menu.querySelector(".setting-profile-action-menu__button");
    const panel = menu.querySelector(".setting-profile-action-menu__panel");
    if (button) button.setAttribute("aria-expanded", "false");
    if (panel) {
      panel.hidden = true;
      panel.setAttribute("aria-hidden", "true");
    }
  });
}

function makeSettingProfileMenuItem(label, onClick, className = "") {
  const button = document.createElement("button");
  button.type = "button";
  button.className = `setting-profile-action-menu__item ui-dropdown-menu__item${className ? ` ${className}` : ""}`;
  button.setAttribute("role", "menuitem");
  button.textContent = label;
  button.onclick = (event) => {
    event.stopPropagation();
    closeSettingProfileActionMenus();
    onClick();
  };
  return button;
}

function appendSettingProfileHeader(profile, container) {
  const panel = document.createElement("div");
  panel.className = "setting-profile-panel";

  const titleRow = document.createElement("div");
  titleRow.className = "setting-profile-panel__titleRow";
  const input = document.createElement("input");
  input.className = "setting-profile-panel__name";
  input.type = "text";
  input.maxLength = 40;
  input.value = profile.name || "";
  input.setAttribute("aria-label", getUIText("setting_profile_name", "Profile name"));
  let nameSaveTimer = 0;
  let nameSaveInFlight = null;
  async function persistProfileName() {
    const nextName = input.value.trim();
    if (!nextName) {
      input.value = profile.name || "";
      return;
    }
    if (nextName === profile.name) return;
    if (nameSaveInFlight) {
      try { await nameSaveInFlight; } catch {}
      if (nextName === profile.name) return;
    }
    try {
      input.classList.add("is-saving");
      nameSaveInFlight = saveSettingProfile(profile.id, { name: nextName });
      const nextProfile = await nameSaveInFlight;
      if (nextProfile) profile.name = nextProfile.name;
      if (itemsTitle) itemsTitle.textContent = profile.name;
      renderGroups({ animateGroups: false });
      renderSettingSubnav();
    } catch (e) {
      showAppToast(e?.message || getUIText("setting_profile_save_failed", "Failed to save profile"), { tone: "error" });
    } finally {
      nameSaveInFlight = null;
      input.classList.remove("is-saving");
    }
  }
  function scheduleProfileNameSave(delay = 500) {
    if (nameSaveTimer) clearTimeout(nameSaveTimer);
    nameSaveTimer = window.setTimeout(() => {
      nameSaveTimer = 0;
      persistProfileName().catch(() => {});
    }, delay);
  }
  input.addEventListener("input", () => scheduleProfileNameSave());
  input.addEventListener("blur", () => {
    if (nameSaveTimer) {
      clearTimeout(nameSaveTimer);
      nameSaveTimer = 0;
    }
    persistProfileName().catch(() => {});
  });
  input.addEventListener("keydown", (event) => {
    if (event.key !== "Enter") return;
    event.preventDefault();
    if (nameSaveTimer) {
      clearTimeout(nameSaveTimer);
      nameSaveTimer = 0;
    }
    persistProfileName().then(() => input.blur()).catch(() => {});
  });
  const menu = document.createElement("div");
  menu.className = "setting-profile-action-menu ui-dropdown-menu";
  const menuBtn = document.createElement("button");
  menuBtn.type = "button";
  menuBtn.className = "setting-profile-action-menu__button ui-dropdown-menu__button";
  menuBtn.setAttribute("aria-haspopup", "menu");
  menuBtn.setAttribute("aria-expanded", "false");
  menuBtn.setAttribute("aria-label", getUIText("setting_profile_menu", "Profile menu"));
  menuBtn.innerHTML = `
    <svg viewBox="0 0 24 24" aria-hidden="true" focusable="false">
      <path fill="currentColor" d="M12 8a2 2 0 1 0 0-4 2 2 0 0 0 0 4m0 2a2 2 0 1 0 0 4 2 2 0 0 0 0-4m0 6a2 2 0 1 0 0 4 2 2 0 0 0 0-4"/>
    </svg>
  `;
  const menuPanel = document.createElement("div");
  menuPanel.className = "setting-profile-action-menu__panel ui-dropdown-menu__panel";
  menuPanel.setAttribute("role", "menu");
  menuPanel.setAttribute("aria-hidden", "true");
  menuPanel.hidden = true;
  menuPanel.appendChild(makeSettingProfileMenuItem(
    getUIText("setting_profile_search", "Search Profile"),
    () => openSettingSearchPanel({ scope: { type: "profile", profileId: profile.id } }).catch(() => {}),
  ));
  menuPanel.appendChild(makeSettingProfileMenuItem(
    getUIText("setting_profile_info", "Info"),
    () => openSettingProfileInfo(profile),
  ));
  menuPanel.appendChild(makeSettingProfileMenuItem(
    getUIText("apply", "Apply"),
    () => applySettingProfile(profile),
    "setting-profile-action-menu__item--primary",
  ));
  menuPanel.appendChild(makeSettingProfileMenuItem(
    getUIText("delete", "Delete"),
    () => deleteSettingProfile(profile),
    "setting-profile-action-menu__item--danger",
  ));
  menuBtn.onclick = (event) => {
    event.stopPropagation();
    const nextOpen = !menu.classList.contains("is-open");
    closeSettingProfileActionMenus(menu);
    menu.classList.toggle("is-open", nextOpen);
    menuBtn.setAttribute("aria-expanded", nextOpen ? "true" : "false");
    menuPanel.hidden = !nextOpen;
    menuPanel.setAttribute("aria-hidden", nextOpen ? "false" : "true");
  };
  menu.appendChild(menuBtn);
  menu.appendChild(menuPanel);
  titleRow.appendChild(input);
  titleRow.appendChild(menu);

  panel.appendChild(titleRow);
  container.appendChild(panel);
}

let settingFabMenuCloseTimer = null;
const SETTING_FAB_MENU_CLOSE_MS = 240;

function syncSettingFabMenuState() {
  if (settingFabActions) {
    if (settingFabMenuCloseTimer) {
      window.clearTimeout(settingFabMenuCloseTimer);
      settingFabMenuCloseTimer = null;
    }
    if (settingFabMenuOpen && settingFabActions.hidden) {
      // Make the element renderable in its closed state first, then let the
      // next style recalc apply the open class so the transition plays.
      settingFabActions.hidden = false;
      void settingFabActions.offsetWidth; // commit closed-state baseline
    }
  }
  if (settingFabMenu) settingFabMenu.classList.toggle("is-open", settingFabMenuOpen);
  if (settingFabActions) {
    settingFabActions.setAttribute("aria-hidden", settingFabMenuOpen ? "false" : "true");
    if (!settingFabMenuOpen && !settingFabActions.hidden) {
      // Defer [hidden] until the close transition finishes — otherwise
      // `display: none` snaps it away with no animation.
      settingFabMenuCloseTimer = window.setTimeout(() => {
        settingFabMenuCloseTimer = null;
        if (!settingFabMenuOpen) settingFabActions.hidden = true;
      }, SETTING_FAB_MENU_CLOSE_MS);
    }
  }
  if (btnSettingSearch) {
    btnSettingSearch.classList.toggle("active", settingFabMenuOpen || Boolean(settingSearchPanel && !settingSearchPanel.hidden));
    btnSettingSearch.setAttribute("aria-expanded", settingFabMenuOpen ? "true" : "false");
  }
}

function closeSettingFabMenu() {
  if (!settingFabMenuOpen) return;
  settingFabMenuOpen = false;
  syncSettingFabMenuState();
}

function toggleSettingFabMenu() {
  settingFabMenuOpen = !settingFabMenuOpen;
  syncSettingFabMenuState();
}

function mountSettingSearchOverlay() {
  if (settingSearchBackdrop && settingSearchBackdrop.parentElement !== document.body) {
    document.body.appendChild(settingSearchBackdrop);
  }
  if (settingSearchPanel && settingSearchPanel.parentElement !== document.body) {
    document.body.appendChild(settingSearchPanel);
  }
}

function makeSettingSearchEntry({ source, profile = null, group, item }) {
  const groupLabel = getSettingGroupLabel(group);
  const title = formatItemText(item, "title", "etitle", "");
  const descr = formatItemText(item, "descr", "edescr", "");
  const isProfile = source === "profile" && profile?.id;
  const profileName = isProfile ? String(profile.name || "") : "";
  const sourceLabel = isProfile
    ? getUIText("setting_search_source_profile", "Profile")
    : getUIText("setting_search_source_carrot", "CarrotPilot");
  const contextLabel = isProfile
    ? `${profileName} / ${groupLabel}`
    : groupLabel;

  return {
    source: isProfile ? "profile" : "carrot",
    sourceLabel,
    profileId: isProfile ? profile.id : "",
    profileName,
    group: isProfile ? settingProfileGroup(profile.id) : group,
    originalGroup: group,
    groupLabel,
    contextLabel,
    name: item.name,
    title,
    descr,
    haystack: [sourceLabel, profileName, groupLabel, item.name, title, descr].join("\n").toLowerCase(),
  };
}

function rebuildSettingSearchEntries() {
  const groups = SETTINGS?.groups || [];
  const entries = [];

  groups.forEach((groupMeta) => {
    const group = groupMeta.group;
    const groupLabel = getSettingGroupLabel(group);
    const list = SETTINGS?.items_by_group?.[group] || [];

    list.forEach((item) => {
      entries.push(makeSettingSearchEntry({ source: "carrot", group, item }));
    });
  });

  (settingProfilesState.profiles || []).forEach((profile) => {
    getProfileSettingEntries(profile).forEach((entry) => {
      entries.push(makeSettingSearchEntry({
        source: "profile",
        profile,
        group: entry.group,
        item: entry.item,
      }));
    });
  });

  settingSearchEntries = entries;
  return entries;
}

function getSettingSearchEntries() {
  return settingSearchEntries;
}

function highlightSettingSearchText(text, query) {
  const raw = String(text ?? "");
  const q = String(query || "").trim().toLowerCase();
  if (!raw || !q) return escapeHtml(raw);

  const lower = raw.toLowerCase();
  const start = lower.indexOf(q);
  if (start < 0) return escapeHtml(raw);

  const end = start + q.length;
  return `${escapeHtml(raw.slice(0, start))}<mark class="setting-search-result__mark">${escapeHtml(raw.slice(start, end))}</mark>${escapeHtml(raw.slice(end))}`;
}

function getSettingSearchScopeLabel() {
  if (settingSearchScope.type === "profile") {
    const profile = getSettingProfileById(settingSearchScope.profileId);
    return profile?.name || getUIText("setting_search_source_profile", "Profile");
  }
  return getUIText("setting_search_all", "All settings");
}

function clearSettingItemFocus() {
  if (settingFocusClearTimer) {
    clearTimeout(settingFocusClearTimer);
    settingFocusClearTimer = null;
  }
  document.querySelectorAll(".setting.is-focus-hit").forEach((el) => el.classList.remove("is-focus-hit"));
}

const settingGroupScrollTops = new Map();
let settingViewportSyncTimer = null;
let settingViewportLayoutSignature = null;

function getSettingViewportLayoutSignature() {
  const width = Math.round(window.innerWidth || document.documentElement.clientWidth || 0);
  return {
    compactLandscape: isCompactLandscapeMode(),
    width,
  };
}

function hasSettingViewportLayoutChanged() {
  const next = getSettingViewportLayoutSignature();
  const prev = settingViewportLayoutSignature;
  settingViewportLayoutSignature = next;
  if (!prev) return true;
  return (
    prev.compactLandscape !== next.compactLandscape ||
    Math.abs(prev.width - next.width) > 2
  );
}

function getSettingItemsScrollContainer() {
  if (isCompactLandscapeMode() && screenItems) return screenItems;
  return document.scrollingElement || document.documentElement || document.body;
}

function getSettingItemsScrollTop() {
  const scroller = getSettingItemsScrollContainer();
  if (!scroller) return 0;
  if (
    scroller === document.body ||
    scroller === document.documentElement ||
    scroller === document.scrollingElement
  ) {
    return window.scrollY || document.documentElement.scrollTop || document.body.scrollTop || 0;
  }
  return scroller.scrollTop || 0;
}

function setSettingItemsScrollTop(top = 0) {
  const nextTop = Math.max(0, Number(top) || 0);
  const scroller = getSettingItemsScrollContainer();
  if (!scroller) return;
  if (
    scroller === document.body ||
    scroller === document.documentElement ||
    scroller === document.scrollingElement
  ) {
    window.scrollTo(0, nextTop);
    return;
  }
  scroller.scrollTop = nextTop;
}

function settleSettingScreenVisibility(which) {
  if (!screenGroups || !screenItems) return;
  const isGroups = which === "groups";
  const showEl = isGroups ? screenGroups : screenItems;
  const hideEl = isGroups ? screenItems : screenGroups;
  showEl.style.display = "";
  showEl.classList.remove("hidden");
  hideEl.classList.add("hidden");
  hideEl.style.display = "none";
}

function saveCurrentSettingScrollPosition(group = CURRENT_GROUP) {
  if (!group) return;
  settingGroupScrollTops.set(group, getSettingItemsScrollTop());
}

function getSavedSettingScrollPosition(group) {
  return settingGroupScrollTops.get(group) || 0;
}

function resetSettingItemsViewport() {
  setSettingItemsScrollTop(0);
}

function hasRenderedSettingItems(group = CURRENT_GROUP) {
  const itemsBox = document.getElementById("items");
  if (!itemsBox || !group) return false;
  return itemsBox.dataset.renderedGroup === group && itemsBox.childElementCount > 0;
}

function isCarrotSettingTabActive() {
  return !(typeof getCurrentSettingTab === "function" && getCurrentSettingTab() === "device");
}

function syncSettingGroupChrome(group = CURRENT_GROUP) {
  const meta = document.getElementById("groupMeta");
  const list = getSettingItemEntriesForGroup(group);
  if (meta && group) meta.textContent = `${group} / ${list.length}`;
  const groupLabel = group ? getSettingGroupLabel(group) : "";
  if (group) {
    settingTitle.textContent = (UI_STRINGS[LANG].setting || "Setting") + " - " + groupLabel;
    if (itemsTitle) itemsTitle.textContent = groupLabel;
  }
}

function settingMarqueeHtml(text, className) {
  const safe = escapeHtml(text);
  return `<div class="${className} setting-marquee"><span class="setting-marquee__content">${safe}</span></div>`;
}

function syncSettingMarqueeOverflow(root = document) {
  root.querySelectorAll(".setting-marquee").forEach((el) => {
    const content = el.querySelector(".setting-marquee__content");
    if (!content) return;
    const elWidth = el.clientWidth || 0;
    if (elWidth <= 0) return;
    const overflow = content.scrollWidth > el.clientWidth + 2;
    const distance = Math.max(0, content.scrollWidth - el.clientWidth + 18);
    const nextDistance = `${distance}px`;
    const prevDistance = el.style.getPropertyValue("--setting-marquee-distance");
    const wasOverflowing = el.classList.contains("is-overflowing");
    el.style.setProperty("--setting-marquee-distance", nextDistance);
    el.scrollLeft = 0;
    if (!overflow) {
      el.classList.remove("is-overflowing");
      content.style.animation = "";
      return;
    }

    if (!wasOverflowing || prevDistance !== nextDistance) {
      el.classList.remove("is-overflowing");
      content.style.animation = "none";
      void content.offsetWidth;
      content.style.animation = "";
    }
    el.classList.toggle("is-overflowing", overflow);
  });
}

function focusSettingItem(name, behavior = "smooth") {
  const itemsBox = document.getElementById("items");
  if (!itemsBox || !name) return false;

  const target = Array.from(itemsBox.querySelectorAll(".setting")).find(
    (el) => el.dataset.settingName === name,
  );
  if (!target) return false;

  const section = target.closest(".setting-profile-section");
  if (section?.classList.contains("is-collapsed")) {
    section.classList.remove("is-collapsed");
    section.querySelector(".setting-profile-section__header")?.setAttribute("aria-expanded", "true");
  }

  clearSettingItemFocus();
  target.classList.add("is-focus-hit");
  target.scrollIntoView({ behavior, block: "center" });

  settingFocusClearTimer = window.setTimeout(() => {
    target.classList.remove("is-focus-hit");
    settingFocusClearTimer = null;
  }, 2200);

  pendingSettingFocus = null;
  return true;
}

function closeSettingSearchPanel(options = {}) {
  const syncHistory = Boolean(options.syncHistory);
  const fromHistory = Boolean(options.fromHistory);
  if (settingSearchDebounceTimer) {
    clearTimeout(settingSearchDebounceTimer);
    settingSearchDebounceTimer = null;
  }
  if (settingSearchPanel) {
    settingSearchPanel.hidden = true;
    settingSearchPanel.setAttribute("aria-hidden", "true");
  }
  if (settingSearchBackdrop) settingSearchBackdrop.hidden = true;
  syncSettingSearchFabState();

  if (settingSearchInput) {
    settingSearchInput.value = "";
    settingSearchInput.placeholder = getUIText("setting_search_placeholder", "Search name, description, group");
    settingSearchInput.removeAttribute("aria-label");
  }
  if (settingSearchResults) settingSearchResults.innerHTML = "";
  settingSearchScope = { type: "all", profileId: "" };
  syncModalBodyLock();

  const state = history.state || {};
  if (!fromHistory && state.page === "setting" && state.search) {
    if (syncHistory) history.back();
    else history.replaceState({
      page: "setting",
      screen: (screenItems && screenItems.style.display !== "none") ? "items" : "groups",
      group: CURRENT_GROUP || null,
    }, "");
  }
}

function renderSettingSearchResults(query = "") {
  if (!settingSearchResults) return;

  const trimmed = String(query || "").trim();
  if (!SETTINGS) {
    settingSearchResults.innerHTML = "";
    return;
  }

  if (!trimmed) {
    settingSearchResults.innerHTML = "";
    return;
  }

  if (!settingSearchEntries.length && SETTINGS) {
    rebuildSettingSearchEntries();
  }

  const q = trimmed.toLowerCase();
  const matches = getSettingSearchEntries()
    .filter((entry) => {
      if (!entry.haystack.includes(q)) return false;
      if (settingSearchScope.type === "profile") {
        return entry.source === "profile" && entry.profileId === settingSearchScope.profileId;
      }
      return true;
    })
    .slice(0, 36);

  settingSearchResults.innerHTML = "";

  if (!matches.length) {
    const empty = document.createElement("div");
    empty.className = "setting-search-result setting-search-result--empty";
    empty.textContent = getUIText("setting_search_empty", "No matching settings found.");
    settingSearchResults.appendChild(empty);
    return;
  }

  const sections = [
    {
      key: "carrot",
      title: getUIText("setting_search_source_carrot", "CarrotPilot"),
      entries: matches.filter((entry) => entry.source === "carrot"),
    },
    {
      key: "profile",
      title: getUIText("setting_search_source_profile", "Profile"),
      entries: matches.filter((entry) => entry.source === "profile"),
    },
  ].filter((section) => section.entries.length);

  sections.forEach((section) => {
    const sectionEl = document.createElement("div");
    sectionEl.className = "setting-search-section";
    sectionEl.innerHTML = `
      <div class="setting-search-section__title">
        <span>${escapeHtml(section.title)}</span>
        <strong>${section.entries.length}</strong>
      </div>
    `;
    settingSearchResults.appendChild(sectionEl);

    section.entries.forEach((entry) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "setting-search-result";
      const metaLabel = entry.source === "profile"
        ? `${entry.profileName} / ${entry.groupLabel}`
        : entry.groupLabel;
      button.innerHTML = `
        <div class="setting-search-result__group">${highlightSettingSearchText(metaLabel, trimmed)}</div>
        <div class="setting-search-result__title">${highlightSettingSearchText(entry.title || entry.name, trimmed)}</div>
        ${entry.name && entry.name !== entry.title ? `<div class="setting-search-result__name">${highlightSettingSearchText(entry.name, trimmed)}</div>` : ""}
        ${entry.descr ? `<div class="setting-search-result__descr">${highlightSettingSearchText(entry.descr, trimmed)}</div>` : ""}
      `;
      button.onclick = async () => {
        try {
          pendingSettingFocus = { group: entry.group, name: entry.name };
          if (entry.source === "profile" && entry.profileId && entry.originalGroup) {
            settingProfileSectionExpandedState.set(`${entry.profileId}:${entry.originalGroup}`, true);
          }
          closeSettingSearchPanel({ syncHistory: false });
          if (CURRENT_GROUP === entry.group && screenItems && screenItems.style.display !== "none") {
            focusSettingItem(entry.name);
            return;
          }
          await activateSettingGroup(entry.group, true);
        } catch (e) {
          showAppToast(e.message || "Search jump failed", { tone: "error" });
        }
      };
      sectionEl.appendChild(button);
    });
  });
}

async function openSettingSearchPanel(options = {}) {
  const pushHistory = options.pushHistory !== false;
  const scope = options.scope || { type: "all", profileId: "" };
  if (CURRENT_PAGE !== "setting") return;
  closeSettingFabMenu();
  closeSettingProfileActionMenus();
  if (!SETTINGS) {
    try {
      await loadSettings();
    } catch (_) {
      // no-op
    }
  }
  await loadSettingProfiles();
  settingSearchScope = {
    type: scope.type === "profile" && scope.profileId ? "profile" : "all",
    profileId: scope.type === "profile" && scope.profileId ? String(scope.profileId) : "",
  };
  rebuildSettingSearchEntries();
  if (!settingSearchPanel) return;
  mountSettingSearchOverlay();
  settingSearchPanel.hidden = false;
  settingSearchPanel.setAttribute("aria-hidden", "false");
  if (settingSearchBackdrop) settingSearchBackdrop.hidden = false;
  syncSettingSearchFabState();
  const state = history.state || {};
  if (pushHistory && !(state.page === "setting" && state.search)) {
    history.pushState({
      page: "setting",
      screen: (screenItems && screenItems.style.display !== "none") ? "items" : "groups",
      group: CURRENT_GROUP || null,
      search: true,
      searchScope: settingSearchScope.type,
      profileId: settingSearchScope.profileId || null,
    }, "");
  }
  syncModalBodyLock();
  if (settingSearchInput) {
    settingSearchInput.placeholder = settingSearchScope.type === "profile"
      ? getUIText("setting_profile_search_placeholder", "Search in this profile")
      : getUIText("setting_search_placeholder", "Search name, description, group");
    settingSearchInput.setAttribute("aria-label", getSettingSearchScopeLabel());
  }
  renderSettingSearchResults(settingSearchInput?.value || "");
  requestAnimationFrame(() => {
    settingSearchInput?.focus({ preventScroll: true });
    settingSearchInput?.select();
  });
}

function toggleSettingSearchPanel() {
  if (!settingSearchPanel) return;
  if (settingSearchPanel.hidden) {
    openSettingSearchPanel().catch(() => {});
  }
  else closeSettingSearchPanel({ syncHistory: true });
}

if (btnSettingSearch) {
  btnSettingSearch.onclick = () => toggleSettingFabMenu();
}

if (btnSettingFabSearch) {
  btnSettingFabSearch.onclick = () => {
    closeSettingFabMenu();
    openSettingSearchPanel().catch(() => {});
  };
}

if (btnSettingFabProfileAdd) {
  btnSettingFabProfileAdd.onclick = () => {
    createSettingProfileFromCurrent().catch(() => {});
  };
}

if (btnSettingFabResetDefaults) {
  btnSettingFabResetDefaults.onclick = async () => {
    closeSettingFabMenu();
    const ok = await appConfirm(getUIText(
      "setting_reset_defaults_confirm",
      "Reset all settings to defaults?"
    ), {
      title: getUIText("setting_reset_defaults", "Reset Settings"),
      confirmLabel: getUIText("ok", "OK"),
    });
    if (!ok) return;

    btnSettingFabResetDefaults.disabled = true;
    try {
      const payload = await postJson("/api/set_default", {});
      if (!payload?.ok) {
        throw new Error(payload?.error || getUIText("setting_reset_defaults_failed", "Settings reset failed"));
      }
      if (payload.values && typeof payload.values === "object") {
        window.dispatchEvent(new CustomEvent("carrot:paramsrestored", {
          detail: { source: "setting_reset_defaults", values: payload.values },
        }));
        Object.entries(payload.values).forEach(([name, value]) => {
          window.dispatchEvent(new CustomEvent("carrot:paramchange", {
            detail: { name, value, source: "setting_reset_defaults" },
          }));
        });
      }
      showAppToast(getUIText(
        "setting_reset_defaults_done",
        payload.message || "Settings reset complete"
      ));
    } catch (e) {
      showAppToast(getUIText("setting_reset_defaults_failed", "Settings reset failed"), { tone: "error" });
    } finally {
      btnSettingFabResetDefaults.disabled = false;
    }
  };
}

if (settingSearchBackdrop) {
  settingSearchBackdrop.onclick = () => closeSettingSearchPanel({ syncHistory: true });
}

if (settingSearchForm) {
  settingSearchForm.addEventListener("submit", (e) => {
    e.preventDefault();
    const firstResult = settingSearchResults?.querySelector("button.setting-search-result");
    if (firstResult) firstResult.click();
  });
}

if (settingSearchInput) {
  settingSearchInput.addEventListener("input", () => {
    if (settingSearchDebounceTimer) clearTimeout(settingSearchDebounceTimer);
    settingSearchDebounceTimer = window.setTimeout(() => {
      settingSearchDebounceTimer = null;
      renderSettingSearchResults(settingSearchInput.value);
    }, 70);
  });
}

window.addEventListener("keydown", (e) => {
  if (e.key === "Escape" && settingSearchPanel && !settingSearchPanel.hidden) {
    closeSettingSearchPanel({ syncHistory: true });
    return;
  }
  if (e.key === "Escape") {
    closeSettingProfileActionMenus();
  }
  if (e.key === "Escape" && settingFabMenuOpen) {
    closeSettingFabMenu();
  }
});

document.addEventListener("pointerdown", (e) => {
  if (!(e.target instanceof Element && e.target.closest(".setting-profile-action-menu"))) {
    closeSettingProfileActionMenus();
  }
  if (!settingFabMenuOpen || !settingFabMenu) return;
  if (settingFabMenu.contains(e.target)) return;
  closeSettingFabMenu();
});

window.addEventListener("carrot:pagechange", (event) => {
  if (event?.detail?.page !== "setting") {
    closeSettingFabMenu();
    closeSettingProfileActionMenus();
  }
});

function updateSettingSubnavLayoutState() {
  if (!settingSubnav || !settingSubnavWrap) {
    syncSettingSubnavFixedOffset();
    return;
  }

  const maxScrollLeft = Math.max(settingSubnav.scrollWidth - settingSubnav.clientWidth, 0);
  const isScrollable = maxScrollLeft > 4;
  settingSubnavWrap.classList.toggle("is-scrollable", isScrollable);
  syncSettingSubnavFixedOffset();
}

function getSettingSubnavGroups() {
  return getSettingGroupsForDisplay().filter((entry) =>
    !isSettingProfilesDivider(entry) &&
    !isSettingFavoritesGroup(entry.group) &&
    !isSettingProfileGroup(entry.group)
  );
}

function getSettingSubnavGroupIndex(group = CURRENT_GROUP) {
  const groups = getSettingSubnavGroups();
  return groups.findIndex((entry) => entry.group === group);
}

function getSettingSubnavShiftTarget(direction) {
  const groups = getSettingSubnavGroups();
  if (!groups.length) return null;

  const currentIndex = Math.max(0, getSettingSubnavGroupIndex());
  const delta = direction === "forward" ? SETTING_SUBNAV_PAGE_STEP : -SETTING_SUBNAV_PAGE_STEP;
  const nextIndex = Math.max(0, Math.min(currentIndex + delta, groups.length - 1));

  return {
    currentIndex,
    nextIndex,
    group: groups[nextIndex]?.group || null,
    reachedEdge: nextIndex === currentIndex,
  };
}

function stripIdsFromClone(root) {
  if (!root) return;
  if (root.id) root.removeAttribute("id");
  root.querySelectorAll("[id]").forEach((node) => node.removeAttribute("id"));
}

async function activateSettingGroup(group, pushHistory = true, options = {}) {
  if (!isCarrotSettingTabActive()) return;
  const nextGroup = group || CURRENT_GROUP;
  const previousGroup = CURRENT_GROUP;
  const scrollMode = options.scrollMode || "top";
  const animateItems = options.animateItems !== false;
  const animateGroups = options.animateGroups !== false;
  const canReuseRenderedGroup =
    options.forceRender !== true &&
    previousGroup === nextGroup &&
    hasRenderedSettingItems(nextGroup);

  if (previousGroup && previousGroup !== nextGroup) {
    saveCurrentSettingScrollPosition(previousGroup);
  }

  CURRENT_GROUP = group;
  renderGroups({ animateGroups });
  if (isCompactLandscapeMode() && CURRENT_PAGE === "setting") {
    showSettingScreen("items", false);
    history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
    syncSettingGroupChrome(group);
    if (typeof centerActiveSettingSubnavTab === "function") centerActiveSettingSubnavTab("auto");
    if (canReuseRenderedGroup) {
      requestAnimationFrame(() => {
        if (scrollMode === "restore") {
          setSettingItemsScrollTop(
            Number.isFinite(options.scrollTop) ? options.scrollTop : getSavedSettingScrollPosition(group),
          );
        } else {
          resetSettingItemsViewport();
        }
      });
      return;
    }
    await renderItems(group, {
      scrollMode,
      scrollTop: options.scrollTop,
      animateItems,
    });
    return;
  }

  showSettingScreen("items", pushHistory);
  if (!pushHistory) {
    history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
  }
  syncSettingGroupChrome(group);
  if (typeof centerActiveSettingSubnavTab === "function") centerActiveSettingSubnavTab("auto");
  if (canReuseRenderedGroup) {
    requestAnimationFrame(() => {
      if (scrollMode === "restore") {
        setSettingItemsScrollTop(
          Number.isFinite(options.scrollTop) ? options.scrollTop : getSavedSettingScrollPosition(group),
        );
      } else {
        resetSettingItemsViewport();
      }
    });
    return;
  }
  await renderItems(group, {
    scrollMode,
    scrollTop: options.scrollTop,
    animateItems,
  });
}

async function animateSettingGroupSwitch(group, direction = "forward") {
  if (!group || group === CURRENT_GROUP) {
    centerActiveSettingSubnavTab("smooth");
    return;
  }

  if (settingGroupTransitionLock || !settingScreenHost || !screenItems || screenItems.style.display === "none") {
    await activateSettingGroup(group, false);
    return;
  }

  settingGroupTransitionLock = true;
  if (typeof stopSettingSubnavMotion === "function") stopSettingSubnavMotion();

  const snapshot = screenItems.cloneNode(true);
  stripIdsFromClone(snapshot);
  snapshot.setAttribute("aria-hidden", "true");
  snapshot.style.pointerEvents = "none";

  try {
    settingScreenHost.appendChild(snapshot);
    prepareSwipeFrame(settingScreenHost, snapshot);
    screenItems.style.visibility = "hidden";
    await activateSettingGroup(group, false);
    screenItems.style.visibility = "";
    const frame = prepareSwipeFrame(settingScreenHost, snapshot, screenItems);
    if (!frame) {
      snapshot.remove();
      settingGroupTransitionLock = false;
      return;
    }

    applySwipeDrag(frame, 0, direction);
    settleSwipe(frame, direction, true, () => {
      clearPageTransitionClasses(screenItems);
      resetPageRuntimeStyles(screenItems);
      if (snapshot.parentElement) snapshot.remove();
      settingScreenHost.style.minHeight = "";
      settingGroupTransitionLock = false;
    });
  } catch (e) {
    screenItems.style.visibility = "";
    if (snapshot.parentElement) snapshot.remove();
    settingScreenHost.style.minHeight = "";
    settingGroupTransitionLock = false;
    throw e;
  }
}

function getCenteredSettingSubnavGroup() {
  if (!settingSubnav) return null;
  const tabs = Array.from(settingSubnav.querySelectorAll(".setting-subnav__tab"));
  if (!tabs.length) return null;

  const viewport = settingSubnav.getBoundingClientRect();
  const centerX = viewport.left + (viewport.width / 2);
  let bestGroup = null;
  let bestDistance = Number.POSITIVE_INFINITY;

  tabs.forEach((tab) => {
    const rect = tab.getBoundingClientRect();
    const tabCenter = rect.left + (rect.width / 2);
    const distance = Math.abs(tabCenter - centerX);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestGroup = tab.dataset.group || null;
    }
  });

  return bestGroup;
}

function centerActiveSettingSubnavTab(behavior = "smooth") {
  if (!settingSubnav) return;
  const activeTab = settingSubnav.querySelector(".setting-subnav__tab.is-active");
  if (activeTab) {
    const maxScrollLeft = Math.max(settingSubnav.scrollWidth - settingSubnav.clientWidth, 0);
    const targetLeft = activeTab.offsetLeft - ((settingSubnav.clientWidth - activeTab.offsetWidth) / 2);
    const nextLeft = Math.max(0, Math.min(targetLeft, maxScrollLeft));
    settingSubnavProgrammaticScroll = true;
    settingSubnav.scrollTo({ left: nextLeft, behavior });
    window.setTimeout(() => {
      settingSubnavProgrammaticScroll = false;
      updateSettingSubnavLayoutState();
    }, behavior === "smooth" ? 260 : 80);
  }
  updateSettingSubnavLayoutState();
}

function scheduleSettingSubnavFocus() {
  if (settingSubnavFocusTimer) clearTimeout(settingSubnavFocusTimer);

  requestAnimationFrame(() => centerActiveSettingSubnavTab("auto"));
  settingSubnavFocusTimer = window.setTimeout(() => {
    centerActiveSettingSubnavTab("auto");
    settingSubnavFocusTimer = window.setTimeout(() => {
      centerActiveSettingSubnavTab("auto");
      settingSubnavFocusTimer = null;
    }, 180);
  }, 60);
}

function stopSettingSubnavMotion() {
  if (settingSubnavSettleTimer) {
    clearTimeout(settingSubnavSettleTimer);
    settingSubnavSettleTimer = null;
  }
  if (settingSubnavFocusTimer) {
    clearTimeout(settingSubnavFocusTimer);
    settingSubnavFocusTimer = null;
  }
  if (!settingSubnav) return;

  settingSubnavProgrammaticScroll = false;
  settingSubnav.scrollTo({ left: settingSubnav.scrollLeft, behavior: "auto" });
  updateSettingSubnavLayoutState();
}

function renderSettingSubnav() {
  if (!settingSubnav) return;

  const groups = getSettingSubnavGroups();
  const signature = groups.map((entry) => `${entry.group}:${entry.count ?? ""}`).join("|");

  if (settingSubnav.dataset.groupsSignature === signature && settingSubnav.children.length === groups.length) {
    Array.from(settingSubnav.children).forEach((button, index) => {
      const entry = groups[index];
      button.className = "setting-subnav__tab";
      if (isSettingFavoritesGroup(entry.group)) button.classList.add("setting-subnav__tab--favorites");
      if (isSettingProfileGroup(entry.group)) button.classList.add("setting-subnav__tab--profile");
      if (entry.group === CURRENT_GROUP) button.classList.add("is-active");
      button.dataset.group = entry.group;
      button.textContent = getSettingGroupLabel(entry.group);
      button.onclick = () => selectGroup(entry.group, screenItems?.style.display === "none");
    });
    scheduleSettingSubnavFocus();
    requestAnimationFrame(syncSettingSubnavFixedOffset);
    return;
  }

  settingSubnav.innerHTML = "";
  settingSubnav.dataset.groupsSignature = signature;

  groups.forEach((entry) => {
    const button = document.createElement("button");
    button.className = "setting-subnav__tab";
    if (isSettingFavoritesGroup(entry.group)) button.classList.add("setting-subnav__tab--favorites");
    if (isSettingProfileGroup(entry.group)) button.classList.add("setting-subnav__tab--profile");
    if (entry.group === CURRENT_GROUP) button.classList.add("is-active");
    button.dataset.group = entry.group;
    button.textContent = getSettingGroupLabel(entry.group);
    button.type = "button";
    button.onclick = () => selectGroup(entry.group, screenItems?.style.display === "none");
    settingSubnav.appendChild(button);
  });

  scheduleSettingSubnavFocus();
  requestAnimationFrame(syncSettingSubnavFixedOffset);
}

if (settingSubnav) {
  settingSubnav.addEventListener("scroll", () => {
    updateSettingSubnavLayoutState();
    if (settingSubnavProgrammaticScroll) return;

    if (settingSubnavSettleTimer) clearTimeout(settingSubnavSettleTimer);
    settingSubnavSettleTimer = window.setTimeout(() => {
      settingSubnavSettleTimer = null;
      const centeredGroup = getCenteredSettingSubnavGroup();
      if (!centeredGroup) return;
      if (centeredGroup !== CURRENT_GROUP) {
        selectGroup(centeredGroup, false);
        return;
      }
      centerActiveSettingSubnavTab("smooth");
    }, 120);
  }, { passive: true });
  window.addEventListener("resize", () => requestAnimationFrame(updateSettingSubnavLayoutState));
  window.addEventListener("orientationchange", () => {
    window.setTimeout(syncSettingSubnavFixedOffset, 80);
  }, { passive: true });
}

if (settingSubnavWrap) {
  if (window.ResizeObserver) {
    const settingSubnavResizeObserver = new ResizeObserver(() => syncSettingSubnavFixedOffset());
    settingSubnavResizeObserver.observe(settingSubnavWrap);
  }

  let gesture = null;

  settingSubnavWrap.addEventListener("touchstart", (e) => {
    if (CURRENT_PAGE === "setting") {
      gesture = null;
      return;
    }
    if (
      e.touches.length !== 1 ||
      CURRENT_PAGE !== "setting" ||
      !screenItems ||
      screenItems.style.display === "none"
    ) {
      gesture = null;
      return;
    }

    const touch = e.touches[0];
    gesture = {
      dragging: false,
      startX: touch.clientX,
      startY: touch.clientY,
      dx: 0,
      velocity: 0,
      lastX: touch.clientX,
      lastTime: performance.now(),
    };
  }, { passive: true });

  settingSubnavWrap.addEventListener("touchmove", (e) => {
    if (CURRENT_PAGE === "setting") {
      gesture = null;
      return;
    }
    if (!gesture || e.touches.length !== 1) return;

    const touch = e.touches[0];
    const dx = touch.clientX - gesture.startX;
    const dy = touch.clientY - gesture.startY;

    if (!gesture.dragging) {
      if (Math.abs(dx) < 10 && Math.abs(dy) < 10) return;
      if (Math.abs(dy) > Math.abs(dx) * 0.9) {
        gesture = null;
        return;
      }
      gesture.dragging = true;
    }

    e.preventDefault();

    const now = performance.now();
    const dt = Math.max(now - gesture.lastTime, 1);
    gesture.velocity = (touch.clientX - gesture.lastX) / dt;
    gesture.lastX = touch.clientX;
    gesture.lastTime = now;
    gesture.dx = dx;
  }, { passive: false });

  settingSubnavWrap.addEventListener("touchend", () => {
    if (CURRENT_PAGE === "setting") {
      gesture = null;
      return;
    }
    if (!gesture) return;
    if (!gesture.dragging) {
      gesture = null;
      return;
    }

    const dx = gesture.dx;
    const direction = dx < 0 ? "forward" : "backward";
    const velocityOk =
      (direction === "forward" && gesture.velocity < -SWIPE_VELOCITY_THRESHOLD) ||
      (direction === "backward" && gesture.velocity > SWIPE_VELOCITY_THRESHOLD);
    const shouldShift = Math.abs(dx) > 48 || velocityOk;
    const shiftTarget = shouldShift ? getSettingSubnavShiftTarget(direction) : null;

    gesture = null;

    if (!shouldShift || !shiftTarget) {
      centerActiveSettingSubnavTab("smooth");
      return;
    }

    if (typeof stopSettingSubnavMotion === "function") stopSettingSubnavMotion();

    if (direction === "backward" && shiftTarget.reachedEdge) {
      history.back();
      return;
    }

    if (direction === "forward" && shiftTarget.reachedEdge) {
      showPage("tools", true, getSwipeTransition(CURRENT_PAGE, "tools"));
      return;
    }

    if (shiftTarget.group && shiftTarget.group !== CURRENT_GROUP) {
      animateSettingGroupSwitch(shiftTarget.group, direction).catch((e) => console.log("[SettingSubnav] switch failed:", e));
      return;
    }

    centerActiveSettingSubnavTab("smooth");
  }, { passive: true });

  settingSubnavWrap.addEventListener("touchcancel", () => {
    gesture = null;
  }, { passive: true });
}

function selectGroup(group, pushHistory = true) {
  const shouldPush = pushHistory && !(isCompactLandscapeMode() && CURRENT_PAGE === "setting");
  const options = (isCompactLandscapeMode() && CURRENT_PAGE === "setting")
    ? { animateGroups: false }
    : {};
  activateSettingGroup(group, shouldPush, options).catch((e) => console.log("[Setting] selectGroup failed:", e));
}

async function renderItems(group, options = {}) {
  if (!isCarrotSettingTabActive()) return;
  const meta = document.getElementById("groupMeta");
  const itemsBox = document.getElementById("items");
  const renderToken = ++settingRenderToken;
  const scrollMode = options.scrollMode || "top";
  const animateItems = options.animateItems !== false;
  const requestedScrollTop = Number.isFinite(options.scrollTop) ? options.scrollTop : null;
  itemsBox.innerHTML = "";
  delete itemsBox.dataset.renderedGroup;
  renderSettingSubnav();

  const entries = getSettingItemEntriesForGroup(group);
  const list = entries.map((entry) => entry.item);
  const profile = getSettingProfileByGroup(group);
  if (screenItems) screenItems.classList.toggle("setting-screen-items--profile", Boolean(profile));
  if (meta) meta.textContent = `${group} / ${list.length}`;
  const groupLabel = getSettingGroupLabel(group);
  settingTitle.textContent = (UI_STRINGS[LANG].setting || "Setting") + " - " + groupLabel;
  if (itemsTitle) itemsTitle.textContent = groupLabel;

  let values = {};
  try {
    values = await fetchSettingGroupValues(group, {
      force: options.forceValues === true,
      ttlMs: Number.isFinite(options.ttlMs) ? options.ttlMs : SETTING_VALUES_TTL_MS,
    });
  } catch (e) {
    values = {};
  }

  if (renderToken !== settingRenderToken || CURRENT_GROUP !== group || !isCarrotSettingTabActive() || screenItems?.style.display === "none") {
    return;
  }

  if (!list.length && isSettingFavoritesGroup(group)) {
    const empty = document.createElement("div");
    empty.className = "setting-favorites-empty";
    const emptyTitle = document.createElement("div");
    emptyTitle.className = "setting-favorites-empty__title";
    emptyTitle.textContent = getUIText("setting_favorites_empty_title", "No favorites");
    const emptyDesc = document.createElement("div");
    emptyDesc.className = "setting-favorites-empty__desc";
    emptyDesc.textContent = getUIText(
      "setting_favorites_empty_desc",
      "Long press a setting to add it. Long press again to remove it.",
    );
    empty.appendChild(emptyTitle);
    empty.appendChild(emptyDesc);
    itemsBox.appendChild(empty);
    itemsBox.dataset.renderedGroup = group;
    requestAnimationFrame(resetSettingItemsViewport);
    return;
  }

  if (profile) appendSettingProfileHeader(profile, itemsBox);

  const profileSectionCounts = new Map();
  if (profile) {
    entries.forEach((entry) => {
      profileSectionCounts.set(entry.group, (profileSectionCounts.get(entry.group) || 0) + 1);
    });
  }
  let lastProfileGroup = "";
  let currentProfileSectionBody = null;
  list.forEach((p, index) => {
    const name = p.name;
    const originGroup = entries[index]?.group || group;
    if (!(name in UNIT_INDEX)) UNIT_INDEX[name] = 0;

    if (profile && originGroup !== lastProfileGroup) {
      lastProfileGroup = originGroup;
      const section = document.createElement("div");
      section.className = animateItems ? "setting-profile-section ui-stagger-item" : "setting-profile-section";
      if (animateItems) section.style.setProperty("--i", String(Math.min(index + 1, 14)));
      const stateKey = `${profile.id}:${originGroup}`;
      const expanded = settingProfileSectionExpandedState.has(stateKey)
        ? settingProfileSectionExpandedState.get(stateKey)
        : true;
      const sectionLabel = getSettingGroupLabel(originGroup);
      const sectionCount = profileSectionCounts.get(originGroup) || 0;
      section.classList.toggle("is-collapsed", !expanded);

      const header = document.createElement("button");
      header.type = "button";
      header.className = "setting-profile-section__header";
      header.setAttribute("aria-expanded", expanded ? "true" : "false");
      header.innerHTML = `
        <span class="setting-profile-section__label">${settingsDiffEscape(sectionLabel)}</span>
        <span class="setting-profile-section__count">${settingsDiffEscape(sectionCount)}</span>
        <svg class="setting-profile-section__chevron" viewBox="0 0 24 24" aria-hidden="true" focusable="false">
          <path d="m6 9 6 6 6-6"></path>
        </svg>
      `;
      const body = document.createElement("div");
      body.className = "setting-profile-section__body";
      const bodyInner = document.createElement("div");
      bodyInner.className = "setting-profile-section__bodyInner";
      header.onclick = () => {
        const wasCollapsed = section.classList.contains("is-collapsed");
        const nextExpanded = wasCollapsed;
        section.classList.remove("is-expanding", "is-collapsing");
        if (section.__settingProfileMotionTimer) {
          window.clearTimeout(section.__settingProfileMotionTimer);
        }
        void section.offsetWidth;
        section.classList.toggle("is-collapsed", !nextExpanded);
        section.classList.add(nextExpanded ? "is-expanding" : "is-collapsing");
        settingProfileSectionExpandedState.set(stateKey, nextExpanded);
        header.setAttribute("aria-expanded", nextExpanded ? "true" : "false");
        section.__settingProfileMotionTimer = window.setTimeout(() => {
          section.classList.remove("is-expanding", "is-collapsing");
          section.__settingProfileMotionTimer = null;
        }, 280);
      };
      body.appendChild(bodyInner);
      section.appendChild(header);
      section.appendChild(body);
      itemsBox.appendChild(section);
      currentProfileSectionBody = bodyInner;
    }

    const title = formatItemText(p, "title", "etitle", "");
    const descr = formatItemText(p, "descr", "edescr", "");

    const el = document.createElement("div");
    el.className = animateItems ? "setting ui-stagger-item" : "setting";
    if (animateItems) el.style.setProperty("--i", String(index));
    el.dataset.settingName = name;
    el.dataset.settingGroup = originGroup;
    el.classList.toggle("is-favorite", isSettingFavorite(name));

    const top = document.createElement("div");
    top.className = "settingTop";

    const left = document.createElement("div");
    left.className = "setting-copy";
    left.innerHTML = `
      <div class="setting-title-row">
        ${settingMarqueeHtml(title, "title")}
        ${renderSettingFavoriteMark(name)}
      </div>
      ${settingMarqueeHtml(name, "name")}
      <div class="muted mt-sm">
        min=${p.min}, max=${p.max}, default=${p.default}
      </div>
    `;

    const ctrl = document.createElement("div");
    ctrl.className = "ctrl";

    const btnMinus = document.createElement("button");
    btnMinus.type = "button";
    btnMinus.className = "smallBtn";
    btnMinus.textContent = "-";

    const val = document.createElement("button");
    val.type = "button";
    val.className = "pill val";
    val.setAttribute("aria-label", getUIText("setting_value_edit", "Edit value"));

    const btnPlus = document.createElement("button");
    btnPlus.type = "button";
    btnPlus.className = "smallBtn";
    btnPlus.textContent = "+";

    const unitBtn = document.createElement("button");
    unitBtn.type = "button";
    unitBtn.className = "smallBtn";
    unitBtn.textContent = "x" + UNIT_CYCLE[UNIT_INDEX[name]];

    unitBtn.onclick = () => {
      UNIT_INDEX[name] = (UNIT_INDEX[name] + 1) % UNIT_CYCLE.length;
      unitBtn.textContent = "x" + UNIT_CYCLE[UNIT_INDEX[name]];
    };

    ctrl.appendChild(btnMinus);
    ctrl.appendChild(val);
    ctrl.appendChild(btnPlus);
    ctrl.appendChild(unitBtn);

    top.appendChild(left);
    top.appendChild(ctrl);

    const d = document.createElement("div");
    d.className = "descr";
    d.textContent = descr;

    el.appendChild(top);
    el.appendChild(d);
    (currentProfileSectionBody || itemsBox).appendChild(el);

    const cur = (name in values) ? values[name] : p.default;
    val.textContent = String(cur);

    function normalizeSettingValue(raw) {
      const text = String(raw).trim();
      if (!text) return null;

      const num = Number(text);
      if (!Number.isFinite(num)) return null;

      const min = Number(p.min);
      const max = Number(p.max);
      let next = clamp(num, min, max);
      if (Number.isInteger(min) && Number.isInteger(max)) {
        next = Math.round(next);
      }
      return next;
    }

    async function commitSettingValue(next) {
      try {
        if (profile) {
          const nextValues = { ...(profile.values || {}), [name]: next };
          const nextProfile = await saveSettingProfile(profile.id, { values: nextValues });
          if (nextProfile) {
            profile.values = { ...(nextProfile.values || nextValues) };
          } else {
            profile.values = nextValues;
          }
        } else {
          await setParam(name, next);
        }
        val.textContent = String(next);
        if (!profile) {
          cacheSettingValue(name, next, group);
          if (originGroup !== group) cacheSettingValue(name, next, originGroup);
        }
      } catch (e) {
        showAppToast((UI_STRINGS[LANG].set_failed || "set failed: ") + e.message, { tone: "error" });
      }
    }

    async function editValueDirect() {
      const input = await appPrompt(
        getUIText("setting_value_prompt", "Enter value for {name}\nRange: {min} - {max}", {
          name,
          min: p.min,
          max: p.max,
        }),
        {
          title: getUIText("setting_value_title", "Edit value"),
          defaultValue: val.textContent,
          placeholder: String(p.default),
          confirmLabel: getUIText("ok", "OK"),
          showCancel: false,
          defaultActionLabel: getUIText("default_value", "Default"),
          defaultActionValue: { settingDefaultAction: true, value: String(p.default) },
        }
      );
      if (input === null) return;

      if (input?.settingDefaultAction) {
        const defaultValue = input.value;
        const ok = await appConfirm(getUIText(
          "default_value_confirm",
          "Restore {name} to default value ({value})?",
          { name, value: defaultValue }
        ), {
          title: getUIText("default_value", "Default"),
          confirmLabel: getUIText("ok", "OK"),
        });
        if (!ok) return;

        const nextDefault = normalizeSettingValue(defaultValue);
        if (nextDefault === null) {
          showAppToast(getUIText("setting_value_invalid", "Enter a valid number."), { tone: "error" });
          return;
        }
        if (String(nextDefault) === String(val.textContent)) return;
        await commitSettingValue(nextDefault);
        return;
      }

      const next = normalizeSettingValue(input);
      if (next === null) {
        showAppToast(getUIText("setting_value_invalid", "Enter a valid number."), { tone: "error" });
        return;
      }
      if (String(next) === String(val.textContent)) return;
      await commitSettingValue(next);
    }

    async function applyDelta(sign) {
      const step = UNIT_CYCLE[UNIT_INDEX[name]];
      let curv = Number(val.textContent);
      if (Number.isNaN(curv)) curv = Number(p.default);

      let next = curv + sign * step;
      next = clamp(next, Number(p.min), Number(p.max));

      if (Number.isInteger(p.min) && Number.isInteger(p.max) && Number.isInteger(step)) {
        next = Math.round(next);
      }

      await commitSettingValue(next);
    }

    btnMinus.onclick = () => applyDelta(-1);
    val.onclick = editValueDirect;
    btnPlus.onclick = () => applyDelta(+1);
  });

  itemsBox.dataset.renderedGroup = group;
  scheduleSettingOverflowSync(itemsBox);

  if (pendingSettingFocus?.group === group) {
    requestAnimationFrame(() => focusSettingItem(pendingSettingFocus.name));
    return;
  }

  requestAnimationFrame(() => {
    if (scrollMode === "restore") {
      setSettingItemsScrollTop(requestedScrollTop ?? getSavedSettingScrollPosition(group));
      return;
    }
    resetSettingItemsViewport();
  });
}

function bindSettingFavoriteLongPress() {
  const itemsBox = document.getElementById("items");
  if (!itemsBox || itemsBox.dataset.favoriteLongPressBound === "1") return;
  itemsBox.dataset.favoriteLongPressBound = "1";

  let press = null;

  function clearPress() {
    if (!press) return;
    if (press.timer) clearTimeout(press.timer);
    press.row?.classList.remove("is-longpressing");
    press = null;
  }

  function isIgnoredFavoritePressTarget(target) {
    return Boolean(target?.closest?.(".ctrl, button, input, select, textarea, a"));
  }

  itemsBox.addEventListener("pointerdown", (event) => {
    if (event.button !== undefined && event.button !== 0) return;
    const row = event.target.closest(".setting[data-setting-name]");
    if (!row || !itemsBox.contains(row) || isIgnoredFavoritePressTarget(event.target)) return;

    clearPress();
    const startX = event.clientX;
    const startY = event.clientY;
    press = {
      pointerId: event.pointerId,
      row,
      startX,
      startY,
      fired: false,
      timer: window.setTimeout(() => {
        if (!press || press.row !== row) return;
        press.fired = true;
        row.classList.remove("is-longpressing");
        toggleSettingFavorite(row.dataset.settingName).catch(() => {});
      }, SETTING_FAVORITES_LONG_PRESS_MS),
    };
    row.classList.add("is-longpressing");
  }, { passive: true });

  itemsBox.addEventListener("pointermove", (event) => {
    if (!press || press.pointerId !== event.pointerId) return;
    const dx = Math.abs(event.clientX - press.startX);
    const dy = Math.abs(event.clientY - press.startY);
    if (dx > SETTING_FAVORITES_MOVE_TOLERANCE || dy > SETTING_FAVORITES_MOVE_TOLERANCE) {
      clearPress();
    }
  }, { passive: true });

  itemsBox.addEventListener("pointerup", clearPress, { passive: true });
  itemsBox.addEventListener("pointercancel", clearPress, { passive: true });
  itemsBox.addEventListener("pointerleave", clearPress, { passive: true });
  itemsBox.addEventListener("contextmenu", (event) => {
    if (!event.target.closest(".setting[data-setting-name]")) return;
    event.preventDefault();
  });
}

bindSettingFavoriteLongPress();

async function syncSettingViewportLayout(options = {}) {
  if (CURRENT_PAGE !== "setting" || !SETTINGS) return;
  settingViewportLayoutSignature = getSettingViewportLayoutSignature();
  const animateChrome = options.animateChrome === true;
  const animateItems = options.animateItems === true || animateChrome;
  const splitLandscape = isCompactLandscapeMode();
  if (typeof syncSettingSplitLayoutClass === "function") {
    syncSettingSplitLayoutClass(splitLandscape);
  }
  syncSettingSearchFabState();

  if (typeof getCurrentSettingTab === "function" && getCurrentSettingTab() === "device") {
    if (splitLandscape) {
      showSettingScreen("items", false);
    }
    if (typeof renderDeviceTab === "function") {
      await renderDeviceTab({ animateGroups: animateChrome, animateItems });
    }
    if (!splitLandscape) {
      const deviceItemsEl = document.getElementById("deviceItems");
      const hasDeviceItems = Boolean(deviceItemsEl && deviceItemsEl.children.length > 0);
      const targetScreen = hasDeviceItems ? "items" : "groups";
      showSettingScreen(targetScreen, false);
      settleSettingScreenVisibility(targetScreen);
    }
    return;
  }

  renderGroups({ animateGroups: animateChrome });
  renderSettingSubnav();

  if (splitLandscape) {
    const targetGroup = CURRENT_GROUP || getLandscapeDefaultSettingGroup();
    if (!targetGroup) return;
    CURRENT_GROUP = targetGroup;
    showSettingScreen("items", false);
    syncSettingGroupChrome(targetGroup);
    if (typeof centerActiveSettingSubnavTab === "function") centerActiveSettingSubnavTab("auto");
    if (!hasRenderedSettingItems(targetGroup)) {
      await renderItems(targetGroup, { scrollMode: "restore", animateItems });
    }
    return;
  }

  if (CURRENT_GROUP) {
    syncSettingGroupChrome(CURRENT_GROUP);
    showSettingScreen("items", false);
    if (typeof centerActiveSettingSubnavTab === "function") centerActiveSettingSubnavTab("auto");
    if (!hasRenderedSettingItems(CURRENT_GROUP)) {
      await renderItems(CURRENT_GROUP, { scrollMode: "restore", animateItems });
    }
  } else {
    showSettingScreen("groups", false);
  }
}

function scheduleSettingViewportLayoutSync(force = false) {
  if (CURRENT_PAGE !== "setting" || !SETTINGS) return;
  if (!force && !hasSettingViewportLayoutChanged()) return;
  if (settingViewportSyncTimer) clearTimeout(settingViewportSyncTimer);
  settingViewportSyncTimer = window.setTimeout(() => {
    settingViewportSyncTimer = null;
    syncSettingViewportLayout({ animateChrome: false, animateItems: false }).catch(() => {});
  }, 80);
}

window.addEventListener("carrot:paramsrestored", (event) => {
  const values = event.detail?.values;
  if (!values || typeof values !== "object") return;
  const changedNames = new Set(Object.keys(values));
  Object.entries(values).forEach(([name, value]) => cacheSettingValue(name, value));
  applyRestoredSettingValuesToRenderedItems(values);
  for (const [group, cachedGroup] of settingGroupValueCache.entries()) {
    if (!cachedGroup?.values) continue;
    let touched = false;
    changedNames.forEach((name) => {
      if (name in cachedGroup.values) {
        cachedGroup.values[name] = values[name];
        touched = true;
      }
    });
    if (touched) cachedGroup.loadedAt = Date.now();
  }

  if (!CURRENT_GROUP || !isCarrotSettingTabActive()) return;
  const currentNames = new Set(getSettingGroupParamNames(CURRENT_GROUP));
  const affectsCurrentGroup = [...changedNames].some((name) => currentNames.has(name));
  if (!affectsCurrentGroup) return;
  if (settingRestoreRefreshTimer) clearTimeout(settingRestoreRefreshTimer);
  const currentTop = getSettingItemsScrollTop();
  settingRestoreRefreshTimer = window.setTimeout(() => {
    settingRestoreRefreshTimer = null;
    renderItems(CURRENT_GROUP, {
      forceValues: true,
      scrollMode: "restore",
      scrollTop: currentTop,
      animateItems: false,
    }).catch(() => {});
  }, 60);
});

window.addEventListener("resize", () => {
  scheduleSettingViewportLayoutSync(false);
  scheduleSettingOverflowSync(document, 80);
}, { passive: true });

window.addEventListener("orientationchange", () => {
  scheduleSettingViewportLayoutSync(true);
  scheduleSettingOverflowSync(document, 180);
}, { passive: true });

if (window.visualViewport) {
  window.visualViewport.addEventListener("resize", () => {
    scheduleSettingOverflowSync(document, 80);
  }, { passive: true });
}

initSettingOverflowObservers();

