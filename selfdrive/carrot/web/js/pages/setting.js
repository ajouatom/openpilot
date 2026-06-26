"use strict";

// Setting page — groups, items, value cache, search, screen layout.

let settingsLoadPromise = null;
let settingValueWarmupTimer = null;
let settingValueWarmupPromise = null;
let settingRestoreRefreshTimer = null;
const SETTING_VALUES_TTL_MS = 60000;
const settingValueCache = new Map();
const settingGroupValueCache = new Map();
const settingGroupValuePromises = new Map();
const settingPopularValuesState = {
  loaded: false,
  loadPromise: null,
  carKey: "",
  values: {},
  fetchedAt: 0,
};

const SETTING_FAVORITES_GROUP = "__setting_favorites__";
const SETTING_PROFILES_DIVIDER = "__setting_profiles_divider__";
const SETTING_PROFILE_GROUP_PREFIX = "__setting_profile__:";
const SETTING_CATEGORY_DIVIDER_PREFIX = "__setting_category__:";
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

function isSettingCategoryDivider(entry) {
  return String(entry?.group || "").startsWith(SETTING_CATEGORY_DIVIDER_PREFIX);
}

function isSettingAnyDivider(entry) {
  return isSettingProfilesDivider(entry) || isSettingCategoryDivider(entry);
}

// 대>중>소 노드(카테고리/그룹/섹션)의 현재 언어 라벨. ko/en/zh 직접 보유 노드용.
function settingNodeLabel(node) {
  if (!node) return "";
  if (LANG === "zh") return node.zh || node.en || node.ko || "";
  if (LANG === "ko") return node.ko || node.en || node.zh || "";
  return node.en || node.ko || node.zh || "";
}

// /api/settings 의 categories(대>중>소)가 있으면 groups/items_by_group 를
// 중-group id 키 기준으로 정규화한다. 각 항목엔 소-섹션 라벨을 __section 으로 부착.
// categories 가 없으면(구버전) 아무것도 바꾸지 않아 기존 평면 UI 로 폴백.
function normalizeSettingCategories(j) {
  if (!j || !Array.isArray(j.categories) || !j.categories.length) return;
  const idx = {};
  Object.values(j.items_by_group || {}).forEach((list) => {
    (list || []).forEach((it) => { if (it && it.name) idx[it.name] = it; });
  });
  const flatGroups = [];
  const newItemsByGroup = {};
  j.categories.forEach((cat) => {
    (cat.groups || []).forEach((g) => {
      flatGroups.push({ group: g.id, ko: g.ko, en: g.en, zh: g.zh, count: g.count, category: cat.id });
      const items = [];
      (g.sections || []).forEach((sec) => {
        // 라벨이 없어도(단일 직속 섹션) 카드는 만들도록 항상 객체로 둔다.
        const secLabel = { id: sec.id, ko: sec.ko, en: sec.en, zh: sec.zh };
        (sec.items || []).forEach((name) => {
          const def = idx[name];
          if (def) items.push(Object.assign({}, def, { __section: secLabel }));
        });
      });
      newItemsByGroup[g.id] = items;
    });
  });
  j.groups = flatGroups;
  j.items_by_group = newItemsByGroup;
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
  const out = [
    {
      group: SETTING_FAVORITES_GROUP,
      count: getFavoriteSettingEntries().length,
      virtual: true,
    },
  ];
  const cats = SETTINGS?.categories;
  if (Array.isArray(cats) && cats.length) {
    cats.forEach((cat) => {
      out.push({
        group: SETTING_CATEGORY_DIVIDER_PREFIX + (cat.id || ""),
        label: settingNodeLabel(cat),
        divider: true,
        virtual: true,
      });
      (cat.groups || []).forEach((g) => {
        out.push({ group: g.id, ko: g.ko, en: g.en, zh: g.zh, count: g.count, category: cat.id });
      });
    });
  } else {
    out.push(...(SETTINGS?.groups || []));
  }
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

function normalizeSettingPopularValues(payload) {
  const values = payload?.popular_values;
  return values && typeof values === "object" && !Array.isArray(values) ? values : {};
}

async function loadSettingPopularValues(force = false) {
  if (!force && settingPopularValuesState.loaded) return settingPopularValuesState.values;
  if (!force && settingPopularValuesState.loadPromise) return settingPopularValuesState.loadPromise;

  settingPopularValuesState.loadPromise = getJson("/api/setting_popular_values")
    .then((payload) => {
      settingPopularValuesState.loaded = true;
      settingPopularValuesState.carKey = String(payload?.car_key || "");
      settingPopularValuesState.fetchedAt = Number(payload?.fetched_at || 0);
      settingPopularValuesState.values = normalizeSettingPopularValues(payload);
      return settingPopularValuesState.values;
    })
    .catch(() => {
      settingPopularValuesState.loaded = true;
      settingPopularValuesState.carKey = "";
      settingPopularValuesState.fetchedAt = 0;
      settingPopularValuesState.values = {};
      return settingPopularValuesState.values;
    })
    .finally(() => {
      settingPopularValuesState.loadPromise = null;
    });

  return settingPopularValuesState.loadPromise;
}

function getSettingPopularValue(name) {
  const entry = settingPopularValuesState.values?.[String(name || "")];
  return entry && typeof entry === "object" ? entry : null;
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
    syncSettingControlState(row, values[name]);
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
    await loadSettingPopularValues(true);
    renderGroups({ animateGroups: false });
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

    normalizeSettingCategories(j);
    SETTINGS = j;
    UNIT_CYCLE = j.unit_cycle || UNIT_CYCLE;
    settingValueCache.clear();
    settingGroupValueCache.clear();
    settingGroupValuePromises.clear();
    await loadSettingFavorites(force);
    await loadSettingProfiles(force);
    await loadSettingPopularValues(force);
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
  const signature = groups.map((g) => isSettingAnyDivider(g) ? `div:${g.label || ""}` : `${g.group}:${g.count ?? ""}:${g.label || ""}`).join("|");

  function setGroupButtonLabel(button, label, count) {
    const text = Number.isFinite(Number(count)) ? `${label} (${count})` : label;
    button.title = text;
    button.innerHTML = `<span class="setting-group-label">${escapeHtml(text)}</span>`;
    requestAnimationFrame(() => measureSettingGroupButtonOverflow(button));
  }

  if (!animateGroups && box.dataset.groupsSignature === signature && box.children.length === groups.length) {
    Array.from(box.children).forEach((button, index) => {
      const g = groups[index];
      if (isSettingAnyDivider(g)) {
        button.className = isSettingCategoryDivider(g) ? "setting-profile-divider setting-category-divider" : "setting-profile-divider";
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
    if (isSettingAnyDivider(g)) {
      const divider = document.createElement("div");
      const base = animateGroups ? "setting-profile-divider ui-stagger-item" : "setting-profile-divider";
      divider.className = isSettingCategoryDivider(g) ? base + " setting-category-divider" : base;
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
  if (meta.ko || meta.en || meta.zh) return settingNodeLabel(meta);
  if (LANG === "zh") return meta.cgroup || meta.egroup || meta.group;
  if (LANG === "ko") return meta.group || meta.egroup || group;
  return meta.egroup || meta.group || group;
}

function getSettingItemContextLabel(group, item) {
  const groupLabel = getSettingGroupLabel(group);
  const sectionLabel = item?.__section ? settingNodeLabel(item.__section) : "";
  if (!sectionLabel || sectionLabel === groupLabel) return groupLabel;
  return `${groupLabel} > ${sectionLabel}`;
}

const SETTING_CONTROL_OVERRIDES = {
  ShowPathMode: { kind: "select" },
  ShowPathColor: { kind: "select" },
  ShowPathColorCruiseOff: { kind: "select" },
  ShowPathModeLane: { kind: "select" },
  ShowPathColorLane: { kind: "select" },
  ShowPlotMode: { kind: "select" },
  ClusterHudScreenMode: { kind: "select" },
  ClusterHudRadarInfo: { kind: "select" },
};

function getSettingControlConfig(p) {
  const override = SETTING_CONTROL_OVERRIDES[p?.name] || {};
  const min = Number(p?.min);
  const max = Number(p?.max);
  const unit = Math.max(1, Number(p?.unit) || 1);
  const optionCount = Number.isFinite(min) && Number.isFinite(max) ? Math.floor(max - min + 1) : 0;
  let kind = override.kind || "slider";

  if (!override.kind) {
    if (min === 0 && max === 1) {
      kind = "toggle";
    } else if (Number.isInteger(min) && Number.isInteger(max) && optionCount >= 2 && optionCount <= 4) {
      kind = "segmented";
    } else if (Number.isInteger(min) && Number.isInteger(max) && optionCount > 4 && optionCount <= 8) {
      kind = "select";
    } else {
      kind = "slider";
    }
  }

  return { kind, min, max, unit, optionCount };
}

const SETTING_DISPLAY_UNIT_TYPES = Object.freeze({
  raw: "",
  speedKph: "km/h",
  distanceCm: "cm",
  timeSec: "s",
  timeMin: "min",
  percent: "%",
  degree: "deg",
});

const SETTING_PARAM_DISPLAY_TYPES = Object.freeze({
  PathOffset: "distanceCm",
  CruiseOnDist: "distanceCm",
  CruiseEcoControl: "speedKph",
  StopDistanceCarrot: "distanceCm",
  TrafficStopDistanceAdjust: "distanceCm",
  AutoTurnControlSpeedTurn: "speedKph",
  CruiseSpeedUnit: "speedKph",
  CruiseSpeedUnitBasic: "speedKph",
  CruiseSpeed1: "speedKph",
  CruiseSpeed2: "speedKph",
  CruiseSpeed3: "speedKph",
  CruiseSpeed4: "speedKph",
  CruiseSpeed5: "speedKph",
  AutoGasTokSpeed: "speedKph",
  AutoGasCancelSpeed: "speedKph",
  MaxTimeOffroadMin: "timeMin",
  AutoSpeedUptoRoadSpeedLimit: "percent",
  AutoRoadSpeedAdjust: "percent",
  ApplyModelSpeed: "percent",
  UseLaneLineSpeed: "speedKph",
  UseLaneLineCurveSpeed: "speedKph",
  AdjustLaneOffset: "distanceCm",
  SoundVolumeAdjust: "percent",
  SoundVolumeAdjustEngage: "percent",
  AutoCurveSpeedLowerLimit: "speedKph",
  AutoNaviSpeedCtrlEnd: "timeSec",
  AutoNaviSpeedBumpTime: "timeSec",
  AutoNaviSpeedBumpSpeed: "speedKph",
  AutoRoadSpeedLimitOffset: "speedKph",
  AutoCurveSpeedFactor: "percent",
  MapTurnSpeedFactor: "percent",
  AutoNaviSpeedSafetyFactor: "percent",
  RadarReactionFactor: "percent",
  SteerRatioRate: "percent",
  DynamicTFollowLC: "percent",
  TFollowDecelBoost: "percent",
  ShowCustomBrightness: "percent",
  ClusterHudBrightness: "percent",
});

function getSettingDisplayType(name) {
  const key = String(name || "").trim();
  return SETTING_PARAM_DISPLAY_TYPES[key] || "raw";
}

function getSettingDisplayUnit(name) {
  return SETTING_DISPLAY_UNIT_TYPES[getSettingDisplayType(name)] || "";
}

function formatSettingDisplayValue(p, value) {
  const text = String(value);
  const unit = getSettingDisplayUnit(p?.name);
  return unit ? `${text}${unit}` : text;
}

function formatSettingRangeMeta(p) {
  return [
    `min=${formatSettingDisplayValue(p, p?.min)}`,
    `max=${formatSettingDisplayValue(p, p?.max)}`,
    `default=${formatSettingDisplayValue(p, p?.default)}`,
  ].join(", ");
}

function formatSettingPopularValue(p, raw) {
  if (raw === null || raw === undefined) return "";
  const min = Number(p?.min);
  const max = Number(p?.max);
  if (min === 0 && max === 1) {
    const text = String(raw).trim().toLowerCase();
    if (text === "1" || text === "true" || text === "on") return "ON";
    if (text === "0" || text === "false" || text === "off") return "OFF";
  }
  return formatSettingDisplayValue(p, raw);
}

function normalizeSettingPopularNumericValue(p, raw) {
  if (raw === null || raw === undefined || raw === "") return null;
  const min = Number(p?.min);
  const max = Number(p?.max);
  const text = String(raw).trim().toLowerCase();
  if (min === 0 && max === 1) {
    if (text === "1" || text === "true" || text === "on") return 1;
    if (text === "0" || text === "false" || text === "off") return 0;
  }
  const value = Number(raw);
  return Number.isFinite(value) ? value : null;
}

function isSettingPopularValueInRange(p, raw) {
  const min = Number(p?.min);
  const max = Number(p?.max);
  if (!Number.isFinite(min) || !Number.isFinite(max)) return true;
  const value = normalizeSettingPopularNumericValue(p, raw);
  if (value === null) return false;
  return value >= min && value <= max;
}

function getSettingPopularDisplayEntry(p, entry) {
  if (!entry || typeof entry !== "object") return null;
  const sample = Number(entry?.sample ?? entry?.sample_count ?? 0);
  if (!Number.isFinite(sample) || sample < 1) return null;
  if (!isSettingPopularValueInRange(p, entry?.value)) return null;
  const topValues = Array.isArray(entry?.top_values)
    ? entry.top_values.filter((item) => {
      const count = Number(item?.count ?? 0);
      return Number.isFinite(count) && count > 0 && isSettingPopularValueInRange(p, item?.value);
    }).slice(0, 10)
    : [];
  return { ...entry, top_values: topValues };
}

function getSettingPopularCarKeyLabel() {
  return String(settingPopularValuesState.carKey || "").trim() || getUIText("setting_popular_value_my_model", "내 차종");
}

function renderSettingPopularChipText(p, entry) {
  const sample = Number(entry?.sample ?? entry?.sample_count ?? 0);
  const value = formatSettingPopularValue(p, entry?.value);
  if (!sample || !value) return "";
  return getUIText("setting_popular_value_chip", "{label} ({sample}대) {value}", {
    label: getUIText("setting_popular_value_chip_label", "내 차종 인기값"),
    sample,
    value: `"${value}"`,
  });
}

function renderSettingPopularChipHtml(p, entry) {
  const sample = Number(entry?.sample ?? entry?.sample_count ?? 0);
  const value = formatSettingPopularValue(p, entry?.value);
  if (!sample || !value) return "";
  return `
    <span class="setting-popular-value-chip__car">${escapeHtml(getUIText("setting_popular_value_chip_label", "내 차종 인기값"))}</span>
    <span class="setting-popular-value-chip__label">(</span><span class="setting-popular-value-chip__accent">${escapeHtml(getUIText("setting_popular_value_chip_sample", "{sample}대", { sample }))}</span><span class="setting-popular-value-chip__label">)</span>
    <span class="setting-popular-value-chip__accent">${escapeHtml(`"${value}"`)}</span>
  `;
}

function getSettingPopularDetailTitle() {
  const carKey = String(settingPopularValuesState.carKey || "").trim();
  if (carKey) return getUIText("setting_popular_value_car_title", "{car} 인기값", { car: carKey });
  return getUIText("setting_popular_value_title", "내 차종 인기값");
}

function formatSettingPopularUpdated(epochSec) {
  const sec = Number(epochSec || 0);
  if (!Number.isFinite(sec) || sec <= 0) return "";
  try {
    return new Date(sec * 1000).toLocaleString(LANG === "ko" ? "ko-KR" : undefined, {
      year: "numeric", month: "2-digit", day: "2-digit",
      hour: "2-digit", minute: "2-digit", second: "2-digit",
    });
  } catch {
    return "";
  }
}

function renderSettingPopularDetailHtml(p, entry) {
  const values = Array.isArray(entry?.top_values) ? entry.top_values : [];
  if (!values.length) {
    return `<div class="setting-popular-detail"><div class="setting-popular-detail__empty">${escapeHtml(getUIText("setting_popular_value_empty", "표시할 설정값이 없습니다."))}</div></div>`;
  }

  const counts = values.map((item) => Number(item?.count ?? 0)).filter((count) => Number.isFinite(count) && count > 0);
  const maxCount = Math.max(1, ...counts);

  const rows = values.map((item, index) => {
    const rank = Number(item?.rank) || index + 1;
    const value = formatSettingPopularValue(p, item?.value);
    const count = Number(item?.count ?? 0);
    const width = Math.max(4, Math.min(100, Math.round((Math.max(0, count) / maxCount) * 100)));
    return `
      <button type="button" class="setting-popular-detail__row" style="--setting-popular-width:${width}%" data-setting-popular-value="${escapeHtml(item?.value ?? "")}">
        <span class="setting-popular-detail__rank">${escapeHtml(`${rank}위`)}</span>
        <span class="setting-popular-detail__main">
          <span class="setting-popular-detail__value">${escapeHtml(value)}</span>
          ${values.length > 1 ? `<span class="setting-popular-detail__bar" aria-hidden="true"></span>` : ""}
        </span>
        <span class="setting-popular-detail__count">${escapeHtml(`${count}대`)}</span>
      </button>
    `;
  }).join("");

  const updated = formatSettingPopularUpdated(settingPopularValuesState.fetchedAt);
  const updatedHtml = updated
    ? `<div class="setting-popular-detail__updated" style="margin-top:8px;font-size:11px;color:var(--md-on-surface-var,#8a8f98)">${escapeHtml(getUIText("setting_popular_value_updated", "최근 업데이트: {time}", { time: updated }))}</div>`
    : "";

  return `
    <div class="setting-popular-detail${values.length <= 1 ? " setting-popular-detail--single" : ""}">
      <div class="setting-popular-detail__head">
        <span class="setting-popular-detail__name">${escapeHtml(getSettingPopularDetailTitle())}</span>
        <span class="setting-popular-detail__range">${escapeHtml(getUIText("setting_popular_value_top10", "1~10위"))}</span>
      </div>
      <div class="setting-popular-detail__rows">${rows}</div>
      ${updatedHtml}
    </div>
  `;
}

const SETTING_UNIT_STORAGE_KEY = "carrot.settingUnitIndex.v1";
let settingUnitIndexStore = null;

function getSettingUnitIndexStore() {
  if (settingUnitIndexStore) return settingUnitIndexStore;
  try {
    settingUnitIndexStore = JSON.parse(localStorage.getItem(SETTING_UNIT_STORAGE_KEY) || "{}") || {};
  } catch (_) {
    settingUnitIndexStore = {};
  }
  return settingUnitIndexStore;
}

function saveSettingUnitIndex(name, index) {
  const key = String(name || "").trim();
  if (!key) return;
  try {
    const store = getSettingUnitIndexStore();
    store[key] = index;
    localStorage.setItem(SETTING_UNIT_STORAGE_KEY, JSON.stringify(store));
  } catch (_) {}
}

function getSettingUnitIndex(name) {
  const key = String(name || "").trim();
  if (!key) return 0;
  if (!(key in UNIT_INDEX)) {
    const saved = Number(getSettingUnitIndexStore()[key]);
    UNIT_INDEX[key] = Number.isInteger(saved) && saved >= 0 && saved < UNIT_CYCLE.length ? saved : 0;
  }
  if (!Number.isInteger(UNIT_INDEX[key]) || UNIT_INDEX[key] < 0 || UNIT_INDEX[key] >= UNIT_CYCLE.length) {
    UNIT_INDEX[key] = 0;
  }
  return UNIT_INDEX[key];
}

function getSettingUnitValue(name) {
  return UNIT_CYCLE[getSettingUnitIndex(name)] || UNIT_CYCLE[0] || 1;
}

function setSettingUnitButtonLabel(button, name) {
  if (button) button.textContent = "x" + getSettingUnitValue(name);
}

function cycleSettingUnitValue(name) {
  const key = String(name || "").trim();
  if (!key) return;
  UNIT_INDEX[key] = (getSettingUnitIndex(key) + 1) % UNIT_CYCLE.length;
  saveSettingUnitIndex(key, UNIT_INDEX[key]);
}

function settingChevronSvg(direction = "left") {
  const path = direction === "right" ? "M8.75 4.75 16 12l-7.25 7.25" : "M15.25 4.75 8 12l7.25 7.25";
  return `
    <svg class="setting-icon setting-icon--chevron" viewBox="0 0 24 24" aria-hidden="true" focusable="false">
      <path d="${path}"></path>
    </svg>
  `;
}

function setSettingItemsTitle(label) {
  if (!itemsTitle) return;
  const safeLabel = escapeHtml(label || "");
  itemsTitle.innerHTML = `
    <span class="setting-title-backIcon" aria-hidden="true">${settingChevronSvg("left")}</span>
    <span class="setting-title-text">${safeLabel}</span>
  `;
}

function getSettingOptionValues(config) {
  if (!config || !Number.isInteger(config.min) || !Number.isInteger(config.max)) return [];
  const out = [];
  for (let value = config.min; value <= config.max; value += 1) out.push(value);
  return out;
}

function getSettingOptionLabel(name, value) {
  return formatSettingDisplayValue({ name }, value);
}

function syncSettingControlState(row, value) {
  if (!row) return;
  const text = String(value);
  const valueButton = row.querySelector(".val");
  if (valueButton) {
    const displayText = formatSettingDisplayValue({ name: row.dataset.settingName || "" }, value);
    valueButton.textContent = displayText;
    valueButton.dataset.rawValue = text;
  }

  const toggle = row.querySelector(".setting-switch__input");
  if (toggle) toggle.checked = Number(value) === 1;

  const slider = row.querySelector(".setting-slider__input");
  if (slider) slider.value = text;

  row.querySelectorAll(".setting-segment").forEach((button) => {
    button.classList.toggle("is-active", String(button.dataset.value) === text);
    button.setAttribute("aria-pressed", String(button.dataset.value) === text ? "true" : "false");
  });

  const select = row.querySelector(".setting-select");
  if (select) {
    select.value = text;
    if (select.tagName === "BUTTON") {
      select.dataset.value = text;
      select.textContent = getSettingOptionLabel(row.dataset.settingName || "", value);
    }
  }
}

let settingGroupTransitionLock = false;
let settingRenderToken = 0;
let pendingSettingFocus = null;
let settingFocusClearTimer = null;
let settingSearchDebounceTimer = null;
let settingSearchEntries = [];
let settingSearchScope = { type: "all", profileId: "" };
const settingPageRoot = document.getElementById("pageSetting");
let settingFabMenuOpen = false;
let CURRENT_SETTING_DETAIL = null;

function isCompactLandscapeMode() {
  return window.matchMedia("(orientation: landscape)").matches;
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
    showSettingScreen("groups", false);
    showAppToast(getUIText("setting_profile_deleted", "Profile deleted"));
  } catch (e) {
    showAppToast(e?.message || getUIText("setting_profile_save_failed", "Failed to save profile"), { tone: "error" });
  }
}

function closeSettingProfileActionMenus(exceptPanel = null) {
  document.querySelectorAll(".setting-profile-menu.is-open").forEach((menu) => {
    if (exceptPanel && menu === exceptPanel) return;
    menu.classList.remove("is-open");
    const button = menu.querySelector(".setting-profile-menu__button");
    const panel = menu.querySelector(".setting-profile-menu__panel");
    if (button) button.setAttribute("aria-expanded", "false");
    if (panel) {
      panel.hidden = true;
      panel.setAttribute("aria-hidden", "true");
    }
  });
}

function settingProfileActionIcon(kind) {
  const paths = {
    edit: "M4 20h4l10.5-10.5a2.12 2.12 0 0 0-3-3L5 17v3m11-12 3 3",
    search: "M10.5 18a7.5 7.5 0 1 1 5.3-12.8 7.5 7.5 0 0 1 0 10.6L20 20",
    info: "M12 17v-6m0-4h.01M12 22a10 10 0 1 0 0-20 10 10 0 0 0 0 20",
    apply: "m5 12 4 4L19 6",
    delete: "M6 7h12m-10 0 .7 13h6.6L16 7M10 7V4h4v3",
  };
  const path = paths[kind] || paths.info;
  return `
    <svg class="setting-profile-action__icon" viewBox="0 0 24 24" aria-hidden="true" focusable="false">
      <path d="${path}"></path>
    </svg>
  `;
}

function makeSettingProfileMenuItem({ label, icon = "info", onClick, className = "" }) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = `setting-profile-menu__item ui-dropdown-menu__item${className ? ` ${className}` : ""}`;
  button.setAttribute("role", "menuitem");
  button.innerHTML = `
    ${settingProfileActionIcon(icon)}
    <span>${settingsDiffEscape(label)}</span>
  `;
  button.onclick = (event) => {
    event.stopPropagation();
    closeSettingProfileActionMenus();
    if (typeof onClick === "function") onClick();
  };
  return button;
}

function renderSettingProfileMetaRows(profile) {
  const meta = profile?.meta || {};
  const created = profile?.created_at ? settingsDiffEscape(formatSettingProfileDate(profile.created_at)) : "-";
  const branch = meta.branch ? settingsDiffEscape(meta.branch) : "-";
  let commit = "-";
  if (meta.commit) {
    const commitText = meta.commit_short || String(meta.commit).slice(0, 7);
    commit = meta.commit_url
      ? `<a href="${settingsDiffEscape(meta.commit_url)}" target="_blank" rel="noopener">${settingsDiffEscape(commitText)}</a>`
      : settingsDiffEscape(commitText);
  }
  return [
    [getUIText("setting_profile_created", "Created"), created],
    [getUIText("branch", "Branch"), branch],
    [getUIText("commit", "Commit"), commit],
  ];
}

function appendSettingProfileHeader(profile, container) {
  if (!container || !profile) return;
  const panel = document.createElement("div");
  panel.className = "setting-profile-panel setting-section-block ui-stagger-item";

  const card = document.createElement("div");
  card.className = "setting-profile-manage-card setting-group-card";
  const valueCount = Object.keys(profile.values || {}).length;

  const nameRow = document.createElement("div");
  nameRow.className = "setting-profile-row setting-profile-row--name";
  const nameLabel = document.createElement("div");
  nameLabel.className = "setting-profile-row__label";
  nameLabel.textContent = getUIText("setting_profile_card_title", "Profile ({count})", { count: valueCount });
  const nameInput = document.createElement("input");
  nameInput.className = "setting-profile-name-input";
  nameInput.type = "text";
  nameInput.maxLength = 40;
  nameInput.value = profile.name || "";
  nameInput.setAttribute("aria-label", getUIText("setting_profile_name", "Profile name"));

  let nameSaveTimer = 0;
  let nameSaveInFlight = null;
  async function persistProfileName() {
    const nextName = nameInput.value.trim();
    if (!nextName) {
      nameInput.value = profile.name || "";
      return;
    }
    if (nextName === profile.name) return;
    if (nameSaveInFlight) {
      try { await nameSaveInFlight; } catch {}
      if (nextName === profile.name) return;
    }
    try {
      nameInput.classList.add("is-saving");
      nameSaveInFlight = saveSettingProfile(profile.id, { name: nextName });
      const nextProfile = await nameSaveInFlight;
      if (nextProfile) profile.name = nextProfile.name;
      renderGroups({ animateGroups: false });
      setSettingItemsTitle(profile.name);
      showAppToast(getUIText("setting_profile_saved", "Profile saved"));
    } catch (e) {
      showAppToast(e?.message || getUIText("setting_profile_save_failed", "Failed to save profile"), { tone: "error" });
    } finally {
      nameSaveInFlight = null;
      nameInput.classList.remove("is-saving");
    }
  }
  function scheduleProfileNameSave(delay = 650) {
    if (nameSaveTimer) clearTimeout(nameSaveTimer);
    nameSaveTimer = window.setTimeout(() => {
      nameSaveTimer = 0;
      persistProfileName().catch(() => {});
    }, delay);
  }
  nameInput.addEventListener("input", () => scheduleProfileNameSave());
  nameInput.addEventListener("blur", () => {
    if (nameSaveTimer) {
      clearTimeout(nameSaveTimer);
      nameSaveTimer = 0;
    }
    persistProfileName().catch(() => {});
  });
  nameInput.addEventListener("keydown", (event) => {
    if (event.key !== "Enter") return;
    event.preventDefault();
    if (nameSaveTimer) {
      clearTimeout(nameSaveTimer);
      nameSaveTimer = 0;
    }
    persistProfileName().then(() => nameInput.blur()).catch(() => {});
  });

  const menu = document.createElement("div");
  menu.className = "setting-profile-menu ui-dropdown-menu";
  const menuBtn = document.createElement("button");
  menuBtn.type = "button";
  menuBtn.className = "setting-profile-menu__button ui-dropdown-menu__button";
  menuBtn.setAttribute("aria-haspopup", "menu");
  menuBtn.setAttribute("aria-expanded", "false");
  menuBtn.setAttribute("aria-label", getUIText("setting_profile_menu", "Profile menu"));
  menuBtn.innerHTML = `
    <svg viewBox="0 0 24 24" aria-hidden="true" focusable="false">
      <path fill="currentColor" d="M12 8a2 2 0 1 0 0-4 2 2 0 0 0 0 4m0 2a2 2 0 1 0 0 4 2 2 0 0 0 0-4m0 6a2 2 0 1 0 0 4 2 2 0 0 0 0-4"/>
    </svg>
  `;
  const menuPanel = document.createElement("div");
  menuPanel.className = "setting-profile-menu__panel ui-dropdown-menu__panel";
  menuPanel.setAttribute("role", "menu");
  menuPanel.setAttribute("aria-hidden", "true");
  menuPanel.hidden = true;
  menuPanel.appendChild(makeSettingProfileMenuItem({
    label: getUIText("apply", "Apply"),
    icon: "apply",
    onClick: () => applySettingProfile(profile),
    className: "setting-profile-menu__item--primary",
  }));
  menuPanel.appendChild(makeSettingProfileMenuItem({
    label: getUIText("setting_profile_search", "Search Profile"),
    icon: "search",
    onClick: () => openSettingSearchPanel({ scope: { type: "profile", profileId: profile.id } }).catch(() => {}),
  }));
  menuPanel.appendChild(makeSettingProfileMenuItem({
    label: getUIText("delete", "Delete"),
    icon: "delete",
    onClick: () => deleteSettingProfile(profile),
    className: "setting-profile-menu__item--danger",
  }));
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

  const rows = document.createElement("div");
  rows.className = "setting-profile-card__rows";

  nameRow.appendChild(nameLabel);
  nameRow.appendChild(nameInput);
  nameRow.appendChild(menu);
  rows.appendChild(nameRow);

  renderSettingProfileMetaRows(profile).forEach(([label, value]) => {
    const row = document.createElement("div");
    row.className = "setting-profile-row";
    const rowLabel = document.createElement("div");
    rowLabel.className = "setting-profile-row__label";
    rowLabel.textContent = label;
    const rowValue = document.createElement("div");
    rowValue.className = "setting-profile-row__value";
    rowValue.innerHTML = value;
    row.appendChild(rowLabel);
    row.appendChild(rowValue);
    rows.appendChild(row);
  });

  card.appendChild(rows);
  panel.appendChild(card);
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
  const contextGroupLabel = getSettingItemContextLabel(group, item);
  const title = formatItemText(item, "title", "etitle", "");
  const descr = formatItemText(item, "descr", "edescr", "");
  const isProfile = source === "profile" && profile?.id;
  const profileName = isProfile ? String(profile.name || "") : "";
  const sourceLabel = isProfile
    ? getUIText("setting_search_source_profile", "Profile")
    : getUIText("setting_search_source_carrot", "CarrotPilot");
  const contextLabel = isProfile
    ? `${profileName} / ${contextGroupLabel}`
    : contextGroupLabel;

  return {
    source: isProfile ? "profile" : "carrot",
    sourceLabel,
    profileId: isProfile ? profile.id : "",
    profileName,
    group: isProfile ? settingProfileGroup(profile.id) : group,
    originalGroup: group,
    groupLabel,
    contextGroupLabel,
    contextLabel,
    name: item.name,
    title,
    descr,
    haystack: [sourceLabel, profileName, groupLabel, contextGroupLabel, item.name, title, descr].join("\n").toLowerCase(),
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

function isPortraitInternalScrollMode() {
  return Boolean(window.matchMedia && window.matchMedia("(max-width: 640px) and (orientation: portrait)").matches);
}

function getSettingItemsScrollContainer() {
  if (isCompactLandscapeMode() && screenItems) return screenItems;
  // 세로 구조(뷰포트 고정 높이 + 화면 내부 스크롤)에서는 활성 화면이 스크롤러다.
  if (isPortraitInternalScrollMode()) {
    if (screenItems && screenItems.style.display !== "none" && !screenItems.classList.contains("hidden")) return screenItems;
    if (screenGroups && screenGroups.style.display !== "none" && !screenGroups.classList.contains("hidden")) return screenGroups;
  }
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
  return itemsBox.dataset.renderedGroup === group && !itemsBox.dataset.renderedDetail && itemsBox.childElementCount > 0;
}

function isCarrotSettingTabActive() {
  return !(typeof getCurrentSettingTab === "function" && getCurrentSettingTab() === "device");
}

function syncSettingGroupChrome(group = CURRENT_GROUP) {
  const meta = document.getElementById("groupMeta");
  const list = getSettingItemEntriesForGroup(group);
  const profile = getSettingProfileByGroup(group);
  const groupLabel = group ? getSettingGroupLabel(group) : "";
  if (CURRENT_SETTING_DETAIL) {
    const detailEntry = getSettingDetailEntry(group, CURRENT_SETTING_DETAIL);
    const detailTitle = detailEntry?.item ? getSettingDetailTitle(detailEntry.item) : CURRENT_SETTING_DETAIL;
    if (meta && group) meta.textContent = `${group} / 1`;
    if (group) {
      settingTitle.textContent = (UI_STRINGS[LANG].setting || "Setting") + " - " + detailTitle;
      setSettingItemsTitle(detailTitle);
    }
    return;
  }
  if (meta && group) {
    meta.classList.remove("setting-profile-meta");
    meta.textContent = profile ? "" : `${group} / ${list.length}`;
  }
  if (group) {
    settingTitle.textContent = (UI_STRINGS[LANG].setting || "Setting") + " - " + groupLabel;
    setSettingItemsTitle(groupLabel);
  }
}

function getSettingDetailEntry(group, name) {
  const target = String(name || "").trim();
  if (!target) return null;
  const entries = getSettingItemEntriesForGroup(group);
  return entries.find((entry) => entry?.item?.name === target) || null;
}

function getSettingDetailTitle(item) {
  return formatItemText(item, "title", "etitle", item?.name || "");
}

function isSettingInlineControlTarget(target) {
  return Boolean(target?.closest?.(".ctrl, button, input, select, textarea, a"));
}

async function selectSettingDetail(group, name, pushHistory = true) {
  const targetGroup = group || CURRENT_GROUP;
  const targetName = String(name || "").trim();
  if (!targetGroup || !targetName) return;

  saveCurrentSettingScrollPosition(targetGroup);
  CURRENT_GROUP = targetGroup;
  CURRENT_SETTING_DETAIL = targetName;
  if (pushHistory) {
    history.pushState({
      page: "setting",
      screen: "detail",
      group: targetGroup,
      settingName: targetName,
    }, "");
  }
  await transitionSettingItemsContent(() => renderItems(targetGroup, {
    detailName: targetName,
    scrollMode: "top",
    animateItems: false,
    forceValues: true,
  }), "forward");
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
    const distance = Math.max(0, content.scrollWidth - el.clientWidth);
    const nextDistance = `${distance}px`;
    const prevDistance = el.style.getPropertyValue("--setting-marquee-distance");
    const wasOverflowing = el.classList.contains("is-overflowing");
    if (el._settingMarqueeResetTimer) {
      clearTimeout(el._settingMarqueeResetTimer);
      el._settingMarqueeResetTimer = null;
    }
    if (el._settingMarqueeRestoreTimer) {
      clearTimeout(el._settingMarqueeRestoreTimer);
      el._settingMarqueeRestoreTimer = null;
    }
    el._settingMarqueeResetting = false;
    el.classList.remove("is-manual");
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
      <div class="setting-search-section__body"></div>
    `;
    settingSearchResults.appendChild(sectionEl);
    const sectionBody = sectionEl.querySelector(".setting-search-section__body");

    section.entries.forEach((entry) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "setting-search-result";
      const metaLabel = entry.source === "profile"
        ? `${entry.profileName} / ${entry.contextGroupLabel || entry.groupLabel}`
        : entry.contextGroupLabel || entry.groupLabel;
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
          if (CURRENT_GROUP === entry.group && !CURRENT_SETTING_DETAIL && screenItems && screenItems.style.display !== "none") {
            focusSettingItem(entry.name);
            return;
          }
          await activateSettingGroup(entry.group, true);
        } catch (e) {
          showAppToast(e.message || "Search jump failed", { tone: "error" });
        }
      };
      sectionBody.appendChild(button);
    });
  });
}

async function openSettingSearchPanel(options = {}) {
  const pushHistory = options.pushHistory !== false;
  const scope = options.scope || { type: "all", profileId: "" };
  if (CURRENT_PAGE !== "setting") return;
  closeSettingFabMenu();
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
      screen: CURRENT_SETTING_DETAIL ? "detail" : ((screenItems && screenItems.style.display !== "none") ? "items" : "groups"),
      group: CURRENT_GROUP || null,
      settingName: CURRENT_SETTING_DETAIL || null,
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

if (btnSettingSearch) {
  btnSettingSearch.addEventListener("animationend", (event) => {
    if (event.animationName === "setting-fab-bounce") {
      btnSettingSearch.classList.remove("is-bouncing");
    }
  });
  btnSettingSearch.onclick = () => {
    // 빠른 연타에도 바운스가 다시 재생되도록 클래스 제거 후 reflow 강제
    btnSettingSearch.classList.remove("is-bouncing");
    void btnSettingSearch.offsetWidth;
    btnSettingSearch.classList.add("is-bouncing");
    toggleSettingFabMenu();
  };
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
  if (e.key === "Escape" && document.querySelector(".setting-profile-menu.is-open")) {
    closeSettingProfileActionMenus();
    return;
  }
  if (e.key === "Escape" && settingFabMenuOpen) {
    closeSettingFabMenu();
  }
});

document.addEventListener("pointerdown", (e) => {
  if (!(e.target instanceof Element) || !e.target.closest(".setting-profile-menu")) {
    closeSettingProfileActionMenus();
  }
  if (settingFabMenuOpen && settingFabMenu && !settingFabMenu.contains(e.target)) {
    closeSettingFabMenu();
  }
});

window.addEventListener("carrot:pagechange", (event) => {
  if (event?.detail?.page !== "setting") {
    closeSettingProfileActionMenus();
    closeSettingFabMenu();
  }
});

async function transitionSettingItemsContent(renderContent, direction = "forward") {
  if (typeof renderContent !== "function") return false;

  const reduceMotion = Boolean(window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches);
  const canAnimate =
    !window.__CARROT_WEB_BOOTSTRAPPING &&
    !reduceMotion &&
    !isCompactLandscapeMode() &&
    !settingGroupTransitionLock &&
    settingScreenHost &&
    screenItems &&
    screenItems.style.display !== "none" &&
    !screenItems.classList.contains("hidden");

  if (!canAnimate) {
    if (screenItems && (screenItems.style.display === "none" || screenItems.classList.contains("hidden"))) {
      showSettingScreen("items", false);
    }
    await renderContent();
    return false;
  }

  settingGroupTransitionLock = true;
  const snapshot = screenItems.cloneNode(true);
  snapshot.classList.add("setting-screen-snapshot");
  snapshot.querySelectorAll(".is-longpressing, .is-bouncing").forEach((el) => {
    el.classList.remove("is-longpressing", "is-bouncing");
  });
  snapshot.setAttribute("aria-hidden", "true");
  snapshot.style.pointerEvents = "none";

  try {
    settingScreenHost.classList.add("setting-screen-transitioning");
    document.getElementById("pageSetting")?.classList.add("setting-screen-transitioning");
    settingScreenHost.appendChild(snapshot);
    prepareSwipeFrame(settingScreenHost, snapshot);
    snapshot.style.zIndex = "1";
    screenItems.style.visibility = "hidden";
    screenItems.style.pointerEvents = "none";

    await renderContent();

    screenItems.querySelectorAll(".ui-stagger-item").forEach((el) => el.classList.remove("ui-stagger-item"));
    screenItems.style.visibility = "";
    const frame = prepareSwipeFrame(settingScreenHost, snapshot, screenItems);
    if (!frame) {
      resetPageRuntimeStyles(screenItems);
      snapshot.remove();
      settingScreenHost.classList.remove("setting-screen-transitioning");
      document.getElementById("pageSetting")?.classList.remove("setting-screen-transitioning");
      settingGroupTransitionLock = false;
      return false;
    }

    applySwipeDrag(frame, 0, direction, false, { fade: false });
    await new Promise((resolve) => {
      settleSwipe(frame, direction, true, resolve, {
        durationMs: SETTING_SCREEN_SLIDE_MS,
        easing: SETTING_SCREEN_SLIDE_EASE,
        fade: false,
      });
    });

    clearPageTransitionClasses(screenItems);
    resetPageRuntimeStyles(screenItems);
    snapshot.remove();
    settingScreenHost.style.minHeight = "";
    settingScreenHost.classList.remove("setting-screen-transitioning");
    document.getElementById("pageSetting")?.classList.remove("setting-screen-transitioning");
    settingGroupTransitionLock = false;
    scheduleSettingOverflowSync(screenItems);
    return true;
  } catch (e) {
    clearPageTransitionClasses(screenItems);
    resetPageRuntimeStyles(screenItems);
    if (snapshot.parentElement) snapshot.remove();
    if (settingScreenHost) {
      settingScreenHost.style.minHeight = "";
      settingScreenHost.classList.remove("setting-screen-transitioning");
    }
    document.getElementById("pageSetting")?.classList.remove("setting-screen-transitioning");
    settingGroupTransitionLock = false;
    throw e;
  }
}

async function activateSettingGroup(group, pushHistory = true, options = {}) {
  if (!isCarrotSettingTabActive()) return;
  const nextGroup = group || CURRENT_GROUP;
  const previousGroup = CURRENT_GROUP;
  CURRENT_SETTING_DETAIL = null;
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
  // 드릴인 시 그룹 목록을 재생성/재-stagger 하지 않는다 — 그 변화가 '슬라이드로 나가는 중인'
  // 최상위 메뉴에 보여서 어색했다. 활성 표시만 제자리 갱신(reuse 경로)한다.
  renderGroups({ animateGroups: false });
  if (isCompactLandscapeMode() && CURRENT_PAGE === "setting") {
    showSettingScreen("items", false);
    history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
    syncSettingGroupChrome(group);
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

  if (canReuseRenderedGroup) {
    showSettingScreen("items", pushHistory);
    if (!pushHistory) {
      history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
    }
    syncSettingGroupChrome(group);
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
  // 단계 진입(items)은 좌우 슬라이드로만 보여준다 — 행별 세로 stagger 없이
  // 한 덩어리로 슬라이드 인 (One UI). 세로 stagger 는 설정 첫 진입에서만.
  await renderItems(group, {
    scrollMode,
    scrollTop: options.scrollTop,
    animateItems: false,
    allowHidden: true,
  });
  showSettingScreen("items", pushHistory);
  if (!pushHistory) {
    history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
  }
  syncSettingGroupChrome(group);
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
  const detailName = String(options.detailName || "").trim();
  const detailMode = Boolean(detailName);
  const scrollMode = options.scrollMode || "top";
  const animateItems = options.animateItems !== false;
  const allowHidden = options.allowHidden === true;
  const requestedScrollTop = Number.isFinite(options.scrollTop) ? options.scrollTop : null;
  itemsBox.innerHTML = "";
  delete itemsBox.dataset.renderedGroup;
  delete itemsBox.dataset.renderedDetail;

  const allEntries = getSettingItemEntriesForGroup(group);
  const detailEntry = detailMode ? getSettingDetailEntry(group, detailName) : null;
  const entries = detailMode ? (detailEntry ? [detailEntry] : []) : allEntries;
  const list = entries.map((entry) => entry.item);
  const profile = getSettingProfileByGroup(group);
  if (screenItems) screenItems.classList.toggle("setting-screen-items--profile", Boolean(profile));
  if (screenItems) screenItems.classList.toggle("setting-screen-items--detail", detailMode);
  CURRENT_SETTING_DETAIL = detailMode ? detailName : null;
  const groupLabel = getSettingGroupLabel(group);
  const detailTitle = detailMode && list[0] ? getSettingDetailTitle(list[0]) : "";
  settingTitle.textContent = (UI_STRINGS[LANG].setting || "Setting") + " - " + (detailTitle || groupLabel);
  setSettingItemsTitle(detailTitle || groupLabel);
  if (meta) {
    meta.classList.remove("setting-profile-meta");
    meta.textContent = profile && !detailMode ? "" : `${group} / ${detailMode ? "1" : list.length}`;
  }

  let values = {};
  try {
    values = await fetchSettingGroupValues(group, {
      force: options.forceValues === true,
      ttlMs: Number.isFinite(options.ttlMs) ? options.ttlMs : SETTING_VALUES_TTL_MS,
    });
  } catch (e) {
    values = {};
  }

  if (
    renderToken !== settingRenderToken ||
    CURRENT_GROUP !== group ||
    !isCarrotSettingTabActive() ||
    (!allowHidden && screenItems?.style.display === "none")
  ) {
    return;
  }

  if (!list.length && detailMode) {
    const empty = document.createElement("div");
    empty.className = "setting-favorites-empty";
    const emptyTitle = document.createElement("div");
    emptyTitle.className = "setting-favorites-empty__title";
    emptyTitle.textContent = getUIText("setting_not_found", "Setting not found");
    const emptyDesc = document.createElement("div");
    emptyDesc.className = "setting-favorites-empty__desc";
    emptyDesc.textContent = detailName;
    empty.appendChild(emptyTitle);
    empty.appendChild(emptyDesc);
    itemsBox.appendChild(empty);
    itemsBox.dataset.renderedGroup = group;
    itemsBox.dataset.renderedDetail = detailName;
    requestAnimationFrame(resetSettingItemsViewport);
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

  if (profile && !detailMode) appendSettingProfileHeader(profile, itemsBox);

  const profileSectionCounts = new Map();
  if (profile) {
    entries.forEach((entry) => {
      profileSectionCounts.set(entry.group, (profileSectionCounts.get(entry.group) || 0) + 1);
    });
  }
  let lastProfileGroup = "";
  let currentProfileSectionBody = null;
  let lastCategorySectionKey = null;
  let currentCategoryCardBody = null;

  if (detailMode && list.length) {
    const detailBlock = document.createElement("section");
    detailBlock.className = animateItems ? "setting-section-block ui-stagger-item" : "setting-section-block";
    if (animateItems) detailBlock.style.setProperty("--i", "1");
    const detailCard = document.createElement("div");
    detailCard.className = "setting-group-card";
    const detailBody = document.createElement("div");
    detailBody.className = "setting-group-card__body";
    detailCard.appendChild(detailBody);
    detailBlock.appendChild(detailCard);
    itemsBox.appendChild(detailBlock);
    currentCategoryCardBody = detailBody;
  }

  // 즐겨찾기도 다른 하위메뉴와 같은 카드 박스(공통분모: setting-section-block +
  // setting-group-card)에 담는다. 즐겨찾기는 소-섹션이 섞여 있으므로 단일 카드 1개로.
  if (!detailMode && isSettingFavoritesGroup(group) && list.length) {
    const favBlock = document.createElement("section");
    favBlock.className = animateItems ? "setting-section-block ui-stagger-item" : "setting-section-block";
    if (animateItems) favBlock.style.setProperty("--i", "1");
    const favCard = document.createElement("div");
    favCard.className = "setting-group-card";
    const favBody = document.createElement("div");
    favBody.className = "setting-group-card__body";
    favCard.appendChild(favBody);
    favBlock.appendChild(favCard);
    itemsBox.appendChild(favBlock);
    currentCategoryCardBody = favBody;
  }

  list.forEach((p, index) => {
    const name = p.name;
    const originGroup = entries[index]?.group || group;
    getSettingUnitIndex(name);

    // 카테고리 모드: 소-섹션마다 카드(그룹박스) 생성 (프로필/즐겨찾기 뷰 제외).
    // 라벨이 있으면 카드 제목으로, 없으면(단일 직속 섹션) 제목 없는 카드.
    if (!detailMode && !profile && !isSettingFavoritesGroup(group) && p.__section) {
      const secKey = p.__section.id || "";
      if (secKey !== lastCategorySectionKey) {
        lastCategorySectionKey = secKey;
        const sectionBlock = document.createElement("section");
        sectionBlock.className = animateItems ? "setting-section-block ui-stagger-item" : "setting-section-block";
        if (animateItems) sectionBlock.style.setProperty("--i", String(Math.min(index + 1, 14)));
        const cardLabel = settingNodeLabel(p.__section);
        if (cardLabel) {
          const cardTitle = document.createElement("div");
          cardTitle.className = "setting-group-card__title";
          cardTitle.textContent = cardLabel;
          sectionBlock.appendChild(cardTitle);
        }
        const card = document.createElement("div");
        card.className = "setting-group-card";
        const cardBody = document.createElement("div");
        cardBody.className = "setting-group-card__body";
        card.appendChild(cardBody);
        sectionBlock.appendChild(card);
        itemsBox.appendChild(sectionBlock);
        currentCategoryCardBody = cardBody;
      }
    }

    if (!detailMode && profile && originGroup !== lastProfileGroup) {
      lastProfileGroup = originGroup;
      const section = document.createElement("div");
      section.className = animateItems ? "setting-section-block setting-profile-section ui-stagger-item" : "setting-section-block setting-profile-section";
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
      bodyInner.className = "setting-group-card setting-group-card__body setting-profile-section__bodyInner";
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
    const rangeMeta = formatSettingRangeMeta(p);

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
        ${escapeHtml(rangeMeta)}
      </div>
    `;

    const controlConfig = getSettingControlConfig(p);
    const compactNumeric = controlConfig.kind === "slider";
    const ctrl = document.createElement("div");
    ctrl.className = `ctrl ctrl--${compactNumeric ? "value" : controlConfig.kind}`;

    const val = document.createElement("button");
    val.type = "button";
    val.className = compactNumeric ? "value-surface val setting-value-compact" : "value-surface val";
    val.setAttribute("aria-label", compactNumeric
      ? getUIText("setting_value_detail", "Open detail")
      : getUIText("setting_value_edit", "Edit value"));

    let btnMinus = null;
    let btnPlus = null;
    let unitBtn = null;
    let sliderInput = null;
    let toggleInput = null;
    let selectInput = null;
    const segmentButtons = [];

    if (controlConfig.kind === "toggle") {
      const switchLabel = document.createElement("label");
      switchLabel.className = "setting-switch";
      toggleInput = document.createElement("input");
      toggleInput.type = "checkbox";
      toggleInput.className = "setting-switch__input";
      toggleInput.setAttribute("aria-label", title || name);
      const switchTrack = document.createElement("span");
      switchTrack.className = "setting-switch__track";
      switchLabel.appendChild(toggleInput);
      switchLabel.appendChild(switchTrack);
      ctrl.appendChild(switchLabel);
      ctrl.appendChild(val);
    } else if (controlConfig.kind === "segmented") {
      const segmentWrap = document.createElement("div");
      segmentWrap.className = "setting-segments";
      getSettingOptionValues(controlConfig).forEach((optionValue) => {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "setting-segment";
        button.dataset.value = String(optionValue);
        button.textContent = getSettingOptionLabel(name, optionValue);
        button.setAttribute("aria-pressed", "false");
        segmentButtons.push(button);
        segmentWrap.appendChild(button);
      });
      ctrl.appendChild(segmentWrap);
      ctrl.appendChild(val);
    } else if (controlConfig.kind === "select") {
      selectInput = document.createElement("button");
      selectInput.type = "button";
      selectInput.className = "setting-select setting-select--button";
      selectInput.setAttribute("aria-label", title || name);
      selectInput.setAttribute("aria-haspopup", "dialog");
      ctrl.appendChild(selectInput);
      ctrl.appendChild(val);
    } else if (compactNumeric) {
      btnMinus = document.createElement("button");
      btnMinus.type = "button";
      btnMinus.className = "smallBtn setting-value-arrow setting-value-arrow--prev";
      btnMinus.textContent = "-";
      btnMinus.setAttribute("aria-label", getUIText("setting_value_previous", "Previous value"));

      btnPlus = document.createElement("button");
      btnPlus.type = "button";
      btnPlus.className = "smallBtn setting-value-arrow setting-value-arrow--next";
      btnPlus.textContent = "+";
      btnPlus.setAttribute("aria-label", getUIText("setting_value_next", "Next value"));

      unitBtn = document.createElement("button");
      unitBtn.type = "button";
      unitBtn.className = "setting-unit-cycle";
      setSettingUnitButtonLabel(unitBtn, name);

      ctrl.appendChild(btnMinus);
      ctrl.appendChild(val);
      ctrl.appendChild(btnPlus);
    } else {
      const sliderWrap = document.createElement("div");
      sliderWrap.className = "setting-slider";
      sliderInput = document.createElement("input");
      sliderInput.type = "range";
      sliderInput.className = "setting-slider__input";
      sliderInput.min = String(controlConfig.min);
      sliderInput.max = String(controlConfig.max);
      sliderInput.step = String(controlConfig.unit);
      sliderInput.setAttribute("aria-label", title || name);
      sliderWrap.appendChild(sliderInput);

      btnMinus = document.createElement("button");
      btnMinus.type = "button";
      btnMinus.className = "smallBtn setting-step setting-step--minus";
      btnMinus.textContent = "-";

      btnPlus = document.createElement("button");
      btnPlus.type = "button";
      btnPlus.className = "smallBtn setting-step setting-step--plus";
      btnPlus.textContent = "+";

      unitBtn = document.createElement("button");
      unitBtn.type = "button";
      unitBtn.className = "setting-unit-cycle";
      setSettingUnitButtonLabel(unitBtn, name);

      ctrl.appendChild(sliderWrap);
      ctrl.appendChild(btnMinus);
      ctrl.appendChild(val);
      ctrl.appendChild(btnPlus);
    }

    const popularEntry = getSettingPopularDisplayEntry(p, getSettingPopularValue(name));
    const popularText = renderSettingPopularChipText(p, popularEntry);
    const popularHtml = renderSettingPopularChipHtml(p, popularEntry);

    top.appendChild(left);
    top.appendChild(ctrl);

    const d = document.createElement("div");
    d.className = "descr";
    d.textContent = descr;

    el.appendChild(top);
    el.appendChild(d);

    const popularTopValues = Array.isArray(popularEntry?.top_values) ? popularEntry.top_values : [];
    let popularDetail = null;
    if (detailMode && popularTopValues.length) {
      popularDetail = document.createElement("div");
      popularDetail.className = "setting-popular-detail-block";
      popularDetail.innerHTML = renderSettingPopularDetailHtml(p, popularEntry);
      el.appendChild(popularDetail);
    }

    // Footer actions row: optional unit-cycle (배율) plus a reset-to-default
    // (기본값) button on every item. Pressing 기본값 confirms then restores
    // the param to its declared default. commitSettingValue / normalizeSettingValue
    // are hoisted function declarations below, so referencing them here is fine.
    const actions = document.createElement("div");
    actions.className = "setting-actions";
    if (!detailMode && popularText && popularHtml) {
      const popularChip = document.createElement("span");
      popularChip.className = "setting-popular-value-chip";
      popularChip.innerHTML = popularHtml;
      popularChip.setAttribute("aria-label", popularText);
      actions.appendChild(popularChip);
    }
    if (unitBtn) {
      el.classList.add("setting--has-unit-cycle");
      actions.appendChild(unitBtn);
    }
    const defaultBtn = document.createElement("button");
    defaultBtn.type = "button";
    defaultBtn.className = "setting-default-reset";
    defaultBtn.textContent = getUIText("setting_reset_default", "Default");
    defaultBtn.setAttribute("aria-label", getUIText("setting_reset_default_aria", "Reset to default"));
    defaultBtn.onclick = async (event) => {
      event.stopPropagation();
      const normalizedDefault = normalizeSettingValue(p.default);
      const target = normalizedDefault === null ? p.default : normalizedDefault;
      const current = val.dataset.committedValue ?? val.dataset.rawValue;
      if (String(target) === String(current)) {
        showAppToast(getUIText("setting_already_default", "Already at default"));
        return;
      }
      const ok = await appConfirm(
        getUIText("setting_reset_default_confirm", "Reset to default ({value})?", {
          value: formatSettingDisplayValue(p, target),
        }),
        {
          title: getUIText("setting_reset_default_title", "Reset to default"),
          confirmLabel: getUIText("ok", "OK"),
          cancelLabel: getUIText("cancel", "Cancel"),
        },
      );
      if (!ok) return;
      await commitSettingValue(target);
      showAppToast(getUIText("setting_reset_default_done", "Restored to default"));
    };
    actions.appendChild(defaultBtn);
    el.classList.add("setting--has-actions");
    el.appendChild(actions);

    (currentProfileSectionBody || currentCategoryCardBody || itemsBox).appendChild(el);

    const cur = (name in values) ? values[name] : p.default;
    syncSettingControlState(el, cur);
    val.dataset.committedValue = String(cur);

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
        syncSettingControlState(el, next);
        val.dataset.committedValue = String(next);
        if (!profile) {
          cacheSettingValue(name, next, group);
          if (originGroup !== group) cacheSettingValue(name, next, originGroup);
        }
      } catch (e) {
        showAppToast((UI_STRINGS[LANG].set_failed || "set failed: ") + e.message, { tone: "error" });
      }
    }

    function bindPopularDetailRows() {
      if (!popularDetail) return;
      popularDetail.querySelectorAll("[data-setting-popular-value]").forEach((button) => {
        button.onclick = async (event) => {
          event.stopPropagation();
          const next = normalizeSettingValue(button.dataset.settingPopularValue);
          if (next === null) {
            showAppToast(getUIText("setting_value_invalid", "Enter a valid number."), { tone: "error" });
            return;
          }
          if (String(next) === String(val.dataset.rawValue)) {
            showAppToast(getUIText("setting_popular_value_already_applied", "Already using this value"));
            return;
          }
          const ok = await appConfirm(
            getUIText("setting_popular_value_apply_confirm", "Apply this setting value ({value})?", {
              value: formatSettingPopularValue(p, next),
            }),
            {
              title: getUIText("setting_popular_value_apply_title", "Apply setting value"),
              confirmLabel: getUIText("ok", "OK"),
              cancelLabel: getUIText("cancel", "Cancel"),
            },
          );
          if (!ok) return;
          await commitSettingValue(next);
        };
      });
    }

    bindPopularDetailRows();

    async function applyDelta(sign) {
      const step = getSettingUnitValue(name);
      let curv = Number(val.dataset.rawValue);
      if (Number.isNaN(curv)) curv = Number(p.default);

      let next = curv + sign * step;
      next = clamp(next, Number(p.min), Number(p.max));

      if (Number.isInteger(Number(p.min)) && Number.isInteger(Number(p.max)) && Number.isInteger(step)) {
        next = Math.round(next);
      }

      await commitSettingValue(next);
    }

    let deltaBusy = false;
    async function requestDelta(sign) {
      if (deltaBusy) return;
      deltaBusy = true;
      try {
        await applyDelta(sign);
      } finally {
        deltaBusy = false;
      }
    }

    function bindDeltaButton(button, sign) {
      if (!button) return;

      let holdTimer = null;
      let repeatTimer = null;
      let pointerActive = false;
      let activePointerId = null;
      let suppressClickUntil = 0;
      const holdDelayMs = 900;
      const repeatDelayMs = 160;
      const clickSuppressMs = 450;

      function clearTimers() {
        if (holdTimer) {
          clearTimeout(holdTimer);
          holdTimer = null;
        }
        if (repeatTimer) {
          clearTimeout(repeatTimer);
          repeatTimer = null;
        }
      }

      function stopHold() {
        clearTimers();
        pointerActive = false;
        button.classList.remove("is-holding");
        if (activePointerId !== null && typeof button.releasePointerCapture === "function") {
          try {
            button.releasePointerCapture(activePointerId);
          } catch (_) {
            /* pointer capture may already be released by the browser */
          }
        }
        activePointerId = null;
      }

      function repeatDelta() {
        if (!pointerActive) return;
        requestDelta(sign);
        repeatTimer = window.setTimeout(repeatDelta, repeatDelayMs);
      }

      button.addEventListener("pointerdown", (event) => {
        if (event.button !== undefined && event.button !== 0) return;
        event.stopPropagation();
        event.preventDefault();
        stopHold();
        pointerActive = true;
        activePointerId = event.pointerId;
        suppressClickUntil = Date.now() + clickSuppressMs;
        button.classList.add("is-holding");
        if (typeof button.setPointerCapture === "function") {
          try {
            button.setPointerCapture(activePointerId);
          } catch (_) {
            /* pointer capture is best-effort for repeated input */
          }
        }
        requestDelta(sign);
        holdTimer = window.setTimeout(repeatDelta, holdDelayMs);
      });

      button.addEventListener("pointerup", (event) => {
        event.stopPropagation();
        suppressClickUntil = Date.now() + clickSuppressMs;
        stopHold();
      });
      button.addEventListener("pointercancel", stopHold);
      button.addEventListener("lostpointercapture", stopHold);
      button.addEventListener("click", (event) => {
        event.stopPropagation();
        if (Date.now() < suppressClickUntil) {
          event.preventDefault();
          return;
        }
        requestDelta(sign);
      });
    }

    async function promptSettingValue() {
      const input = await appPrompt(
        rangeMeta,
        {
          title: title || name,
          defaultValue: val.dataset.rawValue ?? String(p.default),
          placeholder: String(p.default),
          confirmLabel: getUIText("ok", "OK"),
          cancelLabel: getUIText("cancel", "Cancel"),
          showCancel: true,
        },
      );
      if (input === null) return;
      const next = normalizeSettingValue(input);
      if (next === null) {
        showAppToast(getUIText("setting_value_invalid", "Enter a valid number."), { tone: "error" });
        return;
      }
      if (String(next) === String(val.dataset.rawValue)) return;
      await commitSettingValue(next);
    }

    async function promptSettingChoice() {
      const current = String(val.dataset.rawValue ?? p.default);
      const choices = getSettingOptionValues(controlConfig).map((optionValue) => {
        const optionText = String(optionValue);
        const isCurrent = optionText === current;
        return {
          label: getSettingOptionLabel(name, optionValue),
          value: optionText,
          selected: isCurrent,
          className: "setting-choice-option",
        };
      });
      const selected = await openAppDialog({
        mode: "choice",
        choiceLayout: "value-grid",
        title: title || name,
        html: true,
        messageHtml: `<div class="setting-choice-dialog">${escapeHtml(name)}<br>${escapeHtml(rangeMeta)}</div>`,
        choices,
        cancelLabel: getUIText("cancel", "Cancel"),
        showCancel: true,
      });
      if (selected === null) return;
      const next = normalizeSettingValue(selected);
      if (next === null || String(next) === String(val.dataset.rawValue)) return;
      await commitSettingValue(next);
    }

    if (toggleInput) {
      toggleInput.onchange = () => {
        commitSettingValue(toggleInput.checked ? 1 : 0);
      };
    }

    if (unitBtn) {
      unitBtn.onclick = (event) => {
        event.stopPropagation();
        cycleSettingUnitValue(name);
        setSettingUnitButtonLabel(unitBtn, name);
      };
    }

    bindDeltaButton(btnMinus, -1);
    bindDeltaButton(btnPlus, +1);

    val.onclick = (event) => {
      event.stopPropagation();
      if (!detailMode && compactNumeric) {
        selectSettingDetail(originGroup, name).catch(() => {});
        return;
      }
      if (controlConfig.kind === "slider") promptSettingValue();
    };

    segmentButtons.forEach((button) => {
      button.onclick = (event) => {
        event.stopPropagation();
        const next = normalizeSettingValue(button.dataset.value);
        if (next === null || String(next) === String(val.dataset.rawValue)) return;
        commitSettingValue(next);
      };
    });

    if (selectInput) {
      selectInput.onclick = (event) => {
        event.stopPropagation();
        promptSettingChoice();
      };
    }

    if (sliderInput) {
      sliderInput.oninput = () => {
        const next = normalizeSettingValue(sliderInput.value);
        if (next !== null) syncSettingControlState(el, next);
      };
      sliderInput.onchange = () => {
        const next = normalizeSettingValue(sliderInput.value);
        if (next === null || String(next) === String(val.dataset.committedValue ?? val.dataset.rawValue)) return;
        commitSettingValue(next);
      };
    }

    if (!detailMode) {
      el.onclick = (event) => {
        if (el.dataset.settingSuppressClick === "1") return;
        if (isSettingInlineControlTarget(event.target)) return;
        selectSettingDetail(originGroup, name).catch(() => {});
      };
    }
  });

  itemsBox.dataset.renderedGroup = group;
  if (detailMode) itemsBox.dataset.renderedDetail = detailName;
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
    return isSettingInlineControlTarget(target);
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
        row.dataset.settingSuppressClick = "1";
        window.setTimeout(() => {
          if (row.dataset.settingSuppressClick === "1") delete row.dataset.settingSuppressClick;
        }, 420);
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

// Let long titles / param names be panned left-right by the user. Automatic
// movement uses transform, while manual movement uses scrollLeft; never allow
// both coordinate systems to remain active at the same time.
function bindSettingMarqueeDrag() {
  ["items", "deviceItems"].forEach((id) => {
    const box = document.getElementById(id);
    if (!box || box.dataset.marqueeDragBound === "1") return;
    box.dataset.marqueeDragBound = "1";

    let drag = null;

    function cancelManualReset(el) {
      if (!el) return;
      if (el._settingMarqueeResetTimer) {
        clearTimeout(el._settingMarqueeResetTimer);
        el._settingMarqueeResetTimer = null;
      }
      if (el._settingMarqueeRestoreTimer) {
        clearTimeout(el._settingMarqueeRestoreTimer);
        el._settingMarqueeRestoreTimer = null;
      }
    }

    function beginManualScroll(el) {
      cancelManualReset(el);
      el._settingMarqueeResetting = false;
      el.classList.add("is-manual");
    }

    function scheduleManualReset(el) {
      if (!el) return;
      cancelManualReset(el);
      el._settingMarqueeResetTimer = window.setTimeout(() => {
        el._settingMarqueeResetTimer = null;
        el._settingMarqueeResetting = true;
        el.scrollTo({ left: 0, behavior: "smooth" });
        el._settingMarqueeRestoreTimer = window.setTimeout(() => {
          el._settingMarqueeRestoreTimer = null;
          el.scrollLeft = 0;
          el.classList.remove("is-manual");
          el._settingMarqueeResetting = false;
        }, 320);
      }, 1200);
    }

    function endDrag(event) {
      if (!drag || (event && event.pointerId !== drag.pointerId)) return;
      const el = drag.el;
      try { el.releasePointerCapture(drag.pointerId); } catch (_) {}
      el.classList.remove("is-dragging");
      drag = null;
      scheduleManualReset(el);
    }

    box.addEventListener("pointerdown", (event) => {
      if (event.button !== undefined && event.button !== 0) return;
      const marquee = event.target.closest(".setting-marquee");
      if (!marquee || !box.contains(marquee) || !marquee.classList.contains("is-overflowing")) return;
      beginManualScroll(marquee);
      drag = {
        el: marquee,
        pointerId: event.pointerId,
        startX: event.clientX,
        startScroll: marquee.scrollLeft,
        moved: false,
      };
      marquee.classList.add("is-dragging");
    });

    box.addEventListener("pointermove", (event) => {
      if (!drag || event.pointerId !== drag.pointerId) return;
      // Touch pans the overflow container natively — don't double-apply scroll.
      if (event.pointerType === "touch") return;
      const dx = event.clientX - drag.startX;
      if (!drag.moved) {
        if (Math.abs(dx) <= 4) return;
        drag.moved = true;
        try { drag.el.setPointerCapture(drag.pointerId); } catch (_) {}
      }
      drag.el.scrollLeft = drag.startScroll - dx;
      if (event.cancelable) event.preventDefault();
    });

    box.addEventListener("scroll", (event) => {
      const marquee = event.target;
      if (!(marquee instanceof Element) || !marquee.classList.contains("setting-marquee")) return;
      if (!marquee.classList.contains("is-manual")) return;
      if (marquee._settingMarqueeResetting) return;
      cancelManualReset(marquee);
      if (!drag || drag.el !== marquee) scheduleManualReset(marquee);
    }, true);

    box.addEventListener("pointerup", endDrag);
    box.addEventListener("pointercancel", endDrag);
    box.addEventListener("lostpointercapture", endDrag);
  });
}

bindSettingMarqueeDrag();

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

  if (splitLandscape) {
    const targetGroup = CURRENT_GROUP || getLandscapeDefaultSettingGroup();
    if (!targetGroup) return;
    CURRENT_GROUP = targetGroup;
    showSettingScreen("items", false);
    syncSettingGroupChrome(targetGroup);
    if (!hasRenderedSettingItems(targetGroup)) {
      await renderItems(targetGroup, {
        detailName: CURRENT_SETTING_DETAIL || "",
        scrollMode: "restore",
        animateItems,
      });
    }
    return;
  }

  if (CURRENT_GROUP) {
    syncSettingGroupChrome(CURRENT_GROUP);
    showSettingScreen("items", false);
    if (!hasRenderedSettingItems(CURRENT_GROUP)) {
      await renderItems(CURRENT_GROUP, {
        detailName: CURRENT_SETTING_DETAIL || "",
        scrollMode: "restore",
        animateItems,
      });
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
      detailName: CURRENT_SETTING_DETAIL || "",
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

