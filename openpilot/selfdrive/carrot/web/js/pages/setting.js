"use strict";

// Setting page — groups, items, value cache, search, screen layout.

let settingsEntryViewPromise = null;
let settingInitialLoadingTimer = null;
let settingRestoreRefreshTimer = null;
const SETTING_INITIAL_SKELETON_ROWS = 5;
const SETTING_INITIAL_SKELETON_DELAY_MS = 140;
const carrotSettingsRuntime = window.CarrotSettingsRuntime;
if (
  !carrotSettingsRuntime?.aux ||
  !carrotSettingsRuntime?.catalog ||
  !carrotSettingsRuntime?.derived ||
  !carrotSettingsRuntime?.docs ||
  !carrotSettingsRuntime?.context ||
  !carrotSettingsRuntime?.entry ||
  !carrotSettingsRuntime?.view ||
  !carrotSettingsRuntime?.values
) {
  throw new Error("CarrotSettingsRuntime is unavailable");
}
// Row markup, step behaviour and segment keyboard support all come from the
// shared UI bundle. Fail here rather than at the first render of every row.
if (!window.CarrotUI?.settingRow?.createControl || !window.CarrotUI?.numericStepper?.create) {
  throw new Error("Carrot setting row components are unavailable");
}
const settingAuxState = carrotSettingsRuntime.aux;
const settingCatalogState = carrotSettingsRuntime.catalog;
const settingDerivedRuntime = carrotSettingsRuntime.derived;
const settingDocumentationRuntime = carrotSettingsRuntime.docs;
const settingContextRuntime = carrotSettingsRuntime.context;
const settingEntryRuntime = carrotSettingsRuntime.entry;
const settingViewRuntime = carrotSettingsRuntime.view;
const settingValueRepository = carrotSettingsRuntime.values;
const SETTING_VALUES_TTL_MS = settingValueRepository.defaultTtlMs;

const SETTING_FAVORITES_GROUP = settingDerivedRuntime.ids.favoritesGroup;
const SETTING_FAVORITES_LONG_PRESS_MS = 620;
const SETTING_FAVORITES_MOVE_TOLERANCE = 10;
const settingProfileSectionExpandedState = new Map();
const settingProfileMenuActions = new WeakMap();
const settingProfileMenuControllers = new Map();
let settingSoundSampleAudio = null;

function getSettingDerivedModel() {
  return settingDerivedRuntime.getModel({
    catalog: SETTINGS,
    favorites: settingAuxState.favorites.get(),
    profiles: settingAuxState.profiles.get(),
    language: LANG,
    labels: {
      favorites: getSettingFavoritesLabel(),
      profiles: getSettingProfilesLabel(),
    },
  });
}

function isSettingFavoritesGroup(group) {
  return group === SETTING_FAVORITES_GROUP;
}

// 대>중>소 노드(카테고리/그룹/섹션)의 현재 언어 라벨. ko/en/zh 직접 보유 노드용.
function settingNodeLabel(node) {
  return settingDerivedRuntime.localizedNodeLabel(node, LANG);
}

// A parameter item stores its title in title/etitle/ctitle, not ko/en/zh, so
// it needs the item text resolver, not the node label one.
function settingItemTitle(item, fallback = "") {
  return settingDerivedRuntime.localizedItemText(item, "title", "etitle", fallback, LANG);
}

function settingProfileGroup(profileId) {
  return getSettingDerivedModel().profileGroup(profileId);
}

function isSettingProfileGroup(group) {
  return getSettingDerivedModel().isProfileGroup(group);
}

function getSettingProfileById(profileId) {
  return getSettingDerivedModel().getProfileById(profileId);
}

function getSettingProfileByGroup(group) {
  return getSettingDerivedModel().getProfileByGroup(group);
}

function findSettingItemByName(name) {
  return getSettingDerivedModel().findItemByName(name);
}

function getProfileSettingEntries(profile) {
  return getSettingDerivedModel().getProfileEntries(profile);
}

function getValidSettingFavoriteNames() {
  return getSettingDerivedModel().getValidFavoriteNames();
}

function isSettingFavorite(name) {
  return settingAuxState.favorites.get().includes(String(name || "").trim());
}

function getSettingFavoritesLabel() {
  return getUIText("setting_favorites", "Favorites");
}

function getSettingProfilesLabel() {
  return getUIText("setting_profiles", "Profiles");
}

function getSettingGroupsForDisplay() {
  return getSettingDerivedModel().getGroupsForDisplay();
}

function getSettingItemEntriesForGroup(group) {
  return getSettingDerivedModel().getItemEntriesForGroup(group);
}

async function loadSettingPopularValues(force = false) {
  return settingAuxState.popular.load(force);
}

function getSettingPopularValue(name) {
  return settingAuxState.popular.getValue(name);
}

async function loadSettingFavorites(force = false) {
  return settingAuxState.favorites.load(force);
}

function invalidateSettingFavoriteRenderState() {
  settingValueRepository.invalidateGroup(SETTING_FAVORITES_GROUP);
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
  const normalized = settingAuxState.favorites.normalize(nextNames);
  const payload = await postJson("/api/setting_favorites", {
    favorites: normalized,
  });
  return settingAuxState.favorites.replace(payload?.favorites || normalized);
}

async function toggleSettingFavorite(name) {
  const cleanName = String(name || "").trim();
  if (!cleanName || !findSettingItemByName(cleanName)) return;

  const previous = settingAuxState.favorites.get().slice();
  const exists = previous.includes(cleanName);
  const next = exists
    ? previous.filter((entry) => entry !== cleanName)
    : [...previous, cleanName];

  settingAuxState.favorites.replace(next);
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
    settingAuxState.favorites.replace(previous);
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
  settingValueRepository.setValue(name, value, group);
}

function applyRestoredSettingValuesToRenderedItems(values, options = {}) {
  if (!values || typeof values !== "object") return false;
  const animate = options.animate !== false;
  let updated = false;
  document.querySelectorAll(".setting[data-setting-name]").forEach((row) => {
    const name = row.dataset.settingName;
    if (!name || !(name in values)) return;
    const valueButton = row.querySelector(".val");
    if (!valueButton) return;
    syncSettingControlState(row, values[name]);
    if (animate) {
      row.classList.add("is-restored-live");
      window.setTimeout(() => row.classList.remove("is-restored-live"), 900);
    }
    updated = true;
  });
  return updated;
}

async function fetchSettingGroupValues(group, options = {}) {
  if (!group) return {};
  const profile = getSettingProfileByGroup(group);
  if (profile) return { ...(profile.values || {}) };
  const names = getSettingGroupParamNames(group);
  const loadOptions = {
    names,
    force: options.force === true,
    ttlMs: Number.isFinite(options.ttlMs) ? options.ttlMs : SETTING_VALUES_TTL_MS,
    fetchMissing: bulkGet,
  };
  const cached = options.force !== true
    ? settingValueRepository.peekValues?.(names)
    : null;
  if (cached?.complete) {
    settingValueRepository.loadGroup(group, loadOptions).then((freshValues) => {
      if (CURRENT_GROUP === group && isCarrotSettingTabActive()) {
        const latest = settingValueRepository.peekValues?.(names);
        applyRestoredSettingValuesToRenderedItems(
          latest?.complete ? latest.values : freshValues,
          { animate: false },
        );
      }
    }).catch(() => {});
    return { ...cached.values };
  }
  return settingValueRepository.loadGroup(group, loadOptions);
}

function commitSettingsCatalog(catalog, snapshot = null) {
  if (!catalog || typeof catalog !== "object") return null;
  const prepared = settingCatalogState.commit(catalog, snapshot);
  if (prepared.changed || SETTINGS !== prepared.catalog) {
    SETTINGS = prepared.catalog;
    UNIT_CYCLE = SETTINGS.unit_cycle || UNIT_CYCLE;
    settingValueRepository.clear();
    rebuildSettingSearchEntries();
  }
  settingValueRepository.applyValues({
    ...(snapshot?.values || {}),
    ...(snapshot?.device_values || {}),
  });
  return SETTINGS;
}

const settingEntryController = settingEntryRuntime.createController({
  snapshotStore: carrotSettingsRuntime.store,
  getPreparedCatalog: () => SETTINGS,
  loadLegacyCatalog: () => getJson("/api/settings"),
  loadLegacyAuxiliary: (force) => Promise.allSettled([
    loadSettingFavorites(force),
    loadSettingProfiles(force),
    loadSettingPopularValues(force),
  ]),
  commitCatalog: commitSettingsCatalog,
});

function scheduleSettingInitialLoadingState() {
  if (SETTINGS || settingInitialLoadingTimer) return;
  settingInitialLoadingTimer = window.setTimeout(() => {
    settingInitialLoadingTimer = null;
    if (SETTINGS || CURRENT_PAGE !== "setting") return;

    const list = document.getElementById("groupList");
    const page = document.getElementById("pageSetting");
    if (!list || list.childElementCount) return;

    const fragment = document.createDocumentFragment();
    for (let index = 0; index < SETTING_INITIAL_SKELETON_ROWS; index += 1) {
      const row = document.createElement("div");
      row.className = "setting-group-skeleton";
      row.setAttribute("aria-hidden", "true");
      const line = document.createElement("span");
      line.className = "setting-group-skeleton__line";
      row.appendChild(line);
      fragment.appendChild(row);
    }
    list.dataset.settingsLoading = "true";
    list.appendChild(fragment);
    page?.setAttribute("aria-busy", "true");
  }, SETTING_INITIAL_SKELETON_DELAY_MS);
}

function clearSettingInitialLoadingState() {
  if (settingInitialLoadingTimer) {
    window.clearTimeout(settingInitialLoadingTimer);
    settingInitialLoadingTimer = null;
  }

  const list = document.getElementById("groupList");
  if (list?.dataset.settingsLoading === "true") {
    list.replaceChildren();
    delete list.dataset.settingsLoading;
  }
  document.getElementById("pageSetting")?.removeAttribute("aria-busy");
}

async function presentSettingsEntry(catalog, options = {}) {
  const reusePreparedData = options.reusePreparedData === true;
  const animateOnEnter = options.animateOnEnter === true;
  const meta = document.getElementById("settingsMeta");

  if (meta) {
    meta.textContent = `path: ${catalog.path} | has_params: ${catalog.has_params} | type_api: ${catalog.has_param_type}`;
    meta.hidden = !DEBUG_UI;
  }
  if (!DEBUG_UI) {
    const groupMeta = document.getElementById("groupMeta");
    if (groupMeta) groupMeta.style.display = "none";
    const carMeta = document.getElementById("carMeta");
    if (carMeta) carMeta.style.display = "none";
  }

  clearSettingInitialLoadingState();
  rebuildSettingSearchEntries();
  syncSettingSearchFabState();

  if (CURRENT_GROUP && !getSettingDerivedModel().getGroupMeta(CURRENT_GROUP)) {
    CURRENT_GROUP = null;
    CURRENT_SETTING_DETAIL = null;
  }
  const animateLayout = animateOnEnter || !reusePreparedData;
  await syncSettingViewportLayout({
    animateChrome: animateLayout,
    animateItems: animateLayout,
  });
  if (settingSearchPanel && !settingSearchPanel.hidden) {
    renderSettingSearchResults(settingSearchInput?.value || "");
  }

  if (reusePreparedData) {
    // Popular values are already available from the snapshot. Refresh them
    // after the re-entry paint and retain the prepared values on failure.
    loadSettingPopularValues(true).catch(() => {});
  }
  return catalog;
}

function loadSettings(options = {}) {
  if (settingsEntryViewPromise) return settingsEntryViewPromise;
  const force = options.force === true;
  const animateOnEnter = options.animateOnEnter === true;
  const reusePreparedData = Boolean(SETTINGS) && !force;
  if (!reusePreparedData) scheduleSettingInitialLoadingState();

  const task = settingEntryController.prepare({ force })
    .then(({ catalog }) => presentSettingsEntry(catalog, {
      reusePreparedData,
      animateOnEnter,
    }))
    .catch((e) => {
      const meta = document.getElementById("settingsMeta");
      settingSearchEntries = [];
      clearSettingInitialLoadingState();
      if (meta) {
        meta.textContent = "Failed: " + (e?.message || "unknown");
        meta.hidden = false;
      }
      throw e;
    });

  const tracked = task.finally(() => {
    if (settingsEntryViewPromise === tracked) settingsEntryViewPromise = null;
  });
  settingsEntryViewPromise = tracked;
  return tracked;
}

let settingOverflowSyncRaf = 0;
let settingOverflowSyncTimer = 0;
let settingOverflowResizeObserver = null;

function syncSettingGroupLabelOverflow(root = document) {
  const scope = root && typeof root.querySelectorAll === "function" ? root : document;
  let buttons;
  if (scope.matches?.("#groupList .groupBtn, #deviceGroupList .groupBtn")) {
    buttons = [scope];
  } else {
    const selector = (scope.id === "groupList" || scope.id === "deviceGroupList")
      ? ".groupBtn"
      : "#groupList .groupBtn, #deviceGroupList .groupBtn";
    buttons = Array.from(scope.querySelectorAll(selector));
  }
  if (!buttons.length) return;

  // Read every button's geometry first, then apply all writes, so a full group
  // list settles in one reflow instead of one reflow per button (read/write
  // interleaving previously thrashed layout ~18 times).
  const writes = [];
  for (const button of buttons) {
    const labelEl = button.querySelector(".setting-group-label");
    if (!labelEl) continue;
    const buttonWidth = button.clientWidth || 0;
    if (buttonWidth <= 0) continue;
    writes.push([button, Math.min(0, buttonWidth - labelEl.scrollWidth - 8)]);
  }
  for (const [button, shift] of writes) {
    button.style.setProperty("--setting-label-shift", `${shift}px`);
    button.classList.toggle("is-overflowing", shift < 0);
  }
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
  if (!box) return;
  if (box.dataset.settingGroupSelectionBound !== "1") {
    box.dataset.settingGroupSelectionBound = "1";
    box.addEventListener("click", (event) => {
      const button = event.target.closest("button[data-group]");
      if (!button || !box.contains(button)) return;
      selectGroup(button.dataset.group);
    });
  }

  const plan = settingViewRuntime.createGroupPlan({
    groups: getSettingGroupsForDisplay(),
    currentGroup: CURRENT_GROUP,
    ids: settingDerivedRuntime.ids,
    profilesLabel: getSettingProfilesLabel(),
    getGroupLabel: getSettingGroupLabel,
  });
  settingViewRuntime.renderGroupList(box, plan, { animate: animateGroups });
  scheduleSettingOverflowSync(box);
}

function getSettingGroupLabel(group) {
  return getSettingDerivedModel().getGroupLabel(group);
}

// Control kinds a parameter may declare via its "control" field.
const SETTING_CONTROL_KINDS = ["toggle", "segmented", "select", "slider"];

function getSettingControlConfig(p) {
  // A parameter pins its control with a "control" field in
  // carrot_settings.json; otherwise the kind is inferred from its range below.
  const override = SETTING_CONTROL_KINDS.includes(p?.control) ? { kind: p.control } : {};
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


function getSettingDisplayType(name) {
  // The unit is declared by the parameter itself in carrot_settings.json, so
  // adding a parameter no longer means editing a table in here as well.
  const declared = findSettingItemByName(String(name || "").trim())?.item?.display_unit;
  return declared && declared in SETTING_DISPLAY_UNIT_TYPES ? declared : "raw";
}

function getSettingDisplayUnit(name) {
  return SETTING_DISPLAY_UNIT_TYPES[getSettingDisplayType(name)] || "";
}

// A parameter can name its choices with an "options" block in
// carrot_settings.json: { ko: [...], en: [...], zh: [...] } indexed from min.
// Without it the raw number is shown, which is the behaviour for most
// parameters.
function getDeclaredSettingOptionLabel(name, value) {
  const item = findSettingItemByName(String(name || "").trim())?.item;
  const options = item?.options;
  if (!options) return null;

  const language = LANG === "zh" ? "zh" : (LANG === "ko" ? "ko" : "en");
  const labels = options[language] || options.en || options.ko;
  if (!Array.isArray(labels)) return null;

  const index = Number(value) - Number(item.min ?? 0);
  return Number.isInteger(index) ? (labels[index] ?? null) : null;
}

function formatSettingDisplayValue(p, value) {
  if (String(p?.name || "") === "SoundLanguageSetting") {
    return getSoundLanguageSettingOptionLabel(value);
  }
  const declaredOptionLabel = getDeclaredSettingOptionLabel(p?.name, value);
  if (declaredOptionLabel !== null) return declaredOptionLabel;
  const text = String(value);
  const unit = getSettingDisplayUnit(p?.name);
  return unit ? `${text}${unit}` : text;
}

function formatSettingRangeMeta(p) {
  if (String(p?.name || "") === "SoundLanguageSetting") {
    return "";
  }
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

// Popular-value maths (normalize / range / order / display entry) is pure and
// lives in features/settings/popular/popular_values.js. These wrappers inject
// the app's value formatter and locale.
function popularCompareOptions(p) {
  return {
    formatValue: (raw) => formatSettingPopularValue(p, raw),
    locale: LANG === "ko" ? "ko-KR" : undefined,
  };
}

function normalizeSettingPopularNumericValue(p, raw) {
  return carrotSettingsRuntime.popular.normalizeNumeric(p, raw);
}

function isSettingPopularValueInRange(p, raw) {
  return carrotSettingsRuntime.popular.isInRange(p, raw);
}

function getSettingPopularDisplayEntry(p, entry) {
  return carrotSettingsRuntime.popular.buildDisplayEntry(p, entry, popularCompareOptions(p));
}

function getSettingPopularPrimaryCount(entry) {
  return carrotSettingsRuntime.popular.primaryCount(entry);
}

function getSettingPopularSummaryValues(entry) {
  return carrotSettingsRuntime.popular.summaryValues(entry);
}

// Popular-value chip/detail rendering lives in
// features/settings/popular/popular_render.js; these inject the app's value
// formatter, translator, escape and the aux-derived title/updated strings.
function popularRenderOptions(p) {
  return {
    formatValue: (raw) => formatSettingPopularValue(p, raw),
    text: getUIText,
    escape: escapeHtml,
  };
}

function renderSettingPopularChipText(p, entry) {
  return carrotSettingsRuntime.popular.renderChipText(entry, popularRenderOptions(p));
}

function renderSettingPopularChipHtml(p, entry) {
  return carrotSettingsRuntime.popular.renderChipHtml(entry, popularRenderOptions(p));
}

function getSettingPopularDetailTitle() {
  const carKey = String(settingAuxState.popular.getState().carKey || "").trim();
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
  return carrotSettingsRuntime.popular.renderDetailHtml(entry, {
    ...popularRenderOptions(p),
    title: getSettingPopularDetailTitle(),
    updatedText: formatSettingPopularUpdated(settingAuxState.popular.getState().fetchedAt),
  });
}

// The step multiplier lives on the device, not in the browser. Keeping it in
// localStorage meant it was lost on every browser switch and on every "clear
// your cache" instruction, which is advice this project hands out often.
const SETTING_UNIT_STORAGE_KEY = "carrot.settingUnitIndex.v1";
let settingUnitIndexHydrated = false;

function hydrateSettingUnitIndex(units) {
  if (units && typeof units === "object") {
    Object.entries(units).forEach(([name, index]) => {
      const value = Number(index);
      if (Number.isInteger(value) && value > 0 && value < UNIT_CYCLE.length) UNIT_INDEX[name] = value;
    });
  }
  // Runs even when the server sent nothing, so an older server (or an empty
  // store) still gets whatever the browser was holding.
  if (settingUnitIndexHydrated) return;
  settingUnitIndexHydrated = true;
  migrateLocalSettingUnitIndex();
}

// One-time lift of whatever the browser still holds, so nobody loses the
// multipliers they had set before this moved to the server.
function migrateLocalSettingUnitIndex() {
  let stored = null;
  try {
    stored = JSON.parse(localStorage.getItem(SETTING_UNIT_STORAGE_KEY) || "null");
  } catch (_) {
    stored = null;
  }
  if (!stored || typeof stored !== "object") return;

  const units = {};
  Object.entries(stored).forEach(([name, index]) => {
    const value = Number(index);
    if (!Number.isInteger(value) || value <= 0 || value >= UNIT_CYCLE.length) return;
    if (name in UNIT_INDEX) return;
    UNIT_INDEX[name] = value;
    units[name] = value;
  });

  try {
    localStorage.removeItem(SETTING_UNIT_STORAGE_KEY);
  } catch (_) {}
  if (Object.keys(units).length) {
    postJson("/api/setting_unit_index", { units }).catch(() => {});
  }
}

function saveSettingUnitIndex(name, index) {
  const key = String(name || "").trim();
  if (!key) return;
  postJson("/api/setting_unit_index", { units: { [key]: index } }).catch(() => {
    showAppToast(getUIText("setting_unit_save_failed", "배율을 저장하지 못했습니다."), { tone: "error" });
  });
}

function getSettingUnitIndex(name) {
  const key = String(name || "").trim();
  if (!key) return 0;
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

function getSoundLanguageSettingOptions() {
  return [
    { code: "auto", name: getUIText("automatic", "Automatic") },
    { code: "en", name: "English" },
    { code: "ko", name: "한국어" },
    { code: "zh-CHS", name: "中文" },
  ];
}

function getSoundLanguageSettingOptionLabel(value) {
  const text = String(value ?? "").trim();
  const normalized = text.toLowerCase();
  const match = getSoundLanguageSettingOptions().find((option) => String(option.code || "").trim().toLowerCase() === normalized);
  return match?.name || text || getUIText("automatic", "Automatic");
}

function getResolvedSoundLanguageCode(value) {
  const text = String(value ?? "").trim();
  if (text && text.toLowerCase() !== "auto") return text;
  const deviceLang = String(window.__CARROT_BOOTSTRAP__?.deviceLanguage || "").trim();
  return deviceLang || LANG || "en";
}

function getSoundAssetDirForLanguage(value) {
  let normalized = getResolvedSoundLanguageCode(value).replaceAll("_", "-").toLowerCase();
  if (normalized.startsWith("main-")) normalized = normalized.slice(5);
  if (normalized === "ko" || normalized.startsWith("ko-")) return "sounds";
  if (normalized === "zh-chs" || normalized === "zh-hans" || normalized.startsWith("zh")) return "sounds_chs";
  return "sounds_eng";
}

async function playSoundLanguageSample(value) {
  const dir = getSoundAssetDirForLanguage(value);
  const url = `/sound-assets/${encodeURIComponent(dir)}/audio_speed_down.wav`;
  if (settingSoundSampleAudio) {
    settingSoundSampleAudio.pause();
    settingSoundSampleAudio = null;
  }
  const audio = new Audio(url);
  settingSoundSampleAudio = audio;
  audio.addEventListener("ended", () => {
    if (settingSoundSampleAudio === audio) settingSoundSampleAudio = null;
  }, { once: true });
  await audio.play();
}

function getSettingOptionValues(name, config) {
  if (String(name || "") === "SoundLanguageSetting") {
    return getSoundLanguageSettingOptions().map((option) => String(option.code || "").trim()).filter(Boolean);
  }
  if (!config || !Number.isInteger(config.min) || !Number.isInteger(config.max)) return [];
  const out = [];
  for (let value = config.min; value <= config.max; value += 1) out.push(value);
  return out;
}

function getSettingOptionLabel(name, value) {
  if (String(name || "") === "SoundLanguageSetting") {
    return getSoundLanguageSettingOptionLabel(value);
  }
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

  const toggle = row.querySelector(".c-switch__input");
  if (toggle) toggle.checked = Number(value) === 1;

  const slider = row.querySelector(".setting-slider__input");
  if (slider) slider.value = text;

  const segments = row.querySelectorAll(".setting-segment");
  segments.forEach((button) => {
    button.classList.toggle("is-active", String(button.dataset.value) === text);
    button.setAttribute("aria-pressed", String(button.dataset.value) === text ? "true" : "false");
  });
  if (segments.length) {
    // Selection moved, so the keyboard entry point has to move with it.
    // create() returns the already-mounted controller and re-syncs it.
    const group = row.querySelector(".setting-segments");
    if (group) {
      window.CarrotUI?.segmentedControl?.create(group, {
        itemSelector: ".setting-segment",
        selectedAttribute: "aria-pressed",
      });
    }
  }

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
  // Wide layout = nav rail + two-column split. Shared with the whole app via
  // CarrotLayout so fold/large-panel detection matches CSS.
  return window.CarrotLayout
    ? window.CarrotLayout.isWide()
    : window.matchMedia("(min-aspect-ratio: 13/10), (horizontal-viewport-segments: 2), (vertical-viewport-segments: 2), (min-width: 640px) and (min-height: 650px)").matches;
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

function primeSettingsSnapshotForFirstEntry(snapshot = window.CarrotSettingsStore?.peek?.()) {
  hydrateSettingUnitIndex(snapshot?.unit_index);
  const catalog = settingEntryController.primeSnapshot(snapshot);
  if (!catalog || !isCompactLandscapeMode()) return catalog;

  const initialGroup = getLandscapeDefaultSettingGroup();
  if (initialGroup) fetchSettingGroupValues(initialGroup).catch(() => {});
  return catalog;
}

window.addEventListener("carrot:settings-store", (event) => {
  if (event.detail?.status === "ready") primeSettingsSnapshotForFirstEntry();
});

if (window.CarrotSettingsStore?.status === "ready") {
  queueMicrotask(() => primeSettingsSnapshotForFirstEntry());
}

function syncSettingSearchFabState() {
  const isOpen = Boolean(settingSearchPanel && !settingSearchPanel.hidden);
  if (settingPageRoot) settingPageRoot.classList.toggle("setting-search-open", isOpen);
  if (btnSettingSearch) {
    btnSettingSearch.classList.toggle("active", isOpen || settingFabMenuOpen);
    btnSettingSearch.setAttribute("aria-expanded", settingFabMenuOpen ? "true" : "false");
  }
}

async function loadSettingProfiles(force = false) {
  return settingAuxState.profiles.load(force);
}

function updateSettingProfilesFromPayload(payload) {
  if (!payload || !Array.isArray(payload.profiles)) return;
  settingAuxState.profiles.replace(payload.profiles);
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
    // Send only what the preview marked for apply, not the whole profile. The
    // server still re-checks each value, so an entry that quietly became
    // identical is skipped there; the 162 unchanged parameters are just no
    // longer shipped and re-read on every apply. Selection logic is pure and
    // tested in features/settings/profiles/apply_plan.js.
    const applyValues = carrotSettingsRuntime.profiles.selectApplyValues(preview, profile.values);
    const result = await postJson("/api/setting_profiles/apply", { id: profile.id, values: applyValues });
    const restoredValues = carrotSettingsRuntime.profiles.collectRestoredValues(result);
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

function closeSettingProfileActionMenus(exceptMenu = null) {
  settingProfileMenuControllers.forEach((controller, menu) => {
    if (exceptMenu && menu === exceptMenu) return;
    controller.close();
  });
}

function destroySettingProfileActionMenus() {
  settingProfileMenuControllers.forEach((controller) => controller.destroy());
  settingProfileMenuControllers.clear();
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
  if (typeof onClick === "function") settingProfileMenuActions.set(button, onClick);
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
  menu.addEventListener("click", (event) => event.stopPropagation());
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
  // Order: search, apply, delete. Search is the low-stakes, most-used action
  // so it leads; the destructive delete is last, away from the others.
  menuPanel.appendChild(makeSettingProfileMenuItem({
    label: getUIText("setting_profile_search", "Search Profile"),
    icon: "search",
    onClick: () => openSettingSearchPanel({ scope: { type: "profile", profileId: profile.id } }).catch(() => {}),
  }));
  menuPanel.appendChild(makeSettingProfileMenuItem({
    label: getUIText("apply", "Apply"),
    icon: "apply",
    onClick: () => applySettingProfile(profile),
    className: "setting-profile-menu__item--primary",
  }));
  menuPanel.appendChild(makeSettingProfileMenuItem({
    label: getUIText("delete", "Delete"),
    icon: "delete",
    onClick: () => deleteSettingProfile(profile),
    className: "setting-profile-menu__item--danger",
  }));
  menu.appendChild(menuBtn);
  menu.appendChild(menuPanel);
  const menuApi = window.CarrotUI?.menu;
  if (typeof menuApi?.mount === "function") {
    const controller = menuApi.mount({
      root: menu,
      trigger: menuBtn,
      panel: menuPanel,
      itemSelector: ".setting-profile-menu__item",
      beforeOpen: () => {
        closeSettingProfileActionMenus(menu);
        return true;
      },
      onSelect: (item) => {
        const action = settingProfileMenuActions.get(item);
        if (typeof action === "function") {
          Promise.resolve().then(() => action()).catch(() => {});
        }
      },
    });
    settingProfileMenuControllers.set(menu, controller);
  }

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

function rebuildSettingSearchEntries() {
  settingSearchEntries = getSettingDerivedModel().buildSearchEntries({
    carrot: getUIText("setting_search_source_carrot", "CarrotPilot"),
    profile: getUIText("setting_search_source_profile", "Profile"),
  });
  return settingSearchEntries;
}

function getSettingSearchEntries() {
  return settingSearchEntries;
}

// Pure highlight logic (and its escaping guarantees) live in
// features/settings/search/highlight.js; this passes the app's escapeHtml in.
function highlightSettingSearchText(text, query) {
  return carrotSettingsRuntime.search.highlight(text, query, { escape: escapeHtml });
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
  if (window.CarrotLayout?.isWide?.()) return false;
  return Boolean(window.matchMedia && window.matchMedia("(max-width: 640px), (aspect-ratio < 13/10) and (max-width: 639px), (aspect-ratio < 13/10) and (max-height: 649px)").matches);
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

const SETTING_VALUE_CONTROL_HIT_OUTSET = 8;

function isSettingValueControlHit(event, row = null) {
  if (!event || typeof event.clientX !== "number" || typeof event.clientY !== "number") return false;
  const root = row || event.target?.closest?.(".setting[data-setting-name]");
  const ctrl = root?.querySelector?.(".ctrl--value");
  if (!ctrl) return false;
  const rect = ctrl.getBoundingClientRect();
  const outset = SETTING_VALUE_CONTROL_HIT_OUTSET;
  return (
    event.clientX >= rect.left - outset &&
    event.clientX <= rect.right + outset &&
    event.clientY >= rect.top - outset &&
    event.clientY <= rect.bottom + outset
  );
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
  }), "forward");
}

function loadSettingDocumentationPanel(contextPanel, name, renderToken) {
  contextPanel.setLoading(
    "description",
    getUIText("setting_doc_loading", "Loading guide..."),
  );
  settingDocumentationRuntime.load(name, LANG).then((payload) => {
    if (renderToken !== settingRenderToken || !contextPanel.root.isConnected) return;
    if (!payload?.available || !Array.isArray(payload.ast)) {
      contextPanel.setEmpty(
        "description",
        getUIText("setting_context_description_empty", "No detailed description is available yet."),
      );
      return;
    }

    const content = document.createElement("div");
    content.className = "setting-context__documentation";
    if (payload.fallback) {
      const fallback = document.createElement("div");
      fallback.className = "setting-doc-language-fallback";
      fallback.textContent = getUIText(
        "setting_doc_language_fallback",
        "This detailed guide is currently shown in English.",
      );
      content.appendChild(fallback);
    }

    const article = settingDocumentationRuntime.renderAst(document, payload.ast, {
      text: getUIText,
    });
    content.appendChild(article);
    contextPanel.root.dataset.docSource = `${payload.source || ""}#${payload.anchor || ""}`;
    contextPanel.root.dataset.docContentSource = payload.content_source || "";
    contextPanel.setContent("description", content);
  }).catch(() => {
    if (renderToken !== settingRenderToken || !contextPanel.root.isConnected) return;
    contextPanel.setEmpty(
      "description",
      getUIText("setting_doc_load_failed", "The detailed guide could not be loaded."),
    );
  });
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

// One screen that answers "is my configuration still what it was, and can I
// trust the change history?" — the two questions nobody could answer while
// settings were drifting.
if (btnSettingFabFingerprint) {
  btnSettingFabFingerprint.onclick = async () => {
    closeSettingFabMenu();
    try {
      const [fingerprint, integrity, recent] = await Promise.all([
        getJson("/api/param_fingerprint"),
        getJson("/api/param_changes/verify"),
        // Over-fetch so 12 real settings remain after synthetic keys are dropped.
        getJson("/api/param_changes?limit=60").catch(() => ({ changes: [] })),
      ]);
      // Show only real settings, dropping any leftover records for synthetic
      // keys (GitPullTime) or hardware values (DeviceType) already in the log
      // from before they were filtered out. This hides them without a reboot.
      const settingChanges = (recent.changes || [])
        .filter((record) => findSettingItemByName(record?.name))
        .slice(0, 12);
      // Each row is shown by its human title (not the internal key) and its
      // value is formatted the same way the settings screen shows it (units,
      // ON/OFF), so a non-developer reads "타이어공기압 표시  꺼짐 → 켜짐"
      // instead of "ShowTpms  0 → 1".
      const historyHtml = carrotSettingsRuntime.history.renderHtml(settingChanges, {
        escape: escapeHtml,
        text: getUIText,
        locale: LANG === "ko" ? "ko-KR" : undefined,
        withName: true,
        displayName: (record) => {
          const item = findSettingItemByName(record?.name)?.item;
          return settingItemTitle(item, record?.name || "") || record?.name || "";
        },
        formatValue: (value) => String(value ?? ""),
        formatFor: (record, value) => {
          const item = findSettingItemByName(record?.name)?.item;
          return item ? formatSettingPopularValue(item, value) : String(value ?? "");
        },
      });
      // Summary (code + one-line meaning + count/integrity note) is a pure,
      // tested module; the page only supplies the fetched data and formatters.
      const summaryHtml = carrotSettingsRuntime.fingerprint.renderSummary(
        {
          fingerprint: fingerprint.fingerprint,
          count: fingerprint.count,
          integrity,
          baseline: fingerprint.baseline,
          changed: fingerprint.changed,
          changed_count: fingerprint.changed_count,
        },
        { escape: escapeHtml, text: getUIText },
      );
      // "지금을 기준으로" saves the current settings as the reference. The
      // dialog has no per-button hook, so the click is delegated while it is
      // open and detached when it closes.
      const onBaselineSave = async (event) => {
        const button = event.target.closest("[data-setting-fingerprint-save]");
        if (!button) return;
        button.disabled = true;
        try {
          await postJson("/api/param_fingerprint/baseline", {});
          const status = button.parentElement?.querySelector(".setting-fingerprint__status");
          if (status) {
            status.className = "setting-fingerprint__status setting-fingerprint__status--same";
            status.textContent = getUIText("setting_fingerprint_same", "기준과 같아요");
          }
          button.remove();
          showAppToast(getUIText("setting_fingerprint_saved", "지금 설정을 기준으로 저장했어요"));
        } catch (err) {
          button.disabled = false;
          showAppToast(err?.message || getUIText("failed", "Failed"), { tone: "error" });
        }
      };
      document.addEventListener("click", onBaselineSave);
      try {
        await openAppDialog({
          mode: "alert",
          title: getUIText("setting_fingerprint_title", "설정 코드"),
          html: true,
          messageHtml: summaryHtml
            + `<div class="setting-fingerprint-history">`
            + `<div class="setting-fingerprint-history__title">`
            + `${escapeHtml(getUIText("setting_fingerprint_recent", "최근 바뀐 설정"))}</div>${historyHtml}</div>`,
          confirmLabel: getUIText("ok", "OK"),
        });
      } finally {
        document.removeEventListener("click", onBaselineSave);
      }
    } catch (e) {
      showAppToast(e?.message || getUIText("failed", "Failed"), { tone: "error" });
    }
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
  if (e.key === "Escape" && settingFabMenuOpen) {
    closeSettingFabMenu();
  }
});

document.addEventListener("pointerdown", (e) => {
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

function bindSettingProfileSectionToggle(rendered, stateKey) {
  const { section, header } = rendered || {};
  if (!section || !header) return;
  header.onclick = () => {
    const nextExpanded = section.classList.contains("is-collapsed");
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
  destroySettingProfileActionMenus();
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
    settingViewRuntime.renderEmptyState(itemsBox, {
      title: getUIText("setting_not_found", "Setting not found"),
      description: detailName,
    });
    itemsBox.dataset.renderedGroup = group;
    itemsBox.dataset.renderedDetail = detailName;
    requestAnimationFrame(resetSettingItemsViewport);
    return;
  }

  if (!list.length && isSettingFavoritesGroup(group)) {
    settingViewRuntime.renderEmptyState(itemsBox, {
      title: getUIText("setting_favorites_empty_title", "No favorites"),
      description: getUIText(
        "setting_favorites_empty_desc",
        "Long press a setting to add it. Long press again to remove it.",
      ),
    });
    itemsBox.dataset.renderedGroup = group;
    requestAnimationFrame(resetSettingItemsViewport);
    return;
  }

  if (profile && !detailMode) appendSettingProfileHeader(profile, itemsBox);

  // Keep layout decisions independent from row controls: the plan opens a new
  // detail/favorites/category/profile container only where the structure changes.
  const itemLayout = settingViewRuntime.createItemLayoutPlan({
    entries,
    group,
    detailMode,
    profile,
    favoriteMode: isSettingFavoritesGroup(group),
    getSectionLabel: settingNodeLabel,
    getGroupLabel: getSettingGroupLabel,
    getProfileSectionExpanded: (stateKey) => settingProfileSectionExpandedState.has(stateKey)
      ? settingProfileSectionExpandedState.get(stateKey)
      : true,
  });
  let currentItemContainer = itemsBox;

  itemLayout.rows.forEach((rowPlan) => {
    const { item: p, index, originGroup } = rowPlan;
    const name = p.name;
    getSettingUnitIndex(name);

    if (rowPlan.section) {
      const renderedSection = settingViewRuntime.appendItemSection(itemsBox, rowPlan.section, {
        animate: animateItems,
      });
      currentItemContainer = renderedSection?.body || itemsBox;
      if (rowPlan.section.kind === "profile") {
        bindSettingProfileSectionToggle(renderedSection, rowPlan.section.key);
      }
    }

    const title = formatItemText(p, "title", "etitle", "");
    const descr = formatItemText(p, "descr", "edescr", "");
    const rangeMeta = formatSettingRangeMeta(p);
    const rangeMetaHtml = rangeMeta
      ? `<div class="muted mt-sm">${escapeHtml(rangeMeta)}</div>`
      : "";

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
    // A "risk" field on the parameter (carrot_settings.json) renders a warning
    // badge next to its title — declared in data, so no code change is needed
    // to mark a parameter risky.
    const riskBadge = carrotSettingsRuntime.risk.renderBadge(p, { escape: escapeHtml, text: getUIText });
    left.innerHTML = `
      <div class="setting-title-row">
        ${settingMarqueeHtml(title, "title")}
        ${riskBadge}
        ${renderSettingFavoriteMark(name)}
      </div>
      ${settingMarqueeHtml(name, "name")}
      ${rangeMetaHtml}
    `;

    const controlConfig = getSettingControlConfig(p);
    const compactNumeric = controlConfig.kind === "slider";
    // The markup comes from the shared component; this file keeps the wiring.
    const control = window.CarrotUI.settingRow.createControl({
      document,
      kind: compactNumeric ? "stepper" : controlConfig.kind,
      label: title || name,
      valueLabel: getUIText("setting_value_edit", "Edit value"),
      previousLabel: getUIText("setting_value_previous", "Previous value"),
      nextLabel: getUIText("setting_value_next", "Next value"),
      optionValues: controlConfig.kind === "segmented" ? getSettingOptionValues(name, controlConfig) : [],
      optionLabel: (optionValue) => getSettingOptionLabel(name, optionValue),
      min: controlConfig.min,
      max: controlConfig.max,
      step: controlConfig.unit,
    });

    const ctrl = control.ctrl;
    const val = control.value;
    const btnMinus = control.minusButton;
    const btnPlus = control.plusButton;
    const unitBtn = control.unitButton;
    const sliderInput = control.sliderInput;
    const toggleInput = control.toggleInput;
    const selectInput = control.selectInput;
    const segmentGroup = control.segmentGroup;
    const segmentButtons = control.segmentButtons;
    if (unitBtn) setSettingUnitButtonLabel(unitBtn, name);

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
    let contextPanel = null;
    let popularDetail = null;
    let historyBlock = null;
    let historyFullBlock = null;
    if (detailMode) {
      contextPanel = settingContextRuntime.create({
        document,
        text: getUIText,
        locale: settingDocumentationRuntime.normalizeLanguage(LANG),
      });
      if (popularTopValues.length) {
        popularDetail = document.createElement("div");
        popularDetail.className = "setting-popular-detail-block";
        popularDetail.innerHTML = renderSettingPopularDetailHtml(p, popularEntry);
        contextPanel.setContent("popular", popularDetail);
      } else {
        contextPanel.setEmpty(
          "popular",
          getUIText("setting_context_popular_empty", "No popular-value data is available yet."),
        );
      }

      historyBlock = document.createElement("div");
      historyBlock.className = "setting-history-detail-block";
      historyFullBlock = document.createElement("div");
      historyFullBlock.className = "setting-history-detail-block";
      if (profile) {
        contextPanel.setEmpty(
          "history",
          getUIText("setting_context_history_empty", "No changes have been recorded yet."),
        );
      } else {
        contextPanel.setLoading(
          "history",
          getUIText("setting_context_loading", "Loading..."),
        );
      }
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
    if (name === "SoundLanguageSetting") {
      const sampleBtn = document.createElement("button");
      sampleBtn.type = "button";
      sampleBtn.className = "setting-default-reset";
      sampleBtn.textContent = getUIText("sound_sample_play", "Sample");
      sampleBtn.setAttribute("aria-label", getUIText("sound_sample_play", "Sample"));
      sampleBtn.onclick = async (event) => {
        event.stopPropagation();
        try {
          await playSoundLanguageSample(val.dataset.rawValue ?? p.default);
        } catch (e) {
          showAppToast(e?.message || getUIText("failed", "Failed"), { tone: "error" });
        }
      };
      actions.appendChild(sampleBtn);
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

    currentItemContainer.appendChild(el);
    if (contextPanel) {
      // The setting control and its supplementary information are independent
      // surfaces. Keep the context panel outside the setting card so the DOM
      // structure matches the visual hierarchy at every viewport size.
      itemsBox.appendChild(contextPanel.root);
      loadSettingDocumentationPanel(contextPanel, name, renderToken);
    }

    const cur = (name in values) ? values[name] : p.default;
    syncSettingControlState(el, cur);
    val.dataset.committedValue = String(cur);

    function normalizeSettingValue(raw) {
      const text = String(raw).trim();
      if (!text) return null;

      if (name === "SoundLanguageSetting") {
        const values = getSettingOptionValues(name, controlConfig);
        const matched = values.find((value) => String(value).toLowerCase() === text.toLowerCase());
        return matched || null;
      }

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

    async function refreshSettingHistory() {
      if (!contextPanel || !historyBlock || !historyFullBlock || profile) return;
      const runtimeHistory = carrotSettingsRuntime.history;
      if (!runtimeHistory) return;
      try {
        const payload = await getJson(
          `/api/param_changes?name=${encodeURIComponent(name)}`
          + `&limit=${settingContextRuntime.historyFetchLimit}`,
        );
        const changes = Array.isArray(payload.changes) ? payload.changes : [];
        const summaryChanges = changes.slice(0, settingContextRuntime.historyPreviewLimit);
        const renderOptions = {
          escape: escapeHtml,
          text: getUIText,
          locale: settingDocumentationRuntime.normalizeLanguage(LANG),
          formatValue: (value) => formatSettingDisplayValue(p, value),
        };
        historyBlock.innerHTML = runtimeHistory.renderHtml(summaryChanges, renderOptions);
        historyFullBlock.innerHTML = runtimeHistory.renderHtml(changes, renderOptions);
        contextPanel.setHistoryContent(
          historyBlock,
          changes.length > summaryChanges.length ? historyFullBlock : null,
          {
            count: changes.length,
            summaryCount: summaryChanges.length,
            emptyMessage: getUIText(
              "setting_context_history_empty",
              "No changes have been recorded yet.",
            ),
          },
        );
        bindSettingHistoryUndo();
      } catch (_) {
        // History is supplementary: a failed read must not disturb the screen.
        contextPanel.setEmpty(
          "history",
          getUIText("setting_context_history_empty", "No changes have been recorded yet."),
        );
      }
    }

    function bindSettingHistoryUndo() {
      contextPanel?.root.querySelectorAll("[data-setting-history-undo]").forEach((button) => {
        button.onclick = async (event) => {
          event.stopPropagation();
          const target = normalizeSettingValue(button.dataset.settingHistoryUndo);
          if (target === null) return;
          button.disabled = true;
          try {
            // The undo is itself a change, so it is recorded rather than
            // erasing the entry it reverses.
            await commitSettingValue(target, { source: "undo" });
          } finally {
            button.disabled = false;
          }
        };
      });
    }

    async function commitSettingValue(next, commitOptions = {}) {
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
          await setParam(name, next, commitOptions);
        }
        syncSettingControlState(el, next);
        val.dataset.committedValue = String(next);
        if (!profile) {
          cacheSettingValue(name, next, group);
          if (originGroup !== group) cacheSettingValue(name, next, originGroup);
          refreshSettingHistory();
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
    if (historyBlock) refreshSettingHistory();

    // The −/+ buttons run on the shared commit gesture: a press only records
    // where it started and the value is written on release, so a scroll that
    // happens to begin on a button is handed back to the browser untouched.
    // Step size still comes from the per-parameter x1 multiplier.
    function bindDeltaButtons() {
      if (!btnMinus && !btnPlus) return;
      window.CarrotUI.numericStepper.create({
        minusButton: btnMinus,
        plusButton: btnPlus,
        getValue: () => Number(val.dataset.rawValue),
        getFallback: () => Number(p.default),
        getStep: () => getSettingUnitValue(name),
        getRange: () => ({ min: Number(p.min), max: Number(p.max) }),
        commit: (next) => commitSettingValue(next),
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
      const choices = getSettingOptionValues(name, controlConfig).map((optionValue) => {
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

    bindDeltaButtons();

    val.onclick = (event) => {
      event.stopPropagation();
      // Tapping the value opens the number-entry popup directly (both in the
      // items list and the detail screen) — it no longer drills into the detail
      // screen. The detail screen is still reachable by tapping the row's title.
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

    // Reuse the app's segmented control for keyboard behaviour: roving
    // tabindex plus arrow / Home / End navigation, which these buttons never
    // had. No onActivate is passed on purpose — moving focus must not write a
    // value. Activation stays with the native click, which Enter and Space
    // already produce on a <button>.
    if (segmentGroup && segmentButtons.length) {
      window.CarrotUI?.segmentedControl?.create(segmentGroup, {
        itemSelector: ".setting-segment",
        selectedAttribute: "aria-pressed",
      });
    }

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
        if (isSettingValueControlHit(event, el)) return;
        if (isSettingInlineControlTarget(event.target)) return;
        selectSettingDetail(originGroup, name).catch(() => {});
      };
    }
  });

  itemsBox.dataset.renderedGroup = group;
  if (detailMode) itemsBox.dataset.renderedDetail = detailName;
  scheduleSettingOverflowSync(itemsBox);
  window.CarrotMapboxTokenSettings?.sync?.();
  window.CarrotYouTubeLiveSettings?.sync?.();
  syncSettingLiveRefresh();

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

  function isIgnoredFavoritePressTarget(event, row) {
    return isSettingInlineControlTarget(event.target) || isSettingValueControlHit(event, row);
  }

  itemsBox.addEventListener("pointerdown", (event) => {
    if (event.button !== undefined && event.button !== 0) return;
    const row = event.target.closest(".setting[data-setting-name]");
    if (!row || !itemsBox.contains(row) || isIgnoredFavoritePressTarget(event, row)) return;

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
  settingValueRepository.applyValues(values);
  applyRestoredSettingValuesToRenderedItems(values);

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


// ── Live value refresh ──────────────────────────────────────────────
// Parameters are not only written by this screen: the steering-wheel gap
// button changes MyDrivingMode and LongitudinalPersonality directly from the
// driving process (car/cruise.py). Without a refresh the settings screen keeps
// showing the cached value for the whole TTL, so the list can disagree with
// the device — and, now, with the change history right below it.
//
// Only the group currently on screen is re-read, and only while that screen is
// actually visible, so this costs one small bulk read every few seconds at
// most and nothing at all when the page is in the background.
const SETTING_LIVE_REFRESH_MS = 5000;
let settingLiveRefreshTimer = null;
let settingLiveRefreshInFlight = false;

function shouldRefreshSettingValues() {
  return (
    CURRENT_PAGE === "setting" &&
    isCarrotSettingTabActive() &&
    !document.hidden &&
    Boolean(CURRENT_GROUP) &&
    hasRenderedSettingItems(CURRENT_GROUP) &&
    !getSettingProfileByGroup(CURRENT_GROUP)
  );
}

function stopSettingLiveRefresh() {
  if (!settingLiveRefreshTimer) return;
  window.clearTimeout(settingLiveRefreshTimer);
  settingLiveRefreshTimer = null;
}

function scheduleSettingLiveRefresh(delay = SETTING_LIVE_REFRESH_MS) {
  stopSettingLiveRefresh();
  if (!shouldRefreshSettingValues()) return;
  settingLiveRefreshTimer = window.setTimeout(() => {
    settingLiveRefreshTimer = null;
    refreshSettingValuesFromDevice().catch(() => {});
  }, delay);
}

async function refreshSettingValuesFromDevice() {
  if (!shouldRefreshSettingValues() || settingLiveRefreshInFlight) {
    syncSettingLiveRefresh();
    return;
  }

  const group = CURRENT_GROUP;
  settingLiveRefreshInFlight = true;
  try {
    const values = await fetchSettingGroupValues(group, { force: true });
    // The user may have navigated while the read was in flight.
    if (CURRENT_GROUP === group && shouldRefreshSettingValues()) {
      applyRestoredSettingValuesToRenderedItems(values, { animate: false });
    }
  } finally {
    settingLiveRefreshInFlight = false;
    syncSettingLiveRefresh();
  }
}

function syncSettingLiveRefresh() {
  if (shouldRefreshSettingValues()) scheduleSettingLiveRefresh();
  else stopSettingLiveRefresh();
}

// Coming back to the tab is the moment a stale value is most likely and most
// visible, so refresh immediately rather than waiting out the interval.
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stopSettingLiveRefresh();
  else scheduleSettingLiveRefresh(0);
});
window.addEventListener("carrot:pagechange", () => scheduleSettingLiveRefresh(0));
syncSettingLiveRefresh();
