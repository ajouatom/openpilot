/* ---------- Home: current car ---------- */
let recordStateIsOn = false;
let recordTogglePending = false;
let recordStateResyncTimer = null;
let appViewportMetricsBound = false;

function updateAppViewportMetrics() {
  const vv = window.visualViewport;
  const height = Math.max(320, Math.round(vv?.height || window.innerHeight || 0));
  const top = Math.max(0, Math.round(vv?.offsetTop || 0));
  const width = Math.max(320, Math.round(vv?.width || window.innerWidth || 0));
  document.documentElement.style.setProperty("--app-vv-height", `${height}px`);
  document.documentElement.style.setProperty("--app-vv-top", `${top}px`);
  document.documentElement.style.setProperty("--app-vv-width", `${width}px`);
}

function bindAppViewportObservers() {
  if (appViewportMetricsBound) return;
  appViewportMetricsBound = true;

  const handleLayout = () => requestAnimationFrame(updateAppViewportMetrics);
  updateAppViewportMetrics();
  window.addEventListener("resize", handleLayout, { passive: true });
  window.addEventListener("orientationchange", handleLayout, { passive: true });
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", handleLayout, { passive: true });
    window.visualViewport.addEventListener("scroll", handleLayout, { passive: true });
  }
}

bindAppViewportObservers();

function parseRecordStateValue(value) {
  return (
    value === true ||
    value === 1 ||
    value === "1" ||
    value === "true" ||
    value === "True"
  );
}

function scheduleRecordStateResync(delay = 520) {
  if (recordStateResyncTimer) {
    clearTimeout(recordStateResyncTimer);
    recordStateResyncTimer = null;
  }
  recordStateResyncTimer = window.setTimeout(() => {
    recordStateResyncTimer = null;
    loadRecordState({ force: true }).catch(() => {});
  }, delay);
}

function applyRecordFabState(isOn) {
  recordStateIsOn = Boolean(isOn);
  if (!btnRecordToggle) return;

  btnRecordToggle.classList.toggle("active", recordStateIsOn);
  btnRecordToggle.textContent = recordStateIsOn ? "ON" : "OFF";
  if (typeof btnHome !== "undefined" && btnHome) {
    btnHome.classList.toggle("recording", recordStateIsOn);
    btnHome.setAttribute("data-record-badge", recordStateIsOn ? "REC" : "");
  }
  const label = recordStateIsOn
    ? (UI_STRINGS[LANG].record_on || UI_STRINGS[LANG].record || "Recording")
    : (UI_STRINGS[LANG].record_off || UI_STRINGS[LANG].record || "Idle");
  btnRecordToggle.setAttribute("aria-label", label);
  btnRecordToggle.title = label;
  if (typeof btnHome !== "undefined" && btnHome) {
    btnHome.setAttribute("aria-label", recordStateIsOn ? `${UI_STRINGS[LANG].home || "Home"} (${label})` : (UI_STRINGS[LANG].home || "Home"));
    btnHome.title = recordStateIsOn ? label : (UI_STRINGS[LANG].home || "Home");
  }
}

async function loadCurrentCar() {
  try {
    const values = await bulkGet(["CarSelected3"]);
    const v = values["CarSelected3"];
    curCarLabelCar.textContent = (v && String(v).trim().length) ? String(v) : "-";
    curCarLabelSetting.textContent = (v && String(v).trim().length) ? String(v) : "-";
  } catch (e) {
    curCarLabelCar.textContent = "-";
    curCarLabelSetting.textContent = "-";
  }
}

async function loadRecordState(options = {}) {
  if (recordTogglePending && !options.force) return;
  try {
    const values = await bulkGet(["ScreenRecord"]);
    applyRecordFabState(parseRecordStateValue(values["ScreenRecord"]));
  } catch (e) {
    applyRecordFabState(false);
  }
}
async function toggleRecord() {
  if (recordTogglePending) return;

  const prev = recordStateIsOn;
  const next = !prev;
  recordTogglePending = true;
  if (recordStateResyncTimer) {
    clearTimeout(recordStateResyncTimer);
    recordStateResyncTimer = null;
  }
  applyRecordFabState(next);

  try {
    await setParam("ScreenRecord", next);
    scheduleRecordStateResync();
  } catch (e) {
    applyRecordFabState(prev);
    showAppToast((UI_STRINGS[LANG].record || "Failed to toggle record: ") + e.message, { tone: "error" });
  } finally {
    recordTogglePending = false;
  }
}

/* ---------- Cars: load list + maker/model UI ---------- */
async function loadCars() {
  carMeta.textContent = "loading...";
  makerList.innerHTML = "";
  modelList.innerHTML = "";
  CURRENT_MAKER = null;
  showCarScreen("makers", false);

  const r = await fetch("/api/cars");
  const j = await r.json();
  if (!j.ok) {
    carMeta.textContent = "Failed: " + (j.error || "unknown");
    return;
  }
  CARS = j;

  const sources = (j.sources || []).join(", ");
  carMeta.textContent = sources ? ("sources: " + sources) : "ok";

  renderMakers();
}

function renderMakers() {
  makerList.innerHTML = "";
  const makers = CARS && CARS.makers ? Object.keys(CARS.makers) : [];
  makers.sort((a, b) => a.localeCompare(b));

  for (const mk of makers) {
    const arr = CARS.makers[mk] || [];
    const b = document.createElement("button");
    b.className = "btn groupBtn";
    b.textContent = `${mk} (${arr.length})`;
    b.onclick = () => {
      CURRENT_MAKER = mk;
      renderModels(mk);
      showCarScreen("models", true);
    };
    makerList.appendChild(b);
  }
}

function renderModels(maker) {
  modelList.innerHTML = "";
  const arr = (CARS.makers && CARS.makers[maker]) ? CARS.makers[maker] : [];
  modelTitle.textContent = maker;
  modelMeta.textContent = `${arr.length} models`;

  for (const fullLine of arr) {
    const modelOnly = stripMaker(fullLine, maker);

    const b = document.createElement("button");
    b.className = "btn groupBtn";
    b.textContent = modelOnly;
    b.onclick = () => onSelectCar(maker, modelOnly, fullLine);
    modelList.appendChild(b);
  }
}

function stripMaker(fullLine, maker) {
  const prefix = maker + " ";
  if (fullLine.startsWith(prefix)) return fullLine.slice(prefix.length).trim();
  const sp = fullLine.split(" ");
  if (sp.length >= 2) return sp.slice(1).join(" ").trim();
  return fullLine.trim();
}

async function onSelectCar(maker, modelOnly, fullLine) {
  const msg = (UI_STRINGS[LANG].confirm_car || "Select this car?") + `\n\n${maker} ${modelOnly}`;
  if (!await appConfirm(msg, { title: UI_STRINGS[LANG].car_select || "Car Select" })) return;

  try {
    await setParam("CarSelected3", fullLine);
  } catch (e) {
    await appAlert((UI_STRINGS[LANG].failed_set_car || "Failed to set car: ") + e.message, {
      title: UI_STRINGS[LANG].error || "Error",
    });
    return;
  }

  curCarLabelCar.textContent = modelOnly;
  curCarLabelSetting.textContent = modelOnly;

  const rb = await appConfirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?", {
    title: UI_STRINGS[LANG].reboot || "Reboot",
  });
  if (!rb) {
    showAppToast(UI_STRINGS[LANG].reboot_later || "Selected. Reboot later to apply.");
    return;
  }

  try {
    const r = await fetch("/api/reboot", { method: "POST" });
    const j = await r.json();
    if (!j.ok) throw new Error(j.error || "reboot failed");
    showAppToast(UI_STRINGS[LANG].rebooting || "Rebooting...", { tone: "success" });
  } catch (e) {
    await appAlert((UI_STRINGS[LANG].reboot_failed || "Reboot failed: ") + e.message, {
      title: UI_STRINGS[LANG].error || "Error",
    });
  }
}

/* ---------- Settings ---------- */
async function loadSettings() {
  const meta = document.getElementById("settingsMeta");
  meta.textContent = "loading...";

  const r = await fetch("/api/settings");
  const j = await r.json();
  if (!j.ok) {
    settingSearchEntries = [];
    meta.textContent = "Failed: " + (j.error || "unknown");
    return;
  }

  SETTINGS = j;
  UNIT_CYCLE = j.unit_cycle || UNIT_CYCLE;
  rebuildSettingSearchEntries();

  meta.textContent = `path: ${j.path} | has_params: ${j.has_params} | type_api: ${j.has_param_type}`;

  if (!DEBUG_UI) {
    meta.style.display = "none";
    const gm = document.getElementById("groupMeta");
    if (gm) gm.style.display = "none";
    const cm = document.getElementById("carMeta");
    if (cm) cm.style.display = "none";
  }

  renderGroups();
  renderSettingSubnav();
  CURRENT_GROUP = null;
  syncSettingSearchFabState();
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

function renderGroups() {
  const box = document.getElementById("groupList");
  box.innerHTML = "";

  (SETTINGS.groups || []).forEach(g => {
    const label = getSettingGroupLabel(g.group);

    const b = document.createElement("button");
    b.className = "btn groupBtn";
    if (g.group === CURRENT_GROUP) b.classList.add("active");
    b.textContent = `${label} (${g.count})`;
    b.onclick = () => selectGroup(g.group);
    box.appendChild(b);
  });
}

function getSettingGroupMeta(group) {
  const groups = SETTINGS?.groups || [];
  return groups.find((entry) => entry.group === group) || null;
}

function getSettingGroupLabel(group) {
  const meta = getSettingGroupMeta(group);
  if (!meta) return group;
  if (LANG === "zh") return meta.cgroup || meta.egroup || meta.group;
  if (LANG === "en") return meta.egroup || meta.group;
  return meta.group;
}

let settingSubnavSettleTimer = null;
let settingSubnavProgrammaticScroll = false;
let settingSubnavFocusTimer = null;
const SETTING_SUBNAV_PAGE_STEP = 1;
let settingGroupTransitionLock = false;
let settingRenderToken = 0;
let pendingSettingFocus = null;
let settingFocusClearTimer = null;
let settingSearchDebounceTimer = null;
let settingSearchEntries = [];
const settingPageRoot = document.getElementById("pageSetting");

function isCompactLandscapeMode() {
  return window.matchMedia("(orientation: landscape) and (max-height: 560px) and (pointer: coarse)").matches;
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
    btnSettingSearch.classList.toggle("active", isOpen);
    btnSettingSearch.setAttribute("aria-expanded", isOpen ? "true" : "false");
  }
}

function rebuildSettingSearchEntries() {
  const groups = SETTINGS?.groups || [];
  const entries = [];

  groups.forEach((groupMeta) => {
    const group = groupMeta.group;
    const groupLabel = getSettingGroupLabel(group);
    const list = SETTINGS?.items_by_group?.[group] || [];

    list.forEach((item) => {
      const title = formatItemText(item, "title", "etitle", "");
      const descr = formatItemText(item, "descr", "edescr", "");
      entries.push({
        group,
        groupLabel,
        name: item.name,
        title,
        descr,
        haystack: [groupLabel, item.name, title, descr].join("\n").toLowerCase(),
      });
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

function clearSettingItemFocus() {
  if (settingFocusClearTimer) {
    clearTimeout(settingFocusClearTimer);
    settingFocusClearTimer = null;
  }
  document.querySelectorAll(".setting.is-focus-hit").forEach((el) => el.classList.remove("is-focus-hit"));
}

const settingGroupScrollTops = new Map();

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

function syncSettingGroupChrome(group = CURRENT_GROUP) {
  const meta = document.getElementById("groupMeta");
  const list = SETTINGS?.items_by_group?.[group] || [];
  if (meta && group) meta.textContent = `${group} / ${list.length}`;
  const groupLabel = group ? getSettingGroupLabel(group) : "";
  if (group) {
    settingTitle.textContent = (UI_STRINGS[LANG].setting || "Setting") + " - " + groupLabel;
    if (itemsTitle) itemsTitle.textContent = groupLabel;
  }
}

function focusSettingItem(name, behavior = "smooth") {
  const itemsBox = document.getElementById("items");
  if (!itemsBox || !name) return false;

  const target = Array.from(itemsBox.querySelectorAll(".setting")).find(
    (el) => el.dataset.settingName === name,
  );
  if (!target) return false;

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
  const clear = Boolean(options.clear);
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

  if (clear && settingSearchInput) settingSearchInput.value = "";
  if (clear && settingSearchResults) settingSearchResults.innerHTML = "";
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
    .filter((entry) => entry.haystack.includes(q))
    .slice(0, 24);

  settingSearchResults.innerHTML = "";

  if (!matches.length) {
    const empty = document.createElement("div");
    empty.className = "setting-search-result setting-search-result--empty";
    empty.textContent = getUIText("setting_search_empty", "No matching settings found.");
    settingSearchResults.appendChild(empty);
    return;
  }

  matches.forEach((entry) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "setting-search-result";
    button.innerHTML = `
      <div class="setting-search-result__group">${highlightSettingSearchText(entry.groupLabel, trimmed)}</div>
      <div class="setting-search-result__title">${highlightSettingSearchText(entry.title || entry.name, trimmed)}</div>
      ${entry.name && entry.name !== entry.title ? `<div class="setting-search-result__name">${highlightSettingSearchText(entry.name, trimmed)}</div>` : ""}
      ${entry.descr ? `<div class="setting-search-result__descr">${highlightSettingSearchText(entry.descr, trimmed)}</div>` : ""}
    `;
    button.onclick = async () => {
      try {
        pendingSettingFocus = { group: entry.group, name: entry.name };
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
    settingSearchResults.appendChild(button);
  });
}

async function openSettingSearchPanel(options = {}) {
  const pushHistory = options.pushHistory !== false;
  if (CURRENT_PAGE !== "setting") return;
  if (!SETTINGS) {
    try {
      await loadSettings();
    } catch (_) {
      // no-op
    }
  }
  if (!settingSearchPanel) return;
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
    }, "");
  }
  syncModalBodyLock();
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
  btnSettingSearch.onclick = () => toggleSettingSearchPanel();
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
  }
});

function updateSettingSubnavLayoutState() {
  if (!settingSubnav || !settingSubnavWrap) return;

  const maxScrollLeft = Math.max(settingSubnav.scrollWidth - settingSubnav.clientWidth, 0);
  const isScrollable = maxScrollLeft > 4;
  settingSubnavWrap.classList.toggle("is-scrollable", isScrollable);
}

function getSettingSubnavGroups() {
  return SETTINGS?.groups || [];
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
  const nextGroup = group || CURRENT_GROUP;
  const previousGroup = CURRENT_GROUP;
  const scrollMode = options.scrollMode || "top";

  if (previousGroup && previousGroup !== nextGroup) {
    saveCurrentSettingScrollPosition(previousGroup);
  }

  CURRENT_GROUP = group;
  renderGroups();
  if (isCompactLandscapeMode() && CURRENT_PAGE === "setting") {
    showSettingScreen("items", false);
    history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
    await renderItems(group, {
      scrollMode,
      scrollTop: options.scrollTop,
    });
    return;
  }

  showSettingScreen("items", pushHistory);
  if (!pushHistory) {
    history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
  }
  await renderItems(group, {
    scrollMode,
    scrollTop: options.scrollTop,
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
  settingSubnav.innerHTML = "";

  const groups = SETTINGS?.groups || [];
  groups.forEach((entry) => {
    const button = document.createElement("button");
    button.className = "setting-subnav__tab";
    if (entry.group === CURRENT_GROUP) button.classList.add("is-active");
    button.dataset.group = entry.group;
    button.textContent = getSettingGroupLabel(entry.group);
    button.type = "button";
    button.onclick = () => selectGroup(entry.group, screenItems?.style.display === "none");
    settingSubnav.appendChild(button);
  });

  scheduleSettingSubnavFocus();
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
}

if (settingSubnavWrap) {
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
  activateSettingGroup(group, shouldPush).catch((e) => console.log("[Setting] selectGroup failed:", e));
}

async function renderItems(group, options = {}) {
  const meta = document.getElementById("groupMeta");
  const itemsBox = document.getElementById("items");
  const renderToken = ++settingRenderToken;
  const scrollMode = options.scrollMode || "top";
  const requestedScrollTop = Number.isFinite(options.scrollTop) ? options.scrollTop : null;
  itemsBox.innerHTML = "";
  delete itemsBox.dataset.renderedGroup;
  renderSettingSubnav();

  const list = SETTINGS.items_by_group[group] || [];
  if (meta) meta.textContent = `${group} / ${list.length}`;
  const groupLabel = getSettingGroupLabel(group);
  settingTitle.textContent = (UI_STRINGS[LANG].setting || "Setting") + " - " + groupLabel;
  if (itemsTitle) itemsTitle.textContent = groupLabel;

  const names = list.map(p => p.name);
  let values = {};
  try {
    values = await bulkGet(names);
  } catch (e) {
    values = {};
  }

  if (renderToken !== settingRenderToken || CURRENT_GROUP !== group || screenItems?.style.display === "none") {
    return;
  }

  for (const p of list) {
    const name = p.name;
    if (!(name in UNIT_INDEX)) UNIT_INDEX[name] = 0;

    const title = formatItemText(p, "title", "etitle", "");
    const descr = formatItemText(p, "descr", "edescr", "");

    const el = document.createElement("div");
    el.className = "setting";
    el.dataset.settingName = name;
    el.dataset.settingGroup = group;

    const top = document.createElement("div");
    top.className = "settingTop";

    const left = document.createElement("div");
    left.innerHTML = `
      <div class="title">${escapeHtml(title)}</div>
      <div class="name">${escapeHtml(name)}</div>
      <div class="muted mt-sm">
        min=${p.min}, max=${p.max}, default=${p.default}
      </div>
    `;

    const ctrl = document.createElement("div");
    ctrl.className = "ctrl";

    const btnMinus = document.createElement("button");
    btnMinus.className = "smallBtn";
    btnMinus.textContent = "-";

    const val = document.createElement("div");
    val.className = "pill val";

    const btnPlus = document.createElement("button");
    btnPlus.className = "smallBtn";
    btnPlus.textContent = "+";

    const unitBtn = document.createElement("button");
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
    itemsBox.appendChild(el);

    const cur = (name in values) ? values[name] : p.default;
    val.textContent = String(cur);

    async function applyDelta(sign) {
      const step = UNIT_CYCLE[UNIT_INDEX[name]];
      let curv = Number(val.textContent);
      if (Number.isNaN(curv)) curv = Number(p.default);

      let next = curv + sign * step;
      next = clamp(next, Number(p.min), Number(p.max));

      if (Number.isInteger(p.min) && Number.isInteger(p.max) && Number.isInteger(step)) {
        next = Math.round(next);
      }

      try {
        await setParam(name, next);
        val.textContent = String(next);
      } catch (e) {
        showAppToast((UI_STRINGS[LANG].set_failed || "set failed: ") + e.message, { tone: "error" });
      }
    }

    btnMinus.onclick = () => applyDelta(-1);
    btnPlus.onclick = () => applyDelta(+1);
  }

  itemsBox.dataset.renderedGroup = group;

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

async function syncSettingViewportLayout() {
  if (CURRENT_PAGE !== "setting" || !SETTINGS) return;
  syncSettingSearchFabState();
  renderGroups();
  renderSettingSubnav();

  if (isCompactLandscapeMode()) {
    const targetGroup = CURRENT_GROUP || getLandscapeDefaultSettingGroup();
    if (!targetGroup) return;
    CURRENT_GROUP = targetGroup;
    showSettingScreen("items", false);
    syncSettingGroupChrome(targetGroup);
    if (typeof centerActiveSettingSubnavTab === "function") centerActiveSettingSubnavTab("auto");
    if (!hasRenderedSettingItems(targetGroup)) {
      await renderItems(targetGroup, { scrollMode: "restore" });
    }
    return;
  }

  if (CURRENT_GROUP) {
    syncSettingGroupChrome(CURRENT_GROUP);
    showSettingScreen("items", false);
    if (typeof centerActiveSettingSubnavTab === "function") centerActiveSettingSubnavTab("auto");
    if (!hasRenderedSettingItems(CURRENT_GROUP)) {
      await renderItems(CURRENT_GROUP, { scrollMode: "restore" });
    }
  } else {
    showSettingScreen("groups", false);
  }
}

window.addEventListener("resize", () => {
  if (CURRENT_PAGE === "setting" && SETTINGS) {
    syncSettingViewportLayout().catch(() => {});
  }
}, { passive: true });

window.addEventListener("orientationchange", () => {
  if (CURRENT_PAGE === "setting" && SETTINGS) {
    syncSettingViewportLayout().catch(() => {});
  }
}, { passive: true });


/* ---------- Back key / history ---------- */
history.replaceState({ page: "home" }, "");

window.addEventListener("popstate", async (ev) => {
  const st = ev.state || { page: "home" };

  if (settingSearchPanel && !settingSearchPanel.hidden && !st.search) {
    closeSettingSearchPanel({ clear: false, fromHistory: true });
  }

  if (st.page === "home") {
    CURRENT_GROUP = null;
    CURRENT_MAKER = null;
    showPage("home", false);
    return;
  }

  if (st.page === "setting") {
    const screen = st.screen || "groups";
    CURRENT_GROUP = st.group || null;
    showPage("setting", false);

    if (isCompactLandscapeMode()) {
      const targetGroup = CURRENT_GROUP || getLandscapeDefaultSettingGroup();
      if (targetGroup) {
        CURRENT_GROUP = targetGroup;
        await activateSettingGroup(targetGroup, false, { scrollMode: "restore" });
      } else {
        showSettingScreen("groups", false);
      }
      if (st.search) {
        openSettingSearchPanel({ pushHistory: false }).catch(() => {});
      }
      return;
    }

    if (screen === "items" && CURRENT_GROUP) {
      showSettingScreen("items", false);
      renderItems(CURRENT_GROUP, { scrollMode: "restore" });
    } else {
      showSettingScreen("groups", false);
    }
    if (st.search) {
      openSettingSearchPanel({ pushHistory: false }).catch(() => {});
    }
    return;
  }

  if (st.page === "car") {
    showPage("car", false);
    if (!CARS) await loadCars();

    const screen = st.screen || "makers";
    CURRENT_MAKER = st.maker || null;

    if (screen === "models" && CURRENT_MAKER) {
      renderModels(CURRENT_MAKER);
      showCarScreen("models", false);
    } else {
      showCarScreen("makers", false);
    }
    return;
  }

  if (st.page == "tools") {
    showPage("tools", false);
    return;
  }

  if (st.page === "terminal") {
    showPage("terminal", false);
    return;
  }

  if (st.page === "carrot") {
    showPage("carrot", false);
    return;
  }

  if (st.page === "branch") {
    showPage("branch", false);
    if (!BRANCHES || !BRANCHES.length) {
      loadBranchesAndShow().catch(() => {});
    }
    return;
  }

});


let toolsOutHistory = "";
let toolsOutCurrentBlock = "";

function normalizeToolsOutText(s) {
  return String(s ?? "").replace(/\s+$/, "");
}

function renderToolsOut() {
  const out = document.getElementById("toolsOut");
  if (!out) return;
  const historyText = normalizeToolsOutText(toolsOutHistory);
  const currentText = normalizeToolsOutText(toolsOutCurrentBlock);

  if (!historyText && !currentText) {
    out.textContent = " ";
  } else {
    const frag = document.createDocumentFragment();

    if (historyText) {
      const historyBlock = document.createElement("span");
      historyBlock.className = "tools-console-log__history";
      historyBlock.textContent = historyText;
      frag.appendChild(historyBlock);
    }

    if (historyText && currentText) {
      frag.appendChild(document.createTextNode("\n\n"));
    }

    if (currentText) {
      const currentBlock = document.createElement("span");
      currentBlock.className = "tools-console-log__current";
      currentBlock.textContent = currentText;
      frag.appendChild(currentBlock);
    }

    out.replaceChildren(frag);
  }

  requestAnimationFrame(() => {
    out.scrollTop = out.scrollHeight;
  });
}

function toolsOutSet(s) {
  toolsOutCurrentBlock = normalizeToolsOutText(s);
  renderToolsOut();
}

function toolsOutAppend(s) {
  const next = normalizeToolsOutText(s);
  if (!next) return;
  toolsOutHistory = toolsOutHistory ? `${toolsOutHistory}\n\n${next}` : next;
  renderToolsOut();
}

function toolsOutCommitCurrent() {
  if (!toolsOutCurrentBlock) return;
  toolsOutAppend(toolsOutCurrentBlock);
  toolsOutCurrentBlock = "";
  renderToolsOut();
}

function getToolCommandPreview(action, payload = {}) {
  switch (action) {
    case "shell_cmd": return String(payload.cmd || "").trim() || "command";
    case "git_pull": return "git pull";
    case "git_sync": return "git sync";
    case "git_reset": return `git reset --${payload.mode || "hard"} ${payload.target || "HEAD"}`.trim();
    case "git_checkout": return `git checkout ${payload.branch || ""}`.trim();
    case "git_branch_list": return "change branch";
    case "send_tmux_log": return "capture tmux";
    case "server_tmux_log": return "send tmux";
    case "install_required": return "install flask";
    case "delete_all_videos": return "delete all videos";
    case "delete_all_logs": return "delete all logs";
    case "rebuild_all": return "rebuild all";
    case "backup_settings": return "backup settings";
    case "reboot": return "reboot";
    default: return action;
  }
}

function toolsMetaSet(s) {
  const meta = document.getElementById("toolsMeta");
  if (meta) meta.textContent = String(s);
}

function toolsProgressSet(percent = null, opts = {}) {
  const host = document.getElementById("toolsProgress");
  const bar = document.getElementById("toolsProgressBar");
  if (!host || !bar) return;

  const active = opts.active !== false;
  const indeterminate = Boolean(opts.indeterminate);
  if (!active) {
    host.hidden = true;
    host.classList.remove("is-indeterminate");
    bar.style.width = "0%";
    return;
  }

  host.hidden = false;
  host.classList.toggle("is-indeterminate", indeterminate);

  const hasPercent = Number.isFinite(percent);
  const safePercent = hasPercent ? Math.max(4, Math.min(100, Number(percent))) : 28;
  bar.style.width = `${safePercent}%`;
}

async function postJson(url, bodyObj) {
  const r = await fetch(url, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(bodyObj || {})
  });
  const j = await r.json().catch(() => ({}));
  if (!r.ok || !j.ok) {
    const msg = friendlyError(j) || j.error || ("HTTP " + r.status);
    throw new Error(msg);
  }
  return j;
}

async function getJson(url) {
  const r = await fetch(url);
  const j = await r.json().catch(() => ({}));
  if (!r.ok || !j.ok) {
    const msg = friendlyError(j) || j.error || ("HTTP " + r.status);
    throw new Error(msg);
  }
  return j;
}

function waitMs(ms) {
  return new Promise((resolve) => window.setTimeout(resolve, ms));
}

let activeToolRunToken = 0;

function updateToolsRunningState(labels, snapshot) {
  const message = String(snapshot?.message || labels.running || "");
  const stepCurrent = Number(snapshot?.step_current || 0);
  const stepTotal = Number(snapshot?.step_total || 0);
  const progressValue = Number(snapshot?.progress);
  const hasProgress = Number.isFinite(progressValue) && progressValue > 0;

  let metaText = message || labels.running;
  if (hasProgress && progressValue < 100) {
    metaText = `${metaText} · ${Math.round(progressValue)}%`;
  } else if (stepTotal > 1 && stepCurrent > 0) {
    metaText = `${metaText} · ${stepCurrent}/${stepTotal}`;
  }

  toolsMetaSet(metaText);
  toolsProgressSet(hasProgress ? progressValue : null, {
    active: true,
    indeterminate: !hasProgress || progressValue >= 100,
  });
}

async function runTool(action, payload) {
  const labels = getActionLabel(action);
  const runToken = ++activeToolRunToken;
  const commandPreview = getToolCommandPreview(action, payload || {});

  toolsMetaSet(labels.running);
  toolsOutCommitCurrent();
  toolsOutSet(`> ${commandPreview}\n${labels.running}`);
  toolsProgressSet(null, { active: true, indeterminate: true });

  const started = await postJson("/api/tools/start", { action, ...(payload || {}) });
  const jobId = started.job_id;
  let snapshot = null;

  while (runToken === activeToolRunToken) {
    snapshot = await getJson(`/api/tools/job?id=${encodeURIComponent(jobId)}`);

    if (snapshot.log != null) {
      const body = normalizeToolsOutText(snapshot.log) || labels.running;
      toolsOutSet(`> ${commandPreview}\n${body}`);
    }

    if (!snapshot.done) {
      updateToolsRunningState(labels, snapshot);
      await waitMs(320);
      continue;
    }

    const result = snapshot.result || snapshot;
    if (!result.ok) {
      const errMsg = friendlyError(result) || result.error || snapshot.error || labels.failed;
      toolsOutSet(`> ${commandPreview}\n${normalizeToolsOutText(snapshot?.log) || errMsg}`);
      toolsOutCommitCurrent();
      toolsMetaSet(labels.failed);
      toolsProgressSet(null, { active: false });
      throw new Error(errMsg);
    }

    const finalBody = normalizeToolsOutText(result.out ?? snapshot.log) || labels.done;
    toolsOutSet(`> ${commandPreview}\n${finalBody}`);
    toolsOutCommitCurrent();
    toolsMetaSet(labels.done);
    toolsProgressSet(100, { active: true, indeterminate: false });
    window.setTimeout(() => {
      if (activeToolRunToken === runToken) toolsProgressSet(null, { active: false });
    }, 900);
    return result;
  }

  throw new Error("tool run cancelled");
}

async function confirmText(msg, placeholder = "") {
  const v = await appPrompt(msg, {
    title: UI_STRINGS[LANG].input_title || "Input",
    defaultValue: placeholder,
    placeholder,
  });
  if (v === null) return null;
  return String(v).trim();
}

function showError(action, error) {
  const labels = getActionLabel(action);
  const title = labels.failed;
  const msg = (typeof error === "object" && error.message) ? error.message : String(error);
  toolsMetaSet(title);
  toolsProgressSet(null, { active: false });
  appAlert(msg, { title });
}

let branchPickerCloseTimer = null;

function openBranchPicker() {
  if (!appBranchPicker) return false;
  if (branchPickerCloseTimer) {
    clearTimeout(branchPickerCloseTimer);
    branchPickerCloseTimer = null;
  }
  appBranchPicker.hidden = false;
  syncModalBodyLock();
  requestAnimationFrame(() => {
    appBranchPicker.classList.add("is-open");
  });
  return true;
}

function closeBranchPicker(immediate = false) {
  if (!appBranchPicker || appBranchPicker.hidden) return;
  appBranchPicker.classList.remove("is-open");

  if (branchPickerCloseTimer) {
    clearTimeout(branchPickerCloseTimer);
    branchPickerCloseTimer = null;
  }

  const finishClose = () => {
    appBranchPicker.hidden = true;
    syncModalBodyLock();
  };

  if (immediate) {
    finishClose();
    return;
  }

  branchPickerCloseTimer = window.setTimeout(() => {
    branchPickerCloseTimer = null;
    finishClose();
  }, 180);
}

if (appBranchPickerBackdrop) appBranchPickerBackdrop.onclick = () => closeBranchPicker();
if (appBranchPickerClose) appBranchPickerClose.onclick = () => closeBranchPicker();


function initToolsPage() {
  const bindOnce = (id, fn) => {
    const el = document.getElementById(id);
    if (!el || el.dataset.bound === "1") return;
    el.dataset.bound = "1";
    el.onclick = fn;
  };

  const bindNodeOnce = (node, key, fn, eventName = "click") => {
    if (!node || node.dataset[key] === "1") return;
    node.dataset[key] = "1";
    node.addEventListener(eventName, fn);
  };

  const runSystemCommand = async () => {
    const inp = document.getElementById("sysCmdInput");
    const cmd = (inp?.value || "").trim();
    if (!cmd) return;

    try {
      await runTool("shell_cmd", { cmd });
    } catch (e) {
      showError("shell_cmd", e);
    }
  };

  toolsMetaSet(UI_STRINGS[LANG].ready || "Ready");
  toolsProgressSet(null, { active: false });

  bindOnce("btnGitPull", async () => {
    try {
      await runTool("git_pull");
    } catch (e) {
      showError("git_pull", e);
    }
  });

  bindOnce("btnGitSync", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].git_sync_confirm || "Run git sync?", { title: "git sync" })) return;
    try {
      await runTool("git_sync");
    } catch (e) {
      showError("git_sync", e);
    }
  });

  bindOnce("btnGitReset", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].git_reset_confirm || "Run git reset?", { title: "git reset" })) return;

    const mode = await confirmText(UI_STRINGS[LANG].git_reset_mode_prompt || "reset mode? (hard/soft/mixed)", "hard");
    if (!mode) return;

    const target = await confirmText(UI_STRINGS[LANG].git_reset_target_prompt || "reset target?", "HEAD");
    if (!target) return;

    try {
      await runTool("git_reset", { mode, target });
    } catch (e) {
      showError("git_reset", e);
    }
  });
  bindOnce("btnGitBranch", async () => {
    await loadBranchesAndShow();
  });


  bindOnce("btnSendTmuxLog", async () => {
    try {
      const j = await runTool("send_tmux_log");
      if (j.file) {
        window.location.href = j.file;
      }
    } catch (e) {
      showError("send_tmux_log", e);
    }
  });

  bindOnce("btnSendTmuxServerLog", async () => {
    try {
      const j = await runTool("server_tmux_log");
      if (j.file) {
        window.location.href = j.file;
      }
    } catch (e) {
      showError("server_tmux_log", e);
    }
  });

  bindOnce("btnInstallRequired", async () => {
    try {
      const j = await runTool("install_required");

      let summary = "";
      if (j.results && Array.isArray(j.results)) {
        const lines = j.results.map(r => `${r.package}: ${r.status}`);
        summary = lines.join("\n");
      }
      if (summary.trim()) toolsOutAppend(summary);

      if (j.need_reboot) {
        const yes = await appConfirm(UI_STRINGS[LANG].confirm_reboot_after_install, {
          title: UI_STRINGS[LANG].reboot || "Reboot",
        });
        if (yes) {
          await runTool("reboot");
        }
      }
    } catch (e) {
      showError("install_required", e);
    }
  });

  bindOnce("btnDeleteVideos", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].delete_videos_confirm || "Delete ALL videos?", { title: "delete videos" })) return;
    try {
      await runTool("delete_all_videos");
    } catch (e) {
      showError("delete_all_videos", e);
    }
  });

  bindOnce("btnDeleteLogs", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].delete_logs_confirm || "Delete ALL logs?", { title: "delete logs" })) return;
    try {
      await runTool("delete_all_logs");
    } catch (e) {
      showError("delete_all_logs", e);
    }
  });

  bindOnce("btnRebuildAll", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].rebuild_confirm || "Rebuild all?", { title: "Rebuild All" })) return;
    try {
      await runTool("rebuild_all");
    } catch (e) {
      showError("rebuild_all", e);
    }
  });

  bindOnce("btnBackupSettings", async () => {
    try {
      const j = await runTool("backup_settings");
      if (j.file) window.location.href = j.file;
    } catch (e) {
      showError("backup_settings", e);
    }
  });

  bindOnce("btnRestoreSettings", async () => {
    const inp = document.createElement("input");
    inp.type = "file";
    inp.accept = "application/json";
    inp.style.display = "none";

    inp.onchange = async () => {
      if (!inp.files || !inp.files[0]) return;

      if (!await appConfirm(UI_STRINGS[LANG].restore_confirm || "Restore settings from file?", {
        title: UI_STRINGS[LANG].restore || "Restore",
      })) {
        return;
      }

      try {
        const labels = getActionLabel("backup_settings");
        toolsMetaSet(labels.running);
        toolsOutCommitCurrent();
        toolsOutSet(`> restore settings\n${labels.running}`);
        toolsProgressSet(null, { active: true, indeterminate: true });

        const fd = new FormData();
        fd.append("file", inp.files[0]);

        const r = await fetch("/api/params_restore", { method: "POST", body: fd });
        const j = await r.json().catch(() => ({}));
        if (!r.ok || !j.ok) throw new Error(friendlyError(j) || j.error || ("HTTP " + r.status));

        toolsMetaSet(labels.done);
        toolsProgressSet(100, { active: true, indeterminate: false });
        toolsOutSet(`> restore settings\n${JSON.stringify(j.result, null, 2)}`);
        toolsOutCommitCurrent();

        if (await appConfirm(UI_STRINGS[LANG].restore_done_reboot || "Restore done.\nReboot now?", {
          title: UI_STRINGS[LANG].reboot || "Reboot",
        })) {
          await runTool("reboot");
        }
      } catch (e) {
        showError("backup_settings", e);
      } finally {
        window.setTimeout(() => toolsProgressSet(null, { active: false }), 900);
        inp.remove();
      }
    };

    document.body.appendChild(inp);
    inp.click();
  });

  bindOnce("btnReboot", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?", {
      title: UI_STRINGS[LANG].reboot || "Reboot",
    })) return;
    try {
      await runTool("reboot");
    } catch (e) {
      showError("reboot", e);
    }
  });

  const sysCmdForm = document.getElementById("sysCmdForm");
  const sysCmdInput = document.getElementById("sysCmdInput");
  bindNodeOnce(sysCmdForm, "submitBound", (ev) => {
    ev.preventDefault();
    runSystemCommand();
  }, "submit");

  const btnSysCmdInfo = document.getElementById("btnSysCmdInfo");
  const sysCmdHelpPanel = document.getElementById("sysCmdHelpPanel");
  const maybeShowSysCmdHint = () => {
    if (!sysCmdInput || !sysCmdHelpPanel || !sysCmdHelpPanel.hidden) return;
    const now = Date.now();
    const cooldownUntil = Number(sysCmdInput.dataset.hintCooldownUntil || "0");
    if (now < cooldownUntil) return;
    sysCmdInput.dataset.hintCooldownUntil = String(now + 3200);
    showAppToast(getUIText("sys_cmd_help", "Allowed: pull, status, branch, log, git ..., df, free, uptime"), {
      tone: "hint",
      duration: 1700,
    });
  };

  bindNodeOnce(sysCmdInput, "hintFocusBound", () => {
    if (!sysCmdInput.value.trim()) maybeShowSysCmdHint();
  }, "focus");

  bindNodeOnce(sysCmdInput, "hintInputBound", () => {
    if (sysCmdInput.value.trim().length === 1) maybeShowSysCmdHint();
  }, "input");

  bindNodeOnce(btnSysCmdInfo, "toggleBound", () => {
    if (!sysCmdHelpPanel) return;
    const nextOpen = sysCmdHelpPanel.hidden;
    sysCmdHelpPanel.hidden = !nextOpen;
    btnSysCmdInfo.setAttribute("aria-expanded", nextOpen ? "true" : "false");
  });
}

async function loadBranchesAndShow() {
  if (!appBranchPickerMeta || !appBranchPickerList || !openBranchPicker()) {
    showAppToast(UI_STRINGS[LANG].branch_dom_missing || "Branch DOM missing", { tone: "error" });
    return;
  }
  appBranchPickerMeta.textContent = "loading...";
  appBranchPickerList.innerHTML = "";
  BRANCHES = [];
  CURRENT_BRANCH_NAME = "";

  try {
    const j = await runTool("git_branch_list");
    BRANCHES = j.branches || [];
    CURRENT_BRANCH_NAME = (j.current_branch || "").trim();
    appBranchPickerMeta.textContent = `${BRANCHES.length} branches`;

    renderBranchList();
  } catch (e) {
    appBranchPickerMeta.textContent = e.message;
  }
}

function renderBranchList() {
  if (!appBranchPickerList) return;
  appBranchPickerList.innerHTML = "";

  if (!BRANCHES.length) {
    const empty = document.createElement("div");
    empty.className = "muted";
    empty.textContent = "-";
    appBranchPickerList.appendChild(empty);
    return;
  }

  for (const br of BRANCHES) {
    const b = document.createElement("button");
    b.className = "btn groupBtn app-branch-picker__item";
    if (CURRENT_BRANCH_NAME && br === CURRENT_BRANCH_NAME) {
      b.classList.add("is-current");
    }

    const label = document.createElement("span");
    label.className = "app-branch-picker__label";
    label.textContent = br;
    b.appendChild(label);

    if (CURRENT_BRANCH_NAME && br === CURRENT_BRANCH_NAME) {
      const badge = document.createElement("span");
      badge.className = "app-branch-picker__badge";
      badge.textContent = getUIText("branch_current", "Current");
      b.appendChild(badge);
    }

    b.onclick = () => onSelectBranch(br);
    appBranchPickerList.appendChild(b);
  }
}

async function onSelectBranch(branch) {
  closeBranchPicker(true);
  if (!await appConfirm((UI_STRINGS[LANG].checkout_confirm || "Switch to this branch?") + `\n\n${branch}`, {
    title: "git checkout",
  })) return;

  try {
    await runTool("git_checkout", { branch });
    showAppToast(UI_STRINGS[LANG].branch_changed || "Branch changed.", { tone: "success" });
  } catch (e) {
    showError("git_checkout", e);
    return;
  }

  const rb = await appConfirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?", {
    title: UI_STRINGS[LANG].reboot || "Reboot",
  });
  if (!rb) return;

  try {
    await runTool("reboot");
    showAppToast(UI_STRINGS[LANG].rebooting || "Rebooting...", { tone: "success" });
  } catch (e) {
    showError("reboot", e);
  }
}

/* ---------- Terminal ---------- */
const terminalMetaEl = document.getElementById("terminalMeta");
const terminalSessionMetaEl = document.getElementById("terminalSessionMeta");
const terminalScreenEl = document.getElementById("terminalScreen");
const terminalOutputEl = document.getElementById("terminalOutput");
const terminalFormEl = document.getElementById("terminalForm");
const terminalInputEl = document.getElementById("terminalInput");
const btnTerminalCtrlCEl = document.getElementById("btnTerminalCtrlC");
const btnTerminalClearEl = document.getElementById("btnTerminalClear");
const btnTerminalReconnectEl = document.getElementById("btnTerminalReconnect");

let terminalWs = null;
let terminalReconnectTimer = null;
let terminalPageActive = false;
let terminalSessionName = "carrot-web";
let terminalLastScreen = "";
let terminalLayoutBound = false;
let terminalFollowOutput = true;
let terminalCurrentCwd = "/data/openpilot";

function setTerminalMeta(text) {
  if (terminalMetaEl) terminalMetaEl.textContent = String(text || "");
}

function setTerminalSessionMeta(cwd = terminalCurrentCwd) {
  if (!terminalSessionMetaEl) return;
  terminalSessionMetaEl.textContent = String(cwd || "/data/openpilot");
}

function setTerminalSessionInfo(session = terminalSessionName) {
  terminalSessionName = session || terminalSessionName;
  setTerminalSessionMeta();
}

function sanitizeTerminalScreen(text) {
  let nextText = String(text || " ");
  const headLimit = Math.min(nextText.length, 640);
  const head = nextText.slice(0, headLimit);
  const sanitizedHead = head.replace(
    /[^\n]*\$\s*cd(?:\s+\/data\/openpilot)?\n(?:\/data\/openpilot\n)?(?=[^\n]*:\/data\/openpilot\$)/,
    "",
  );

  if (sanitizedHead !== head) {
    nextText = sanitizedHead + nextText.slice(headLimit);
    nextText = nextText.replace(/^\n+/, "");
  }

  const lines = nextText.replace(/\r/g, "").split("\n");
  while (lines.length > 1 && !lines[lines.length - 1].trim()) {
    lines.pop();
  }
  nextText = lines.join("\n");

  if (!nextText.trim()) return " ";
  return nextText;
}

function extractTerminalCwd(text) {
  const lines = String(text || "").split("\n");
  for (let i = lines.length - 1; i >= 0; i -= 1) {
    const line = lines[i].trim();
    if (!line) continue;
    const match = line.match(/^[^\s:@]+@[^\s:]+:(.+?)[#$]\s*$/);
    if (match) return match[1].trim();
  }
  return "";
}

function renderTerminalScreenMarkup(text) {
  return String(text || " ")
    .split("\n")
    .map((line) => {
      const match = line.match(/^([^\s:@]+@[^\s:]+)(?=:[^$]*\$ ?)/);
      if (!match) return escapeHtml(line);
      const promptHost = match[1];
      return `<span class="terminal-output__promptHost">${escapeHtml(promptHost)}</span>${escapeHtml(line.slice(promptHost.length))}`;
    })
    .join("\n");
}

function isTerminalPinnedToBottom() {
  if (!terminalScreenEl) return true;
  return (terminalScreenEl.scrollHeight - terminalScreenEl.scrollTop - terminalScreenEl.clientHeight) < 28;
}

function pinTerminalToBottom() {
  if (!terminalScreenEl) return;
  requestAnimationFrame(() => {
    terminalScreenEl.scrollTop = terminalScreenEl.scrollHeight;
  });
}

function updateTerminalOverflowState() {
  if (!terminalScreenEl || !terminalOutputEl) return;
  const overflowX = (terminalOutputEl.scrollWidth - terminalScreenEl.clientWidth) > 20;
  const atRight = (terminalScreenEl.scrollWidth - terminalScreenEl.scrollLeft - terminalScreenEl.clientWidth) < 8;
  terminalScreenEl.classList.toggle("is-x-overflow", overflowX && !atRight);
}

function clearTerminalViewport() {
  terminalLastScreen = "";
  if (terminalOutputEl) terminalOutputEl.innerHTML = "";
  updateTerminalOverflowState();
  pinTerminalToBottom();
}

function setTerminalScreen(text, forceStick = false) {
  if (!terminalOutputEl) return;
  const nextText = sanitizeTerminalScreen(text);
  if (nextText === terminalLastScreen) return;

  const shouldStick = forceStick || terminalFollowOutput || isTerminalPinnedToBottom();
  terminalLastScreen = nextText;
  const nextCwd = extractTerminalCwd(nextText);
  if (nextCwd && nextCwd !== terminalCurrentCwd) {
    terminalCurrentCwd = nextCwd;
    setTerminalSessionMeta(nextCwd);
  }
  terminalOutputEl.innerHTML = renderTerminalScreenMarkup(nextText);
  requestAnimationFrame(updateTerminalOverflowState);

  if (shouldStick) pinTerminalToBottom();
}

function clearTerminalReconnectTimer() {
  if (terminalReconnectTimer) {
    clearTimeout(terminalReconnectTimer);
    terminalReconnectTimer = null;
  }
}

function updateTerminalToastAnchor() {
  if (!terminalFormEl || document.body?.dataset?.page !== "terminal") {
    document.documentElement.style.removeProperty("--terminal-toast-bottom");
    document.documentElement.style.removeProperty("--terminal-toast-left");
    document.documentElement.style.removeProperty("--terminal-toast-width");
    return;
  }

  const rect = terminalFormEl.getBoundingClientRect();
  if (!rect.width || !rect.height) {
    document.documentElement.style.removeProperty("--terminal-toast-bottom");
    document.documentElement.style.removeProperty("--terminal-toast-left");
    document.documentElement.style.removeProperty("--terminal-toast-width");
    return;
  }
  const gap = 10;
  const offset = Math.max(0, Math.round(window.innerHeight - rect.top + gap));
  document.documentElement.style.setProperty("--terminal-toast-bottom", `${offset}px`);
  document.documentElement.style.setProperty("--terminal-toast-left", `${Math.round(rect.left)}px`);
  document.documentElement.style.setProperty("--terminal-toast-width", `${Math.round(rect.width)}px`);
}

function updateTerminalViewportMetrics() {
  updateAppViewportMetrics();
  const vv = window.visualViewport;
  const height = Math.max(320, Math.round(vv?.height || window.innerHeight || 0));
  const top = (typeof isLandscapeRailMode === "function" && isLandscapeRailMode())
    ? Math.max(0, Math.round(vv?.offsetTop || 0))
    : 0;
  const keyboardInset = Math.max(0, Math.round((window.innerHeight || 0) - height - top));
  const keyboardOpen = !(typeof isLandscapeRailMode === "function" && isLandscapeRailMode()) && keyboardInset > 120;
  document.documentElement.style.setProperty("--terminal-vv-height", `${height}px`);
  document.documentElement.style.setProperty("--terminal-vv-top", `${top}px`);
  document.documentElement.style.setProperty(
    "--terminal-bottom-gap",
    keyboardOpen
      ? `calc(6px + env(safe-area-inset-bottom, 0px))`
      : `calc(var(--nav-bar-height) + env(safe-area-inset-bottom, 0px))`,
  );
}

function bindTerminalLayoutObservers() {
  if (terminalLayoutBound) return;
  terminalLayoutBound = true;

  const handleLayout = () => requestAnimationFrame(() => {
    updateTerminalViewportMetrics();
    updateTerminalToastAnchor();
    updateTerminalOverflowState();
  });
  window.addEventListener("resize", handleLayout, { passive: true });
  window.addEventListener("orientationchange", handleLayout, { passive: true });
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", handleLayout, { passive: true });
    window.visualViewport.addEventListener("scroll", handleLayout, { passive: true });
  }
}

function closeTerminalSocket() {
  if (!terminalWs) return;
  const ws = terminalWs;
  terminalWs = null;
  try {
    ws.onopen = null;
    ws.onmessage = null;
    ws.onclose = null;
    ws.onerror = null;
    if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) ws.close();
  } catch (e) {
    console.log("[Terminal] ws close failed:", e);
  }
}

function getTerminalWsUrl() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}/ws/terminal?session=${encodeURIComponent(terminalSessionName)}`;
}

function scheduleTerminalReconnect(delay = 1200) {
  clearTerminalReconnectTimer();
  if (!terminalPageActive) return;
  setTerminalMeta(getUIText("reconnecting", "reconnecting..."));
  terminalReconnectTimer = window.setTimeout(() => {
    terminalReconnectTimer = null;
    connectTerminal();
  }, delay);
}

function sendTerminalPacket(payload, options = {}) {
  const { quiet = false } = options;
  if (!terminalWs || terminalWs.readyState !== WebSocket.OPEN) {
    if (!quiet) showAppToast(getUIText("terminal_offline", "terminal offline"), { tone: "error" });
    return false;
  }

  try {
    terminalWs.send(JSON.stringify(payload));
    return true;
  } catch (e) {
    if (!quiet) showAppToast(e.message || "Terminal send failed", { tone: "error" });
    return false;
  }
}

function sendTerminalControl(action, options = {}) {
  return sendTerminalPacket({ type: "control", action }, options);
}

function connectTerminal(force = false) {
  clearTerminalReconnectTimer();

  if (terminalWs && (terminalWs.readyState === WebSocket.OPEN || terminalWs.readyState === WebSocket.CONNECTING)) {
    if (!force) return;
    closeTerminalSocket();
  }

  setTerminalMeta(getUIText("connecting", "connecting..."));

  let ws;
  try {
    ws = new WebSocket(getTerminalWsUrl());
  } catch (e) {
    setTerminalMeta(e.message || getUIText("terminal_unavailable", "terminal unavailable"));
    scheduleTerminalReconnect(1600);
    return;
  }

  terminalWs = ws;

  ws.onopen = () => {
    if (terminalWs !== ws) return;
    setTerminalMeta(getUIText("connecting", "connecting..."));
  };

  ws.onmessage = (ev) => {
    if (terminalWs !== ws) return;

    let data;
    try {
      data = JSON.parse(ev.data);
    } catch (e) {
      return;
    }

    if (data.type === "meta") {
      setTerminalSessionInfo(data.session || terminalSessionName);
      setTerminalMeta(data.created ? getUIText("terminal_ready", "tmux ready") : getUIText("connected", "connected"));
      return;
    }

    if (data.type === "screen") {
      setTerminalScreen(data.text, false);
      if (terminalMetaEl && terminalMetaEl.textContent === getUIText("connecting", "connecting...")) {
        setTerminalMeta(getUIText("connected", "connected"));
      }
      return;
    }

    if (data.type === "error") {
      const errorText = String(data.error || getUIText("error", "Error"));
      setTerminalMeta(errorText);
      showAppToast(errorText, { tone: "error" });
    }
  };

  ws.onclose = () => {
    if (terminalWs !== ws) return;
    terminalWs = null;
    if (!terminalPageActive) return;
    setTerminalMeta(getUIText("terminal_disconnected", "disconnected"));
    scheduleTerminalReconnect(1250);
  };

  ws.onerror = () => {
    if (terminalWs !== ws) return;
    setTerminalMeta(getUIText("terminal_unavailable", "terminal unavailable"));
  };
}

function initTerminalBindings() {
  const bindNodeOnce = (node, key, fn, eventName = "click") => {
    if (!node || node.dataset[key] === "1") return;
    node.dataset[key] = "1";
    node.addEventListener(eventName, fn);
  };

  bindTerminalLayoutObservers();

  bindNodeOnce(terminalScreenEl, "scrollBound", () => {
    terminalFollowOutput = isTerminalPinnedToBottom();
    updateTerminalOverflowState();
  }, "scroll");

  bindNodeOnce(terminalFormEl, "submitBound", (ev) => {
    ev.preventDefault();
    const line = (terminalInputEl?.value || "").trim();
    if (!line) return;
    terminalFollowOutput = true;
    pinTerminalToBottom();
    if (sendTerminalPacket({ type: "input", data: line })) {
      terminalInputEl.value = "";
    }
  }, "submit");

  bindNodeOnce(btnTerminalCtrlCEl, "clickBound", () => {
    terminalFollowOutput = true;
    pinTerminalToBottom();
    sendTerminalControl("ctrl_c");
  });

  bindNodeOnce(btnTerminalClearEl, "clickBound", () => {
    terminalFollowOutput = true;
    clearTerminalViewport();
    sendTerminalControl("clear");
  });

  bindNodeOnce(btnTerminalReconnectEl, "clickBound", () => {
    terminalFollowOutput = true;
    pinTerminalToBottom();
    const metaText = String(terminalMetaEl?.textContent || "");
    const blockedByPassword = terminalLastScreen.includes("Password:");
    const blockedByStartup = metaText.includes("returned non-zero exit status");
    if ((blockedByPassword || blockedByStartup) && sendTerminalControl("new_session", { quiet: true })) {
      setTerminalMeta(getUIText("connecting", "connecting..."));
      return;
    }
    connectTerminal(true);
  });
}

function initTerminalPage() {
  terminalPageActive = true;
  terminalFollowOutput = true;
  terminalCurrentCwd = "/data/openpilot";
  initTerminalBindings();
  setTerminalSessionMeta();
  updateTerminalViewportMetrics();
  if (!terminalLastScreen) setTerminalScreen(" ", true);
  requestAnimationFrame(updateTerminalToastAnchor);
  requestAnimationFrame(updateTerminalOverflowState);
  window.setTimeout(updateTerminalToastAnchor, 90);
  connectTerminal(false);
}

function teardownTerminalPage() {
  terminalPageActive = false;
  clearTerminalReconnectTimer();
  closeTerminalSocket();
  document.documentElement.style.removeProperty("--terminal-vv-height");
  document.documentElement.style.removeProperty("--terminal-vv-top");
  document.documentElement.style.removeProperty("--terminal-bottom-gap");
  document.documentElement.style.removeProperty("--terminal-toast-bottom");
  document.documentElement.style.removeProperty("--terminal-toast-left");
  document.documentElement.style.removeProperty("--terminal-toast-width");
}
