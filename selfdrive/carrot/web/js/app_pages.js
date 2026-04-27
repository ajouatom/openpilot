/* ---------- Shared runtime ---------- */
let recordStateIsOn = false;
let recordTogglePending = false;
let recordStateResyncTimer = null;
let appViewportMetricsBound = false;
const CURRENT_CAR_CACHE_KEY = "carrot_web_current_car_label";
const CURRENT_CAR_RETRY_DELAYS_MS = [350, 800, 1500, 2500, 4000];
const PAGE_DATA_TTL_MS = 15000;
let currentCarRetryTimer = null;
let currentCarRetryIndex = 0;
let currentCarLastKnownLabel = "";
let currentCarLoadPromise = null;
let currentCarLoadedAt = 0;
let currentCarHasSnapshot = false;
let recordStateLoadPromise = null;
let recordStateLoadedAt = 0;
let carsLoadPromise = null;
let settingsLoadPromise = null;
let toolsMetaLoadPromise = null;
let toolsMetaLoadedAt = 0;
let toolsMetaLastValues = null;
let uiWarmupTimer = null;
const SETTING_VALUES_TTL_MS = 60000;
let settingValueWarmupTimer = null;
let settingValueWarmupPromise = null;
const settingValueCache = new Map();
const settingGroupValueCache = new Map();
const settingGroupValuePromises = new Map();

let ORIGIN_USERNAME = "origin";
let BRANCH_REMOTE_NAMES = ["origin"];
let BRANCH_REMOTE_OWNERS = Object.create(null);
let BRANCH_GROUP_OPEN = Object.create(null);

function hasFreshPageData(lastLoadedAt, ttlMs = PAGE_DATA_TTL_MS) {
  return Number.isFinite(lastLoadedAt) && lastLoadedAt > 0 && (Date.now() - lastLoadedAt) < ttlMs;
}

function requestIdleTask(callback, timeout = 900) {
  if (typeof window.requestIdleCallback === "function") {
    return window.requestIdleCallback(callback, { timeout });
  }
  return window.setTimeout(callback, Math.min(timeout, 180));
}

function getSettingGroupParamNames(group) {
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

async function fetchSettingGroupValues(group, options = {}) {
  if (!group) return {};
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

function updateSettingCarEntryState(label) {
  if (!settingCarRow) return;
  const text = String(label || "").trim();
  const isEmpty = !text || text === "-";
  settingCarRow.classList.toggle("is-empty", isEmpty);
  settingCarRow.setAttribute("aria-label", isEmpty ? "차량 선택 열기" : `${text} 차량 선택 열기`);
}

function updateAppViewportMetrics() {
  const vv = window.visualViewport;
  const height = Math.max(320, Math.round(vv?.height || window.innerHeight || 0));
  const top = Math.max(0, Math.round(vv?.offsetTop || 0));
  const width = Math.max(320, Math.round(vv?.width || window.innerWidth || 0));
  document.documentElement.style.setProperty("--app-vv-height", `${height}px`);
  document.documentElement.style.setProperty("--app-vv-top", `${top}px`);
  document.documentElement.style.setProperty("--app-vv-width", `${width}px`);

  const topbarEl = document.querySelector(".topbar");
  let navLeftGap = 0;
  let navBottomGap = 0;
  if (topbarEl) {
    const styles = window.getComputedStyle(topbarEl);
    if (styles.display !== "none" && styles.visibility !== "hidden") {
      const rect = topbarEl.getBoundingClientRect();
      const rectWidth = Math.max(0, Math.round(rect.width));
      const rectHeight = Math.max(0, Math.round(rect.height));
      const isRailLayout =
        rectWidth > 0 &&
        rectHeight > 0 &&
        rectHeight >= Math.max(Math.round(rectWidth * 1.25), Math.round(height * 0.5)) &&
        rectWidth <= Math.max(160, Math.round(width * 0.35));

      if (isRailLayout) {
        navLeftGap = rectWidth;
      } else if (rectHeight > 0) {
        const visibleTop = Math.max(0, Math.round(rect.top));
        const visibleBottom = Math.min(height, Math.round(rect.bottom));
        navBottomGap = Math.max(0, visibleBottom - visibleTop);
      }
    }
  }

  document.documentElement.style.setProperty("--app-nav-left-gap", `${navLeftGap}px`);
  document.documentElement.style.setProperty("--app-nav-bottom-gap", `${navBottomGap}px`);
}

function bindAppViewportObservers() {
  if (appViewportMetricsBound) return;
  appViewportMetricsBound = true;

  const handleLayout = () => requestAnimationFrame(updateAppViewportMetrics);
  updateAppViewportMetrics();
  window.addEventListener("resize", handleLayout, { passive: true });
  window.addEventListener("orientationchange", handleLayout, { passive: true });
  document.addEventListener("fullscreenchange", handleLayout);
  document.addEventListener("webkitfullscreenchange", handleLayout);
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", handleLayout, { passive: true });
    window.visualViewport.addEventListener("scroll", handleLayout, { passive: true });
  }
}

bindAppViewportObservers();

const driveHudCardEl = document.getElementById("driveHudCard");
let driveHudLayoutObserversBound = false;
let driveHudLayoutRaf = 0;

function syncDriveHudLayout() {
  driveHudLayoutRaf = 0;
  if (!driveHudCardEl || !window.DrivingHud) return;
  window.DrivingHud.relayout();
}

function scheduleDriveHudLayout() {
  if (driveHudLayoutRaf) return;
  driveHudLayoutRaf = requestAnimationFrame(syncDriveHudLayout);
}

function bindDriveHudLayoutObservers() {
  if (driveHudLayoutObserversBound) return;
  driveHudLayoutObserversBound = true;

  const handleLayout = () => scheduleDriveHudLayout();
  window.addEventListener("resize", handleLayout, { passive: true });
  window.addEventListener("orientationchange", handleLayout, { passive: true });
  window.addEventListener("carrot:pagechange", handleLayout);
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", handleLayout, { passive: true });
    window.visualViewport.addEventListener("scroll", handleLayout, { passive: true });
  }

  scheduleDriveHudLayout();
}

bindDriveHudLayoutObservers();

function applyCurrentCarLabel(label, { persist = true, blank = false } = {}) {
  const text = String(label || "").trim();
  if (text) {
    currentCarLastKnownLabel = text;
    if (curCarLabelCar) curCarLabelCar.textContent = text;
    if (curCarLabelSetting) curCarLabelSetting.textContent = text;
    updateSettingCarEntryState(text);
    if (persist) {
      try {
        localStorage.setItem(CURRENT_CAR_CACHE_KEY, text);
      } catch {}
    }
    return;
  }

  if (currentCarLastKnownLabel) {
    if (curCarLabelCar) curCarLabelCar.textContent = currentCarLastKnownLabel;
    if (curCarLabelSetting) curCarLabelSetting.textContent = currentCarLastKnownLabel;
    updateSettingCarEntryState(currentCarLastKnownLabel);
    return;
  }

  if (blank) {
    if (curCarLabelCar) curCarLabelCar.textContent = "-";
    if (curCarLabelSetting) curCarLabelSetting.textContent = "-";
    updateSettingCarEntryState("-");
  }
}

function restoreCurrentCarLabelFromCache() {
  try {
    const cached = localStorage.getItem(CURRENT_CAR_CACHE_KEY);
    if (cached && String(cached).trim()) {
      applyCurrentCarLabel(cached, { persist: false });
    }
  } catch {}
}

function cancelCurrentCarRetry() {
  if (!currentCarRetryTimer) return;
  clearTimeout(currentCarRetryTimer);
  currentCarRetryTimer = null;
}

function scheduleCurrentCarRetry() {
  if (currentCarRetryTimer || currentCarRetryIndex >= CURRENT_CAR_RETRY_DELAYS_MS.length) return;
  const delay = CURRENT_CAR_RETRY_DELAYS_MS[currentCarRetryIndex++];
  currentCarRetryTimer = window.setTimeout(() => {
    currentCarRetryTimer = null;
    loadCurrentCar({ resetRetry: false }).catch(() => {});
  }, delay);
}

function resolveCurrentCarLabel(values) {
  const selected = String(values?.CarSelected3 || "").trim();
  if (selected) return selected;
  const effective = String(values?.CarName || "").trim();
  if (effective) return effective;
  return "";
}

restoreCurrentCarLabelFromCache();
updateSettingCarEntryState(curCarLabelSetting?.textContent || curCarLabelCar?.textContent || "-");

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
  btnRecordToggle.textContent = "REC";
  btnRecordToggle.dataset.state = recordStateIsOn ? "on" : "off";
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

async function loadCurrentCar(options = {}) {
  const resetRetry = options.resetRetry !== false;
  const force = options.force === true;
  const ttlMs = Number.isFinite(options.ttlMs) ? options.ttlMs : PAGE_DATA_TTL_MS;
  if (resetRetry) {
    currentCarRetryIndex = 0;
    cancelCurrentCarRetry();
  }
  if (!force && currentCarLoadPromise) return currentCarLoadPromise;
  if (!force && currentCarHasSnapshot && hasFreshPageData(currentCarLoadedAt, ttlMs)) {
    return currentCarLastKnownLabel;
  }

  currentCarLoadPromise = (async () => {
    try {
      const values = await bulkGet(["CarSelected3", "CarName"]);
      const label = resolveCurrentCarLabel(values);
      if (label) {
        cancelCurrentCarRetry();
        currentCarRetryIndex = 0;
        applyCurrentCarLabel(label);
      } else {
        applyCurrentCarLabel("", { blank: !currentCarLastKnownLabel });
        scheduleCurrentCarRetry();
      }
      currentCarHasSnapshot = true;
      currentCarLoadedAt = Date.now();
      return label;
    } catch (e) {
      applyCurrentCarLabel("", { blank: !currentCarLastKnownLabel });
      scheduleCurrentCarRetry();
      throw e;
    } finally {
      currentCarLoadPromise = null;
    }
  })();

  return currentCarLoadPromise;
}

window.addEventListener("pageshow", () => {
  loadCurrentCar({ resetRetry: true }).catch(() => {});
  scheduleUiWarmup(90);
});

document.addEventListener("visibilitychange", () => {
  if (!document.hidden) {
    loadCurrentCar({ resetRetry: true, force: true }).catch(() => {});
    scheduleUiWarmup(70);
  }
});

window.addEventListener("online", () => {
  scheduleUiWarmup(40);
});

async function loadRecordState(options = {}) {
  const force = options.force === true;
  const ttlMs = Number.isFinite(options.ttlMs) ? options.ttlMs : PAGE_DATA_TTL_MS;
  if (recordTogglePending && !force) return;
  if (!force && recordStateLoadPromise) return recordStateLoadPromise;
  if (!force && hasFreshPageData(recordStateLoadedAt, ttlMs)) return recordStateIsOn;

  recordStateLoadPromise = (async () => {
    try {
      const values = await bulkGet(["ScreenRecord"]);
      applyRecordFabState(parseRecordStateValue(values["ScreenRecord"]));
      recordStateLoadedAt = Date.now();
      return recordStateIsOn;
    } catch (e) {
      applyRecordFabState(false);
      throw e;
    } finally {
      recordStateLoadPromise = null;
    }
  })();

  return recordStateLoadPromise;
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
let carPickerCloseTimer = null;
let carPickerMode = "makers";
let carPickerMaker = null;

function openCarPicker() {
  if (!appCarPicker) return false;
  if (carPickerCloseTimer) {
    clearTimeout(carPickerCloseTimer);
    carPickerCloseTimer = null;
  }
  appCarPicker.hidden = false;
  syncModalBodyLock();
  requestAnimationFrame(() => {
    appCarPicker.classList.add("is-open");
  });
  return true;
}

function closeCarPicker(immediate = false) {
  if (!appCarPicker || appCarPicker.hidden) return;
  appCarPicker.classList.remove("is-open");

  if (carPickerCloseTimer) {
    clearTimeout(carPickerCloseTimer);
    carPickerCloseTimer = null;
  }

  const finishClose = () => {
    appCarPicker.hidden = true;
    syncModalBodyLock();
  };

  if (immediate) {
    finishClose();
    return;
  }

  carPickerCloseTimer = window.setTimeout(() => {
    carPickerCloseTimer = null;
    finishClose();
  }, 180);
}

function syncCarPickerChrome() {
  if (!appCarPickerTitle || !appCarPickerMeta || !appCarPickerClose) return;
  const currentLabel = String(curCarLabelSetting?.textContent || curCarLabelCar?.textContent || "").trim() || "-";
  if (carPickerMode === "models" && carPickerMaker) {
    appCarPickerTitle.textContent = carPickerMaker;
    const arr = (CARS?.makers && CARS.makers[carPickerMaker]) ? CARS.makers[carPickerMaker] : [];
    appCarPickerMeta.textContent = `${arr.length} ${getUIText("models", "Models")}`;
    appCarPickerClose.textContent = getUIText("back", "Back");
    return;
  }
  appCarPickerTitle.textContent = getUIText("car_select", "Car Select");
  appCarPickerMeta.textContent = currentLabel;
  appCarPickerClose.textContent = getUIText("cancel", "Cancel");
}

function renderCarPickerMakers() {
  if (!appCarPickerList) return;
  carPickerMode = "makers";
  carPickerMaker = null;
  syncCarPickerChrome();
  appCarPickerList.innerHTML = "";

  const makers = CARS && CARS.makers ? Object.keys(CARS.makers) : [];
  makers.sort((a, b) => a.localeCompare(b));

  for (const mk of makers) {
    const arr = CARS.makers[mk] || [];
    const b = document.createElement("button");
    b.className = "btn groupBtn app-branch-picker__item app-car-picker__item";
    b.innerHTML = `<span class="app-branch-picker__label">${mk}</span><span class="app-branch-picker__badge">${arr.length}</span>`;
    b.onclick = () => renderCarPickerModels(mk);
    appCarPickerList.appendChild(b);
  }
}

function renderCarPickerModels(maker) {
  if (!appCarPickerList) return;
  carPickerMode = "models";
  carPickerMaker = maker;
  syncCarPickerChrome();
  appCarPickerList.innerHTML = "";

  const arr = (CARS?.makers && CARS.makers[maker]) ? CARS.makers[maker] : [];
  for (const fullLine of arr) {
    const modelOnly = stripMaker(fullLine, maker);
    const b = document.createElement("button");
    b.className = "btn groupBtn app-branch-picker__item app-car-picker__item";
    b.innerHTML = `<span class="app-branch-picker__label">${modelOnly}</span>`;
    b.onclick = () => onSelectCar(maker, modelOnly, fullLine);
    appCarPickerList.appendChild(b);
  }
}

async function ensureCarsLoaded() {
  if (CARS?.makers) return CARS;
  if (carsLoadPromise) return carsLoadPromise;

  carsLoadPromise = (async () => {
    const r = await fetch("/api/cars");
    const j = await r.json();
    if (!j.ok) throw new Error(j.error || "failed to load cars");
    CARS = j;
    return CARS;
  })();

  try {
    return await carsLoadPromise;
  } finally {
    carsLoadPromise = null;
  }
}

async function runOpenCarPickerFlow() {
  if (!appCarPickerList || !openCarPicker()) return false;
  carPickerMode = "makers";
  carPickerMaker = null;
  syncCarPickerChrome();
  appCarPickerMeta.textContent = "loading...";
  appCarPickerList.innerHTML = "";
  await ensureCarsLoaded();
  renderCarPickerMakers();
  return true;
}

window.openCarPickerFlow = () => {
  runOpenCarPickerFlow().catch((e) => {
    closeCarPicker(true);
    appAlert(e?.message || String(e), { title: getUIText("error", "Error") });
  });
};

if (appCarPickerBackdrop) appCarPickerBackdrop.onclick = () => closeCarPicker();
if (appCarPickerClose) {
  appCarPickerClose.onclick = () => {
    if (carPickerMode === "models") renderCarPickerMakers();
    else closeCarPicker();
  };
}

document.addEventListener("keydown", (ev) => {
  if (!appCarPicker || appCarPicker.hidden || ev.key !== "Escape") return;
  ev.preventDefault();
  if (carPickerMode === "models") renderCarPickerMakers();
  else closeCarPicker();
});

async function loadCars(options = {}) {
  const background = options.background === true;
  if (CARS?.makers) {
    if (!background) {
      const sources = (CARS.sources || []).join(", ");
      carMeta.textContent = sources ? ("sources: " + sources) : "ok";
      renderMakers();
      CURRENT_MAKER = null;
      showCarScreen("makers", false);
    }
    return CARS;
  }

  if (background) {
    return ensureCarsLoaded();
  }

  carMeta.textContent = "loading...";
  makerList.innerHTML = "";
  modelList.innerHTML = "";
  CURRENT_MAKER = null;
  showCarScreen("makers", false);

  try {
    const j = await ensureCarsLoaded();
    const sources = (j.sources || []).join(", ");
    carMeta.textContent = sources ? ("sources: " + sources) : "ok";
    renderMakers();
  } catch (e) {
    carMeta.textContent = "Failed: " + (e?.message || "unknown");
    return;
  }
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

  if (typeof applyCurrentCarLabel === "function") applyCurrentCarLabel(modelOnly);
  else {
    curCarLabelCar.textContent = modelOnly;
    curCarLabelSetting.textContent = modelOnly;
  }
  closeCarPicker(true);

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
async function loadSettings(options = {}) {
  const background = options.background === true;
  const force = options.force === true;
  const meta = document.getElementById("settingsMeta");

  if (SETTINGS && !force) {
    renderGroups();
    renderSettingSubnav();
    syncSettingSearchFabState();
    if (!background && CURRENT_PAGE === "setting" && typeof syncSettingViewportLayout === "function") {
      await syncSettingViewportLayout();
    }
    return SETTINGS;
  }

  if (!force && settingsLoadPromise) return settingsLoadPromise;
  if (!background && meta) meta.textContent = "loading...";

  settingsLoadPromise = (async () => {
    const r = await fetch("/api/settings");
    const j = await r.json();
    if (!j.ok) throw new Error(j.error || "unknown");

    SETTINGS = j;
    UNIT_CYCLE = j.unit_cycle || UNIT_CYCLE;
    settingValueCache.clear();
    settingGroupValueCache.clear();
    settingGroupValuePromises.clear();
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

function renderGroups() {
  const box = document.getElementById("groupList");
  box.innerHTML = "";

  (SETTINGS.groups || []).forEach(g => {
    const label = getSettingGroupLabel(g.group);

    const b = document.createElement("button");
    b.className = "btn groupBtn ui-stagger-item";
    b.style.setProperty("--i", String(box.children.length));
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
  const canReuseRenderedGroup =
    options.forceRender !== true &&
    previousGroup === nextGroup &&
    hasRenderedSettingItems(nextGroup);

  if (previousGroup && previousGroup !== nextGroup) {
    saveCurrentSettingScrollPosition(previousGroup);
  }

  CURRENT_GROUP = group;
  renderGroups();
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

  let values = {};
  try {
    values = await fetchSettingGroupValues(group, {
      force: options.forceValues === true,
      ttlMs: Number.isFinite(options.ttlMs) ? options.ttlMs : SETTING_VALUES_TTL_MS,
    });
  } catch (e) {
    values = {};
  }

  if (renderToken !== settingRenderToken || CURRENT_GROUP !== group || screenItems?.style.display === "none") {
    return;
  }

  list.forEach((p, index) => {
    const name = p.name;
    if (!(name in UNIT_INDEX)) UNIT_INDEX[name] = 0;

    const title = formatItemText(p, "title", "etitle", "");
    const descr = formatItemText(p, "descr", "edescr", "");

    const el = document.createElement("div");
    el.className = "setting ui-stagger-item";
    el.style.setProperty("--i", String(index));
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
        cacheSettingValue(name, next, group);
      } catch (e) {
        showAppToast((UI_STRINGS[LANG].set_failed || "set failed: ") + e.message, { tone: "error" });
      }
    }

    btnMinus.onclick = () => applyDelta(-1);
    btnPlus.onclick = () => applyDelta(+1);
  });

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
history.replaceState({ page: "carrot" }, "");

window.addEventListener("popstate", async (ev) => {
  const st = ev.state || { page: "carrot" };

  if (settingSearchPanel && !settingSearchPanel.hidden && !st.search) {
    closeSettingSearchPanel({ clear: false, fromHistory: true });
  }

  if (st.page === "home" || st.page === "carrot") {
    CURRENT_GROUP = null;
    CURRENT_MAKER = null;
    showPage("carrot", false);
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

  if (st.page === "logs") {
    showPage("logs", false);
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

showPage("carrot", false);


let toolsOutHistory = "";
let toolsOutCurrentBlock = "";
let toolsLogAttentionTimer = null;

function normalizeToolsOutText(s) {
  return String(s ?? "").replace(/\s+$/, "");
}

function scrollToolsLogToBottom(delay = 0) {
  window.setTimeout(() => {
    const out = document.getElementById("toolsOut");
    if (!out) return;
    out.scrollTop = out.scrollHeight;
  }, delay);
}

function pulseToolsLogPanel() {
  const page = document.getElementById("pageTools");
  if (!page || page.classList.contains("tools-log-expanded")) return;
  page.classList.add("tools-log-attention");
  if (toolsLogAttentionTimer) window.clearTimeout(toolsLogAttentionTimer);
  toolsLogAttentionTimer = window.setTimeout(() => {
    page.classList.remove("tools-log-attention");
    toolsLogAttentionTimer = null;
    scrollToolsLogToBottom();
    scrollToolsLogToBottom(280);
  }, 3200);
  scrollToolsLogToBottom(280);
}

function setToolsLogExpanded(expanded) {
  const page = document.getElementById("pageTools");
  if (!page) return;
  page.classList.toggle("tools-log-expanded", expanded);
  document.getElementById("toolsOut")?.setAttribute("aria-expanded", expanded ? "true" : "false");
  if (expanded) {
    page.classList.remove("tools-log-attention");
    if (toolsLogAttentionTimer) {
      window.clearTimeout(toolsLogAttentionTimer);
      toolsLogAttentionTimer = null;
    }
  }
  scrollToolsLogToBottom();
  scrollToolsLogToBottom(280);
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

  requestAnimationFrame(() => scrollToolsLogToBottom());
  scrollToolsLogToBottom(280);
}

function toolsOutSet(s) {
  toolsOutCurrentBlock = normalizeToolsOutText(s);
  renderToolsOut();
  pulseToolsLogPanel();
}

function toolsOutAppend(s) {
  const next = normalizeToolsOutText(s);
  if (!next) return;
  toolsOutHistory = toolsOutHistory ? `${toolsOutHistory}\n\n${next}` : next;
  renderToolsOut();
  pulseToolsLogPanel();
}

function toolsOutCommitCurrent() {
  if (!toolsOutCurrentBlock) return;
  toolsOutAppend(toolsOutCurrentBlock);
  toolsOutCurrentBlock = "";
  renderToolsOut();
}

function toolsLogNotice(message, options = {}) {
  const text = normalizeToolsOutText(message);
  if (!text) return;
  const label = options.label ? String(options.label).trim() : "notice";
  toolsOutCommitCurrent();
  toolsOutAppend(`[${label}]\n${text}`);
  if (options.meta !== false) toolsMetaSet(text.split("\n")[0]);
  if (options.clearProgress !== false) toolsProgressSet(null, { active: false });
}

function getToolCommandPreview(action, payload = {}) {
  switch (action) {
    case "shell_cmd": return String(payload.cmd || "").trim() || "command";
    case "git_pull": return "git pull";
    case "git_sync": return "git sync";
    case "git_reset": return `git reset --${payload.mode || "hard"} ${payload.target || "HEAD"}`.trim();
    case "git_checkout": return `git checkout ${payload.branch || ""}`.trim();
    case "git_branch_list": return "change branch";
    case "git_remote_add": return `git remote add/set-url ${payload.name || ""}`.trim();
    case "send_tmux_log": return "capture tmux";
    case "server_tmux_log": return "send tmux";
    case "install_required": return "install flask";
    case "delete_all_videos": return "delete all videos";
    case "delete_all_logs": return "delete all logs";
    case "rebuild_all": return "rebuild all";
    case "backup_settings": return "backup settings";
    case "reboot": return "reboot";
    case "git_reset_repo_fetch": return "reset repo (fetch)";
    case "git_reset_repo_checkout": return `reset repo (checkout ${payload.branch || "unknown"})`;
    case "reset_calib": return "reset calib";
    default: return action;
  }
}

let toolsMetaStatusText = "";
let toolsMetaInfoText = "";
let toolsMetaInfoDialogText = "";

function renderToolsMeta() {
  const meta = document.getElementById("toolsMeta");
  if (!meta) return;

  meta.textContent = "";

  const statusEl = document.createElement("span");
  statusEl.className = "tools-meta__status";
  statusEl.textContent = toolsMetaStatusText || "-";
  meta.appendChild(statusEl);

  const actionsEl = document.createElement("div");
  actionsEl.className = "tools-meta__actions";

  const langBtn = document.createElement("button");
  langBtn.type = "button";
  langBtn.className = "tools-meta__langBtn";
  langBtn.textContent = (typeof LANG_EMOJI === "object" && LANG_EMOJI[LANG]) ? LANG_EMOJI[LANG] : "🌐";
  const langTitle = LANG === "en"
    ? "Language"
    : LANG === "zh"
      ? "语言"
      : "언어";
  langBtn.setAttribute("aria-label", langTitle);
  langBtn.title = langTitle;
  langBtn.addEventListener("click", () => {
    if (typeof toggleLang === "function") toggleLang();
  });
  actionsEl.appendChild(langBtn);



  meta.appendChild(actionsEl);
}

function toolsMetaSet(s) {
  toolsMetaStatusText = String(s || "");
  renderToolsMeta();
}

function buildToolsMetaInfo(values = {}) {
  const branch = String(values.GitBranch || "").trim();
  const commit = String(values.GitCommit || "").trim();
  const dongleId = String(values.DongleId || "").trim();
  const serial = String(values.HardwareSerial || "").trim();
  const parts = [];
  if (branch) parts.push(branch);
  if (commit) parts.push(commit.slice(0, 7));
  if (dongleId) parts.push(dongleId);
  if (serial) parts.push(serial);
  return parts.join("  ·  ");
}

function formatToolsMetaDate(value) {
  let raw = String(value || "").replace(/['"]/g, "").trim();
  if (!raw) return "";

  // Support "1730000000 2024-10-27..."
  const parts = raw.split(" ");
  if (parts.length > 1 && /^\d{10,}$/.test(parts[0])) {
    const d = new Date(parseInt(parts[0], 10) * 1000);
    if (!isNaN(d)) return `${d.getFullYear()}.${String(d.getMonth() + 1).padStart(2, "0")}.${String(d.getDate()).padStart(2, "0")}`;
  }

  const m = raw.match(/^(\d{4})[-./](\d{2})[-./](\d{2})/);
  if (m) return `${m[1]}.${m[2]}.${m[3]}`;
  const ts = Date.parse(raw);
  if (!Number.isFinite(ts)) return "";
  const d = new Date(ts);
  const yyyy = d.getFullYear();
  const mm = String(d.getMonth() + 1).padStart(2, "0");
  const dd = String(d.getDate()).padStart(2, "0");
  return `${yyyy}.${mm}.${dd}`;
}

function formatToolsMetaDateTime(value) {
  const raw = String(value || "").trim();
  if (!raw) return "";

  let ms = NaN;
  if (/^\d+$/.test(raw)) {
    const num = Number(raw);
    if (Number.isFinite(num)) ms = raw.length >= 13 ? num : num * 1000;
  } else {
    ms = Date.parse(raw);
  }
  if (!Number.isFinite(ms)) return "";

  const d = new Date(ms);
  const yyyy = d.getFullYear();
  const mm = String(d.getMonth() + 1).padStart(2, "0");
  const dd = String(d.getDate()).padStart(2, "0");
  const hh = String(d.getHours()).padStart(2, "0");
  const min = String(d.getMinutes()).padStart(2, "0");
  const ss = String(d.getSeconds()).padStart(2, "0");
  return `${yyyy}.${mm}.${dd} ${hh}:${min}:${ss}`;
}

function buildToolsMetaInfoDialog(values = {}) {
  const branch = String(values.GitBranch || "").trim();
  const commit = String(values.GitCommit || "").trim();
  const commitDate = formatToolsMetaDate(values.GitCommitDate);
  const dongleId = String(values.DongleId || "").trim();
  const serial = String(values.HardwareSerial || "").trim();
  const gitPullTime = formatToolsMetaDateTime(values.GitPullTime);
  const labels = LANG === "en"
    ? { branch: "Branch", commit: "Commit", deviceType: "Device", dongle: "Dongle ID", serial: "Serial", gitPull: "Recent update", position: "Position" }
    : LANG === "zh"
      ? { branch: "分支", commit: "提交", deviceType: "设备型号", dongle: "Dongle ID", serial: "序列号", gitPull: "最近更新", position: "安装角度" }
      : { branch: "브랜치", commit: "커밋", deviceType: "기기", dongle: "동글ID", serial: "시리얼", gitPull: "최근 업데이트", position: "설치각도" };
  const htmlEscape = typeof escapeHtml === "function"
    ? escapeHtml
    : (value) => String(value)
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;")
      .replaceAll('"', "&quot;")
      .replaceAll("'", "&#039;");
  const lines = [];
  const remote = String(values.GitRemote || "").trim();
  const shortRemote = remote ? remote.replace(/^https?:\/\/[^/]+\//, "").replace(/\.git$/, "") : "";
  const branchText = branch + (shortRemote ? ` (${shortRemote})` : "");
  if (branchText) lines.push(`<div class="app-dialog__metaLine">${htmlEscape(labels.branch)}: ${htmlEscape(branchText)}</div>`);
  if (commit) {
    const commitText = `${commit.slice(0, 7)}${commitDate ? ` (${commitDate})` : ""}`;
    lines.push(`<div class="app-dialog__metaLine">${htmlEscape(labels.commit)}: ${htmlEscape(commitText)}</div>`);
  }

  if (dongleId) lines.push(`<div class="app-dialog__metaLine">${htmlEscape(labels.dongle)}: ${htmlEscape(dongleId)}</div>`);
  if (serial) lines.push(`<div class="app-dialog__metaLine">${htmlEscape(labels.serial)}: ${htmlEscape(serial)}</div>`);
  const position = String(values.DevicePosition || "").trim();
  lines.push(`<div class="app-dialog__metaLine">${htmlEscape(labels.position)}: ${htmlEscape(position)}</div>`);
  if (gitPullTime) {
    lines.push(`<div class="app-dialog__metaSubtle">${htmlEscape(labels.gitPull)}: ${htmlEscape(gitPullTime)}</div>`);
  }
  return `<div class="app-dialog__metaList" id="toolsMetaListContent">${lines.join("")}</div>`;
}

function buildToolsMetaPlainText(values = {}) {
  const branch = String(values.GitBranch || "").trim();
  const commit = String(values.GitCommit || "").trim();
  const commitDate = formatToolsMetaDate(values.GitCommitDate);
  const dongleId = String(values.DongleId || "").trim();
  const serial = String(values.HardwareSerial || "").trim();
  const gitPullTime = formatToolsMetaDateTime(values.GitPullTime);
  const remote = String(values.GitRemote || "").trim();
  const shortRemote = remote ? remote.replace(/^https?:\/\/[^/]+\//, "").replace(/\.git$/, "") : "";
  const branchText = branch + (shortRemote ? ` (${shortRemote})` : "");
  const position = String(values.DevicePosition || "").trim();
  const lines = [];
  if (branchText) lines.push(`Branch: ${branchText}`);
  if (commit) lines.push(`Commit: ${commit.slice(0, 7)}${commitDate ? ` (${commitDate})` : ""}`);
  if (dongleId) lines.push(`DongleID: ${dongleId}`);
  if (serial) lines.push(`Serial: ${serial}`);
  if (position) lines.push(`Position: ${position}`);
  if (gitPullTime) lines.push(`Updated: ${gitPullTime}`);
  return lines.join("\n");
}

function rerenderPageLangUi() {
  renderToolsMeta();
  refreshToolsMetaInfo().catch(() => {});

  const terminalMetaEl = document.getElementById("terminalMeta");
  if (!terminalMetaEl) return;

  const current = String(terminalMetaEl.textContent || "").trim();
  const terminalStates = [
    ["connecting", "connecting..."],
    ["connected", "connected"],
    ["reconnecting", "reconnecting..."],
    ["terminal_ready", "tmux ready"],
    ["terminal_disconnected", "disconnected"],
    ["terminal_unavailable", "terminal unavailable"],
    ["terminal_offline", "terminal offline"],
  ];

  for (const [key, fallback] of terminalStates) {
    const variants = ["ko", "en", "zh"]
      .map((langKey) => UI_STRINGS[langKey]?.[key] || fallback)
      .filter(Boolean);
    if (variants.includes(current)) {
      terminalMetaEl.textContent = getUIText(key, fallback);
      break;
    }
  }
}

async function refreshToolsMetaInfo(options = {}) {
  const force = options.force === true;
  const silent = options.silent === true;
  const ttlMs = Number.isFinite(options.ttlMs) ? options.ttlMs : PAGE_DATA_TTL_MS;
  if (typeof bulkGet !== "function") return;
  if (!force && toolsMetaLoadPromise) return toolsMetaLoadPromise;
  if (!force && hasFreshPageData(toolsMetaLoadedAt, ttlMs)) {
    if (!silent || CURRENT_PAGE === "tools") renderToolsMeta();
    return {
      text: toolsMetaInfoText,
      dialog: toolsMetaInfoDialogText,
    };
  }

  toolsMetaLoadPromise = (async () => {
    const values = await bulkGet(["GitBranch", "GitCommit", "GitCommitDate", "GitRemote", "DeviceType", "DongleId", "HardwareSerial", "GitPullTime", "DevicePosition"]);
    toolsMetaInfoText = buildToolsMetaInfo(values);
    toolsMetaInfoDialogText = buildToolsMetaInfoDialog(values);
    toolsMetaLoadedAt = Date.now();
    toolsMetaLastValues = values;
    if (!silent || CURRENT_PAGE === "tools") renderToolsMeta();
    return values;
  })().finally(() => {
    toolsMetaLoadPromise = null;
  });

  return toolsMetaLoadPromise;
}

async function syncDeviceLanguageOnce() {
  if (typeof bulkGet !== "function" || typeof setParam !== "function") return;
  try {
    const SYNC_KEY = "carrot_device_lang_synced";
    if (localStorage.getItem(SYNC_KEY) === "1") return;

    const values = await bulkGet(["LanguageSetting"]);
    const currentLang = String(values["LanguageSetting"] || "").trim();

    const browserLang = (navigator.language || navigator.userLanguage || "en").toLowerCase();
    let targetParam = "main_en";
    if (browserLang.startsWith("ko")) targetParam = "main_ko";
    else if (browserLang.startsWith("zh")) targetParam = browserLang.includes("tw") || browserLang.includes("hk") ? "main_zh-CHT" : "main_zh-CHS";
    else if (browserLang.startsWith("ja")) targetParam = "main_ja";
    else if (browserLang.startsWith("de")) targetParam = "main_de";
    else if (browserLang.startsWith("fr")) targetParam = "main_fr";
    else if (browserLang.startsWith("es")) targetParam = "main_es";
    else if (browserLang.startsWith("pt")) targetParam = "main_pt-BR";
    else if (browserLang.startsWith("tr")) targetParam = "main_tr";
    else if (browserLang.startsWith("ar")) targetParam = "main_ar";
    else if (browserLang.startsWith("th")) targetParam = "main_th";

    if (currentLang !== targetParam) {
      await setParam("LanguageSetting", targetParam);
      localStorage.setItem(SYNC_KEY, "1");
      // show notification after a short delay so the page finishes loading
      setTimeout(() => {
        const msg = LANG === "ko"
          ? "기기 언어를 변경했습니다.\n기기를 재부팅해야 적용됩니다."
          : "Device language has been changed.\nPlease reboot the device to apply.";
        openAppDialog({ mode: "alert", title: "Device Language", message: msg });
      }, 800);
      return;
    }
    localStorage.setItem(SYNC_KEY, "1");
  } catch (e) {
    console.log("Language sync failed:", e);
  }
}

function runUiWarmup() {
  return Promise.allSettled([
    syncDeviceLanguageOnce(),
    loadCurrentCar({ resetRetry: false, ttlMs: PAGE_DATA_TTL_MS }),
    loadRecordState({ ttlMs: PAGE_DATA_TTL_MS }),
    refreshToolsMetaInfo({ silent: true, ttlMs: PAGE_DATA_TTL_MS }),
    typeof updateQuickLink === "function" ? updateQuickLink({ silent: true, ttlMs: PAGE_DATA_TTL_MS }) : Promise.resolve(),
    loadSettings({ background: true }),
    ensureCarsLoaded(),
  ]);
}

function scheduleUiWarmup(delay = 140) {
  if (uiWarmupTimer) return;
  uiWarmupTimer = window.setTimeout(() => {
    uiWarmupTimer = null;
    requestIdleTask(() => {
      runUiWarmup().catch(() => {});
    });
  }, Math.max(0, delay));
}

if (document.readyState === "complete") {
  scheduleUiWarmup(120);
} else {
  window.addEventListener("load", () => {
    scheduleUiWarmup(120);
  }, { once: true });
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

function didGitPullUpdate(result) {
  const body = normalizeToolsOutText(result?.out || result?.log || "");
  if (!body) return false;
  const lower = body.toLowerCase();
  if (lower.includes("already up to date") || lower.includes("already up-to-date")) {
    return false;
  }
  return (
    lower.includes("fast-forward") ||
    lower.includes("merge made by") ||
    lower.includes("updating ") ||
    /[0-9]+\s+files?\s+changed/.test(lower) ||
    lower.includes("create mode ") ||
    lower.includes("delete mode ") ||
    lower.includes("rewrite ")
  );
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
  toolsLogNotice(msg, { label: action, meta: false });
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

function parseGitHubOwner(url) {
  const text = String(url || "").trim();
  if (!text) return "";
  const httpsMatch = text.match(/github\.com\/([^\/:\s]+)\//);
  if (httpsMatch) return httpsMatch[1];
  const sshMatch = text.match(/github\.com[:\/]([^\/:\s]+)\//);
  return sshMatch ? sshMatch[1] : "";
}

function resetBranchRemoteContext() {
  ORIGIN_USERNAME = "origin";
  BRANCH_REMOTE_NAMES = ["origin"];
  BRANCH_REMOTE_OWNERS = Object.create(null);
}

function syncBranchRemoteContext(result = {}) {
  const remotes = Array.isArray(result.remotes) ? result.remotes.map((r) => String(r || "").trim()).filter(Boolean) : [];
  BRANCH_REMOTE_NAMES = remotes.length ? remotes : ["origin"];
  BRANCH_REMOTE_OWNERS = Object.create(null);

  const remoteUrls = result.remote_urls && typeof result.remote_urls === "object" ? result.remote_urls : {};
  for (const [name, url] of Object.entries(remoteUrls)) {
    const owner = parseGitHubOwner(url);
    if (owner) BRANCH_REMOTE_OWNERS[name] = owner;
  }

  if (BRANCH_REMOTE_OWNERS.origin) {
    ORIGIN_USERNAME = BRANCH_REMOTE_OWNERS.origin;
  }
}

function normalizeBranchItems(result = {}) {
  const items = Array.isArray(result.branch_items) ? result.branch_items : [];
  if (items.length) {
    return items
      .map((item) => {
        const kind = item && item.kind === "remote" ? "remote" : "local";
        const remote = String(item?.remote || "").trim();
        const name = String(item?.name || item?.label || item?.ref || "").trim();
        const ref = String(item?.ref || (kind === "remote" && remote && name ? `${remote}/${name}` : name)).trim();
        if (!ref || !name) return null;
        return {
          id: String(item?.id || `${kind}:${remote}:${name}`),
          kind,
          ref,
          remote,
          name,
          label: String(item?.label || name).trim() || name,
        };
      })
      .filter(Boolean);
  }

  const refs = Array.isArray(result.branches) ? result.branches : [];
  return refs
    .map((ref) => ({ kind: "legacy", ref: String(ref || "").trim() }))
    .filter((item) => item.ref);
}

function getBranchDisplayInfo(entry) {
  if (entry && typeof entry === "object" && entry.kind !== "legacy") {
    const kind = entry.kind === "remote" ? "remote" : "local";
    const ref = String(entry.ref || "").trim();
    const name = String(entry.name || entry.label || ref).trim();

    if (kind === "local") {
      return {
        ref,
        label: name,
        groupKey: "local",
        groupTitle: "local",
        groupRank: 0,
        kind: "local",
        name,
        checkoutPayload: { branch: ref, kind: "local", name },
      };
    }

    const remoteName = String(entry.remote || "").trim();
    const owner = BRANCH_REMOTE_OWNERS[remoteName] || (remoteName === "origin" ? ORIGIN_USERNAME : remoteName);
    return {
      ref,
      label: String(entry.label || name).trim() || name,
      groupKey: `remote:${remoteName}:${owner}`,
      groupTitle: remoteName === "origin" ? `${remoteName} / ${owner}` : `remote / ${owner}`,
      groupRank: remoteName === "origin" ? 1 : 2,
      kind: "remote",
      remote: remoteName,
      name,
      checkoutPayload: { branch: ref, kind: "remote", remote: remoteName, name },
    };
  }

  const raw = String(entry?.ref || entry || "").trim();
  const parts = raw.split("/").filter(Boolean);
  const first = parts[0] || "";
  const isRemoteRef = parts.length >= 2 && BRANCH_REMOTE_NAMES.includes(first);

  if (!isRemoteRef) {
    return {
      ref: raw,
      label: raw,
      groupKey: "local",
      groupTitle: "local",
      groupRank: 0,
      kind: "local",
      name: raw,
      checkoutPayload: { branch: raw },
    };
  }

  const remoteName = first;
  const rest = parts.slice(1);

  const owner = BRANCH_REMOTE_OWNERS[remoteName] || (remoteName === "origin" ? ORIGIN_USERNAME : remoteName);
  return {
    ref: raw,
    label: rest.join("/") || raw,
    groupKey: `remote:${remoteName}:${owner}`,
    groupTitle: remoteName === "origin" ? `${remoteName} / ${owner}` : `remote / ${owner}`,
    groupRank: remoteName === "origin" ? 1 : 2,
    kind: "remote",
    remote: remoteName,
    name: rest.join("/") || raw,
    checkoutPayload: { branch: raw },
  };
}

function isCurrentBranchItem(info) {
  return Boolean(info && info.kind === "local" && CURRENT_BRANCH_NAME && info.name === CURRENT_BRANCH_NAME);
}

function getBranchGroups() {
  const groups = new Map();
  for (const br of BRANCHES) {
    const info = getBranchDisplayInfo(br);
    if (!info.ref || !info.label) continue;

    let group = groups.get(info.groupKey);
    if (!group) {
      group = {
        key: info.groupKey,
        title: info.groupTitle,
        rank: info.groupRank,
        items: [],
        hasCurrent: false,
      };
      groups.set(info.groupKey, group);
    }

    const current = isCurrentBranchItem(info);
    group.hasCurrent = group.hasCurrent || current;
    group.items.push({ ...info, current });
  }

  return Array.from(groups.values()).sort((a, b) => {
    if (a.rank !== b.rank) return a.rank - b.rank;
    return a.title.localeCompare(b.title);
  });
}

function isBranchGroupOpen(group) {
  if (Object.prototype.hasOwnProperty.call(BRANCH_GROUP_OPEN, group.key)) {
    return Boolean(BRANCH_GROUP_OPEN[group.key]);
  }
  return group.key === "local" || group.rank === 1 || group.hasCurrent;
}

function branchCountLabel(count) {
  return count === 1 ? "1 branch" : `${count} branches`;
}


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

  const initToolsGroups = () => {
    const groups = Array.from(document.querySelectorAll("#pageTools .tools-group"));
    const applyToolsStagger = () => {
      const items = Array.from(document.querySelectorAll(
        "#pageTools .tools-scroll-stack > .row-wrap:first-child, #pageTools .tools-group, #pageTools .tools-group.is-open .tools-group__body > *"
      )).filter((node) => !node.hidden && !node.classList.contains("hidden"));
      items.forEach((node, index) => {
        node.classList.add("ui-stagger-item");
        node.style.setProperty("--i", String(index));
      });
    };

    groups.forEach((group) => {
      const toggle = group.querySelector(".tools-group__toggle");
      const body = group.querySelector(".tools-group__body");
      if (!toggle || !body) return;

      const groupName = group.dataset.toolsGroup;
      const savedState = localStorage.getItem("tools_group_" + groupName);
      const shouldOpen = savedState !== null ? savedState === "true" : true;

      group.classList.toggle("is-open", shouldOpen);
      body.hidden = !shouldOpen;
      body.classList.toggle("hidden", !shouldOpen);
      toggle.setAttribute("aria-expanded", shouldOpen ? "true" : "false");

      bindNodeOnce(toggle, "toolsGroupToggle", () => {
        const nextOpen = body.hidden;
        body.hidden = !nextOpen;
        body.classList.toggle("hidden", !nextOpen);
        group.classList.toggle("is-open", nextOpen);
        toggle.setAttribute("aria-expanded", nextOpen ? "true" : "false");
        localStorage.setItem("tools_group_" + groupName, nextOpen ? "true" : "false");
        applyToolsStagger();
      });
    });
    applyToolsStagger();
  };

  const initToolsLogPanel = () => {
    const out = document.getElementById("toolsOut");
    if (!out || out.dataset.toolsLogBound === "1") return;
    out.dataset.toolsLogBound = "1";
    out.setAttribute("role", "button");
    out.setAttribute("tabindex", "0");
    out.setAttribute("aria-expanded", "false");
    out.setAttribute("aria-label", LANG === "ko" ? "로그창 펼치기 또는 접기" : "Expand or collapse log panel");
    out.addEventListener("click", () => {
      const page = document.getElementById("pageTools");
      setToolsLogExpanded(!page?.classList.contains("tools-log-expanded"));
    });
    out.addEventListener("keydown", (ev) => {
      if (ev.key !== "Enter" && ev.key !== " ") return;
      ev.preventDefault();
      const page = document.getElementById("pageTools");
      setToolsLogExpanded(!page?.classList.contains("tools-log-expanded"));
    });
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
  refreshToolsMetaInfo().catch(() => {});
  initToolsGroups();
  initToolsLogPanel();

  bindOnce("btnDeviceInfo", async () => {
    let title = LANG === "en" ? "Device Info" : LANG === "zh" ? "设备信息" : "기기정보";
    
    try {
      if (!toolsMetaLastValues && !toolsMetaLoadPromise) {
        await refreshToolsMetaInfo({ ttlMs: 3600000 });
      }
      const values = toolsMetaLoadPromise ? await toolsMetaLoadPromise : toolsMetaLastValues;
      if (values) {
        const deviceType = String(values.DeviceType || "").trim();
        if (deviceType) {
          const deviceFriendly = { tici: "c3", tizi: "c3x", mici: "c4" };
          const friendly = deviceFriendly[deviceType] || deviceType;
          const label = friendly !== deviceType ? `${friendly}/${deviceType}` : deviceType;
          title += `(${label})`;
        }
      }
    } catch (e) {}

    appAlert(toolsMetaInfoDialogText || toolsMetaInfoText, {
      title,
      html: true,
      messageHtml: toolsMetaInfoDialogText,
      copyText: buildToolsMetaPlainText(toolsMetaLastValues || {}),
    });
  });

  bindOnce("btnGitPull", async () => {
    try {
      const result = await runTool("git_pull");
      await refreshToolsMetaInfo();
      if (!didGitPullUpdate(result)) return;
      if (await appConfirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?", {
        title: UI_STRINGS[LANG].reboot || "Reboot",
      })) {
        await runTool("reboot");
      }
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

  async function runGitResetMode(mode) {
    const safeMode = String(mode || "hard").trim().toLowerCase();
    const title = `git reset --${safeMode}`;
    const message = UI_STRINGS[LANG].git_reset_confirm || "Run git reset?";
    if (!await appConfirm(`${message}\n\nHEAD`, { title })) return;
    try {
      await runTool("git_reset", { mode: safeMode, target: "HEAD" });
    } catch (e) {
      showError("git_reset", e);
    }
  }

  bindOnce("btnGitReset", async () => {
    const mode = await openAppDialog({
      mode: "choice",
      title: "git reset",
      message: "HEAD 기준 리셋 방식을 선택하세요.",
      cancelLabel: UI_STRINGS[LANG].cancel || "Cancel",
      choices: [
        { label: "reset hard", value: "hard", danger: true },
        { label: "reset mixed", value: "mixed" },
        { label: "reset soft", value: "soft" },
      ],
    });
    if (!mode) return;
    await runGitResetMode(mode);
  });

  bindOnce("btnGitRemote", async () => {
    const title = LANG === "ko" ? "저장소 주소 변경" : "Change Repository";
    let defaultUrl = "";
    try {
      const v = await bulkGet(["GitRemote"]);
      if (v && v.GitRemote) defaultUrl = String(v.GitRemote).trim();
    } catch (e) {}

    const msg = LANG === "ko"
      ? `현재 주소: ${defaultUrl}\n\n새로운 GitHub 저장소 주소를 붙여넣으세요.\n(해당 저장소로 연결을 덮어씁니다)`
      : `Current: ${defaultUrl}\n\nEnter new GitHub repository URL.\n(This will overwrite the current connection)`;
    
    const newUrl = await appPrompt(msg, defaultUrl, { title });
    if (!newUrl || newUrl.trim() === "" || newUrl.trim() === defaultUrl) return;

    try {
      const waitMsg = LANG === "ko"
        ? "저장소 데이터를 받아오는 중입니다.\n처음 연결하는 저장소의 경우 수 분이 걸릴 수 있습니다.\n잠시만 기다려 주세요..."
        : "Fetching repository data.\nThis may take a few minutes for new repositories.\nPlease wait...";
      toolsLogNotice(waitMsg, { label: "change repository" });
      await runTool("git_remote_set", { url: newUrl.trim() });
      await refreshToolsMetaInfo();
      const successMsg = LANG === "ko" 
        ? "저장소가 성공적으로 변경되었습니다.\n[change branch] 버튼을 눌러 새 저장소의 브랜치를 선택해 주세요." 
        : "Repository changed successfully.\nClick [change branch] to select a branch.";
      toolsLogNotice(successMsg, { label: "change repository" });
    } catch (e) {
      showError("change repository", e);
    }
  });
  bindOnce("btnGitBranch", async () => {
    await loadBranchesAndShow();
  });

  bindOnce("btnGitAddRemote", async () => {
    const title = LANG === "ko" ? "리모트 추가/갱신" : "Add/Update Remote";
    const nameInput = await appPrompt(
      LANG === "ko" ? "리모트 이름을 입력하세요 (예: remote)" : "Enter remote name (e.g. remote)",
      { title, placeholder: "remote" }
    );
    if (!nameInput || !nameInput.trim()) return;
    const remoteName = nameInput.trim();

    const urlInput = await appPrompt(
      LANG === "ko" ? `'${remoteName}' 리모트의 URL을 입력하세요` : `Enter URL for '${remoteName}'`,
      { title, placeholder: "https://github.com/user/repo" }
    );
    if (!urlInput || !urlInput.trim()) return;

    try {
      await runTool("git_remote_add", { name: remoteName, url: urlInput.trim() });
      toolsLogNotice(LANG === "ko" ? `리모트 '${remoteName}' 추가/갱신 완료` : `Remote '${remoteName}' added/updated`, { label: "git_remote_add" });
    } catch (e) {
      showError("git_remote_add", e);
    }
  });

  bindOnce("btnGitLog", async () => {
    try {
      // 1. 터미널 출력
      await runTool("git_log", { count: 20 });

      // 2. 팝업 UI 표시를 위한 데이터 다시 로드
      const res = await postJson("/api/tools", { action: "git_log", count: 20 });
      if (!res.ok) throw new Error(res.error || "Failed to load git log");
      const commits = res.commits || [];
      const currentCommit = res.current_commit || "";
      if (!commits.length) {
        toolsLogNotice("No commits found", { label: "git_log" });
        return;
      }

      const selected = await openAppDialog({
        mode: "choice",
        title: "git log",
        message: LANG === "ko" ? "이동할 커밋을 선택하세요" : "Select commit to checkout",
        cancelLabel: UI_STRINGS[LANG].cancel || "Cancel",
        choices: commits.map(c => {
          const isCurrent = currentCommit && c.hash.startsWith(currentCommit);
          const badgeHtml = isCurrent
            ? ` <span class="app-branch-picker__badge">${getUIText("branch_current", "Current")}</span>`
            : "";
          return {
            labelHtml: `<span class="app-branch-picker__label"><span style="color:#4caf50;font-weight:700;font-family:monospace;margin-right:8px;">${escapeHtml(c.hash)}</span>${escapeHtml(c.message)}</span>${badgeHtml}`,
            value: c.hash,
            className: isCurrent ? "is-current" : "",
          };
        }),
      });
      if (!selected) return;

      const confirmMsg = LANG === "ko"
        ? `이 커밋으로 이동하시겠습니까?\n\n${selected}`
        : `Checkout this commit?\n\n${selected}`;
      if (!await appConfirm(confirmMsg, { title: "git checkout" })) return;

      const resetRes = await postJson("/api/tools", { action: "git_reset", mode: "hard", target: selected });
      if (!resetRes.ok) throw new Error(resetRes.error || "Reset failed");
      
      toolsLogNotice(LANG === "ko" ? "이동 완료" : "Checkout complete", { label: "git_log" });
      await refreshToolsMetaInfo();
    } catch (e) {
      showError("git_log", e);
    }
  });

  bindOnce("btnGitResetRepo", async () => {
    const title = LANG === "ko" ? "저장소 초기화" : "Reset Repository";
    const msg = LANG === "ko"
      ? "주의: 기존 origin을 삭제하고 'ajouatom/openpilot'으로 재설정합니다.\n모든 로컬 변경사항이 삭제됩니다. 진행하시겠습니까?"
      : "Warning: This will remove origin and re-add 'ajouatom/openpilot'.\nAll local changes will be lost. Proceed?";
    
    if (!await appConfirm(msg, { title, danger: true })) return;

    try {
      // Phase 1: fetch remote and get branch list
      const fetchResult = await runTool("git_reset_repo_fetch");
      const branches = fetchResult.branches || [];
      if (!branches.length) {
        toolsLogNotice(LANG === "ko" ? "브랜치를 찾을 수 없습니다" : "No branches found", { label: "git_reset_repo" });
        return;
      }

      // Phase 2: let user pick a branch
      const selected = await openAppDialog({
        mode: "choice",
        title: LANG === "ko" ? "브랜치 선택" : "Select Branch",
        message: LANG === "ko" ? "초기화할 브랜치를 선택하세요" : "Select branch to reset to",
        cancelLabel: UI_STRINGS[LANG].cancel || "Cancel",
        choices: branches.map(b => ({ label: b, value: b })),
      });
      if (!selected) return;

      // Phase 3: checkout selected branch
      await runTool("git_reset_repo_checkout", { branch: selected });
      toolsLogNotice(LANG === "ko" ? `'${selected}' 브랜치로 초기화 완료` : `Reset to '${selected}' complete`, { label: "git_reset_repo" });
      await refreshToolsMetaInfo();

      if (await appConfirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?", {
        title: UI_STRINGS[LANG].reboot || "Reboot",
      })) {
        await runTool("reboot");
      }
    } catch (e) {
      showError("git_reset_repo", e);
    }
  });

  bindOnce("btnResetCalib", async () => {
    const title = LANG === "ko" ? "캘리브레이션 초기화" : "ReCalibration";
    const msg = LANG === "ko" 
      ? "캘리브레이션을 초기화하시겠습니까?\n초기화 후 자동으로 재부팅됩니다."
      : "Are you sure you want to reset calibration?\nDevice will reboot automatically.";
    if (!await appConfirm(msg, { title })) return;
    try {
      await runTool("reset_calib");
    } catch (e) {
      showError("reset_calib", e);
    }
  });

  bindOnce("btnDeviceLang", async () => {
    const choices = [
      { label: "한국어", value: "main_ko" },
      { label: "English", value: "main_en" },
      { label: "中文(简体)", value: "main_zh-CHS" },
      { label: "中文(繁體)", value: "main_zh-CHT" },
      { label: "日本語", value: "main_ja" },
      { label: "Deutsch", value: "main_de" },
      { label: "Français", value: "main_fr" },
      { label: "Português", value: "main_pt-BR" },
      { label: "Español", value: "main_es" },
      { label: "Türkçe", value: "main_tr" },
      { label: "العربية", value: "main_ar" },
      { label: "ไทย", value: "main_th" },
    ];
    const val = await openAppDialog({
      mode: "choice",
      title: "Device Language",
      message: LANG === "ko" ? "기기 언어를 선택하세요." : "Select language for the device UI",
      cancelLabel: UI_STRINGS[LANG]?.cancel || "Cancel",
      choices
    });
    if (!val) return;
    try {
      await setParam("LanguageSetting", val);
      const rebootMsg = LANG === "ko" ? "설정이 변경되었습니다. 지금 재부팅하시겠습니까?" : "Setting changed. Reboot now?";
      if (await appConfirm(rebootMsg, { title: "Reboot" })) {
        await runTool("reboot");
      }
    } catch (e) {
      showError("shell_cmd", e);
    }
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
    inp.accept = "application/json,text/plain,*/*";
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

  function getAllSettingNames(settingsObj) {
    if (!settingsObj || !settingsObj.items_by_group) return [];
    const names = [];
    Object.values(settingsObj.items_by_group).forEach(list => {
      list.forEach(item => names.push(item.name));
    });
    return names;
  }

  bindOnce("btnCopySettings", async () => {
    try {
      if (!SETTINGS || !SETTINGS.items_by_group) {
        const r = await fetch("/api/settings");
        const j = await r.json();
        if (j.ok) SETTINGS = j;
      }
      if (!SETTINGS || !SETTINGS.items_by_group) {
        toolsLogNotice("Settings not loaded", { label: "copy settings" });
        return;
      }
      const allNames = getAllSettingNames(SETTINGS);
      const values = await bulkGet(allNames);
      const lines = allNames.map(n => `${n}=${values[n] ?? ""}`);
      const text = lines.join("\n");
      copyToClipboard(text);
      toolsLogNotice(LANG === "ko" ? `${allNames.length}개 파라미터 복사됨` : `${allNames.length} params copied`, { label: "copy settings" });
    } catch (e) {
      showError("copy settings", e);
    }
  });

  bindOnce("btnViewSettings", async () => {
    try {
      if (!SETTINGS || !SETTINGS.items_by_group) {
        const r = await fetch("/api/settings");
        const j = await r.json();
        if (j.ok) SETTINGS = j;
      }
      if (!SETTINGS || !SETTINGS.items_by_group) {
        toolsLogNotice("Settings not loaded", { label: "view settings" });
        return;
      }
      const allNames = getAllSettingNames(SETTINGS);
      const values = await bulkGet(allNames);
      const lines = allNames.map(n => `${n} = ${values[n] ?? "(empty)"}`);
      const text = lines.join("\n");
      appAlert(text, {
        title: `Settings (${allNames.length} params)`,
        copyText: text,
      });
    } catch (e) {
      showError("view settings", e);
    }
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
    toolsLogNotice(UI_STRINGS[LANG].branch_dom_missing || "Branch DOM missing", { label: "git_branch_list" });
    return;
  }
  appBranchPickerMeta.textContent = "loading...";
  appBranchPickerList.innerHTML = "";
  BRANCHES = [];
  CURRENT_BRANCH_NAME = "";
  resetBranchRemoteContext();

  try {
    const v = await bulkGet(["GitRemote"]);
    if (v && v.GitRemote) {
      const owner = parseGitHubOwner(v.GitRemote);
      if (owner) ORIGIN_USERNAME = owner;
    }
  } catch(e) {}

  try {
    const j = await runTool("git_branch_list");
    syncBranchRemoteContext(j);
    BRANCHES = normalizeBranchItems(j);
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

  for (const group of getBranchGroups()) {
    const open = isBranchGroupOpen(group);
    const section = document.createElement("div");
    section.className = "app-branch-picker__group";
    section.classList.toggle("is-open", open);

    const head = document.createElement("button");
    head.className = "app-branch-picker__groupHead";
    head.type = "button";
    head.setAttribute("aria-expanded", open ? "true" : "false");

    const icon = document.createElement("span");
    icon.className = "app-branch-picker__groupIcon";
    icon.textContent = open ? "▼" : "▶";
    head.appendChild(icon);

    const title = document.createElement("span");
    title.className = "app-branch-picker__groupTitle";
    title.textContent = group.title;
    head.appendChild(title);

    const count = document.createElement("span");
    count.className = "app-branch-picker__groupCount";
    count.textContent = branchCountLabel(group.items.length);
    head.appendChild(count);

    head.onclick = () => {
      BRANCH_GROUP_OPEN[group.key] = !open;
      renderBranchList();
    };
    section.appendChild(head);

    if (open) {
      const items = document.createElement("div");
      items.className = "app-branch-picker__groupItems";

      for (const item of group.items) {
        const b = document.createElement("button");
        b.className = "btn groupBtn app-branch-picker__item";
        if (item.current) {
          b.classList.add("is-current");
        }
        b.title = item.ref;

        const label = document.createElement("span");
        label.className = "app-branch-picker__label";
        label.textContent = item.label;
        b.appendChild(label);

        if (item.current) {
          const badge = document.createElement("span");
          badge.className = "app-branch-picker__badge";
          badge.textContent = getUIText("branch_current", "Current");
          b.appendChild(badge);
        }

        b.onclick = () => onSelectBranch(item);
        items.appendChild(b);
      }

      section.appendChild(items);
    }

    appBranchPickerList.appendChild(section);
  }
}

async function onSelectBranch(item) {
  const branch = String(item?.ref || item || "").trim();
  closeBranchPicker(true);
  if (!await appConfirm((UI_STRINGS[LANG].checkout_confirm || "Switch to this branch?") + `\n\n${branch}`, {
    title: "git checkout",
  })) return;

  try {
    await runTool("git_checkout", item?.checkoutPayload || { branch });
    toolsLogNotice(UI_STRINGS[LANG].branch_changed || "Branch changed.", { label: "git_checkout" });
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
    toolsLogNotice(UI_STRINGS[LANG].rebooting || "Rebooting...", { label: "reboot" });
  } catch (e) {
    showError("reboot", e);
  }
}

/* ---------- Logs / Dashcam ---------- */
const dashcamState = {
  initialized: false,
  loading: false,
  routes: [],
  expanded: new Set(),
  selected: new Set(),
  refreshTimer: null,
  scrollBusy: false,
  scrollTimer: null,
  loadSeq: 0,
  layoutBound: false,
  layoutTimer: null,
  landscape: null,
  signature: "",
};

const screenrecordState = {
  initialized: false,
  loading: false,
  videos: [],
  loadSeq: 0,
  signature: "",
};

let logsActiveTab = "dashcam";
let logsLazyImageObserver = null;

function dashcamSegmentIndex(segment) {
  const parts = String(segment || "").split("--");
  const n = Number.parseInt(parts[parts.length - 1] || "0", 10);
  return Number.isFinite(n) ? n : 0;
}

function dashcamRouteTitle(route) {
  return String(route || "").replace(/^0+(?=\d{3})/, "");
}

function dashcamApiPath(kind, segment) {
  return `/api/dashcam/${kind}/${encodeURIComponent(segment)}`;
}

function setDashcamStatus(message, tone = "") {
  const status = document.getElementById("dashcamStatus");
  if (!status) return;
  status.textContent = message || "";
  status.hidden = !message;
  status.classList.toggle("is-error", tone === "error");
}

function setDashcamMeta(message) {
  const meta = document.getElementById("dashcamMeta");
  if (meta) meta.textContent = message;
}

function setScreenrecordStatus(message, tone = "") {
  const status = document.getElementById("screenrecordStatus");
  if (!status) return;
  status.textContent = message || "";
  status.hidden = !message;
  status.classList.toggle("is-error", tone === "error");
}

function formatLogBytes(bytes) {
  const n = Number(bytes) || 0;
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} KB`;
  if (n < 1024 * 1024 * 1024) return `${(n / (1024 * 1024)).toFixed(1)} MB`;
  return `${(n / (1024 * 1024 * 1024)).toFixed(1)} GB`;
}

function screenrecordApiPath(kind, fileId) {
  return `/api/screenrecord/${kind}/${encodeURIComponent(fileId)}`;
}

function dashcamRoutesSignature(routes) {
  return (routes || []).map((entry) => [
    entry.route || "",
    entry.latestModifiedLabel || "",
    ...(entry.segmentFolders || []),
  ].join("|")).join("\n");
}

function screenrecordVideosSignature(videos) {
  return (videos || []).map((video) => [
    video.id || "",
    video.name || "",
    video.modifiedLabel || video.relativeModifiedLabel || "",
    video.size || 0,
  ].join("|")).join("\n");
}

function loadLogsLazyImage(img) {
  if (!img) return;
  const src = img.dataset?.src || "";
  if (!src) return;
  img.src = src;
  img.removeAttribute("data-src");
}

function hydrateLogsLazyImages(root) {
  const scope = root || document;
  const images = Array.from(scope.querySelectorAll?.("img[data-src]") || []);
  if (!images.length) return;

  if (!("IntersectionObserver" in window)) {
    images.forEach(loadLogsLazyImage);
    return;
  }

  if (!logsLazyImageObserver) {
    logsLazyImageObserver = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (!entry.isIntersecting) return;
        logsLazyImageObserver.unobserve(entry.target);
        loadLogsLazyImage(entry.target);
      });
    }, { root: null, rootMargin: "720px 0px", threshold: 0.01 });
  }

  images.forEach((img) => logsLazyImageObserver.observe(img));
}

function logsLoadingSkeletonHtml(type = "dashcam") {
  const count = type === "screen" ? 6 : 4;
  const itemClass = type === "screen" ? "logs-loading-row" : "logs-loading-card";
  return `<div class="logs-loading-list" aria-hidden="true">${Array.from({ length: count }, (_, i) =>
    `<div class="${itemClass}" style="--i:${i}"></div>`
  ).join("")}</div>`;
}

function dashcamSelectedForRoute(entry) {
  return (entry.segmentFolders || []).filter((segment) => dashcamState.selected.has(segment));
}

function dashcamRouteCardHtml(entry, index = 0, options = {}) {
  const animate = options.animate !== false;
  const route = String(entry.route || "");
  const segments = Array.isArray(entry.segmentFolders) ? entry.segmentFolders : [];
  const expanded = dashcamState.expanded.has(route);
  const compactSegments = isCompactLandscapeMode();
  const shouldRenderSegments = expanded || compactSegments;
  const selected = dashcamSelectedForRoute(entry);
  const allSelected = segments.length > 0 && selected.length === segments.length;
  const representative = segments[0] || "";
  const routeAttr = escapeHtml(route);
  const title = escapeHtml(entry.title || dashcamRouteTitle(route));
  const dateLabel = escapeHtml(entry.dateLabel || route);
  const latest = escapeHtml(entry.latestModifiedLabel || "-");
  const preview = representative
    ? `<div class="dashcam-route-media">
        <div class="dashcam-route-preview" data-action="play" data-route="${routeAttr}" data-segment="${escapeHtml(representative)}">
          <img class="logs-lazy-img" loading="lazy" decoding="async" fetchpriority="low" data-src="${dashcamApiPath("preview", representative)}" data-fallback="${dashcamApiPath("thumbnail", representative)}" onerror="this.onerror=null;if(this.dataset.fallback)this.src=this.dataset.fallback;" alt="">
          <div class="dashcam-route-preview__shade"></div>
          <div class="dashcam-route-preview__chips">
            <span class="dashcam-chip">세그먼트 ${segments.length}개</span>
            <span class="dashcam-chip">${latest}</span>
          </div>
          <div class="dashcam-play-mark" aria-hidden="true">
            <svg viewBox="0 0 24 24"><path fill="currentColor" d="M8 5v14l11-7z"/></svg>
          </div>
        </div>
        <div class="dashcam-route-media-info" data-action="toggle-route" data-route="${routeAttr}">
          <div class="dashcam-route-title">${title}</div>
          <div class="dashcam-route-subtitle">${dateLabel}</div>
        </div>
      </div>`
    : "";
  const segmentList = shouldRenderSegments ? segments.map((segment, segmentIndex) => {
    const segAttr = escapeHtml(segment);
    const checked = dashcamState.selected.has(segment) ? " checked" : "";
    if (compactSegments) {
      return `<div class="dashcam-segment-tile dashcam-segment-tile--compact ui-stagger-item" style="--i:${segmentIndex}" data-action="play" data-route="${routeAttr}" data-segment="${segAttr}">
        <div class="dashcam-segment-thumb dashcam-segment-thumb--compact">
          <img class="logs-lazy-img" loading="lazy" decoding="async" fetchpriority="low" data-src="${dashcamApiPath("thumbnail", segment)}" alt="">
          <label class="dashcam-segment-check dashcam-segment-check--compact" title="선택" onclick="event.stopPropagation()">
            <input type="checkbox" data-action="select-segment" data-segment="${segAttr}"${checked}>
          </label>
        </div>
        <div class="dashcam-segment-body">
          <div class="dashcam-segment-badge">SEG ${dashcamSegmentIndex(segment)}</div>
          <div class="dashcam-segment-name">${segAttr}</div>
        </div>
        <button class="dashcam-menu-btn" type="button" data-action="segment-menu" data-route="${routeAttr}" data-segment="${segAttr}" aria-label="세그먼트 메뉴" title="세그먼트 메뉴">
          <svg viewBox="0 0 24 24"><path fill="currentColor" d="M12 8a2 2 0 1 0 0-4 2 2 0 0 0 0 4m0 2a2 2 0 1 0 0 4 2 2 0 0 0 0-4m0 6a2 2 0 1 0 0 4 2 2 0 0 0 0-4"/></svg>
        </button>
      </div>`;
    }
    return `<div class="dashcam-segment-tile ui-stagger-item" style="--i:${segmentIndex}" data-action="play" data-route="${routeAttr}" data-segment="${segAttr}">
      <div class="dashcam-segment-thumb">
        <img class="logs-lazy-img" loading="lazy" decoding="async" fetchpriority="low" data-src="${dashcamApiPath("thumbnail", segment)}" alt="">
        <label class="dashcam-segment-check" title="선택" onclick="event.stopPropagation()">
          <input type="checkbox" data-action="select-segment" data-segment="${segAttr}"${checked}>
        </label>
      </div>
      <div class="dashcam-segment-body">
        <div class="dashcam-segment-badge">SEG ${dashcamSegmentIndex(segment)}</div>
        <div class="dashcam-segment-name">${segAttr}</div>
      </div>
      <button class="dashcam-menu-btn" type="button" data-action="segment-menu" data-route="${routeAttr}" data-segment="${segAttr}" aria-label="세그먼트 메뉴" title="세그먼트 메뉴">
        <svg viewBox="0 0 24 24"><path fill="currentColor" d="M12 8a2 2 0 1 0 0-4 2 2 0 0 0 0 4m0 2a2 2 0 1 0 0 4 2 2 0 0 0 0-4m0 6a2 2 0 1 0 0 4 2 2 0 0 0 0-4"/></svg>
      </button>
    </div>`;
  }).join("") : "";

  return `<article class="dashcam-route-card${animate ? " ui-stagger-item" : ""}"${animate ? ` style="--i:${index}"` : ""} data-route-card="${routeAttr}">
    ${preview}
    <div class="dashcam-route-main">
      <div class="dashcam-route-head" data-action="toggle-route" data-route="${routeAttr}">
        <div class="dashcam-route-titleblock">
          <div class="dashcam-route-title">${title}</div>
          <div class="dashcam-route-subtitle">${dateLabel}</div>
        </div>
        <button class="dashcam-expand-btn" type="button" data-action="toggle-route" data-route="${routeAttr}" aria-expanded="${expanded ? "true" : "false"}" title="${expanded ? "접기" : "세그먼트 보기"}">
          <svg viewBox="0 0 24 24"><path fill="currentColor" d="${expanded ? "M7.41 15.41 12 10.83l4.59 4.58L18 14l-6-6-6 6z" : "M7.41 8.59 12 13.17l4.59-4.58L18 10l-6 6-6-6z"}"/></svg>
        </button>
      </div>
      <div class="dashcam-segments ${expanded ? "" : "is-collapsed"}">
        <div class="dashcam-selection-row">
          <span class="dashcam-selection-count">선택 ${selected.length}개</span>
          <button class="smallBtn" type="button" data-action="select-route" data-route="${routeAttr}" data-selected="${allSelected ? "1" : "0"}">${allSelected ? "전체 해제" : "전체 선택"}</button>
          <button class="smallBtn btn--filled" type="button" data-action="upload-selected" data-route="${routeAttr}" ${selected.length ? "" : "disabled"}>선택 전송</button>
        </div>
        <div class="dashcam-segment-list">${segmentList}</div>
      </div>
    </div>
  </article>`;
}

function renderDashcamRoutes() {
  const host = document.getElementById("dashcamRoutes");
  if (!host) return;
  const routes = dashcamState.routes || [];
  if (dashcamState.loading && !routes.length) {
    setDashcamStatus("");
    host.innerHTML = logsLoadingSkeletonHtml("dashcam");
    return;
  }
  if (!routes.length) {
    host.innerHTML = "";
    setDashcamStatus("주행 기록이 없습니다.");
    return;
  }
  setDashcamStatus("");
  host.innerHTML = routes.map((entry, index) => dashcamRouteCardHtml(entry, index, { animate: false })).join("");
  hydrateLogsLazyImages(host);
}

function renderDashcamRoute(route) {
  const host = document.getElementById("dashcamRoutes");
  if (!host) return false;
  const routes = dashcamState.routes || [];
  const index = routes.findIndex((entry) => entry.route === route);
  if (index < 0) return false;

  const current = Array.from(host.querySelectorAll("[data-route-card]"))
    .find((node) => node.dataset.routeCard === route);
  if (!current) return false;

  const tpl = document.createElement("template");
  tpl.innerHTML = dashcamRouteCardHtml(routes[index], index, { animate: false });
  const nextMain = tpl.content.querySelector(".dashcam-route-main");
  const currentMain = current.querySelector(".dashcam-route-main");
  if (!nextMain || !currentMain) return false;

  currentMain.replaceWith(nextMain);
  hydrateLogsLazyImages(nextMain);
  return true;
}

function updateDashcamRouteSelectionUi(route) {
  const host = document.getElementById("dashcamRoutes");
  if (!host) return false;
  const entry = (dashcamState.routes || []).find((item) => item.route === route);
  if (!entry) return false;

  const card = Array.from(host.querySelectorAll("[data-route-card]"))
    .find((node) => node.dataset.routeCard === route);
  if (!card) return false;

  const segments = Array.isArray(entry.segmentFolders) ? entry.segmentFolders : [];
  const selected = dashcamSelectedForRoute(entry);
  const allSelected = segments.length > 0 && selected.length === segments.length;

  const countEl = card.querySelector(".dashcam-selection-count");
  if (countEl) countEl.textContent = `선택 ${selected.length}개`;

  const selectBtn = card.querySelector('[data-action="select-route"]');
  if (selectBtn) {
    selectBtn.dataset.selected = allSelected ? "1" : "0";
    selectBtn.textContent = allSelected ? "전체 해제" : "전체 선택";
  }

  const uploadBtn = card.querySelector('[data-action="upload-selected"]');
  if (uploadBtn) uploadBtn.disabled = selected.length === 0;

  card.querySelectorAll('input[data-action="select-segment"]').forEach((input) => {
    const segment = input.dataset.segment || "";
    input.checked = dashcamState.selected.has(segment);
  });

  return true;
}

async function loadDashcamRoutes({ silent = false } = {}) {
  const seq = ++dashcamState.loadSeq;
  if (!silent) {
    dashcamState.loading = true;
    renderDashcamRoutes();
  }
  try {
    const json = await getJson("/api/dashcam/routes");
    if (seq !== dashcamState.loadSeq) return;
    const routes = Array.isArray(json.routes) ? json.routes : [];
    const nextSignature = dashcamRoutesSignature(routes);
    if (silent && nextSignature === dashcamState.signature) {
      dashcamState.loading = false;
      return;
    }
    const validRoutes = new Set(routes.map((entry) => entry.route));
    const validSegments = new Set(routes.flatMap((entry) => entry.segmentFolders || []));
    dashcamState.expanded = new Set(Array.from(dashcamState.expanded).filter((route) => validRoutes.has(route)));
    dashcamState.selected = new Set(Array.from(dashcamState.selected).filter((segment) => validSegments.has(segment)));
    dashcamState.routes = routes;
    dashcamState.signature = nextSignature;
    dashcamState.loading = false;
    renderDashcamRoutes();
  } catch (e) {
    if (seq !== dashcamState.loadSeq) return;
    dashcamState.loading = false;
    if (!silent) {
      setDashcamStatus(`대시캠 목록 로드 실패: ${e.message || e}`, "error");
      showAppToast(e.message || "대시캠 목록 로드 실패", { tone: "error" });
    }
  }
}

function startDashcamAutoRefresh() {
  if (dashcamState.refreshTimer) return;
  dashcamState.refreshTimer = window.setInterval(() => {
    if (CURRENT_PAGE !== "logs" || dashcamState.scrollBusy) return;
    if (logsActiveTab === "screen") loadScreenrecordVideos({ silent: true }).catch(() => {});
    else loadDashcamRoutes({ silent: true }).catch(() => {});
  }, 10000);
}

function markDashcamScrollBusy() {
  dashcamState.scrollBusy = true;
  if (dashcamState.scrollTimer) window.clearTimeout(dashcamState.scrollTimer);
  dashcamState.scrollTimer = window.setTimeout(() => {
    dashcamState.scrollBusy = false;
  }, 380);
}

function openLogsVideoPlayer(title, src) {
  const overlay = document.createElement("div");
  overlay.className = "dashcam-player-overlay";
  overlay.innerHTML = `<div class="dashcam-player-dialog" role="dialog" aria-modal="true">
    <div class="dashcam-player-frame">
      <video class="dashcam-player-video" autoplay controls playsinline src="${src}"></video>
      <div class="dashcam-player-top">
        <div class="dashcam-player-title">${escapeHtml(title || "Video")}</div>
        <button class="dashcam-player-close" type="button" aria-label="닫기" title="닫기">
          <svg viewBox="0 0 24 24"><path fill="currentColor" d="M18.3 5.71 12 12l6.3 6.29-1.41 1.41L10.59 13.41 4.29 19.71 2.88 18.3 9.17 12 2.88 5.7 4.29 4.29l6.3 6.3 6.29-6.3z"/></svg>
        </button>
      </div>
    </div>
  </div>`;
  const close = () => {
    const video = overlay.querySelector("video");
    try { video?.pause?.(); } catch {}
    overlay.remove();
  };
  overlay.addEventListener("click", (ev) => {
    if (ev.target === overlay) close();
  });
  overlay.querySelector(".dashcam-player-close")?.addEventListener("click", close);
  document.body.appendChild(overlay);
  requestAnimationFrame(() => overlay.classList.add("is-open"));
}

function openDashcamPlayer(route, segment) {
  openLogsVideoPlayer(
    `${dashcamRouteTitle(route)} · Segment ${dashcamSegmentIndex(segment)}`,
    dashcamApiPath("video", segment),
  );
}

function openScreenrecordPlayer(id, name) {
  if (!id) return;
  openLogsVideoPlayer(name || "화면녹화", screenrecordApiPath("video", id));
}

function dashcamUploadResultHtml(result) {
  const text = String(result?.shareText || result?.message || "");
  const discord = result?.discord || {};
  let discordLabel = "Discord: 설정 없음";
  let discordClass = "is-muted";
  if (discord.configured && discord.ok) {
    discordLabel = "Discord: 전송 완료";
    discordClass = "is-ok";
  } else if (discord.configured) {
    discordLabel = `Discord: 실패${discord.status ? ` (${discord.status})` : ""}`;
    discordClass = "is-error";
  }
  return `<div class="dashcam-share-card">
    <div class="dashcam-share-card__summary">
      <span>업로드 ${Number(result?.uploaded || 0)}/${Number(result?.total || 0)}</span>
      <span class="${discordClass}">${escapeHtml(discordLabel)}</span>
    </div>
    <pre>${escapeHtml(text)}</pre>
  </div>`;
}

async function showDashcamUploadResult(result) {
  const text = String(result?.shareText || result?.message || "").trim();
  const selected = await openAppDialog({
    mode: "choice",
    title: "로그 전송 결과",
    html: true,
    messageHtml: `<div class="dashcam-share-dialog">${dashcamUploadResultHtml(result)}</div>`,
    cancelLabel: "닫기",
    choices: [
      { label: "복사", value: "copy", className: "btn--filled" },
    ],
  });
  if (selected === "copy") {
    copyToClipboard(text);
    showAppToast("복사되었습니다.");
  }
}

function openDashcamUploadProgress(total) {
  const overlay = document.createElement("div");
  overlay.className = "dashcam-upload-progress";
  overlay.innerHTML = `<div class="dashcam-upload-progress__sheet" role="dialog" aria-modal="true">
    <div class="dashcam-upload-progress__title">로그 전송 중</div>
    <div class="dashcam-upload-progress__message">0/${Number(total || 0)}</div>
    <div class="dashcam-upload-progress__bar" aria-hidden="true"><span></span></div>
  </div>`;
  document.body.appendChild(overlay);
  document.body.classList.add("dialog-open");
  requestAnimationFrame(() => overlay.classList.add("is-open"));
  return {
    close() {
      overlay.classList.remove("is-open");
      window.setTimeout(() => {
        overlay.remove();
        syncModalBodyLock();
      }, 160);
    },
  };
}

async function uploadDashcamSegments(segments) {
  const targets = Array.from(new Set(segments || [])).filter(Boolean);
  if (!targets.length) {
    showAppToast("선택된 세그먼트가 없습니다.", { tone: "error" });
    return;
  }
  const ok = await appConfirm(`당근 서버에 ${targets.length}개 로그를 전송할까요?`, { title: "로그 전송" });
  if (!ok) return;
  const progress = openDashcamUploadProgress(targets.length);
  try {
    const result = await postJson("/api/dashcam/upload", { segments: targets });
    const message = result.message || `전송 완료 ${result.uploaded || 0}/${result.total || targets.length}`;
    showAppToast(message, { tone: result.ok ? "default" : "error", duration: 3600 });
    progress.close();
    await showDashcamUploadResult(result);
  } catch (e) {
    progress.close();
    showAppToast(`로그 전송 실패: ${e.message || e}`, { tone: "error", duration: 4200 });
  }
}

async function showDashcamSegmentMenu(route, segment) {
  const selected = await openAppDialog({
    mode: "choice",
    title: `SEG ${dashcamSegmentIndex(segment)}`,
    message: segment,
    choices: [
      { label: "재생", value: "play" },
      { label: "로그 전송", value: "upload" },
      { label: "qcamera 다운로드", value: "download_qcamera" },
      { label: "rlog 다운로드", value: "download_rlog" },
      { label: "qlog 다운로드", value: "download_qlog" },
    ],
  });
  if (selected === "play") openDashcamPlayer(route, segment);
  else if (selected === "upload") await uploadDashcamSegments([segment]);
  else if (selected?.startsWith?.("download_")) {
    const kind = selected.replace("download_", "");
    window.open(dashcamApiPath(`download/${encodeURIComponent(segment)}`, kind), "_blank", "noopener");
  }
}

function screenrecordVideoRowHtml(video, index = 0) {
  const id = escapeHtml(video.id || "");
  const name = escapeHtml(video.name || "-");
  const date = escapeHtml(video.modifiedLabel || video.relativeModifiedLabel || "-");
  const size = escapeHtml(formatLogBytes(video.size));
  const ext = escapeHtml((video.ext || "video").toUpperCase());
  return `<article class="screenrecord-row ui-stagger-item" style="--i:${index}" data-action="play-screenrecord" data-id="${id}" data-name="${name}">
    <div class="screenrecord-row__thumb" aria-hidden="true">
      <img class="logs-lazy-img" loading="lazy" decoding="async" fetchpriority="low" data-src="${screenrecordApiPath("thumbnail", video.id || "")}" alt="">
    </div>
    <div class="screenrecord-row__main">
      <div class="screenrecord-row__name">${name}</div>
      <div class="screenrecord-row__meta">
        <span>${date}</span>
        <span>${size}</span>
        <span>${ext}</span>
      </div>
    </div>
    <button class="screenrecord-download" type="button" data-action="download-screenrecord" data-id="${id}" aria-label="다운로드" title="다운로드">
      <svg viewBox="0 0 24 24"><path fill="currentColor" d="M5 20h14v-2H5m14-9h-4V3H9v6H5l7 7z"/></svg>
    </button>
  </article>`;
}

function renderScreenrecordVideos() {
  const host = document.getElementById("screenrecordVideos");
  if (!host) return;
  const videos = screenrecordState.videos || [];
  if (screenrecordState.loading && !videos.length) {
    setScreenrecordStatus("");
    host.innerHTML = logsLoadingSkeletonHtml("screen");
    return;
  }
  if (!videos.length) {
    host.innerHTML = "";
    setScreenrecordStatus("화면녹화 폴더/영상이 없습니다.");
    return;
  }
  setScreenrecordStatus("");
  host.innerHTML = videos.map(screenrecordVideoRowHtml).join("");
  hydrateLogsLazyImages(host);
}

async function loadScreenrecordVideos({ silent = false } = {}) {
  const seq = ++screenrecordState.loadSeq;
  if (!silent) {
    screenrecordState.loading = true;
    renderScreenrecordVideos();
  }
  try {
    const json = await getJson("/api/screenrecord/videos");
    if (seq !== screenrecordState.loadSeq) return;
    const videos = Array.isArray(json.videos) ? json.videos : [];
    const nextSignature = screenrecordVideosSignature(videos);
    if (silent && nextSignature === screenrecordState.signature) {
      screenrecordState.loading = false;
      return;
    }
    screenrecordState.videos = videos;
    screenrecordState.signature = nextSignature;
    screenrecordState.loading = false;
    renderScreenrecordVideos();
  } catch (e) {
    if (seq !== screenrecordState.loadSeq) return;
    screenrecordState.loading = false;
    if (!silent) {
      setScreenrecordStatus(`화면녹화 목록 로드 실패: ${e.message || e}`, "error");
      showAppToast(e.message || "화면녹화 목록 로드 실패", { tone: "error" });
    }
  }
}

function activateLogsTab(tab) {
  logsActiveTab = tab === "screen" ? "screen" : "dashcam";
  const dashTab = document.getElementById("logsTabDashcam");
  const screenTab = document.getElementById("logsTabScreen");
  const dashPanel = document.getElementById("logsDashcamPanel");
  const screenPanel = document.getElementById("logsScreenPanel");

  dashTab?.classList.toggle("is-active", logsActiveTab === "dashcam");
  screenTab?.classList.toggle("is-active", logsActiveTab === "screen");
  dashTab?.setAttribute("aria-selected", logsActiveTab === "dashcam" ? "true" : "false");
  screenTab?.setAttribute("aria-selected", logsActiveTab === "screen" ? "true" : "false");
  if (dashPanel) dashPanel.hidden = logsActiveTab !== "dashcam";
  if (screenPanel) screenPanel.hidden = logsActiveTab !== "screen";

  if (logsActiveTab === "screen" && !screenrecordState.initialized) {
    screenrecordState.initialized = true;
    loadScreenrecordVideos().catch(() => {});
  } else if (logsActiveTab === "screen") {
    renderScreenrecordVideos();
    loadScreenrecordVideos({ silent: true }).catch(() => {});
  } else if (dashcamState.initialized) {
    loadDashcamRoutes({ silent: true }).catch(() => {});
  }
}

function bindLogsPage() {
  const dashTab = document.getElementById("logsTabDashcam");
  const screenTab = document.getElementById("logsTabScreen");
  const routesHost = document.getElementById("dashcamRoutes");
  const screenHost = document.getElementById("screenrecordVideos");

  if (!dashcamState.layoutBound) {
    dashcamState.layoutBound = true;
    dashcamState.landscape = isCompactLandscapeMode();
    window.addEventListener("resize", () => {
      if (CURRENT_PAGE !== "logs") return;
      if (dashcamState.layoutTimer) window.clearTimeout(dashcamState.layoutTimer);
      dashcamState.layoutTimer = window.setTimeout(() => {
        const nextLandscape = isCompactLandscapeMode();
        if (dashcamState.landscape === nextLandscape) return;
        dashcamState.landscape = nextLandscape;
        renderDashcamRoutes();
      }, 120);
    }, { passive: true });
  }

  if (dashTab && dashTab.dataset.bound !== "1") {
    dashTab.dataset.bound = "1";
    dashTab.addEventListener("click", () => activateLogsTab("dashcam"));
  }

  if (screenTab && screenTab.dataset.bound !== "1") {
    screenTab.dataset.bound = "1";
    screenTab.addEventListener("click", () => activateLogsTab("screen"));
  }

  if (routesHost && routesHost.dataset.bound !== "1") {
    routesHost.dataset.bound = "1";
    routesHost.addEventListener("scroll", markDashcamScrollBusy, { passive: true });
    routesHost.addEventListener("click", (ev) => {
      const actionEl = ev.target?.closest?.("[data-action]");
      if (!actionEl) return;
      const action = actionEl.dataset.action;
      const route = actionEl.dataset.route || "";
      const segment = actionEl.dataset.segment || "";
      if (action === "toggle-route") {
        if (dashcamState.expanded.has(route)) dashcamState.expanded.delete(route);
        else dashcamState.expanded.add(route);
        if (!renderDashcamRoute(route)) renderDashcamRoutes();
      } else if (action === "play") {
        openDashcamPlayer(route, segment);
      } else if (action === "segment-menu") {
        ev.stopPropagation();
        showDashcamSegmentMenu(route, segment).catch(() => {});
      } else if (action === "select-route") {
        const entry = dashcamState.routes.find((item) => item.route === route);
        if (!entry) return;
        const shouldClear = actionEl.dataset.selected === "1";
        for (const item of entry.segmentFolders || []) {
          if (shouldClear) dashcamState.selected.delete(item);
          else dashcamState.selected.add(item);
        }
        if (!updateDashcamRouteSelectionUi(route)) renderDashcamRoutes();
      } else if (action === "upload-selected") {
        const entry = dashcamState.routes.find((item) => item.route === route);
        const targets = dashcamSelectedForRoute(entry || { segmentFolders: [] });
        uploadDashcamSegments(targets).catch(() => {});
      }
    });
    routesHost.addEventListener("change", (ev) => {
      const input = ev.target;
      if (!input?.matches?.('input[data-action="select-segment"]')) return;
      const segment = input.dataset.segment || "";
      if (input.checked) dashcamState.selected.add(segment);
      else dashcamState.selected.delete(segment);
      const route = input.closest("[data-route-card]")?.dataset.routeCard || "";
      if (!updateDashcamRouteSelectionUi(route)) renderDashcamRoutes();
    });
  }

  if (screenHost && screenHost.dataset.bound !== "1") {
    screenHost.dataset.bound = "1";
    screenHost.addEventListener("scroll", markDashcamScrollBusy, { passive: true });
    screenHost.addEventListener("click", (ev) => {
      const actionEl = ev.target?.closest?.("[data-action]");
      if (!actionEl) return;
      if (actionEl.dataset.action === "download-screenrecord") {
        const id = actionEl.dataset.id || "";
        if (id) window.open(screenrecordApiPath("download", id), "_blank", "noopener");
      } else if (actionEl.dataset.action === "play-screenrecord") {
        openScreenrecordPlayer(actionEl.dataset.id || "", actionEl.dataset.name || "");
      }
    });
  }
}

function initLogsPage() {
  bindLogsPage();
  activateLogsTab(logsActiveTab);
  startDashcamAutoRefresh();
  if (!dashcamState.initialized) {
    dashcamState.initialized = true;
    loadDashcamRoutes().catch(() => {});
  } else {
    renderDashcamRoutes();
    loadDashcamRoutes({ silent: true }).catch(() => {});
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
