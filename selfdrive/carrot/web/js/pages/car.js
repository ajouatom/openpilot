"use strict";

// Car page + Drive-page record FAB + currentCar status (cross-cutting with setting page).

const CURRENT_CAR_CACHE_KEY = "carrot_web_current_car_label";
const CURRENT_CAR_PROMPT_SESSION_KEY = "carrot_web_missing_car_prompted";
const CURRENT_CAR_RETRY_DELAYS_MS = [350, 800, 1500, 2500, 4000];

let recordStateIsOn = false;
let recordTogglePending = false;
let recordStateResyncTimer = null;
let currentCarRetryTimer = null;
let currentCarRetryIndex = 0;
let currentCarLastKnownLabel = "";
let currentCarLoadPromise = null;
let currentCarLoadedAt = 0;
let currentCarHasSnapshot = false;
let currentCarPromptActive = false;
let recordStateLoadPromise = null;
let recordStateLoadedAt = 0;
let carsLoadPromise = null;

let carPickerCloseTimer = null;
let carPickerMode = "makers";
let carPickerMaker = null;

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
        if (isMissingCarSelectionValues(values)) {
          window.setTimeout(() => {
            promptMissingCurrentCarSelection(values).catch(() => {});
          }, 350);
        }
      } else {
        applyCurrentCarLabel("", { blank: !currentCarLastKnownLabel });
        scheduleCurrentCarRetry();
        window.setTimeout(() => {
          promptMissingCurrentCarSelection(values).catch(() => {});
        }, 350);
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
  appCarPickerMeta.textContent = getUIText("loading", "Loading...");
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

  carMeta.textContent = getUIText("loading", "Loading...");
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
