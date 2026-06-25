"use strict";

// Device tab coordinator. Rendering, network refresh, and action handlers live
// in adjacent setting_device_* files to keep this file focused on tab state.

let CURRENT_SETTING_TAB = "carrot";
let CURRENT_DEVICE_GROUP = "Device";
let deviceParamValues = {};
let deviceGroupLoadPromises = new Map();
let deviceNetworkInfo = null;
let deviceNetworkLoadPromise = null;
let deviceSshStatus = null;
let deviceSshRefreshTimer = null;
let deviceSshRefreshInFlight = false;
let deviceTabLoaded = false;

function mergeDeviceParamValues(values) {
  if (values && typeof values === "object") {
    deviceParamValues = { ...deviceParamValues, ...values };
  }
  return deviceParamValues;
}

function getDeviceTypeValue(values = deviceParamValues) {
  return String(values.DeviceType || "unknown").trim().toLowerCase();
}

function getVisibleDeviceGroups() {
  return DEVICE_GROUPS.filter((group) => {
    if (group.id !== "Software") return true;
    return isTruthyDeviceFlag(deviceParamValues.SoftwareMenu ?? 1);
  });
}

function getDeviceGroupLabel(groupId) {
  const group = getVisibleDeviceGroups().find((entry) => entry.id === groupId) || DEVICE_GROUPS.find((entry) => entry.id === groupId);
  return getUIText(group?.labelKey || groupId, group?.defaultLabel || groupId);
}

function getCurrentSettingTab() {
  return CURRENT_SETTING_TAB;
}

function setSettingDeviceHidden(el, hidden) {
  if (!el) return;
  el.hidden = hidden;
  el.style.display = hidden ? "none" : "";
}

function syncSettingTabChrome(tab = CURRENT_SETTING_TAB) {
  const isDevice = tab === "device";
  const page = document.getElementById("pageSetting");
  if (page) page.classList.toggle("setting-tab-device", isDevice);

  if (settingTabDevice) {
    settingTabDevice.classList.toggle("is-active", isDevice);
    settingTabDevice.setAttribute("aria-selected", isDevice ? "true" : "false");
  }
  if (settingTabCarrot) {
    settingTabCarrot.classList.toggle("is-active", !isDevice);
    settingTabCarrot.setAttribute("aria-selected", isDevice ? "false" : "true");
  }
}

function syncSettingTabPanels(tab = CURRENT_SETTING_TAB) {
  const isDevice = tab === "device";
  const carrotTabContent = document.getElementById("carrotTabContent");
  const items = document.getElementById("items");
  const deviceItems = document.getElementById("deviceItems");

  setSettingDeviceHidden(carrotTabContent, isDevice);
  setSettingDeviceHidden(deviceTabContent, !isDevice);
  setSettingDeviceHidden(settingSubnav, isDevice);
  setSettingDeviceHidden(items, isDevice);
  setSettingDeviceHidden(deviceItems, !isDevice);
}

function syncSettingTabState(tab = CURRENT_SETTING_TAB) {
  CURRENT_SETTING_TAB = tab === "device" ? "device" : "carrot";
  syncSettingTabChrome(CURRENT_SETTING_TAB);
  syncSettingTabPanels(CURRENT_SETTING_TAB);
}

async function loadDeviceParams(groupId, force = false) {
  if (!force && deviceGroupLoadPromises.has(groupId)) return deviceGroupLoadPromises.get(groupId);

  const promise = (async () => {
    let values = {};
    if (groupId === "Device") {
      values = await bulkGet(DEVICE_INFO_PARAMS);
    } else if (groupId === "Software") {
      values = await bulkGet(DEVICE_SOFTWARE_PARAMS);
    } else if (groupId === "Toggles") {
      const names = DEVICE_TOGGLES.map((entry) => entry.param);
      DEVICE_TOGGLES.forEach((entry) => {
        if (entry.confirmedParam) names.push(entry.confirmedParam);
      });
      names.push("LongitudinalPersonality");
      values = await bulkGet(names);
    } else if (groupId === "Developer") {
      values = await bulkGet([
        ...DEVICE_DEVELOPER_TOGGLES.map((entry) => entry.param),
        "GithubUsername",
        "GithubSshKeys",
      ]);
      values.SshKeyStatus = await loadDeviceSshStatus(false);
    }
    return mergeDeviceParamValues(values);
  })().catch((err) => {
    console.error("[DeviceTab]", err);
    return deviceParamValues;
  }).finally(() => {
    deviceGroupLoadPromises.delete(groupId);
  });

  deviceGroupLoadPromises.set(groupId, promise);
  return promise;
}

async function loadDeviceNetwork(useCache = true) {
  if (deviceNetworkInfo && useCache) return deviceNetworkInfo;
  if (deviceNetworkLoadPromise) return deviceNetworkLoadPromise;

  deviceNetworkLoadPromise = requestJson("/api/device_network", { cache: "no-store" })
    .then((payload) => {
      deviceNetworkInfo = payload.network || {};
      return deviceNetworkInfo;
    })
    .catch((err) => {
      console.error("[DeviceTab]", err);
      return deviceNetworkInfo || {};
    })
    .finally(() => {
      deviceNetworkLoadPromise = null;
    });

  return deviceNetworkLoadPromise;
}

function renderDeviceGroups(options = {}) {
  const groupContainer = document.getElementById("deviceGroupList");
  if (!groupContainer) return;
  const animateGroups = options.animateGroups !== false;

  const visibleGroups = getVisibleDeviceGroups();
  if (!visibleGroups.some((group) => group.id === CURRENT_DEVICE_GROUP)) {
    CURRENT_DEVICE_GROUP = visibleGroups[0]?.id || "Device";
  }
  const groupEntries = visibleGroups.map((group) => ({
    group,
    label: getDeviceGroupLabel(group.id),
  }));
  const signature = groupEntries.map((entry) => `${entry.group.id}:${entry.label}`).join("|");

  if (
    !animateGroups &&
    groupContainer.dataset.deviceGroupsSignature === signature &&
    groupContainer.children.length === groupEntries.length
  ) {
    Array.from(groupContainer.children).forEach((button, index) => {
      const entry = groupEntries[index];
      button.className = "btn groupBtn";
      if (entry.group.id === CURRENT_DEVICE_GROUP) button.classList.add("active");
      button.dataset.deviceGroup = entry.group.id;
      button.innerHTML = `<span class="setting-group-label">${escapeHtml(entry.label)}</span>`;
      button.onclick = () => selectDeviceGroup(entry.group.id);
    });
    if (typeof scheduleSettingOverflowSync === "function") scheduleSettingOverflowSync(groupContainer);
    return;
  }

  groupContainer.innerHTML = "";
  groupContainer.dataset.deviceGroupsSignature = signature;

  groupEntries.forEach((entry, index) => {
    const group = entry.group;
    const label = entry.label;
    const button = document.createElement("button");
    button.type = "button";
    button.className = animateGroups ? "btn groupBtn ui-stagger-item" : "btn groupBtn";
    if (animateGroups) button.style.setProperty("--i", String(index));
    if (group.id === CURRENT_DEVICE_GROUP) button.classList.add("active");
    button.dataset.deviceGroup = group.id;
    button.innerHTML = `<span class="setting-group-label">${escapeHtml(label)}</span>`;
    button.onclick = () => selectDeviceGroup(group.id);
    groupContainer.appendChild(button);
  });
  if (typeof scheduleSettingOverflowSync === "function") scheduleSettingOverflowSync(groupContainer);
}

function applyDeviceItemsStagger(container) {
  if (!container) return;
  // Stagger the section-block card(s) like the CarrotPilot tab. Falls back to
  // direct .setting children if items aren't card-wrapped (defensive).
  const blocks = container.querySelectorAll(".setting-section-block");
  const targets = blocks.length
    ? Array.from(blocks)
    : Array.from(container.children).filter((c) => c.classList?.contains("setting"));
  targets.forEach((el, index) => {
    el.classList.add("ui-stagger-item");
    el.style.setProperty("--i", String(index));
  });
}

async function renderDeviceTab(options = {}) {
  syncSettingTabState("device");
  const animateGroups = options.animateGroups !== false;
  const animateItems = options.animateItems !== false;
  renderDeviceGroups({ animateGroups });
  syncDeviceGroupChrome(CURRENT_DEVICE_GROUP);
  if (!deviceTabLoaded) {
    deviceTabLoaded = true;
    loadDeviceParams("Device", true).then(() => {
      if (CURRENT_SETTING_TAB === "device") renderDeviceGroups({ animateGroups: false });
    });
  }
  if (typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode()) {
    await renderDeviceItems(CURRENT_DEVICE_GROUP, false, { animateItems });
  }
}

async function selectDeviceGroup(groupId, pushHistory = true) {
  CURRENT_DEVICE_GROUP = groupId || CURRENT_DEVICE_GROUP;
  renderDeviceGroups();
  syncSettingTabState("device");
  syncDeviceGroupChrome(CURRENT_DEVICE_GROUP);
  // Same history-based navigation as the CarrotPilot tab: an "items" entry lets
  // the title back-chevron / device back button return to the device groups
  // screen. Skip in compact-landscape split (it always shows items).
  const splitLandscape =
    typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode() && CURRENT_PAGE === "setting";
  if (pushHistory && !splitLandscape) {
    history.pushState({ page: "setting", tab: "device", screen: "items", deviceGroup: CURRENT_DEVICE_GROUP }, "");
  }
  await renderDeviceItems(CURRENT_DEVICE_GROUP, true, { animateItems: true });
}

// Restore the device tab from a popstate without touching history (no push /
// replace) — mirrors how app.js restores the CarrotPilot tab.
async function restoreSettingDeviceTab(screen, deviceGroup) {
  if (typeof CURRENT_SETTING_TAB !== "undefined") CURRENT_SETTING_TAB = "device";
  syncSettingTabState("device");
  await renderDeviceTab({ animateGroups: false, animateItems: false });
  if (screen === "items" && deviceGroup) {
    await selectDeviceGroup(deviceGroup, false);
  } else if (typeof showSettingScreen === "function") {
    showSettingScreen("groups", false);
  }
  syncDeviceGroupChrome(CURRENT_DEVICE_GROUP);
  if (typeof syncDeviceSshRefresh === "function") syncDeviceSshRefresh();
}
window.restoreSettingDeviceTab = restoreSettingDeviceTab;

async function loadDeviceSshStatus(useCache = true) {
  if (deviceSshStatus && useCache) return deviceSshStatus;
  const payload = await requestJson("/api/ssh_keys", { cache: "no-store" });
  deviceSshStatus = {
    username: payload.username || "",
    has_keys: Boolean(payload.has_keys),
    key_count: Number(payload.key_count || 0),
    fingerprints: Array.isArray(payload.fingerprints) ? payload.fingerprints : [],
    updated_at: payload.updated_at || "",
  };
  mergeDeviceParamValues({
    GithubUsername: deviceSshStatus.username,
    GithubSshKeys: deviceSshStatus.has_keys ? "1" : "",
    SshKeyStatus: deviceSshStatus,
  });
  return deviceSshStatus;
}

function shouldRefreshDeviceSsh() {
  const deviceItems = document.getElementById("deviceItems");
  return (
    CURRENT_PAGE === "setting" &&
    CURRENT_SETTING_TAB === "device" &&
    CURRENT_DEVICE_GROUP === "Developer" &&
    !document.hidden &&
    deviceItems &&
    !deviceItems.hidden &&
    deviceItems.style.display !== "none"
  );
}

function stopDeviceSshRefresh() {
  if (!deviceSshRefreshTimer) return;
  window.clearTimeout(deviceSshRefreshTimer);
  deviceSshRefreshTimer = null;
}

function scheduleDeviceSshRefresh(delay = DEVICE_SSH_REFRESH_MS) {
  stopDeviceSshRefresh();
  if (!shouldRefreshDeviceSsh()) return;
  deviceSshRefreshTimer = window.setTimeout(() => {
    deviceSshRefreshTimer = null;
    refreshDeviceSshPanel().catch((err) => console.error("[DeviceTab]", err));
  }, delay);
}

function syncDeviceSshRefresh() {
  if (shouldRefreshDeviceSsh()) scheduleDeviceSshRefresh();
  else stopDeviceSshRefresh();
}

async function refreshDeviceSshPanel() {
  if (!shouldRefreshDeviceSsh() || deviceSshRefreshInFlight) {
    syncDeviceSshRefresh();
    return;
  }

  deviceSshRefreshInFlight = true;
  try {
    const previous = JSON.stringify(deviceSshStatus || {});
    await loadDeviceSshStatus(false);
    const next = JSON.stringify(deviceSshStatus || {});
    if (previous !== next) {
      await renderDeviceItems("Developer", false, { silentRefresh: true });
    }
  } finally {
    deviceSshRefreshInFlight = false;
    syncDeviceSshRefresh();
  }
}

async function getDeviceGroupValues(groupId) {
  if (groupId === "Network") {
    await loadDeviceNetwork(false);
    return deviceParamValues;
  }
  return loadDeviceParams(groupId, true);
}

async function renderDeviceItems(groupId, showItemsScreen = true, options = {}) {
  const itemsContainer = document.getElementById("deviceItems");
  if (!itemsContainer) return;
  const silentRefresh = options.silentRefresh === true;

  // A drill-in from the groups screen triggers the left/right screen slide.
  // Don't ALSO play the per-item rise (stagger) then — the slide + rise mix is
  // the jarring combo the user saw. CarrotPilot is slide-only in this case.
  // Detect it before the screen swaps (items screen still hidden = drill-in).
  const screenItemsEl = document.getElementById("settingScreenItems");
  const willSlide = showItemsScreen && !!screenItemsEl &&
    (screenItemsEl.style.display === "none" || screenItemsEl.classList.contains("hidden"));

  syncSettingTabState("device");
  if (showItemsScreen && typeof showSettingScreen === "function") {
    showSettingScreen("items", false);
  }

  if (!silentRefresh) {
    itemsContainer.innerHTML = `<div class="muted mt-md text-center">${escapeHtml(getUIText("loading", "Loading..."))}</div>`;
  }

  const values = await getDeviceGroupValues(groupId);

  if (CURRENT_SETTING_TAB !== "device" || CURRENT_DEVICE_GROUP !== groupId) {
    syncDeviceNetworkRefresh();
    syncDeviceSshRefresh();
    return;
  }

  // Wrap device items in the same card box the CarrotPilot tab uses
  // (setting-section-block > setting-group-card > setting-group-card__body) so
  // the device submenu looks identical, not the old flat rows.
  const deviceItemsHtml = renderDeviceGroupItems(groupId, values);
  itemsContainer.innerHTML = deviceItemsHtml
    ? `<div class="setting-section-block"><div class="setting-group-card"><div class="setting-group-card__body">${deviceItemsHtml}</div></div></div>`
    : `<div class="muted mt-md text-center">-</div>`;
  if (!silentRefresh && options.animateItems !== false && !willSlide) {
    applyDeviceItemsStagger(itemsContainer);
  }
  bindDeviceTabEvents(itemsContainer);
  syncDeviceGroupActiveState(groupId);
  syncDeviceGroupChrome(groupId);
  syncDeviceNetworkRefresh();
  syncDeviceSshRefresh();
}

function renderDeviceGroupItems(groupId, values) {
  const data = { ...deviceParamValues, ...(values || {}) };

  if (groupId === "Device") {
    let html = "";
    html += renderDeviceInfoRow(getUIText("dongle_id", "Dongle ID"), data.DongleId || "N/A");
    html += renderDeviceInfoRow(getUIText("serial", "Serial"), data.HardwareSerial || "N/A");
    html += renderDeviceActionRow(getUIText("reboot", "Reboot"), getUIText("reboot_device_desc", "Reboot device"), getUIText("reboot", "Reboot"), "btnDeviceReboot");
    html += renderDeviceActionRow(getUIText("recalibration", "ReCalibration"), "", getUIText("reset", "Reset"), "btnDeviceRecalib");
    html += renderDeviceActionRow(getUIText("power_off", "Power Off"), getUIText("power_off_desc", "Power off device"), getUIText("power_off", "Power Off"), "btnDevicePoweroff", "smallBtn btn--danger");
    html += renderDeviceActionRow(getUIText("pair_device", "Pair Device"), getUIText("pair_device_desc", "Pair your device with comma connect (connect.comma.ai) and claim your comma prime offer."), getUIText("pair", "PAIR"), "btnDevicePair", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("driver_camera", "Driver Camera"), getUIText("driver_camera_desc", "Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"), getUIText("preview", "PREVIEW"), "btnDeviceDriverCamera", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("review_training_guide", "Review Training Guide"), getUIText("review_training_desc", "Review the rules, features, and limitations of openpilot"), getUIText("review", "Review"), "btnDeviceTraining");
    html += renderDeviceActionRow(getUIText("calibration_status", "Calibration Status"), "", getUIText("show_upper", "SHOW"), "btnDeviceCalibrationStatus");
    if (getDeviceTypeValue(data) === "tici") {
      html += renderDeviceActionRow(getUIText("regulatory", "Regulatory"), "", getUIText("view_upper", "VIEW"), "btnDeviceRegulatory");
    }
    html += renderDeviceLanguageRow({
      language: data.LanguageSetting || "main_en",
      languages: DEVICE_LANGUAGES,
    });
    return html;
  }

  if (groupId === "Network") return renderNetworkPanel(deviceNetworkInfo || {});
  if (groupId === "Toggles") return renderDeviceToggleItems(data);
  if (groupId === "Developer") return renderDeviceDeveloperItems(data);
  if (groupId === "Software") {
    let html = "";
    html += renderDeviceInfoRow(getUIText("updates_offroad_only", "Updates are only downloaded while the car is off."), "");
    html += renderDeviceVersionRow(getUIText("current_version", "Current Version"), data.UpdaterCurrentDescription || "-");
    html += renderDeviceActionRow(getUIText("download", "Download"), data.UpdaterState || "-", getUIText("check_upper", "CHECK"), "btnDeviceUpdateCheck", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("install_update", "Install Update"), data.UpdaterNewDescription || "-", getUIText("install_upper", "INSTALL"), "btnDeviceInstallUpdate", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("target_branch", "Target Branch"), data.UpdaterTargetBranch || data.GitBranch || "-", getUIText("select_upper", "SELECT"), "btnDeviceTargetBranch", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("uninstall_openpilot", "Uninstall openpilot"), "", getUIText("uninstall_upper", "UNINSTALL"), "btnDeviceUninstall", "smallBtn btn--danger", true);
    return html;
  }

  return "";
}

function renderDeviceToggleItems(values) {
  let html = "";
  DEVICE_TOGGLES.forEach((toggle) => {
    html += renderDeviceToggleRow(
      toggle.param,
      getUIText(toggle.labelKey, toggle.defaultLabel),
      Boolean(values[toggle.param]),
      {
        confirmKey: toggle.confirmKey || "",
        confirmedParam: toggle.confirmedParam || "",
        confirmed: Boolean(values[toggle.confirmedParam]),
      },
    );
  });

  const personality = Number(values.LongitudinalPersonality ?? 1);
  const option = PERSONALITY_OPTIONS.find((entry) => entry.value === personality) || PERSONALITY_OPTIONS[1];
  html += renderDeviceActionRow(
    getUIText("driving_personality", "Driving Personality"),
    getUIText("driving_personality_desc", "Aggressive, Standard, Relaxed"),
    getUIText(option.labelKey, option.defaultLabel),
    "btnDevicePersonality",
    "val value-surface",
  );
  return html;
}

function renderDeviceDeveloperItems(values) {
  let html = renderSshKeysRow(values.SshKeyStatus || values.GithubUsername || "", Boolean(values.GithubSshKeys));
  DEVICE_DEVELOPER_TOGGLES.forEach((toggle) => {
    html += renderDeviceToggleRow(
      toggle.param,
      getUIText(toggle.labelKey, toggle.defaultLabel),
      Boolean(values[toggle.param]),
      { disabled: toggle.disabled === true, confirmKey: toggle.confirmKey || "" },
    );
  });
  return html;
}

function syncDeviceGroupActiveState(groupId = CURRENT_DEVICE_GROUP) {
  document.querySelectorAll("[data-device-group]").forEach((button) => {
    button.classList.toggle("active", button.dataset.deviceGroup === groupId);
    button.classList.toggle("is-active", button.dataset.deviceGroup === groupId);
  });
}

function syncDeviceGroupChrome(groupId = CURRENT_DEVICE_GROUP) {
  const label = getDeviceGroupLabel(groupId);
  const meta = document.getElementById("groupMeta");
  const itemCount = document.getElementById("deviceItems")?.children.length || 0;
  if (meta && groupId) meta.textContent = `${groupId} / ${itemCount}`;
  if (typeof settingTitle !== "undefined" && settingTitle) {
    settingTitle.textContent = (UI_STRINGS[LANG].setting || "Setting") + " - " + label;
  }
  // Use the shared title renderer so the device submenu gets the same
  // "‹ back" chevron as the CarrotPilot tab (the global itemsTitle click
  // handler then drives history.back()).
  if (typeof setSettingItemsTitle === "function") {
    setSettingItemsTitle(label);
  } else if (typeof itemsTitle !== "undefined" && itemsTitle) {
    itemsTitle.textContent = label;
  }
}

async function switchSettingTab(tab) {
  const nextTab = tab === "device" ? "device" : "carrot";
  if (CURRENT_SETTING_TAB === nextTab) {
    syncSettingTabState(nextTab);
    if (nextTab !== "device") {
      stopDeviceNetworkRefresh();
      stopDeviceSshRefresh();
      if (typeof syncSettingGroupChrome === "function") syncSettingGroupChrome(CURRENT_GROUP);
    } else {
      syncDeviceGroupChrome(CURRENT_DEVICE_GROUP);
      syncDeviceNetworkRefresh();
      syncDeviceSshRefresh();
    }
    return;
  }

  CURRENT_SETTING_TAB = nextTab;
  syncSettingTabState(nextTab);
  if (nextTab !== "device") {
    stopDeviceNetworkRefresh();
    stopDeviceSshRefresh();
  }

  if (nextTab === "device") {
    await renderDeviceTab();
    if (!(typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode()) && typeof showSettingScreen === "function") {
      showSettingScreen("groups", false);
      // Mark the device-groups base entry so back from a device submenu returns
      // here (not to the CarrotPilot groups). Mirrors the CarrotPilot flow.
      history.replaceState({ page: "setting", tab: "device", screen: "groups" }, "");
    }
    syncDeviceGroupChrome(CURRENT_DEVICE_GROUP);
    syncDeviceSshRefresh();
    return;
  }

  if (typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode() && typeof activateSettingGroup === "function") {
    const targetGroup = CURRENT_GROUP || (typeof getLandscapeDefaultSettingGroup === "function" ? getLandscapeDefaultSettingGroup() : null);
    if (targetGroup) {
      await activateSettingGroup(targetGroup, false, {
        animateGroups: false,
        animateItems: false,
        scrollMode: "restore",
      });
      return;
    }
  }

  if (typeof showSettingScreen === "function") {
    showSettingScreen("groups", false);
    // Match the device tab: re-render the CarrotPilot groups with the stagger
    // entrance so switching tabs animates both sides consistently (device
    // re-renders via renderDeviceTab, CarrotPilot didn't → no animation).
    if (typeof renderGroups === "function") renderGroups({ animateGroups: true });
    // Re-sync history to the CarrotPilot groups so back/forward stays in step
    // with the visible tab after a tab switch.
    if (!(typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode())) {
      history.replaceState({ page: "setting", screen: "groups", group: null }, "");
    }
  }
  if (typeof syncSettingGroupChrome === "function") syncSettingGroupChrome(CURRENT_GROUP);
}

if (settingTabDevice) {
  settingTabDevice.addEventListener("click", () => {
    switchSettingTab("device").catch((err) => console.error("[DeviceTab]", err));
  });
}

if (settingTabCarrot) {
  settingTabCarrot.addEventListener("click", () => {
    switchSettingTab("carrot").catch((err) => console.error("[DeviceTab]", err));
  });
}

window.addEventListener("carrot:languagechange", () => {
  if (CURRENT_SETTING_TAB === "device") {
    renderDeviceTab().catch((err) => console.error("[DeviceTab]", err));
  }
});

window.addEventListener("carrot:paramchange", (event) => {
  const name = event.detail?.name;
  if (!name) return;
  mergeDeviceParamValues({ [name]: event.detail?.value });
});

document.addEventListener("visibilitychange", syncDeviceNetworkRefresh);
document.addEventListener("visibilitychange", syncDeviceSshRefresh);
window.addEventListener("carrot:pagechange", syncDeviceNetworkRefresh);
window.addEventListener("carrot:pagechange", syncDeviceSshRefresh);
window.addEventListener("resize", syncDeviceNetworkRefresh);
window.addEventListener("resize", syncDeviceSshRefresh);

syncSettingTabState("carrot");
