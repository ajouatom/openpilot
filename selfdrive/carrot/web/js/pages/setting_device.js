"use strict";

// Device tab coordinator. Rendering, network refresh, and action handlers live
// in adjacent setting_device_* files to keep this file focused on tab state.

let CURRENT_SETTING_TAB = "carrot";
let CURRENT_DEVICE_GROUP = "Device";
let deviceParamValues = {};
let deviceGroupLoadPromises = new Map();
let deviceNetworkInfo = null;
let deviceNetworkLoadPromise = null;
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
  const deviceSubnav = document.getElementById("deviceSubnav");
  const items = document.getElementById("items");
  const deviceItems = document.getElementById("deviceItems");

  setSettingDeviceHidden(carrotTabContent, isDevice);
  setSettingDeviceHidden(deviceTabContent, !isDevice);
  setSettingDeviceHidden(settingSubnav, isDevice);
  setSettingDeviceHidden(deviceSubnav, !isDevice);
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

function renderDeviceGroups() {
  const groupContainer = document.getElementById("deviceGroupList");
  const subnavContainer = document.getElementById("deviceSubnav");
  if (!groupContainer) return;

  groupContainer.innerHTML = "";
  if (subnavContainer) subnavContainer.innerHTML = "";

  const visibleGroups = getVisibleDeviceGroups();
  if (!visibleGroups.some((group) => group.id === CURRENT_DEVICE_GROUP)) {
    CURRENT_DEVICE_GROUP = visibleGroups[0]?.id || "Device";
  }

  visibleGroups.forEach((group) => {
    const label = getDeviceGroupLabel(group.id);
    const button = document.createElement("button");
    button.type = "button";
    button.className = "btn groupBtn";
    if (group.id === CURRENT_DEVICE_GROUP) button.classList.add("active");
    button.dataset.deviceGroup = group.id;
    button.innerHTML = `<span class="setting-group-label">${escapeHtml(label)}</span>`;
    button.onclick = () => selectDeviceGroup(group.id);
    groupContainer.appendChild(button);

    if (subnavContainer) {
      const tab = document.createElement("button");
      tab.type = "button";
      tab.className = "setting-subnav__tab";
      if (group.id === CURRENT_DEVICE_GROUP) tab.classList.add("is-active");
      tab.dataset.deviceGroup = group.id;
      tab.textContent = label;
      tab.onclick = () => selectDeviceGroup(group.id);
      subnavContainer.appendChild(tab);
    }
  });
}

async function renderDeviceTab() {
  syncSettingTabState("device");
  renderDeviceGroups();
  if (!deviceTabLoaded) {
    deviceTabLoaded = true;
    loadDeviceParams("Device", true).then(() => {
      if (CURRENT_SETTING_TAB === "device") renderDeviceGroups();
    });
  }
  if (typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode()) {
    await renderDeviceItems(CURRENT_DEVICE_GROUP, false);
  }
}

async function selectDeviceGroup(groupId) {
  CURRENT_DEVICE_GROUP = groupId || CURRENT_DEVICE_GROUP;
  renderDeviceGroups();
  syncSettingTabState("device");
  await renderDeviceItems(CURRENT_DEVICE_GROUP, true);
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
    return;
  }

  itemsContainer.innerHTML = renderDeviceGroupItems(groupId, values) || `<div class="muted mt-md text-center">-</div>`;
  bindDeviceTabEvents(itemsContainer);
  syncDeviceGroupActiveState(groupId);
  syncDeviceNetworkRefresh();
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
    "val pill",
  );
  return html;
}

function renderDeviceDeveloperItems(values) {
  let html = renderSshKeysRow(values.GithubUsername || "", Boolean(values.GithubSshKeys));
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

async function switchSettingTab(tab) {
  const nextTab = tab === "device" ? "device" : "carrot";
  if (CURRENT_SETTING_TAB === nextTab) {
    syncSettingTabState(nextTab);
    if (nextTab !== "device") stopDeviceNetworkRefresh();
    else syncDeviceNetworkRefresh();
    return;
  }

  CURRENT_SETTING_TAB = nextTab;
  syncSettingTabState(nextTab);
  if (nextTab !== "device") stopDeviceNetworkRefresh();

  if (nextTab === "device") {
    await renderDeviceTab();
    if (!(typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode()) && typeof showSettingScreen === "function") {
      showSettingScreen("groups", false);
    }
    return;
  }

  if (typeof showSettingScreen === "function") {
    showSettingScreen("groups", false);
  }
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
window.addEventListener("carrot:pagechange", syncDeviceNetworkRefresh);
window.addEventListener("resize", syncDeviceNetworkRefresh);

syncSettingTabState("carrot");
