"use strict";

// Device tab coordinator. Rendering, network refresh, and action handlers live
// in adjacent setting_device_* files to keep this file focused on tab state.

let CURRENT_SETTING_TAB = "carrot";
let CURRENT_DEVICE_GROUP = "Device";
let deviceInfo = null;
let deviceInfoLoadPromise = null;
let deviceTabLoaded = false;

function getVisibleDeviceGroups() {
  return DEVICE_GROUPS.filter((group) => {
    if (group.id !== "Software") return true;
    return isTruthyDeviceFlag(deviceInfo?.software_menu);
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

async function loadDeviceInfo(force = false) {
  if (deviceInfo && !force) return deviceInfo;
  if (deviceInfoLoadPromise && !force) return deviceInfoLoadPromise;

  deviceInfoLoadPromise = (async () => {
    const res = await fetch("/api/device_info");
    if (!res.ok) throw new Error("Failed to load device info");
    const payload = await res.json();
    if (!payload || payload.ok === false) throw new Error(payload?.error || "Failed to load device info");
    deviceInfo = payload;
    return deviceInfo;
  })().catch((err) => {
    console.error("[DeviceTab]", err);
    deviceInfo = null;
    return null;
  }).finally(() => {
    deviceInfoLoadPromise = null;
  });

  return deviceInfoLoadPromise;
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
  if (!deviceTabLoaded) {
    await loadDeviceInfo(true);
    deviceTabLoaded = true;
  }
  renderDeviceGroups();
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
  if (groupId === "Toggles") {
    const names = DEVICE_TOGGLES.map((entry) => entry.param);
    DEVICE_TOGGLES.forEach((entry) => {
      if (entry.confirmedParam) names.push(entry.confirmedParam);
    });
    names.push("LongitudinalPersonality");
    try {
      return await bulkGet(names);
    } catch {
      return {};
    }
  }

  if (groupId === "Developer") {
    try {
      return await bulkGet([
        ...DEVICE_DEVELOPER_TOGGLES.map((entry) => entry.param),
        "GithubUsername",
        "GithubSshKeys",
      ]);
    } catch {
      return {};
    }
  }

  await loadDeviceInfo(true);
  return {};
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

  if (!deviceInfo && groupId !== "Toggles" && groupId !== "Developer") {
    itemsContainer.innerHTML = `<div class="device-tab-error">${escapeHtml(getUIText("device_tab_error", "Failed to load device info."))}</div>`;
    return;
  }

  itemsContainer.innerHTML = renderDeviceGroupItems(groupId, values) || `<div class="muted mt-md text-center">-</div>`;
  bindDeviceTabEvents(itemsContainer);
  syncDeviceGroupActiveState(groupId);
  syncDeviceNetworkRefresh();
}

function renderDeviceGroupItems(groupId, values) {
  const info = deviceInfo || {};
  const calib = info.calibration || {};
  const update = info.update || {};

  if (groupId === "Device") {
    const calibValue = calib.calibrated
      ? `pitch ${calib.pitch ?? "-"}, yaw ${calib.yaw ?? "-"}`
      : getUIText("uncalibrated", "Uncalibrated");
    let html = "";
    html += renderDeviceInfoRow(getUIText("dongle_id", "Dongle ID"), info.dongle_id || "N/A");
    html += renderDeviceInfoRow(getUIText("serial", "Serial"), info.serial || "N/A");
    html += renderDeviceActionRow(getUIText("reboot", "Reboot"), getUIText("reboot_device_desc", "Reboot device"), getUIText("reboot", "Reboot"), "btnDeviceReboot");
    html += renderDeviceActionRow(getUIText("recalibration", "ReCalibration"), "", getUIText("reset", "Reset"), "btnDeviceRecalib");
    html += renderDeviceActionRow(getUIText("power_off", "Power Off"), getUIText("power_off_desc", "Power off device"), getUIText("power_off", "Power Off"), "btnDevicePoweroff", "smallBtn btn--danger");
    html += renderDeviceActionRow(getUIText("pair_device", "Pair Device"), getUIText("pair_device_desc", "Pair your device with comma connect (connect.comma.ai) and claim your comma prime offer."), getUIText("pair", "PAIR"), "btnDevicePair", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("driver_camera", "Driver Camera"), getUIText("driver_camera_desc", "Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"), getUIText("preview", "PREVIEW"), "btnDeviceDriverCamera", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("review_training_guide", "Review Training Guide"), getUIText("review_training_desc", "Review the rules, features, and limitations of openpilot"), getUIText("review", "Review"), "btnDeviceTraining");
    html += renderDeviceActionRow(getUIText("calibration_status", "Calibration Status"), calibValue, getUIText("show_upper", "SHOW"), "btnDeviceCalibrationStatus");
    if (String(info.device_type || "").toLowerCase() === "tici") {
      html += renderDeviceActionRow(getUIText("regulatory", "Regulatory"), "", getUIText("view_upper", "VIEW"), "btnDeviceRegulatory");
    }
    html += renderDeviceLanguageRow(info);
    return html;
  }

  if (groupId === "Network") return renderNetworkPanel(info.network || {});
  if (groupId === "Toggles") return renderDeviceToggleItems(values);
  if (groupId === "Developer") return renderDeviceDeveloperItems(values);
  if (groupId === "Software") {
    let html = "";
    html += renderDeviceInfoRow(getUIText("updates_offroad_only", "Updates are only downloaded while the car is off."), "");
    html += renderDeviceVersionRow(getUIText("current_version", "Current Version"), update.version || "-");
    html += renderDeviceActionRow(getUIText("download", "Download"), update.state || "-", getUIText("check_upper", "CHECK"), "btnDeviceUpdateCheck", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("install_update", "Install Update"), update.new_description || "-", getUIText("install_upper", "INSTALL"), "btnDeviceInstallUpdate", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("target_branch", "Target Branch"), update.target_branch || update.git_branch || "-", getUIText("select_upper", "SELECT"), "btnDeviceTargetBranch", "smallBtn", true);
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

document.addEventListener("visibilitychange", syncDeviceNetworkRefresh);
window.addEventListener("carrot:pagechange", syncDeviceNetworkRefresh);
window.addEventListener("resize", syncDeviceNetworkRefresh);

syncSettingTabState("carrot");
