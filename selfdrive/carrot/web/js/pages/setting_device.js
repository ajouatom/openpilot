"use strict";

// Device tab for the Settings page. Keep all Device/Carrot tab state local to
// this page module so shared DOM/navigation code only manages page-level state.

let CURRENT_SETTING_TAB = "carrot";
let CURRENT_DEVICE_GROUP = "Device";
let deviceInfo = null;
let deviceInfoLoadPromise = null;
let deviceTabLoaded = false;

const DEVICE_GROUPS = [
  { id: "Device", labelKey: "device_group_info", defaultLabel: "Device Info" },
  { id: "Network", labelKey: "device_group_network", defaultLabel: "Network" },
  { id: "Toggles", labelKey: "device_group_toggles", defaultLabel: "Toggles" },
  { id: "Software", labelKey: "device_group_software", defaultLabel: "Software" },
];

const DEVICE_TOGGLES = [
  { param: "OpenpilotEnabledToggle", labelKey: "enable_openpilot", defaultLabel: "Enable openpilot" },
  { param: "ExperimentalMode", labelKey: "experimental_mode", defaultLabel: "Experimental Mode" },
  { param: "DisengageOnAccelerator", labelKey: "disengage_on_accelerator", defaultLabel: "Disengage on Accelerator" },
  { param: "IsLdwEnabled", labelKey: "enable_ldw", defaultLabel: "Enable Lane Departure Warnings" },
  { param: "AlwaysOnDM", labelKey: "always_on_dm", defaultLabel: "Always-on DM" },
  { param: "RecordFront", labelKey: "record_front", defaultLabel: "Record and Upload Driver Camera" },
  { param: "RecordAudio", labelKey: "record_audio", defaultLabel: "Record and Upload Microphone Audio" },
  { param: "IsMetric", labelKey: "is_metric", defaultLabel: "Use Metric System" },
];

const PERSONALITY_OPTIONS = [
  { value: 0, labelKey: "aggressive", defaultLabel: "Aggressive" },
  { value: 1, labelKey: "standard", defaultLabel: "Standard" },
  { value: 2, labelKey: "relaxed", defaultLabel: "Relaxed" },
  { value: 3, labelKey: "more_relaxed", defaultLabel: "MoreRelaxed" },
];

const TRAINING_STEP_COUNT = 19;

function isTruthyDeviceFlag(value) {
  return value === true || value === 1 || value === "1" || value === "true" || value === "True";
}

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
  syncSettingTabChrome(tab);
  syncSettingTabPanels(tab);
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

async function renderDeviceItems(groupId, showItemsScreen = true) {
  const itemsContainer = document.getElementById("deviceItems");
  if (!itemsContainer) return;

  syncSettingTabState("device");
  if (showItemsScreen && typeof showSettingScreen === "function") {
    showSettingScreen("items", false);
  }

  itemsContainer.innerHTML = `<div class="muted mt-md text-center">${escapeHtml(getUIText("loading", "Loading..."))}</div>`;

  let toggleValues = {};
  if (groupId === "Toggles") {
    const names = DEVICE_TOGGLES.map((entry) => entry.param);
    names.push("LongitudinalPersonality");
    try {
      toggleValues = await bulkGet(names);
    } catch {
      toggleValues = {};
    }
  } else {
    await loadDeviceInfo(true);
  }

  if (!deviceInfo && groupId !== "Toggles") {
    itemsContainer.innerHTML = `<div class="device-tab-error">${escapeHtml(getUIText("device_tab_error", "Failed to load device info."))}</div>`;
    return;
  }

  const info = deviceInfo || {};
  const calib = info.calibration || {};
  const update = info.update || {};

  let html = "";
  if (groupId === "Device") {
    const calibValue = calib.calibrated
      ? `pitch ${calib.pitch ?? "-"}, yaw ${calib.yaw ?? "-"}`
      : getUIText("uncalibrated", "Uncalibrated");
    html += renderDeviceInfoRow(getUIText("dongle_id", "Dongle ID"), info.dongle_id || "N/A");
    html += renderDeviceInfoRow(getUIText("serial", "Serial"), info.serial || "N/A");
    html += renderDeviceActionRow(getUIText("reboot", "Reboot"), getUIText("reboot_device_desc", "Reboot device"), getUIText("reboot", "Reboot"), "btnDeviceReboot");
    html += renderDeviceActionRow(getUIText("recalibration", "ReCalibration"), "", getUIText("reset", "Reset"), "btnDeviceRecalib");
    html += renderDeviceActionRow(getUIText("power_off", "Power Off"), getUIText("power_off_desc", "Power off device"), getUIText("power_off", "Power Off"), "btnDevicePoweroff", "smallBtn btn--danger");
    html += renderDeviceActionRow(getUIText("pair_device", "Pair Device"), getUIText("pair_device_desc", "Pair your device with comma connect (connect.comma.ai) and claim your comma prime offer."), getUIText("pair", "PAIR"), "btnDevicePair", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("driver_camera", "Driver Camera"), getUIText("driver_camera_desc", "Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)"), getUIText("preview", "PREVIEW"), "btnDeviceDriverCamera", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("review_training_guide", "Review Training Guide"), getUIText("review_training_desc", "Review the rules, features, and limitations of openpilot"), getUIText("review", "Review"), "btnDeviceTraining");
    html += renderDeviceInfoRow(getUIText("calibration_status", "Calibration Status"), calibValue);
    if (String(info.device_type || "").toLowerCase() === "tici") {
      html += renderDeviceActionRow(getUIText("regulatory", "Regulatory"), "", getUIText("view_upper", "VIEW"), "btnDeviceRegulatory");
    }
    html += renderDeviceLanguageRow(info);
  } else if (groupId === "Network") {
    html += renderNetworkPanel(info.network || {});
  } else if (groupId === "Toggles") {
    DEVICE_TOGGLES.forEach((toggle) => {
      html += renderDeviceToggleRow(
        toggle.param,
        getUIText(toggle.labelKey, toggle.defaultLabel),
        Boolean(toggleValues[toggle.param]),
      );
    });

    const personality = Number(toggleValues.LongitudinalPersonality ?? 1);
    const option = PERSONALITY_OPTIONS.find((entry) => entry.value === personality) || PERSONALITY_OPTIONS[1];
    html += renderDeviceActionRow(
      getUIText("driving_personality", "Driving Personality"),
      getUIText("driving_personality_desc", "Aggressive, Standard, Relaxed"),
      getUIText(option.labelKey, option.defaultLabel),
      "btnDevicePersonality",
      "val pill",
    );
  } else if (groupId === "Software") {
    html += renderDeviceInfoRow(getUIText("updates_offroad_only", "Updates are only downloaded while the car is off."), "");
    html += renderDeviceVersionRow(getUIText("current_version", "Current Version"), update.version || "-");
    html += renderDeviceActionRow(getUIText("download", "Download"), update.state || "-", getUIText("check_upper", "CHECK"), "btnDeviceUpdateCheck", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("install_update", "Install Update"), update.new_description || "-", getUIText("install_upper", "INSTALL"), "btnDeviceInstallUpdate", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("target_branch", "Target Branch"), update.target_branch || update.git_branch || "-", getUIText("select_upper", "SELECT"), "btnDeviceTargetBranch", "smallBtn", true);
    html += renderDeviceActionRow(getUIText("uninstall_openpilot", "Uninstall openpilot"), "", getUIText("uninstall_upper", "UNINSTALL"), "btnDeviceUninstall", "smallBtn btn--danger", true);
  }

  itemsContainer.innerHTML = html || `<div class="muted mt-md text-center">-</div>`;
  bindDeviceTabEvents(itemsContainer);
  syncDeviceGroupActiveState(groupId);
}

function syncDeviceGroupActiveState(groupId = CURRENT_DEVICE_GROUP) {
  document.querySelectorAll("[data-device-group]").forEach((button) => {
    button.classList.toggle("active", button.dataset.deviceGroup === groupId);
    button.classList.toggle("is-active", button.dataset.deviceGroup === groupId);
  });
}

function renderDeviceInfoRow(title, value) {
  return `
    <div class="setting device-setting">
      <div class="settingTop">
        <div>
          <div class="title">${escapeHtml(title)}</div>
        </div>
        <div class="ctrl">
          <div class="title muted device-value">${escapeHtml(String(value))}</div>
        </div>
      </div>
    </div>`;
}

function renderDeviceVersionRow(title, value) {
  const parts = String(value || "-")
    .split(/\s+\/\s+/)
    .map((part) => part.trim())
    .filter(Boolean);
  const lines = parts.length ? parts : ["-"];
  return `
    <div class="setting device-setting">
      <div class="settingTop">
        <div>
          <div class="title">${escapeHtml(title)}</div>
        </div>
        <div class="ctrl">
          <div class="title muted device-value device-version-value">
            ${lines.map((line) => `<span>${escapeHtml(line)}</span>`).join("")}
          </div>
        </div>
      </div>
    </div>`;
}

function renderDeviceActionRow(title, descr, buttonText, buttonId, buttonClass = "smallBtn", disabled = false) {
  const disabledAttrs = disabled ? "data-device-disabled=\"true\"" : "";
  const disabledClass = disabled ? " device-setting--disabled" : "";
  const buttonAttrs = disabled ? "aria-disabled=\"true\" class=\"" + buttonClass + " is-device-disabled\"" : "class=\"" + buttonClass + "\"";
  return `
    <div class="setting device-setting${disabledClass}" ${disabledAttrs}>
      <div class="settingTop">
        <div>
          <div class="title">${escapeHtml(title)}</div>
          ${descr ? `<div class="descr">${escapeHtml(descr)}</div>` : ""}
        </div>
        <div class="ctrl">
          <button type="button" ${buttonAttrs} id="${buttonId}">${escapeHtml(buttonText)}</button>
        </div>
      </div>
    </div>`;
}

function renderDeviceLanguageRow(info) {
  const currentLang = info.language || "main_en";
  return `
    <div class="setting device-setting">
      <div class="settingTop">
        <div>
          <div class="title">${escapeHtml(getUIText("change_language", "Change Language"))}</div>
          <div class="descr">${escapeHtml(getUIText("language_note", "Requires reboot"))}</div>
        </div>
        <div class="ctrl">
          <select id="deviceLanguageSelect" class="input-text device-select">
            ${(info.languages || []).map((lang) => `
              <option value="${escapeHtml(lang.code)}" ${lang.code === currentLang ? "selected" : ""}>${escapeHtml(lang.name)}</option>
            `).join("")}
          </select>
        </div>
      </div>
    </div>`;
}

function wifiSignalLabel(signal) {
  if (signal === null || signal === undefined || signal === "") return "-";
  return `${signal}%`;
}

function renderNetworkPanel(network) {
  let html = renderDeviceActionRow(getUIText("advanced", "Advanced"), "", getUIText("view_upper", "VIEW"), "btnDeviceNetworkAdvanced", "smallBtn", true);
  html += renderWifiList(network.wifi || []);
  html += renderDeviceToggleViewRow(getUIText("enable_tethering", "Enable Tethering"), Boolean(network.tethering_enabled));
  html += renderDeviceInfoRow(getUIText("tethering_password", "Tethering Password"), "********");
  html += renderDeviceInfoRow(getUIText("ip_address", "IP Address"), network.ip_address || "-");
  html += renderDeviceToggleViewRow(getUIText("enable_roaming", "Enable Roaming"), Boolean(network.roaming_enabled));
  html += renderDeviceActionRow(getUIText("apn_setting", "APN Setting"), network.apn || getUIText("automatic", "automatic"), getUIText("edit_upper", "EDIT"), "btnDeviceApn", "smallBtn", true);
  html += renderDeviceToggleViewRow(getUIText("cellular_metered", "Cellular Metered"), Boolean(network.gsm_metered), getUIText("cellular_metered_desc", "Prevent large data uploads when on a metered connection"));
  html += renderDeviceActionRow(getUIText("hidden_network", "Hidden Network"), "", getUIText("connect_upper", "CONNECT"), "btnDeviceHiddenNetwork", "smallBtn", true);
  return html;
}

function renderWifiList(networks) {
  if (!Array.isArray(networks) || networks.length === 0) {
    return `
      <div class="setting device-setting">
        <div class="settingTop">
          <div>
            <div class="title">${escapeHtml(getUIText("scanning_networks", "Scanning for networks..."))}</div>
            <div class="descr">${escapeHtml(getUIText("wifi_viewer_only", "Viewer only"))}</div>
          </div>
          <div class="ctrl">
            <div class="title muted device-value">-</div>
          </div>
        </div>
      </div>`;
  }

  return networks.map((network) => {
    const ssid = network.ssid || "-";
    const meta = [
      network.connected ? getUIText("connected", "Connected") : getUIText("not_connected", "Not connected"),
      network.secure ? getUIText("secured", "Secured") : getUIText("open_network", "Open"),
      wifiSignalLabel(network.signal),
    ].filter(Boolean).join(" · ");
    return `
      <div class="setting device-setting wifi-view-row">
        <div class="settingTop">
          <div>
            <div class="title">${escapeHtml(ssid)}</div>
            <div class="descr">${escapeHtml(meta)}</div>
          </div>
          <div class="ctrl">
            <div class="wifi-view-status ${network.connected ? "is-connected" : ""}">
              ${escapeHtml(network.connected ? getUIText("connected", "Connected") : wifiSignalLabel(network.signal))}
            </div>
          </div>
        </div>
      </div>`;
  }).join("");
}

function renderDeviceToggleViewRow(title, checked, descr = "") {
  return `
    <div class="setting device-setting device-setting--disabled" data-device-disabled="true">
      <div class="settingTop">
        <div>
          <div class="title">${escapeHtml(title)}</div>
          ${descr ? `<div class="descr">${escapeHtml(descr)}</div>` : ""}
        </div>
        <div class="ctrl">
          <span class="device-toggle device-toggle--readonly">
            <span class="ctrl">
              <input type="checkbox" class="device-toggle__input" ${checked ? "checked" : ""} disabled />
              <span class="device-toggle__slider"></span>
            </span>
          </span>
        </div>
      </div>
    </div>`;
}

function renderDeviceToggleRow(param, title, checked) {
  return `
    <div class="setting device-setting">
      <div class="settingTop">
        <label class="device-toggle" data-param="${escapeHtml(param)}">
          <span class="device-toggle__text">
            <span class="title">${escapeHtml(title)}</span>
          </span>
          <span class="ctrl">
            <input type="checkbox" class="device-toggle__input" ${checked ? "checked" : ""} />
            <span class="device-toggle__slider"></span>
          </span>
        </label>
      </div>
    </div>`;
}

function bindDeviceTabEvents(container) {
  bindDeviceDisabledControls(container);

  container.querySelectorAll(".device-toggle__input").forEach((input) => {
    input.addEventListener("change", async (event) => {
      const toggle = event.target.closest(".device-toggle");
      const param = toggle?.dataset.param;
      if (!param) return;
      try {
        await setParam(param, event.target.checked ? 1 : 0);
      } catch (err) {
        showAppToast(err.message || getUIText("failed", "Failed"), { tone: "error" });
        event.target.checked = !event.target.checked;
      }
    });
  });

  const btnPersonality = container.querySelector("#btnDevicePersonality");
  if (btnPersonality) {
    btnPersonality.addEventListener("click", async () => {
      let current = 1;
      try {
        const values = await bulkGet(["LongitudinalPersonality"]);
        current = Number(values.LongitudinalPersonality ?? 1);
      } catch {}

      const currentIndex = PERSONALITY_OPTIONS.findIndex((entry) => entry.value === current);
      const nextOption = PERSONALITY_OPTIONS[(Math.max(0, currentIndex) + 1) % PERSONALITY_OPTIONS.length];
      try {
        await setParam("LongitudinalPersonality", nextOption.value);
        btnPersonality.textContent = getUIText(nextOption.labelKey, nextOption.defaultLabel);
      } catch (err) {
        showAppToast(err.message || getUIText("failed", "Failed"), { tone: "error" });
      }
    });
  }

  const langSelect = container.querySelector("#deviceLanguageSelect");
  if (langSelect) {
    langSelect.addEventListener("change", async (event) => {
      try {
        await setParam("LanguageSetting", event.target.value);
        if (typeof setWebLanguage === "function") {
          setWebLanguage(event.target.value, { persist: true });
        }
        showAppToast(getUIText("device_lang_changed", "Language changed, reboot required."), { tone: "info" });
      } catch (err) {
        showAppToast(err.message || getUIText("failed", "Failed"), { tone: "error" });
      }
    });
  }

  bindDeviceAction(container, "btnDeviceReboot", "/api/reboot", getUIText("confirm_reboot", "Reboot now?"));
  bindDeviceAction(container, "btnDevicePoweroff", "/api/poweroff", getUIText("power_off_confirm", "Power off device?"));
  bindDeviceAction(container, "btnDeviceRecalib", "/api/recalibrate", getUIText("reset_calibration_confirm", "Reset calibration and reboot?"));

  const trainingButton = container.querySelector("#btnDeviceTraining");
  if (trainingButton) {
    trainingButton.addEventListener("click", () => openTrainingGuide());
  }

  const regulatoryButton = container.querySelector("#btnDeviceRegulatory");
  if (regulatoryButton) {
    regulatoryButton.addEventListener("click", () => {
      openRegulatoryInfo().catch((err) => {
        showAppToast(err.message || getUIText("regulatory_load_failed", "Failed to load regulatory information."), { tone: "error" });
      });
    });
  }
}

function bindDeviceDisabledControls(container) {
  const showMessage = (event) => {
    event.preventDefault();
    event.stopPropagation();
    showAppToast(getUIText("device_only_control", "This can only be controlled on the device."), { tone: "info" });
  };

  container.querySelectorAll("[data-device-disabled=\"true\"]").forEach((el) => {
    el.addEventListener("click", showMessage);
    el.addEventListener("keydown", (event) => {
      if (event.key !== "Enter" && event.key !== " ") return;
      showMessage(event);
    });
  });
}

function bindDeviceAction(container, id, endpoint, confirmMessage = "") {
  const button = container.querySelector(`#${id}`);
  if (!button) return;
  button.addEventListener("click", async () => {
    if (confirmMessage && !confirm(confirmMessage)) return;
    try {
      await postJson(endpoint, {});
      showAppToast(getUIText("action_triggered", "Action triggered"), { tone: "info" });
    } catch (err) {
      showAppToast(err.message || getUIText("failed", "Failed"), { tone: "error" });
    }
  });
}

function openDeviceInfoModal(title, html) {
  const overlay = document.createElement("div");
  overlay.className = "training-guide-modal device-info-modal";
  overlay.innerHTML = `
    <div class="training-guide-modal__surface device-info-modal__surface" role="dialog" aria-modal="true" aria-label="${escapeHtml(title)}">
      <button type="button" class="training-guide-modal__close" data-device-info-close aria-label="${escapeHtml(getUIText("close", "Close"))}">×</button>
      <div class="device-info-modal__header">${escapeHtml(title)}</div>
      <div class="device-info-modal__body"></div>
    </div>`;

  const close = () => overlay.remove();
  overlay.querySelector(".device-info-modal__body").innerHTML = html || "";
  overlay.querySelector("[data-device-info-close]").addEventListener("click", close);
  overlay.addEventListener("click", (event) => {
    if (event.target === overlay) close();
  });
  document.body.appendChild(overlay);
}

async function openRegulatoryInfo() {
  const payload = await getJson("/api/regulatory");
  openDeviceInfoModal(getUIText("regulatory", "Regulatory"), payload.html || "");
}

function openTrainingGuide() {
  let index = 0;
  const overlay = document.createElement("div");
  overlay.className = "training-guide-modal";
  overlay.innerHTML = `
    <div class="training-guide-modal__surface" role="dialog" aria-modal="true" aria-label="${escapeHtml(getUIText("review_training_guide", "Review Training Guide"))}">
      <button type="button" class="training-guide-modal__close" data-training-close aria-label="${escapeHtml(getUIText("close", "Close"))}">×</button>
      <img class="training-guide-modal__image" alt="" />
      <div class="training-guide-modal__bar">
        <button type="button" class="smallBtn" data-training-prev>${escapeHtml(getUIText("back", "Back"))}</button>
        <div class="training-guide-modal__count"></div>
        <button type="button" class="smallBtn" data-training-next>${escapeHtml(getUIText("next", "Next"))}</button>
      </div>
    </div>`;

  const image = overlay.querySelector(".training-guide-modal__image");
  const count = overlay.querySelector(".training-guide-modal__count");
  const prev = overlay.querySelector("[data-training-prev]");
  const next = overlay.querySelector("[data-training-next]");

  const close = () => overlay.remove();
  const render = () => {
    image.src = `/training/step${index}.png`;
    count.textContent = `${index + 1} / ${TRAINING_STEP_COUNT}`;
    prev.disabled = index === 0;
    next.textContent = getUIText(index === TRAINING_STEP_COUNT - 1 ? "close" : "next", index === TRAINING_STEP_COUNT - 1 ? "Close" : "Next");
  };

  overlay.querySelector("[data-training-close]").addEventListener("click", close);
  prev.addEventListener("click", () => {
    index = Math.max(0, index - 1);
    render();
  });
  next.addEventListener("click", () => {
    if (index >= TRAINING_STEP_COUNT - 1) {
      close();
      return;
    }
    index += 1;
    render();
  });
  overlay.addEventListener("click", (event) => {
    if (event.target === overlay) close();
  });

  render();
  document.body.appendChild(overlay);
}

async function switchSettingTab(tab) {
  const nextTab = tab === "device" ? "device" : "carrot";
  if (CURRENT_SETTING_TAB === nextTab) {
    syncSettingTabState(nextTab);
    return;
  }

  CURRENT_SETTING_TAB = nextTab;
  syncSettingTabState(nextTab);

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

syncSettingTabState("carrot");
