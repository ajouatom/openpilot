"use strict";

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

function renderSshKeysRow(username, hasKeys) {
  const displayName = String(username || "").trim();
  const accountText = hasKeys ? (displayName || "-") : getUIText("not_configured", "Not configured");
  const buttonText = hasKeys ? getUIText("remove_upper", "REMOVE") : getUIText("add_upper", "ADD");
  const buttonClass = hasKeys ? "smallBtn device-ssh-action device-ssh-action--remove" : "smallBtn device-ssh-action";
  return `
    <div class="setting device-setting">
      <div class="settingTop">
        <div>
          <div class="title">${escapeHtml(getUIText("ssh_keys", "SSH Keys"))}</div>
          <div class="descr">${escapeHtml(getUIText("ssh_keys_desc", "Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username other than your own."))}</div>
        </div>
        <div class="ctrl device-ssh-control">
          <div class="device-ssh-account ${hasKeys ? "is-configured" : ""}" title="${escapeHtml(accountText)}">
            <span class="device-ssh-account__label">GitHub</span>
            <span class="device-ssh-account__value">${escapeHtml(accountText)}</span>
          </div>
          <button type="button" class="${buttonClass}" id="btnDeviceSshKeys" data-has-keys="${hasKeys ? "1" : "0"}">${escapeHtml(buttonText)}</button>
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

function renderDeviceToggleRow(param, title, checked, options = {}) {
  const disabled = options.disabled === true;
  const disabledAttrs = disabled ? "data-device-disabled=\"true\"" : "";
  const disabledClass = disabled ? " device-setting--disabled" : "";
  const confirmKey = options.confirmKey || "";
  const confirmedParam = options.confirmedParam || "";
  const confirmed = options.confirmed === true ? "1" : "0";
  return `
    <div class="setting device-setting${disabledClass}" ${disabledAttrs}>
      <div class="settingTop">
        <label class="device-toggle" data-param="${escapeHtml(param)}" data-confirm-key="${escapeHtml(confirmKey)}" data-confirmed-param="${escapeHtml(confirmedParam)}" data-confirmed="${confirmed}">
          <span class="device-toggle__text">
            <span class="title">${escapeHtml(title)}</span>
          </span>
          <span class="ctrl">
            <input type="checkbox" class="device-toggle__input" ${checked ? "checked" : ""} ${disabled ? "disabled" : ""} />
            <span class="device-toggle__slider"></span>
          </span>
        </label>
      </div>
    </div>`;
}
