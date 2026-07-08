"use strict";

// Web settings dialog rendering — turns the UI schema (WEB_SETTINGS_GROUPS) into
// DOM strings and keeps the active panel in sync. Control widgets are dispatched
// by row.type; add a new widget by adding a branch in renderWebSettingsItem.
// Reuses the shared .c-switch component and getUIText / escapeHtml helpers.

let activeWebSettingsGroup = "general";

function getWebSettingValue(item) {
  return getWebSettingByKey(item.id, WEB_SETTING_DEFAULTS[item.id]);
}

function setWebSettingValue(item, value) {
  return setWebSettingByKey(item.id, value);
}

function renderWebSettingsItem(item) {
  const title = getUIText(item.titleKey, item.defaultTitle);
  const desc = getUIText(item.descKey, item.defaultDesc);
  if (item.type === "toggle") {
    const checked = getWebSettingValue(item);
    return `
      <label class="web-settings-row web-settings-row--toggle" data-web-setting="${escapeHtml(item.id)}">
        <span class="web-settings-row__copy">
          <span class="web-settings-row__title">${escapeHtml(title)}</span>
          <span class="web-settings-row__desc">${escapeHtml(desc)}</span>
        </span>
        <span class="c-switch c-switch--sm">
          <input class="c-switch__input" type="checkbox" ${checked ? "checked" : ""} />
          <span class="c-switch__track" aria-hidden="true"></span>
        </span>
      </label>`;
  }

  if (item.type === "select") {
    const value = getWebSettingValue(item);
    const options = (item.options || []).map((option) => `
      <option value="${escapeHtml(option.value)}" ${option.value === value ? "selected" : ""}>
        ${escapeHtml(getUIText(option.labelKey, option.defaultLabel))}
      </option>
    `).join("");
    return `
      <label class="web-settings-row web-settings-row--select" data-web-setting="${escapeHtml(item.id)}">
        <span class="web-settings-row__copy">
          <span class="web-settings-row__title">${escapeHtml(title)}</span>
          <span class="web-settings-row__desc">${escapeHtml(desc)}</span>
        </span>
        <select class="web-settings-select" aria-label="${escapeHtml(title)}">
          ${options}
        </select>
      </label>`;
  }

  if (item.type === "text" || item.type === "password") {
    const value = String(getWebSettingValue(item) ?? "");
    return `
      <label class="web-settings-row web-settings-row--input" data-web-setting="${escapeHtml(item.id)}">
        <span class="web-settings-row__copy">
          <span class="web-settings-row__title">${escapeHtml(title)}</span>
          <span class="web-settings-row__desc">${escapeHtml(desc)}</span>
        </span>
        <input class="web-settings-input" type="${item.type === "password" ? "password" : "text"}"
          value="${escapeHtml(value)}" placeholder="${escapeHtml(item.placeholder || "")}"
          autocomplete="off" autocapitalize="off" spellcheck="false" aria-label="${escapeHtml(title)}" />
      </label>`;
  }

  if (item.type === "action") {
    const enabled = typeof item.enabledWhen === "function" ? Boolean(item.enabledWhen()) : true;
    const disabledHint = getUIText(item.disabledHintKey, item.defaultDisabledHint || "");
    return `
      <div class="web-settings-row web-settings-row--action" data-web-setting-action="${escapeHtml(item.id)}">
        <span class="web-settings-row__copy">
          <span class="web-settings-row__title">${escapeHtml(title)}</span>
          <span class="web-settings-row__desc">${escapeHtml(desc)}</span>
        </span>
        <button type="button" class="web-settings-action" ${enabled ? "" : "disabled"}
          title="${enabled ? "" : escapeHtml(disabledHint)}">${escapeHtml(getUIText(item.buttonKey, item.defaultButton || "Run"))}</button>
      </div>`;
  }

  return `
    <div class="web-settings-row">
      <div class="web-settings-row__title">${escapeHtml(title)}</div>
      ${desc ? `<div class="web-settings-row__desc">${escapeHtml(desc)}</div>` : ""}
    </div>`;
}

function renderWebSettingsGroup(group) {
  const title = getUIText(group.labelKey, group.defaultLabel);
  const body = group.items.length
    ? group.items.map(renderWebSettingsItem).join("")
    : `<div class="web-settings-empty">${escapeHtml(getUIText("web_settings_empty", "No general web settings yet."))}</div>`;

  return `
    <section class="web-settings-group" data-web-settings-panel="${escapeHtml(group.id)}" ${group.id === activeWebSettingsGroup ? "" : "hidden"}>
      <div class="web-settings-group__title">${escapeHtml(title)}</div>
      <div class="web-settings-group__body">${body}</div>
    </section>`;
}

function renderWebSettingsDialogHtml() {
  const groups = WEB_SETTINGS_GROUPS.map((group) => {
    const active = group.id === activeWebSettingsGroup;
    return `
      <button type="button" class="web-settings-nav__item ${active ? "is-active" : ""}" data-web-settings-group="${escapeHtml(group.id)}" aria-pressed="${active ? "true" : "false"}">
        ${escapeHtml(getUIText(group.labelKey, group.defaultLabel))}
      </button>`;
  }).join("");

  return `
    <div class="web-settings-dialog">
      <nav class="web-settings-nav" aria-label="${escapeHtml(getUIText("web_settings", "Web Settings"))}">
        ${groups}
      </nav>
      <div class="web-settings-panels">
        ${WEB_SETTINGS_GROUPS.map(renderWebSettingsGroup).join("")}
      </div>
    </div>`;
}

function syncWebSettingsDialog() {
  document.querySelectorAll("[data-web-settings-group]").forEach((button) => {
    const active = button.dataset.webSettingsGroup === activeWebSettingsGroup;
    button.classList.toggle("is-active", active);
    button.setAttribute("aria-pressed", active ? "true" : "false");
  });
  document.querySelectorAll("[data-web-settings-panel]").forEach((panel) => {
    panel.hidden = panel.dataset.webSettingsPanel !== activeWebSettingsGroup;
  });
}
