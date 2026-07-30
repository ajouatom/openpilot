"use strict";

import { WEB_SETTINGS_GROUPS } from "./schema.js";
import {
  WEB_SPEC_BY_KEY,
  WEB_SETTING_DEFAULTS,
  getWebSettingCapabilitySpec,
  getWebSettingByKey,
  isWebSettingUnlocked,
  setWebSettingByKey,
} from "./state.js";

// Declarative web-settings renderer. The backend field spec decides the
// control type/default/choices; the presentation schema only groups and labels.

let activeWebSettingsGroup = "general";

function setActiveWebSettingsGroup(group) {
  activeWebSettingsGroup = String(group || "general");
  return activeWebSettingsGroup;
}

function webSettingsText(key) {
  return getUIText(key) || String(key || "");
}

function getWebSettingsField(item) {
  return item?.id ? WEB_SPEC_BY_KEY[item.id] || null : null;
}

function isWebSettingsItemVisible(item) {
  if (item?.component) {
    return Boolean(globalThis.WebSettingsComponents?.isVisible?.(item.component, WEB_SPEC_BY_KEY, item));
  }
  return Boolean(getWebSettingsField(item));
}

function getVisibleWebSettingsGroups() {
  return WEB_SETTINGS_GROUPS.map((group) => ({
    ...group,
    items: group.items.filter(isWebSettingsItemVisible),
  })).filter((group) => group.keepWhenEmpty || group.items.length > 0);
}

function findWebSettingsItem(id) {
  for (const group of getVisibleWebSettingsGroups()) {
    const item = group.items.find((entry) => entry.id === id);
    if (item) return item;
  }
  return null;
}

function getWebSettingValue(item) {
  return getWebSettingByKey(item.id, WEB_SETTING_DEFAULTS[item.id]);
}

function setWebSettingValue(item, value) {
  return setWebSettingByKey(item.id, value);
}

function webSettingLockLabel(item) {
  if (isWebSettingUnlocked(item.id)) return "";
  const capability = getWebSettingCapabilitySpec(item.id);
  return webSettingsText(capability?.lockedLabelKey || "web_feature_locked");
}

function renderWebSettingsCopy(title, desc, lockLabel = "") {
  return `
    <span class="web-settings-row__copy">
      <span class="web-settings-row__title-line">
        <span class="web-settings-row__title">${escapeHtml(title)}</span>
      </span>
      ${desc ? `<span class="web-settings-row__desc">${escapeHtml(desc)}</span>` : ""}
    </span>`;
}

function renderWebSettingsItem(item) {
  if (item?.component) return globalThis.WebSettingsComponents?.render?.(item.component, item) || "";
  const field = getWebSettingsField(item);
  if (!field) return "";
  const title = webSettingsText(item.titleKey);
  const desc = webSettingsText(item.descKey);
  const lockLabel = webSettingLockLabel(item);
  const disabled = item.disabled === true || Boolean(lockLabel);

  if (field.type === "bool") {
    const checked = disabled ? false : Boolean(getWebSettingValue(item));
    return `
      <label class="web-settings-row web-settings-row--toggle ${disabled ? "web-settings-row--disabled" : ""}" data-web-setting="${escapeHtml(item.id)}" ${disabled ? 'aria-disabled="true"' : ""}>
        ${renderWebSettingsCopy(title, desc, lockLabel)}
        <span class="c-switch c-switch--sm">
          <input class="c-switch__input" type="checkbox" ${checked ? "checked" : ""} ${disabled ? "disabled" : ""} />
          <span class="c-switch__track" aria-hidden="true"></span>
        </span>
      </label>`;
  }

  if (field.type === "enum") {
    const value = String(getWebSettingValue(item));
    const allowed = new Set(field.choices || []);
    const options = (item.options || [])
      .filter((option) => allowed.size === 0 || allowed.has(option.value))
      .map((option) => `
        <option value="${escapeHtml(option.value)}" ${option.value === value ? "selected" : ""}>
          ${escapeHtml(webSettingsText(option.labelKey))}
        </option>
      `).join("");
    return `
      <label class="web-settings-row web-settings-row--select ${disabled ? "web-settings-row--disabled" : ""}" data-web-setting="${escapeHtml(item.id)}" ${disabled ? 'aria-disabled="true"' : ""}>
        ${renderWebSettingsCopy(title, desc, lockLabel)}
        <select class="web-settings-select" aria-label="${escapeHtml(title)}" ${disabled ? "disabled" : ""}>
          ${options}
        </select>
      </label>`;
  }

  return `
    <div class="web-settings-row ${disabled ? "web-settings-row--disabled" : ""}" data-web-setting="${escapeHtml(item.id)}" ${disabled ? 'aria-disabled="true"' : ""}>
      ${renderWebSettingsCopy(title, desc, lockLabel)}
    </div>`;
}

function renderWebSettingsGroup(group) {
  const title = webSettingsText(group.labelKey);
  const body = group.items.map(renderWebSettingsItem).join("");
  return `
    <section class="web-settings-group" data-web-settings-panel="${escapeHtml(group.id)}" ${group.id === activeWebSettingsGroup ? "" : "hidden"}>
      <div class="web-settings-group__title">${escapeHtml(title)}</div>
      <div class="web-settings-group__body">${body}</div>
    </section>`;
}

function renderWebSettingsDialogHtml() {
  const visibleGroups = getVisibleWebSettingsGroups();
  if (!visibleGroups.some((group) => group.id === activeWebSettingsGroup)) {
    activeWebSettingsGroup = visibleGroups[0]?.id || "general";
  }
  const tabs = visibleGroups.map((group) => {
    const active = group.id === activeWebSettingsGroup;
    return `
      <button type="button" class="web-settings-nav__item ${active ? "is-active" : ""}" data-web-settings-group="${escapeHtml(group.id)}" aria-pressed="${active ? "true" : "false"}">
        ${escapeHtml(webSettingsText(group.labelKey))}
      </button>`;
  }).join("");

  return `
    <div class="web-settings-dialog">
      <nav class="web-settings-nav" aria-label="${escapeHtml(webSettingsText("web_settings"))}">
        ${tabs}
      </nav>
      <div class="web-settings-panels">
        ${visibleGroups.map(renderWebSettingsGroup).join("")}
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

export {
  webSettingsText,
  findWebSettingsItem,
  getWebSettingValue,
  setWebSettingValue,
  renderWebSettingsDialogHtml,
  syncWebSettingsDialog,
  setActiveWebSettingsGroup,
};
