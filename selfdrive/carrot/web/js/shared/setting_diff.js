"use strict";

function settingsDiffEscape(value) {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#39;");
}

function settingsDiffText(key, fallback, vars = null) {
  if (typeof getUIText === "function") return getUIText(key, fallback, vars);
  let text = fallback;
  if (vars) {
    Object.entries(vars).forEach(([name, value]) => {
      text = text.replaceAll(`{${name}}`, String(value));
    });
  }
  return text;
}

function getSettingsDiffSelectedCount(preview) {
  const summary = preview?.summary || {};
  const selected = Number(summary.selected || 0);
  if (Number.isFinite(selected) && selected > 0) return selected;
  const entries = Array.isArray(preview?.entries) ? preview.entries : [];
  return entries.filter((entry) => entry?.apply).length;
}

function renderSettingsDiffSummary(summary = {}) {
  const items = [
    ["changed", "settings_diff_changed", "changed"],
    ["same", "settings_diff_same", "same"],
    ["skipped", "settings_diff_skipped", "skipped"],
    ["invalid", "settings_diff_invalid", "invalid"],
  ];
  return `
    <div class="settings-diff-summary">
      ${items.map(([key, labelKey, fallback]) => `
        <div class="settings-diff-summary__item settings-diff-summary__item--${key}">
          <span>${settingsDiffEscape(settingsDiffText(labelKey, fallback))}</span>
          <strong>${settingsDiffEscape(summary?.[key] ?? 0)}</strong>
        </div>
      `).join("")}
    </div>
  `;
}

function renderSettingsDiffHtml(preview, options = {}) {
  const entries = Array.isArray(preview?.entries) ? preview.entries : [];
  const changed = entries.filter((entry) => entry?.apply || entry?.status === "changed");
  const limit = Number.isFinite(options.limit) ? Math.max(1, options.limit) : 80;
  const shown = changed.slice(0, limit);
  const hiddenCount = Math.max(0, changed.length - shown.length);
  const currentLabel = options.currentLabel || settingsDiffText("settings_diff_current", "Current");
  const nextLabel = options.nextLabel || settingsDiffText("settings_diff_apply", "Apply");
  const changedLabel = options.changedLabel || settingsDiffText("settings_diff_changed_status", "Changed");

  if (!changed.length) {
    return `
      ${renderSettingsDiffSummary(preview?.summary)}
      <div class="settings-diff-empty">${settingsDiffEscape(settingsDiffText("settings_diff_no_changes", "No changes to apply."))}</div>
    `;
  }

  const rows = shown.map((entry) => `
    <div class="settings-diff__row">
      <div class="settings-diff__head">
        <div class="settings-diff__key">${settingsDiffEscape(entry.key)}</div>
        <span class="settings-diff__status">${settingsDiffEscape(changedLabel)}</span>
      </div>
      <div class="settings-diff__compare">
        <div class="settings-diff__value settings-diff__value--old">
          <span>${settingsDiffEscape(currentLabel)}</span>
          <code>${settingsDiffEscape(entry.current)}</code>
        </div>
        <div class="settings-diff__arrow" aria-hidden="true">&gt;</div>
        <div class="settings-diff__value settings-diff__value--new">
          <span>${settingsDiffEscape(nextLabel)}</span>
          <code>${settingsDiffEscape(entry.value)}</code>
        </div>
      </div>
    </div>
  `).join("");

  return `
    ${renderSettingsDiffSummary(preview?.summary)}
    <div class="settings-diff__list">${rows}</div>
    ${hiddenCount ? `<div class="settings-diff-more">${settingsDiffEscape(settingsDiffText("settings_diff_more", "{count} more changes hidden", { count: hiddenCount }))}</div>` : ""}
  `;
}
