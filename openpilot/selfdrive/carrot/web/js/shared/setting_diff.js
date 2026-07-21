"use strict";

// Thin adapter over the shared diff-table component
// (src/ui/components/diff_table/). The rendering logic and its tests live
// there; this file only keeps the classic-script globals the two callers
// (profile apply, QR restore) still reference, wiring in the app's escape and
// getUIText. Once both callers are ESM they can import the component directly
// and this file can go.

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
  return window.CarrotUI.diffTable.selectedCount(preview);
}

function renderSettingsDiffSummary(summary = {}) {
  return window.CarrotUI.diffTable.renderSummary(summary, {
    escape: settingsDiffEscape,
    text: settingsDiffText,
  });
}

function renderSettingsDiffHtml(preview, options = {}) {
  return window.CarrotUI.diffTable.renderTable(preview, {
    ...options,
    escape: settingsDiffEscape,
    text: settingsDiffText,
  });
}
