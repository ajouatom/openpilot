/**
 * Renders a parameter-restore preview: the summary counts, the list of changed
 * rows, and a collapsed detail of anything the server could not classify.
 *
 * Shared by the profile-apply dialog and the QR backup restore, which used to
 * lean on a classic-script file of globals (js/shared/setting_diff.js). This is
 * the pure module version — it takes escape/text as inputs and returns a
 * string, so it renders the same whether or not a translation provider exists
 * and can be tested without a DOM.
 */

function defaultText(_key, fallback, vars) {
  let text = String(fallback ?? "");
  if (vars) {
    for (const [name, value] of Object.entries(vars)) {
      text = text.replaceAll(`{${name}}`, String(value));
    }
  }
  return text;
}

function identityEscape(value) {
  return String(value ?? "");
}

export function getDiffSelectedCount(preview) {
  const selected = Number(preview?.summary?.selected || 0);
  if (Number.isFinite(selected) && selected > 0) return selected;
  const entries = Array.isArray(preview?.entries) ? preview.entries : [];
  return entries.filter((entry) => entry?.apply).length;
}

export function renderDiffSummary(summary, options = {}) {
  const escape = options.escape || identityEscape;
  const text = options.text || defaultText;
  const items = [
    ["changed", "settings_diff_changed", "changed"],
    ["same", "settings_diff_same", "same"],
    ["skipped", "settings_diff_skipped", "skipped"],
    ["invalid", "settings_diff_invalid", "invalid"],
  ];
  return `<div class="settings-diff-summary">`
    + items.map(([key, labelKey, fallback]) => (
      `<div class="settings-diff-summary__item settings-diff-summary__item--${key}">`
      + `<span>${escape(text(labelKey, fallback))}</span>`
      + `<strong>${escape(summary?.[key] ?? 0)}</strong></div>`
    )).join("")
    + `</div>`;
}

// The entries that could not be classified, with the reason the server gave.
// Previously "invalid N" was a bare count with no way to see which parameters
// it meant.
function renderDiffInvalid(entries, escape, text) {
  const invalid = entries.filter((entry) => entry?.status === "invalid");
  if (!invalid.length) return "";
  const rows = invalid.map((entry) => (
    `<li class="settings-diff-invalid__item">`
    + `<code class="settings-diff-invalid__key">${escape(entry.key)}</code>`
    + (entry.reason ? `<span class="settings-diff-invalid__reason">${escape(entry.reason)}</span>` : "")
    + `</li>`
  )).join("");
  return `<details class="settings-diff-invalid">`
    + `<summary class="settings-diff-invalid__summary">${escape(
      text("settings_diff_invalid_detail", "{count} could not be applied", { count: invalid.length }),
    )}</summary>`
    + `<ul class="settings-diff-invalid__list">${rows}</ul></details>`;
}

export function renderDiffTable(preview, options = {}) {
  const escape = options.escape || identityEscape;
  const text = options.text || defaultText;
  const entries = Array.isArray(preview?.entries) ? preview.entries : [];
  const changed = entries.filter((entry) => entry?.apply || entry?.status === "changed");
  const limit = Number.isFinite(options.limit) ? Math.max(1, options.limit) : 80;
  const shown = changed.slice(0, limit);
  const hiddenCount = Math.max(0, changed.length - shown.length);
  const currentLabel = options.currentLabel || text("settings_diff_current", "Current");
  const nextLabel = options.nextLabel || text("settings_diff_apply", "Apply");
  const changedLabel = options.changedLabel || text("settings_diff_changed_status", "Changed");
  const invalidHtml = renderDiffInvalid(entries, escape, text);
  const summaryHtml = renderDiffSummary(preview?.summary, { escape, text });

  if (!changed.length) {
    return `${summaryHtml}`
      + `<div class="settings-diff-empty">${escape(text("settings_diff_no_changes", "No changes to apply."))}</div>`
      + invalidHtml;
  }

  const rows = shown.map((entry) => (
    `<div class="settings-diff__row">`
    + `<div class="settings-diff__head">`
    + `<div class="settings-diff__key">${escape(entry.key)}</div>`
    + `<span class="settings-diff__status">${escape(changedLabel)}</span></div>`
    + `<div class="settings-diff__compare">`
    + `<div class="settings-diff__value settings-diff__value--old">`
    + `<span>${escape(currentLabel)}</span><code>${escape(entry.current)}</code></div>`
    + `<div class="settings-diff__arrow" aria-hidden="true">&gt;</div>`
    + `<div class="settings-diff__value settings-diff__value--new">`
    + `<span>${escape(nextLabel)}</span><code>${escape(entry.value)}</code></div>`
    + `</div></div>`
  )).join("");

  const moreHtml = hiddenCount
    ? `<div class="settings-diff-more">${escape(
        text("settings_diff_more", "{count} more changes hidden", { count: hiddenCount }),
      )}</div>`
    : "";

  return `${summaryHtml}<div class="settings-diff__list">${rows}</div>${moreHtml}${invalidHtml}`;
}
