/**
 * The top of the settings-fingerprint dialog: the code, one plain line of what
 * it means, and a small note with the setting count and the integrity verdict.
 *
 * Pure and escape/text-injected, so the wording and structure are testable and
 * the settings page only has to fetch the data.
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

export function integrityText(integrity, text = defaultText) {
  if (!integrity) return "";
  return integrity.valid
    ? text("setting_fingerprint_intact", "기록 {n}건 · 이상 없음", { n: integrity.checked ?? 0 })
    : text("setting_fingerprint_broken", "기록이 손상됨 ({n})", { n: (integrity.broken_at ?? 0) + 1 });
}

/**
 * The status line: whether the current settings differ from the saved
 * reference, and how many changed. This is what makes the code useful without
 * the user writing anything down.
 */
function statusLine(data, escape, text) {
  if (!data.baseline) return "";
  if (data.changed) {
    const label = text("setting_fingerprint_changed", "기준 이후 {n}개 바뀜", { n: data.changed_count ?? 0 });
    return `<div class="setting-fingerprint__status setting-fingerprint__status--changed">${escape(label)}</div>`;
  }
  return `<div class="setting-fingerprint__status setting-fingerprint__status--same">`
    + `${escape(text("setting_fingerprint_same", "기준과 같아요"))}</div>`;
}

export function renderFingerprintSummary(data = {}, options = {}) {
  const escape = typeof options.escape === "function" ? options.escape : identityEscape;
  const text = typeof options.text === "function" ? options.text : defaultText;

  const code = escape(data.fingerprint || "-");
  const hint = escape(text("setting_fingerprint_hint", "설정이 같으면 코드도 같아요"));
  const note = `${escape(text("setting_fingerprint_count", "설정 {n}개", { n: data.count ?? 0 }))}`
    + ` · ${escape(integrityText(data.integrity, text))}`;

  // Offered only when something changed — there is nothing to re-baseline when
  // the settings already match the reference.
  const saveButton = data.baseline && data.changed
    ? `<button type="button" class="setting-fingerprint__save" data-setting-fingerprint-save>`
      + `${escape(text("setting_fingerprint_save", "지금을 기준으로"))}</button>`
    : "";

  return `<div class="setting-fingerprint">`
    + `<div class="setting-fingerprint__code">${code}</div>`
    + `<div class="setting-fingerprint__hint">${hint}</div>`
    + statusLine(data, escape, text)
    + saveButton
    + `<div class="setting-fingerprint__note">${note}</div>`
    + `</div>`;
}
