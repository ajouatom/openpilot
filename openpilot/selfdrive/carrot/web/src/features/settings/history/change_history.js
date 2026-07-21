/**
 * Renders the "modification history" block on a setting's detail screen.
 *
 * The block deliberately reuses the popular-values block's structure
 * (__head / __rows / __row) so the detail screen keeps one visual language
 * instead of growing a second one.
 *
 * Timestamps are emitted as <time datetime="..."> carrying an ISO 8601 value,
 * with the same instant shown to the reader down to the second.
 */

export const CHANGE_SOURCE_LABEL_KEYS = Object.freeze({
  web_ui: "setting_history_source_web",
  profile: "setting_history_source_profile",
  restore: "setting_history_source_restore",
  reset_defaults: "setting_history_source_reset",
  intro: "setting_history_source_intro",
  undo: "setting_history_source_undo",
  device: "setting_history_source_device",
  unknown: "setting_history_source_unknown",
});

/**
 * Stand-in for the app's getUIText(key, fallback, params). It interpolates
 * {name} placeholders the same way, so the module renders correctly even when
 * no translation provider is supplied.
 */
function defaultText(_key, fallback, params) {
  const template = String(fallback ?? "");
  if (!params) return template;
  return template.replace(/\{(\w+)\}/g, (match, name) => (
    Object.prototype.hasOwnProperty.call(params, name) ? String(params[name]) : match
  ));
}

export function toIsoTimestamp(epochSec) {
  const seconds = Number(epochSec);
  if (!Number.isFinite(seconds) || seconds <= 0) return "";
  try {
    return new Date(seconds * 1000).toISOString();
  } catch {
    return "";
  }
}

/**
 * An exact wall-clock time, down to the second.
 *
 * A relative string ("3분 전") reads nicely but is useless for the job this
 * block exists to do: lining a change up against a git pull, a drive, or
 * somebody else's report. Two changes seconds apart have to be
 * distinguishable, so the seconds are always shown.
 */
export function formatChangeTimestamp(epochSec, options = {}) {
  const seconds = Number(epochSec);
  const iso = toIsoTimestamp(seconds);
  if (!iso) return { iso: "", absolute: "" };

  const locale = options.locale || undefined;
  let absolute = "";
  try {
    absolute = new Date(seconds * 1000).toLocaleString(locale, {
      year: "numeric", month: "2-digit", day: "2-digit",
      hour: "2-digit", minute: "2-digit", second: "2-digit",
      hour12: false,
    });
  } catch {
    absolute = iso;
  }
  return { iso, absolute };
}

function requireFunction(value, fallback) {
  return typeof value === "function" ? value : fallback;
}

// The visible text is already the complete, unambiguous timestamp, so there is
// nothing left for an <abbr> to expand for assistive technology.
function renderTimeCell(record, options) {
  const escape = options.escape;
  const { iso, absolute } = formatChangeTimestamp(record?.ts, options);
  if (!iso) return `<span class="setting-history-detail__time">-</span>`;
  return `<time class="setting-history-detail__time" datetime="${escape(iso)}">`
    + `${escape(absolute)}</time>`;
}

export function renderSettingHistoryHtml(changes, options = {}) {
  const escape = requireFunction(options.escape, (value) => String(value ?? ""));
  const text = requireFunction(options.text, defaultText);
  const baseFormat = requireFunction(options.formatValue, (value) => String(value ?? ""));
  // formatFor(record, value) lets an across-parameter list format each value by
  // its own parameter (units, ON/OFF); it falls back to the plain formatter.
  const formatValue = (record, value) => (
    typeof options.formatFor === "function" ? options.formatFor(record, value) : baseFormat(value)
  );
  const records = Array.isArray(changes) ? changes : [];

  const head = `<div class="setting-history-detail__head">`
    + `<span class="setting-history-detail__name">${escape(text("setting_history_title", "수정이력"))}</span>`
    + (records.length
      ? `<span class="setting-history-detail__range">${escape(text("setting_history_count", "최근 {n}건", { n: records.length }))}</span>`
      : "")
    + `</div>`;

  if (!records.length) {
    return `<div class="setting-history-detail setting-history-detail--empty">${head}`
      + `<div class="setting-history-detail__empty">`
      + `${escape(text("setting_history_empty", "기록된 변경이 없습니다."))}</div></div>`;
  }

  const rows = records.map((record, index) => {
    const source = String(record?.source || "unknown");
    const sourceKey = CHANGE_SOURCE_LABEL_KEYS[source] || CHANGE_SOURCE_LABEL_KEYS.unknown;
    const sourceBadge = `<span class="setting-history-detail__badge">${escape(text(sourceKey, source))}</span>`;
    const engaged = record?.engaged
      ? `<span class="setting-history-detail__badge setting-history-detail__badge--engaged">`
        + `${escape(text("setting_history_engaged", "주행중"))}</span>`
      : "";
    // Only the newest entry can be undone, and only in a single-parameter
    // history: an across-parameter list (withName) has no one row to undo, and
    // rolling back an older entry would silently discard everything after it.
    const undo = index === 0 && !options.withName
      ? `<button type="button" class="setting-history-detail__undo"`
        + ` data-setting-history-undo="${escape(record?.prev ?? "")}">`
        + `${escape(text("setting_history_undo", "되돌리기"))}</button>`
      : "";

    // Across-parameter listings (the fingerprint dialog) name each row; a
    // single-parameter history omits the name because the screen already shows
    // which parameter it is. displayName maps the raw key to a human title
    // (e.g. ShowTpms -> "타이어공기압 표시") so non-developers recognise it.
    const displayName = typeof options.displayName === "function"
      ? options.displayName(record)
      : record?.name;
    const nameCell = options.withName && displayName
      ? `<span class="setting-history-detail__param">${escape(displayName)}</span>`
      : "";

    return `<div class="setting-history-detail__row">`
      + `<span class="setting-history-detail__marker" aria-hidden="true"></span>`
      + `<span class="setting-history-detail__main">`
      + nameCell
      + `<span class="setting-history-detail__change">`
      + `<span class="setting-history-detail__prev">${escape(formatValue(record, record?.prev))}</span>`
      + `<span class="setting-history-detail__arrow" aria-hidden="true">→</span>`
      + `<span class="setting-history-detail__next">${escape(formatValue(record, record?.next))}</span>`
      + `</span>`
      + `<span class="setting-history-detail__meta">${renderTimeCell(record, { ...options, escape })}`
      + `${sourceBadge}${engaged}</span>`
      + `</span>${undo}</div>`;
  }).join("");

  return `<div class="setting-history-detail">${head}`
    + `<div class="setting-history-detail__rows">${rows}</div></div>`;
}
