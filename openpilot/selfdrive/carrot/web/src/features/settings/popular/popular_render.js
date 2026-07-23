/**
 * Renders the popular-value chip and detail block.
 *
 * The ordering/range maths is in popular_values.js; this turns a display entry
 * into markup, keeping the exact template (including whitespace) the settings
 * page produced so there is no visual change. Formatting, translation,
 * escaping and the two aux-derived strings (title, "updated" time) are injected
 * so the module stays pure.
 */

import { popularPrimaryCount, popularSummaryValues } from "./popular_values.js";

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

function summaryFor(entry, formatValue) {
  const summary = popularSummaryValues(entry);
  if (!summary.length) return null;
  const sample = popularPrimaryCount(entry);
  const values = summary.map((item) => formatValue(item?.value)).filter(Boolean);
  if (!sample || !values.length) return null;
  return { sample, values };
}

export function renderPopularChipText(entry, options = {}) {
  const formatValue = options.formatValue || identityEscape;
  const text = options.text || defaultText;
  const summary = summaryFor(entry, formatValue);
  if (!summary) return "";

  const label = text("setting_popular_value_chip_label", "내 차종 인기값");
  if (summary.values.length === 1) {
    return text("setting_popular_value_chip", "{label} ({sample}대) {value}", {
      label,
      sample: summary.sample,
      value: summary.values[0],
    });
  }
  return text("setting_popular_value_chip_tied", "{label} (각 {sample}대) {values}", {
    label,
    sample: summary.sample,
    values: summary.values.join(" · "),
  });
}

export function renderPopularChipHtml(entry, options = {}) {
  const formatValue = options.formatValue || identityEscape;
  const text = options.text || defaultText;
  const escape = options.escape || identityEscape;
  const summary = summaryFor(entry, formatValue);
  if (!summary) return "";

  const sampleText = summary.values.length === 1
    ? text("setting_popular_value_chip_sample", "{sample}대", { sample: summary.sample })
    : text("setting_popular_value_chip_each_sample", "각 {sample}대", { sample: summary.sample });
  return `
    <span class="setting-popular-value-chip__car">${escape(text("setting_popular_value_chip_label", "내 차종 인기값"))}</span>
    <span class="setting-popular-value-chip__label">(</span><span class="setting-popular-value-chip__accent">${escape(sampleText)}</span><span class="setting-popular-value-chip__label">)</span>
    <span class="setting-popular-value-chip__accent">${escape(summary.values.join(" · "))}</span>
  `;
}

export function renderPopularDetailHtml(entry, options = {}) {
  const formatValue = options.formatValue || identityEscape;
  const text = options.text || defaultText;
  const escape = options.escape || identityEscape;
  const title = String(options.title || "");
  const updatedText = String(options.updatedText || "");

  const values = Array.isArray(entry?.top_values) ? entry.top_values : [];
  if (!values.length) {
    return `<div class="setting-popular-detail"><div class="setting-popular-detail__empty">${escape(text("setting_popular_value_empty", "표시할 설정값이 없습니다."))}</div></div>`;
  }

  const counts = values.map((item) => Number(item?.count ?? 0)).filter((count) => Number.isFinite(count) && count > 0);
  const maxCount = Math.max(1, ...counts);

  const rows = values.map((item) => {
    const value = formatValue(item?.value);
    const count = Number(item?.count ?? 0);
    const normalizedCount = Number.isFinite(count) ? Math.max(0, count) : 0;
    const countText = text("setting_popular_value_chip_sample", "{sample} vehicles", {
      sample: normalizedCount,
    });
    return `
      <button type="button" class="setting-popular-detail__row" data-setting-popular-value="${escape(item?.value ?? "")}">
        <span class="setting-popular-detail__marker" aria-hidden="true"></span>
        <span class="setting-popular-detail__main">
          <span class="setting-popular-detail__value">${escape(value)}</span>
          ${values.length > 1 ? `<progress class="setting-popular-detail__bar" max="${maxCount}" value="${normalizedCount}" aria-label="${escape(countText)}"></progress>` : ""}
        </span>
        <span class="setting-popular-detail__count">${escape(countText)}</span>
      </button>
    `;
  }).join("");

  const updatedHtml = updatedText
    ? `<div class="setting-popular-detail__updated">${escape(text("setting_popular_value_updated", "최근 업데이트: {time}", { time: updatedText }))}</div>`
    : "";

  return `
    <div class="setting-popular-detail${values.length <= 1 ? " setting-popular-detail--single" : ""}">
      <div class="setting-popular-detail__head">
        <span class="setting-popular-detail__name">${escape(title)}</span>
        <span class="setting-popular-detail__range">${escape(text("setting_popular_value_common_values", "많이 쓰는 값"))}</span>
      </div>
      <div class="setting-popular-detail__rows">${rows}</div>
      ${updatedHtml}
    </div>
  `;
}
