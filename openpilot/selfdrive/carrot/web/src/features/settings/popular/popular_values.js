/**
 * Fleet "popular value" maths: normalize a raw value to a number, range-check
 * it, order the candidates, and assemble the display entry.
 *
 * Split out of the settings page because the ordering and range rules are easy
 * to get subtly wrong and had no tests. Formatting (which needs the catalog and
 * the language) is injected so this stays pure.
 *
 * `range` is the parameter definition, read only for `min`/`max`.
 */

const MAX_TOP_VALUES = 10;

function bounds(range) {
  return { min: Number(range?.min), max: Number(range?.max) };
}

export function normalizePopularNumericValue(range, raw) {
  if (raw === null || raw === undefined || raw === "") return null;
  const { min, max } = bounds(range);
  const text = String(raw).trim().toLowerCase();
  if (min === 0 && max === 1) {
    if (text === "1" || text === "true" || text === "on") return 1;
    if (text === "0" || text === "false" || text === "off") return 0;
  }
  const value = Number(raw);
  return Number.isFinite(value) ? value : null;
}

export function isPopularValueInRange(range, raw) {
  const { min, max } = bounds(range);
  if (!Number.isFinite(min) || !Number.isFinite(max)) return true;
  const value = normalizePopularNumericValue(range, raw);
  if (value === null) return false;
  return value >= min && value <= max;
}

/**
 * Order: most common first; ties broken by numeric value, then by formatted
 * text, then by original position so the sort is stable.
 */
export function comparePopularItems(range, a, b, options = {}) {
  const formatValue = typeof options.formatValue === "function" ? options.formatValue : (value) => String(value ?? "");
  const locale = options.locale;
  const aIndex = Number(options.aIndex || 0);
  const bIndex = Number(options.bIndex || 0);

  const aCount = Number(a?.count ?? 0);
  const bCount = Number(b?.count ?? 0);
  const safeA = Number.isFinite(aCount) ? aCount : 0;
  const safeB = Number.isFinite(bCount) ? bCount : 0;
  if (safeA !== safeB) return safeB - safeA;

  const aValue = normalizePopularNumericValue(range, a?.value);
  const bValue = normalizePopularNumericValue(range, b?.value);
  if (aValue !== null && bValue !== null && aValue !== bValue) return aValue - bValue;
  if (aValue !== null && bValue === null) return -1;
  if (aValue === null && bValue !== null) return 1;

  const order = String(formatValue(a?.value)).localeCompare(
    String(formatValue(b?.value)),
    locale,
    { numeric: true, sensitivity: "base" },
  );
  if (order !== 0) return order;

  return aIndex - bIndex;
}

/**
 * The entry to render, or null when it should not show: no sample, or the
 * headline value is out of range. `top_values` is filtered to in-range,
 * positive-count items, ordered, and capped.
 */
export function buildPopularDisplayEntry(range, entry, options = {}) {
  if (!entry || typeof entry !== "object") return null;
  const sample = Number(entry.sample ?? entry.sample_count ?? 0);
  if (!Number.isFinite(sample) || sample < 1) return null;
  if (!isPopularValueInRange(range, entry.value)) return null;

  const topValues = Array.isArray(entry.top_values)
    ? entry.top_values
      .map((item, index) => ({ item, index }))
      .filter(({ item }) => {
        const count = Number(item?.count ?? 0);
        return Number.isFinite(count) && count > 0 && isPopularValueInRange(range, item?.value);
      })
      .sort((a, b) => comparePopularItems(range, a.item, b.item, {
        ...options,
        aIndex: a.index,
        bIndex: b.index,
      }))
      .slice(0, MAX_TOP_VALUES)
      .map(({ item }) => item)
    : [];

  return { ...entry, top_values: topValues };
}

export function popularPrimaryCount(entry) {
  const values = Array.isArray(entry?.top_values) ? entry.top_values : [];
  const count = Number(values[0]?.count ?? entry?.top_count ?? entry?.count ?? 0);
  return Number.isFinite(count) ? count : 0;
}

/**
 * The one or two values worth putting on the summary chip: only when the top
 * count is shared by at most two values (a clear winner or a clean tie).
 */
export function popularSummaryValues(entry) {
  const values = Array.isArray(entry?.top_values) ? entry.top_values : [];
  if (!values.length) return [];

  const topCount = popularPrimaryCount(entry);
  if (topCount < 2) return [];

  const tied = values.filter((item) => {
    const count = Number(item?.count ?? 0);
    return Number.isFinite(count) && count === topCount;
  });
  return tied.length <= 2 ? tied : [];
}
