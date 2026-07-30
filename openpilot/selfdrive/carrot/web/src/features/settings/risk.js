/**
 * Per-parameter risk level, declared in carrot_settings.json.
 *
 * A parameter that directly affects driving (the cruise-speed incident's
 * ApplyModelSpeed is the archetype) can carry `"risk": "high"`. The badge that
 * renders from it warns before a value is even touched — the point being that
 * the warning lives in the data, so marking a parameter risky never means
 * editing the settings code.
 */

export const RISK_LEVELS = Object.freeze(["high", "medium"]);

const RISK_LABEL_KEYS = Object.freeze({
  high: "setting_risk_high",
  medium: "setting_risk_medium",
});

const RISK_TONES = Object.freeze({
  high: "warning",
  medium: "warning",
});

export function getSettingRiskLevel(item) {
  const level = String(item?.risk || "").trim().toLowerCase();
  return RISK_LEVELS.includes(level) ? level : null;
}

export function renderSettingRiskBadge(item, options = {}) {
  const level = getSettingRiskLevel(item);
  if (!level) return "";
  const escape = typeof options.escape === "function" ? options.escape : (value) => String(value ?? "");
  const text = typeof options.text === "function" ? options.text : (_key, fallback) => fallback;
  const label = level === "high"
    ? text(RISK_LABEL_KEYS.high, "주의")
    : text(RISK_LABEL_KEYS.medium, "참고");
  const tone = RISK_TONES[level];
  return `<span class="chip chip--compact chip--${tone} setting-risk-badge setting-risk-badge--${level}" data-risk-level="${level}">${escape(label)}</span>`;
}
