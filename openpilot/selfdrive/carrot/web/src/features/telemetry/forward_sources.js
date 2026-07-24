export const TELEMETRY_FORWARD_SOURCE_ORDER = Object.freeze([
  "lead",
  "vision",
  "front",
  "scc",
  "corner",
]);

export const TELEMETRY_FORWARD_SOURCE_STYLE = Object.freeze({
  vision: Object.freeze({
    shape: "circle",
    tone: "vision",
    labelKey: "replay_sensor_source_vision",
    label: "Camera",
  }),
  front: Object.freeze({
    shape: "diamond",
    tone: "track",
    labelKey: "replay_sensor_source_front",
    label: "Front radar",
  }),
  scc: Object.freeze({
    shape: "square",
    tone: "track",
    labelKey: "replay_sensor_source_scc",
    label: "SCC radar",
  }),
  corner: Object.freeze({
    shape: "triangle",
    tone: "track",
    labelKey: "replay_sensor_source_corner",
    label: "Corner radar",
  }),
  fusion: Object.freeze({
    shape: "circle",
    tone: "lead",
    labelKey: "replay_sensor_source_fusion",
    label: "Combined sensors",
  }),
  lead: Object.freeze({
    shape: "circle",
    tone: "lead",
    labelKey: "replay_sensor_source_lead",
    label: "Lead",
  }),
  unknown: Object.freeze({
    shape: "circle",
    tone: "track",
    labelKey: "replay_sensor_source_unknown",
    label: "Unidentified",
  }),
});

const RADAR_SOURCE_BY_CODE = Object.freeze(["front", "scc", "corner", "corner"]);

export function normalizeTelemetryForwardSource(value, fallback = "unknown") {
  if (value !== null && value !== undefined && String(value).trim() !== "" && Number.isFinite(Number(value))) {
    return RADAR_SOURCE_BY_CODE[Math.max(0, Math.min(3, Number(value)))] || fallback;
  }
  const source = String(value || "").trim().toLowerCase();
  if (["vision", "model", "camera"].includes(source)) return "vision";
  if (["front", "frontradar", "front_radar", "radar", "live_tracks"].includes(source)) return "front";
  if (source === "scc") return "scc";
  if (["corner", "corner235", "corner180", "corner430"].includes(source)) return "corner";
  if (source === "fusion") return "fusion";
  if (source === "lead") return "lead";
  return Object.prototype.hasOwnProperty.call(TELEMETRY_FORWARD_SOURCE_STYLE, fallback)
    ? fallback
    : "unknown";
}

export function telemetryForwardSourceStyle(value, fallback = "unknown") {
  return TELEMETRY_FORWARD_SOURCE_STYLE[normalizeTelemetryForwardSource(value, fallback)]
    || TELEMETRY_FORWARD_SOURCE_STYLE.unknown;
}

export function telemetryForwardSourceLabel(value, text, fallback = "unknown") {
  const style = telemetryForwardSourceStyle(value, fallback);
  return typeof text === "function" ? text(style.labelKey, style.label) : style.label;
}
