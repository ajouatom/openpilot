const DEFAULT_CAPACITY = 240;
const MAX_CAPACITY = 1200;
const MAX_ARRAY_ITEMS = 64;
const MAX_OBJECT_KEYS = 64;
const MAX_DEPTH = 6;

function finite(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function capacityValue(value) {
  const number = Math.floor(finite(value, DEFAULT_CAPACITY));
  return Math.max(1, Math.min(MAX_CAPACITY, number));
}

function snapshotValue(value, depth = 0) {
  if (value === null || value === undefined) return value ?? null;
  if (["string", "number", "boolean"].includes(typeof value)) return value;
  if (depth >= MAX_DEPTH) return "[depth-limit]";
  if (Array.isArray(value)) {
    return Object.freeze(value.slice(0, MAX_ARRAY_ITEMS).map((item) => snapshotValue(item, depth + 1)));
  }
  if (typeof value === "object") {
    const output = {};
    for (const key of Object.keys(value).slice(0, MAX_OBJECT_KEYS)) {
      output[key] = snapshotValue(value[key], depth + 1);
    }
    return Object.freeze(output);
  }
  return String(value);
}

export function createArTrace(options = {}) {
  const capacity = capacityValue(options.capacity);
  const now = typeof options.now === "function"
    ? options.now
    : () => globalThis.performance?.now?.() ?? Date.now();
  let enabled = options.enabled === true;
  let nextSequence = 1;
  let dropped = 0;
  let entries = [];

  function record(kind, payload = {}, atMs = now()) {
    if (!enabled) return null;
    const entry = Object.freeze({
      sequence: nextSequence++,
      kind: String(kind || "unknown"),
      atMs: finite(atMs, null),
      payload: snapshotValue(payload),
    });
    if (entries.length >= capacity) {
      entries.shift();
      dropped += 1;
    }
    entries.push(entry);
    return entry;
  }

  function clear() {
    const hadEntries = entries.length > 0 || dropped > 0;
    entries = [];
    dropped = 0;
    nextSequence = 1;
    return hadEntries;
  }

  function setEnabled(value, settings = {}) {
    if (settings.clear === true) clear();
    enabled = value === true;
    return enabled;
  }

  function status() {
    return Object.freeze({ enabled, capacity, size: entries.length, dropped });
  }

  function snapshot() {
    return Object.freeze({
      ...status(),
      entries: Object.freeze([...entries]),
    });
  }

  return Object.freeze({
    record,
    clear,
    setEnabled,
    isEnabled: () => enabled,
    status,
    snapshot,
  });
}

export const AR_TRACE_LIMITS = Object.freeze({
  defaultCapacity: DEFAULT_CAPACITY,
  maxCapacity: MAX_CAPACITY,
});
