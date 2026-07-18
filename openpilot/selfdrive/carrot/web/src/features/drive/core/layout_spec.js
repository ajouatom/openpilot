import { resolveDriveContentDescriptor } from "./registry.js";

export const ORIENTATION = Object.freeze({
  HORIZONTAL: "horizontal",
  VERTICAL: "vertical",
});

export const MODE = Object.freeze({
  SPLIT: "split",
  AREA_1: "area_1",
  AREA_2: "area_2",
});

// Compatibility constants for classic consumers. Registry metadata is the
// authority for validation and ordering; new code must not enumerate CONTENT.
export const CONTENT = Object.freeze({
  VISION: "vision",
  NAVIGATION: "navigation",
});

export const SETTINGS = Object.freeze({
  [ORIENTATION.HORIZONTAL]: Object.freeze({
    mode: "carrot_navi_horizontal_mode",
    area1Content: "carrot_navi_horizontal_area_1",
    area2Content: "carrot_navi_horizontal_area_2",
    ratio: "carrot_navi_split_ratio",
  }),
  [ORIENTATION.VERTICAL]: Object.freeze({
    mode: "carrot_navi_vertical_mode",
    area1Content: "carrot_navi_vertical_area_1",
    area2Content: "carrot_navi_vertical_area_2",
    ratio: "carrot_navi_vertical_split_ratio",
  }),
});

export const settingKeys = Object.freeze(Object.values(SETTINGS).flatMap((entry) => Object.values(entry)));

export const DRIVE_LAYOUT_CONTENT_DEFAULTS = Object.freeze({
  primary: CONTENT.VISION,
  secondary: CONTENT.NAVIGATION,
});

export function normalizeOrientation(value) {
  return String(value || "").toLowerCase() === ORIENTATION.VERTICAL
    ? ORIENTATION.VERTICAL
    : ORIENTATION.HORIZONTAL;
}

export function orientationForViewport(target = globalThis, documentRoot = target?.document) {
  const shared = target?.CarrotLayout?.orientation?.();
  if (shared === "portrait") return ORIENTATION.VERTICAL;
  if (shared === "landscape") return ORIENTATION.HORIZONTAL;
  const viewport = target?.visualViewport;
  const width = Number(
    viewport?.width
    || documentRoot?.documentElement?.clientWidth
    || target?.innerWidth
    || 0,
  );
  const height = Number(
    viewport?.height
    || documentRoot?.documentElement?.clientHeight
    || target?.innerHeight
    || 0,
  );
  return height >= width ? ORIENTATION.VERTICAL : ORIENTATION.HORIZONTAL;
}

export function normalizeMode(value) {
  return Object.values(MODE).includes(value) ? value : MODE.SPLIT;
}

export function normalizeRatio(value) {
  const numeric = Number(value);
  const ratio = Number.isFinite(numeric) ? numeric : 0.5;
  const normalized = Math.max(0.3, Math.min(0.7, Math.round(ratio / 0.05) * 0.05));
  return Number(normalized.toFixed(2));
}

export function keysFor(value) {
  return SETTINGS[normalizeOrientation(value)];
}

function settingValue(settings, defaults, key) {
  if (Object.prototype.hasOwnProperty.call(settings || {}, key)) return settings[key];
  return defaults?.[key];
}

function resolvedDefaults(defaults) {
  const value = typeof defaults === "function" ? defaults() : defaults;
  return value && typeof value === "object" ? value : {};
}

function normalizeContentDefaults(value) {
  const defaults = value && typeof value === "object" ? value : DRIVE_LAYOUT_CONTENT_DEFAULTS;
  return Object.freeze({
    primary: typeof defaults.primary === "string" ? defaults.primary : DRIVE_LAYOUT_CONTENT_DEFAULTS.primary,
    secondary: typeof defaults.secondary === "string" ? defaults.secondary : DRIVE_LAYOUT_CONTENT_DEFAULTS.secondary,
  });
}

export function normalizeDriveLayoutContents(values = {}, options = {}) {
  const registry = options.registry;
  const source = options.source || "live";
  const defaults = normalizeContentDefaults(options.contentDefaults);

  // Area 1 always resolves first. Its singleton selection is then authoritative
  // while area 2 resolves its requested id, field fallback, and registry scan.
  const area1 = resolveDriveContentDescriptor(registry, values.area1, {
    slot: "primary",
    source,
    fallbackId: defaults.primary,
    selectedIds: [],
  });
  const area2 = resolveDriveContentDescriptor(registry, values.area2, {
    slot: "secondary",
    source,
    fallbackId: defaults.secondary,
    selectedIds: [area1.id],
  });

  return Object.freeze({ area1: area1.id, area2: area2.id });
}

export function readDriveLayout(settings, value, options = {}) {
  const orientation = normalizeOrientation(value);
  const keys = keysFor(orientation);
  const defaults = resolvedDefaults(options.settingDefaults);
  const contents = normalizeDriveLayoutContents({
    area1: settingValue(settings, defaults, keys.area1Content),
    area2: settingValue(settings, defaults, keys.area2Content),
  }, options);

  return Object.freeze({
    orientation,
    mode: normalizeMode(settingValue(settings, defaults, keys.mode)),
    ratio: normalizeRatio(settingValue(settings, defaults, keys.ratio)),
    area1Content: contents.area1,
    area2Content: contents.area2,
  });
}

function assertRegistry(registry) {
  if (!registry || typeof registry !== "object") throw new TypeError("drive layout registry is required");
  for (const method of ["register", "get", "list", "create", "has"]) {
    if (typeof registry[method] !== "function") throw new TypeError(`drive layout registry.${method} is required`);
  }
  return registry;
}

export function createDriveLayoutSpec(options = {}) {
  const registry = assertRegistry(options.registry);
  const contentDefaults = normalizeContentDefaults(options.contentDefaults);
  const settingDefaults = options.settingDefaults || {};
  const source = options.source || "live";

  function normalizeContents(values, context = {}) {
    return normalizeDriveLayoutContents(values, {
      registry,
      source: context.source || source,
      contentDefaults: context.contentDefaults || contentDefaults,
    });
  }

  function read(settings, orientation, context = {}) {
    return readDriveLayout(settings, orientation, {
      registry,
      source: context.source || source,
      contentDefaults: context.contentDefaults || contentDefaults,
      settingDefaults: context.settingDefaults || settingDefaults,
    });
  }

  return Object.freeze({
    ORIENTATION,
    MODE,
    CONTENT,
    SETTINGS,
    settingKeys,
    normalizeOrientation,
    orientationForViewport,
    normalizeMode,
    normalizeRatio,
    keysFor,
    normalizeContents,
    read,
  });
}

export function installDriveLayoutSpecFacade(spec, target = globalThis) {
  if (!spec || typeof spec !== "object" || typeof spec.read !== "function") {
    throw new TypeError("drive layout spec facade requires a layout spec");
  }
  if (!target || (typeof target !== "object" && typeof target !== "function")) {
    throw new TypeError("drive layout spec facade target must be an object");
  }
  target.CarrotDriveLayoutSpec = spec;
  return spec;
}
