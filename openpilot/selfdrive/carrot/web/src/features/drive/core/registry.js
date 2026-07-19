export const DRIVE_CONTENT_SLOTS = Object.freeze(["primary", "secondary"]);
export const DRIVE_CONTENT_SOURCES = Object.freeze(["live", "replay"]);
export const NO_SUPPORTED_CONTENT = "NO_SUPPORTED_CONTENT";

const CONTENT_ID_PATTERN = /^[a-z][a-z0-9_]*$/;
const DESCRIPTOR_FIELDS = Object.freeze([
  "id",
  "labelKey",
  "factory",
  "supportedSlots",
  "supportedSources",
  "singleton",
]);
const CATALOG_FIELDS = Object.freeze(["schemaVersion", "defaults", "contents"]);
const CATALOG_CONTENT_FIELDS = Object.freeze([
  "id",
  "labelKey",
  "supportedSlots",
  "supportedSources",
  "singleton",
]);
const CATALOG_DEFAULT_FIELDS = Object.freeze(["primary", "secondary"]);
const CONTEXT_FIELDS = Object.freeze(["slot", "source", "fallbackId", "selectedIds"]);
const CONTENT_INSTANCE_METHODS = Object.freeze([
  "mount",
  "activate",
  "deactivate",
  "resize",
  "status",
  "destroy",
]);

export class DriveContentRegistryError extends Error {
  constructor(code, message) {
    super(message);
    this.name = "DriveContentRegistryError";
    this.code = code;
  }
}

function fail(code, message) {
  throw new DriveContentRegistryError(code, message);
}

function isRecord(value) {
  return Boolean(value && typeof value === "object" && !Array.isArray(value));
}

function assertExactFields(value, fields, label, code) {
  if (!isRecord(value)) fail(code, `${label} must be an object`);
  const actual = Object.keys(value).sort();
  const expected = [...fields].sort();
  if (actual.length !== expected.length || actual.some((field, index) => field !== expected[index])) {
    fail(code, `${label} must contain exactly: ${fields.join(", ")}`);
  }
}

function normalizeId(value, label, code) {
  if (typeof value !== "string" || !CONTENT_ID_PATTERN.test(value)) {
    fail(code, `${label} must match ${CONTENT_ID_PATTERN}`);
  }
  return value;
}

function normalizeLabelKey(value, label, code) {
  if (typeof value !== "string" || !value.trim()) fail(code, `${label} must be a non-empty string`);
  return value.trim();
}

function normalizeEnumList(value, allowed, label, code) {
  if (!Array.isArray(value) || value.length === 0) fail(code, `${label} must be a non-empty array`);
  const normalized = [];
  const seen = new Set();
  for (const entry of value) {
    if (typeof entry !== "string" || !allowed.includes(entry)) {
      fail(code, `${label} contains unsupported value: ${String(entry)}`);
    }
    if (seen.has(entry)) fail(code, `${label} must not contain duplicates`);
    seen.add(entry);
    normalized.push(entry);
  }
  return Object.freeze(normalized);
}

function normalizeCatalogContent(content, index) {
  const code = "INVALID_CONTENT_CATALOG";
  const label = `content catalog entry ${index}`;
  assertExactFields(content, CATALOG_CONTENT_FIELDS, label, code);
  if (typeof content.singleton !== "boolean") fail(code, `${label}.singleton must be boolean`);
  return Object.freeze({
    id: normalizeId(content.id, `${label}.id`, code),
    labelKey: normalizeLabelKey(content.labelKey, `${label}.labelKey`, code),
    supportedSlots: normalizeEnumList(content.supportedSlots, DRIVE_CONTENT_SLOTS, `${label}.supportedSlots`, code),
    supportedSources: normalizeEnumList(content.supportedSources, DRIVE_CONTENT_SOURCES, `${label}.supportedSources`, code),
    singleton: content.singleton,
  });
}

export function validateDriveContentCatalog(catalog) {
  const code = "INVALID_CONTENT_CATALOG";
  assertExactFields(catalog, CATALOG_FIELDS, "content catalog", code);
  if (catalog.schemaVersion !== 1) fail(code, "content catalog schemaVersion must be 1");
  assertExactFields(catalog.defaults, CATALOG_DEFAULT_FIELDS, "content catalog defaults", code);
  if (!Array.isArray(catalog.contents) || catalog.contents.length === 0) {
    fail(code, "content catalog contents must be a non-empty array");
  }

  const contents = catalog.contents.map(normalizeCatalogContent);
  const ids = new Set();
  for (const content of contents) {
    if (ids.has(content.id)) fail(code, `duplicate content catalog id: ${content.id}`);
    ids.add(content.id);
  }

  const defaults = Object.freeze({
    primary: normalizeId(catalog.defaults.primary, "content catalog defaults.primary", code),
    secondary: normalizeId(catalog.defaults.secondary, "content catalog defaults.secondary", code),
  });
  for (const [slot, id] of Object.entries(defaults)) {
    if (!ids.has(id)) fail(code, `content catalog defaults.${slot} references unknown id: ${id}`);
  }

  return Object.freeze({
    schemaVersion: 1,
    defaults,
    contents: Object.freeze(contents),
  });
}

function normalizeDescriptor(descriptor) {
  const code = "INVALID_DESCRIPTOR";
  assertExactFields(descriptor, DESCRIPTOR_FIELDS, "drive content descriptor", code);
  if (typeof descriptor.factory !== "function") fail(code, "drive content descriptor.factory must be a function");
  if (typeof descriptor.singleton !== "boolean") fail(code, "drive content descriptor.singleton must be boolean");
  return Object.freeze({
    id: normalizeId(descriptor.id, "drive content descriptor.id", code),
    labelKey: normalizeLabelKey(descriptor.labelKey, "drive content descriptor.labelKey", code),
    factory: descriptor.factory,
    supportedSlots: normalizeEnumList(
      descriptor.supportedSlots,
      DRIVE_CONTENT_SLOTS,
      "drive content descriptor.supportedSlots",
      code,
    ),
    supportedSources: normalizeEnumList(
      descriptor.supportedSources,
      DRIVE_CONTENT_SOURCES,
      "drive content descriptor.supportedSources",
      code,
    ),
    singleton: descriptor.singleton,
  });
}

function normalizeContext(context) {
  const code = "INVALID_CONTEXT";
  assertExactFields(context, CONTEXT_FIELDS, "drive content context", code);
  if (!DRIVE_CONTENT_SLOTS.includes(context.slot)) fail(code, `unsupported drive content slot: ${String(context.slot)}`);
  if (!DRIVE_CONTENT_SOURCES.includes(context.source)) fail(code, `unsupported drive content source: ${String(context.source)}`);
  const fallbackId = normalizeId(context.fallbackId, "drive content context.fallbackId", code);
  if (!Array.isArray(context.selectedIds)) fail(code, "drive content context.selectedIds must be an array");
  const selectedIds = context.selectedIds.map((id, index) => (
    normalizeId(id, `drive content context.selectedIds[${index}]`, code)
  ));
  return Object.freeze({
    slot: context.slot,
    source: context.source,
    fallbackId,
    selectedIds: Object.freeze(selectedIds),
  });
}

function assertRegistry(registry) {
  if (!registry || typeof registry !== "object") fail("INVALID_REGISTRY", "drive content registry is required");
  for (const method of ["register", "get", "list", "create", "has"]) {
    if (typeof registry[method] !== "function") fail("INVALID_REGISTRY", `drive content registry.${method} is required`);
  }
  return registry;
}

function supportsContext(descriptor, context) {
  return descriptor.supportedSlots.includes(context.slot)
    && descriptor.supportedSources.includes(context.source);
}

function conflictsWithSelection(descriptor, selectedIds) {
  return descriptor.singleton && selectedIds.has(descriptor.id);
}

function isContentInstance(value) {
  return Boolean(
    value
    && typeof value === "object"
    && CONTENT_INSTANCE_METHODS.every((method) => typeof value[method] === "function")
  );
}

export function resolveDriveContentDescriptor(registry, id, context) {
  const activeRegistry = assertRegistry(registry);
  const normalizedContext = normalizeContext(context);
  const selectedIds = new Set(normalizedContext.selectedIds);
  const eligible = (descriptor) => Boolean(
    descriptor
    && supportsContext(descriptor, normalizedContext)
    && !conflictsWithSelection(descriptor, selectedIds)
  );

  const requested = typeof id === "string" ? activeRegistry.get(id) : null;
  if (eligible(requested)) return requested;

  const fallback = activeRegistry.get(normalizedContext.fallbackId);
  if (eligible(fallback)) return fallback;

  for (const descriptor of activeRegistry.list(normalizedContext)) {
    if (eligible(descriptor)) return descriptor;
  }

  fail(
    NO_SUPPORTED_CONTENT,
    `No supported drive content for ${normalizedContext.slot}/${normalizedContext.source}`,
  );
}

export function createDriveContentRegistry() {
  const descriptors = [];
  const descriptorsById = new Map();
  const instancesById = new Map();
  let api;

  function register(descriptor) {
    const normalized = normalizeDescriptor(descriptor);
    if (descriptorsById.has(normalized.id)) {
      fail("DUPLICATE_CONTENT_ID", `drive content id is already registered: ${normalized.id}`);
    }
    descriptors.push(normalized);
    descriptorsById.set(normalized.id, normalized);
    return normalized;
  }

  function get(id) {
    return typeof id === "string" ? (descriptorsById.get(id) || null) : null;
  }

  function has(id) {
    return typeof id === "string" && descriptorsById.has(id);
  }

  function list(context) {
    const normalizedContext = normalizeContext(context);
    return Object.freeze(descriptors.filter((descriptor) => supportsContext(descriptor, normalizedContext)));
  }

  function create(id, context) {
    const normalizedContext = normalizeContext(context);
    const descriptor = resolveDriveContentDescriptor(api, id, normalizedContext);
    if (instancesById.has(descriptor.id)) return instancesById.get(descriptor.id);
    const instance = descriptor.factory(normalizedContext);
    if (!isContentInstance(instance)) {
      fail(
        "INVALID_CONTENT_INSTANCE",
        `drive content factory ${descriptor.id} must return a DriveContent instance`,
      );
    }
    instancesById.set(descriptor.id, instance);
    return instance;
  }

  api = Object.freeze({ register, get, list, create, has });
  return api;
}

function factoryMap(factories) {
  if (factories instanceof Map) return new Map(factories);
  if (!isRecord(factories)) fail("INVALID_FACTORY_MAP", "drive content factories must be an object or Map");
  return new Map(Object.keys(factories).map((id) => [id, factories[id]]));
}

export function createDriveContentRegistryFromCatalog(catalog, factories) {
  const normalizedCatalog = validateDriveContentCatalog(catalog);
  const byId = factoryMap(factories);
  const catalogIds = new Set(normalizedCatalog.contents.map(({ id }) => id));

  for (const id of byId.keys()) {
    if (!catalogIds.has(id)) fail("UNKNOWN_FACTORY_ID", `factory has no content catalog entry: ${String(id)}`);
  }

  const registry = createDriveContentRegistry();
  for (const content of normalizedCatalog.contents) {
    if (!byId.has(content.id)) fail("MISSING_FACTORY", `content catalog id has no factory: ${content.id}`);
    const factory = byId.get(content.id);
    if (typeof factory !== "function") fail("INVALID_FACTORY", `factory must be a function: ${content.id}`);
    registry.register({ ...content, factory });
  }
  return registry;
}

export function installDriveContentRegistryFacade(registry, target = globalThis) {
  const activeRegistry = assertRegistry(registry);
  if (!target || (typeof target !== "object" && typeof target !== "function")) {
    fail("INVALID_FACADE_TARGET", "drive content registry facade target must be an object");
  }
  target.DriveContentRegistry = activeRegistry;
  return activeRegistry;
}
