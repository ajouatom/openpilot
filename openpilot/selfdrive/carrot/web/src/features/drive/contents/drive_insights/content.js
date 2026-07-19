import { getOrCreateDriveInsightsRuntime } from "./runtime.js";

export const DRIVE_INSIGHTS_CONTENT_ID = "drive_insights";
export const DRIVE_INSIGHTS_SUPPORTED_SLOTS = Object.freeze(["primary", "secondary"]);
export const DRIVE_INSIGHTS_SUPPORTED_SOURCES = Object.freeze(["live", "replay"]);

const contentSingletons = new WeakMap();

function resolveTarget(options) {
  return options.target || globalThis;
}

export function createDriveInsightsContent(options = {}) {
  const target = resolveTarget(options);
  const driveContentApi = options.driveContentApi || target.DriveContent;
  const runtime = options.runtime || getOrCreateDriveInsightsRuntime({ ...options, target });
  if (typeof driveContentApi?.create !== "function" || !runtime?.root) return null;

  let mountedHost = null;
  let source = String(options.context?.source || options.source || "live");
  const content = driveContentApi.create(DRIVE_INSIGHTS_CONTENT_ID, {
    mount(host) {
      if (!host || typeof host.appendChild !== "function") {
        throw new TypeError("Drive Insights content requires a slot host");
      }
      if (runtime.root.parentNode !== host) host.appendChild(runtime.root);
      mountedHost = host;
    },
    activate(context = {}) {
      source = String(context.source || source || "live");
      return runtime.activate({ ...context, source });
    },
    deactivate(context = {}) {
      return runtime.deactivate(context);
    },
    resize(rect) {
      return runtime.resize(rect);
    },
    status() {
      return {
        source,
        mountedHostId: String(mountedHost?.id || ""),
        runtime: runtime.status(),
      };
    },
    destroy() {
      runtime.destroy();
      mountedHost = null;
      if (contentSingletons.get(target) === content) contentSingletons.delete(target);
    },
  });
  return content;
}

export function getOrCreateDriveInsightsContent(options = {}) {
  const target = resolveTarget(options);
  const existing = contentSingletons.get(target);
  if (existing) return existing;
  const content = createDriveInsightsContent({ ...options, target });
  if (content) contentSingletons.set(target, content);
  return content;
}

export function createDriveInsightsContentFactory(defaultOptions = {}) {
  return function driveInsightsContentFactory(context = {}) {
    const target = defaultOptions.target || globalThis;
    return getOrCreateDriveInsightsContent({
      ...defaultOptions,
      target,
      context,
      source: context.source,
    });
  };
}

export function createDriveInsightsContentDescriptor(options = {}) {
  return Object.freeze({
    id: DRIVE_INSIGHTS_CONTENT_ID,
    labelKey: "web_drive_layout_content_drive_insights",
    factory: createDriveInsightsContentFactory(options),
    supportedSlots: DRIVE_INSIGHTS_SUPPORTED_SLOTS,
    supportedSources: DRIVE_INSIGHTS_SUPPORTED_SOURCES,
    singleton: true,
  });
}

export function installDriveInsightsContentGlobal(target = globalThis, content = null) {
  target.DriveContentInsights = content;
  return content;
}
