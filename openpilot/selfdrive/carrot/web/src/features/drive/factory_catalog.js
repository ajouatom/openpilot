import {
  createDriveContentRegistryFromCatalog,
  installDriveContentRegistryFacade,
} from "./core/registry.js";
import {
  createVisionContentDescriptor,
} from "./contents/vision/registry_adapter.js";
import {
  VISION_CONTENT_ID,
  installDriveVisionContentFacade,
} from "./contents/vision/content.js";
import {
  createCarrotNaviContentFactory,
  installCarrotNaviContentGlobal,
} from "./contents/carrot_navi/content.js";
import {
  DRIVE_INSIGHTS_CONTENT_ID,
  createDriveInsightsContentFactory,
  installDriveInsightsContentGlobal,
} from "./contents/drive_insights/content.js";
import { installDriveInsightsRuntimeFacade } from "./contents/drive_insights/runtime.js";
import { TEMPORARY_TELEMETRY_SURFACES } from "../telemetry/temporary_surface_flags.js";

export const NAVIGATION_CONTENT_ID = "navigation";
export const DRIVE_CONTENT_FACTORY_IDS = Object.freeze([
  VISION_CONTENT_ID,
  NAVIGATION_CONTENT_ID,
  DRIVE_INSIGHTS_CONTENT_ID,
]);

function createTemporarilyDisabledDriveInsights(target) {
  const driveContentApi = target.DriveContent;
  if (typeof driveContentApi?.create !== "function") return null;
  return driveContentApi.create(DRIVE_INSIGHTS_CONTENT_ID, {
    mount() {},
    activate() { return true; },
    deactivate() { return true; },
    resize() { return false; },
    status() { return { temporarilyDisabled: true }; },
    destroy() {},
  });
}

export function createDriveContentFactories(target = globalThis, options = {}) {
  const vision = createVisionContentDescriptor({
    ...(options.vision || {}),
    target,
  });
  const navigation = createCarrotNaviContentFactory({
    ...(options.navigation || {}),
    target,
  });
  const driveInsightsOptions = { ...(options.driveInsights || {}), target };
  const driveInsights = createDriveInsightsContentFactory(driveInsightsOptions);
  return Object.freeze({
    [VISION_CONTENT_ID](context = {}) {
      const content = vision.factory(context);
      return content ? installDriveVisionContentFacade(target, { content }) : null;
    },
    [NAVIGATION_CONTENT_ID](context = {}) {
      const content = navigation(context);
      return content ? installCarrotNaviContentGlobal(target, content) : null;
    },
    [DRIVE_INSIGHTS_CONTENT_ID](context = {}) {
      if (!TEMPORARY_TELEMETRY_SURFACES.driveInsights) {
        return installDriveInsightsContentGlobal(
          target,
          createTemporarilyDisabledDriveInsights(target),
        );
      }
      const runtime = driveInsightsOptions.runtime
        || installDriveInsightsRuntimeFacade(target, driveInsightsOptions);
      if (!runtime) return null;
      if (driveInsightsOptions.runtime) target.DriveInsightsRuntime = runtime;
      const content = driveInsights(context);
      return content ? installDriveInsightsContentGlobal(target, content) : null;
    },
  });
}

export function createDriveContentPlatformRegistry(catalog, target = globalThis, options = {}) {
  const factories = createDriveContentFactories(target, options);
  const registry = createDriveContentRegistryFromCatalog(catalog, factories);
  installDriveContentRegistryFacade(registry, target);
  return registry;
}
