import contentCatalogSource from "../features/drive/core/content_catalog.json";
import { installDriveContentFacade } from "../features/drive/core/content.js";
import {
  createDriveLayoutSpec,
  installDriveLayoutSpecFacade,
} from "../features/drive/core/layout_spec.js";
import { validateDriveContentCatalog } from "../features/drive/core/registry.js";
import { installDriveWorkspaceFacade } from "../features/drive/core/workspace.js";
import { installDriveWorkspaceRuntimeFacade } from "../features/drive/core/workspace_runtime.js";
import { createDriveContentPlatformRegistry } from "../features/drive/factory_catalog.js";
import { installDriveLiveStateProviderFacade } from "../features/drive/core/live_state_provider.js";
import { installDriveDataActivityFacade } from "../shared/activity/drive_data.js";
import {
  finalizeCarrotNaviModules,
  installCarrotNaviModuleGlobals,
} from "../features/drive/contents/carrot_navi/index.js";
import {
  finalizeDriveVisionFacade,
  installDriveVisionLeafFacades,
} from "../features/drive/contents/vision/index.js";

const target = globalThis;
const catalog = validateDriveContentCatalog(contentCatalogSource);

// Classic realtime consumers load after this bundle. Install the shared
// activity and authoritative state facades before any of them evaluate.
installDriveDataActivityFacade(target);
installDriveLiveStateProviderFacade(target);
installDriveContentFacade(target);
installDriveWorkspaceFacade(target);
const registry = createDriveContentPlatformRegistry(catalog, target);
const layoutSpec = createDriveLayoutSpec({
  registry,
  contentDefaults: catalog.defaults,
  settingDefaults: () => target.CarrotWebSettingDefaults || {},
});
installDriveLayoutSpecFacade(layoutSpec, target);

// Leaf renderers must exist before the remaining synchronous runtime scripts
// evaluate. Runtime/content factories are finalized once those scripts finish.
installDriveVisionLeafFacades(target);
installCarrotNaviModuleGlobals(target);

let finalized = false;
let workspaceRuntime = null;
const DRIVE_PLATFORM_READY_EVENT = "carrot:driveplatformready";

function finalizationDependenciesAvailable() {
  return Boolean(
    typeof target.HomeDrive?.refresh === "function"
    && target.DriveContentIntro?.create
    && target.document?.getElementById("carrotDriveWorkspace")
    && target.document?.getElementById("carrotStage")
    && target.document?.getElementById("carrotNaviPane")
  );
}

function finalizeDrivePlatform() {
  if (finalized) return true;
  if (!finalizationDependenciesAvailable()) return false;
  try {
    const visionFacade = finalizeDriveVisionFacade(target);
    workspaceRuntime ||= installDriveWorkspaceRuntimeFacade(target, {
      registry,
      layoutSpec,
      syncOnCreate: false,
    });
    const navigation = finalizeCarrotNaviModules(target, {
      compatibilityContent: false,
      runtime: { workspaceRuntime },
    });
    if (!visionFacade || !workspaceRuntime || !navigation.runtime || !navigation.facade) {
      throw new Error("Drive platform finalization dependencies are unavailable");
    }
    workspaceRuntime.sync();
    const primaryContent = workspaceRuntime.shell.getContent("primary");
    const secondaryContent = workspaceRuntime.shell.getContent("secondary");
    if (!primaryContent || !secondaryContent) {
      throw new Error("Drive workspace content assignments are unavailable");
    }
    finalized = true;
    return true;
  } catch (error) {
    target.console?.error?.("[drive] platform finalization failed", error);
    return false;
  }
}

target.addEventListener(DRIVE_PLATFORM_READY_EVENT, finalizeDrivePlatform, { once: true });
if (target.document?.readyState === "loading") {
  target.document.addEventListener("DOMContentLoaded", () => {
    if (!finalizeDrivePlatform() && !finalizationDependenciesAvailable()) {
      target.console?.error?.(
        "[drive] platform finalization failed",
        new Error("Drive platform finalization dependencies are unavailable"),
      );
    }
  }, { once: true });
} else {
  finalizeDrivePlatform();
}
