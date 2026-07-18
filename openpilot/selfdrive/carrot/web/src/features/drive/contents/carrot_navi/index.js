import { installCarrotNaviOverlayPrimitivesGlobal } from "./overlay_primitives.js";
import { installCarrotNaviOverlayGlobal } from "./overlay.js";
import { installCarrotNaviFooterStateGlobal } from "./footer_state.js";
import { installCarrotNaviOverlayStoreGlobal } from "./overlay_store.js";
import { installCarrotNaviLifecycleGlobal } from "./lifecycle.js";
import { installCarrotNaviCompositorGlobal } from "./compositor.js";
import { installCarrotNaviMseGlobal } from "./mse.js";
import { installCarrotNaviTransportGlobal } from "./transport.js";
import { installCarrotNaviHealthGlobal } from "./health.js";
import { installCarrotNaviSplitGlobal } from "./split.js";
import { installCarrotNaviRuntimeGlobal } from "./runtime.js";
import { installCarrotNaviFacadeGlobal } from "./facade.js";
import {
  finalizeCarrotNaviCompatibilityContent,
  installCarrotNaviContentGlobal,
} from "./content.js";

export function installCarrotNaviModuleGlobals(target = globalThis) {
  installCarrotNaviOverlayPrimitivesGlobal(target);
  installCarrotNaviOverlayGlobal(target);
  installCarrotNaviFooterStateGlobal(target);
  installCarrotNaviOverlayStoreGlobal(target);
  installCarrotNaviLifecycleGlobal(target);
  installCarrotNaviCompositorGlobal(target);
  installCarrotNaviMseGlobal(target);
  installCarrotNaviTransportGlobal(target);
  installCarrotNaviHealthGlobal(target);
  installCarrotNaviSplitGlobal(target);
  return target;
}

export function finalizeCarrotNaviModules(target = globalThis, options = {}) {
  installCarrotNaviModuleGlobals(target);
  const runtime = installCarrotNaviRuntimeGlobal(target, options.runtime || options);
  const facade = installCarrotNaviFacadeGlobal(target, runtime);
  let content = null;
  if (facade && options.compatibilityContent !== false) {
    content = finalizeCarrotNaviCompatibilityContent(target, {
      ...options.content,
      runtime: facade.content,
    });
  } else if (options.compatibilityContent !== false) {
    installCarrotNaviContentGlobal(target, null);
  }
  return Object.freeze({ runtime, facade, content });
}

export * from "./overlay_primitives.js";
export * from "./overlay.js";
export * from "./footer_state.js";
export * from "./overlay_store.js";
export * from "./lifecycle.js";
export * from "./compositor.js";
export * from "./mse.js";
export * from "./transport.js";
export * from "./health.js";
export * from "./split.js";
export * from "./runtime.js";
export * from "./facade.js";
export * from "./content.js";
