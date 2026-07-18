"use strict";

import {
  finalizeDriveVisionFacade,
  getDriveVisionFacade,
} from "./facade.js";
import {
  getDriveVisionHudContent,
  installDriveVisionHudContentFacade,
} from "./hud_content.js";

export const VISION_CONTENT_ID = "vision";
export const VISION_COMPATIBILITY_ROOT_ID = "carrotStage";
export const VISION_WORKSPACE_ROOT_ID = "carrotDriveWorkspace";

let contentSingleton = null;
let compatibilityRegistration = null;
const contentMetadata = new WeakMap();

export let DriveContentVision = null;

function resolveTarget(options) {
  return options.target || globalThis;
}

function resolveDocument(target, options) {
  return options.documentRoot || target?.document || null;
}

function resolveCompatibilityRoot(target, options) {
  if (options.compatibilityRoot) return options.compatibilityRoot;
  const documentRoot = resolveDocument(target, options);
  return documentRoot?.getElementById?.(VISION_COMPATIBILITY_ROOT_ID) || null;
}

function resolveHostRoot(target, options, compatibilityRoot) {
  return options.hostRoot
    || options.host?.root
    || options.context?.hostRoot
    || compatibilityRoot
    || null;
}

function resolveWorkspace(target, options) {
  if (options.workspace) return options.workspace;
  const documentRoot = resolveDocument(target, options);
  const workspaceRoot = options.workspaceRoot
    || documentRoot?.getElementById?.(VISION_WORKSPACE_ROOT_ID)
    || null;
  return target?.DriveWorkspace?.get?.(workspaceRoot) || null;
}

function resolveSlot(target, options) {
  return options.slot
    || options.context?.slot
    || target?.DriveWorkspace?.SLOT?.PRIMARY
    || "primary";
}

export function createDriveVisionContent(options = {}) {
  const target = resolveTarget(options);
  const compatibilityRoot = resolveCompatibilityRoot(target, options);
  const hostRoot = resolveHostRoot(target, options, compatibilityRoot);
  const driveContentApi = options.driveContentApi || target?.DriveContent;
  const runtime = options.runtime || target?.CarrotVisionContentRuntime;
  const renderer = options.renderer || getDriveVisionFacade() || target?.DriveVisionFacade;
  const hud = options.hud || getDriveVisionHudContent(target) || target?.DriveVisionHudContent;

  if (
    !hostRoot
    || typeof driveContentApi?.create !== "function"
    || typeof runtime?.setActive !== "function"
    || typeof renderer?.lifecycle?.resize !== "function"
    || typeof hud?.activate !== "function"
  ) return null;

  let mountedHostRoot = null;
  let hostContext = Object.freeze({
    slot: String(options.context?.slot || options.slot || "primary"),
    source: String(options.context?.source || "live"),
    compatibilityRoot,
  });

  function updateHostContext(context = {}) {
    hostContext = Object.freeze({
      slot: String(context.slot || context.host?.slot || hostContext.slot),
      source: String(context.source || context.host?.source || hostContext.source),
      compatibilityRoot,
    });
    return hostContext;
  }

  const content = driveContentApi.create(VISION_CONTENT_ID, {
    mount(nextRoot) {
      if (!nextRoot || typeof nextRoot !== "object") {
        throw new TypeError("Vision content requires a host root");
      }
      if (compatibilityRoot && nextRoot !== compatibilityRoot && compatibilityRoot.parentElement !== nextRoot) {
        if (typeof nextRoot.appendChild !== "function") {
          throw new TypeError("Vision content host must support appendChild");
        }
        nextRoot.appendChild(compatibilityRoot);
      }
      mountedHostRoot = nextRoot;
    },
    activate(context = {}) {
      const nextHostContext = updateHostContext(context);
      hud.activate({ ...context, host: nextHostContext, renderRoot: compatibilityRoot });
      const changed = runtime.setActive(true, context);
      renderer.lifecycle.resize();
      return changed;
    },
    deactivate(deactivateOptions = {}) {
      hud.deactivate(deactivateOptions);
      return runtime.setActive(false, deactivateOptions);
    },
    resize(_rect) {
      if (!runtime.isActive?.()) return false;
      renderer.lifecycle.resize();
      hud.resize();
      return true;
    },
    status() {
      const state = target?.CarrotVisionState || {};
      return {
        runtime: runtime.status?.() || {},
        userActive: Boolean(state.active),
        available: Boolean(state.available),
        controlState: String(state.controlState || ""),
        replayActive: Boolean(target?.CarrotVisionReplay?.isActive?.()),
        hud: hud.status(),
        renderer: renderer.status(),
      };
    },
    destroy() {
      if (compatibilityRegistration?.content === content) {
        compatibilityRegistration.workspace.unregisterContent(
          compatibilityRegistration.slot,
          content,
        );
        compatibilityRegistration = null;
      }
      hud.destroy();
      runtime.setActive(false, { keepWarm: false, reason: "content destroyed" });
      mountedHostRoot = null;
    },
  });

  contentMetadata.set(content, {
    hostContext: () => hostContext,
    renderRoot: compatibilityRoot,
    mountedHostRoot: () => mountedHostRoot,
  });
  return content;
}

export function getDriveVisionContentMetadata(content = contentSingleton) {
  const metadata = content && contentMetadata.get(content);
  if (!metadata) return null;
  return Object.freeze({
    hostContext: metadata.hostContext(),
    renderRoot: metadata.renderRoot,
    mountedHostRoot: metadata.mountedHostRoot(),
  });
}

export function getDriveVisionContent() {
  return contentSingleton;
}

export function getOrCreateDriveVisionContent(options = {}) {
  if (!contentSingleton) {
    contentSingleton = createDriveVisionContent(options);
    if (!contentSingleton) return null;
    DriveContentVision = contentSingleton;
  }
  return contentSingleton;
}

export function installDriveVisionContentFacade(target = globalThis, options = {}) {
  const content = options.content || getOrCreateDriveVisionContent({ ...options, target });
  if (!content) return null;
  target.DriveContentVision = content;
  return content;
}

export function finalizeDriveVisionContent(target = globalThis, options = {}) {
  const compatibilityWorkspace = options.compatibilityRegister === true
    ? resolveWorkspace(target, options)
    : null;
  const compatibilitySlot = options.compatibilityRegister === true
    ? resolveSlot(target, options)
    : null;
  if (
    options.compatibilityRegister === true
    && (!compatibilityWorkspace?.registerContent || !compatibilityWorkspace?.unregisterContent)
  ) return null;

  const renderer = options.renderer
    || getDriveVisionFacade()
    || finalizeDriveVisionFacade(target, options.facadeOptions || {});
  const hud = options.hud
    || getDriveVisionHudContent(target)
    || installDriveVisionHudContentFacade(target, options.hudOptions || {});
  if (!renderer || !hud) return null;

  const compatibilityRoot = resolveCompatibilityRoot(target, options);
  const hostRoot = resolveHostRoot(target, options, compatibilityRoot);
  const content = installDriveVisionContentFacade(target, {
    ...options,
    target,
    renderer,
    hud,
    compatibilityRoot,
    hostRoot,
  });
  if (!content || !hostRoot) return null;

  if (options.compatibilityRegister === true) {
    const occupyingContent = compatibilityWorkspace.getContent?.(compatibilitySlot) || null;
    if (occupyingContent && occupyingContent !== content) return null;
    if (compatibilityRegistration && (
      compatibilityRegistration.workspace !== compatibilityWorkspace
      || compatibilityRegistration.slot !== compatibilitySlot
      || compatibilityRegistration.content !== content
    )) {
      throw new Error("Vision compatibility registration is already fixed");
    }
  }

  const statusBefore = content.status?.() || {};
  let activatedHere = false;
  try {
    content.mount(hostRoot);
    if (options.activate !== false) {
      content.activate(options.activationContext || {});
      activatedHere = !statusBefore.active;
    }
    if (options.resize !== false) content.resize(options.rect);

    if (options.compatibilityRegister === true && !compatibilityRegistration) {
      compatibilityWorkspace.registerContent(compatibilitySlot, content);
      compatibilityRegistration = {
        workspace: compatibilityWorkspace,
        slot: compatibilitySlot,
        content,
      };
    }
  } catch (error) {
    if (activatedHere && content.status?.().active) {
      try {
        content.deactivate({ keepWarm: false, reason: "Vision compatibility registration failed" });
      } catch {}
    }
    throw error;
  }
  return content;
}

export function finalizeDriveVisionCompatibility(target = globalThis, options = {}) {
  return finalizeDriveVisionContent(target, {
    ...options,
    compatibilityRegister: true,
  });
}
