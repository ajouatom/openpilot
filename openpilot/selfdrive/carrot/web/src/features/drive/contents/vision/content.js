"use strict";

import {
  finalizeDriveVisionFacade,
  getDriveVisionFacade,
} from "./facade.js";
import {
  getDriveVisionHudContent,
  installDriveVisionHudContentFacade,
} from "./hud_content.js";
import {
  createVisionArActivationGate,
  installArRuntimeFacade,
} from "./ar/index.js";
import { createArDebugOverlay } from "./ar/debug/index.js";

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

  // Diagnostic: skip the world-pose projection (steps 11/12) and render from
  // the stored route-relative anchor — the pre-worldPose path. If signs reappear
  // with this on, the fault is world-pose projection, not anchor creation.
  //
  // The replay view is an in-app screen, so editing the browser URL would drop
  // the selected route. Instead this is a live flag that can be toggled from the
  // devtools console mid-replay, no reload needed:
  //   carrotArBypassWorld(true)   // signs render from route-relative anchor
  //   carrotArBypassWorld(false)  // back to world-pose projection
  //   carrotArBypassWorld()       // read current state
  // Initial value still honours ?ar_bypass_world=1 and a persisted localStorage
  // flag, so a reload keeps whatever was set.
  const bypassInitial = (() => {
    try {
      if (new URLSearchParams(target?.location?.search || "").get("ar_bypass_world") === "1") return true;
      return target?.localStorage?.getItem?.("ar_bypass_world") === "1";
    } catch {
      return false;
    }
  })();
  if (typeof target.__carrotArBypassWorld !== "boolean") {
    target.__carrotArBypassWorld = bypassInitial;
  }
  if (typeof target.carrotArBypassWorld !== "function") {
    target.carrotArBypassWorld = (next) => {
      if (next === undefined) return target.__carrotArBypassWorld === true;
      target.__carrotArBypassWorld = next === true;
      try { target.localStorage?.setItem?.("ar_bypass_world", next === true ? "1" : "0"); } catch {}
      return target.__carrotArBypassWorld;
    };
  }
  const bypassWorldAnchor = () => target.__carrotArBypassWorld === true;

  const arGate = createVisionArActivationGate({
    target,
    createRuntime: () => installArRuntimeFacade(target, {
      document: target?.document,
      host: compatibilityRoot,
      /* 합성 표지는 createArRuntime()을 직접 만드는 진단 harness에서만
       * diagnosticProbe=true로 허용한다. 제품/리플레이 entry는 항상 차단한다. */
      diagnosticProbe: false,
      bypassWorldAnchor,
      probeDistanceM: 40,
    }),
  });

  /* AR 진단 오버레이. 웹 설정 "AR 디버그"(vision_ar_debug)가 켜져 있을 때만
   * 스테이지 안에 패널을 만들고, 꺼져 있으면 DOM도 타이머도 남기지 않는다. */
  const debugOverlay = createArDebugOverlay({
    target,
    document: target?.document,
    mountRoot: compatibilityRoot || hostRoot,
  });
  if (typeof target.CarrotArLog !== "object" || target.CarrotArLog === null) {
    // 버튼 복사가 막히는 환경(비보안 컨텍스트 등)을 위한 콘솔 경로.
    target.CarrotArLog = Object.freeze({
      text: () => debugOverlay.logText(),
      json: () => debugOverlay.json(),
      snapshot: () => debugOverlay.snapshot(),
      capture: (settings) => debugOverlay.captureStart(settings),
      stopCapture: () => debugOverlay.captureStop(),
      captureStatus: () => debugOverlay.captureStatus(),
    });
  }

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
      arGate.activateVision();
      debugOverlay.activate();
      return changed;
    },
    deactivate(deactivateOptions = {}) {
      debugOverlay.deactivate();
      arGate.deactivateVision();
      hud.deactivate(deactivateOptions);
      return runtime.setActive(false, deactivateOptions);
    },
    resize(_rect) {
      if (!runtime.isActive?.()) return false;
      renderer.lifecycle.resize();
      hud.resize();
      arGate.resize();
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
        ar: arGate.status(),
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
      debugOverlay.destroy();
      arGate.destroy();
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
