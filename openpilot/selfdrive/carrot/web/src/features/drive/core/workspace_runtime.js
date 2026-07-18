import {
  DriveWorkspace,
  DRIVE_WORKSPACE_LAYOUT_MODE,
  DRIVE_WORKSPACE_ORIENTATION,
  DRIVE_WORKSPACE_SLOT,
  createDriveWorkspace,
} from "./workspace.js";

export const DRIVE_WORKSPACE_DOM_IDS = Object.freeze({
  root: "carrotDriveWorkspace",
  primary: "carrotDrivePrimarySlot",
  secondary: "carrotDriveSecondarySlot",
  parking: "carrotDriveParking",
  divider: "carrotNaviDivider",
  visionRoot: "carrotStage",
  navigationRoot: "carrotNaviPane",
});

const runtimes = new WeakMap();

function ensureDirectChild(root, element, before = null) {
  if (element.parentElement === root || element.parentNode === root) return;
  root.insertBefore(element, before && (before.parentElement === root || before.parentNode === root) ? before : null);
}

function ensureSlotHost(documentRoot, root, id, slot, before) {
  let host = documentRoot.getElementById(id);
  if (!host) {
    host = documentRoot.createElement("div");
    host.id = id;
  }
  host.classList?.add("drive-workspace__slot");
  host.dataset.driveWorkspaceSlot = slot;
  ensureDirectChild(root, host, before);
  return host;
}

function ensureParkingHost(documentRoot, root) {
  let host = documentRoot.getElementById(DRIVE_WORKSPACE_DOM_IDS.parking);
  if (!host) {
    host = documentRoot.createElement("div");
    host.id = DRIVE_WORKSPACE_DOM_IDS.parking;
  }
  host.hidden = true;
  host.inert = true;
  host.dataset.driveWorkspaceParking = "";
  host.removeAttribute?.("aria-hidden");
  ensureDirectChild(root, host);
  return host;
}

export function ensureDriveWorkspaceHosts(documentRoot = document) {
  const root = documentRoot.getElementById(DRIVE_WORKSPACE_DOM_IDS.root);
  const visionRoot = documentRoot.getElementById(DRIVE_WORKSPACE_DOM_IDS.visionRoot);
  const navigationRoot = documentRoot.getElementById(DRIVE_WORKSPACE_DOM_IDS.navigationRoot);
  const divider = documentRoot.getElementById(DRIVE_WORKSPACE_DOM_IDS.divider);
  if (!root || !visionRoot || !navigationRoot || !divider) return null;

  // Compatibility roots are feature-owned. Keep them outside every visible
  // slot until the Workspace assigns an explicit DriveContent owner.
  const parking = ensureParkingHost(documentRoot, root);
  if (visionRoot.parentElement !== parking && visionRoot.parentNode !== parking) parking.appendChild(visionRoot);
  if (navigationRoot.parentElement !== parking && navigationRoot.parentNode !== parking) {
    parking.appendChild(navigationRoot);
  }

  const primary = ensureSlotHost(
    documentRoot,
    root,
    DRIVE_WORKSPACE_DOM_IDS.primary,
    DRIVE_WORKSPACE_SLOT.PRIMARY,
    divider,
  );

  const secondary = ensureSlotHost(
    documentRoot,
    root,
    DRIVE_WORKSPACE_DOM_IDS.secondary,
    DRIVE_WORKSPACE_SLOT.SECONDARY,
    parking,
  );

  // Geometry and accessibility belong to the neutral hosts. The legacy IDs
  // remain the stable feature roots for existing Vision/Navi consumers.
  delete visionRoot.dataset.driveWorkspaceSlot;
  delete navigationRoot.dataset.driveWorkspaceSlot;
  visionRoot.removeAttribute?.("aria-hidden");
  navigationRoot.removeAttribute?.("aria-hidden");

  return Object.freeze({ root, primary, secondary, parking, divider, visionRoot, navigationRoot });
}

function runtimeEnvironment(options) {
  const target = options.target || globalThis;
  const documentRoot = options.document || target.document;
  return {
    target,
    documentRoot,
    MutationObserver: options.MutationObserver || target.MutationObserver,
  };
}

export function createDriveWorkspaceRuntime(options = {}) {
  const { target, documentRoot, MutationObserver } = runtimeEnvironment(options);
  const elements = options.elements || ensureDriveWorkspaceHosts(documentRoot);
  const workspaceApi = options.workspaceApi || target.DriveWorkspace || DriveWorkspace;
  const registry = options.registry || target.DriveContentRegistry;
  const layoutSpec = options.layoutSpec || target.CarrotDriveLayoutSpec;
  if (!elements || !workspaceApi?.create || !registry?.create || !layoutSpec?.read) return null;

  const { root, primary, secondary, parking, divider, visionRoot, navigationRoot } = elements;
  const existing = runtimes.get(root);
  if (existing) return existing;
  const listeners = [];
  const observers = [];
  let destroyed = false;
  let replayPresentationActive = visionRoot.classList.contains("is-replay");
  let releaseSettingsGuard = null;
  let lastLayoutError = null;

  function listen(eventTarget, name, handler, listenerOptions) {
    if (typeof eventTarget?.addEventListener !== "function") return;
    eventTarget.addEventListener(name, handler, listenerOptions);
    listeners.push([eventTarget, name, handler, listenerOptions]);
  }

  function viewportOrientation() {
    if (typeof layoutSpec.orientationForViewport === "function") {
      return layoutSpec.orientationForViewport(target, documentRoot);
    }
    const shared = target.CarrotLayout?.orientation?.();
    if (shared === "portrait") return DRIVE_WORKSPACE_ORIENTATION.VERTICAL;
    if (shared === "landscape") return DRIVE_WORKSPACE_ORIENTATION.HORIZONTAL;
    const viewport = target.visualViewport;
    const width = Number(
      viewport?.width
      || documentRoot.documentElement?.clientWidth
      || target.innerWidth
      || 0,
    );
    const height = Number(
      viewport?.height
      || documentRoot.documentElement?.clientHeight
      || target.innerHeight
      || 0,
    );
    return height >= width ? DRIVE_WORKSPACE_ORIENTATION.VERTICAL : DRIVE_WORKSPACE_ORIENTATION.HORIZONTAL;
  }

  function readLayout(orientation = viewportOrientation(), settings = options.settings || target.CarrotWebSettingsState) {
    return layoutSpec.read(
      settings,
      orientation,
      { source: options.source || "live" },
    );
  }

  const horizontal = readLayout(DRIVE_WORKSPACE_ORIENTATION.HORIZONTAL);
  const vertical = readLayout(DRIVE_WORKSPACE_ORIENTATION.VERTICAL);
  const initial = viewportOrientation() === DRIVE_WORKSPACE_ORIENTATION.VERTICAL ? vertical : horizontal;
  const shell = (options.createWorkspace || createDriveWorkspace)({
    root,
    primary,
    secondary,
    parking,
    divider,
    registry,
    contentApi: options.contentApi || target.DriveContent,
    target,
    document: documentRoot,
    orientation: initial.orientation,
    layout: {
      mode: initial.mode,
      area1Slot: DRIVE_WORKSPACE_SLOT.PRIMARY,
    },
    geometry: {
      dividerWidth: 8,
      dividerHeight: 8,
      settleMs: 240,
      minRatio: 0.3,
      maxRatio: 0.7,
      compactPrimaryRatio: 0.5,
      compactPrimaryMinWidth: 360,
      compactPrimaryMinHeight: 300,
      ratioStep: 0.05,
      initialRatio: horizontal.ratio,
      verticalRatio: vertical.ratio,
      ...(options.geometry || {}),
    },
    persist(ratio, orientation) {
      const key = layoutSpec.keysFor(orientation).ratio;
      if (typeof target.setWebSettingByKey === "function") {
        target.setWebSettingByKey(key, ratio.toFixed(2)).catch(() => {});
      }
      options.persist?.(ratio, orientation, key);
    },
  });
  if (!shell) return null;

  function applyLayout(layout = readLayout()) {
    if (!layout || destroyed) return false;
    // Content creation/mount is the only fallible part of a normalized layout.
    // Complete it first so an invalid factory cannot leave geometry half-applied.
    const contentsChanged = shell.setContents({
      primaryId: layout.area1Content,
      secondaryId: layout.area2Content,
    }, { source: options.source || "live" });
    const orientationChanged = shell.setOrientation(layout.orientation);
    const ratioChanged = shell.setRatio(layout.ratio, { orientation: layout.orientation });
    const layoutChanged = shell.setLayout({
      mode: layout.mode,
      area1Slot: DRIVE_WORKSPACE_SLOT.PRIMARY,
    });
    shell.assertDomOwnership();
    lastLayoutError = null;
    root.dataset.driveWorkspaceLayoutState = "applied";
    return orientationChanged || ratioChanged || contentsChanged || layoutChanged;
  }

  function reportLayoutFailure(error, reason = "sync") {
    lastLayoutError = error;
    root.dataset.driveWorkspaceLayoutState = "error";
    target.console?.error?.(`[drive] workspace layout ${reason} failed`, error);
    if (typeof target.dispatchEvent === "function" && typeof target.CustomEvent === "function") {
      let requested = null;
      try {
        requested = readLayout();
      } catch (_) {}
      target.dispatchEvent(new target.CustomEvent("drive:workspacelayoutapplyerror", {
        detail: Object.freeze({
          reason,
          message: String(error?.message || error || "layout apply failed"),
          requested,
          actual: shell.snapshot(),
        }),
      }));
    }
  }

  function mountReplayVisionPresentation() {
    const content = target.DriveContentVision;
    if (!content) return false;
    shell.presentContent(content, root, {
      reason: "recorded replay presentation",
      slot: "standalone",
      source: "replay",
    });
    return true;
  }

  function restoreReplayVisionPresentation() {
    const content = target.DriveContentVision;
    return content ? shell.restorePresentedContent(content) : false;
  }

  function setReplayPresentation(value) {
    if (destroyed) return false;
    const next = Boolean(value);
    if (next === replayPresentationActive) {
      if (next) {
        root.classList?.add("is-drive-workspace-replay");
        shell.setSuspended?.(true);
        shell.setActive(false);
        if (!mountReplayVisionPresentation()) {
          throw new Error("Recorded replay presentation is unavailable");
        }
      }
      return true;
    }

    if (next) {
      replayPresentationActive = true;
      root.classList?.add("is-drive-workspace-replay");
      root.dataset.driveWorkspacePresentation = "replay";
      try {
        // Suspend first: no live slot can remain visible or active while Vision
        // moves from its configured slot into the standalone replay surface.
        shell.setSuspended?.(true);
        shell.setActive(false);
        if (!mountReplayVisionPresentation()) {
          throw new Error("Recorded replay presentation is unavailable");
        }
        shell.applyGeometry();
        return true;
      } catch (error) {
        replayPresentationActive = false;
        root.classList?.remove("is-drive-workspace-replay");
        delete root.dataset.driveWorkspacePresentation;
        try {
          restoreReplayVisionPresentation();
          shell.setSuspended?.(false);
          applyLayout(readLayout());
          shell.setActive(presentationActive());
        } catch (restoreError) {
          error.restoreError = restoreError;
        }
        throw error;
      }
    }

    // Restore while the live slots are still suspended. They become visible
    // only after Vision is back home and the requested live layout is applied.
    let restoreError = null;
    try {
      restoreReplayVisionPresentation();
    } catch (error) {
      restoreError = error;
    }
    replayPresentationActive = false;
    root.classList?.remove("is-drive-workspace-replay");
    delete root.dataset.driveWorkspacePresentation;
    try {
      applyLayout(readLayout());
    } catch (error) {
      reportLayoutFailure(error, "replay restore");
    }
    shell.setSuspended?.(false);
    shell.setActive(presentationActive());
    shell.applyGeometry();
    if (restoreError) reportLayoutFailure(restoreError, "replay release");
    return true;
  }

  function presentationActive() {
    return documentRoot.body?.dataset?.page === "carrot"
      && !documentRoot.hidden
      && !replayPresentationActive;
  }

  function sync(syncOptions = {}) {
    if (destroyed) return null;
    try {
      if (replayPresentationActive) {
        root.classList?.add("is-drive-workspace-replay");
        shell.setSuspended?.(true);
        shell.setActive(false);
        if (!mountReplayVisionPresentation()) {
          throw new Error("Recorded replay presentation is unavailable");
        }
      } else {
        restoreReplayVisionPresentation();
        applyLayout(readLayout());
        shell.setSuspended?.(false);
        shell.setActive(presentationActive());
      }
      shell.applyGeometry();
    } catch (error) {
      reportLayoutFailure(error, "sync");
      if (syncOptions.throwOnError === true) throw error;
    }
    return shell.snapshot();
  }

  function contentForSlot(slot) {
    return shell.contentIdForSlot(slot);
  }

  function slotForContent(contentId) {
    const id = String(contentId || "");
    for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
      if (contentForSlot(slot) === id) return slot;
    }
    return null;
  }

  function status() {
    return Object.freeze({
      replayPresentationActive,
      layoutState: String(root.dataset.driveWorkspaceLayoutState || "idle"),
      layoutError: lastLayoutError ? String(lastLayoutError?.message || lastLayoutError) : "",
      workspace: shell.snapshot(),
    });
  }

  function handleSettingsChange(event) {
    const key = event?.detail?.key;
    const keys = Array.isArray(event?.detail?.keys) ? event.detail.keys : [key];
    if (keys.some((entry) => layoutSpec.settingKeys.includes(entry))) sync();
  }

  function handleVisionPresentationChange() {
    // The stage also carries Workspace-owned compact/layout classes. Only replay
    // presentation changes whether the Workspace itself should be active.
    const nextReplayPresentationActive = visionRoot.classList.contains("is-replay")
      && Boolean(target.CarrotVisionReplay?.isActive?.());
    if (nextReplayPresentationActive === replayPresentationActive) return;
    try {
      setReplayPresentation(nextReplayPresentationActive);
    } catch (error) {
      reportLayoutFailure(error, "replay presentation");
    }
  }

  if (typeof target.registerWebSettingsGuard === "function") {
    releaseSettingsGuard = target.registerWebSettingsGuard((change) => {
      if (replayPresentationActive) return;
      if (!change.keys.some((key) => layoutSpec.settingKeys.includes(key))) return;
      applyLayout(readLayout(viewportOrientation(), change.settings));
    });
  }

  for (const eventName of [
    "resize",
    "orientationchange",
    "carrot:viewportlayout",
    "carrot:pagechange",
  ]) listen(target, eventName, sync, { passive: true });
  listen(target, "carrot:websettingschange", handleSettingsChange);
  listen(documentRoot, "visibilitychange", sync, { passive: true });
  listen(target.visualViewport, "resize", sync, { passive: true });
  if (typeof MutationObserver === "function") {
    const observer = new MutationObserver(handleVisionPresentationChange);
    observer.observe(visionRoot, { attributes: true, attributeFilter: ["class"] });
    observers.push(observer);
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    for (const [eventTarget, name, handler, listenerOptions] of listeners.splice(0)) {
      eventTarget.removeEventListener?.(name, handler, listenerOptions);
    }
    for (const observer of observers.splice(0)) observer.disconnect?.();
    releaseSettingsGuard?.();
    releaseSettingsGuard = null;
    shell.destroy();
    runtimes.delete(root);
    return true;
  }

  const api = Object.freeze({
    root,
    primary,
    secondary,
    parking,
    divider,
    compatibilityRoots: Object.freeze({ vision: visionRoot, navigation: navigationRoot }),
    shell,
    viewportOrientation,
    readLayout,
    applyLayout,
    setReplayPresentation,
    presentationActive,
    sync,
    status,
    slotForContent,
    contentForSlot,
    destroy,
  });

  runtimes.set(root, api);
  try {
    if (options.syncOnCreate !== false) sync({ throwOnError: true });
  } catch (error) {
    destroy();
    throw error;
  }
  return api;
}

export function installDriveWorkspaceRuntimeFacade(target = globalThis, options = {}) {
  const runtime = createDriveWorkspaceRuntime({ ...options, target });
  target.DriveWorkspaceRuntime = runtime;
  return runtime;
}

export function installDriveWorkspaceCoreFacades(target = globalThis) {
  target.DriveWorkspace = DriveWorkspace;
  return DriveWorkspace;
}

export {
  DRIVE_WORKSPACE_LAYOUT_MODE as LAYOUT_MODE,
  DRIVE_WORKSPACE_ORIENTATION as ORIENTATION,
  DRIVE_WORKSPACE_SLOT as SLOT,
};
