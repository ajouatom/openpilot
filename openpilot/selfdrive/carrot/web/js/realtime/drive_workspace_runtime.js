"use strict";

// Page-level owner for the neutral Drive Workspace shell. Feature modules only
// register content or observe its events; no individual content owns geometry.
globalThis.DriveWorkspaceRuntime = (() => {
  const root = document.getElementById("carrotDriveWorkspace");
  const primary = document.getElementById("carrotStage");
  const secondary = document.getElementById("carrotNaviPane");
  const divider = document.getElementById("carrotNaviDivider");
  const workspaceApi = globalThis.DriveWorkspace;
  const layoutSpec = globalThis.CarrotDriveLayoutSpec;
  if (!root || !primary || !secondary || !divider || !workspaceApi?.create || !layoutSpec) return null;

  function slotForContent(content) {
    return content === layoutSpec.CONTENT.NAVIGATION
      ? workspaceApi.SLOT.SECONDARY
      : workspaceApi.SLOT.PRIMARY;
  }

  function contentForSlot(slot) {
    return slot === workspaceApi.SLOT.SECONDARY
      ? layoutSpec.CONTENT.NAVIGATION
      : layoutSpec.CONTENT.VISION;
  }

  function viewportOrientation() {
    const shared = globalThis.CarrotLayout?.orientation?.();
    if (shared === "portrait") return workspaceApi.ORIENTATION.VERTICAL;
    if (shared === "landscape") return workspaceApi.ORIENTATION.HORIZONTAL;
    const viewport = globalThis.visualViewport;
    const width = Number(viewport?.width || document.documentElement.clientWidth || globalThis.innerWidth || 0);
    const height = Number(viewport?.height || document.documentElement.clientHeight || globalThis.innerHeight || 0);
    return height >= width ? workspaceApi.ORIENTATION.VERTICAL : workspaceApi.ORIENTATION.HORIZONTAL;
  }

  function readLayout(orientation = viewportOrientation()) {
    return layoutSpec.read(globalThis.CarrotWebSettingsState, orientation);
  }

  const horizontal = readLayout(layoutSpec.ORIENTATION.HORIZONTAL);
  const vertical = readLayout(layoutSpec.ORIENTATION.VERTICAL);
  const initial = viewportOrientation() === workspaceApi.ORIENTATION.VERTICAL ? vertical : horizontal;
  const shell = workspaceApi.create({
    root,
    primary,
    secondary,
    divider,
    orientation: initial.orientation,
    layout: {
      mode: initial.mode,
      area1Slot: slotForContent(initial.area1Content),
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
    },
    persist(ratio, orientation) {
      const key = layoutSpec.keysFor(orientation).ratio;
      if (typeof globalThis.setWebSettingByKey === "function") {
        globalThis.setWebSettingByKey(key, ratio.toFixed(2)).catch(() => {});
      }
    },
  });
  if (!shell) return null;

  function applyLayout(layout = readLayout()) {
    if (!layout) return false;
    const orientationChanged = shell.setOrientation(layout.orientation);
    const ratioChanged = shell.setRatio(layout.ratio, { orientation: layout.orientation });
    const layoutChanged = shell.setLayout({
      mode: layout.mode,
      area1Slot: slotForContent(layout.area1Content),
    });
    return orientationChanged || ratioChanged || layoutChanged;
  }

  function presentationActive() {
    return document.body?.dataset?.page === "carrot"
      && !document.hidden
      && !primary.classList.contains("is-replay");
  }

  function sync() {
    applyLayout(readLayout());
    shell.setActive(presentationActive());
    shell.applyGeometry();
    return shell.snapshot();
  }

  for (const eventName of [
    "resize",
    "orientationchange",
    "carrot:viewportlayout",
    "carrot:pagechange",
  ]) globalThis.addEventListener(eventName, sync, { passive: true });
  globalThis.addEventListener("carrot:websettingschange", (event) => {
    const key = event?.detail?.key;
    const keys = Array.isArray(event?.detail?.keys) ? event.detail.keys : [key];
    if (keys.some((entry) => layoutSpec.settingKeys.includes(entry))) sync();
  });
  document.addEventListener("visibilitychange", sync, { passive: true });
  if (globalThis.visualViewport) {
    globalThis.visualViewport.addEventListener("resize", sync, { passive: true });
  }
  if (typeof globalThis.MutationObserver === "function") {
    new globalThis.MutationObserver(sync).observe(primary, {
      attributes: true,
      attributeFilter: ["class"],
    });
  }

  sync();

  return Object.freeze({
    root,
    shell,
    viewportOrientation,
    readLayout,
    applyLayout,
    presentationActive,
    sync,
    slotForContent,
    contentForSlot,
  });
})();
