"use strict";

globalThis.CarrotNaviSplit = (() => {
  const workspaceRuntime = globalThis.DriveWorkspaceRuntime;

  function create(options = {}) {
    const workspace = options.workspace;
    const overlay = options.overlay;
    const shell = workspaceRuntime?.shell;
    if (!workspace || !shell || workspaceRuntime.root !== workspace) return null;

    function navigationDetail(state = shell.snapshot()) {
      return {
        active: state.active,
        ratio: state.ratio,
        orientation: state.orientation,
        mode: state.mode,
        area1Content: workspaceRuntime.contentForSlot(state.area1Slot),
        compactStage: state.compactPrimary,
      };
    }

    globalThis.addEventListener("drive:workspacelayoutchange", (event) => {
      if (event?.detail?.workspaceId !== String(workspace.id || "")) return;
      const detail = navigationDetail(event.detail);
      globalThis.dispatchEvent(new CustomEvent("carrot:navigationlayoutchange", { detail }));
      options.onLayout?.(detail);
    });
    globalThis.addEventListener("drive:workspaceresizestart", (event) => {
      if (event?.detail?.workspaceId !== String(workspace.id || "")) return;
      overlay?.setRenderSuspended?.(true);
      globalThis.dispatchEvent(new CustomEvent("carrot:navigationresizestart"));
    });
    globalThis.addEventListener("drive:workspaceresizeend", (event) => {
      if (event?.detail?.workspaceId !== String(workspace.id || "")) return;
      overlay?.setRenderSuspended?.(false);
      globalThis.dispatchEvent(new CustomEvent("carrot:navigationresizeend"));
    });

    return Object.freeze({
      applyGeometry: shell.applyGeometry,
      snapshot() {
        const state = shell.snapshot();
        return { ...navigationDetail(state), ratios: state.ratios };
      },
    });
  }

  return Object.freeze({ create });
})();
