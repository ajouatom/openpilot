export function createSplit(options = {}) {
  const target = options.target || globalThis;
  const workspaceRuntime = options.workspaceRuntime || target.DriveWorkspaceRuntime;
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

  target.addEventListener("drive:workspacelayoutchange", (event) => {
    if (event?.detail?.workspaceId !== String(workspace.id || "")) return;
    const detail = navigationDetail(event.detail);
    target.dispatchEvent(new target.CustomEvent("carrot:navigationlayoutchange", { detail }));
    options.onLayout?.(detail);
  });
  target.addEventListener("drive:workspaceresizestart", (event) => {
    if (event?.detail?.workspaceId !== String(workspace.id || "")) return;
    overlay?.setRenderSuspended?.(true);
    target.dispatchEvent(new target.CustomEvent("carrot:navigationresizestart"));
  });
  target.addEventListener("drive:workspaceresizeend", (event) => {
    if (event?.detail?.workspaceId !== String(workspace.id || "")) return;
    overlay?.setRenderSuspended?.(false);
    target.dispatchEvent(new target.CustomEvent("carrot:navigationresizeend"));
  });

  return Object.freeze({
    applyGeometry: shell.applyGeometry,
    snapshot() {
      const state = shell.snapshot();
      return { ...navigationDetail(state), ratios: state.ratios };
    },
  });
}

export const CarrotNaviSplit = Object.freeze({ create: createSplit });

export function installCarrotNaviSplitGlobal(target = globalThis) {
  target.CarrotNaviSplit = CarrotNaviSplit;
  return CarrotNaviSplit;
}
