"use strict";

globalThis.DriveContentVision = (() => {
  const root = document.getElementById("carrotStage");
  const workspaceRoot = document.getElementById("carrotDriveWorkspace");
  const workspace = globalThis.DriveWorkspace?.get?.(workspaceRoot);
  const slot = globalThis.DriveWorkspace?.SLOT?.PRIMARY;
  const runtime = window.CarrotVisionContentRuntime;
  const renderer = window.DriveVisionFacade;
  const hud = window.DriveVisionHudContent;
  if (!root || !workspace || !slot || !globalThis.DriveContent?.create || !runtime?.setActive || !renderer?.lifecycle?.resize || !hud?.activate) return null;

  const content = globalThis.DriveContent.create("vision", {
    mount(nextRoot) {
      if (nextRoot !== root) throw new Error("Vision content must use #carrotStage");
    },
    activate(context = {}) {
      hud.activate(context);
      const changed = runtime.setActive(true, context);
      renderer.lifecycle.resize();
      return changed;
    },
    deactivate(options = {}) {
      hud.deactivate(options);
      return runtime.setActive(false, options);
    },
    resize() {
      if (!runtime.isActive?.()) return false;
      renderer.lifecycle.resize();
      hud.resize();
      return true;
    },
    status() {
      const state = window.CarrotVisionState || {};
      return {
        runtime: runtime.status?.() || {},
        userActive: Boolean(state.active),
        available: Boolean(state.available),
        controlState: String(state.controlState || ""),
        replayActive: Boolean(window.CarrotVisionReplay?.isActive?.()),
        hud: hud.status(),
        renderer: renderer.status(),
      };
    },
    destroy() {
      workspace.unregisterContent(slot, content);
      hud.destroy();
      runtime.setActive(false, { keepWarm: false, reason: "content destroyed" });
    },
  });

  content.mount(root);
  content.activate({ reason: "initial Vision content" });
  workspace.registerContent(slot, content);
  return content;
})();
