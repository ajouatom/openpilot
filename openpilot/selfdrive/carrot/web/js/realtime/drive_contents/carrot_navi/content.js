"use strict";

globalThis.DriveContentCarrotNavi = (() => {
  const root = document.getElementById("carrotNaviPane");
  const workspaceRoot = document.getElementById("carrotDriveWorkspace");
  const workspace = globalThis.DriveWorkspace?.get?.(workspaceRoot);
  const slot = globalThis.DriveWorkspace?.SLOT?.SECONDARY;
  const runtime = globalThis.DriveCarrotNaviFacade?.content;
  if (!root || !workspace || !slot || !globalThis.DriveContent?.create || !runtime?.activate) return null;

  const content = globalThis.DriveContent.create("carrot-navi", {
    mount(nextRoot) {
      if (nextRoot !== root) throw new Error("Carrot Navi content must use #carrotNaviPane");
    },
    activate(context = {}) {
      return runtime.activate(context);
    },
    deactivate(options = {}) {
      return runtime.deactivate(options);
    },
    resize(rect) {
      return runtime.resize(rect);
    },
    status() {
      return runtime.status?.() || {};
    },
    destroy() {
      workspace.unregisterContent(slot, content);
      runtime.deactivate({ keepWarm: false, reason: "content destroyed" });
    },
  });

  content.mount(root);
  content.activate({ reason: "initial TMAP content" });
  workspace.registerContent(slot, content);
  return content;
})();
