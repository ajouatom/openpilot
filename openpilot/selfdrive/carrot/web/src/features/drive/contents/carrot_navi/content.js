const contentSingletons = new WeakMap();

function resolveContentContext(context = {}) {
  const target = context.target || globalThis;
  const documentRoot = context.document || target.document;
  const compatibilityRoot = context.compatibilityRoot || documentRoot?.getElementById("carrotNaviPane") || null;
  const workspaceRoot = context.workspaceRoot || documentRoot?.getElementById("carrotDriveWorkspace") || null;
  const workspace = context.workspace
    || target.DriveWorkspace?.get?.(workspaceRoot)
    || null;
  const runtime = context.runtime || target.DriveCarrotNaviFacade?.content || null;
  const driveContentApi = context.driveContentApi || target.DriveContent;
  return {
    target,
    documentRoot,
    compatibilityRoot,
    workspace,
    slot: context.slot || null,
    runtime,
    driveContentApi,
    manageWorkspaceRegistration: context.manageWorkspaceRegistration === true,
  };
}

export function createCarrotNaviContent(context = {}) {
  const resolved = resolveContentContext(context);
  const {
    compatibilityRoot,
    workspace,
    slot,
    runtime,
    driveContentApi,
    manageWorkspaceRegistration,
  } = resolved;
  if (!compatibilityRoot || !driveContentApi?.create || !runtime?.activate) return null;

  let content;
  let currentHost = null;
  content = driveContentApi.create("carrot-navi", {
    mount(nextRoot) {
      if (!nextRoot || typeof nextRoot.appendChild !== "function") {
        throw new TypeError("Carrot Navi content mount requires a slot host");
      }
      currentHost = nextRoot;
      if (nextRoot !== compatibilityRoot && compatibilityRoot.parentNode !== nextRoot) {
        nextRoot.appendChild(compatibilityRoot);
      }
    },
    activate(activateContext = {}) {
      return runtime.activate(activateContext);
    },
    deactivate(options = {}) {
      return runtime.deactivate(options);
    },
    resize(rect) {
      return runtime.resize(rect);
    },
    status() {
      return {
        ...(runtime.status?.() || {}),
        contentHostId: String(currentHost?.id || ""),
      };
    },
    destroy() {
      if (manageWorkspaceRegistration) workspace?.unregisterContent?.(slot, content);
      runtime.deactivate({ keepWarm: false, reason: "content destroyed" });
      currentHost = null;
      if (contentSingletons.get(resolved.target) === content) contentSingletons.delete(resolved.target);
    },
  });
  return content;
}

export function getCarrotNaviContent(context = {}) {
  const target = context.target || globalThis;
  const existing = contentSingletons.get(target);
  if (existing) return existing;
  const content = createCarrotNaviContent(context);
  if (content) contentSingletons.set(target, content);
  return content;
}

export function registerCarrotNaviContent(content, context = {}) {
  if (!content) return false;
  const { workspace, slot } = resolveContentContext(context);
  if (!workspace || !slot) return false;
  return workspace.registerContent(slot, content);
}

export function installCarrotNaviContentGlobal(target = globalThis, content = null) {
  target.DriveContentCarrotNavi = content;
  return content;
}

export function finalizeCarrotNaviCompatibilityContent(target = globalThis, context = {}) {
  const documentRoot = context.document || target.document;
  const workspaceRoot = context.workspaceRoot || documentRoot?.getElementById("carrotDriveWorkspace") || null;
  const workspaceApi = context.workspaceApi || target.DriveWorkspace;
  const workspace = context.workspace || workspaceApi?.get?.(workspaceRoot);
  const slot = context.slot || workspaceApi?.SLOT?.SECONDARY;
  const compatibilityRoot = context.compatibilityRoot || documentRoot?.getElementById("carrotNaviPane") || null;
  const content = getCarrotNaviContent({
    ...context,
    target,
    document: documentRoot,
    compatibilityRoot,
    workspace,
    slot,
    manageWorkspaceRegistration: true,
  });
  if (!content) return installCarrotNaviContentGlobal(target, null);
  content.mount(context.root || compatibilityRoot);
  content.activate({ reason: context.reason || "initial TMAP content" });
  if (workspace?.getContent?.(slot) !== content) workspace?.registerContent?.(slot, content);
  return installCarrotNaviContentGlobal(target, content);
}

export function createCarrotNaviContentFactory(defaultContext = {}) {
  return function carrotNaviContentFactory(context = {}) {
    return getCarrotNaviContent({ ...defaultContext, ...context });
  };
}
