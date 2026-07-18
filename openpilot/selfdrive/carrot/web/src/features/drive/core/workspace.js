import { DriveContent, isDriveContent } from "./content.js";

export const DRIVE_WORKSPACE_SLOT = Object.freeze({
  PRIMARY: "primary",
  SECONDARY: "secondary",
});

export const DRIVE_WORKSPACE_ORIENTATION = Object.freeze({
  HORIZONTAL: "horizontal",
  VERTICAL: "vertical",
});

export const DRIVE_WORKSPACE_LAYOUT_MODE = Object.freeze({
  SPLIT: "split",
  AREA_1: "area_1",
  AREA_2: "area_2",
});

const DEFAULT_GEOMETRY = Object.freeze({
  dividerWidth: 8,
  dividerHeight: 8,
  settleMs: 240,
  minRatio: 0.3,
  maxRatio: 0.7,
  compactPrimaryRatio: 0.5,
  compactPrimaryMinWidth: 0,
  compactPrimaryMinHeight: 0,
  ratioStep: 0.05,
  initialRatio: 0.7,
  verticalRatio: null,
});

const CORE_CLASS_NAMES = Object.freeze({
  active: "is-drive-workspace-active",
  resizing: "is-drive-workspace-resizing",
  settling: "is-drive-workspace-settling",
  primarySplit: "is-drive-workspace-split",
  primaryCompact: "is-drive-workspace-compact-primary",
  documentCompact: "is-drive-workspace-compact-primary",
  horizontal: "is-drive-workspace-horizontal",
  vertical: "is-drive-workspace-vertical",
  swapped: "is-drive-workspace-swapped",
  area1Only: "is-drive-workspace-area-1-only",
  area2Only: "is-drive-workspace-area-2-only",
  suspended: "is-drive-workspace-suspended",
});

const workspaces = new WeakMap();

function workspaceEnvironment(options, root) {
  const target = options.target || globalThis;
  const documentRoot = options.document || root?.ownerDocument || target.document;
  const setTimer = options.setTimeout || target.setTimeout?.bind(target) || globalThis.setTimeout.bind(globalThis);
  const clearTimer = options.clearTimeout || target.clearTimeout?.bind(target) || globalThis.clearTimeout.bind(globalThis);
  const requestFrame = options.requestAnimationFrame
    || target.requestAnimationFrame?.bind(target)
    || ((callback) => setTimer(callback, 0));
  const cancelFrame = options.cancelAnimationFrame
    || target.cancelAnimationFrame?.bind(target)
    || clearTimer;
  return {
    target,
    documentRoot,
    setTimer,
    clearTimer,
    requestFrame,
    cancelFrame,
    ResizeObserver: options.ResizeObserver || target.ResizeObserver,
  };
}

function childNodesOf(element) {
  return Array.from(element?.childNodes || element?.children || []);
}

function restoreChildren(element, children) {
  if (!element) return;
  if (typeof element.replaceChildren === "function") {
    element.replaceChildren(...children);
    return;
  }
  while (element.firstChild) element.removeChild(element.firstChild);
  for (const child of children) element.appendChild(child);
}

function datasetValue(element, key) {
  return Object.prototype.hasOwnProperty.call(element?.dataset || {}, key)
    ? element.dataset[key]
    : undefined;
}

function restoreDatasetValue(element, key, value) {
  if (!element?.dataset) return;
  if (value === undefined) delete element.dataset[key];
  else element.dataset[key] = value;
}

export function createDriveWorkspace(options = {}) {
  const root = options.root;
  const primary = options.primary;
  const secondary = options.secondary;
  const divider = options.divider;
  if (!root || !primary || !secondary || !divider) return null;
  const existing = workspaces.get(root);
  if (existing) return existing;

  const env = workspaceEnvironment(options, root);
  const { target, documentRoot } = env;
  const parking = options.parking || documentRoot?.createElement?.("div") || null;
  if (!parking || typeof parking.appendChild !== "function") {
    throw new Error("DriveWorkspace requires a parking host");
  }
  parking.hidden = true;
  parking.inert = true;
  parking.dataset.driveWorkspaceParking = "";
  parking.removeAttribute?.("aria-hidden");
  if (parking.parentElement !== root && parking.parentNode !== root) root.appendChild(parking);
  const contentApi = options.contentApi || target.DriveContent || DriveContent;
  const registry = options.registry || target.DriveContentRegistry || null;
  const geometry = { ...DEFAULT_GEOMETRY, ...(options.geometry || {}) };
  const classNames = { ...CORE_CLASS_NAMES, ...(options.classNames || {}) };
  const styleProperties = {
    dividerWidth: "--drive-workspace-divider-width",
    secondaryWidth: "--drive-workspace-secondary-width",
    dividerHeight: "--drive-workspace-divider-height",
    secondaryHeight: "--drive-workspace-secondary-height",
    ...(options.styleProperties || {}),
  };
  const events = {
    layout: "drive:workspacelayoutchange",
    resizeStart: "drive:workspaceresizestart",
    resizeEnd: "drive:workspaceresizeend",
    content: "drive:workspacecontentchange",
    ...(options.events || {}),
  };

  root.classList?.add("drive-workspace");
  primary.dataset.driveWorkspaceSlot = DRIVE_WORKSPACE_SLOT.PRIMARY;
  secondary.dataset.driveWorkspaceSlot = DRIVE_WORKSPACE_SLOT.SECONDARY;
  divider.dataset.driveWorkspaceDivider = "";

  let destroyed = false;
  let active = false;
  let suspended = false;
  let orientation = normalizeOrientation(options.orientation);
  let layoutMode = normalizeLayoutMode(options.layout?.mode);
  let area1Slot = String(options.layout?.area1Slot || DRIVE_WORKSPACE_SLOT.PRIMARY);
  let ratios = null;
  let ratioFrameRequest = 0;
  let pendingRatio = null;
  let pointerId = null;
  let dragRect = null;
  let pointerOffset = 0;
  let settleTimer = 0;
  let appliedGeometrySignature = "";
  let contentResizeFrameRequest = 0;
  let activationResizeFrameRequest = 0;
  let forceContentResize = false;
  let assignmentSource = "live";
  const assignments = new Map([
    [DRIVE_WORKSPACE_SLOT.PRIMARY, null],
    [DRIVE_WORKSPACE_SLOT.SECONDARY, null],
  ]);
  const contentRectSignatures = new Map();
  const contentNodes = new WeakMap();
  const presentations = new Map();
  const listeners = [];
  const observers = [];

  area1Slot = normalizeSlotValue(area1Slot);

  function assertAlive(operation) {
    if (destroyed) throw new Error(`DriveWorkspace cannot ${operation} after destroy`);
  }

  function listen(eventTarget, type, handler, listenerOptions) {
    if (typeof eventTarget?.addEventListener !== "function") return;
    eventTarget.addEventListener(type, handler, listenerOptions);
    listeners.push([eventTarget, type, handler, listenerOptions]);
  }

  function dispatch(name, detail) {
    if (!name || typeof target.dispatchEvent !== "function" || typeof target.CustomEvent !== "function") return;
    target.dispatchEvent(new target.CustomEvent(name, { detail }));
  }

  function stateClassNames(key) {
    return [...new Set([CORE_CLASS_NAMES[key], classNames[key]].filter(Boolean))];
  }

  function toggleStateClass(element, key, enabled) {
    if (!element?.classList) return;
    const nextEnabled = Boolean(enabled);
    for (const name of stateClassNames(key)) {
      if (element.classList.contains(name) !== nextEnabled) {
        element.classList.toggle(name, nextEnabled);
      }
    }
  }

  function hasStateClass(element, key) {
    return Boolean(element?.classList && stateClassNames(key).some((name) => element.classList.contains(name)));
  }

  function removeStateClasses(element, ...keys) {
    if (!element?.classList) return;
    const names = keys.flatMap(stateClassNames);
    if (names.length) element.classList.remove(...names);
  }

  function syncContentSplitClass(enabled) {
    toggleStateClass(primary, "primarySplit", enabled);
    for (const host of [primary, secondary]) {
      for (const child of childNodesOf(host)) toggleStateClass(child, "primarySplit", enabled);
    }
  }

  function normalizeOrientation(value) {
    return String(value || "").toLowerCase() === DRIVE_WORKSPACE_ORIENTATION.VERTICAL
      ? DRIVE_WORKSPACE_ORIENTATION.VERTICAL
      : DRIVE_WORKSPACE_ORIENTATION.HORIZONTAL;
  }

  function normalizeLayoutMode(value) {
    const normalized = String(value || "").trim().toLowerCase();
    return Object.values(DRIVE_WORKSPACE_LAYOUT_MODE).includes(normalized)
      ? normalized
      : DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT;
  }

  function normalizeSlotValue(slot) {
    const normalized = String(slot || "").trim().toLowerCase();
    if (!assignments.has(normalized)) throw new RangeError(`Unknown DriveWorkspace slot: ${slot}`);
    return normalized;
  }

  function slotElement(slot) {
    return normalizeSlotValue(slot) === DRIVE_WORKSPACE_SLOT.PRIMARY ? primary : secondary;
  }

  function clamp(value) {
    const numeric = Number(value);
    const configuredFallback = Number(geometry.initialRatio);
    const fallback = Number.isFinite(configuredFallback) ? configuredFallback : DEFAULT_GEOMETRY.initialRatio;
    return Math.max(geometry.minRatio, Math.min(
      geometry.maxRatio,
      Number.isFinite(numeric) ? numeric : fallback,
    ));
  }

  function snap(value) {
    const clamped = clamp(value);
    const stepped = Math.round(clamped / geometry.ratioStep) * geometry.ratioStep;
    return Number(Math.max(geometry.minRatio, Math.min(geometry.maxRatio, stepped)).toFixed(2));
  }

  ratios = {
    [DRIVE_WORKSPACE_ORIENTATION.HORIZONTAL]: snap(geometry.initialRatio),
    [DRIVE_WORKSPACE_ORIENTATION.VERTICAL]: snap(
      geometry.verticalRatio != null && Number.isFinite(Number(geometry.verticalRatio))
        ? geometry.verticalRatio
        : geometry.initialRatio,
    ),
  };

  function effectiveRatio() {
    return ratios[orientation] ?? snap(geometry.initialRatio);
  }

  function slotVisible(slot) {
    const normalized = normalizeSlotValue(slot);
    if (suspended) return false;
    // Preserve the legacy compatibility surface while the page shell is not
    // presenting the workspace. Feature runtimes still gate expensive work on
    // page visibility/replay state.
    if (!active) return normalized === DRIVE_WORKSPACE_SLOT.PRIMARY;
    if (layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT) return true;
    const visibleSlot = layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.AREA_1
      ? area1Slot
      : (area1Slot === DRIVE_WORKSPACE_SLOT.PRIMARY
        ? DRIVE_WORKSPACE_SLOT.SECONDARY
        : DRIVE_WORKSPACE_SLOT.PRIMARY);
    return normalized === visibleSlot;
  }

  function syncDividerState() {
    const interactive = active && !suspended && layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT;
    divider.setAttribute?.("aria-disabled", String(!interactive));
    divider.setAttribute?.("aria-hidden", String(!interactive));
    divider.tabIndex = interactive ? 0 : -1;
  }

  function syncOrientationState() {
    const vertical = orientation === DRIVE_WORKSPACE_ORIENTATION.VERTICAL;
    toggleStateClass(root, "horizontal", !vertical);
    toggleStateClass(root, "vertical", vertical);
    root.dataset.driveWorkspaceOrientation = orientation;
    divider.setAttribute?.("aria-orientation", vertical ? "horizontal" : "vertical");
    syncDividerState();
  }

  function syncLayoutState() {
    const swapped = area1Slot === DRIVE_WORKSPACE_SLOT.SECONDARY;
    toggleStateClass(root, "swapped", swapped);
    toggleStateClass(root, "area1Only", layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.AREA_1);
    toggleStateClass(root, "area2Only", layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.AREA_2);
    root.dataset.driveWorkspaceMode = layoutMode;
    primary.dataset.driveWorkspaceArea = swapped
      ? DRIVE_WORKSPACE_LAYOUT_MODE.AREA_2
      : DRIVE_WORKSPACE_LAYOUT_MODE.AREA_1;
    secondary.dataset.driveWorkspaceArea = swapped
      ? DRIVE_WORKSPACE_LAYOUT_MODE.AREA_1
      : DRIVE_WORKSPACE_LAYOUT_MODE.AREA_2;
    syncDividerState();
  }

  function syncSlotPresentation() {
    for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
      const element = slotElement(slot);
      const visible = slotVisible(slot);
      element.hidden = !visible;
      element.inert = !visible;
      element.dataset.driveWorkspaceVisible = String(visible);
      if (visible) element.removeAttribute?.("aria-hidden");
      else element.setAttribute?.("aria-hidden", "true");
    }
    divider.hidden = !active || layoutMode !== DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT;
    syncDividerState();
  }

  function measureSlot(slot) {
    const rect = slotElement(slot).getBoundingClientRect?.() || {};
    return Object.freeze({
      x: Number(rect.x ?? rect.left ?? 0),
      y: Number(rect.y ?? rect.top ?? 0),
      top: Number(rect.top || 0),
      right: Number(rect.right || 0),
      bottom: Number(rect.bottom || 0),
      left: Number(rect.left || 0),
      width: Math.max(0, Number(rect.width || 0)),
      height: Math.max(0, Number(rect.height || 0)),
      devicePixelRatio: Math.max(1, Number(target.devicePixelRatio || 1)),
    });
  }

  function rectSignature(rect) {
    return [rect.left, rect.top, rect.width, rect.height, rect.devicePixelRatio]
      .map((value) => Number(value).toFixed(2))
      .join("|");
  }

  function resizeContent(slot, force = false) {
    const normalized = normalizeSlotValue(slot);
    const record = assignments.get(normalized);
    if (!record || !slotVisible(normalized)) return false;
    const rect = measureSlot(normalized);
    const signature = rectSignature(rect);
    if (!force && contentRectSignatures.get(normalized) === signature) return false;
    record.content.resize(rect);
    contentRectSignatures.set(normalized, signature);
    return true;
  }

  function resizeContents(resizeOptions = {}) {
    assertAlive("resize contents");
    if (pointerId !== null || hasStateClass(root, "resizing")) return false;
    const force = Boolean(resizeOptions.force);
    let changed = false;
    for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
      changed = resizeContent(slot, force) || changed;
    }
    return changed;
  }

  function scheduleContentResize(force = false) {
    if (destroyed) return;
    forceContentResize = forceContentResize || Boolean(force);
    if (contentResizeFrameRequest) return;
    contentResizeFrameRequest = env.requestFrame(() => {
      contentResizeFrameRequest = 0;
      const shouldForce = forceContentResize;
      forceContentResize = false;
      if (!destroyed) resizeContents({ force: shouldForce });
    });
  }

  function assignmentStatus(slot) {
    const record = assignments.get(normalizeSlotValue(slot));
    return record?.content?.status?.() || null;
  }

  function syncContentActivity(reason = "workspace layout") {
    assertAlive("sync content activity");
    let changed = false;
    for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
      const record = assignments.get(slot);
      if (!record) continue;
      const status = record.content.status?.() || {};
      const shouldBeActive = active && slotVisible(slot);
      if (shouldBeActive && !status.active) {
        record.content.activate({ reason, slot, source: record.source });
        changed = true;
      } else if (!shouldBeActive && status.active) {
        record.content.deactivate({ keepWarm: false, reason, slot, source: record.source });
        changed = true;
      }
    }
    return changed;
  }

  function contentChangeDetail(slot, action, record) {
    return {
      workspaceId: String(root.id || ""),
      slot,
      action,
      contentId: String(record?.id || ""),
      contentName: String(record?.content?.name || ""),
    };
  }

  function descriptorFor(slot, id, source) {
    const descriptor = registry?.get?.(id);
    if (!descriptor) throw new RangeError(`Unknown drive content id: ${String(id)}`);
    if (!descriptor.supportedSlots?.includes(slot)) {
      throw new RangeError(`Drive content ${id} does not support slot ${slot}`);
    }
    if (!descriptor.supportedSources?.includes(source)) {
      throw new RangeError(`Drive content ${id} does not support source ${source}`);
    }
    return descriptor;
  }

  function resolveAssignments(selection = {}, context = {}) {
    if (!registry?.get || !registry?.create) {
      throw new Error("DriveWorkspace setContents requires a DriveContent registry");
    }
    const source = String(context.source || "live");
    const ids = {
      [DRIVE_WORKSPACE_SLOT.PRIMARY]: String(selection.primaryId || ""),
      [DRIVE_WORKSPACE_SLOT.SECONDARY]: String(selection.secondaryId || ""),
    };
    const descriptors = {
      [DRIVE_WORKSPACE_SLOT.PRIMARY]: descriptorFor(DRIVE_WORKSPACE_SLOT.PRIMARY, ids.primary, source),
      [DRIVE_WORKSPACE_SLOT.SECONDARY]: descriptorFor(DRIVE_WORKSPACE_SLOT.SECONDARY, ids.secondary, source),
    };
    if (ids.primary === ids.secondary && descriptors.primary.singleton) {
      throw new Error(`Drive content ${ids.primary} is singleton and cannot occupy both slots`);
    }

    const records = new Map();
    const selectedIds = [];
    for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
      const id = ids[slot];
      const content = registry.create(id, {
        slot,
        source,
        fallbackId: id,
        selectedIds: [...selectedIds],
      });
      if (!(contentApi?.isContent?.(content) || isDriveContent(content))) {
        throw new TypeError(`Drive content factory ${id} must return a DriveContent instance`);
      }
      if ([...records.values()].some((record) => record.content === content)) {
        throw new Error(`Drive content instance ${id} cannot occupy both slots`);
      }
      records.set(slot, Object.freeze({ id, descriptor: descriptors[slot], content, source }));
      selectedIds.push(id);
    }
    return { records, source };
  }

  function setHostDataset(slot, record) {
    const host = slotElement(slot);
    if (!record) {
      delete host.dataset.driveContent;
      delete host.dataset.driveContentName;
      return;
    }
    host.dataset.driveContent = record.id;
    host.dataset.driveContentName = record.content.name;
  }

  function uniqueNodes(nodes) {
    return [...new Set((nodes || []).filter(Boolean))];
  }

  function rememberContentNodes(content, host, before = []) {
    const beforeSet = new Set(before);
    const previous = contentNodes.get(content) || [];
    const added = childNodesOf(host).filter((node) => !beforeSet.has(node));
    const owned = uniqueNodes([
      ...added,
      ...previous.filter((node) => node.parentNode === host || node.parentElement === host),
    ]);
    if (owned.length) contentNodes.set(content, owned);
    return owned;
  }

  function releaseFocusWithin(nodes) {
    const activeElement = documentRoot?.activeElement;
    if (!activeElement) return false;
    const ownsFocus = (nodes || []).some((node) => (
      node === activeElement || node.contains?.(activeElement)
    ));
    if (!ownsFocus) return false;
    activeElement.blur?.();
    return true;
  }

  function parkContent(content) {
    if (!content) return [];
    if (content.status?.().active) {
      content.deactivate({ keepWarm: false, reason: "workspace content parking" });
    }
    releaseFocusWithin(contentNodes.get(content) || []);
    const before = childNodesOf(parking);
    content.mount(parking);
    return rememberContentNodes(content, parking, before);
  }

  function moveChildrenToParking(host) {
    if (!host || host === parking) return [];
    const moved = childNodesOf(host);
    releaseFocusWithin(moved);
    for (const node of moved) parking.appendChild(node);
    return moved;
  }

  function mountAssignedContent(content, host) {
    if (childNodesOf(host).length) {
      throw new Error(`DriveWorkspace slot ${String(host.dataset?.driveWorkspaceSlot || "")} is not empty`);
    }
    const before = childNodesOf(host);
    content.mount(host);
    const owned = rememberContentNodes(content, host, before);
    if (!owned.length || childNodesOf(host).some((node) => !owned.includes(node))) {
      throw new Error(`Drive content ${String(content.name || "")} must own every top-level slot node`);
    }
    return owned;
  }

  function presentationFor(content) {
    return presentations.get(content) || null;
  }

  function domOwnershipStatus() {
    let consistent = true;
    const slots = {};
    for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
      const record = assignments.get(slot);
      const host = slotElement(slot);
      const actual = childNodesOf(host);
      const presented = Boolean(record && presentationFor(record.content));
      const expected = record && !presented ? (contentNodes.get(record.content) || []) : [];
      const orphanCount = actual.filter((node) => !expected.includes(node)).length;
      const missingCount = expected.filter((node) => node.parentNode !== host && node.parentElement !== host).length;
      const slotConsistent = orphanCount === 0
        && missingCount === 0
        && actual.length === expected.length
        && (!record || presented || expected.length > 0);
      consistent = consistent && slotConsistent;
      slots[slot] = Object.freeze({
        contentId: String(record?.id || ""),
        presented,
        rootCount: actual.length,
        orphanCount,
        missingCount,
        consistent: slotConsistent,
      });
    }
    for (const [content, presentation] of presentations) {
      const owned = contentNodes.get(content) || [];
      const presentationConsistent = owned.length > 0 && owned.every((node) => (
        node.parentNode === presentation.host || node.parentElement === presentation.host
      ));
      consistent = consistent && presentationConsistent;
    }
    return Object.freeze({ consistent, slots: Object.freeze(slots) });
  }

  function assertDomOwnership() {
    const status = domOwnershipStatus();
    if (!status.consistent) {
      throw new Error(`DriveWorkspace DOM ownership violation: ${JSON.stringify(status.slots)}`);
    }
    return status;
  }

  function sameAssignments(records, source) {
    if (assignmentSource !== source) return false;
    return Object.values(DRIVE_WORKSPACE_SLOT).every((slot) => (
      assignments.get(slot)?.id === records.get(slot)?.id
      && assignments.get(slot)?.content === records.get(slot)?.content
    ));
  }

  function setContents(selection = {}, context = {}) {
    assertAlive("set contents");
    if (presentations.size) {
      throw new Error("DriveWorkspace cannot change slot contents during standalone presentation");
    }
    const requestedSource = String(context.source || "live");
    const currentOwnership = domOwnershipStatus();
    if (assignmentSource === requestedSource
        && assignments.get(DRIVE_WORKSPACE_SLOT.PRIMARY)?.id === String(selection.primaryId || "")
        && assignments.get(DRIVE_WORKSPACE_SLOT.SECONDARY)?.id === String(selection.secondaryId || "")
        && currentOwnership.consistent) {
      return false;
    }

    // Descriptor, slot/source, singleton, factory and instance validation all
    // complete before the first existing content is deactivated or moved.
    const { records: nextAssignments, source } = resolveAssignments(selection, context);
    if (sameAssignments(nextAssignments, source) && currentOwnership.consistent) return false;

    const previousAssignments = new Map(assignments);
    const previousSource = assignmentSource;
    const previousActive = new Map();
    for (const record of previousAssignments.values()) {
      if (record) previousActive.set(record.content, Boolean(record.content.status?.().active));
    }
    const hostSnapshots = new Map();
    const contentNodeSnapshots = new Map();
    for (const record of [...previousAssignments.values(), ...nextAssignments.values()]) {
      if (record && !contentNodeSnapshots.has(record.content)) {
        contentNodeSnapshots.set(record.content, [...(contentNodes.get(record.content) || [])]);
      }
    }
    const parkingSnapshot = childNodesOf(parking);
    for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
      const host = slotElement(slot);
      hostSnapshots.set(slot, {
        children: childNodesOf(host),
        driveContent: datasetValue(host, "driveContent"),
        driveContentName: datasetValue(host, "driveContentName"),
      });
    }
    try {
      const unchangedSlots = new Set(Object.values(DRIVE_WORKSPACE_SLOT).filter((slot) => {
        const previous = previousAssignments.get(slot);
        const next = nextAssignments.get(slot);
        return previous?.id === next?.id
          && previous?.content === next?.content
          && source === previousSource
          && currentOwnership.slots[slot].consistent;
      }));
      const changingContents = new Set();
      for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
        if (unchangedSlots.has(slot)) continue;
        const previous = previousAssignments.get(slot);
        const next = nextAssignments.get(slot);
        if (previous) changingContents.add(previous.content);
        if (next) changingContents.add(next.content);
      }
      for (const content of changingContents) parkContent(content);
      for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
        if (!unchangedSlots.has(slot)) moveChildrenToParking(slotElement(slot));
      }
      for (const [slot, record] of nextAssignments) {
        if (!unchangedSlots.has(slot)) mountAssignedContent(record.content, slotElement(slot));
      }

      for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
        assignments.set(slot, nextAssignments.get(slot));
        contentRectSignatures.delete(slot);
        setHostDataset(slot, nextAssignments.get(slot));
      }
      assignmentSource = source;
      syncSlotPresentation();
      syncContentActivity("workspace content changed");
      syncContentSplitClass(active && !suspended && layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT);
      syncCompactPrimary();
      scheduleContentResize(true);
      assertDomOwnership();

      for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
        const previous = previousAssignments.get(slot);
        const next = nextAssignments.get(slot);
        if (previous?.id !== next?.id || previous?.content !== next?.content) {
          dispatch(events.content, contentChangeDetail(slot, previous ? "switch" : "register", next));
        }
      }
      return true;
    } catch (error) {
      let rollbackFailure = null;
      try {
        const previousContents = new Set(
          [...previousAssignments.values()].filter(Boolean).map((record) => record.content),
        );
        for (const record of nextAssignments.values()) {
          if (record.content.status?.().active && !previousActive.get(record.content)) {
            record.content.deactivate({ keepWarm: false, reason: "workspace content rollback" });
          }
          if (!previousContents.has(record.content)) {
            parkContent(record.content);
          }
        }
        for (const record of previousAssignments.values()) {
          if (record?.content?.status?.().active) {
            record.content.deactivate({ keepWarm: false, reason: "workspace content rollback move" });
          }
        }
        restoreChildren(parking, parkingSnapshot);
        for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
          restoreChildren(slotElement(slot), hostSnapshots.get(slot).children);
        }
        for (const [slot, record] of previousAssignments) {
          if (!record) continue;
          record.content.mount(slotElement(slot));
        }
        for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) {
          const snapshot = hostSnapshots.get(slot);
          restoreDatasetValue(slotElement(slot), "driveContent", snapshot.driveContent);
          restoreDatasetValue(slotElement(slot), "driveContentName", snapshot.driveContentName);
          assignments.set(slot, previousAssignments.get(slot));
        }
        for (const [content, nodes] of contentNodeSnapshots) contentNodes.set(content, nodes);
        assignmentSource = previousSource;
        contentRectSignatures.clear();
        syncSlotPresentation();
        syncContentSplitClass(active && !suspended && layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT);
        syncCompactPrimary();
        for (const [content, wasActive] of previousActive) {
          if (wasActive && !content.status?.().active) {
            content.activate({ reason: "workspace content rollback" });
          }
        }
      } catch (rollbackError) {
        rollbackFailure = rollbackError;
      }
      if (rollbackFailure) {
        error.rollbackError = rollbackFailure;
      }
      throw error;
    }
  }

  function registerContent(slot, content) {
    assertAlive("register content");
    const normalized = normalizeSlotValue(slot);
    if (!(contentApi?.isContent?.(content) || isDriveContent(content))) {
      throw new TypeError(`DriveWorkspace ${normalized} requires a DriveContent instance`);
    }
    const current = assignments.get(normalized);
    if (current?.content === content) return false;
    if (current) throw new Error(`DriveWorkspace ${normalized} already contains ${current.content.name}`);
    if (presentations.has(content)) {
      throw new Error(`Drive content ${content.name} is in standalone presentation`);
    }
    parkContent(content);
    moveChildrenToParking(slotElement(normalized));
    mountAssignedContent(content, slotElement(normalized));
    const record = Object.freeze({
      id: String(content.name || ""),
      descriptor: null,
      content,
      source: assignmentSource,
    });
    assignments.set(normalized, record);
    setHostDataset(normalized, record);
    syncSlotPresentation();
    syncContentActivity("workspace content registered");
    scheduleContentResize(true);
    assertDomOwnership();
    dispatch(events.content, contentChangeDetail(normalized, "register", record));
    return true;
  }

  function unregisterContent(slot, expectedContent = null) {
    assertAlive("unregister content");
    const normalized = normalizeSlotValue(slot);
    const current = assignments.get(normalized);
    if (!current || (expectedContent && current.content !== expectedContent)) return false;
    if (current.content.status?.().active) {
      current.content.deactivate({ keepWarm: false, reason: "workspace content unregistered", slot: normalized });
    }
    parkContent(current.content);
    moveChildrenToParking(slotElement(normalized));
    assignments.set(normalized, null);
    contentRectSignatures.delete(normalized);
    setHostDataset(normalized, null);
    syncSlotPresentation();
    assertDomOwnership();
    dispatch(events.content, contentChangeDetail(normalized, "unregister", current));
    return true;
  }

  function presentContent(content, host, context = {}) {
    assertAlive("present content");
    if (!(contentApi?.isContent?.(content) || isDriveContent(content))) {
      throw new TypeError("DriveWorkspace presentation requires a DriveContent instance");
    }
    if (!host || typeof host.appendChild !== "function") {
      throw new TypeError("DriveWorkspace presentation requires a host element");
    }
    if (!suspended) {
      throw new Error("DriveWorkspace must be suspended before standalone presentation");
    }
    const existingPresentation = presentations.get(content);
    if (existingPresentation?.host === host) {
      if (!content.status?.().active && context.activate !== false) content.activate(context);
      content.resize?.(host.getBoundingClientRect?.());
      return false;
    }
    if (existingPresentation) restorePresentedContent(content);
    const slot = Object.values(DRIVE_WORKSPACE_SLOT).find((candidate) => (
      assignments.get(candidate)?.content === content
    )) || null;
    if (content.status?.().active) {
      content.deactivate({ keepWarm: false, reason: "workspace standalone presentation", slot });
    }
    releaseFocusWithin(contentNodes.get(content) || []);
    const before = childNodesOf(host);
    content.mount(host);
    const owned = rememberContentNodes(content, host, before);
    if (!owned.length) {
      throw new Error(`Drive content ${content.name} did not mount a presentation root`);
    }
    presentations.set(content, Object.freeze({ host, slot }));
    assertDomOwnership();
    if (context.activate !== false) content.activate(context);
    content.resize?.(host.getBoundingClientRect?.());
    return true;
  }

  function restorePresentedContent(content) {
    assertAlive("restore presented content");
    const presentation = presentations.get(content);
    if (!presentation) return false;
    if (content.status?.().active) {
      content.deactivate({ keepWarm: false, reason: "workspace presentation restore" });
    }
    releaseFocusWithin(contentNodes.get(content) || []);
    presentations.delete(content);
    const assignedSlot = Object.values(DRIVE_WORKSPACE_SLOT).find((candidate) => (
      assignments.get(candidate)?.content === content
    )) || null;
    if (assignedSlot) {
      moveChildrenToParking(slotElement(assignedSlot));
      mountAssignedContent(content, slotElement(assignedSlot));
    } else {
      parkContent(content);
    }
    assertDomOwnership();
    return true;
  }

  function getContent(slot) {
    return assignments.get(normalizeSlotValue(slot))?.content || null;
  }

  function contentIdForSlot(slot) {
    return assignments.get(normalizeSlotValue(slot))?.id || "";
  }

  function contentSnapshot() {
    const status = {};
    for (const slot of Object.values(DRIVE_WORKSPACE_SLOT)) status[slot] = assignmentStatus(slot);
    return status;
  }

  function syncCompactPrimary(measuredExtent = 0) {
    const area1Ratio = effectiveRatio();
    const visionEntry = [...assignments.entries()].find(([, record]) => (
      record?.id === "vision" || record?.content?.name === "vision"
    ));
    const compactSlot = visionEntry?.[0] || DRIVE_WORKSPACE_SLOT.PRIMARY;
    const contentRatio = compactSlot === area1Slot ? area1Ratio : 1 - area1Ratio;
    const vertical = orientation === DRIVE_WORKSPACE_ORIENTATION.VERTICAL;
    const availableExtent = Math.max(0, measuredExtent || (vertical ? root.clientHeight : root.clientWidth));
    const dividerExtent = active && !suspended && layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT
      ? (vertical ? geometry.dividerHeight : geometry.dividerWidth)
      : 0;
    const primaryExtent = Math.max(0, availableExtent - dividerExtent) * contentRatio;
    const crossExtent = Math.max(0, vertical ? root.clientWidth : root.clientHeight);
    const primaryWidth = vertical ? crossExtent : primaryExtent;
    const primaryHeight = vertical ? primaryExtent : crossExtent;
    const belowReadableSize = primaryWidth < geometry.compactPrimaryMinWidth
      || primaryHeight < geometry.compactPrimaryMinHeight;
    const compactPrimary = active
      && !suspended
      && layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT
      && (contentRatio <= geometry.compactPrimaryRatio || belowReadableSize);
    const compactHost = slotElement(compactSlot);
    const primaryContentRoot = compactHost.firstElementChild || childNodesOf(compactHost)[0];
    for (const host of [primary, secondary]) {
      toggleStateClass(host, "primaryCompact", compactPrimary && host === compactHost);
      for (const child of childNodesOf(host)) {
        toggleStateClass(child, "primaryCompact", compactPrimary && child === primaryContentRoot);
      }
    }
    toggleStateClass(documentRoot?.documentElement, "documentCompact", compactPrimary);
    return compactPrimary;
  }

  function layoutDetail() {
    const domOwnership = domOwnershipStatus();
    return {
      workspaceId: String(root.id || ""),
      active,
      suspended,
      ratio: effectiveRatio(),
      ratios: { ...ratios },
      orientation,
      mode: layoutMode,
      area1Slot,
      visibleSlots: Object.values(DRIVE_WORKSPACE_SLOT).filter(slotVisible),
      contentIds: {
        primary: contentIdForSlot(DRIVE_WORKSPACE_SLOT.PRIMARY),
        secondary: contentIdForSlot(DRIVE_WORKSPACE_SLOT.SECONDARY),
      },
      domConsistent: domOwnership.consistent,
      domOwnership,
      compactPrimary: syncCompactPrimary(),
    };
  }

  function dispatchLayout() {
    const detail = layoutDetail();
    dispatch(events.layout, detail);
    options.onLayout?.(detail);
  }

  function persist(targetOrientation = orientation) {
    const normalized = normalizeOrientation(targetOrientation);
    options.persist?.(ratios[normalized], normalized);
  }

  function applyGeometry(renderContent = true, measuredExtent = 0) {
    assertAlive("apply geometry");
    const vertical = orientation === DRIVE_WORKSPACE_ORIENTATION.VERTICAL;
    const currentRatio = effectiveRatio();
    const splitActive = active && !suspended && layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT;
    const availableExtent = Math.max(0, measuredExtent || (vertical ? root.clientHeight : root.clientWidth));
    const compactPrimary = syncCompactPrimary(availableExtent);
    const configuredDividerExtent = vertical ? geometry.dividerHeight : geometry.dividerWidth;
    const dividerExtent = splitActive && availableExtent > configuredDividerExtent ? configuredDividerExtent : 0;
    const secondaryExtent = dividerExtent
      ? Math.max(0, (availableExtent - configuredDividerExtent) * (1 - currentRatio))
      : 0;
    const dividerWidth = vertical ? 0 : dividerExtent;
    const secondaryWidth = vertical ? 0 : secondaryExtent;
    const dividerHeight = vertical ? dividerExtent : 0;
    const secondaryHeight = vertical ? secondaryExtent : 0;
    const ratioPercent = Math.round(currentRatio * 100);
    const ratioLabel = `${ratioPercent}%`;
    divider.dataset.ratio = ratioLabel;
    const secondaryRatioLabel = `${100 - ratioPercent}%`;
    primary.dataset.splitRatio = area1Slot === DRIVE_WORKSPACE_SLOT.PRIMARY ? ratioLabel : secondaryRatioLabel;
    secondary.dataset.splitRatio = area1Slot === DRIVE_WORKSPACE_SLOT.SECONDARY ? ratioLabel : secondaryRatioLabel;
    divider.setAttribute?.("aria-valuenow", String(ratioPercent));
    const geometrySignature = [
      orientation,
      layoutMode,
      area1Slot,
      dividerWidth,
      secondaryWidth.toFixed(2),
      dividerHeight,
      secondaryHeight.toFixed(2),
      compactPrimary ? 1 : 0,
    ].join("|");
    const geometryChanged = geometrySignature !== appliedGeometrySignature;
    if (geometryChanged) {
      appliedGeometrySignature = geometrySignature;
      root.style?.setProperty(styleProperties.dividerWidth, `${dividerWidth}px`);
      root.style?.setProperty(styleProperties.secondaryWidth, `${secondaryWidth}px`);
      root.style?.setProperty(styleProperties.dividerHeight, `${dividerHeight}px`);
      root.style?.setProperty(styleProperties.secondaryHeight, `${secondaryHeight}px`);
      options.onGeometry?.({
        active,
        ratio: currentRatio,
        orientation,
        mode: layoutMode,
        area1Slot,
        compactPrimary,
        renderContent,
      });
    }
    if (renderContent) scheduleContentResize();
    return geometryChanged;
  }

  function cancelDrag() {
    const previousPointerId = pointerId;
    const wasDragging = previousPointerId !== null || hasStateClass(root, "resizing");
    pointerId = null;
    dragRect = null;
    pointerOffset = 0;
    if (previousPointerId !== null && divider.hasPointerCapture?.(previousPointerId)) {
      divider.releasePointerCapture?.(previousPointerId);
    }
    if (ratioFrameRequest) env.cancelFrame(ratioFrameRequest);
    ratioFrameRequest = 0;
    env.clearTimer(settleTimer);
    settleTimer = 0;
    if (pendingRatio !== null) ratios[orientation] = pendingRatio;
    pendingRatio = null;
    ratios[orientation] = snap(ratios[orientation]);
    syncCompactPrimary();
    removeStateClasses(root, "resizing", "settling");
    options.onResizeStateChange?.(false);
    if (wasDragging && !destroyed) {
      const detail = layoutDetail();
      dispatch(events.resizeEnd, detail);
      options.onResizeEnd?.(detail);
    }
  }

  function setActive(value) {
    assertAlive("set active");
    const next = Boolean(value);
    if (next === active) return false;
    active = next;
    root.dataset.driveWorkspaceActive = String(active);
    if (active) {
      syncLayoutState();
      toggleStateClass(root, "active", true);
      syncContentSplitClass(!suspended && layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT);
      syncSlotPresentation();
      applyGeometry(false);
      syncContentActivity("workspace activated");
      if (activationResizeFrameRequest) env.cancelFrame(activationResizeFrameRequest);
      activationResizeFrameRequest = env.requestFrame(() => {
        activationResizeFrameRequest = 0;
        if (!destroyed && active) scheduleContentResize(true);
      });
    } else {
      if (activationResizeFrameRequest) env.cancelFrame(activationResizeFrameRequest);
      activationResizeFrameRequest = 0;
      cancelDrag();
      toggleStateClass(root, "active", false);
      syncContentSplitClass(false);
      syncSlotPresentation();
      syncContentActivity("workspace deactivated");
      applyGeometry();
    }
    dispatchLayout();
    return true;
  }

  function setSuspended(value) {
    assertAlive("set suspended");
    const next = Boolean(value);
    if (next === suspended) return false;
    suspended = next;
    root.dataset.driveWorkspaceSuspended = String(suspended);
    toggleStateClass(root, "suspended", suspended);
    if (suspended) cancelDrag();
    syncSlotPresentation();
    syncContentSplitClass(active && !suspended && layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT);
    syncContentActivity(suspended ? "workspace suspended" : "workspace resumed");
    applyGeometry(false);
    dispatchLayout();
    return true;
  }

  function flushPendingRatio() {
    ratioFrameRequest = 0;
    if (pendingRatio === null || destroyed) return;
    ratios[orientation] = clamp(pendingRatio);
    pendingRatio = null;
    const extent = orientation === DRIVE_WORKSPACE_ORIENTATION.VERTICAL ? dragRect?.height : dragRect?.width;
    applyGeometry(false, extent || 0);
  }

  function queueRatio(value) {
    pendingRatio = snap(value);
    if (!ratioFrameRequest) ratioFrameRequest = env.requestFrame(flushPendingRatio);
  }

  function updateFromPointer(event) {
    if (!dragRect || layoutMode !== DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT) return;
    const coalesced = event.getCoalescedEvents?.();
    const point = coalesced?.length ? coalesced[coalesced.length - 1] : event;
    const vertical = orientation === DRIVE_WORKSPACE_ORIENTATION.VERTICAL;
    const dividerExtent = vertical ? geometry.dividerHeight : geometry.dividerWidth;
    const dragExtent = vertical ? dragRect.height : dragRect.width;
    if (dragExtent <= dividerExtent) return;
    const pointPosition = vertical ? point.clientY : point.clientX;
    const dragStart = vertical ? dragRect.top : dragRect.left;
    const dividerCenter = pointPosition - pointerOffset;
    queueRatio((dividerCenter - dragStart - (dividerExtent / 2)) / (dragExtent - dividerExtent));
  }

  function finishDrag(event) {
    if (pointerId === null || (event && event.pointerId !== pointerId)) return;
    const previousPointerId = pointerId;
    pointerId = null;
    dragRect = null;
    pointerOffset = 0;
    if (divider.hasPointerCapture?.(previousPointerId)) divider.releasePointerCapture?.(previousPointerId);
    if (ratioFrameRequest) env.cancelFrame(ratioFrameRequest);
    ratioFrameRequest = 0;
    if (pendingRatio !== null) ratios[orientation] = clamp(pendingRatio);
    pendingRatio = null;
    toggleStateClass(root, "resizing", false);
    toggleStateClass(root, "settling", true);
    ratios[orientation] = snap(ratios[orientation]);
    options.onResizeStateChange?.(false);
    applyGeometry();
    env.clearTimer(settleTimer);
    settleTimer = env.setTimer(() => {
      settleTimer = 0;
      if (!destroyed) toggleStateClass(root, "settling", false);
    }, geometry.settleMs);
    persist();
    dispatchLayout();
    const detail = layoutDetail();
    dispatch(events.resizeEnd, detail);
    options.onResizeEnd?.(detail);
  }

  function handlePointerDown(event) {
    if (!active || layoutMode !== DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT) return;
    event.preventDefault?.();
    pointerId = event.pointerId;
    dragRect = root.getBoundingClientRect?.() || {};
    const dividerRect = divider.getBoundingClientRect?.() || {};
    pointerOffset = orientation === DRIVE_WORKSPACE_ORIENTATION.VERTICAL
      ? event.clientY - (Number(dividerRect.top || 0) + (Number(dividerRect.height || 0) / 2))
      : event.clientX - (Number(dividerRect.left || 0) + (Number(dividerRect.width || 0) / 2));
    env.clearTimer(settleTimer);
    settleTimer = 0;
    toggleStateClass(root, "settling", false);
    toggleStateClass(root, "resizing", true);
    options.onResizeStateChange?.(true);
    const detail = layoutDetail();
    dispatch(events.resizeStart, detail);
    options.onResizeStart?.(detail);
    divider.setPointerCapture?.(event.pointerId);
    updateFromPointer(event);
  }

  function handleDividerPointerMove(event) {
    if (event.pointerId !== pointerId || !divider.hasPointerCapture?.(event.pointerId)) return;
    updateFromPointer(event);
  }

  function handleWindowPointerMove(event) {
    if (event.pointerId === pointerId) updateFromPointer(event);
  }

  function handleBlur() {
    finishDrag();
  }

  function handleDividerKeydown(event) {
    if (!active || layoutMode !== DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT) return;
    const decreaseKey = orientation === DRIVE_WORKSPACE_ORIENTATION.VERTICAL ? "ArrowUp" : "ArrowLeft";
    const increaseKey = orientation === DRIVE_WORKSPACE_ORIENTATION.VERTICAL ? "ArrowDown" : "ArrowRight";
    if (![decreaseKey, increaseKey, "Home", "End"].includes(event.key)) return;
    event.preventDefault?.();
    if (event.key === "Home") ratios[orientation] = geometry.minRatio;
    else if (event.key === "End") ratios[orientation] = geometry.maxRatio;
    else ratios[orientation] = snap(
      ratios[orientation] + (event.key === decreaseKey ? -geometry.ratioStep : geometry.ratioStep),
    );
    applyGeometry();
    persist();
    dispatchLayout();
  }

  function handleViewportResize() {
    scheduleContentResize(true);
  }

  listen(divider, "pointerdown", handlePointerDown);
  listen(divider, "pointermove", handleDividerPointerMove);
  for (const eventName of ["pointerup", "pointercancel", "lostpointercapture"]) {
    listen(divider, eventName, finishDrag);
  }
  listen(target, "pointermove", handleWindowPointerMove, { passive: true });
  listen(target, "pointerup", finishDrag, { capture: true, passive: true });
  listen(target, "pointercancel", finishDrag, { capture: true, passive: true });
  listen(target, "blur", handleBlur, { passive: true });
  listen(divider, "keydown", handleDividerKeydown);
  listen(target, "resize", handleViewportResize, { passive: true });
  listen(target, "orientationchange", handleViewportResize, { passive: true });
  listen(target.visualViewport, "resize", handleViewportResize, { passive: true });

  if (typeof env.ResizeObserver === "function") {
    const workspaceResizeObserver = new env.ResizeObserver(() => {
      if (destroyed) return;
      if (active) applyGeometry();
      else scheduleContentResize(true);
    });
    workspaceResizeObserver.observe(root);
    observers.push(workspaceResizeObserver);
    const contentResizeObserver = new env.ResizeObserver(() => scheduleContentResize());
    contentResizeObserver.observe(primary);
    contentResizeObserver.observe(secondary);
    observers.push(contentResizeObserver);
  }

  function setOrientation(value) {
    assertAlive("set orientation");
    const nextOrientation = normalizeOrientation(value);
    if (nextOrientation === orientation) return false;
    cancelDrag();
    orientation = nextOrientation;
    appliedGeometrySignature = "";
    syncOrientationState();
    syncLayoutState();
    applyGeometry();
    dispatchLayout();
    return true;
  }

  function setRatio(value, setOptions = {}) {
    assertAlive("set ratio");
    const config = typeof setOptions === "boolean" ? { persist: setOptions } : setOptions;
    const targetOrientation = normalizeOrientation(config.orientation || orientation);
    if (targetOrientation === orientation && pointerId !== null) return false;
    const nextRatio = snap(value);
    if (ratios[targetOrientation] === nextRatio) return false;
    ratios[targetOrientation] = nextRatio;
    if (targetOrientation === orientation) applyGeometry();
    if (config.persist) persist(targetOrientation);
    if (targetOrientation === orientation) dispatchLayout();
    return true;
  }

  function setLayout(value = {}) {
    assertAlive("set layout");
    const nextMode = normalizeLayoutMode(value.mode);
    const nextArea1Slot = normalizeSlotValue(value.area1Slot || DRIVE_WORKSPACE_SLOT.PRIMARY);
    if (nextMode === layoutMode && nextArea1Slot === area1Slot) return false;
    cancelDrag();
    layoutMode = nextMode;
    area1Slot = nextArea1Slot;
    appliedGeometrySignature = "";
    syncLayoutState();
    syncSlotPresentation();
    syncContentSplitClass(active && !suspended && layoutMode === DRIVE_WORKSPACE_LAYOUT_MODE.SPLIT);
    syncContentActivity("workspace layout changed");
    applyGeometry();
    dispatchLayout();
    return true;
  }

  function snapshot() {
    return layoutDetail();
  }

  function destroy() {
    if (destroyed) return false;
    // Mark first so callbacks queued by observer/event delivery cannot schedule
    // new work while resources are being released.
    destroyed = true;
    const wasActive = active;
    active = false;
    if (ratioFrameRequest) env.cancelFrame(ratioFrameRequest);
    if (contentResizeFrameRequest) env.cancelFrame(contentResizeFrameRequest);
    if (activationResizeFrameRequest) env.cancelFrame(activationResizeFrameRequest);
    ratioFrameRequest = 0;
    contentResizeFrameRequest = 0;
    activationResizeFrameRequest = 0;
    env.clearTimer(settleTimer);
    settleTimer = 0;
    pendingRatio = null;
    forceContentResize = false;
    pointerId = null;
    dragRect = null;
    for (const [eventTarget, type, handler, listenerOptions] of listeners.splice(0)) {
      eventTarget.removeEventListener?.(type, handler, listenerOptions);
    }
    for (const observer of observers.splice(0)) observer.disconnect?.();
    const mountedContents = new Set([
      ...[...assignments.values()].filter(Boolean).map((record) => record.content),
      ...presentations.keys(),
    ]);
    for (const content of mountedContents) parkContent(content);
    presentations.clear();
    for (const [slot] of assignments) {
      moveChildrenToParking(slotElement(slot));
      assignments.set(slot, null);
      setHostDataset(slot, null);
    }
    contentRectSignatures.clear();
    removeStateClasses(root, "active", "resizing", "settling", "swapped", "area1Only", "area2Only", "suspended");
    syncContentSplitClass(false);
    toggleStateClass(primary, "primaryCompact", false);
    toggleStateClass(documentRoot?.documentElement, "documentCompact", false);
    root.dataset.driveWorkspaceActive = "false";
    root.dataset.driveWorkspaceSuspended = "false";
    workspaces.delete(root);
    options.onDestroy?.({ wasActive });
    return true;
  }

  root.dataset.driveWorkspaceActive = String(active);
  root.dataset.driveWorkspaceSuspended = String(suspended);
  syncOrientationState();
  syncLayoutState();
  syncSlotPresentation();

  const api = Object.freeze({
    setActive,
    setSuspended,
    setOrientation,
    setRatio,
    setLayout,
    setContents,
    applyGeometry,
    cancelDrag,
    snapshot,
    registerContent,
    unregisterContent,
    getContent,
    contentIdForSlot,
    contentForSlot: contentIdForSlot,
    contentSnapshot,
    domOwnershipStatus,
    assertDomOwnership,
    presentContent,
    restorePresentedContent,
    syncContentActivity,
    resizeContents,
    destroy,
  });
  workspaces.set(root, api);
  return api;
}

export function getDriveWorkspace(root) {
  return root && typeof root === "object" ? (workspaces.get(root) || null) : null;
}

export const DriveWorkspace = Object.freeze({
  SLOT: DRIVE_WORKSPACE_SLOT,
  ORIENTATION: DRIVE_WORKSPACE_ORIENTATION,
  LAYOUT_MODE: DRIVE_WORKSPACE_LAYOUT_MODE,
  create: createDriveWorkspace,
  get: getDriveWorkspace,
});

export function installDriveWorkspaceFacade(target = globalThis) {
  target.DriveWorkspace = DriveWorkspace;
  return DriveWorkspace;
}
