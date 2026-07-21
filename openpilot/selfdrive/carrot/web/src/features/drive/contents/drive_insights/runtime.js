import { createSegmentedControl } from "../../../../ui/components/segmented_control/segmented_control.js";
import {
  STATE as STATE_SURFACE_STATE,
  TONE as STATE_SURFACE_TONE,
  VARIANT as STATE_SURFACE_VARIANT,
  createStateSurface,
} from "../../../../ui/components/state_surface/state_surface.js";
import { createDriveInsightsForwardRenderer } from "./forward.js";
import { createDriveInsightsGraphRenderer } from "./graph.js";
import {
  DRIVE_INSIGHTS_HISTORY_SOURCE,
  DRIVE_INSIGHTS_HISTORY_STATE,
  createDriveInsightsHistory,
} from "./history.js";
import {
  DRIVE_INSIGHTS_RENDER_CADENCE_MS,
  createDriveInsightsRenderScheduler,
} from "./scheduler.js";
import { createDriveInsightsLiveSource } from "./source_live.js";
import { createDriveInsightsReplaySource } from "./source_replay.js";

export const DRIVE_INSIGHTS_VIEW = Object.freeze({
  GRAPH: "graph",
  FORWARD: "forward",
});

const runtimeSingletons = new WeakMap();
const VIEWS = new Set(Object.values(DRIVE_INSIGHTS_VIEW));

function normalizeView(value) {
  const view = String(value || "").trim().toLowerCase();
  return VIEWS.has(view) ? view : DRIVE_INSIGHTS_VIEW.GRAPH;
}

function normalizeSource(value) {
  return String(value || "").trim().toLowerCase() === DRIVE_INSIGHTS_HISTORY_SOURCE.REPLAY
    ? DRIVE_INSIGHTS_HISTORY_SOURCE.REPLAY
    : DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE;
}

function text(target, key, fallback) {
  return typeof target.getUIText === "function" ? target.getUIText(key, fallback) : fallback;
}

function append(parent, ...children) {
  if (typeof parent?.append === "function") parent.append(...children);
  else for (const child of children) parent?.appendChild?.(child);
}

function createRuntimeDom(documentRoot, target) {
  const root = documentRoot.createElement("section");
  root.className = "drive-insights";
  root.dataset.source = DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE;
  root.dataset.view = DRIVE_INSIGHTS_VIEW.GRAPH;

  const tabs = documentRoot.createElement("div");
  tabs.className = "c-segmented-control drive-insights__tabs";
  tabs.setAttribute("role", "tablist");
  tabs.setAttribute("aria-label", text(target, "web_drive_layout_content_drive_insights", "Drive Insights"));

  const views = documentRoot.createElement("div");
  views.className = "drive-insights__views";
  const graph = documentRoot.createElement("div");
  graph.className = "drive-insights__view drive-insights__view--graph";
  graph.dataset.driveInsightsPanel = DRIVE_INSIGHTS_VIEW.GRAPH;
  graph.setAttribute("role", "tabpanel");
  const forward = documentRoot.createElement("div");
  forward.className = "drive-insights__view drive-insights__view--forward";
  forward.dataset.driveInsightsPanel = DRIVE_INSIGHTS_VIEW.FORWARD;
  forward.setAttribute("role", "tabpanel");
  forward.hidden = true;

  const state = documentRoot.createElement("div");
  state.className = "drive-insights__state";

  const buttons = new Map();
  for (const [view, key, fallback] of [
    [DRIVE_INSIGHTS_VIEW.GRAPH, "drive_insights_graph", "Graphs"],
    [DRIVE_INSIGHTS_VIEW.FORWARD, "drive_insights_forward", "Forward"],
  ]) {
    const button = documentRoot.createElement("button");
    button.type = "button";
    button.className = "c-segmented-control__item";
    button.dataset.driveInsightsView = view;
    button.dataset.value = view;
    button.setAttribute("role", "tab");
    button.setAttribute("aria-selected", String(view === DRIVE_INSIGHTS_VIEW.GRAPH));
    button.textContent = text(target, key, fallback);
    tabs.appendChild(button);
    buttons.set(view, button);
  }

  append(views, graph, forward, state);
  append(root, tabs, views);
  return Object.freeze({ root, tabs, views, graph, forward, state, buttons });
}

function viewFreshness(snapshot, view) {
  if (view === DRIVE_INSIGHTS_VIEW.GRAPH) {
    return String(snapshot?.freshness?.ego?.state || "missing");
  }
  const sensing = ["lanes", "leads", "radar"].map((domain) => (
    String(snapshot?.freshness?.[domain]?.state || "missing")
  ));
  if (sensing.includes("fresh")) return "fresh";
  if (sensing.includes("stale")) return "stale";
  return "missing";
}

export function createDriveInsightsRuntime(options = {}) {
  const target = options.target || globalThis;
  const documentRoot = options.document || target.document;
  if (!documentRoot?.createElement) return null;
  const dom = options.dom || createRuntimeDom(documentRoot, target);
  const history = options.history || createDriveInsightsHistory({
    nowMs: () => Number(target.CarrotDriveLiveStateProvider?.nowMs?.() ?? target.performance?.now?.() ?? 0),
  });
  const graphRenderer = options.graphRenderer || createDriveInsightsGraphRenderer({
    root: dom.graph,
    target,
    document: documentRoot,
    text: (key, fallback) => text(target, key, fallback),
  });
  const forwardRenderer = options.forwardRenderer || createDriveInsightsForwardRenderer({
    root: dom.forward,
    target,
    document: documentRoot,
    text: (key, fallback) => text(target, key, fallback),
  });
  const stateSurface = options.stateSurface || createStateSurface({
    host: dom.state,
    className: "drive-insights__status",
    featureLabel: () => text(target, "web_drive_layout_content_drive_insights", "Drive Insights"),
  }, { document: documentRoot });
  if (!history || !graphRenderer || !forwardRenderer || !stateSurface) return null;

  const setIntervalFn = options.setInterval || target.setInterval?.bind(target);
  const clearIntervalFn = options.clearInterval || target.clearInterval?.bind(target);
  const listeners = [];
  let active = false;
  let destroyed = false;
  let view = normalizeView(options.initialView);
  let sourceKind = normalizeSource(options.source);
  let sourceAdapter = null;
  let sourceUnsubscribe = null;
  let dataLease = null;
  let trackLease = null;
  let staleTimer = null;
  let lastSnapshot = null;
  let replayInput = options.replayInput || null;
  let lastError = null;
  let generation = 0;
  let queuedLiveSnapshot = null;

  function viewCadenceMs() {
    return view === DRIVE_INSIGHTS_VIEW.FORWARD
      ? DRIVE_INSIGHTS_RENDER_CADENCE_MS.forward
      : DRIVE_INSIGHTS_RENDER_CADENCE_MS.graph;
  }

  const renderScheduler = options.renderScheduler || createDriveInsightsRenderScheduler({
    target,
    onFlush: () => flushLiveUpdate(),
  });
  if (!renderScheduler) return null;

  function showWaiting() {
    stateSurface.show(STATE_SURFACE_STATE.LOADING, {
      tone: STATE_SURFACE_TONE.INFO,
      variant: STATE_SURFACE_VARIANT.COVER,
      title: () => text(target, "drive_insights_waiting_title", "Waiting for driving data"),
      description: () => text(
        target,
        "drive_insights_waiting_detail",
        "Live driving information will appear here when it is available.",
      ),
      actions: [],
    });
  }

  function syncSurface() {
    const historyStatus = history.status({
      nowMonotonicMs: Math.max(0, Number(target.CarrotDriveLiveStateProvider?.nowMs?.() ?? 0)),
    });
    const freshness = lastSnapshot ? viewFreshness(lastSnapshot, view) : "missing";
    if (!lastSnapshot || historyStatus.state === DRIVE_INSIGHTS_HISTORY_STATE.MISSING) showWaiting();
    else stateSurface.hide();
    dom.root.dataset.dataState = historyStatus.state === DRIVE_INSIGHTS_HISTORY_STATE.STALE
      ? "stale"
      : freshness;
  }

  function render() {
    if (destroyed) return false;
    if (view === DRIVE_INSIGHTS_VIEW.GRAPH) graphRenderer.render(history.window());
    else forwardRenderer.render(lastSnapshot);
    syncSurface();
    return true;
  }

  function sourceContext(adapter = sourceAdapter) {
    const context = adapter?.context?.();
    return context && typeof context === "object" ? context : {};
  }

  function ingestLive(snapshot = sourceAdapter?.snapshot?.()) {
    if (!active || sourceKind !== DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE || !snapshot) return false;
    const context = sourceContext();
    lastSnapshot = snapshot;
    history.pushLive(snapshot, {
      receivedAtMonotonicMs: Number.isFinite(Number(context.receivedAtMonotonicMs))
        ? Number(context.receivedAtMonotonicMs)
        : Number(snapshot.timestampMs),
      connectionState: context.connectionState || "connected",
      connectionId: context.connectionId ?? null,
      routeId: context.routeId ?? null,
      reconnected: context.reconnected === true,
    });
    generation += 1;
    return render();
  }

  function flushLiveUpdate() {
    if (!active || sourceKind !== DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE) return false;
    const snapshot = queuedLiveSnapshot || sourceAdapter?.snapshot?.();
    queuedLiveSnapshot = null;
    return snapshot ? ingestLive(snapshot) : false;
  }

  function scheduleLiveUpdate(snapshot = null, scheduleOptions = {}) {
    if (!active || sourceKind !== DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE) return false;
    if (snapshot && typeof snapshot === "object") queuedLiveSnapshot = snapshot;
    return renderScheduler.request(viewCadenceMs(), scheduleOptions);
  }

  function rebuildReplay() {
    if (!sourceAdapter || sourceKind !== DRIVE_INSIGHTS_HISTORY_SOURCE.REPLAY) return false;
    const context = sourceContext();
    const currentTimestampMs = Number(
      context.currentTimestampMs
      ?? replayInput?.currentTimestampMs
      ?? sourceAdapter.snapshot?.()?.timestampMs,
    );
    if (!Number.isFinite(currentTimestampMs) || currentTimestampMs < 0) {
      lastSnapshot = null;
      history.reset("replay-position-missing");
      return render();
    }
    const timestamps = history.replayTimestamps(currentTimestampMs);
    const samples = typeof sourceAdapter.snapshotsAt === "function"
      ? sourceAdapter.snapshotsAt(timestamps)
      : timestamps.map((timestampMs) => sourceAdapter.snapshotAt?.(timestampMs)).filter(Boolean);
    history.rebuildReplay(samples, {
      currentTimestampMs,
      routeId: context.routeId ?? replayInput?.routeId ?? null,
    });
    lastSnapshot = sourceAdapter.snapshotAt?.(currentTimestampMs) || sourceAdapter.snapshot?.() || null;
    generation += 1;
    return render();
  }

  // Radar track lists are only drawn by the forward view, so the heavier
  // "tracks" subscription follows the visible view instead of the runtime.
  function syncTrackLease() {
    const wanted = active
      && sourceKind === DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE
      && view === DRIVE_INSIGHTS_VIEW.FORWARD;
    if (wanted === Boolean(trackLease?.active)) return false;
    if (!wanted) {
      trackLease?.release?.();
      trackLease = null;
      return true;
    }
    const activity = options.activity || target.CarrotDriveDataActivity;
    if (!activity?.acquire) return false;
    trackLease = activity.acquire({ owner: "drive_insights_forward", tracks: true });
    return true;
  }

  function stopSource() {
    renderScheduler.cancel();
    queuedLiveSnapshot = null;
    sourceUnsubscribe?.();
    sourceUnsubscribe = null;
    sourceAdapter?.destroy?.();
    sourceAdapter = null;
    dataLease?.release?.();
    dataLease = null;
    trackLease?.release?.();
    trackLease = null;
    if (staleTimer !== null && clearIntervalFn) clearIntervalFn(staleTimer);
    staleTimer = null;
  }

  function startLive() {
    const activity = options.activity || target.CarrotDriveDataActivity;
    const provider = options.provider || target.CarrotDriveLiveStateProvider;
    if (!activity?.acquire || !provider?.snapshot || !provider?.subscribe) {
      showWaiting();
      return false;
    }
    dataLease = activity.acquire({ owner: "drive_insights", hud: true, overlay: true });
    sourceAdapter = (options.createLiveSource || createDriveInsightsLiveSource)({ provider });
    if (!sourceAdapter) {
      dataLease.release();
      dataLease = null;
      showWaiting();
      return false;
    }
    syncTrackLease();
    sourceUnsubscribe = typeof sourceAdapter.subscribeUpdates === "function"
      ? sourceAdapter.subscribeUpdates(() => scheduleLiveUpdate())
      : sourceAdapter.subscribe?.((snapshot) => scheduleLiveUpdate(snapshot)) || null;
    const initial = sourceAdapter.snapshot?.();
    if (initial) scheduleLiveUpdate(initial, { immediate: true });
    else showWaiting();
    if (setIntervalFn) staleTimer = setIntervalFn(syncSurface, 250);
    return true;
  }

  function startReplay() {
    if (!replayInput) {
      showWaiting();
      return false;
    }
    sourceAdapter = (options.createReplaySource || createDriveInsightsReplaySource)(replayInput);
    if (!sourceAdapter) {
      showWaiting();
      return false;
    }
    return rebuildReplay();
  }

  function startSource() {
    stopSource();
    history.setContext({ source: sourceKind });
    return sourceKind === DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE ? startLive() : startReplay();
  }

  function setView(nextView) {
    if (destroyed) return false;
    const normalized = normalizeView(nextView);
    const changed = normalized !== view;
    view = normalized;
    dom.root.dataset.view = view;
    for (const [candidate, button] of dom.buttons) {
      const selected = candidate === view;
      button.setAttribute("aria-selected", String(selected));
      button.classList?.toggle("is-selected", selected);
      const panel = candidate === DRIVE_INSIGHTS_VIEW.GRAPH ? dom.graph : dom.forward;
      panel.hidden = !selected;
    }
    segmented.sync();
    syncTrackLease();
    if (active && sourceKind === DRIVE_INSIGHTS_HISTORY_SOURCE.LIVE) {
      scheduleLiveUpdate(null, { immediate: true });
    } else {
      render();
    }
    return changed;
  }

  function handleTabClick(event) {
    const button = event.target?.closest?.("[data-drive-insights-view]");
    if (button && dom.tabs.contains?.(button)) setView(button.dataset.driveInsightsView);
  }

  const segmented = options.segmented || createSegmentedControl(dom.tabs, {
    itemSelector: "[data-drive-insights-view]",
    selectedAttribute: "aria-selected",
    activation: "automatic",
    onActivate(item) {
      setView(item?.dataset?.driveInsightsView);
    },
  });
  if (!segmented) return null;
  dom.tabs.addEventListener("click", handleTabClick);
  listeners.push([dom.tabs, "click", handleTabClick]);
  setView(view);

  function activate(context = {}) {
    if (destroyed) return false;
    const nextSource = normalizeSource(context.source || sourceKind);
    const sourceChanged = nextSource !== sourceKind;
    sourceKind = nextSource;
    dom.root.dataset.source = sourceKind;
    if (active && !sourceChanged) return false;
    active = true;
    dom.root.dataset.active = "true";
    startSource();
    return true;
  }

  function deactivate() {
    if (destroyed || !active) return false;
    active = false;
    dom.root.dataset.active = "false";
    stopSource();
    return true;
  }

  function resize(rect) {
    if (destroyed) return false;
    return view === DRIVE_INSIGHTS_VIEW.GRAPH
      ? graphRenderer.resize(rect)
      : forwardRenderer.resize(rect);
  }

  function setReplayInput(input) {
    if (destroyed) return false;
    replayInput = input && typeof input === "object" ? input : null;
    if (active && sourceKind === DRIVE_INSIGHTS_HISTORY_SOURCE.REPLAY) startSource();
    return Boolean(replayInput);
  }

  function status() {
    const providerNow = Number(target.CarrotDriveLiveStateProvider?.nowMs?.() ?? 0);
    return Object.freeze({
      active,
      destroyed,
      source: sourceKind,
      view,
      generation,
      lastError: lastError ? String(lastError.message || lastError) : "",
      history: history.status({ nowMonotonicMs: Math.max(0, providerNow) }),
      graph: graphRenderer.status(),
      forward: forwardRenderer.status(),
      stateSurface: stateSurface.snapshot(),
      renderScheduler: renderScheduler.status(),
      leaseActive: Boolean(dataLease?.active),
      trackLeaseActive: Boolean(trackLease?.active),
    });
  }

  function destroy() {
    if (destroyed) return false;
    deactivate();
    destroyed = true;
    for (const [eventTarget, name, listener] of listeners.splice(0)) {
      eventTarget.removeEventListener?.(name, listener);
    }
    segmented.destroy();
    stateSurface.destroy();
    renderScheduler.destroy();
    graphRenderer.destroy();
    forwardRenderer.destroy();
    history.reset("runtime-destroyed");
    dom.root.remove?.();
    runtimeSingletons.delete(target);
    return true;
  }

  const api = Object.freeze({
    root: dom.root,
    activate,
    deactivate,
    resize,
    setView,
    setReplayInput,
    render,
    status,
    destroy,
  });
  return api;
}

export function getOrCreateDriveInsightsRuntime(options = {}) {
  const target = options.target || globalThis;
  const existing = runtimeSingletons.get(target);
  if (existing) return existing;
  const runtime = createDriveInsightsRuntime({ ...options, target });
  if (runtime) runtimeSingletons.set(target, runtime);
  return runtime;
}

export function installDriveInsightsRuntimeFacade(target = globalThis, options = {}) {
  const runtime = getOrCreateDriveInsightsRuntime({ ...options, target });
  target.DriveInsightsRuntime = runtime;
  return runtime;
}
