"use strict";

export const VISION_SESSION_DESIRED = Object.freeze({
  INACTIVE: "inactive",
  SUSPENDED: "suspended",
  WAITING: "waiting",
  RUNNING: "running",
});

export const VISION_SESSION_PAGE = Object.freeze({
  VISIBLE: "visible",
  HIDDEN: "hidden",
  FROZEN: "frozen",
});

export const VISION_SESSION_NETWORK = Object.freeze({
  UNKNOWN: "unknown",
  REACHABLE: "reachable",
  UNREACHABLE: "unreachable",
});

export const VISION_SESSION_PHASE = Object.freeze({
  IDLE: "idle",
  CONNECTING: "connecting",
  CONNECTED: "connected",
  RECOVERING: "recovering",
  BLOCKED: "blocked",
});

export const VISION_SESSION_EVENT = Object.freeze({
  USER_START: "user-start",
  USER_STOP: "user-stop",
  PAGE_HIDDEN: "page-hidden",
  PAGE_FROZEN: "page-frozen",
  PAGE_VISIBLE: "page-visible",
  PAGE_RESTORED: "page-restored",
  WINDOW_FOCUS: "window-focus",
  SOURCE_AVAILABLE: "source-available",
  SOURCE_UNAVAILABLE: "source-unavailable",
  NETWORK_REACHABLE: "network-reachable",
  NETWORK_UNREACHABLE: "network-unreachable",
  CONNECT_BEGIN: "connect-begin",
  CONNECT_SUCCESS: "connect-success",
  CONNECT_FAILURE: "connect-failure",
  CONNECT_ABORTED: "connect-aborted",
  CONNECTION_LOST: "connection-lost",
  OWNERSHIP_BLOCKED: "ownership-blocked",
  OWNERSHIP_RELEASED: "ownership-released",
  DISCONNECT_COMPLETE: "disconnect-complete",
});

export const VISION_SESSION_EFFECT = Object.freeze({
  ENSURE_RUNNING: "ensure-running",
  ENSURE_SUSPENDED: "ensure-suspended",
  ENSURE_STOPPED: "ensure-stopped",
  SCHEDULE_RECOVERY: "schedule-recovery",
});

const VALID_EVENTS = new Set(Object.values(VISION_SESSION_EVENT));
const VALID_PAGES = new Set(Object.values(VISION_SESSION_PAGE));
const VALID_NETWORKS = new Set(Object.values(VISION_SESSION_NETWORK));
const VALID_PHASES = new Set(Object.values(VISION_SESSION_PHASE));

function desiredFor(state) {
  if (!state.requested) return VISION_SESSION_DESIRED.INACTIVE;
  if (state.page !== VISION_SESSION_PAGE.VISIBLE) return VISION_SESSION_DESIRED.SUSPENDED;
  if (!state.sourceAvailable || state.network === VISION_SESSION_NETWORK.UNREACHABLE) {
    return VISION_SESSION_DESIRED.WAITING;
  }
  return VISION_SESSION_DESIRED.RUNNING;
}

function effectForDesired(desired) {
  if (desired === VISION_SESSION_DESIRED.RUNNING) return VISION_SESSION_EFFECT.ENSURE_RUNNING;
  if (desired === VISION_SESSION_DESIRED.INACTIVE) return VISION_SESSION_EFFECT.ENSURE_STOPPED;
  return VISION_SESSION_EFFECT.ENSURE_SUSPENDED;
}

function normalizePage(value) {
  return VALID_PAGES.has(value) ? value : VISION_SESSION_PAGE.VISIBLE;
}

function normalizeNetwork(value) {
  return VALID_NETWORKS.has(value) ? value : VISION_SESSION_NETWORK.UNKNOWN;
}

function normalizePhase(value) {
  return VALID_PHASES.has(value) ? value : VISION_SESSION_PHASE.IDLE;
}

function freezeState(state) {
  return Object.freeze({
    requested: Boolean(state.requested),
    page: normalizePage(state.page),
    sourceAvailable: state.sourceAvailable !== false,
    network: normalizeNetwork(state.network),
    desired: state.desired,
    phase: normalizePhase(state.phase),
    revision: Math.max(0, Number.parseInt(state.revision, 10) || 0),
  });
}

export function createVisionSessionState(overrides = {}) {
  const base = {
    requested: false,
    page: VISION_SESSION_PAGE.VISIBLE,
    sourceAvailable: true,
    network: VISION_SESSION_NETWORK.UNKNOWN,
    phase: VISION_SESSION_PHASE.IDLE,
    revision: 0,
    ...overrides,
  };
  base.desired = desiredFor(base);
  return freezeState(base);
}

function transitionChanged(previous, next) {
  return previous.requested !== next.requested
    || previous.page !== next.page
    || previous.sourceAvailable !== next.sourceAvailable
    || previous.network !== next.network
    || previous.desired !== next.desired
    || previous.phase !== next.phase;
}

export function reduceVisionSessionState(current, event) {
  const previous = createVisionSessionState(current);
  const type = typeof event === "string" ? event : event?.type;
  if (!VALID_EVENTS.has(type)) {
    throw new TypeError(`Unknown Carrot Vision session event: ${String(type)}`);
  }

  const next = { ...previous };
  let explicitEffect = null;

  switch (type) {
    case VISION_SESSION_EVENT.USER_START:
      next.requested = true;
      break;
    case VISION_SESSION_EVENT.USER_STOP:
      next.requested = false;
      break;
    case VISION_SESSION_EVENT.PAGE_HIDDEN:
      next.page = VISION_SESSION_PAGE.HIDDEN;
      break;
    case VISION_SESSION_EVENT.PAGE_FROZEN:
      next.page = VISION_SESSION_PAGE.FROZEN;
      break;
    case VISION_SESSION_EVENT.PAGE_VISIBLE:
    case VISION_SESSION_EVENT.PAGE_RESTORED:
    case VISION_SESSION_EVENT.WINDOW_FOCUS:
      next.page = VISION_SESSION_PAGE.VISIBLE;
      break;
    case VISION_SESSION_EVENT.SOURCE_AVAILABLE:
      next.sourceAvailable = true;
      break;
    case VISION_SESSION_EVENT.SOURCE_UNAVAILABLE:
      next.sourceAvailable = false;
      break;
    case VISION_SESSION_EVENT.NETWORK_REACHABLE:
      next.network = VISION_SESSION_NETWORK.REACHABLE;
      break;
    case VISION_SESSION_EVENT.NETWORK_UNREACHABLE:
      next.network = VISION_SESSION_NETWORK.UNREACHABLE;
      break;
    case VISION_SESSION_EVENT.CONNECT_BEGIN:
      next.phase = VISION_SESSION_PHASE.CONNECTING;
      break;
    case VISION_SESSION_EVENT.CONNECT_SUCCESS:
      next.phase = VISION_SESSION_PHASE.CONNECTED;
      if (previous.desired !== VISION_SESSION_DESIRED.RUNNING) {
        explicitEffect = effectForDesired(previous.desired);
      }
      break;
    case VISION_SESSION_EVENT.CONNECT_FAILURE:
    case VISION_SESSION_EVENT.CONNECTION_LOST:
      next.phase = previous.desired === VISION_SESSION_DESIRED.RUNNING
        ? VISION_SESSION_PHASE.RECOVERING
        : VISION_SESSION_PHASE.IDLE;
      if (previous.desired === VISION_SESSION_DESIRED.RUNNING) {
        explicitEffect = VISION_SESSION_EFFECT.SCHEDULE_RECOVERY;
      }
      break;
    case VISION_SESSION_EVENT.CONNECT_ABORTED:
    case VISION_SESSION_EVENT.DISCONNECT_COMPLETE:
      next.phase = VISION_SESSION_PHASE.IDLE;
      break;
    case VISION_SESSION_EVENT.OWNERSHIP_BLOCKED:
      next.phase = VISION_SESSION_PHASE.BLOCKED;
      break;
    case VISION_SESSION_EVENT.OWNERSHIP_RELEASED:
      next.phase = VISION_SESSION_PHASE.IDLE;
      if (previous.desired === VISION_SESSION_DESIRED.RUNNING) {
        explicitEffect = VISION_SESSION_EFFECT.ENSURE_RUNNING;
      }
      break;
    default:
      break;
  }

  next.desired = desiredFor(next);
  const effects = [];
  if (previous.desired !== next.desired) {
    effects.push(effectForDesired(next.desired));
  } else if (explicitEffect) {
    effects.push(explicitEffect);
  }

  const changed = transitionChanged(previous, next);
  next.revision = changed ? previous.revision + 1 : previous.revision;
  return Object.freeze({
    changed,
    state: freezeState(next),
    effects: Object.freeze(effects),
  });
}

const SESSION_POLICY_API = Object.freeze({
  desired: VISION_SESSION_DESIRED,
  page: VISION_SESSION_PAGE,
  network: VISION_SESSION_NETWORK,
  phase: VISION_SESSION_PHASE,
  event: VISION_SESSION_EVENT,
  effect: VISION_SESSION_EFFECT,
  createState: createVisionSessionState,
  reduce: reduceVisionSessionState,
});

export function installDriveVisionSessionPolicyFacade(target = globalThis) {
  if (!target) return SESSION_POLICY_API;
  if (target.DriveVisionSessionPolicy) return target.DriveVisionSessionPolicy;
  target.DriveVisionSessionPolicy = SESSION_POLICY_API;
  return target.DriveVisionSessionPolicy;
}

export const DriveVisionSessionPolicy = SESSION_POLICY_API;
