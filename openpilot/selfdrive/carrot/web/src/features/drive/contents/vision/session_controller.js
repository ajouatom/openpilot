"use strict";

import {
  DriveVisionSessionPolicy,
  VISION_SESSION_DESIRED,
  VISION_SESSION_EFFECT,
  VISION_SESSION_EVENT,
  VISION_SESSION_NETWORK,
  VISION_SESSION_PAGE,
} from "./session_policy.js";

const EFFECT_FOR_DESIRED = Object.freeze({
  [VISION_SESSION_DESIRED.INACTIVE]: VISION_SESSION_EFFECT.ENSURE_STOPPED,
  [VISION_SESSION_DESIRED.SUSPENDED]: VISION_SESSION_EFFECT.ENSURE_SUSPENDED,
  [VISION_SESSION_DESIRED.WAITING]: VISION_SESSION_EFFECT.ENSURE_SUSPENDED,
  [VISION_SESSION_DESIRED.RUNNING]: VISION_SESSION_EFFECT.ENSURE_RUNNING,
});

function defaultSchedule(task) {
  if (typeof queueMicrotask === "function") {
    queueMicrotask(task);
    return;
  }
  Promise.resolve().then(task);
}

function pageEventFor(value, restored = false) {
  if (value === VISION_SESSION_PAGE.FROZEN) return VISION_SESSION_EVENT.PAGE_FROZEN;
  if (value === VISION_SESSION_PAGE.HIDDEN) return VISION_SESSION_EVENT.PAGE_HIDDEN;
  return restored ? VISION_SESSION_EVENT.PAGE_RESTORED : VISION_SESSION_EVENT.PAGE_VISIBLE;
}

function networkEventFor(value) {
  if (value === VISION_SESSION_NETWORK.REACHABLE) return VISION_SESSION_EVENT.NETWORK_REACHABLE;
  if (value === VISION_SESSION_NETWORK.UNREACHABLE) return VISION_SESSION_EVENT.NETWORK_UNREACHABLE;
  return null;
}

function mergeEffectDetail(previous, next) {
  return Object.freeze({
    ...(previous || {}),
    ...(next || {}),
    reasons: Object.freeze([
      ...((previous?.reasons || []).filter(Boolean)),
      ...((next?.reasons || []).filter(Boolean)),
      next?.reason,
    ].filter(Boolean)),
    freshSession: Boolean(previous?.freshSession || next?.freshSession),
  });
}

export function createVisionLifecycleController(options = {}) {
  const policy = options.policy || DriveVisionSessionPolicy;
  const handlers = options.handlers || {};
  const schedule = typeof options.schedule === "function" ? options.schedule : defaultSchedule;
  const onTransition = typeof options.onTransition === "function" ? options.onTransition : null;
  const onEffect = typeof options.onEffect === "function" ? options.onEffect : null;
  const onError = typeof options.onError === "function" ? options.onError : null;

  let state = policy.createState(options.initialState || {});
  let pendingEffect = null;
  let scheduled = false;
  let processing = false;
  let destroyed = false;
  let idleWaiters = [];

  function resolveIdleWaiters() {
    if (scheduled || processing || pendingEffect) return;
    const waiters = idleWaiters;
    idleWaiters = [];
    waiters.forEach((resolve) => resolve(state));
  }

  async function drain() {
    if (processing || destroyed) {
      resolveIdleWaiters();
      return;
    }
    processing = true;
    try {
      while (pendingEffect && !destroyed) {
        const current = pendingEffect;
        pendingEffect = null;
        const handler = handlers[current.effect];
        onEffect?.(current);
        if (typeof handler === "function") {
          try {
            await handler(Object.freeze({
              effect: current.effect,
              detail: current.detail,
              state,
              revision: current.revision,
            }));
          } catch (error) {
            onError?.(error, current);
          }
        }
      }
    } finally {
      processing = false;
      if (pendingEffect && !destroyed) scheduleDrain();
      resolveIdleWaiters();
    }
  }

  function scheduleDrain() {
    if (scheduled || processing || destroyed) return;
    scheduled = true;
    schedule(() => {
      scheduled = false;
      void drain();
    });
  }

  function queueEffect(effect, detail = {}) {
    if (!effect || destroyed) return;
    const previousDetail = pendingEffect?.effect === effect ? pendingEffect.detail : null;
    pendingEffect = Object.freeze({
      effect,
      detail: mergeEffectDetail(previousDetail, detail),
      revision: state.revision,
    });
    scheduleDrain();
  }

  function dispatch(event, detail = {}) {
    if (destroyed) return Object.freeze({ changed: false, state, effects: Object.freeze([]) });
    const type = typeof event === "string" ? event : event?.type;
    const transition = policy.reduce(state, event);
    const previous = state;
    state = transition.state;
    onTransition?.(Object.freeze({ type, previous, ...transition, detail }));
    const freshSession = type === VISION_SESSION_EVENT.PAGE_RESTORED;
    transition.effects.forEach((effect) => {
      queueEffect(effect, {
        ...detail,
        event: type,
        freshSession: Boolean(detail.freshSession || freshSession),
      });
    });
    return transition;
  }

  function updateInputs(inputs = {}, detail = {}) {
    if (Object.prototype.hasOwnProperty.call(inputs, "requested")) {
      const requested = Boolean(inputs.requested);
      if (requested !== state.requested) {
        dispatch(requested ? VISION_SESSION_EVENT.USER_START : VISION_SESSION_EVENT.USER_STOP, detail);
      }
    }

    if (Object.prototype.hasOwnProperty.call(inputs, "page")) {
      const page = inputs.page;
      if (page !== state.page || detail.restored === true) {
        dispatch(pageEventFor(page, detail.restored === true), detail);
      }
    }

    if (Object.prototype.hasOwnProperty.call(inputs, "sourceAvailable")) {
      const sourceAvailable = inputs.sourceAvailable !== false;
      if (sourceAvailable !== state.sourceAvailable) {
        dispatch(
          sourceAvailable ? VISION_SESSION_EVENT.SOURCE_AVAILABLE : VISION_SESSION_EVENT.SOURCE_UNAVAILABLE,
          detail,
        );
      }
    }

    if (Object.prototype.hasOwnProperty.call(inputs, "network")) {
      const network = inputs.network;
      const event = networkEventFor(network);
      if (event && network !== state.network) dispatch(event, detail);
    }

    return state;
  }

  function reconcile(detail = {}) {
    queueEffect(EFFECT_FOR_DESIRED[state.desired], {
      ...detail,
      event: "reconcile",
    });
  }

  function whenIdle() {
    if (!scheduled && !processing && !pendingEffect) return Promise.resolve(state);
    return new Promise((resolve) => idleWaiters.push(resolve));
  }

  function destroy() {
    destroyed = true;
    pendingEffect = null;
    resolveIdleWaiters();
  }

  return Object.freeze({
    dispatch,
    updateInputs,
    reconcile,
    whenIdle,
    destroy,
    snapshot: () => state,
    status: () => Object.freeze({
      state,
      scheduled,
      processing,
      pendingEffect: pendingEffect?.effect || null,
      destroyed,
    }),
  });
}

const SESSION_CONTROLLER_API = Object.freeze({
  create: createVisionLifecycleController,
});

export function installDriveVisionSessionControllerFacade(target = globalThis) {
  if (!target) return SESSION_CONTROLLER_API;
  if (target.DriveVisionLifecycleController) return target.DriveVisionLifecycleController;
  target.DriveVisionLifecycleController = SESSION_CONTROLLER_API;
  return target.DriveVisionLifecycleController;
}

export const DriveVisionLifecycleController = SESSION_CONTROLLER_API;
