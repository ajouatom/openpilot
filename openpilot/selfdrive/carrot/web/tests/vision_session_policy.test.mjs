import assert from "node:assert/strict";
import test from "node:test";

import {
  VISION_SESSION_DESIRED,
  VISION_SESSION_EFFECT,
  VISION_SESSION_EVENT,
  VISION_SESSION_NETWORK,
  VISION_SESSION_PAGE,
  VISION_SESSION_PHASE,
  createVisionSessionState,
  reduceVisionSessionState,
} from "../src/features/drive/contents/vision/session_policy.js";

function apply(initial, ...events) {
  let state = initial;
  const effects = [];
  for (const event of events) {
    const transition = reduceVisionSessionState(state, event);
    state = transition.state;
    effects.push(...transition.effects);
  }
  return { state, effects };
}

test("session policy starts inactive with a usable foreground source", () => {
  const state = createVisionSessionState();
  assert.equal(state.requested, false);
  assert.equal(state.page, VISION_SESSION_PAGE.VISIBLE);
  assert.equal(state.network, VISION_SESSION_NETWORK.UNKNOWN);
  assert.equal(state.desired, VISION_SESSION_DESIRED.INACTIVE);
  assert.equal(state.phase, VISION_SESSION_PHASE.IDLE);
});

test("duplicate foreground lifecycle events produce one resume connect", () => {
  const started = apply(
    createVisionSessionState(),
    VISION_SESSION_EVENT.USER_START,
    VISION_SESSION_EVENT.CONNECT_BEGIN,
    VISION_SESSION_EVENT.CONNECT_SUCCESS,
  );
  const backgrounded = apply(
    started.state,
    VISION_SESSION_EVENT.PAGE_HIDDEN,
    VISION_SESSION_EVENT.PAGE_FROZEN,
    VISION_SESSION_EVENT.DISCONNECT_COMPLETE,
  );
  const resumed = apply(
    backgrounded.state,
    VISION_SESSION_EVENT.PAGE_VISIBLE,
    VISION_SESSION_EVENT.PAGE_RESTORED,
    VISION_SESSION_EVENT.WINDOW_FOCUS,
  );

  assert.deepEqual(backgrounded.effects, [VISION_SESSION_EFFECT.ENSURE_SUSPENDED]);
  assert.deepEqual(resumed.effects, [VISION_SESSION_EFFECT.ENSURE_RUNNING]);
  assert.equal(resumed.state.desired, VISION_SESSION_DESIRED.RUNNING);
});

test("backgrounding an in-flight connection suspends it once", () => {
  const connecting = apply(
    createVisionSessionState(),
    VISION_SESSION_EVENT.USER_START,
    VISION_SESSION_EVENT.CONNECT_BEGIN,
  );
  const suspended = apply(
    connecting.state,
    VISION_SESSION_EVENT.PAGE_HIDDEN,
    VISION_SESSION_EVENT.PAGE_FROZEN,
  );

  assert.equal(suspended.state.desired, VISION_SESSION_DESIRED.SUSPENDED);
  assert.equal(suspended.state.phase, VISION_SESSION_PHASE.CONNECTING);
  assert.deepEqual(suspended.effects, [VISION_SESSION_EFFECT.ENSURE_SUSPENDED]);
});

test("a stale connect success cannot reactivate a suspended page", () => {
  const suspended = createVisionSessionState({
    requested: true,
    page: VISION_SESSION_PAGE.HIDDEN,
    phase: VISION_SESSION_PHASE.CONNECTING,
  });
  const transition = reduceVisionSessionState(suspended, VISION_SESSION_EVENT.CONNECT_SUCCESS);

  assert.equal(transition.state.desired, VISION_SESSION_DESIRED.SUSPENDED);
  assert.equal(transition.state.phase, VISION_SESSION_PHASE.CONNECTED);
  assert.deepEqual(transition.effects, [VISION_SESSION_EFFECT.ENSURE_SUSPENDED]);
});

test("stopping while connecting rejects a late successful connection", () => {
  const connecting = createVisionSessionState({
    requested: true,
    phase: VISION_SESSION_PHASE.CONNECTING,
  });
  const stopped = reduceVisionSessionState(connecting, VISION_SESSION_EVENT.USER_STOP);
  const staleSuccess = reduceVisionSessionState(stopped.state, VISION_SESSION_EVENT.CONNECT_SUCCESS);

  assert.deepEqual(stopped.effects, [VISION_SESSION_EFFECT.ENSURE_STOPPED]);
  assert.equal(staleSuccess.state.desired, VISION_SESSION_DESIRED.INACTIVE);
  assert.deepEqual(staleSuccess.effects, [VISION_SESSION_EFFECT.ENSURE_STOPPED]);
});

test("network reachability loss and recovery serialize suspend and connect", () => {
  const running = createVisionSessionState({
    requested: true,
    network: VISION_SESSION_NETWORK.REACHABLE,
    phase: VISION_SESSION_PHASE.CONNECTED,
  });
  const result = apply(
    running,
    VISION_SESSION_EVENT.NETWORK_UNREACHABLE,
    VISION_SESSION_EVENT.NETWORK_UNREACHABLE,
    VISION_SESSION_EVENT.NETWORK_REACHABLE,
    VISION_SESSION_EVENT.NETWORK_REACHABLE,
  );

  assert.deepEqual(result.effects, [
    VISION_SESSION_EFFECT.ENSURE_SUSPENDED,
    VISION_SESSION_EFFECT.ENSURE_RUNNING,
  ]);
  assert.equal(result.state.desired, VISION_SESSION_DESIRED.RUNNING);
});

test("source availability gates connection without clearing the user request", () => {
  const running = createVisionSessionState({
    requested: true,
    phase: VISION_SESSION_PHASE.CONNECTED,
  });
  const unavailable = reduceVisionSessionState(running, VISION_SESSION_EVENT.SOURCE_UNAVAILABLE);
  const available = reduceVisionSessionState(unavailable.state, VISION_SESSION_EVENT.SOURCE_AVAILABLE);

  assert.equal(unavailable.state.requested, true);
  assert.equal(unavailable.state.desired, VISION_SESSION_DESIRED.WAITING);
  assert.deepEqual(unavailable.effects, [VISION_SESSION_EFFECT.ENSURE_SUSPENDED]);
  assert.deepEqual(available.effects, [VISION_SESSION_EFFECT.ENSURE_RUNNING]);
});

test("connection failure requests recovery only while running is desired", () => {
  const running = createVisionSessionState({
    requested: true,
    phase: VISION_SESSION_PHASE.CONNECTING,
  });
  const failed = reduceVisionSessionState(running, VISION_SESSION_EVENT.CONNECT_FAILURE);
  assert.equal(failed.state.phase, VISION_SESSION_PHASE.RECOVERING);
  assert.deepEqual(failed.effects, [VISION_SESSION_EFFECT.SCHEDULE_RECOVERY]);

  const stopped = createVisionSessionState({
    requested: false,
    phase: VISION_SESSION_PHASE.CONNECTING,
  });
  const ignored = reduceVisionSessionState(stopped, VISION_SESSION_EVENT.CONNECT_FAILURE);
  assert.equal(ignored.state.phase, VISION_SESSION_PHASE.IDLE);
  assert.deepEqual(ignored.effects, []);
});

test("ownership release reconnects only an active requested session", () => {
  const blocked = createVisionSessionState({
    requested: true,
    phase: VISION_SESSION_PHASE.BLOCKED,
  });
  const released = reduceVisionSessionState(blocked, VISION_SESSION_EVENT.OWNERSHIP_RELEASED);
  assert.equal(released.state.phase, VISION_SESSION_PHASE.IDLE);
  assert.deepEqual(released.effects, [VISION_SESSION_EFFECT.ENSURE_RUNNING]);
});

test("unknown lifecycle events fail loudly", () => {
  assert.throws(
    () => reduceVisionSessionState(createVisionSessionState(), "page-maybe"),
    /Unknown Carrot Vision session event/,
  );
});
