import assert from "node:assert/strict";
import test from "node:test";

import {
  DASHCAM_SEGMENT_READ_STATE,
  createDashcamPlayerSession,
  createDashcamReadStateStore,
  dashcamSegmentReadState,
} from "../src/features/logs/dashcam_player_session.js";

const SEGMENTS = ["route--0", "route--1", "route--2", "route--3"];

test("dashcam player session keeps reading and recent states mutually exclusive", () => {
  const session = createDashcamPlayerSession({
    group: { route: "route", dateLabel: "2026.07.23", timeRange: "08:00–08:04", segmentCount: 4 },
    segments: SEGMENTS,
    activeSegment: "route--2",
    previousSegment: "route--1",
  });
  const state = session.snapshot();

  assert.equal(session.statusFor("route--2"), DASHCAM_SEGMENT_READ_STATE.READING);
  assert.equal(session.statusFor("route--1"), DASHCAM_SEGMENT_READ_STATE.RECENT);
  assert.equal(session.statusFor("route--0"), "");
  const states = SEGMENTS.map((segment) => session.statusFor(segment));
  assert.equal(states.filter((value) => value === DASHCAM_SEGMENT_READ_STATE.READING).length, 1);
  assert.equal(states.filter((value) => value === DASHCAM_SEGMENT_READ_STATE.RECENT).length, 1);
  assert.equal(state.activeIndex, 2);
  assert.equal(state.previousIndex, 1);
  assert.equal(state.group.segmentCount, 4);
});

test("select moves the recent marker to the segment that was just left", () => {
  const session = createDashcamPlayerSession({ segments: SEGMENTS, activeSegment: "route--1" });

  const changed = session.select("route--3");
  assert.equal(changed.changed, true);
  assert.equal(changed.state.activeSegment, "route--3");
  assert.equal(changed.state.previousSegment, "route--1");

  const unchanged = session.select("route--3");
  assert.equal(unchanged.changed, false);
  assert.equal(unchanged.state.previousSegment, "route--1");
});

test("previous and next navigation share the same reading-state transition", () => {
  const session = createDashcamPlayerSession({ segments: SEGMENTS, activeSegment: "route--1" });

  const next = session.move(1);
  assert.equal(next.state.activeSegment, "route--2");
  assert.equal(next.state.previousSegment, "route--1");

  const previous = session.move(-1);
  assert.equal(previous.state.activeSegment, "route--1");
  assert.equal(previous.state.previousSegment, "route--2");

  session.move(-1);
  const atStart = session.move(-1);
  assert.equal(atStart.changed, false);
  assert.equal(atStart.state.activeSegment, "route--0");
  assert.equal(atStart.state.previousSegment, "route--1");
});

test("segment refresh preserves valid reading state and clears stale recent state", () => {
  const session = createDashcamPlayerSession({
    segments: SEGMENTS,
    activeSegment: "route--2",
    previousSegment: "route--1",
  });

  const refreshed = session.replaceSegments(["route--0", "route--2", "route--3"]);
  assert.equal(refreshed.state.activeSegment, "route--2");
  assert.equal(refreshed.state.previousSegment, "");
  assert.deepEqual(refreshed.state.segments, ["route--0", "route--2", "route--3"]);
});

test("read-state helper never labels the active segment as recent", () => {
  const state = { activeSegment: "route--1", previousSegment: "route--1" };
  assert.equal(dashcamSegmentReadState(state, "route--1"), DASHCAM_SEGMENT_READ_STATE.READING);
});

test("shared read-state store keeps one reading and one recent segment across routes", () => {
  const store = createDashcamReadStateStore();
  const changes = [];
  const unsubscribe = store.subscribe((state) => changes.push(state));

  store.select("route-a--1");
  store.select("route-b--2");

  assert.equal(store.statusFor("route-b--2"), DASHCAM_SEGMENT_READ_STATE.READING);
  assert.equal(store.statusFor("route-a--1"), DASHCAM_SEGMENT_READ_STATE.RECENT);
  assert.equal(store.statusFor("route-a--0"), "");
  assert.equal(changes.length, 2);
  assert.deepEqual(store.snapshot(), {
    activeSegment: "route-b--2",
    previousSegment: "route-a--1",
  });

  unsubscribe();
  store.select("route-b--3");
  assert.equal(changes.length, 2);
});

test("server-restored recent state remains until the first live transition", () => {
  const store = createDashcamReadStateStore();

  store.restoreRecent("route-a--1");
  store.select("route-b--2");
  assert.equal(store.statusFor("route-a--1"), DASHCAM_SEGMENT_READ_STATE.RECENT);
  assert.equal(store.statusFor("route-b--2"), DASHCAM_SEGMENT_READ_STATE.READING);

  store.select("route-b--3");
  assert.equal(store.statusFor("route-b--2"), DASHCAM_SEGMENT_READ_STATE.RECENT);
  assert.equal(store.statusFor("route-a--1"), "");
});

test("closing the player clears reading and promotes it to the only recent segment", () => {
  const store = createDashcamReadStateStore({
    activeSegment: "route-b--2",
    previousSegment: "route-a--1",
  });

  const closed = store.finish();
  assert.equal(closed.changed, true);
  assert.equal(store.statusFor("route-b--2"), DASHCAM_SEGMENT_READ_STATE.RECENT);
  assert.equal(store.statusFor("route-a--1"), "");
  assert.deepEqual(store.snapshot(), {
    activeSegment: "",
    previousSegment: "route-b--2",
  });
});
