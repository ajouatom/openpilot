import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";
import vm from "node:vm";
import { fileURLToPath } from "node:url";

import "../js/realtime/raw_capnp.js";

// A single-segment cereal.Event carrying CarrotNaviState. The payload was
// serialized from the checked-in custom.capnp schema, so this catches wire
// discriminant, physical slot, nested-struct, and list-layout regressions.
const CARROT_NAVI_EVENT_BASE64 = [
  "AAAAAHEAAAAAAAAAAgABAAgamb4cAAAAagAAAAAAAAAAAAAAAwALAAEAAQAAAAAAKgAAAAAAAAAgFpm+HAAAACkAAAB6AAAALAAAAAQAAgBYAAAABAAGAJwAAAAEAAYA0AAAAAMABAAAAAAAAAAAAPgAAAAKAAEAIAEAAAMAAgAwAQAAAgACADwBAAADAAIAcAEAAAEAAgByZXBsYXktc2Vzc2lvbgAAbXzcbc2PQkDAM+vW/PBfQAAAr0IAAFlCAAAAAAAAAAAEAAAABAAAABEAAABSAAAAAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB0ZXN0LXJvYWQAAAAAAAAAxQAAAB8AAAANAAAAAQAAAB1aZDvfj0JAGy/dJAbxX0AUAAAABAAAACEAAAByAAAAJQAAAFoAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAGdhbmduYW0tZGFlcm8AAAB0dXJuLXJpZ2h0AAAAAAAAIAMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAUAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAMAAQB4AAAAAAAAAAAAAAAAAAAAAAAAAAwAAAAEAAAAAAAAAAAAAAAAAAAAAAAAABEAAAAbAAAAAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAEAAQAAAAAAAAAPADwAAAAAAPoAAAAyAFAAAACTQgAAAAAAUJxEAAAAAAAAwD4AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABjAAAAXwAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAQAAANIAAAAHAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAApCQAAQQEAAAAAAAAAAAAAECcAAAAAAAAEAAAABAAAABEAAAAnAAAAAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAIAAAAAgAAAG183G3Nj0JAwDPr1vzwX0AdWmQ7349CQBsv3SQG8V9ABQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
].join("");

const webRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");

function fixtureBytes() {
  return Uint8Array.from(Buffer.from(CARROT_NAVI_EVENT_BASE64, "base64"));
}

test("recorded carrotNavi Event decodes to the live compact-state shape", () => {
  const rawCapnp = globalThis.CarrotRawCapnp;
  assert.ok(rawCapnp);

  const bytes = fixtureBytes();
  const event = rawCapnp.decodeReplayEvent(bytes);

  assert.equal(event.service, "carrotNavi");
  assert.equal(event.logMonoTime, 123_456_789_000);
  assert.equal(event.valid, true);
  assert.equal(event.decoded.schemaVersion, 1);
  assert.equal(event.decoded.generation, 42);
  assert.equal(event.decoded.sessionId, "replay-session");
  assert.equal(event.decoded.publishMonoTimeNanos, 123_456_788_000);
  assert.equal(event.decoded.connected, true);

  assert.equal(event.decoded.vehicle.present, true);
  assert.equal(event.decoded.vehicle.roadName, "test-road");
  assert.equal(event.decoded.vehicle.headingDeg, 87.5);
  assert.ok(Math.abs(event.decoded.vehicle.latitude - 37.1234567) < 1e-7);

  assert.equal(event.decoded.guidanceCurrent.present, true);
  assert.equal(event.decoded.guidanceCurrent.distanceM, 197);
  assert.equal(event.decoded.guidanceCurrent.turnType, 13);
  assert.equal(event.decoded.guidanceCurrent.mainText, "turn-right");
  assert.equal(event.decoded.guidanceCurrent.pointValid, true);
  assert.equal(event.decoded.guidanceNext.present, false);

  assert.equal(event.decoded.laneCurrent.present, true);
  assert.equal(event.decoded.laneCurrent.visible, true);
  assert.deepEqual(event.decoded.laneCurrent.available, [0, 1, 1]);

  assert.equal(event.decoded.speed.roadLimitKph, 60);
  assert.equal(event.decoded.speed.sdiDistanceM, 250);
  assert.equal(event.decoded.speed.sectionActive, true);
  assert.equal(event.decoded.speed.sectionAverageKph, 73.5);
  assert.equal(event.decoded.speed.sectionRemainingDistanceM, 1250.5);
  assert.equal(event.decoded.speed.sectionProgress, 0.375);

  assert.equal(event.decoded.trafficSignal.visible, true);
  assert.equal(event.decoded.trafficSignal.greenOn, true);
  assert.equal(event.decoded.crossroad.imageCode, 7);
  assert.equal(event.decoded.route.present, true);
  assert.equal(event.decoded.route.totalDistanceM, 10_000);
  assert.equal(event.decoded.route.polyline.length, 2);
  assert.ok(Math.abs(event.decoded.route.polyline[1].longitude - 127.766) < 1e-9);
  assert.deepEqual(event.decoded.navigationStatus, {
    guidanceActive: true,
    offRoute: false,
    routePresent: true,
  });
});

test("replay log builder retains carrotNavi in its browser timeline", () => {
  const workerSource = fs.readFileSync(
    path.join(webRoot, "js", "realtime", "replay_log_worker.js"),
    "utf8",
  );
  const context = {
    AbortController,
    ArrayBuffer,
    DataView,
    Map,
    Set,
    TextDecoder,
    Uint8Array,
    clearTimeout,
    console,
    fetch: globalThis.fetch,
    setTimeout,
    CarrotRawCapnp: globalThis.CarrotRawCapnp,
    fixture: fixtureBytes(),
    fzstd: {},
    importScripts() {},
    postMessage() {},
  };
  context.self = context;
  vm.createContext(context);
  vm.runInContext(workerSource, context, { filename: "replay_log_worker.js" });

  const timeline = vm.runInContext(`(() => {
    const builder = new ReplayLogBuilder(1);
    builder.push(fixture);
    return builder.finish();
  })()`, context);

  assert.equal(timeline.metadata.services.carrotNavi, 1);
  assert.equal(timeline.records.length, 1);
  assert.equal(timeline.records[0].timeMs, 0);
  assert.equal(timeline.records[0].frames.length, 1);
  assert.equal(timeline.records[0].frames[0].service, "carrotNavi");
  assert.equal(timeline.records[0].frames[0].decoded.guidanceCurrent.distanceM, 197);
  assert.equal(timeline.metadata.eventIndexVersion, 2);
  assert.ok(timeline.metadata.eventIndex.some((event) => (
    event.type === "navigation_maneuver_current" && event.sourceTag === "CarrotNavi"
  )));
});

test("replay event index tags CarrotNavi and suppresses duplicate CarrotMan guidance", () => {
  const workerSource = fs.readFileSync(
    path.join(webRoot, "js", "realtime", "replay_log_worker.js"),
    "utf8",
  );
  const context = {
    AbortController,
    ArrayBuffer,
    DataView,
    Map,
    Set,
    TextDecoder,
    Uint8Array,
    clearTimeout,
    console,
    fetch: globalThis.fetch,
    setTimeout,
    CarrotRawCapnp: globalThis.CarrotRawCapnp,
    fzstd: {},
    importScripts() {},
    postMessage() {},
  };
  context.self = context;
  vm.createContext(context);
  vm.runInContext(workerSource, context, { filename: "replay_log_worker.js" });

  const events = vm.runInContext(`(() => {
    const indexer = new ReplayEventIndexer();
    indexer.ingest({
      service: "carrotMan",
      logMonoTime: 1000000000,
      decoded: { xTurnInfo: 1, xDistToTurn: 240, szTBTMainText: "legacy turn" },
    });
    const navi = {
      service: "carrotNavi",
      logMonoTime: 1100000000,
      decoded: {
        connected: true,
        sessionId: "route-1",
        generation: 1,
        navigationStatus: { guidanceActive: true, routePresent: true, offRoute: false },
        guidanceCurrent: {
          present: true,
          distanceM: 230,
          turnType: 13,
          mainText: "right turn",
          roadName: "test road",
          pointValid: false,
        },
      },
    };
    indexer.ingest(navi);
    indexer.ingest({
      ...navi,
      logMonoTime: 1200000000,
      decoded: { ...navi.decoded, generation: 2 },
    });
    return indexer.finalize(1000000000, 1000);
  })()`, context);

  assert.equal(events.some((event) => event.type === "navigation_maneuver"), false);
  const current = events.find((event) => event.type === "navigation_maneuver_current");
  assert.equal(current.sourceTag, "CarrotNavi");
  assert.equal(current.sourceTitle, "right turn");
  assert.equal(events.some((event) => event.type === "navigation_session_changed"), false);
});

test("replay timeline seeds nearby CarrotNavi state at video start", () => {
  const workerSource = fs.readFileSync(
    path.join(webRoot, "js", "realtime", "replay_log_worker.js"),
    "utf8",
  );
  const context = {
    AbortController,
    ArrayBuffer,
    DataView,
    Map,
    Set,
    TextDecoder,
    Uint8Array,
    clearTimeout,
    console,
    fetch: globalThis.fetch,
    setTimeout,
    CarrotRawCapnp: globalThis.CarrotRawCapnp,
    fzstd: {},
    importScripts() {},
    postMessage() {},
  };
  context.self = context;
  vm.createContext(context);
  vm.runInContext(workerSource, context, { filename: "replay_log_worker.js" });

  const timeline = vm.runInContext(`(() => {
    const builder = new ReplayLogBuilder(1);
    builder.cameraIndexes.push({
      frameId: 1,
      segmentNum: 1,
      segmentId: 1,
      logMonoTime: 1000000000,
      timestampSof: 1000000000,
    });
    builder.selected.get("carrotNavi").set(2, {
      service: "carrotNavi",
      logMonoTime: 1400000000,
      decoded: { connected: true, sessionId: "seed-route" },
    });
    return builder.finish();
  })()`, context);

  assert.deepEqual(Array.from(timeline.metadata.preRollServices), ["carrotNavi"]);
  assert.equal(timeline.metadata.preRollMode, "bounded-lookahead");
  assert.equal(timeline.metadata.preRollSourceTimeMs, 400);
  assert.equal(timeline.records[0].timeMs, 0);
  assert.equal(timeline.records[0].frames[0].service, "carrotNavi");
  assert.equal(timeline.records[0].frames[0].decoded.sessionId, "seed-route");
  assert.equal(timeline.records.at(-1).timeMs, 400);
});

test("replay selection preserves every frame-critical geometry sample", () => {
  const workerSource = fs.readFileSync(
    path.join(webRoot, "js", "realtime", "replay_log_worker.js"),
    "utf8",
  );
  const context = {
    AbortController,
    ArrayBuffer,
    DataView,
    Map,
    Set,
    TextDecoder,
    Uint8Array,
    clearTimeout,
    console,
    fetch: globalThis.fetch,
    setTimeout,
    CarrotRawCapnp: globalThis.CarrotRawCapnp,
    fzstd: {},
    importScripts() {},
    postMessage() {},
  };
  context.self = context;
  vm.createContext(context);
  vm.runInContext(workerSource, context, { filename: "replay_log_worker.js" });

  const keys = vm.runInContext(`[
    replaySelectionKey("modelV2", 100000001),
    replaySelectionKey("modelV2", 100000002),
    replaySelectionKey("lateralPlan", 100000003),
    replaySelectionKey("cameraOdometry", 100000004),
    replaySelectionKey("carState", 100000001),
    replaySelectionKey("carState", 100000002),
  ]`, context);

  assert.deepEqual(Array.from(keys), [
    100000001,
    100000002,
    100000003,
    100000004,
    3,
    3,
  ]);
});
