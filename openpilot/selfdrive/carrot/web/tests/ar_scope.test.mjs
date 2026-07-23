import assert from "node:assert/strict";
import test from "node:test";

import {
  AR_NAV_SOURCE_QUALITY,
  AR_SOURCE,
  classifyTurnType,
  describeMarkers,
} from "../src/features/drive/contents/vision/ar/tmap_catalog.js";
import { signboardFromMarker } from "../src/features/drive/contents/vision/ar/signboard.js";
import {
  AR_MARKER_KIND,
  AR_PRODUCT_MARKERS,
} from "../src/features/drive/contents/vision/ar/tokens.js";
import {
  VISION_AR_AVAILABLE,
  readVisionArEnabled,
} from "../src/features/drive/contents/vision/ar/activation_gate.js";

test("Vision AR follows the explicit web setting when the feature is available", () => {
  assert.equal(VISION_AR_AVAILABLE, true);
  assert.equal(readVisionArEnabled({ getWebSettingByKey: () => false }), false);
  assert.equal(readVisionArEnabled({ getWebSettingByKey: () => true }), true);
  assert.equal(readVisionArEnabled({
    __CARROT_BOOTSTRAP__: { webSettings: { vision_ar_enabled: true } },
  }), true);

  const gatedTarget = {
    getWebSettingByKey: () => true,
    CarrotWebSettingsSpec: [{
      key: "vision_ar_enabled",
      requiresCapability: "web_lab",
    }],
    CarrotWebCapabilitiesState: { web_lab: false },
  };
  assert.equal(readVisionArEnabled(gatedTarget), false);
  gatedTarget.CarrotWebCapabilitiesState.web_lab = true;
  assert.equal(readVisionArEnabled(gatedTarget), true);
});

test("route polyline supports placement but never creates a route ribbon marker", () => {
  const markers = describeMarkers({
    route: {
      polyline: [
        { latitude: 37.5, longitude: 127.0 },
        { latitude: 37.501, longitude: 127.001 },
      ],
    },
  });

  assert.deepEqual(markers, []);
  assert.equal(Object.hasOwn(AR_MARKER_KIND, "ROUTE_RIBBON"), false);
});

test("product policy withholds the non-intuitive lane band from driver view", () => {
  const markers = describeMarkers({
    laneCurrent: {
      present: true,
      visible: true,
      count: 4,
      distanceM: 60,
      available: [false, true, true, false],
    },
  });

  assert.equal(AR_PRODUCT_MARKERS.laneBand, false);
  assert.deepEqual(markers, []);
});

test("laneAhead recommendations also stay hidden while the product policy is off", () => {
  const markers = describeMarkers({
    laneCurrent: { present: false },
    laneAhead: [
      { present: true, visible: true, count: 3, distanceM: 140, available: [false, true, false] },
      { present: true, visible: true, count: 3, distanceM: 80, available: [true, false, false] },
    ],
  });

  assert.deepEqual(markers, []);
});

test("missing or hidden lane recommendations do not infer a lane band", () => {
  const hidden = describeMarkers({
    laneCurrent: {
      present: true,
      visible: false,
      count: 3,
      distanceM: 60,
      available: [false, true, false],
    },
  });
  const empty = describeMarkers({
    laneCurrent: {
      present: true,
      visible: true,
      count: 3,
      distanceM: 60,
      available: [false, false, false],
    },
  });

  assert.deepEqual(hidden, []);
  assert.deepEqual(empty, []);
});

test("persistent road limit state never creates a synthetic world marker", () => {
  const markers = describeMarkers({
    speed: {
      roadLimitValid: true,
      roadLimitKph: 60,
      sdiPresent: false,
    },
  });

  assert.deepEqual(markers, []);
  assert.equal(Object.hasOwn(AR_SOURCE, "SPEED_LIMIT"), false);
});

test("turn table covers left, right, U-turn, fork, ramp, rotary, straight and destination semantics", () => {
  const cases = [
    [12, "turn", "left"], [13, "turn", "right"], [14, "uturn", "uturn"],
    [1002, "fork", "slight_left"], [1007, "ramp", "right"],
    [142, "rotary", "straight"], [201, "arrive", "straight"],
  ];
  for (const [code, family, direction] of cases) {
    const turn = classifyTurnType(code);
    assert.equal(turn.family, family, `family for ${code}`);
    assert.equal(turn.direction, direction, `direction for ${code}`);
  }
  assert.equal(classifyTurnType(999999).family, "unknown");
});

test("straight guidance never creates a persistent AR gate", () => {
  const markers = describeMarkers({
    guidanceCurrent: { present: true, distanceM: 180, turnType: 142, roadName: "직진 도로" },
    guidanceNext: { present: true, distanceM: 300, turnType: 201, mainText: "목적지" },
  });

  assert.equal(markers.some((marker) => marker.kind === AR_MARKER_KIND.TURN_GATE), false);
  assert.equal(markers.some((marker) => marker.kind === AR_MARKER_KIND.COMMIT_ARROW), false);
  assert.equal(markers.filter((marker) => marker.kind === AR_MARKER_KIND.DESTINATION_PIN).length, 1);
});

test("distance-bearing Navi meanings map to distinct product markers without duplicates", () => {
  const markers = describeMarkers({
    guidanceCurrent: { present: true, distanceM: 120, turnType: 13, roadName: "테헤란로" },
    guidanceNext: { present: true, distanceM: 260, turnType: 201, mainText: "목적지" },
    speed: {
      sdiPresent: true, sdiType: 1, sdiDistanceM: 180, sdiSpeedLimitKph: 60,
      secondarySdiPresent: true, secondarySdiType: 22, secondarySdiDistanceM: 90,
      sectionPresent: true, sectionActive: true, sectionSpeedLimitKph: 80,
      sectionAverageKph: 72.4, sectionRemainingDistanceM: 900,
      sectionRemainingTimeSec: 42, sectionProgress: 0.25,
    },
    trafficSignal: {
      visible: true, distanceM: 70,
      redValid: true, redOn: true, redRemainSec: 13,
      uiCounterValid: true, uiCounterRemainSec: 13,
    },
    crossroad: { visible: true, distanceM: 55, imageCode: 7 },
  });

  const sources = markers.map((marker) => marker.source);
  assert.ok(sources.includes(AR_SOURCE.GUIDANCE_CURRENT));
  assert.ok(markers.some((marker) => marker.kind === AR_MARKER_KIND.DESTINATION_PIN));
  assert.equal(markers.filter((marker) => marker.source === AR_SOURCE.SDI).length, 1);
  assert.equal(markers.filter((marker) => marker.source === AR_SOURCE.SDI_SECONDARY).length, 1);
  assert.equal(markers.filter((marker) => marker.source === AR_SOURCE.SECTION).length, 1);
  assert.equal(markers.filter((marker) => marker.source === AR_SOURCE.TRAFFIC_SIGNAL).length, 1);
  assert.equal(markers.filter((marker) => marker.source === AR_SOURCE.CROSSROAD).length, 1);

  const signal = markers.find((marker) => marker.source === AR_SOURCE.TRAFFIC_SIGNAL);
  const descriptor = signboardFromMarker(signal);
  assert.equal(descriptor.primary, "정지");
  assert.equal(descriptor.secondary, "13초");
});

test("section SDI does not duplicate its section gate and off-route section is hidden", () => {
  const base = {
    sdiPresent: true, sdiType: 2, sdiDistanceM: 100,
    sectionPresent: true, sectionRemainingDistanceM: 800,
  };
  const active = describeMarkers({ speed: base });
  assert.equal(active.filter((marker) => marker.kind === AR_MARKER_KIND.CAUTION_SIGN).length, 0);
  assert.equal(active.filter((marker) => marker.kind === AR_MARKER_KIND.SECTION_GATE).length, 1);

  const offRoute = describeMarkers({ speed: { ...base, sectionOffRoute: true } });
  assert.deepEqual(offRoute, []);
});

test("CarrotMan and stock navigation stay HUD-only when world placement is unavailable", () => {
  assert.equal(AR_NAV_SOURCE_QUALITY.CARROT_NAVI, "world-anchor");
  assert.equal(AR_NAV_SOURCE_QUALITY.CARROT_MAN, "hud-only");
  assert.equal(AR_NAV_SOURCE_QUALITY.STOCK_NAVI, "hud-only");
});
