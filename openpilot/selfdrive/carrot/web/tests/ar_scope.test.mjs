import assert from "node:assert/strict";
import test from "node:test";

import {
  AR_SOURCE,
  describeMarkers,
} from "../src/features/drive/contents/vision/ar/tmap_catalog.js";
import { AR_MARKER_KIND } from "../src/features/drive/contents/vision/ar/tokens.js";
import {
  VISION_AR_AVAILABLE,
  readVisionArEnabled,
} from "../src/features/drive/contents/vision/ar/activation_gate.js";

test("unfinished Vision AR stays unavailable even when a stored setting is true", () => {
  assert.equal(VISION_AR_AVAILABLE, false);
  assert.equal(readVisionArEnabled({ getWebSettingByKey: () => true }), false);
  assert.equal(readVisionArEnabled({
    __CARROT_BOOTSTRAP__: { webSettings: { vision_ar_enabled: true } },
  }), false);
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

test("explicit visible TMap recommendations create one short lane band", () => {
  const markers = describeMarkers({
    laneCurrent: {
      present: true,
      visible: true,
      count: 4,
      distanceM: 60,
      available: [false, true, true, false],
    },
  });

  assert.equal(markers.length, 1);
  assert.equal(markers[0].source, AR_SOURCE.LANE);
  assert.equal(markers[0].kind, AR_MARKER_KIND.LANE_BAND);
  assert.equal(markers[0].laneOffsetM, 0);
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
