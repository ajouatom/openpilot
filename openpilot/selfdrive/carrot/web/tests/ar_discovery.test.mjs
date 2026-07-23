import assert from "node:assert/strict";
import test from "node:test";

import { collectSigns, composeFrame } from "../src/features/drive/contents/vision/ar/compose.js";
import {
  AR_DISCOVERY_POLICY,
  discoveryRangeM,
  isWithinDiscoveryRange,
} from "../src/features/drive/contents/vision/ar/discovery.js";
import { AR_MARKER_KIND, phaseBoundaries, phaseForDistance } from "../src/features/drive/contents/vision/ar/tokens.js";

const LATITUDE = 37.5;
const LONGITUDE = 127;

function longitudeAt(meters) {
  return LONGITUDE + meters / (111320 * Math.cos(LATITUDE * Math.PI / 180));
}

function navi(currentDistanceM, nextDistanceM = null) {
  return {
    sessionId: "discovery-route",
    vehicle: {
      present: true,
      latitude: LATITUDE,
      longitude: LONGITUDE,
      headingDeg: 90,
    },
    route: {
      polyline: [0, 100, 200, 300, 400].map((meters) => ({
        latitude: LATITUDE,
        longitude: longitudeAt(meters),
      })),
    },
    guidanceCurrent: {
      present: true,
      pointValid: false,
      distanceM: currentDistanceM,
      turnType: 12,
      roadName: "조기 좌회전",
    },
    ...(nextDistanceM === null ? {} : {
      guidanceNext: {
        present: true,
        pointValid: false,
        distanceM: nextDistanceM,
        turnType: 13,
        roadName: "다음 우회전",
      },
    }),
  };
}

const modelPosition = Object.freeze({
  x: [0, 20, 40, 60, 80],
  y: [0, 0, 0, 0, 0],
  z: [0, 0, 0, 0, 0],
});

test("discovery range stays between 200m and 300m independently of speed", () => {
  assert.equal(discoveryRangeM(), AR_DISCOVERY_POLICY.defaultRangeM);
  assert.equal(discoveryRangeM(180), 200);
  assert.equal(discoveryRangeM(420), 300);
  assert.equal(isWithinDiscoveryRange(300), true);
  assert.equal(isWithinDiscoveryRange(300.01), false);

  assert.equal(phaseBoundaries(0).preview, 300);
  assert.equal(phaseForDistance(250, 0), "preview");
  assert.equal(phaseForDistance(250, 50), "approach", "visual emphasis remains speed-aware");
  assert.equal(phaseForDistance(301, 50), "passed");
});

test("a 250m route maneuver is discovered as one low-confidence preview cue", () => {
  const composed = composeFrame({
    navi: navi(250),
    naviUsable: true,
    geoAllowed: false,
    routeAllowed: true,
    canDrawPrecise: true,
    egoSpeedMps: 0,
    modelPosition,
  });

  assert.equal(composed.signs.length, 1, "far commit arrow must not duplicate the preview sign");
  assert.equal(composed.fresh.length, 1);
  const marker = composed.fresh[0];
  assert.equal(marker.descriptor.kind, AR_MARKER_KIND.TURN_GATE);
  assert.equal(marker.descriptor.phase, "preview");
  assert.equal(marker.confidence, "low");
  assert.equal(marker.discovery.rangeM, 300);
  assert.equal(marker.discovery.early, true);
  assert.equal(marker.anchor.routeDerived, true);
  assert.ok(Math.abs(marker.anchor.x - 250) < 0.5);
});

test("discovery never invents a 250m world point without route or geo coverage", () => {
  const composed = composeFrame({
    navi: { ...navi(250), route: null },
    naviUsable: true,
    geoAllowed: false,
    routeAllowed: false,
    canDrawPrecise: true,
    egoSpeedMps: 0,
    modelPosition,
  });

  assert.equal(composed.signs.length, 1, "the semantic cue remains available to diagnostics/HUD");
  assert.equal(composed.fresh?.length || 0, 0, "no route/model coverage means no fabricated world anchor");
});

test("events beyond 300m are not inserted into the world-marker lifecycle", () => {
  const signs = collectSigns({
    navi: navi(301),
    naviUsable: true,
    egoSpeedMps: 40,
  });
  assert.deepEqual(signs, []);
});

test("current guidance and a distant next maneuver coexist with explicit hierarchy", () => {
  const signs = collectSigns({
    navi: navi(40, 250),
    naviUsable: true,
    egoSpeedMps: 0,
  });
  const current = signs.filter((item) => item.source === "guidanceCurrent");
  const next = signs.find((item) => item.source === "guidanceNext");

  assert.equal(current.length, 2, "near current gate keeps its commit arrow");
  assert.ok(next);
  assert.equal(next.descriptor.phase, "preview");
  assert.equal(next.confidence, "low");
  assert.equal(current[0].confidence, "high");
});
