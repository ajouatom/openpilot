import assert from "node:assert/strict";
import test from "node:test";

import { composeFrame } from "../src/features/drive/contents/vision/ar/compose.js";
import { routeDistanceAnchor, routeTangentHeading } from "../src/features/drive/contents/vision/ar/geo.js";
import { evaluateGeoPositionQuality } from "../src/features/drive/contents/vision/ar/position_quality.js";
import { AR_MARKER_KIND } from "../src/features/drive/contents/vision/ar/tokens.js";

const closeTo = (actual, expected, tolerance = 1e-9) => {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
};

function qualityInput(overrides = {}) {
  return {
    nowMs: 10_000,
    receivedAtMonotonic: { gpsLocationExternal: 9_950 },
    naviUsable: true,
    naviVehicle: {
      present: true,
      latitude: 37.5,
      longitude: 127.0,
      headingDeg: 90,
      speedKph: 36,
    },
    gpsLocationExternal: {
      hasFix: true,
      latitude: 37.5,
      longitude: 127.00005,
      speed: 10,
      bearingDeg: 92,
      bearingAccuracyDeg: 3,
      horizontalAccuracy: 3,
    },
    ...overrides,
  };
}

test("fresh consistent TMap and Comma positions allow a geo anchor", () => {
  const quality = evaluateGeoPositionQuality(qualityInput());

  assert.equal(quality.canUseGeo, true);
  assert.equal(quality.fallback, null);
  assert.ok(quality.separationM > 4 && quality.separationM < 5);
  assert.equal(quality.headingDisagreementDeg, 2);
  closeTo(quality.positionSigmaM, Math.hypot(3, quality.separationM * 0.5));
  closeTo(quality.headingSigmaDeg, Math.hypot(3, 1));
});

test("poor horizontal accuracy degrades to model-path placement", () => {
  const input = qualityInput();
  input.gpsLocationExternal.horizontalAccuracy = 45;
  const quality = evaluateGeoPositionQuality(input);

  assert.equal(quality.canUseGeo, false);
  assert.equal(quality.fallback, "model-path");
  assert.match(quality.reasons.join(" | "), /수평 오차 45.0m/);
});

test("stale Comma GPS cannot approve a fresh TMap geo anchor", () => {
  const input = qualityInput();
  input.receivedAtMonotonic.gpsLocationExternal = 4_000;
  const quality = evaluateGeoPositionQuality(input);

  assert.equal(quality.canUseGeo, false);
  assert.equal(quality.gpsAgeMs, 6_000);
  assert.match(quality.reasons.join(" | "), /Comma GPS age 6000ms/);
});

test("large cross-source position disagreement rejects geo placement", () => {
  const input = qualityInput();
  input.gpsLocationExternal.latitude = 37.501;
  const quality = evaluateGeoPositionQuality(input);

  assert.equal(quality.canUseGeo, false);
  assert.ok(quality.separationM > 100);
  assert.match(quality.reasons.join(" | "), /위치 불일치/);
});

test("moving heading disagreement rejects geo placement", () => {
  const input = qualityInput();
  input.gpsLocationExternal.bearingDeg = 180;
  const quality = evaluateGeoPositionQuality(input);

  assert.equal(quality.canUseGeo, false);
  assert.equal(quality.headingDisagreementDeg, 90);
  assert.match(quality.reasons.join(" | "), /방향 불일치 90.0°/);
});

test("low speed skips unreliable bearing comparison but keeps position checks", () => {
  const input = qualityInput();
  input.gpsLocationExternal.speed = 0.5;
  input.gpsLocationExternal.bearingDeg = 270;
  input.gpsLocationExternal.bearingAccuracyDeg = 90;
  const quality = evaluateGeoPositionQuality(input);

  assert.equal(quality.canUseGeo, true);
  assert.equal(quality.headingDisagreementDeg, null);
  assert.equal(quality.headingSigmaDeg, null);
});

test("fresh Navi route remains usable when Comma GPS cannot approve an absolute geo fix", () => {
  const input = qualityInput({
    gpsLocationExternal: null,
    receivedAtMonotonic: {},
    naviRoutePolyline: [
      { latitude: 37.5, longitude: 127.0 },
      { latitude: 37.5, longitude: 127.003 },
    ],
  });
  const quality = evaluateGeoPositionQuality(input);

  assert.equal(quality.canUseGeo, false);
  assert.equal(quality.canUseRoute, true);
  assert.equal(quality.fallback, "navi-route");
  assert.deepEqual(quality.routeReasons, []);
});

test("compose uses TMap geo only when allowed and otherwise keeps local path AR", () => {
  const longitudeAt40mEast = 127 + 40 / (111320 * Math.cos(37.5 * Math.PI / 180));
  const navi = {
    vehicle: { present: true, latitude: 37.5, longitude: 127, headingDeg: 90 },
    guidanceCurrent: {
      present: true,
      distanceM: 40,
      turnType: 13,
      roadName: "테스트로",
      pointValid: true,
      latitude: 37.5,
      longitude: longitudeAt40mEast,
    },
  };
  const modelPosition = {
    x: [0, 20, 40, 60],
    y: [0, 0, 0, 0],
    z: [0, 0, 0, 0],
  };
  const base = {
    navi,
    naviUsable: true,
    canDrawPrecise: true,
    modelPosition,
    egoSpeedMps: 10,
  };

  const geo = composeFrame({ ...base, geoAllowed: true });
  const local = composeFrame({ ...base, geoAllowed: false });
  const geoGate = geo.fresh.find((item) => item.descriptor.kind === AR_MARKER_KIND.TURN_GATE);
  const localGate = local.fresh.find((item) => item.descriptor.kind === AR_MARKER_KIND.TURN_GATE);

  assert.equal(geoGate.placedBy, "geo");
  assert.equal(localGate.placedBy, "path");
  assert.equal(localGate.anchor.x, 40);
});

test("a distant event beyond model range uses the ordered TMap route polyline", () => {
  const latitude = 37.5;
  const longitude = 127;
  const longitudeAtEastM = (meters) => (
    longitude + meters / (111320 * Math.cos(latitude * Math.PI / 180))
  );
  const navi = {
    vehicle: { present: true, latitude, longitude, headingDeg: 90 },
    route: {
      polyline: [0, 100, 200, 350].map((meters) => ({
        latitude,
        longitude: longitudeAtEastM(meters),
      })),
    },
    guidanceCurrent: {
      present: true,
      distanceM: 250,
      turnType: 13,
      roadName: "원거리로",
      pointValid: false,
    },
  };
  const composed = composeFrame({
    navi,
    naviUsable: true,
    canDrawPrecise: true,
    geoAllowed: true,
    egoSpeedMps: 16.7,
    modelPosition: {
      x: [0, 20, 40, 60, 80],
      y: [0, 0, 0, 0, 0],
      z: [0, 0, 0, 0, 0],
    },
  });
  const marker = composed.fresh.find((item) => item.source === "guidanceCurrent");

  assert.ok(marker);
  assert.equal(marker.placedBy, "geo");
  assert.equal(marker.anchor.routeDerived, true);
  assert.ok(Math.abs(marker.anchor.x - 250) < 0.5);
});

test("a distant event uses the Navi route without an approved Comma geo fix", () => {
  const latitude = 37.5;
  const longitude = 127;
  const longitudeAtEastM = (meters) => (
    longitude + meters / (111320 * Math.cos(latitude * Math.PI / 180))
  );
  const navi = {
    vehicle: { present: true, latitude, longitude, headingDeg: 90 },
    route: {
      polyline: [0, 100, 200, 300].map((meters) => ({
        latitude,
        longitude: longitudeAtEastM(meters),
      })),
    },
    guidanceCurrent: {
      present: true,
      distanceM: 250,
      turnType: 13,
      pointValid: false,
    },
  };
  const composed = composeFrame({
    navi,
    naviUsable: true,
    canDrawPrecise: true,
    geoAllowed: false,
    routeAllowed: true,
    egoSpeedMps: 16.7,
    modelPosition: {
      x: [0, 20, 40, 60, 80],
      y: [0, 0, 0, 0, 0],
      z: [0, 0, 0, 0, 0],
    },
  });
  const marker = composed.fresh.find((item) => item.source === "guidanceCurrent");

  assert.ok(marker);
  assert.equal(marker.placedBy, "geo");
  assert.equal(marker.anchor.routeDerived, true);
  assert.ok(Math.abs(marker.anchor.x - 250) < 0.5);
});

test("a route-trusted TMap guidance point wins over an ambiguous distance walk", () => {
  const vehicle = {
    present: true,
    latitude: 36.62288290429861,
    longitude: 127.52000757270982,
    headingDeg: 317,
  };
  const point = {
    pointValid: true,
    latitude: 36.62328007646266,
    longitude: 127.5192437480293,
  };
  const composed = composeFrame({
    navi: {
      sessionId: "00001d9f--28e0608712",
      vehicle,
      route: {
        present: true,
        polyline: [
          { latitude: vehicle.latitude, longitude: vehicle.longitude },
          { latitude: vehicle.latitude, longitude: vehicle.longitude - 0.001 },
        ],
      },
      guidanceCurrent: {
        present: true,
        distanceM: 82,
        turnType: 13,
        ...point,
      },
    },
    naviUsable: true,
    canDrawPrecise: true,
    geoAllowed: false,
    routeAllowed: true,
    egoSpeedMps: 15,
  });
  const marker = composed.fresh.find((item) => item.source === "guidanceCurrent");

  assert.ok(marker);
  assert.equal(marker.placedBy, "geo");
  assert.ok(marker.anchor.x > 70 && marker.anchor.x < 90);
  assert.ok(marker.anchor.y > 10 && marker.anchor.y < 30);
  assert.equal(marker.anchor.routeDerived, undefined);
});

test("route-distance anchor walks bends instead of using a straight-line guess", () => {
  const latitude = 37.5;
  const longitude = 127;
  const eastLongitude = longitude + 100 / (111320 * Math.cos(latitude * Math.PI / 180));
  const northLatitude = latitude + 100 / 111320;
  const anchor = routeDistanceAnchor(
    { present: true, latitude, longitude, headingDeg: 90 },
    [
      { latitude, longitude },
      { latitude, longitude: eastLongitude },
      { latitude: northLatitude, longitude: eastLongitude },
    ],
    150,
  );

  assert.ok(anchor);
  assert.ok(Math.abs(anchor.x - 100) < 0.5);
  assert.ok(Math.abs(anchor.y - 50) < 0.5);
  assert.ok(Math.abs(anchor.headingRad - Math.PI / 2) < 0.01);
});

test("coordinate-less turn guidance prefers the Navi branch over the straight model path", () => {
  const latitude = 37.5;
  const longitude = 127;
  const eastLongitude = longitude + 30 / (111320 * Math.cos(latitude * Math.PI / 180));
  const northLatitude = latitude + 50 / 111320;
  const composed = composeFrame({
    navi: {
      vehicle: { present: true, latitude, longitude, headingDeg: 90 },
      route: { polyline: [
        { latitude, longitude },
        { latitude, longitude: eastLongitude },
        { latitude: northLatitude, longitude: eastLongitude },
      ] },
      guidanceCurrent: {
        present: true,
        pointValid: false,
        distanceM: 40,
        turnType: 12,
        roadName: "좌회전 도로",
      },
    },
    naviUsable: true,
    canDrawPrecise: true,
    geoAllowed: true,
    egoSpeedMps: 10,
    modelPosition: {
      x: [0, 20, 40, 60],
      y: [0, 0, 0, 0],
      z: [0, 0, 0, 0],
    },
  });
  const marker = composed.fresh.find((item) => item.source === "guidanceCurrent");

  assert.equal(marker.placedBy, "geo");
  assert.equal(marker.anchor.routeDerived, true);
  assert.ok(Math.abs(marker.anchor.x - 30) < 0.5);
  assert.ok(Math.abs(marker.anchor.y - 10) < 0.5);
  assert.ok(Math.abs(marker.anchor.headingRad - Math.PI / 2) < 0.05);
});

test("maneuver tangent uses the outgoing branch at an intersection vertex", () => {
  const latitude = 37.5;
  const longitude = 127;
  const eastLongitude = longitude + 30 / (111320 * Math.cos(latitude * Math.PI / 180));
  const northLatitude = latitude + 50 / 111320;
  const heading = routeTangentHeading(
    { present: true, latitude, longitude, headingDeg: 90 },
    { pointValid: true, latitude, longitude: eastLongitude },
    [
      { latitude, longitude },
      { latitude, longitude: eastLongitude },
      { latitude: northLatitude, longitude: eastLongitude },
    ],
    { lookAheadM: 12 },
  );

  assert.ok(Math.abs(heading - Math.PI / 2) < 0.05);
});
