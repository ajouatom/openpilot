import assert from "node:assert/strict";
import test from "node:test";

import { composeFrame } from "../src/features/drive/contents/vision/ar/compose.js";
import { routeDistanceAnchor } from "../src/features/drive/contents/vision/ar/geo.js";
import {
  AR_ROUTE_MATCH_LIMITS,
  createRouteMatcher,
  matchRoutePosition,
} from "../src/features/drive/contents/vision/ar/route_matcher.js";

const LATITUDE = 37.5;
const LONGITUDE = 127;

function coordinate(eastM, northM) {
  return {
    latitude: LATITUDE + northM / 111320,
    longitude: LONGITUDE + eastM / (111320 * Math.cos(LATITUDE * Math.PI / 180)),
  };
}

function vehicle(northM = 0) {
  return {
    present: true,
    ...coordinate(0, northM),
    headingDeg: 90,
  };
}

test("route matcher keeps the production heading gate at 75 degrees", () => {
  assert.ok(Math.abs(AR_ROUTE_MATCH_LIMITS.maxHeadingErrorRad - 75 * Math.PI / 180) < 1e-12);
});

test("route matcher rejects the closer opposite carriageway", () => {
  const route = {
    present: true,
    polyline: [
      coordinate(100, 3),
      coordinate(-100, 3),
      coordinate(-100, 0),
      coordinate(200, 0),
    ],
  };
  const match = matchRoutePosition(vehicle(), route);

  assert.ok(match);
  assert.equal(match.index, 3);
  assert.ok(match.headingErrorRad < 0.01);
  assert.ok(match.snapDistanceM < 0.1);
});

test("route progress selects the later same-direction parallel branch", () => {
  const polyline = [
    coordinate(-100, 2),
    coordinate(100, 2),
    coordinate(100, 100),
    coordinate(-100, 0),
    coordinate(100, 0),
  ];
  const routeLengthM = 200 + 98 + Math.hypot(200, 100) + 200;
  const route = {
    present: true,
    totalDistanceM: Math.round(routeLengthM),
    remainingDistanceM: Math.round(routeLengthM * 0.15),
    polyline,
  };
  const match = matchRoutePosition(vehicle(1.2), route);

  assert.ok(match);
  assert.equal(match.index, 4);
  assert.ok(match.progressErrorM < match.routeLengthM * 0.2);
});

test("stateful matcher prevents a backward jump after route progress disappears", () => {
  const polyline = [
    coordinate(-100, 2),
    coordinate(100, 2),
    coordinate(100, 100),
    coordinate(-100, 0),
    coordinate(100, 0),
  ];
  const routeLengthM = 200 + 98 + Math.hypot(200, 100) + 200;
  const matcher = createRouteMatcher();
  const first = matcher.match(vehicle(1.2), {
    present: true,
    totalDistanceM: Math.round(routeLengthM),
    remainingDistanceM: Math.round(routeLengthM * 0.15),
    polyline,
  }, { sessionId: "route-a" });
  const heldBranch = matcher.match(vehicle(1.8), {
    present: true,
    totalDistanceM: 0,
    remainingDistanceM: 0,
    polyline,
  }, { sessionId: "route-a" });

  assert.equal(first.index, 4);
  assert.equal(heldBranch.index, 4);
  assert.ok(heldBranch.alongTrackM >= first.alongTrackM - matcher.limits.maxBackwardM);
});

test("GPS quality dynamically tightens route start snapping", () => {
  const route = {
    present: true,
    polyline: [coordinate(-100, 70), coordinate(100, 70)],
  };

  assert.equal(matchRoutePosition(vehicle(), route, { positionSigmaM: 2 }), null);
  assert.equal(matchRoutePosition(vehicle(), route), null);
});

test("route-distance anchor starts from the approved ordered segment", () => {
  const polyline = [
    coordinate(100, 3),
    coordinate(-100, 3),
    coordinate(-100, 0),
    coordinate(200, 0),
  ];
  const route = { present: true, polyline };
  const match = matchRoutePosition(vehicle(), route);
  const anchor = routeDistanceAnchor(vehicle(), polyline, 40, { routeMatch: match });

  assert.ok(anchor);
  assert.equal(anchor.routeStartIndex, 3);
  assert.ok(Math.abs(anchor.x - 40) < 0.1);
  assert.ok(Math.abs(anchor.y) < 0.1);
});

// The regression that hid AR entirely: a matcher miss must not delete the
// route sign. routeDistanceAnchor still places it from its own search; the
// matcher only sharpens the branch when it succeeds.
test("a route sign is still placed when the matcher returns no match", () => {
  // A route offset far enough that matchRoutePosition rejects it (snap limit).
  const polyline = [coordinate(0, 70), coordinate(200, 70)];
  assert.equal(matchRoutePosition(vehicle(), polyline && { present: true, polyline }), null);

  const composed = composeFrame({
    navi: {
      sessionId: "far-route",
      vehicle: vehicle(),
      route: { present: true, polyline },
      guidanceCurrent: {
        present: true,
        distanceM: 40,
        turnType: 12,
        roadName: "좌회전 도로",
        pointValid: true,
        ...coordinate(0, 70),
      },
    },
    naviUsable: true,
    canDrawPrecise: true,
    geoAllowed: false,   // force the route path, not geo
    routeAllowed: true,
  });

  const marker = composed.fresh?.find((item) => item.source === "guidanceCurrent");
  assert.ok(marker, "route sign must survive a matcher miss");
  assert.ok(marker.anchor, "and it must carry a placed anchor");
});

test("maneuver face follows the matched outgoing branch at an intersection", () => {
  const east30 = coordinate(30, 0);
  const north50 = coordinate(30, 50);
  const navi = {
    sessionId: "turn-route",
    vehicle: vehicle(),
    route: {
      present: true,
      polyline: [coordinate(0, 0), east30, north50],
    },
    guidanceCurrent: {
      present: true,
      distanceM: 30,
      turnType: 12,
      roadName: "좌회전 도로",
      pointValid: true,
      ...east30,
    },
  };
  const composed = composeFrame({
    navi,
    naviUsable: true,
    canDrawPrecise: true,
    geoAllowed: true,
    routeAllowed: true,
    routePositionSigmaM: 2,
    egoSpeedMps: 10,
    modelPosition: {
      x: [0, 20, 40, 60],
      y: [0, 0, 0, 0],
      z: [0, 0, 0, 0],
    },
  });
  const marker = composed.fresh.find((item) => item.source === "guidanceCurrent");

  assert.ok(marker);
  assert.ok(Math.abs(marker.anchor.roadForward[0]) < 0.05);
  assert.ok(marker.anchor.roadForward[1] > 0.95);
});
