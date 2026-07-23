import assert from "node:assert/strict";
import test from "node:test";

import { geographicWorldObservation } from "../src/features/drive/contents/vision/ar/runtime.js";
import { AR_POSITION_QUALITY } from "../src/features/drive/contents/vision/ar/tokens.js";

const navi = {
  publishMonoTimeNanos: 76_442_676_584,
  vehicle: {
    present: true,
    latitude: 36.62288290429861,
    longitude: 127.52000757270982,
    headingDeg: 317,
  },
};

test("fresh TMap route bounds world pose when Comma GPS covariance is absent", () => {
  const observation = geographicWorldObservation(navi, null, {
    canUseGeo: false,
    canUseRoute: true,
    positionSigmaM: null,
    headingSigmaDeg: null,
    reasons: ["Comma GPS unavailable"],
    routeReasons: [],
  });

  assert.equal(observation.valid, true);
  assert.equal(observation.source, "tmap-route");
  assert.equal(observation.positionSigmaM, AR_POSITION_QUALITY.routeWorldPositionSigmaM);
  assert.equal(observation.headingSigmaDeg, AR_POSITION_QUALITY.routeWorldHeadingSigmaDeg);
  assert.equal(observation.yawUsable, true);
  assert.deepEqual(observation.reasons, []);
});

test("missing route and geo quality cannot create a world observation", () => {
  const observation = geographicWorldObservation(navi, null, {
    canUseGeo: false,
    canUseRoute: false,
    positionSigmaM: null,
    headingSigmaDeg: null,
    reasons: ["GPS unavailable"],
    routeReasons: ["route unavailable"],
  });

  assert.equal(observation.valid, false);
  assert.equal(observation.source, "unavailable");
  assert.equal(observation.positionSigmaM, null);
  assert.equal(observation.headingSigmaDeg, null);
  assert.equal(observation.yawUsable, false);
  assert.deepEqual(observation.reasons, ["route unavailable"]);
});
