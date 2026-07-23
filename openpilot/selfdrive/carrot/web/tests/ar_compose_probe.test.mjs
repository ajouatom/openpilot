import assert from "node:assert/strict";
import test from "node:test";

import { collectSigns } from "../src/features/drive/contents/vision/ar/compose.js";
import { AR_SHAPE, AR_TONE } from "../src/features/drive/contents/vision/ar/design_tokens.js";

const probeInput = Object.freeze({
  calibrationProbe: true,
  probeDistanceM: 40,
  egoSpeedMps: 10,
});

test("field probe uses the approved token-preview GUIDE/BAR component", () => {
  const signs = collectSigns(probeInput);

  assert.equal(signs.length, 1);
  assert.equal(signs[0].source, "calibrationProbe");
  assert.equal(signs[0].descriptor.tone, AR_TONE.GUIDE);
  assert.equal(signs[0].descriptor.shape, AR_SHAPE.BAR);
  assert.equal(signs[0].descriptor.primary, "40m");
  assert.match(signs[0].descriptor.secondary, /AR 표시 확인/);
  assert.doesNotMatch(signs[0].descriptor.secondary, /직진/);
  assert.equal(signs[0].descriptor.turnSign, 0);
});

test("idle Navi may show the explicit field probe", () => {
  const signs = collectSigns({
    ...probeInput,
    navi: {
      connected: true,
      navigationStatus: { guidanceActive: false, offRoute: false },
    },
    naviUsable: false,
  });

  assert.equal(signs.length, 1);
  assert.equal(signs[0].source, "calibrationProbe");
});

test("active off-route guidance never falls back to a synthetic marker", () => {
  const signs = collectSigns({
    ...probeInput,
    navi: {
      connected: true,
      navigationStatus: { guidanceActive: true, offRoute: true },
    },
    naviUsable: false,
  });

  assert.deepEqual(signs, []);
});

test("real Navi markers always win over the field probe", () => {
  const signs = collectSigns({
    ...probeInput,
    navi: {
      connected: true,
      navigationStatus: { guidanceActive: true, offRoute: false },
      guidanceCurrent: {
        present: true,
        distanceM: 80,
        turnType: 13,
        roadName: "테헤란로",
      },
    },
    naviUsable: true,
  });

  assert.ok(signs.length >= 1);
  assert.equal(signs.some((item) => item.source === "calibrationProbe"), false);
  assert.equal(signs[0].source, "guidanceCurrent");
});

test("distance and label updates keep one stable guidance marker identity", () => {
  const naviAt = (distanceM, roadName, latitude = 37.5001) => ({
    navigationStatus: { guidanceActive: true, offRoute: false },
    guidanceCurrent: {
      present: true,
      distanceM,
      turnType: 13,
      roadName,
      pointValid: true,
      latitude,
      longitude: 127.0001,
    },
  });
  const first = collectSigns({ navi: naviAt(120, "강남대로"), naviUsable: true });
  const updated = collectSigns({ navi: naviAt(95, "강남대로 방면"), naviUsable: true });
  const nextEvent = collectSigns({ navi: naviAt(180, "강남대로", 37.5010), naviUsable: true });

  assert.equal(first[0].markerId, updated[0].markerId);
  assert.equal(first[0].lifecycleSlot, updated[0].lifecycleSlot);
  assert.notEqual(first[0].markerId, nextEvent[0].markerId);
});
