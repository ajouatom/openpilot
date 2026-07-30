import assert from "node:assert/strict";
import test from "node:test";

import { DriveVisionHudModel } from "../src/features/drive/contents/vision/hud_model.js";
import { resolveLaneModeState } from "../src/features/drive/contents/vision/lane_mode.js";
import { createRoadOverlayPolicy } from "../src/features/drive/contents/vision/road_overlay_policy.js";

test("older recordings fall back to the available final-control state", () => {
  assert.deepEqual(resolveLaneModeState({
    controlsState: { activeLaneLine: true },
  }), {
    requested: null,
    planned: null,
    controlled: true,
    pathActive: true,
    presentation: "active",
  });
  assert.equal(resolveLaneModeState({
    controlsState: { activeLaneLine: false },
  }).presentation, "laneless");
});

test("road overlay mode follows planner output rather than request alone", () => {
  assert.equal(DriveVisionHudModel.isLaneMode({
    carState: { useLaneLineSpeed: 50 },
    lateralPlan: { useLaneLines: false },
    controlsState: { activeLaneLine: false },
  }), false);
  assert.equal(DriveVisionHudModel.isLaneMode({
    carState: { useLaneLineSpeed: 50 },
    lateralPlan: { useLaneLines: true },
    controlsState: { activeLaneLine: false },
  }), true);
});

test("road overlay selects matching geometry and style without mixing user intent", () => {
  const policy = createRoadOverlayPolicy();
  const modelPath = { x: [0, 10, 20], y: [0, 0, 0] };
  const lanePath = { x: [0, 10, 20], y: [0, 0.1, 0.2] };
  const overlayState = {
    modelV2: { position: modelPath },
    lateralPlan: { position: lanePath, useLaneLines: false },
    carControl: { longActive: true },
  };
  const params = {
    ShowPathMode: 1,
    ShowPathColor: 11,
    ShowPathModeLane: 2,
    ShowPathColorLane: 12,
  };

  const requestedOnly = policy.resolve(overlayState, {
    carState: { useLaneLineSpeed: 50 },
    lateralPlan: { useLaneLines: false },
    controlsState: { activeLaneLine: false },
  }, params);
  assert.equal(requestedOnly.selectedPath.pathSource, "modelV2");
  assert.equal(requestedOnly.pathStyle.mode, 1);
  assert.equal(requestedOnly.pathStyle.laneMode, false);

  overlayState.lateralPlan.useLaneLines = true;
  const planned = policy.resolve(overlayState, {
    carState: { useLaneLineSpeed: 50 },
    lateralPlan: { useLaneLines: true },
    controlsState: { activeLaneLine: false },
  }, params);
  assert.equal(planned.selectedPath.pathSource, "lateralPlan");
  assert.equal(planned.pathStyle.mode, 2);
  assert.equal(planned.pathStyle.laneMode, true);
});
