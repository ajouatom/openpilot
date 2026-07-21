import assert from "node:assert/strict";
import test from "node:test";

import { createRoadOverlayProjection } from "../src/features/drive/contents/vision/road_overlay_projection.js";

function ribbon(x) {
  const left = [{ x, y: 0 }, { x, y: 10 }];
  const right = [{ x: x + 2, y: 0 }, { x: x + 2, y: 10 }];
  return { left, right, center: [{ x: x + 1, y: 0 }], polygon: [...left, ...right] };
}

test("recorded replay uses the same temporal ribbon smoothing as live vision", () => {
  const projection = createRoadOverlayProjection({
    projectPoint() {},
    projectPointPrecise() {},
    isRecordedReplayActive: () => true,
  });

  projection.smoothRibbon("path", ribbon(0), 0.2);
  const next = projection.smoothRibbon("path", ribbon(10), 0.2);

  assert.equal(next.left[0].x, 2);
  assert.equal(next.right[0].x, 4);
  assert.equal(next.center[0].x, 3);
});
