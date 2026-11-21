// classic 0x238 주행정보 라벨은 레이더 객체 ID와 대지 속도를 표시한다.
import assert from "node:assert/strict";
import test from "node:test";

import { formatClassicRadarLabel } from "../src/features/drive/contents/vision/road_overlay_lead_renderer.js";

test("shows the radar object ID and ground speed", () => {
  // Raw frames carry vLead directly.
  assert.equal(formatClassicRadarLabel({ trackId: 7, vLead: 10.0, vRel: -2.0 }, { vEgo: 12 }), "7 36");
  // The compact wire has only vRel: ego speed + vRel.
  assert.equal(formatClassicRadarLabel({ trackId: 63, vRel: -2.0 }, { vEgo: 12 }), "63 36");
  assert.equal(formatClassicRadarLabel({ trackId: 0, vRel: 0.0 }, { vEgo: 0 }), "0 0");
  assert.equal(formatClassicRadarLabel({ trackId: 5, vRel: -5.0 }, { vEgo: 0 }), "5 -18");
  assert.equal(formatClassicRadarLabel({ trackId: 7, vLead: 10.0 }, { isMetric: false }), "7 22");
});

test("keeps the ID when speed is unknown and hides fallback IDs", () => {
  assert.equal(formatClassicRadarLabel({ trackId: 7 }), "7");
  assert.equal(formatClassicRadarLabel({ trackId: 1003, vLead: 0 }), "- 0");
  assert.equal(formatClassicRadarLabel({}), "-");
});
