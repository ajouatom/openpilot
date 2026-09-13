import test from 'node:test';
import assert from 'node:assert/strict';
import { createRoadOverlayAuxRenderer } from '../src/features/drive/contents/vision/road_overlay_aux_renderer.js';

test('TF marker follows the published distance and labels metres', () => {
  const labels = [], positions = [];
  const renderer = createRoadOverlayAuxRenderer({
    getParams: () => ({ ShowPathEnd: 1 }),
    geometry: {
      buildVerticalRibbon() {}, drawPolygon() {}, samplePathY: () => 0, interpolate: () => 0,
      projectPoint: (_calib, distance, y) => ({ x: distance, y }),
      drawPolyline: (points) => positions.push(points[0].x),
    },
    ui: {
      getScale: () => 1, displayDistance: value => value * 3.28,
      clampTextAnchor: point => point, drawText: text => labels.push(text),
    },
  });
  for (const desiredDistance of [25.1, 35.4, 0, 101]) {
    renderer.drawProjectedTfMarker({ x: [0, 100], y: [0, 0], z: [0, 0] }, { desiredDistance }, null, 1920, 1080);
  }
  assert.deepEqual(positions, [25.1, 35.4]);
  assert.deepEqual(labels, ['25 m', '35 m']);
});
