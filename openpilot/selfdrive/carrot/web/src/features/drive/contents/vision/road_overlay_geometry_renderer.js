/* ============================================================================
 * CARROT VISION GRAPHICS - DO NOT CHANGE CASUALLY
 *
 * This is the driving view the user actually looks at. Its smoothness was tuned
 * against real device behaviour and several "obvious" simplifications have
 * already been tried and reverted. If you are here while working on AR, replay
 * or any other feature, prefer adding your own layer over editing this one.
 *
 * Invariants. Breaking any of these brings back stutter, judder or heat:
 *
 *  1. One overlay update per presented video frame. requestVideoFrameCallback
 *     is taken first and unthrottled; the interval constant only paces the
 *     fallback. Never put a timer between a video frame and its overlay.
 *  2. Live and replay share one scheduler, one filter and one cadence. They are
 *     the same user experience; do not special-case one of them.
 *  3. Temporal smoothing is a time constant, never a per-call alpha. Render
 *     spacing moves constantly, and a fixed alpha makes the response wander.
 *  4. Geometry stays on the GPU: fills, strokes and dashes all land in one
 *     batched draw call. Moving any of them back to Canvas2D reintroduces a
 *     full-surface raster and its clear every frame.
 *  5. The 2D overlay is cleared only when something actually drew to it. If you
 *     add a direct ctx draw, mark that layer dirty or you will ship ghosting.
 *  6. Vertex pools are reused. Do not rebuild per-frame arrays.
 *
 * Already tried and rejected:
 *  - Overlay projection/triangulation in a worker: the extra hop makes geometry
 *    trail the video by a frame (see OFFSCREEN_WORKER_ENABLED).
 *  - Interpolating geometry between 20 Hz samples: the video is 20 fps, so the
 *    overlay slides against a still image.
 *  - Uniform Catmull-Rom resampling: overshoots on unevenly spaced projected
 *    points and draws streaks across the frame. Centripetal only.
 *
 * Measured state: worst curve kink 8.1deg -> 1.6deg, geometry fully GPU-batched,
 * per-frame 2D clear removed.
 * ==========================================================================*/

const PATH_HALF_WIDTH = 0.9;
const TEST_PATH_VISIBILITY_SOLID_ALPHA = 0.50;
const TEST_PATH_VISIBILITY_MID_ALPHA = 0.24;
const TEST_LANE_PROB_MIN = 0.003;
const TEST_LANE_PROB_BOOST = 6;
/* Smoothing time constants (ms).
 *
 * These were fixed per-call alphas (path 0.20 / lane 0.16). Because the render
 * interval keeps moving (20 Hz data, 30 fps cap, scheduler jitter), a fixed
 * alpha let the effective time constant wander between ~148 ms and ~287 ms.
 * That inconsistency, more than the lag itself, is what read as steppy.
 *
 * With a time constant the geometry approaches its target at the same rate
 * whatever the spacing. The values sit below the old effective range so the
 * response is quicker while still damping jitter. The path leads the lanes
 * because it has to show steering response immediately. */
const PATH_SMOOTH_TAU_MS = 110;
const LANE_SMOOTH_TAU_MS = 140;
const LANE_VISUAL_WIDTH_GAIN = 1.4;
const GEOMETRY_QUALITY_LANE = "lane";
const GEOMETRY_QUALITY_ROAD_EDGE = "road-edge";

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function finiteNumber(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

export function createRoadOverlayGeometryRenderer(options = {}) {
  const context = options.context;
  const getParams = typeof options.getParams === "function" ? options.getParams : () => ({});
  const isPerformanceActive = typeof options.isPerformanceActive === "function"
    ? options.isPerformanceActive
    : () => false;
  const paletteColor = options.paletteColor;
  const rgba = options.rgba;
  const getCachedGradient = options.getCachedGradient;
  const geometry = options.geometry || {};
  const pathZOffset = finiteNumber(options.pathZOffset, 1.22);
  const maxDrawDistance = finiteNumber(options.maxDrawDistance, 100);

  if (
    !context
    || typeof paletteColor !== "function"
    || typeof rgba !== "function"
    || typeof getCachedGradient !== "function"
    || typeof geometry.drawPolygon !== "function"
    || typeof geometry.drawPolyline !== "function"
    || typeof geometry.buildRibbon !== "function"
    || typeof geometry.smoothRibbon !== "function"
    || typeof geometry.buildBandPolygon !== "function"
    || typeof geometry.getSceneMaxDistance !== "function"
    || typeof geometry.getPathMaxDistance !== "function"
    || typeof geometry.getPathLengthIndex !== "function"
  ) {
    return null;
  }

  function drawSegmentBands(left, right, style, step) {
    if (!Array.isArray(left) || left.length < 3 || left.length !== right.length) return;
    const baseColor = paletteColor(style.paletteIndex);
    const stroke = style.emphasisStroke ? style.strokeColor : "";
    for (let i = 0; i < left.length - 2; i += step) {
      const next = Math.min(i + 2, left.length - 1);
      const segment = [left[i], left[next], right[next], right[i]];
      geometry.drawPolygon(segment, rgba(baseColor, 0.28), stroke, 1.1);
    }
  }

  function getPathGradientAlpha(style) {
    const mode = finiteNumber(style?.mode, 0);
    let solidAlpha = mode === 0 ? 0.40 : 0.24;
    let midAlpha = mode === 0 ? 0.20 : 0.12;
    if (mode === 0 || style?.isCruiseOff) {
      solidAlpha = Math.max(solidAlpha, TEST_PATH_VISIBILITY_SOLID_ALPHA);
      midAlpha = Math.max(midAlpha, TEST_PATH_VISIBILITY_MID_ALPHA);
    }
    return { solidAlpha, midAlpha };
  }

  function createPathGradient(baseColor, canvasHeight, style) {
    const { solidAlpha, midAlpha } = getPathGradientAlpha(style);
    const cacheKey = `g:${baseColor.r},${baseColor.g},${baseColor.b}|${Math.round(canvasHeight)}|${solidAlpha}|${midAlpha}`;
    return getCachedGradient(cacheKey, () => {
      const gradient = context.createLinearGradient(0, canvasHeight, 0, canvasHeight * 0.32);
      gradient.addColorStop(0, rgba(baseColor, solidAlpha));
      gradient.addColorStop(0.55, rgba(baseColor, midAlpha));
      gradient.addColorStop(1, rgba(baseColor, 0.0));
      return gradient;
    });
  }

  function drawPathRibbon(ribbon, style, canvasHeight) {
    if (!ribbon.polygon.length) return;
    const baseColor = paletteColor(style.paletteIndex);
    if (style.mode === 0) {
      geometry.drawPolygon(
        ribbon.polygon,
        rgba(baseColor, 0.42),
        style.emphasisStroke ? style.strokeColor : "",
        style.emphasisStroke ? 2.0 : 0,
      );
      return;
    }
    const gradientAlpha = getPathGradientAlpha(style);
    const fill = isPerformanceActive()
      ? {
          type: "vertical-gradient",
          color: baseColor,
          bottomAlpha: gradientAlpha.solidAlpha,
          midAlpha: gradientAlpha.midAlpha,
        }
      : createPathGradient(baseColor, canvasHeight, style);
    geometry.drawPolygon(
      ribbon.polygon,
      fill,
      style.emphasisStroke ? style.strokeColor : "",
      style.emphasisStroke ? 1.7 : 0,
    );
  }

  function drawAnimatedPath(ribbon, style) {
    if (!ribbon.center.length) return;
    const dashPresets = {
      1: [14, 11],
      2: [10, 8],
      3: [26, 12],
      4: [18, 10, 5, 10],
      5: [30, 12],
      6: [12, 7],
      7: [20, 8, 4, 8],
      8: [8, 6],
    };
    const baseColor = paletteColor(style.paletteIndex);
    const dash = dashPresets[style.mode] || dashPresets[1];
    const dashLength = dash.reduce((sum, value) => sum + value, 0);
    const dashOffset = -((performance.now() / 120) % dashLength);
    geometry.drawPolyline(ribbon.center, rgba(baseColor, 0.78), 5.5, dash, dashOffset);
    if (style.emphasisStroke) {
      geometry.drawPolyline(ribbon.center, style.strokeColor, 1.6);
    }
  }

  function drawComplexPath(ribbon, style) {
    const step = style.mode === 9 ? 3 : style.mode === 10 ? 4 : style.mode === 11 ? 5 : 6;
    drawSegmentBands(ribbon.left, ribbon.right, style, step);
  }

  function drawSpecialPath(ribbon, style) {
    const baseColor = paletteColor(style.paletteIndex);
    const bands = [];
    if (style.mode === 13 || style.mode === 14) {
      bands.push(geometry.buildBandPolygon(ribbon.left, ribbon.right, 0.0, 0.18));
      bands.push(geometry.buildBandPolygon(ribbon.left, ribbon.right, 0.82, 1.0));
    }
    if (style.mode === 13 || style.mode === 15) {
      bands.push(geometry.buildBandPolygon(ribbon.left, ribbon.right, 0.38, 0.62));
    }

    for (const band of bands) {
      geometry.drawPolygon(
        band,
        rgba(baseColor, 0.34),
        style.emphasisStroke ? style.strokeColor : "",
        style.emphasisStroke ? 1.4 : 0,
      );
    }
  }

  function getPathHalfWidth() {
    const widthRatio = clamp(finiteNumber(getParams().ShowPathWidth, 100) / 100, 0.1, 2.0);
    return PATH_HALF_WIDTH * widthRatio;
  }

  function drawPath(pathData, model, overlayState, calibTransform, canvasHeight, style) {
    if (!pathData || !Array.isArray(pathData.x) || !pathData.x.length) return;
    const sceneMaxDistance = geometry.getSceneMaxDistance(model, overlayState);
    const rawRibbon = geometry.buildRibbon(
      calibTransform,
      pathData,
      getPathHalfWidth(),
      pathZOffset,
      geometry.getPathMaxDistance(sceneMaxDistance),
      false,
    );
    const ribbon = geometry.smoothRibbon(
      `path:${style?.laneMode ? "lane" : "model"}:${style?.mode ?? 0}`,
      rawRibbon,
      PATH_SMOOTH_TAU_MS,
    );
    if (ribbon.polygon.length < 3) return;

    drawPathRibbon(ribbon, style, canvasHeight);
    if (style.mode === 0) return;
    if (style.mode >= 13 && style.mode <= 15) {
      drawSpecialPath(ribbon, style);
      return;
    }
    if (style.mode >= 9) {
      drawComplexPath(ribbon, style);
      return;
    }
    drawAnimatedPath(ribbon, style);
  }

  function drawLaneLines(model, overlayState, hudState, calibTransform) {
    const laneLines = Array.isArray(model?.laneLines) ? model.laneLines : [];
    const laneLineProbs = Array.isArray(model?.laneLineProbs) ? model.laneLineProbs : [];
    if (!laneLines.length) return;

    const leftLaneLine = finiteNumber(hudState?.carState?.leftLaneLine, 0);
    const rightLaneLine = finiteNumber(hudState?.carState?.rightLaneLine, 0);
    const sceneMaxDistance = geometry.getSceneMaxDistance(model, overlayState);
    const maxIdx = geometry.getPathLengthIndex(laneLines[0], sceneMaxDistance);
    for (let i = 0; i < laneLines.length; i += 1) {
      const prob = clamp(finiteNumber(laneLineProbs[i], 0), 0, 0.9);
      const renderProb = prob >= 0.02
        ? prob
        : (prob >= TEST_LANE_PROB_MIN ? clamp(prob * TEST_LANE_PROB_BOOST, 0.02, 0.12) : 0);
      if (renderProb <= 0) continue;

      const highlightedLeft = i === 1 && Math.floor(leftLaneLine / 10) === 2;
      const highlightedRight = i === 2 && Math.floor(rightLaneLine / 10) === 2;
      const laneColor = highlightedLeft || highlightedRight
        ? { r: 255, g: 217, b: 94 }
        : { r: 255, g: 255, b: 255 };
      const baseHalfWidth = Math.max(highlightedLeft || highlightedRight ? 0.025 : 0.010, 0.025 * renderProb);
      const halfWidth = baseHalfWidth * LANE_VISUAL_WIDTH_GAIN;
      const fillAlpha = prob >= 0.02 ? clamp(renderProb, 0.12, 0.7) : clamp(renderProb * 3.0, 0.16, 0.26);
      const strokeAlpha = prob >= 0.02 ? 0.20 : 0.24;
      const maxDistance = finiteNumber(laneLines[i]?.x?.[maxIdx], maxDrawDistance);
      const rawRibbon = geometry.buildRibbon(
        calibTransform,
        laneLines[i],
        halfWidth,
        0,
        maxDistance,
        false,
        0,
        GEOMETRY_QUALITY_LANE,
      );
      const ribbon = geometry.smoothRibbon(`lane:${i}`, rawRibbon, LANE_SMOOTH_TAU_MS);
      geometry.drawPolygon(
        ribbon.polygon,
        `rgba(${laneColor.r},${laneColor.g},${laneColor.b},${fillAlpha.toFixed(3)})`,
        `rgba(${laneColor.r},${laneColor.g},${laneColor.b},${strokeAlpha.toFixed(3)})`,
        1,
      );

      if ((i === 1 && (leftLaneLine % 10) === 4) || (i === 2 && (rightLaneLine % 10) === 4)) {
        const shift = i === 1 ? -0.3 : 0.3;
        const rawDoubleRibbon = geometry.buildRibbon(
          calibTransform,
          laneLines[i],
          halfWidth,
          0,
          maxDistance,
          false,
          shift,
          GEOMETRY_QUALITY_LANE,
        );
        const doubleRibbon = geometry.smoothRibbon(
          `lane:${i}:double:${shift}`,
          rawDoubleRibbon,
          LANE_SMOOTH_TAU_MS,
        );
        geometry.drawPolygon(
          doubleRibbon.polygon,
          `rgba(${laneColor.r},${laneColor.g},${laneColor.b},${fillAlpha.toFixed(3)})`,
          `rgba(${laneColor.r},${laneColor.g},${laneColor.b},${strokeAlpha.toFixed(3)})`,
          1,
        );
      }
    }
  }

  function drawRoadEdges(model, overlayState, calibTransform) {
    const roadEdges = Array.isArray(model?.roadEdges) ? model.roadEdges : [];
    const roadEdgeStds = Array.isArray(model?.roadEdgeStds) ? model.roadEdgeStds : [];
    if (!roadEdges.length) return;

    const sceneMaxDistance = geometry.getSceneMaxDistance(model, overlayState);
    const maxIdx = geometry.getPathLengthIndex(roadEdges[0], sceneMaxDistance);
    for (let i = 0; i < roadEdges.length; i += 1) {
      const edgeStd = clamp(finiteNumber(roadEdgeStds[i], 0.4), 0, 1);
      const alpha = clamp(1 - edgeStd, 0.12, 0.66);
      const ribbon = geometry.buildRibbon(
        calibTransform,
        roadEdges[i],
        0.025,
        0,
        finiteNumber(roadEdges[i]?.x?.[maxIdx], maxDrawDistance),
        false,
        0,
        GEOMETRY_QUALITY_ROAD_EDGE,
      );
      geometry.drawPolygon(
        ribbon.polygon,
        `rgba(255,78,59,${alpha.toFixed(3)})`,
        "rgba(255,124,104,0.28)",
        1,
      );
    }
  }

  return Object.freeze({ drawPath, drawLaneLines, drawRoadEdges });
}

export const DriveVisionRoadOverlayGeometryRenderer = Object.freeze({ create: createRoadOverlayGeometryRenderer });

export function installDriveVisionRoadOverlayGeometryRendererFacade(target = globalThis) {
  target.DriveVisionRoadOverlayGeometryRenderer = DriveVisionRoadOverlayGeometryRenderer;
  return DriveVisionRoadOverlayGeometryRenderer;
}
