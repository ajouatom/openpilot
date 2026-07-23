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

const POLYLINE_SMOOTH_NEAR_DISTANCE = 16;
const POLYLINE_SMOOTH_FAR_DISTANCE = 52;
const POLYLINE_SMOOTH_MAX_STRENGTH = 0.34;
const POLYLINE_CENTER_SMOOTH_MAX_STRENGTH = 0.24;
const GEOMETRY_QUALITY_DEFAULT = "default";
const GEOMETRY_QUALITY_LANE = "lane";
const GEOMETRY_QUALITY_ROAD_EDGE = "road-edge";

/** Interval assumed on the first frame, before resetFrame() has a clock (~30fps). */
const DEFAULT_FRAME_DT_MS = 33;
/** Longer gaps (tab restore, replay seek, resume) snap instead of easing in. */
const MAX_SMOOTH_DT_MS = 250;
/** Spline steps per model span on the GPU path. Vertices are cheap; facets are not. */
const CURVE_SUBDIVISIONS = 4;

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function finiteNumber(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

export function createRoadOverlayProjection(options = {}) {
  const projectPoint = options.projectPoint;
  const projectPointPrecise = options.projectPointPrecise;
  const maxDrawDistance = finiteNumber(options.maxDrawDistance, 100);
  /* Sampling every model point is what keeps a curve reading as a curve. The
   * stride below exists because the Canvas2D path pays a CPU path operation per
   * point; on the WebGL2 path the extra vertices are effectively free while the
   * decimation is exactly what makes distant geometry look faceted. So keep full
   * density whenever the GPU path is driving. */
  const isFullDetailActive = typeof options.isFullDetailActive === "function"
    ? options.isFullDetailActive
    : () => false;

  if (typeof projectPoint !== "function" || typeof projectPointPrecise !== "function") {
    return null;
  }

  let frameCache;
  let temporalRibbonState;
  // Frame interval, refreshed at the frame boundary so smoothing can be
  // expressed as a time constant rather than a per-call fraction.
  let frameNowMs = null;
  let frameDtMs = DEFAULT_FRAME_DT_MS;

  function resetFrame(nowMs) {
    const now = Number(nowMs);
    if (Number.isFinite(now)) {
      frameDtMs = frameNowMs === null
        ? DEFAULT_FRAME_DT_MS
        : Math.max(0, now - frameNowMs);
      frameNowMs = now;
    }
    frameCache = {
      pathLengthIdx: new WeakMap(),
      ribbon: new WeakMap(),
      verticalRibbon: new WeakMap(),
      pathZ: new WeakMap(),
      pathY: new WeakMap(),
    };
  }

  function resetTemporal() {
    temporalRibbonState = new Map();
    // Do not let the first frame after a seek/reactivation inherit a stale
    // interval, which would make the geometry jump.
    frameNowMs = null;
    frameDtMs = DEFAULT_FRAME_DT_MS;
  }

  function getWeakCacheBucket(weakMap, target) {
    if (!target || typeof target !== "object") return null;
    let bucket = weakMap.get(target);
    if (!bucket) {
      bucket = new Map();
      weakMap.set(target, bucket);
    }
    return bucket;
  }

  function getProjectionSampleStride(distance, maximumDistance) {
    if (isFullDetailActive()) return 1;
    const dist = finiteNumber(distance, 0);
    const maxDist = finiteNumber(maximumDistance, maxDrawDistance);
    if (maxDist <= 36) return 1;
    if (dist >= Math.min(maxDist * 0.72, 62)) return 3;
    if (dist >= Math.min(maxDist * 0.44, 34)) return 2;
    return 1;
  }

  function getProjectionSampleStrideForQuality(distance, maximumDistance, quality = GEOMETRY_QUALITY_DEFAULT) {
    const baseStride = getProjectionSampleStride(distance, maximumDistance);
    if (baseStride === 1 && isFullDetailActive()) return 1;
    const dist = finiteNumber(distance, 0);

    if (quality === GEOMETRY_QUALITY_ROAD_EDGE) {
      if (dist >= 64) return Math.min(baseStride + 2, 5);
      if (dist >= 40) return Math.min(baseStride + 1, 4);
      return baseStride;
    }
    if (quality === GEOMETRY_QUALITY_LANE) {
      if (dist >= 58) return Math.min(baseStride + 1, 4);
      return baseStride;
    }
    return baseStride;
  }

  function forEachProjectedSampleIndex(xs, maxIdx, maximumDistance, visitor, quality = GEOMETRY_QUALITY_DEFAULT) {
    if (maxIdx < 0) return;
    let i = 0;
    let lastVisited = -1;
    while (i <= maxIdx) {
      visitor(i);
      lastVisited = i;
      i += getProjectionSampleStrideForQuality(xs[i], maximumDistance, quality);
    }
    if (lastVisited !== maxIdx) visitor(maxIdx);
  }

  function getGeometrySmoothingGain(quality, axis = "side") {
    if (quality === GEOMETRY_QUALITY_ROAD_EDGE) {
      return axis === "center" ? 1.12 : 1.22;
    }
    if (quality === GEOMETRY_QUALITY_LANE) {
      return axis === "center" ? 1.06 : 1.12;
    }
    return 1.0;
  }

  function getPolylineSmoothingStrength(distance, maximumDistance, maxStrength) {
    const dist = finiteNumber(distance, 0);
    const smoothFarDistance = Math.max(
      POLYLINE_SMOOTH_FAR_DISTANCE,
      Math.min(finiteNumber(maximumDistance, maxDrawDistance), maxDrawDistance),
    );
    if (dist <= POLYLINE_SMOOTH_NEAR_DISTANCE) return 0;
    const ratio = clamp(
      (dist - POLYLINE_SMOOTH_NEAR_DISTANCE) /
      Math.max(1, smoothFarDistance - POLYLINE_SMOOTH_NEAR_DISTANCE),
      0,
      1,
    );
    return maxStrength * ratio;
  }

  function smoothProjectedPolyline(points, distances, maximumDistance, maxStrength = POLYLINE_SMOOTH_MAX_STRENGTH) {
    if (!Array.isArray(points) || points.length < 3) return points;

    let smoothed = null;
    for (let i = 1; i < points.length - 1; i += 1) {
      const strength = getPolylineSmoothingStrength(distances?.[i], maximumDistance, maxStrength);
      if (strength <= 0.001) continue;

      const previous = points[i - 1];
      const current = points[i];
      const next = points[i + 1];
      const targetX = previous.x * 0.25 + current.x * 0.5 + next.x * 0.25;
      const targetY = previous.y * 0.25 + current.y * 0.5 + next.y * 0.25;
      if (!smoothed) {
        smoothed = points.map((point) => ({ x: point.x, y: point.y }));
      }
      smoothed[i].x = current.x + (targetX - current.x) * strength;
      smoothed[i].y = current.y + (targetY - current.y) * strength;
    }
    return smoothed || points;
  }

  /**
   * Time-based exponential smoothing factor.
   *
   * A fixed per-call alpha makes the effective time constant drift with the
   * render interval (20 Hz data, 30 fps cap, scheduler jitter), which is what
   * makes the overlay feel steppy and stiff. Deriving the factor from elapsed
   * time and a target time constant keeps the geometry approaching its target
   * at the same rate no matter how the frame spacing moves.
   *
   * A gap much longer than the time constant means the previous value is stale,
   * so snap rather than easing in from it.
   */
  function temporalAlpha(dtMs, tauMs) {
    const dt = Number(dtMs);
    const tau = Number(tauMs);
    if (!Number.isFinite(tau) || tau <= 0) return 1;
    if (!Number.isFinite(dt) || dt <= 0) return 0;
    if (dt >= MAX_SMOOTH_DT_MS) return 1;
    return 1 - Math.exp(-dt / tau);
  }

  /* Resample every span along a Catmull-Rom spline.
   *
   * The model only publishes ~33 path samples, so even at full density a long
   * curve resolves into visible straight spans once projected. Catmull-Rom
   * follows the curve those samples describe rather than the chord between
   * them, which is what removes the faceted look. Endpoints are duplicated so
   * the first and last spans keep their exact positions.
   *
   * Vertex count grows by CURVE_SUBDIVISIONS but fill rate does not: the same
   * silhouette is simply described more faithfully, so no extra pixels are
   * covered. On the GPU path that trade is essentially free.
   */
  function catmullRomKnot(previousKnot, a, b) {
    // Centripetal parameterisation (alpha = 0.5). Uniform spacing is what makes
    // a Catmull-Rom spline overshoot when the samples are not evenly spaced, and
    // projected road points bunch up hard toward the horizon. Centripetal knots
    // are the standard fix: the curve stays inside its control points, so no
    // cusps or loops.
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const distance = Math.sqrt(Math.hypot(dx, dy));
    return previousKnot + Math.max(distance, 1e-4);
  }

  function catmullRomAt(p0, p1, p2, p3, t) {
    const t0 = 0;
    const t1 = catmullRomKnot(t0, p0, p1);
    const t2 = catmullRomKnot(t1, p1, p2);
    const t3 = catmullRomKnot(t2, p2, p3);
    const time = t1 + (t2 - t1) * t;

    const lerp = (a, b, ta, tb) => {
      const span = tb - ta;
      if (!(Math.abs(span) > 1e-9)) return { x: a.x, y: a.y };
      const w = (time - ta) / span;
      return { x: a.x + (b.x - a.x) * w, y: a.y + (b.y - a.y) * w };
    };

    const a1 = lerp(p0, p1, t0, t1);
    const a2 = lerp(p1, p2, t1, t2);
    const a3 = lerp(p2, p3, t2, t3);
    const b1 = lerp(a1, a2, t0, t2);
    const b2 = lerp(a2, a3, t1, t3);
    const point = lerp(b1, b2, t1, t2);

    // Belt and braces: a resampled point must never leave the span it belongs
    // to. Any residual overshoot would draw as a streak across the frame.
    return {
      x: clamp(point.x, Math.min(p1.x, p2.x), Math.max(p1.x, p2.x)),
      y: clamp(point.y, Math.min(p1.y, p2.y), Math.max(p1.y, p2.y)),
    };
  }

  /* Resample every span along the spline.
   *
   * The model publishes only ~33 path samples, so a curve still breaks into
   * visible straight spans once projected. Resampling follows the curve those
   * samples describe instead of the chord between them. Endpoints are duplicated
   * so the first and last spans keep their exact positions.
   *
   * Vertex count grows but fill rate does not: the same silhouette is described
   * more faithfully, so no extra pixels are covered.
   */
  function subdivideProjectedPolyline(points, divisions = CURVE_SUBDIVISIONS) {
    const steps = Math.max(1, Math.round(divisions));
    if (!Array.isArray(points) || points.length < 2 || steps === 1) return points;
    const at = (index) => points[clamp(index, 0, points.length - 1)];
    const dense = [points[0]];
    for (let i = 0; i < points.length - 1; i += 1) {
      const p0 = at(i - 1);
      const p1 = points[i];
      const p2 = points[i + 1];
      const p3 = at(i + 2);
      for (let step = 1; step < steps; step += 1) {
        dense.push(catmullRomAt(p0, p1, p2, p3, step / steps));
      }
      dense.push(p2);
    }
    return dense;
  }

  function smoothPointListTemporal(previous, next, alpha) {
    if (!Array.isArray(previous) || !Array.isArray(next) || previous.length !== next.length) {
      return next.map((point) => ({ x: point.x, y: point.y }));
    }
    const a = clamp(alpha, 0, 1);
    return next.map((point, index) => ({
      x: previous[index].x + (point.x - previous[index].x) * a,
      y: previous[index].y + (point.y - previous[index].y) * a,
    }));
  }

  function smoothRibbon(key, ribbon, tauMs) {
    if (!key || !ribbon?.polygon?.length) return ribbon;
    const alpha = temporalAlpha(frameDtMs, tauMs);
    // Live and replay share the same temporal filter. Replay seek/reset already
    // clears this state, so bypassing it only makes recorded 20 Hz geometry
    // snap between samples while the video keeps advancing.
    const previous = temporalRibbonState.get(key);
    const left = smoothPointListTemporal(previous?.left, ribbon.left, alpha);
    const right = smoothPointListTemporal(previous?.right, ribbon.right, alpha);
    const center = smoothPointListTemporal(previous?.center, ribbon.center, alpha);
    const next = {
      left,
      right,
      center,
      polygon: left.length >= 2 && right.length >= 2 ? left.concat([...right].reverse()) : [],
    };
    temporalRibbonState.set(key, next);
    return next;
  }

  function getPathLengthIndex(line, maximumDistance) {
    const cacheBucket = getWeakCacheBucket(frameCache.pathLengthIdx, line);
    const cacheKey = Number.isFinite(Number(maximumDistance))
      ? Math.round(Number(maximumDistance) * 100)
      : "default";
    if (cacheBucket?.has(cacheKey)) return cacheBucket.get(cacheKey);

    const xs = Array.isArray(line?.x) ? line.x : [];
    let maxIdx = 0;
    for (let i = 1; i < xs.length; i += 1) {
      if (Number(xs[i]) > maximumDistance) break;
      maxIdx = i;
    }
    cacheBucket?.set(cacheKey, maxIdx);
    return maxIdx;
  }

  function buildRibbon(calibTransform, line, halfWidth, zOffset, maximumDistance, allowInvert = false, centerShift = 0, quality = GEOMETRY_QUALITY_DEFAULT) {
    const cacheBucket = getWeakCacheBucket(frameCache.ribbon, line);
    const cacheKey = [
      Math.round(finiteNumber(halfWidth, 0) * 1000),
      Math.round(finiteNumber(zOffset, 0) * 1000),
      Math.round(finiteNumber(maximumDistance, maxDrawDistance) * 100),
      allowInvert ? 1 : 0,
      Math.round(finiteNumber(centerShift, 0) * 1000),
      quality,
    ].join("|");
    if (cacheBucket?.has(cacheKey)) return cacheBucket.get(cacheKey);

    const xs = Array.isArray(line?.x) ? line.x : [];
    const ys = Array.isArray(line?.y) ? line.y : [];
    const zs = Array.isArray(line?.z) ? line.z : [];
    const left = [];
    const right = [];
    const center = [];
    const distances = [];
    const maxIdx = getPathLengthIndex(line, maximumDistance);

    forEachProjectedSampleIndex(xs, maxIdx, maximumDistance, (i) => {
      const x = finiteNumber(xs[i], NaN);
      if (!Number.isFinite(x) || x < 0) return;

      const y = finiteNumber(ys[i], 0) + centerShift;
      const z = finiteNumber(zs[i], 0) + zOffset;
      const leftPoint = projectPointPrecise(calibTransform, x, y - halfWidth, z);
      const rightPoint = projectPointPrecise(calibTransform, x, y + halfWidth, z);
      const centerPoint = projectPointPrecise(calibTransform, x, y, z);
      if (!leftPoint || !rightPoint || !centerPoint) return;
      if (!allowInvert && center.length && centerPoint.y > center[center.length - 1].y) return;

      left.push(leftPoint);
      right.push(rightPoint);
      center.push(centerPoint);
      distances.push(x);
    }, quality);

    const smoothMaxDistance = finiteNumber(maximumDistance, maxDrawDistance);
    const sideSmoothGain = getGeometrySmoothingGain(quality, "side");
    const centerSmoothGain = getGeometrySmoothingGain(quality, "center");
    const smoothedLeft = smoothProjectedPolyline(left, distances, smoothMaxDistance, POLYLINE_SMOOTH_MAX_STRENGTH * sideSmoothGain);
    const smoothedRight = smoothProjectedPolyline(right, distances, smoothMaxDistance, POLYLINE_SMOOTH_MAX_STRENGTH * sideSmoothGain);
    const smoothedCenter = smoothProjectedPolyline(center, distances, smoothMaxDistance, POLYLINE_CENTER_SMOOTH_MAX_STRENGTH * centerSmoothGain);
    // Only the GPU path pays for the extra vertices; Canvas2D keeps the cheaper
    // chord geometry so its known thermal cost is not increased.
    const dense = isFullDetailActive();
    const finalLeft = dense ? subdivideProjectedPolyline(smoothedLeft) : smoothedLeft;
    const finalRight = dense ? subdivideProjectedPolyline(smoothedRight) : smoothedRight;
    const finalCenter = dense ? subdivideProjectedPolyline(smoothedCenter) : smoothedCenter;
    const result = {
      left: finalLeft,
      right: finalRight,
      center: finalCenter,
      polygon: finalLeft.length >= 2 && finalRight.length >= 2 ? finalLeft.concat([...finalRight].reverse()) : [],
    };
    cacheBucket?.set(cacheKey, result);
    return result;
  }

  function buildVerticalRibbon(calibTransform, line, centerShift, topZOffset, bottomZOffset, maximumDistance) {
    const cacheBucket = getWeakCacheBucket(frameCache.verticalRibbon, line);
    const cacheKey = [
      Math.round(finiteNumber(centerShift, 0) * 1000),
      Math.round(finiteNumber(topZOffset, 0) * 1000),
      Math.round(finiteNumber(bottomZOffset, 0) * 1000),
      Math.round(finiteNumber(maximumDistance, maxDrawDistance) * 100),
    ].join("|");
    if (cacheBucket?.has(cacheKey)) return cacheBucket.get(cacheKey);

    const xs = Array.isArray(line?.x) ? line.x : [];
    const ys = Array.isArray(line?.y) ? line.y : [];
    const zs = Array.isArray(line?.z) ? line.z : [];
    const top = [];
    const bottom = [];
    const distances = [];
    const maxIdx = getPathLengthIndex(line, maximumDistance);

    forEachProjectedSampleIndex(xs, maxIdx, maximumDistance, (i) => {
      const x = finiteNumber(xs[i], NaN);
      if (!Number.isFinite(x) || x < 0) return;

      const y = finiteNumber(ys[i], 0) + centerShift;
      const z = finiteNumber(zs[i], 0);
      const topPoint = projectPoint(calibTransform, x, y, z + topZOffset);
      const bottomPoint = projectPoint(calibTransform, x, y, z + bottomZOffset);
      if (!topPoint || !bottomPoint) return;
      if (top.length && topPoint.y > top[top.length - 1].y) return;

      top.push(topPoint);
      bottom.push(bottomPoint);
      distances.push(x);
    });

    const smoothMaxDistance = finiteNumber(maximumDistance, maxDrawDistance);
    const smoothedTop = smoothProjectedPolyline(top, distances, smoothMaxDistance, POLYLINE_SMOOTH_MAX_STRENGTH);
    const smoothedBottom = smoothProjectedPolyline(bottom, distances, smoothMaxDistance, POLYLINE_SMOOTH_MAX_STRENGTH);
    const result = smoothedTop.length >= 2 && smoothedBottom.length >= 2
      ? smoothedTop.concat([...smoothedBottom].reverse())
      : [];
    cacheBucket?.set(cacheKey, result);
    return result;
  }

  function buildBandPolygon(left, right, startRatio, endRatio) {
    if (!Array.isArray(left) || !Array.isArray(right) || left.length < 2 || right.length < 2 || left.length !== right.length) {
      return [];
    }

    const first = [];
    const second = [];
    for (let i = 0; i < left.length; i += 1) {
      first.push({
        x: left[i].x + (right[i].x - left[i].x) * startRatio,
        y: left[i].y + (right[i].y - left[i].y) * startRatio,
      });
      second.push({
        x: left[i].x + (right[i].x - left[i].x) * endRatio,
        y: left[i].y + (right[i].y - left[i].y) * endRatio,
      });
    }
    return first.concat(second.reverse());
  }

  function interpolate(x, xs, ys) {
    if (!Array.isArray(xs) || !Array.isArray(ys) || xs.length < 2 || ys.length < 2) return NaN;
    const target = finiteNumber(x, NaN);
    if (!Number.isFinite(target)) return NaN;
    const lastIdx = Math.min(xs.length, ys.length) - 1;
    if (target <= finiteNumber(xs[0], 0)) return finiteNumber(ys[0], NaN);

    for (let i = 1; i <= lastIdx; i += 1) {
      const x0 = finiteNumber(xs[i - 1], NaN);
      const x1 = finiteNumber(xs[i], NaN);
      if (!Number.isFinite(x0) || !Number.isFinite(x1)) continue;
      if (target > x1 && i < lastIdx) continue;

      const y0 = finiteNumber(ys[i - 1], NaN);
      const y1 = finiteNumber(ys[i], NaN);
      if (!Number.isFinite(y0) || !Number.isFinite(y1)) return NaN;
      if (Math.abs(x1 - x0) < 1e-5) return y1;

      const ratio = clamp((target - x0) / (x1 - x0), 0, 1);
      return y0 + (y1 - y0) * ratio;
    }

    return finiteNumber(ys[lastIdx], NaN);
  }

  function samplePathZ(position, distance) {
    const cacheBucket = getWeakCacheBucket(frameCache.pathZ, position);
    const cacheKey = Math.round(finiteNumber(distance, 0) * 100);
    if (cacheBucket?.has(cacheKey)) return cacheBucket.get(cacheKey);

    const zs = Array.isArray(position?.z) ? position.z : [];
    const idx = getPathLengthIndex(position, distance);
    const value = finiteNumber(zs[idx], 0);
    cacheBucket?.set(cacheKey, value);
    return value;
  }

  function samplePathY(position, distance) {
    const cacheBucket = getWeakCacheBucket(frameCache.pathY, position);
    const cacheKey = Math.round(finiteNumber(distance, 0) * 100);
    if (cacheBucket?.has(cacheKey)) return cacheBucket.get(cacheKey);

    const value = interpolate(
      distance,
      Array.isArray(position?.x) ? position.x : [],
      Array.isArray(position?.y) ? position.y : [],
    );
    cacheBucket?.set(cacheKey, value);
    return value;
  }

  /* A fixed 12-gon is visibly angular once the marker grows on screen. Scale the
   * segment count with the radius so the silhouette stays round at any size; the
   * extra vertices are free next to the fill they enclose. */
  function circleSegmentCount(radius) {
    const r = Math.max(1, finiteNumber(radius, 1));
    return Math.round(clamp(Math.PI * 2 / Math.acos(1 - 0.35 / r), 12, 64));
  }

  function circlePolygon(cx, cy, radius, points = circleSegmentCount(radius)) {
    const polygon = [];
    for (let i = 0; i < points; i += 1) {
      const theta = (Math.PI * 2 * i) / points;
      polygon.push({
        x: cx + Math.cos(theta) * radius,
        y: cy + Math.sin(theta) * radius,
      });
    }
    return polygon;
  }

  resetFrame();
  resetTemporal();

  return Object.freeze({
    resetFrame,
    resetTemporal,
    getPathLengthIndex,
    buildRibbon,
    buildVerticalRibbon,
    smoothRibbon,
    buildBandPolygon,
    interpolate,
    samplePathZ,
    samplePathY,
    circlePolygon,
  });
}

export const DriveVisionRoadOverlayProjection = Object.freeze({ create: createRoadOverlayProjection });

export function installDriveVisionRoadOverlayProjectionFacade(target = globalThis) {
  target.DriveVisionRoadOverlayProjection = DriveVisionRoadOverlayProjection;
  return DriveVisionRoadOverlayProjection;
}
