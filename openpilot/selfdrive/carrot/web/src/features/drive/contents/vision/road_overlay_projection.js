const POLYLINE_SMOOTH_NEAR_DISTANCE = 16;
const POLYLINE_SMOOTH_FAR_DISTANCE = 52;
const POLYLINE_SMOOTH_MAX_STRENGTH = 0.34;
const POLYLINE_CENTER_SMOOTH_MAX_STRENGTH = 0.24;
const GEOMETRY_QUALITY_DEFAULT = "default";
const GEOMETRY_QUALITY_LANE = "lane";
const GEOMETRY_QUALITY_ROAD_EDGE = "road-edge";

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
  const isRecordedReplayActive = typeof options.isRecordedReplayActive === "function"
    ? options.isRecordedReplayActive
    : () => false;
  const maxDrawDistance = finiteNumber(options.maxDrawDistance, 100);

  if (typeof projectPoint !== "function" || typeof projectPointPrecise !== "function") {
    return null;
  }

  let frameCache;
  let temporalRibbonState;

  function resetFrame() {
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
    const dist = finiteNumber(distance, 0);
    const maxDist = finiteNumber(maximumDistance, maxDrawDistance);
    if (maxDist <= 36) return 1;
    if (dist >= Math.min(maxDist * 0.72, 62)) return 3;
    if (dist >= Math.min(maxDist * 0.44, 34)) return 2;
    return 1;
  }

  function getProjectionSampleStrideForQuality(distance, maximumDistance, quality = GEOMETRY_QUALITY_DEFAULT) {
    const baseStride = getProjectionSampleStride(distance, maximumDistance);
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

  function smoothRibbon(key, ribbon, alpha) {
    if (!key || !ribbon?.polygon?.length) return ribbon;
    // Replay samples already follow the recorded timeline. Frame EMA would
    // intentionally lag the geometry after seek or skipped browser frames.
    if (isRecordedReplayActive()) {
      temporalRibbonState.delete(key);
      return ribbon;
    }
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
    const result = {
      left: smoothedLeft,
      right: smoothedRight,
      center: smoothedCenter,
      polygon: smoothedLeft.length >= 2 && smoothedRight.length >= 2 ? smoothedLeft.concat([...smoothedRight].reverse()) : [],
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

  function circlePolygon(cx, cy, radius, points = 12) {
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
