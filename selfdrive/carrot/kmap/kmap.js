(function () {
  "use strict";

  const KAKAO_JS_KEY = "3492cfb289f76c41d54b83d0923e4fcc";
  const KAKAO_SDK_URL = "https://dapi.kakao.com/v2/maps/sdk.js";
  const SDK_TIMEOUT_MS = 9000;
  const DEFAULT_CENTER = Object.freeze({ lat: 37.5665, lon: 126.9780 });
  const INTERP_BASE_MS = 1100;
  const INTERP_MAX_MS = 1800;
  const INTERP_MIN_MS = 350;
  const NAV_STALE_MS = 5000;
  const EARTH_RADIUS_M = 6378137;
  const MAP_CENTER_DEFAULT_INTERVAL_MS = 520;
  const MAP_CENTER_PARKED_INTERVAL_MS = 2600;
  const MAP_CENTER_SLOW_INTERVAL_MS = 900;
  const MAP_CENTER_EXPANDED_INTERVAL_MS = 280;
  const MAP_CENTER_DEFAULT_DISTANCE_M = 5.5;
  const MAP_CENTER_PARKED_DISTANCE_M = 14;
  const MAP_CENTER_SLOW_DISTANCE_M = 4;
  const MAP_CENTER_EXPANDED_DISTANCE_M = 1.5;
  const KAKAO_MIN_LEVEL = 1;
  const KAKAO_MAX_LEVEL = 8;
  const ROUTE_FIT_PADDING_MIN_PX = 28;
  const ROUTE_FIT_PADDING_RATIO = 0.08;

  const root = document.getElementById("kmapRoot");
  const kakaoMapEl = document.getElementById("kakaoMap");
  const overlayCanvas = document.getElementById("overlayCanvas");
  const surface = document.getElementById("mapSurface");
  const marker = document.getElementById("vehicleMarker");
  const statusText = document.getElementById("statusText");
  const navInfo = document.getElementById("navInfo");
  const navRoad = document.getElementById("navRoad");
  const navMeta = document.getElementById("navMeta");
  const demoPanel = document.getElementById("demoPanel");
  const demoMove = document.getElementById("demoMove");
  const demoMode = document.getElementById("demoMode");

  const state = {
    lat: DEFAULT_CENTER.lat,
    lon: DEFAULT_CENTER.lon,
    heading: 0,
    speed: 0,
    level: 4,
    zoomBias: 0,        // global zoom offset from parent (+ = farther, - = closer)
    lastTs: 0,
    provider: "mock",
    mode: "box",
    overlayHeadingUp: true,
    curvatureColor: false,
    mapType: "roadmap",   // roadmap | satellite | hybrid (Kakao base layer)
    themeMode: "day",
    displayTheme: "day",
    kakaoReady: false,
    map: null,
    markerOverlay: null,
    markerEl: null,
    lastLevelChangeAt: 0,
    status: "idle",
    error: "",
    sdkLoadedAt: 0,
    hasVehicle: false,
    lastDebugPostAt: 0,
    overlayRaf: 0,
    lastMapCenterLat: null,
    lastMapCenterLon: null,
    lastMapCenterAt: 0,
  };

  const navState = {
    active: false,
    path: "",
    points: [],
    road: "",
    turn: null,
    goal: null,
    sdi: null,
    origin: null,
    heading: null,
    dirty: true,
    updatedAt: 0,
    staleTimer: 0,
    lastViewRange: 0,
    lastCanvasWidth: 0,
    lastCanvasHeight: 0,
    lastProjectionSig: "",
  };

  const routeState = {
    active: false,
    expanded: false,
    coordinates: [],
    bounds: null,
    dirty: true,
    fitted: false,
  };

  // RAF-driven interpolation state. `display` is what's currently on screen.
  // `source` is where the last interp segment started; `target` is the most
  // recent sample. We lerp display from source -> target across `durationMs`.
  const interp = {
    source: { lat: DEFAULT_CENTER.lat, lon: DEFAULT_CENTER.lon, heading: 0 },
    target: { lat: DEFAULT_CENTER.lat, lon: DEFAULT_CENTER.lon, heading: 0 },
    display: { lat: DEFAULT_CENTER.lat, lon: DEFAULT_CENTER.lon, heading: 0 },
    segmentStart: 0,
    durationMs: INTERP_BASE_MS,
    lastSampleAt: 0,
    raf: 0,
    active: false,
  };

  function finiteNumber(value) {
    const number = Number(value);
    return Number.isFinite(number) ? number : null;
  }

  function validLatLon(lat, lon) {
    return lat !== null && lon !== null && Math.abs(lat) <= 90 && Math.abs(lon) <= 180 && !(lat === 0 && lon === 0);
  }

  function distanceMeters(aLat, aLon, bLat, bLon) {
    if (!validLatLon(aLat, aLon) || !validLatLon(bLat, bLon)) return Infinity;
    const lat1 = aLat * Math.PI / 180;
    const lat2 = bLat * Math.PI / 180;
    const dLat = (bLat - aLat) * Math.PI / 180;
    const dLon = (bLon - aLon) * Math.PI / 180;
    const sinLat = Math.sin(dLat / 2);
    const sinLon = Math.sin(dLon / 2);
    const h = sinLat * sinLat + Math.cos(lat1) * Math.cos(lat2) * sinLon * sinLon;
    return 2 * EARTH_RADIUS_M * Math.atan2(Math.sqrt(h), Math.sqrt(Math.max(0, 1 - h)));
  }

  function normalizeHeading(value) {
    const heading = finiteNumber(value);
    if (heading === null) return state.heading;
    return ((heading % 360) + 360) % 360;
  }

  function boolParam(params, key, fallback) {
    const value = params.get(key);
    if (value === null) return fallback;
    return ["1", "true", "yes", "on"].includes(value.trim().toLowerCase());
  }

  function setDisplayTheme() {
    state.themeMode = "day";
    state.displayTheme = "day";
    root.dataset.themeMode = state.themeMode;
    root.dataset.theme = state.displayTheme;
  }

  function levelForSpeed(speed, previousLevel = state.level) {
    // Keep automatic zoom deliberately simple. High speed stays at the same
    // default level as normal driving; only low-speed/parked states zoom in.
    // Small hysteresis prevents zoom flutter near the low/default thresholds.
    if (speed >= 32 || (previousLevel >= 3 && speed >= 24)) return 3;
    if (speed >= 5 || (previousLevel >= 2 && speed >= 2)) return 2;
    return 1;
  }

  function motionForSpeed(speed) {
    if (speed >= 105) return "highway";
    if (speed >= 70) return "fast";
    if (speed >= 25) return "city";
    if (speed >= 4) return "slow";
    return "parked";
  }

  function viewRangeMeters(speedKph) {
    // Match the simplified Kakao zoom policy: high speed does not zoom out.
    let base;
    if (speedKph >= 30) base = 140;
    else base = 100;
    // Global zoom bias: each step ~ one Kakao level (≈ 1.7x scale). Negative
    // bias = zoom in (smaller range), positive = zoom out (larger range).
    return base * Math.pow(1.7, state.zoomBias || 0);
  }

  function expandedFallbackRange(speedKph) {
    return Math.round(viewRangeMeters(speedKph) * 2.2);
  }

  function kakaoDisplayLevel() {
    const bias = state.zoomBias || 0;
    if (routeState.expanded && !routeState.active) {
      return Math.max(KAKAO_MIN_LEVEL, Math.min(KAKAO_MAX_LEVEL, state.level + 2 + bias));
    }
    // Apply global zoom bias on top of the automatic speed-based level.
    return Math.max(KAKAO_MIN_LEVEL, Math.min(KAKAO_MAX_LEVEL, state.level + bias));
  }

  function resizeOverlayCanvas() {
    if (!overlayCanvas) return false;
    const rect = overlayCanvas.getBoundingClientRect();
    const cssWidth = Math.max(1, Math.round(rect.width));
    const cssHeight = Math.max(1, Math.round(rect.height));
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.round(cssWidth * dpr);
    const height = Math.round(cssHeight * dpr);
    if (overlayCanvas.width === width && overlayCanvas.height === height) return false;
    overlayCanvas.width = width;
    overlayCanvas.height = height;
    navState.lastCanvasWidth = cssWidth;
    navState.lastCanvasHeight = cssHeight;
    navState.dirty = true;
    routeState.dirty = true;
    return true;
  }

  function parsePath(path) {
    if (!path) return [];
    const points = [];
    const chunks = String(path).split(";");
    for (const chunk of chunks) {
      if (!chunk) continue;
      const parts = chunk.split(",");
      if (parts.length < 2) continue;
      const x = finiteNumber(parts[0]);
      const y = finiteNumber(parts[1]);
      const d = finiteNumber(parts[2]);
      if (x === null || y === null) continue;
      const forward = x;
      const lateral = y;
      if (forward < -20 || forward > 1200 || Math.abs(lateral) > 80) continue;
      points.push({ forward, lateral, d: d === null ? forward : d });
      if (points.length >= 160) break;
    }
    return points;
  }

  function formatDistance(meters) {
    const value = finiteNumber(meters);
    if (value === null || value <= 0) return "";
    if (value < 950) return `${Math.round(value)}m`;
    return `${(value / 1000).toFixed(value < 10000 ? 1 : 0)}km`;
  }

  function formatDuration(seconds) {
    const value = finiteNumber(seconds);
    if (value === null || value <= 0) return "";
    const minutes = Math.max(1, Math.round(value / 60));
    if (minutes < 60) return `${minutes}분`;
    const hours = Math.floor(minutes / 60);
    const rest = minutes % 60;
    return rest ? `${hours}시간 ${rest}분` : `${hours}시간`;
  }

  function updateNavInfo() {
    if (!navInfo || !navRoad || !navMeta) return;
    if (!navState.active) {
      navInfo.hidden = true;
      navRoad.textContent = "";
      navMeta.textContent = "";
      return;
    }

    const road = navState.road || navState.turn?.text || "";
    const goalDist = formatDistance(navState.goal?.dist);
    const goalTime = formatDuration(navState.goal?.timeSec);
    const turnDist = formatDistance(navState.turn?.dist);
    const sdiDist = formatDistance(navState.sdi?.dist);
    const sdiLimit = finiteNumber(navState.sdi?.limit);
    const meta = [];
    if (goalDist) meta.push(goalTime ? `${goalDist} · ${goalTime}` : goalDist);
    if (turnDist) meta.push(`회전 ${turnDist}`);
    if (sdiDist) meta.push(sdiLimit && sdiLimit > 0 ? `${sdiLimit} 제한 ${sdiDist}` : `단속 ${sdiDist}`);

    if (!road && meta.length === 0) {
      navInfo.hidden = true;
      return;
    }
    navRoad.textContent = road || "경로 안내";
    navMeta.textContent = meta.slice(0, 2).join(" / ");
    navMeta.hidden = navMeta.textContent.length === 0;
    navInfo.hidden = false;
  }

  function clearNav() {
    if (!navState.active && navState.points.length === 0 && !navState.path) return;
    if (navState.staleTimer) {
      window.clearTimeout(navState.staleTimer);
      navState.staleTimer = 0;
    }
    navState.active = false;
    navState.path = "";
    navState.points = [];
    navState.road = "";
    navState.turn = null;
    navState.goal = null;
    navState.sdi = null;
    navState.origin = null;
    navState.heading = null;
    navState.updatedAt = 0;
    navState.dirty = true;
    updateNavInfo();
    renderOverlay();
    updateStatus();
  }

  function expireNavIfStale(now = Date.now()) {
    if (!navState.active || !navState.updatedAt) return false;
    if (now - navState.updatedAt <= NAV_STALE_MS) return false;
    clearNav();
    return true;
  }

  function setNav(payload) {
    const path = String(payload.path || "").trim();
    if (!payload.active || !path) {
      clearNav();
      return;
    }
    navState.active = true;
    navState.path = path;
    navState.points = parsePath(path);
    navState.road = String(payload.road || "");
    navState.turn = payload.turn || null;
    navState.goal = payload.goal || null;
    navState.sdi = payload.sdi || null;
    const originLat = finiteNumber(payload.origin?.lat);
    const originLon = finiteNumber(payload.origin?.lon);
    navState.origin = validLatLon(originLat, originLon)
      ? {
          lat: originLat,
          lon: originLon,
          distanceM: finiteNumber(payload.origin?.distanceM),
          index: finiteNumber(payload.origin?.index),
          ratio: finiteNumber(payload.origin?.ratio),
        }
      : null;
    navState.heading = finiteNumber(payload.heading);
    navState.updatedAt = Date.now();
    if (navState.staleTimer) window.clearTimeout(navState.staleTimer);
    navState.staleTimer = window.setTimeout(clearNav, NAV_STALE_MS + 150);
    navState.dirty = true;
    updateNavInfo();
    renderOverlay();
    updateStatus();
  }

  function routeBounds(coordinates) {
    if (!coordinates.length) return null;
    const bounds = {
      minLat: coordinates[0].lat,
      maxLat: coordinates[0].lat,
      minLon: coordinates[0].lon,
      maxLon: coordinates[0].lon,
    };
    for (const point of coordinates) {
      bounds.minLat = Math.min(bounds.minLat, point.lat);
      bounds.maxLat = Math.max(bounds.maxLat, point.lat);
      bounds.minLon = Math.min(bounds.minLon, point.lon);
      bounds.maxLon = Math.max(bounds.maxLon, point.lon);
    }
    return bounds;
  }

  function setRoute(payload) {
    const raw = Array.isArray(payload.coordinates) ? payload.coordinates : [];
    const coordinates = [];
    for (const point of raw) {
      const lat = finiteNumber(point?.lat ?? point?.latitude);
      const lon = finiteNumber(point?.lon ?? point?.longitude);
      if (!validLatLon(lat, lon)) continue;
      coordinates.push({ lat, lon });
      if (coordinates.length >= 1200) break;
    }
    routeState.active = Boolean(payload.active) && coordinates.length > 1;
    routeState.coordinates = routeState.active ? coordinates : [];
    routeState.bounds = routeState.active ? routeBounds(coordinates) : null;
    routeState.dirty = true;
    routeState.fitted = false;
    if (routeState.expanded) fitRouteView();
    applyMarkerPosition();
    renderOverlay();
    updateStatus();
  }

  function setExpanded(expanded) {
    routeState.expanded = Boolean(expanded);
    routeState.dirty = true;
    routeState.fitted = false;
    if (routeState.expanded) {
      fitRouteView();
    } else {
      applyKakaoPosition(state.lat, state.lon, true);
    }
    if (routeState.expanded && !routeState.active) applyKakaoPosition(state.lat, state.lon, true);
    applyMarkerPosition();
    renderOverlay();
  }

  function applyZoomBias(bias) {
    const next = Math.max(-3, Math.min(3, Math.round(Number(bias) || 0)));
    if (next === state.zoomBias) return;
    state.zoomBias = next;
    // Re-apply Kakao level immediately (force past the throttle) and redraw
    // the schematic overlay with the new view range.
    if (state.map && window.kakao?.maps) {
      applyKakaoPosition(state.lat, state.lon, true);
    }
    navState.dirty = true;
    routeState.dirty = true;
    renderOverlay();
    updateStatus();
  }

  function clearOverlay(ctx, width, height) {
    ctx.clearRect(0, 0, width, height);
  }

  function shouldUseMapProjection() {
    return state.provider === "kakao" && state.map && window.kakao?.maps;
  }

  function localPointToLatLng(point) {
    const originLat = finiteNumber(navState.origin?.lat) ?? finiteNumber(interp.display.lat) ?? state.lat;
    const originLon = finiteNumber(navState.origin?.lon) ?? finiteNumber(interp.display.lon) ?? state.lon;
    if (!validLatLon(originLat, originLon)) return null;

    const heading = finiteNumber(navState.heading) ?? finiteNumber(interp.display.heading) ?? state.heading ?? 0;
    const headingRad = heading * Math.PI / 180;
    const forward = finiteNumber(point?.forward) ?? 0;
    const lateral = finiteNumber(point?.lateral) ?? 0;
    const northMeters = forward * Math.cos(headingRad) - lateral * Math.sin(headingRad);
    const eastMeters = forward * Math.sin(headingRad) + lateral * Math.cos(headingRad);
    const latRad = originLat * Math.PI / 180;
    const nextLat = originLat + (northMeters / EARTH_RADIUS_M) * 180 / Math.PI;
    const nextLon = originLon + (eastMeters / (EARTH_RADIUS_M * Math.max(0.01, Math.cos(latRad)))) * 180 / Math.PI;
    return new window.kakao.maps.LatLng(nextLat, nextLon);
  }

  function projectedPathPointToCanvas(point) {
    if (!shouldUseMapProjection()) return null;
    try {
      const latlng = localPointToLatLng(point);
      const projected = latlng ? state.map.getProjection()?.containerPointFromCoords?.(latlng) : null;
      if (projected && Number.isFinite(projected.x) && Number.isFinite(projected.y)) {
        return { x: projected.x, y: projected.y };
      }
    } catch (_) {
      // Projection can be temporarily unavailable while Kakao is relayouting.
    }
    return null;
  }

  function summarizePathPoint(point) {
    if (!point) return null;
    let latlng = null;
    let projected = null;
    if (shouldUseMapProjection()) {
      try {
        latlng = localPointToLatLng(point);
        projected = latlng ? state.map.getProjection()?.containerPointFromCoords?.(latlng) : null;
      } catch (_) {
        latlng = null;
        projected = null;
      }
    }
    return {
      forward: finiteNumber(point.forward),
      lateral: finiteNumber(point.lateral),
      d: finiteNumber(point.d),
      lat: latlng?.getLat?.() ?? null,
      lon: latlng?.getLng?.() ?? null,
      canvasX: Number.isFinite(projected?.x) ? projected.x : null,
      canvasY: Number.isFinite(projected?.y) ? projected.y : null,
    };
  }

  function buildDebugSnapshot(reason = "") {
    const rect = overlayCanvas?.getBoundingClientRect?.();
    const center = state.map?.getCenter?.();
    const points = navState.points || [];
    const sampleIndexes = points.length
      ? Array.from(new Set([0, Math.floor(points.length / 2), points.length - 1]))
      : [];
    return {
      reason,
      ts: Date.now(),
      provider: state.provider,
      status: state.status,
      error: state.error,
      map: {
        projection: shouldUseMapProjection(),
        level: state.map?.getLevel?.() ?? null,
        centerLat: center?.getLat?.() ?? null,
        centerLon: center?.getLng?.() ?? null,
      },
      vehicle: {
        lat: state.lat,
        lon: state.lon,
        heading: state.heading,
        speed: state.speed,
        displayLat: interp.display.lat,
        displayLon: interp.display.lon,
        displayHeading: interp.display.heading,
      },
      canvas: {
        cssWidth: rect?.width ?? 0,
        cssHeight: rect?.height ?? 0,
        width: overlayCanvas?.width || 0,
        height: overlayCanvas?.height || 0,
        dpr: Math.min(window.devicePixelRatio || 1, 2),
      },
      nav: {
        active: navState.active,
        points: points.length,
        pathLength: navState.path.length,
        updatedAgeMs: navState.updatedAt ? Date.now() - navState.updatedAt : null,
        origin: navState.origin,
        heading: navState.heading,
        road: navState.road,
        turn: navState.turn,
        goal: navState.goal,
        sdi: navState.sdi,
        projectionSig: navState.lastProjectionSig,
        samples: sampleIndexes.map((index) => ({ index, ...summarizePathPoint(points[index]) })),
      },
      route: {
        active: routeState.active,
        expanded: routeState.expanded,
        coordinates: routeState.coordinates.length,
      },
      options: {
        headingUp: state.overlayHeadingUp,
        curvatureColor: state.curvatureColor,
        themeMode: state.themeMode,
        displayTheme: state.displayTheme,
      },
    };
  }

  function safeDebugSnapshot(reason = "") {
    try {
      return buildDebugSnapshot(reason);
    } catch (error) {
      return {
        reason,
        ts: Date.now(),
        provider: state.provider,
        status: state.status,
        error: error?.message || "debug_snapshot_failed",
      };
    }
  }

  function postDebugSnapshot(reason = "", force = false) {
    if (!force) return;
    const now = Date.now();
    if (!force && now - state.lastDebugPostAt < 1000) return;
    state.lastDebugPostAt = now;
    try {
      if (window.parent && window.parent !== window) {
        window.parent.postMessage({
          source: "carrot-kmap",
          type: "debug-snapshot",
          snapshot: safeDebugSnapshot(reason),
        }, "*");
      }
    } catch (_) {
      // Standalone file preview can ignore parent messaging failures.
    }
  }

  function requestOverlayRender(reason = "") {
    navState.dirty = true;
    routeState.dirty = true;
    if (state.overlayRaf) return;
    state.overlayRaf = window.requestAnimationFrame(() => {
      state.overlayRaf = 0;
      renderOverlay();
      postDebugSnapshot(reason);
    });
  }

  function pathPointToCanvas(point, cx, cy, pxPerMeter) {
    const projected = projectedPathPointToCanvas(point);
    if (projected) return projected;

    const headingRad = state.overlayHeadingUp ? 0 : (interp.display.heading || state.heading || 0) * Math.PI / 180;
    const sin = Math.sin(headingRad);
    const cos = Math.cos(headingRad);
    return {
      x: cx + (point.lateral * cos + point.forward * sin) * pxPerMeter,
      y: cy + (point.lateral * sin - point.forward * cos) * pxPerMeter,
    };
  }

  function pathDistance(point) {
    const distance = finiteNumber(point?.d);
    return distance === null ? finiteNumber(point?.forward) ?? 0 : distance;
  }

  function pointAlongPath(points, targetDistance) {
    const target = finiteNumber(targetDistance);
    if (!Array.isArray(points) || points.length === 0 || target === null || target < 0) return null;
    let previous = points[0];
    let previousDistance = pathDistance(previous);
    if (target <= previousDistance) return previous;
    for (let index = 1; index < points.length; index += 1) {
      const current = points[index];
      const currentDistance = pathDistance(current);
      if (target <= currentDistance) {
        const span = Math.max(0.001, currentDistance - previousDistance);
        const ratio = Math.max(0, Math.min(1, (target - previousDistance) / span));
        return {
          forward: previous.forward + (current.forward - previous.forward) * ratio,
          lateral: previous.lateral + (current.lateral - previous.lateral) * ratio,
          d: target,
        };
      }
      previous = current;
      previousDistance = currentDistance;
    }
    return points[points.length - 1];
  }

  function drawTurnMarker(ctx, cx, cy, pxPerMeter, minY, maxY, width) {
    const turnDistance = finiteNumber(navState.turn?.dist);
    if (turnDistance === null || turnDistance <= 0 || !navState.points.length) return;
    const point = pointAlongPath(navState.points, turnDistance);
    if (!point || point.forward < minY || point.forward > maxY) return;

    const before = pointAlongPath(navState.points, Math.max(0, turnDistance - 10)) || point;
    const after = pointAlongPath(navState.points, turnDistance + 10) || point;
    const canvasPoint = pathPointToCanvas(point, cx, cy, pxPerMeter);
    const beforePoint = pathPointToCanvas(before, cx, cy, pxPerMeter);
    const afterPoint = pathPointToCanvas(after, cx, cy, pxPerMeter);
    const angle = Math.atan2(afterPoint.y - beforePoint.y, afterPoint.x - beforePoint.x) + Math.PI / 2;
    const radius = Math.max(11, Math.min(18, width * 0.038));

    ctx.save();
    ctx.translate(canvasPoint.x, canvasPoint.y);
    ctx.rotate(angle);
    ctx.beginPath();
    ctx.arc(0, 0, radius * 0.9, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(9, 12, 16, .62)";
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(0, -radius);
    ctx.lineTo(radius * 0.72, radius * 0.76);
    ctx.lineTo(0, radius * 0.42);
    ctx.lineTo(-radius * 0.72, radius * 0.76);
    ctx.closePath();
    ctx.strokeStyle = "rgba(0, 0, 0, .55)";
    ctx.lineWidth = Math.max(2, radius * 0.18);
    ctx.stroke();
    ctx.fillStyle = "rgba(255, 125, 32, .96)";
    ctx.fill();
    ctx.beginPath();
    ctx.arc(0, radius * 0.18, Math.max(2.2, radius * 0.17), 0, Math.PI * 2);
    ctx.fillStyle = "rgba(255,255,255,.82)";
    ctx.fill();
    ctx.restore();
  }

  function drawPathColorStroke(ctx, points, cx, cy, pxPerMeter, width, maxY) {
    const canvasPoints = points.map((point) => pathPointToCanvas(point, cx, cy, pxPerMeter));
    drawSmoothedPath(ctx, canvasPoints);
    const gradient = ctx.createLinearGradient(cx, cy, cx, Math.max(0, cy - maxY * pxPerMeter));
    gradient.addColorStop(0, "rgba(255, 190, 72, .99)");
    gradient.addColorStop(0.48, state.curvatureColor ? "rgba(255, 136, 38, .97)" : "rgba(255, 132, 42, .97)");
    gradient.addColorStop(1, state.curvatureColor ? "rgba(255, 86, 42, .96)" : "rgba(255, 92, 42, .96)");
    ctx.strokeStyle = gradient;
    ctx.lineWidth = Math.max(9, Math.min(16, width * 0.032));
    ctx.stroke();
  }

  function fitRouteView() {
    if (!routeState.expanded || !routeState.active || !routeState.bounds || !state.map || !window.kakao?.maps) return;
    try {
      const bounds = new window.kakao.maps.LatLngBounds();
      for (const point of routeState.coordinates) {
        bounds.extend(new window.kakao.maps.LatLng(point.lat, point.lon));
      }
      const rect = kakaoMapEl?.getBoundingClientRect?.();
      const shortSide = Math.max(1, Math.min(rect?.width || 0, rect?.height || 0));
      const pad = Math.round(Math.max(ROUTE_FIT_PADDING_MIN_PX, shortSide * ROUTE_FIT_PADDING_RATIO));
      window.kakao.maps.event.trigger(state.map, "resize");
      state.map.setBounds(bounds, pad, pad, pad, pad);
      routeState.fitted = true;
      routeState.dirty = true;
    } catch (_) {
      // If Kakao projection is not ready yet, the canvas fallback still draws the route.
    }
  }

  function routePointToCanvas(point, bounds, width, height) {
    try {
      const projection = state.map?.getProjection?.();
      const projected = projection?.containerPointFromCoords?.(new window.kakao.maps.LatLng(point.lat, point.lon));
      if (projected && Number.isFinite(projected.x) && Number.isFinite(projected.y)) {
        return { x: projected.x, y: projected.y };
      }
    } catch (_) {
      // Fall through to normalized bounds projection.
    }
    const pad = Math.max(18, Math.min(width, height) * 0.08);
    const lonSpan = Math.max(0.000001, bounds.maxLon - bounds.minLon);
    const latSpan = Math.max(0.000001, bounds.maxLat - bounds.minLat);
    return {
      x: pad + ((point.lon - bounds.minLon) / lonSpan) * Math.max(1, width - pad * 2),
      y: pad + ((bounds.maxLat - point.lat) / latSpan) * Math.max(1, height - pad * 2),
    };
  }

  function drawSmoothedPath(ctx, canvasPoints) {
    if (!Array.isArray(canvasPoints) || canvasPoints.length < 2) return false;
    ctx.beginPath();
    ctx.moveTo(canvasPoints[0].x, canvasPoints[0].y);
    if (canvasPoints.length === 2) {
      ctx.lineTo(canvasPoints[1].x, canvasPoints[1].y);
      return true;
    }
    for (let index = 1; index < canvasPoints.length - 1; index += 1) {
      const current = canvasPoints[index];
      const next = canvasPoints[index + 1];
      const midX = (current.x + next.x) / 2;
      const midY = (current.y + next.y) / 2;
      ctx.quadraticCurveTo(current.x, current.y, midX, midY);
    }
    const last = canvasPoints[canvasPoints.length - 1];
    ctx.lineTo(last.x, last.y);
    return true;
  }

  function renderFullRoute(ctx, width, height) {
    if (!routeState.expanded || !routeState.active || !routeState.bounds || routeState.coordinates.length < 2) return false;
    const points = routeState.coordinates.map((point) => routePointToCanvas(point, routeState.bounds, width, height));
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    drawSmoothedPath(ctx, points);
    ctx.strokeStyle = "rgba(0, 0, 0, .28)";
    ctx.lineWidth = Math.max(4.5, Math.min(8, width * 0.010));
    ctx.stroke();

    drawSmoothedPath(ctx, points);
    ctx.strokeStyle = "rgba(255, 128, 38, .96)";
    ctx.lineWidth = Math.max(3, Math.min(5.5, width * 0.0068));
    ctx.stroke();
    routeState.dirty = false;
    return true;
  }

  function renderOverlay() {
    if (!overlayCanvas) return;
    const resized = resizeOverlayCanvas();
    const ctx = overlayCanvas.getContext("2d");
    if (!ctx) return;

    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = overlayCanvas.width / dpr;
    const height = overlayCanvas.height / dpr;
    const viewRange = routeState.expanded && !routeState.active ? expandedFallbackRange(state.speed) : viewRangeMeters(state.speed);
    expireNavIfStale();
    if (Math.abs(viewRange - navState.lastViewRange) > 1) {
      navState.lastViewRange = viewRange;
      navState.dirty = true;
    }
    const projectedPath = shouldUseMapProjection();
    if (projectedPath && navState.active) {
      const projectionSig = [
        navState.origin ? navState.origin.lat.toFixed(6) : "",
        navState.origin ? navState.origin.lon.toFixed(6) : "",
        Number.isFinite(navState.heading) ? Math.round(navState.heading) : "",
        interp.display.lat.toFixed(6),
        interp.display.lon.toFixed(6),
        Math.round(interp.display.heading),
        state.map?.getLevel?.() ?? "",
        Math.round(width),
        Math.round(height),
      ].join("|");
      if (projectionSig !== navState.lastProjectionSig) {
        navState.lastProjectionSig = projectionSig;
        navState.dirty = true;
      }
    }
    if (!navState.dirty && !routeState.dirty && !resized) return;

    ctx.save();
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    clearOverlay(ctx, width, height);
    if (routeState.expanded && routeState.active) {
      if (renderFullRoute(ctx, width, height)) {
        ctx.restore();
        navState.dirty = false;
        return;
      }
    }
    if (!navState.active || navState.points.length < 2) {
      ctx.restore();
      navState.dirty = false;
      return;
    }

    const cx = width / 2;
    const cy = height / 2;
    const pxPerMeter = height / viewRange;
    const maxY = viewRange * 0.66;
    const minY = -viewRange * 0.18;
    const visible = projectedPath ? navState.points : navState.points.filter((point) => point.forward >= minY && point.forward <= maxY);
    if (visible.length < 2) {
      ctx.restore();
      navState.dirty = false;
      return;
    }

    ctx.lineCap = "round";
    ctx.lineJoin = "round";
    const visibleCanvasPoints = visible.map((point) => pathPointToCanvas(point, cx, cy, pxPerMeter));
    drawSmoothedPath(ctx, visibleCanvasPoints);
    ctx.strokeStyle = "rgba(0, 0, 0, .34)";
    ctx.lineWidth = Math.max(11, Math.min(20, width * 0.039));
    ctx.stroke();

    drawPathColorStroke(ctx, visible, cx, cy, pxPerMeter, width, maxY);

    // Turn-point marker removed per user request — the route line itself
    // already conveys the turn; the extra icon cluttered the map. (Function
    // kept for possible future opt-in.)
    ctx.restore();
    navState.dirty = false;
  }

  function relayoutKakaoMap() {
    if (!state.map || !window.kakao?.maps) return;
    try {
      const resized = resizeOverlayCanvas();
      window.kakao.maps.event.trigger(state.map, "resize");
      if (routeState.expanded && routeState.active) {
        fitRouteView();
        requestOverlayRender("relayout-route");
        return;
      }
      applyKakaoPosition(state.lat, state.lon, true);
      if (resized) {
        navState.dirty = true;
        routeState.dirty = true;
      }
      requestOverlayRender("relayout");
    } catch (_) {
      // Resize events can race while the iframe is still settling.
    }
  }

  function bindMapResizeObserver() {
    if (typeof ResizeObserver !== "function" || !kakaoMapEl) return;
    const observer = new ResizeObserver(() => {
      window.requestAnimationFrame(relayoutKakaoMap);
    });
    observer.observe(kakaoMapEl);
  }

  function applyMotionState() {
    root.dataset.motion = motionForSpeed(state.speed);
    root.style.setProperty("--kmap-level", String(state.level));
  }

  function setKakaoLevel(position, force = false) {
    if (!state.map) return;
    const now = Date.now();
    const level = kakaoDisplayLevel();
    if (level === state.map.getLevel?.() || (!force && now - state.lastLevelChangeAt <= 2800)) return;
    state.map.setLevel(level, { animate: false, anchor: position });
    state.lastLevelChangeAt = now;
  }

  function easeOutCubic(t) {
    const c = 1 - t;
    return 1 - c * c * c;
  }

  function lerpAngle(a, b, t) {
    const diff = (((b - a) % 360) + 540) % 360 - 180;
    return ((a + diff * t) % 360 + 360) % 360;
  }

  function lerp(a, b, t) {
    return a + (b - a) * t;
  }

  function setMode(mode) {
    state.mode = "box";
    root.dataset.mode = state.mode;
  }

  function setProvider(provider) {
    state.provider = provider === "kakao" ? "kakao" : "mock";
    root.dataset.provider = state.provider;
  }

  function applyOverlayOptions(params) {
    state.overlayHeadingUp = boolParam(params, "heading_up", true);
    state.curvatureColor = boolParam(params, "curvature", false);
    const mt = String(params.get("map_type") || "roadmap").toLowerCase();
    state.mapType = (mt === "satellite" || mt === "hybrid") ? mt : "roadmap";
    applyKakaoMapType();   // no-op until the Kakao map exists
    setDisplayTheme();
    root.dataset.headingUp = state.overlayHeadingUp ? "1" : "0";
    root.dataset.curvature = state.curvatureColor ? "1" : "0";
    root.dataset.mapType = state.mapType;
    navState.dirty = true;
    routeState.dirty = true;
  }

  // Apply the selected base layer to the live Kakao map (roadmap / satellite
  // (SKYVIEW) / hybrid). Safe no-op before the map is created or on SDKs that
  // lack setMapTypeId.
  function applyKakaoMapType() {
    if (!state.map || !window.kakao?.maps?.MapTypeId) return;
    const M = window.kakao.maps.MapTypeId;
    const id = state.mapType === "satellite" ? M.SKYVIEW
             : state.mapType === "hybrid" ? M.HYBRID
             : M.ROADMAP;
    try { state.map.setMapTypeId(id); } catch (_) {}
  }

  function updateStatus() {
    if (!statusText) return;
    const label = state.provider;
    const age = state.lastTs ? Math.max(0, Math.round((Date.now() - state.lastTs) / 1000)) : 0;
    const parts = [
      label,
      state.status,
      `${state.lat.toFixed(5)}, ${state.lon.toFixed(5)}`,
      `${Math.round(state.heading)}deg`,
      `${Math.round(state.speed)}km/h`,
      `L${state.level}`,
      `${age}s`,
    ];
    if (navState.active) parts.push(`P${navState.points.length}`);
    if (routeState.active) parts.push(`R${routeState.coordinates.length}`);
    if (routeState.expanded) parts.push("expanded");
    if (!state.overlayHeadingUp) parts.push("north-up");
    if (state.curvatureColor) parts.push("curve");
    if (state.error) parts.push(state.error);
    statusText.textContent = parts.join(" / ");
  }

  function updateMockPan(lat, lon) {
    const x = Math.round((((lon * 10000) % 90) - 45) * 0.8);
    const y = Math.round((((lat * 10000) % 90) - 45) * 0.8);
    surface.style.setProperty("--map-pan-x", `${x}px`);
    surface.style.setProperty("--map-pan-y", `${y}px`);
  }

  function applyMarkerRotation(heading) {
    const rotation = `${heading}deg`;
    marker.style.setProperty("--heading", rotation);
  }

  function applyMarkerPosition() {
    if (routeState.expanded && routeState.active && routeState.bounds) {
      const point = routePointToCanvas({ lat: state.lat, lon: state.lon }, routeState.bounds, overlayCanvas?.clientWidth || 1, overlayCanvas?.clientHeight || 1);
      marker.style.setProperty("--vehicle-marker-left", `${point.x}px`);
      marker.style.setProperty("--vehicle-marker-top", `${point.y}px`);
      return;
    }
    marker.style.setProperty("--vehicle-marker-left", "50%");
    marker.style.setProperty("--vehicle-marker-top", "50%");
  }

  function applyKakaoPosition(lat, lon, forceLevel = false) {
    if (!state.map || !window.kakao?.maps) return;
    if (routeState.expanded && routeState.active) return;
    if (!validLatLon(lat, lon)) return;
    const position = new window.kakao.maps.LatLng(lat, lon);
    const now = Date.now();
    const expanded = routeState.expanded && !routeState.active;
    const parked = state.speed < 3;
    const slow = state.speed < 12;
    const minInterval = expanded
      ? MAP_CENTER_EXPANDED_INTERVAL_MS
      : parked
        ? MAP_CENTER_PARKED_INTERVAL_MS
        : slow
          ? MAP_CENTER_SLOW_INTERVAL_MS
          : MAP_CENTER_DEFAULT_INTERVAL_MS;
    const minDistance = expanded
      ? MAP_CENTER_EXPANDED_DISTANCE_M
      : parked
        ? MAP_CENTER_PARKED_DISTANCE_M
        : slow
          ? MAP_CENTER_SLOW_DISTANCE_M
          : MAP_CENTER_DEFAULT_DISTANCE_M;
    const moved = distanceMeters(state.lastMapCenterLat, state.lastMapCenterLon, lat, lon);
    const firstCenter = state.lastMapCenterLat === null || state.lastMapCenterLon === null;
    const shouldCenter = forceLevel || firstCenter || (now - state.lastMapCenterAt >= minInterval && moved >= minDistance);
    if (shouldCenter) {
      state.map.setCenter(position);
      state.lastMapCenterLat = lat;
      state.lastMapCenterLon = lon;
      state.lastMapCenterAt = now;
    }
    setKakaoLevel(position, forceLevel);
    if (shouldCenter) requestOverlayRender("position");
  }

  function renderDisplay() {
    applyMarkerRotation(interp.display.heading);
    applyMarkerPosition();
    updateMockPan(interp.display.lat, interp.display.lon);
    if (state.provider !== "kakao") {
      postDebugSnapshot("render");
      return;
    }
    applyKakaoPosition(interp.display.lat, interp.display.lon);
    renderOverlay();
    postDebugSnapshot("render");
  }

  function ensureRenderLoop() {
    if (interp.raf) return;
    const step = () => {
      interp.raf = 0;
      const now = performance.now();
      const elapsed = now - interp.segmentStart;
      const duration = Math.max(60, interp.durationMs);
      const tRaw = Math.min(1, elapsed / duration);
      const t = easeOutCubic(tRaw);
      interp.display.lat = lerp(interp.source.lat, interp.target.lat, t);
      interp.display.lon = lerp(interp.source.lon, interp.target.lon, t);
      interp.display.heading = lerpAngle(interp.source.heading, interp.target.heading, t);
      renderDisplay();
      // Keep ticking while we haven't reached target, or for a small grace
      // window after to absorb late samples without visible stutter.
      const idleMs = now - interp.lastSampleAt;
      if (tRaw < 1 || idleMs < 2500) {
        interp.raf = window.requestAnimationFrame(step);
      } else {
        interp.active = false;
      }
    };
    interp.active = true;
    interp.raf = window.requestAnimationFrame(step);
  }

  function seedInterp(lat, lon, heading) {
    interp.source.lat = interp.target.lat = interp.display.lat = lat;
    interp.source.lon = interp.target.lon = interp.display.lon = lon;
    interp.source.heading = interp.target.heading = interp.display.heading = heading;
    interp.segmentStart = performance.now();
    interp.lastSampleAt = interp.segmentStart;
  }

  function pushSample(lat, lon, heading) {
    const now = performance.now();
    // Estimate sample interval from observed cadence so interpolation tracks
    // the actual upstream rate (1Hz vs 2Hz vs render-request bursts).
    if (interp.lastSampleAt > 0) {
      const dt = now - interp.lastSampleAt;
      if (dt > INTERP_MIN_MS && dt < 4000) {
        interp.durationMs = Math.max(INTERP_MIN_MS, Math.min(INTERP_MAX_MS, dt * 1.05));
      }
    } else {
      interp.durationMs = INTERP_BASE_MS;
    }
    interp.source.lat = interp.display.lat;
    interp.source.lon = interp.display.lon;
    interp.source.heading = interp.display.heading;
    interp.target.lat = lat;
    interp.target.lon = lon;
    interp.target.heading = heading;
    interp.segmentStart = now;
    interp.lastSampleAt = now;
    ensureRenderLoop();
  }

  function loadScript(src) {
    return new Promise((resolve, reject) => {
      const existing = document.querySelector(`script[data-kmap-sdk="kakao"]`);
      if (existing) {
        existing.addEventListener("load", resolve, { once: true });
        existing.addEventListener("error", () => reject(new Error("kakao_sdk_load_failed")), { once: true });
        return;
      }

      const script = document.createElement("script");
      script.src = src;
      script.async = true;
      script.defer = true;
      script.dataset.kmapSdk = "kakao";
      script.onload = resolve;
      script.onerror = () => reject(new Error("kakao_sdk_load_failed"));
      document.head.appendChild(script);
    });
  }

  function waitForKakaoLoad() {
    return new Promise((resolve, reject) => {
      if (!window.kakao?.maps?.load) {
        reject(new Error("kakao_sdk_unavailable"));
        return;
      }
      window.kakao.maps.load(resolve);
    });
  }

  function initKakaoMap() {
    if (!window.kakao?.maps || !kakaoMapEl || state.map) return;
    const center = new window.kakao.maps.LatLng(state.lat, state.lon);
    state.map = new window.kakao.maps.Map(kakaoMapEl, {
      center,
      level: state.level,
      draggable: false,
      scrollwheel: false,
      disableDoubleClickZoom: true,
      keyboardShortcuts: false,
      tileAnimation: false,
    });
    state.map.setMinLevel?.(KAKAO_MIN_LEVEL);
    state.map.setMaxLevel?.(KAKAO_MAX_LEVEL);
    applyKakaoMapType();
    if (state.map.setCopyrightPosition && window.kakao.maps.CopyrightPosition) {
      state.map.setCopyrightPosition(window.kakao.maps.CopyrightPosition.BOTTOMRIGHT, true);
    }
    for (const eventName of ["center_changed", "zoom_changed", "bounds_changed", "idle"]) {
      try {
        window.kakao.maps.event.addListener(state.map, eventName, () => requestOverlayRender(eventName));
      } catch (_) {
        // Older SDK surfaces can ignore optional event hooks.
      }
    }

    // Marker stays as a shell-positioned div (#vehicleMarker) instead of a
    // Kakao CustomOverlay child of the map. This keeps the marker outside
    // the map's opacity/filter stack so we can fade the map underneath
    // without dimming the marker. The map auto-centers on the vehicle each
    // frame, so a fixed 50%/50% marker visually tracks position.
    applyMarkerRotation(interp.display.heading);
    setProvider("kakao");
    if (state.status === "idle") state.status = "waiting";
  }

  async function initProvider() {
    const params = new URLSearchParams(window.location.search);
    if (params.get("mock") === "1") {
      setProvider("mock");
      return;
    }

    try {
      const appkey = params.get("appkey") || KAKAO_JS_KEY;
      const sdkUrl = `${KAKAO_SDK_URL}?appkey=${encodeURIComponent(appkey)}&autoload=false`;
      await Promise.race([
        loadScript(sdkUrl).then(waitForKakaoLoad),
        new Promise((_, reject) => window.setTimeout(() => reject(new Error("kakao_sdk_timeout")), SDK_TIMEOUT_MS)),
      ]);
      state.kakaoReady = true;
      state.sdkLoadedAt = Date.now();
      initKakaoMap();
    } catch (error) {
      setProvider("mock");
      postError(error?.message || "kakao_sdk_load_failed", { soft: true });
    }
  }

  function applyVehicle(payload) {
    const lat = finiteNumber(payload.lat);
    const lon = finiteNumber(payload.lon);
    if (!validLatLon(lat, lon)) {
      root.dataset.status = "invalid";
      state.status = "invalid";
      updateStatus();
      return false;
    }

    const speed = finiteNumber(payload.speed);
    const isFirstSample = state.lastTs === 0;
    state.lat = lat;
    state.lon = lon;
    state.heading = normalizeHeading(payload.heading);
    state.speed = speed === null ? state.speed : Math.max(0, speed);
    state.lastTs = finiteNumber(payload.ts) || Date.now();
    state.hasVehicle = true;
    root.dataset.hasVehicle = "1";
    setMode(state.mode);
    state.level = levelForSpeed(state.speed, state.level);
    applyMotionState();

    if (isFirstSample) {
      seedInterp(state.lat, state.lon, state.heading);
      renderDisplay();
    } else {
      pushSample(state.lat, state.lon, state.heading);
    }
    state.error = "";
    state.status = "ready";
    updateStatus();
    root.dataset.status = "ready";
    return true;
  }

  function handleMessage(event) {
    const data = event.data || {};
    if (data.source !== "carrot-vision") return;
    if (data.type === "vehicle") {
      applyVehicle(data);
    } else if (data.type === "nav") {
      setNav(data);
    } else if (data.type === "route") {
      setRoute(data);
    } else if (data.type === "expanded") {
      setExpanded(data.expanded);
    } else if (data.type === "zoom-bias") {
      applyZoomBias(data.bias);
    } else if (data.type === "debug-request") {
      if (state.status === "ready" || state.status === "fallback") {
        postReady("debug-request");
      }
      postDebugSnapshot("request", true);
    }
  }

  function postReady(reason = "ready", extra = {}) {
    try {
      if (window.parent && window.parent !== window) {
        window.parent.postMessage({
          source: "carrot-kmap",
          type: "ready",
          provider: state.provider,
          // sdkLoadedAt is only non-zero when the Kakao SDK actually executed
          // (= 1 quota count). Parent uses this to track daily SDK load count.
          sdkLoadedAt: state.provider === "kakao" ? state.sdkLoadedAt || Date.now() : 0,
          snapshot: safeDebugSnapshot(reason),
          ...extra,
        }, "*");
      }
    } catch (_) {
      // Standalone file preview can ignore parent messaging failures.
    }
  }

  function postToggleExpanded() {
    try {
      if (window.parent && window.parent !== window) {
        window.parent.postMessage({
          source: "carrot-kmap",
          type: "toggle-expanded",
        }, "*");
      }
    } catch (_) {
      // Standalone file preview can ignore parent messaging failures.
    }
  }

  function postError(error, options = {}) {
    state.error = error || "";
    state.status = options.soft ? "fallback" : "error";
    updateStatus();
    if (options.soft) {
      postReady("fallback", { error, fallback: "mock" });
      return;
    }
    try {
      if (window.parent && window.parent !== window) {
        window.parent.postMessage({
          source: "carrot-kmap",
          type: "error",
          provider: state.provider,
          error,
          fallback: "",
          snapshot: safeDebugSnapshot(options.soft ? "fallback" : "error"),
        }, "*");
      }
    } catch (_) {
      // Standalone file preview can ignore parent messaging failures.
    }
  }

  function runDemoStep() {
    const next = {
      source: "carrot-vision",
      type: "vehicle",
      lat: state.lat + 0.0008,
      lon: state.lon + 0.0011,
      heading: state.heading + 28,
      speed: state.speed >= 100 ? 8 : state.speed + 18,
      ts: Date.now(),
    };
    applyVehicle(next);
  }

  function initDemoControls() {
    const params = new URLSearchParams(window.location.search);
    const embedded = window.parent && window.parent !== window;
    root.dataset.embedded = embedded ? "1" : "0";
    applyOverlayOptions(params);
    if (embedded || params.get("demo") === "0") {
      demoPanel.hidden = true;
    } else {
      demoMove.addEventListener("click", runDemoStep);
      demoMode.hidden = true;
    }
    const requestedMode = params.get("mode");
    if (requestedMode) setMode(requestedMode);
  }

  async function init() {
    window.addEventListener("message", handleMessage);
    root.addEventListener("click", (event) => {
      if (event.target?.closest?.("button")) return;
      postToggleExpanded();
    }, true);
    initDemoControls();
    state.status = "waiting";
    root.dataset.status = "waiting";
    applyMotionState();
    bindMapResizeObserver();
    resizeOverlayCanvas();
    window.addEventListener("resize", () => {
      relayoutKakaoMap();
      navState.dirty = true;
      routeState.dirty = true;
      renderOverlay();
    });
    await initProvider();
    updateStatus();
    postReady();
  }

  window.KmapDebug = {
    snapshot: buildDebugSnapshot,
    post: () => postDebugSnapshot("manual", true),
  };

  init().catch((error) => {
    postError(error?.message || "kmap_init_failed");
  });
})();
