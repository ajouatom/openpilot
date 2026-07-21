/* 승인 AR 컴포넌트와 world/stage adapter의 유일한 Three.js 경계.
 * 다른 AR 모듈은 descriptor, painter, layout, projection만 만들고 Three type을 알지 않는다.
 * 번들 크기를 위해 필요한 심볼만 named import 한다(esbuild tree-shaking 대상).
 */

import {
  Camera,
  CanvasTexture,
  Color,
  LinearFilter,
  Group,
  Mesh,
  MeshBasicMaterial,
  PlaneGeometry,
  Scene,
  SRGBColorSpace,
  WebGLRenderer,
  DoubleSide,
  FrontSide,
} from "three";

import {
  AR_MARKER_FORM,
  AR_MARKER_KIND,
  AR_BILLBOARD,
  AR_RENDER,
  markerForm,
} from "./tokens.js";
import {
  AR_LEGIBILITY,
  AR_OPACITY,
  AR_SHAPE,
} from "./design_tokens.js";
import { paintSignboard, textureSizeFor } from "./signboard.js";
import { signboardLayout } from "./three_signboard_layout.js";
import { markerWorldMatrix, stageProjectionMatrix } from "./three_projection.js";
import { projectPoint } from "./projection.js";
import { planMarker, selectVisibleMarkers } from "./responsive.js";
import { markerLifecycleSlot } from "./marker_identity.js";
import {
  constrainedBillboardAnchor,
  roadFrameFields,
  roadFrameForAnchor,
  roadFrameFromForward,
} from "./road_frame.js";
import { createMarkerPresentationFilter } from "./presentation_filter.js";

function defaultCanvasFactory(width, height) {
  if (typeof OffscreenCanvas !== "undefined") return new OffscreenCanvas(width, height);
  if (typeof document !== "undefined") {
    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    return canvas;
  }
  throw new Error("Three signboard rendering requires a canvasFactory");
}

function configureTexture(texture, anisotropy, options = {}) {
  texture.colorSpace = SRGBColorSpace;
  texture.minFilter = LinearFilter;
  texture.magFilter = LinearFilter;
  texture.anisotropy = Math.max(1, Math.min(16, Number(anisotropy) || 1));
  if (typeof options.flipY === "boolean") texture.flipY = options.flipY;
  if (options.flipX === true && texture.repeat && texture.offset) {
    // Carrot의 calibrated device projection은 일반 Three camera의 view-X와
    // 반대 handedness를 가진다. UV만 0..1 범위 안에서 정규화해 글자 방향을
    // 바로잡고, world anchor/heading 행렬에는 반사(scale -1)를 넣지 않는다.
    texture.repeat.x = -1;
    texture.offset.x = 1;
  }
  texture.needsUpdate = true;
  return texture;
}

function paintedTexture(descriptor, canvasFactory) {
  const { width, height } = textureSizeFor(descriptor);
  const canvas = canvasFactory(width, height);
  canvas.width = width;
  canvas.height = height;
  const context = canvas.getContext?.("2d");
  if (!context) throw new Error("Three signboard canvas has no 2D context");
  paintSignboard(context, descriptor);
  return new CanvasTexture(canvas);
}

function shadowTexture(canvasFactory) {
  const canvas = canvasFactory(128, 128);
  canvas.width = 128;
  canvas.height = 128;
  const context = canvas.getContext?.("2d");
  if (!context) throw new Error("Three signboard shadow canvas has no 2D context");
  const gradient = context.createRadialGradient(64, 64, 4, 64, 64, 62);
  gradient.addColorStop(0, `rgba(0,0,0,${AR_LEGIBILITY.shadowPlaneOpacity})`);
  gradient.addColorStop(1, "rgba(0,0,0,0)");
  context.fillStyle = gradient;
  context.fillRect(0, 0, 128, 128);
  return configureTexture(new CanvasTexture(canvas), 1);
}

function clamp01(value) {
  return Math.max(0, Math.min(1, Number(value) || 0));
}

/**
 * Build the approved preview signboard as a Three.Group.
 *
 * `textureFactory` and `shadowTextureFactory` are injectable so the geometry
 * can be checked in Node without a DOM. Production callers normally provide
 * only canvasFactory (OffscreenCanvas in a Worker, HTMLCanvasElement elsewhere).
 */
export function createThreeSignboardGroup(descriptor, {
  canvasFactory = defaultCanvasFactory,
  textureFactory = null,
  shadowTextureFactory = null,
  anisotropy = 8,
  mountLiftM = 0,
} = {}) {
  const layout = signboardLayout(descriptor, { mountLiftM });
  const group = new Group();
  group.name = `signboard:${descriptor.shape}`;

  const faceTexture = configureTexture(
    textureFactory
      ? textureFactory(descriptor)
      : paintedTexture(descriptor, canvasFactory),
    anisotropy,
    // OffscreenCanvas의 위/아래 행은 현재 production worker에서 false일 때
    // 정상이다. calibrated projection의 X handedness만 별도로 정규화한다.
    // markerWorldMatrix already maps preview-right to screen-right. A second
    // UV reflection renders otherwise readable labels backwards (40m -> m04).
    { flipY: false, flipX: false },
  );
  const faceMaterial = new MeshBasicMaterial({
    map: faceTexture,
    transparent: true,
    // 읽을 수 있는 표지는 앞면만 존재한다. DoubleSide는 잘못된 winding/면 방향도
    // 그려서 거울상·뒤집힌 글자를 정상처럼 통과시킨다.
    side: FrontSide,
    depthWrite: false,
    opacity: clamp01(descriptor.opacity * AR_OPACITY.group),
  });
  const face = new Mesh(
    new PlaneGeometry(layout.face.widthM, layout.face.heightM),
    faceMaterial,
  );
  face.name = "signboard:face";
  face.position.set(layout.face.xM, layout.face.yM, layout.face.zM);
  face.rotation.x = layout.face.rotationXRad;
  group.add(face);

  if (layout.shadow) {
    const groundTexture = configureTexture(
      shadowTextureFactory ? shadowTextureFactory(descriptor) : shadowTexture(canvasFactory),
      1,
    );
    const shadow = new Mesh(
      new PlaneGeometry(layout.shadow.widthM, layout.shadow.heightM),
      new MeshBasicMaterial({
        map: groundTexture,
        transparent: true,
        depthWrite: false,
        opacity: clamp01(AR_OPACITY.shadow * 2),
      }),
    );
    shadow.name = "signboard:shadow";
    shadow.position.set(layout.shadow.xM, layout.shadow.yM, layout.shadow.zM);
    shadow.rotation.x = layout.shadow.rotationXRad;
    group.add(shadow);
  }

  if (layout.pole) {
    const poleColor = new Color(descriptor.palette.surface).multiplyScalar(layout.pole.colorMultiplier);
    const pole = new Mesh(
      new PlaneGeometry(layout.pole.widthM, layout.pole.heightM),
      new MeshBasicMaterial({
        color: poleColor,
        transparent: true,
        opacity: AR_OPACITY.pole,
        side: DoubleSide,
        depthWrite: false,
      }),
    );
    pole.name = descriptor.shape === AR_SHAPE.PIN ? "signboard:pin-stem" : "signboard:pole";
    pole.position.set(layout.pole.xM, layout.pole.yM, layout.pole.zM);
    group.add(pole);
  }

  group.userData = {
    kind: "approved-preview-signboard",
    descriptor,
    coordinateSystem: layout.coordinateSystem,
    baseOpacity: descriptor.opacity,
  };
  return group;
}

/** Dispose geometries, materials and owned texture maps in a signboard group. */
export function disposeThreeSignboardGroup(group) {
  if (!group) return false;
  const textures = new Set();
  group.traverse?.((node) => {
    node.geometry?.dispose?.();
    const materials = Array.isArray(node.material) ? node.material : [node.material];
    for (const material of materials) {
      if (!material) continue;
      if (material.map) textures.add(material.map);
      material.dispose?.();
    }
  });
  textures.forEach((texture) => texture.dispose?.());
  group.parent?.remove?.(group);
  return true;
}

/** Create a camera whose projection is supplied by the existing Carrot stage transform. */
export function createThreeStageCamera() {
  const camera = new Camera();
  camera.name = "carrot-ar-stage-camera";
  camera.matrixAutoUpdate = false;
  camera.matrix.identity();
  camera.matrixWorld.identity();
  camera.matrixWorldInverse.identity();
  return camera;
}

export function applyThreeStageProjection(camera, stage, viewport = {}) {
  const values = stageProjectionMatrix(stage, viewport);
  if (!camera || !values) return false;
  camera.projectionMatrix.set(...values);
  camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert();
  camera.matrixWorldNeedsUpdate = false;
  return true;
}

/** Place a preview-local signboard at a route anchor and apply LOD/hold opacity. */
export function placeThreeSignboardGroup(group, anchor, {
  scale = 1,
  alpha = 1,
  held = false,
} = {}) {
  const values = markerWorldMatrix(anchor, scale);
  if (!group || !values) return false;
  group.matrixAutoUpdate = false;
  group.matrix.set(...values);
  group.matrixWorldNeedsUpdate = true;

  setThreeSignboardOpacity(group, alpha, held);
  return true;
}

function descriptorCacheKey(item) {
  const d = item?.descriptor || {};
  return [
    item?.eventKey || item?.source || "marker",
    d.kind, d.shape, d.widthM, d.heightM, d.mountHeightM,
  ].join("|");
}

function descriptorContentKey(descriptor = {}) {
  return [
    descriptor.tone, descriptor.primary, descriptor.secondary,
    descriptor.turnSign, descriptor.chevronCount, descriptor.phase,
    descriptor.opacity,
  ].join("|");
}

/** 같은 event/geometry의 CanvasTexture를 버리지 않고 내용만 다시 칠한다. */
function repaintThreeSignboardGroup(group, descriptor) {
  const face = group?.getObjectByName?.("signboard:face");
  const texture = face?.material?.map;
  const canvas = texture?.image;
  const context = canvas?.getContext?.("2d");
  if (!context) return false;
  const { width, height } = textureSizeFor(descriptor);
  if (canvas.width !== width) canvas.width = width;
  if (canvas.height !== height) canvas.height = height;
  paintSignboard(context, descriptor);
  texture.needsUpdate = true;
  face.material.opacity = clamp01(descriptor.opacity * AR_OPACITY.group);
  face.material.userData.arBaseOpacity = face.material.opacity;
  group.userData.descriptor = descriptor;
  group.userData.baseOpacity = descriptor.opacity;
  return true;
}

function setThreeSignboardOpacity(group, alpha, held = false) {
  const opacityMultiplier = clamp01(alpha) * (held ? AR_OPACITY.held : 1);
  group?.traverse?.((node) => {
    const materials = Array.isArray(node.material) ? node.material : [node.material];
    for (const material of materials) {
      if (!material) continue;
      if (!Number.isFinite(material.userData?.arBaseOpacity)) {
        material.userData.arBaseOpacity = Number.isFinite(material.opacity) ? material.opacity : 1;
      }
      material.opacity = clamp01(material.userData.arBaseOpacity * opacityMultiplier);
    }
  });
}

function stagePoint(stage, point) {
  const projected = point && projectPoint(stage?.calibTransform, point.x, point.y, point.z);
  if (!projected) return null;
  return {
    x: projected.x * (Number(stage.scale) || 1) + (Number(stage.tx) || 0),
    y: projected.y * (Number(stage.scale) || 1) + (Number(stage.ty) || 0),
    depth: projected.depth,
  };
}

function worldFromMarkerLocal(anchor, frame, local) {
  const x = Number(local?.x) || 0;
  const y = Number(local?.y) || 0;
  const z = Number(local?.z) || 0;
  return {
    x: (Number(anchor.x) || 0) + frame.right[0] * x + frame.up[0] * y - frame.forward[0] * z,
    y: (Number(anchor.y) || 0) + frame.right[1] * x + frame.up[1] * y - frame.forward[1] * z,
    z: (Number(anchor.z) || 0) + frame.right[2] * x + frame.up[2] * y - frame.forward[2] * z,
  };
}

function pixelDistance(a, b) {
  return a && b ? Math.hypot(a.x - b.x, a.y - b.y) : 0;
}

/** Align a standing marker with the active calibrated stage-up direction. */
export function stageUprightAnchor(anchor, stage, maxYawRad = AR_BILLBOARD.maxYawRad) {
  const billboard = constrainedBillboardAnchor(anchor, maxYawRad);
  if (!billboard || !stage?.calibTransform) return billboard;
  const frame = roadFrameForAnchor(billboard);
  const base = {
    x: Number(billboard.x) || 0,
    y: Number(billboard.y) || 0,
    z: Number(billboard.z) || 0,
  };
  const projectedBase = stagePoint(stage, base);
  const projectedTip = stagePoint(stage, {
    x: base.x + frame.up[0],
    y: base.y + frame.up[1],
    z: base.z + frame.up[2],
  });
  if (!projectedBase || !projectedTip || projectedTip.y < projectedBase.y) {
    return Object.freeze({ ...billboard, stageUpFlipped: false });
  }
  const corrected = roadFrameFromForward(
    frame.forward,
    frame.up.map((value) => -value),
  );
  return Object.freeze({
    ...billboard,
    ...roadFrameFields(corrected),
    stageUpFlipped: true,
  });
}

export function markerScreenMetrics(descriptor, anchor, stage, options = {}) {
  if (!descriptor || !anchor || !stage?.calibTransform) return null;
  const frame = roadFrameForAnchor(anchor);
  const layout = signboardLayout(descriptor, { mountLiftM: options.mountLiftM });
  const isBand = descriptor.shape === AR_SHAPE.BAND;
  const opticalHeightM = isBand ? layout.face.widthM : layout.face.heightM;
  const opticalWidthM = layout.face.widthM;
  if (!(opticalHeightM > 0) || !(opticalWidthM > 0)) return null;

  const centerLocal = { x: layout.face.xM, y: layout.face.yM, z: layout.face.zM };
  const center = stagePoint(stage, worldFromMarkerLocal(anchor, frame, centerLocal));
  let heightStart;
  let heightEnd;
  if (isBand) {
    // A road band has no upright height. Its transverse lane width is the
    // optical legibility dimension used by the responsive clamp.
    heightStart = stagePoint(stage, worldFromMarkerLocal(anchor, frame, {
      ...centerLocal,
      x: centerLocal.x - opticalHeightM / 2,
    }));
    heightEnd = stagePoint(stage, worldFromMarkerLocal(anchor, frame, {
      ...centerLocal,
      x: centerLocal.x + opticalHeightM / 2,
    }));
  } else {
    heightStart = stagePoint(stage, worldFromMarkerLocal(anchor, frame, {
      ...centerLocal,
      y: centerLocal.y - opticalHeightM / 2,
    }));
    heightEnd = stagePoint(stage, worldFromMarkerLocal(anchor, frame, {
      ...centerLocal,
      y: centerLocal.y + opticalHeightM / 2,
    }));
  }
  const widthStart = stagePoint(stage, worldFromMarkerLocal(anchor, frame, {
    ...centerLocal,
    x: centerLocal.x - opticalWidthM / 2,
  }));
  const widthEnd = stagePoint(stage, worldFromMarkerLocal(anchor, frame, {
    ...centerLocal,
    x: centerLocal.x + opticalWidthM / 2,
  }));
  if (!center || !heightStart || !heightEnd || !widthStart || !widthEnd) return null;
  const nearestDepth = Math.min(
    center.depth,
    heightStart.depth,
    heightEnd.depth,
    widthStart.depth,
    widthEnd.depth,
  );
  // 프리뷰의 face-above/pole-below 배치를 바꾸지 않는다. 대신 표지가 카메라
  // 근평면을 통과해 원근이 폭발하기 전에 해당 앵커의 수명을 끝낸다.
  const nearDepthM = isBand
    ? AR_RENDER.nearMarkerDepthM
    : AR_RENDER.nearUprightMarkerDepthM;
  if (!(nearestDepth >= nearDepthM)) return null;
  const heightPx = pixelDistance(heightStart, heightEnd);
  const widthPx = pixelDistance(widthStart, widthEnd);
  if (!(heightPx > 0.5)) return null;
  return {
    centerX: center.x,
    centerY: center.y,
    heightPx,
    widthPx,
    opticalHeightM,
    opticalWidthM,
    focalPx: heightPx * Math.max(0.5, Number(center.depth) || 0.5)
      / opticalHeightM,
  };
}

/**
 * Three-only AR renderer. It consumes the same composed frame as renderer.js,
 * but owns one WebGL2 context and fails closed through `onFatal`.
 */
export function createThreeArRenderer(options = {}) {
  const canvas = options.surface;
  if (!canvas) throw new Error("Three AR requires an OffscreenCanvas surface");
  const rendererFactory = options.rendererFactory
    || ((rendererOptions) => new WebGLRenderer(rendererOptions));
  const renderer = rendererFactory({
    canvas,
    alpha: true,
    antialias: true,
    premultipliedAlpha: true,
    powerPreference: "high-performance",
  });
  if (!renderer) throw new Error("Three WebGL renderer initialization failed");

  renderer.outputColorSpace = SRGBColorSpace;
  renderer.setClearColor?.(0x000000, 0);
  const scene = new Scene();
  const surfaceRoot = new Group();
  const signRoot = new Group();
  surfaceRoot.name = "carrot-ar-surfaces";
  signRoot.name = "carrot-ar-signs";
  scene.add(surfaceRoot, signRoot);
  const camera = createThreeStageCamera();
  const signCache = new Map();
  const surfaceCache = new Map();
  const selectedSignKeys = new Set();
  let destroyed = false;
  let failed = "";
  let frameNumber = 0;
  let lastSize = { width: 0, height: 0 };
  let hasProjection = false;
  const stats = {
    drawn: 0,
    skipped: 0,
    selectionSuppressed: 0,
    billboarded: 0,
    farRouteAnchors: 0,
    lastReason: "",
    contextLost: false,
  };

  function fatal(reason) {
    if (failed || destroyed) return;
    failed = String(reason || "Three AR renderer failed");
    stats.lastReason = failed;
    try { renderer.clear?.(); } catch (_) {}
    options.onFatal?.(failed);
  }

  function onContextLost(event) {
    event?.preventDefault?.();
    stats.contextLost = true;
    fatal("WebGL context lost");
  }
  canvas.addEventListener?.("webglcontextlost", onContextLost);

  function syncSize() {
    const width = Math.max(1, Number(canvas.width) || 1);
    const height = Math.max(1, Number(canvas.height) || 1);
    if (width !== lastSize.width || height !== lastSize.height) {
      renderer.setSize?.(width, height, false);
      lastSize = { width, height };
    }
    return lastSize;
  }

  function clearFrame(reason = "") {
    signRoot.visible = false;
    surfaceRoot.visible = false;
    stats.drawn = 0;
    stats.skipped = 0;
    stats.selectionSuppressed = 0;
    stats.billboarded = 0;
    stats.farRouteAnchors = 0;
    stats.lastReason = reason;
    renderer.clear?.();
    return false;
  }

  function transitionOpacity(entry, visible, nowMs) {
    const now = Number.isFinite(Number(nowMs)) ? Number(nowMs) : frameNumber * 50;
    const previousAt = Number.isFinite(entry.opacityAt) ? entry.opacityAt : now;
    const dt = Math.max(0, Math.min(250, now - previousAt));
    entry.opacityAt = now;
    if (!Number.isFinite(entry.visibilityAlpha)) entry.visibilityAlpha = visible ? 0.01 : 0;
    const duration = visible ? AR_RENDER.fadeInMs : AR_RENDER.fadeOutMs;
    const delta = duration > 0 ? dt / duration : 1;
    entry.visibilityAlpha = clamp01(entry.visibilityAlpha + (visible ? delta : -delta));
    return entry.visibilityAlpha;
  }

  function useEntry(cache, root, key, item, factories, nowMs) {
    const descriptor = item.descriptor;
    const contentKey = descriptorContentKey(descriptor);
    let entry = cache.get(key);
    if (!entry) {
      const group = createThreeSignboardGroup(descriptor, factories);
      entry = {
        group,
        contentKey,
        lastSeenAt: nowMs,
        opacityAt: nowMs,
        visibilityAlpha: 0.01,
        lastPlanAlpha: 1,
        lastHeld: false,
        lifecycleSlot: item.lifecycleSlot || markerLifecycleSlot(item),
        presentation: createMarkerPresentationFilter(),
      };
      cache.set(key, entry);
      root.add(group);
    } else if (entry.contentKey !== contentKey) {
      if (!repaintThreeSignboardGroup(entry.group, descriptor)) {
        disposeThreeSignboardGroup(entry.group);
        entry.group = createThreeSignboardGroup(descriptor, factories);
        root.add(entry.group);
      }
      entry.contentKey = contentKey;
    }
    entry.lifecycleSlot = item.lifecycleSlot || markerLifecycleSlot(item);
    entry.lastSeenAt = nowMs;
    return entry;
  }

  function sweepCache(cache, used, present, activeSlots, nowMs) {
    for (const [key, entry] of cache) {
      if (used.has(key)) continue;
      if (present.has(key)) {
        entry.group.visible = false;
        entry.lastSeenAt = nowMs;
        continue;
      }
      // A new event in the same semantic slot replaces the old object as one
      // transaction. Cross-fading two world positions creates ghost markers.
      if (activeSlots.has(entry.lifecycleSlot)) {
        disposeThreeSignboardGroup(entry.group);
        cache.delete(key);
        continue;
      }
      const alpha = transitionOpacity(entry, false, nowMs);
      entry.group.visible = alpha > 0;
      if (entry.group.visible) {
        setThreeSignboardOpacity(
          entry.group,
          entry.lastPlanAlpha * alpha,
          entry.lastHeld,
        );
        stats.drawn += 1;
      }
      const unseenMs = Math.max(0, Number(nowMs) - Number(entry.lastSeenAt));
      if (alpha <= 0 && unseenMs >= AR_RENDER.cacheGraceMs) {
        disposeThreeSignboardGroup(entry.group);
        cache.delete(key);
      }
    }
  }

  function renderSurfaces(frame, size, nowMs) {
    const used = new Set();
    const present = new Set();
    const activeSlots = new Set();
    surfaceRoot.visible = true;
    for (const item of Array.isArray(frame.signs) ? frame.signs : []) {
      const descriptor = item.descriptor;
      if (markerForm(descriptor?.kind) !== AR_MARKER_FORM.SURFACE) continue;
      const key = descriptorCacheKey(item);
      present.add(key);
      activeSlots.add(item.lifecycleSlot || markerLifecycleSlot(item));
      if (descriptor.kind !== AR_MARKER_KIND.LANE_BAND) { stats.skipped += 1; continue; }
      const baseAnchor = item.anchor;
      const anchor = baseAnchor && Number(descriptor.laneOffsetM)
        ? { ...baseAnchor, y: (Number(baseAnchor.y) || 0) + Number(descriptor.laneOffsetM) }
        : baseAnchor;
      const metrics = markerScreenMetrics(descriptor, anchor, frame.stage);
      if (!anchor || !metrics) { stats.skipped += 1; continue; }
      const plan = planMarker({
        distanceM: item.distanceM,
        egoSpeedMps: frame.egoSpeedMps,
        // BAND는 수평면이므로 폭을 화면 크기 clamp의 광학 높이로 사용한다.
        worldHeightM: metrics.opticalHeightM,
        worldWidthM: metrics.opticalWidthM,
        focalPx: metrics.focalPx,
        centerX: metrics.centerX,
        projectedHeightPx: metrics.heightPx,
        projectedWidthPx: metrics.widthPx,
        descriptorScale: descriptor.scale,
        canvas: {
          width: Number(frame.stage?.stageWidth) || size.width,
          height: Number(frame.stage?.stageHeight) || size.height,
        },
      });
      if (!plan.visible) { stats.skipped += 1; continue; }

      const entry = useEntry(surfaceCache, surfaceRoot, key, item, {
        canvasFactory: options.canvasFactory,
        textureFactory: options.textureFactory,
        anisotropy: options.anisotropy || 8,
      }, nowMs);
      used.add(key);
      entry.group.visible = true;
      entry.lastPlanAlpha = plan.alpha;
      entry.lastHeld = frame.held === true;
      const presentation = entry.presentation.update(anchor, plan.worldScale, nowMs);
      placeThreeSignboardGroup(entry.group, presentation.anchor, {
        scale: presentation.scale,
        alpha: plan.alpha * transitionOpacity(entry, true, nowMs),
        held: frame.held === true,
      });
      stats.drawn += 1;
    }
    sweepCache(surfaceCache, used, present, activeSlots, nowMs);
  }

  function renderSigns(frame, size, nowMs) {
    const used = new Set();
    const present = new Set();
    const activeSlots = new Set();
    const candidates = [];
    const canvas = {
      width: Number(frame.stage?.stageWidth) || size.width,
      height: Number(frame.stage?.stageHeight) || size.height,
    };
    signRoot.visible = true;
    for (const item of Array.isArray(frame.signs) ? frame.signs : []) {
      if (markerForm(item.descriptor?.kind) === AR_MARKER_FORM.SURFACE) continue;
      const key = descriptorCacheKey(item);
      present.add(key);
      activeSlots.add(item.lifecycleSlot || markerLifecycleSlot(item));
      const anchor = stageUprightAnchor(item.anchor, frame.stage, AR_BILLBOARD.maxYawRad);
      const metrics = markerScreenMetrics(item.descriptor, anchor, frame.stage, {
        mountLiftM: AR_RENDER.presentation.mountLiftM,
      });
      if (!anchor || !metrics) { stats.skipped += 1; continue; }
      const plan = planMarker({
        distanceM: item.distanceM,
        egoSpeedMps: frame.egoSpeedMps,
        worldHeightM: metrics.opticalHeightM,
        worldWidthM: metrics.opticalWidthM,
        focalPx: metrics.focalPx,
        centerX: metrics.centerX,
        projectedHeightPx: metrics.heightPx,
        projectedWidthPx: metrics.widthPx,
        descriptorScale: item.descriptor?.scale,
        minimumWorldScale: AR_RENDER.presentation.nearWorldScaleMin,
        canvas,
      });
      if (!plan.visible) { stats.skipped += 1; continue; }

      const halfWidth = plan.projectedWidthPx / 2;
      const halfHeight = plan.projectedPx / 2;
      candidates.push({
        item,
        key,
        anchor,
        metrics,
        plan,
        distanceM: item.distanceM,
        screenBounds: Object.freeze({
          left: metrics.centerX - halfWidth,
          right: metrics.centerX + halfWidth,
          top: metrics.centerY - halfHeight,
          bottom: metrics.centerY + halfHeight,
        }),
      });
    }

    const selected = selectVisibleMarkers(candidates, canvas, {
      preferredKeys: selectedSignKeys,
    });
    if (candidates.length > 0) {
      selectedSignKeys.clear();
      for (const candidate of selected) selectedSignKeys.add(candidate.key);
    }
    stats.selectionSuppressed += candidates.length - selected.length;
    stats.skipped += candidates.length - selected.length;
    for (const candidate of selected) {
      const { item, key, anchor, plan } = candidate;
      if (Math.abs(Number(anchor.billboardYawRad) || 0) > 1e-4) stats.billboarded += 1;
      if (item.anchor?.routeDerived === true) stats.farRouteAnchors += 1;

      const entry = useEntry(signCache, signRoot, key, item, {
        canvasFactory: options.canvasFactory,
        textureFactory: options.textureFactory,
        shadowTextureFactory: options.shadowTextureFactory,
        anisotropy: options.anisotropy || 8,
        mountLiftM: AR_RENDER.presentation.mountLiftM,
      }, nowMs);
      used.add(key);
      entry.group.visible = true;
      entry.lastPlanAlpha = plan.alpha;
      entry.lastHeld = frame.held === true;
      const presentation = entry.presentation.update(
        anchor,
        plan.worldScale * AR_RENDER.presentation.uprightScale,
        nowMs,
      );
      placeThreeSignboardGroup(entry.group, presentation.anchor, {
        scale: presentation.scale,
        alpha: plan.alpha * transitionOpacity(entry, true, nowMs),
        held: frame.held === true,
      });
      stats.drawn += 1;
    }
    sweepCache(signCache, used, present, activeSlots, nowMs);
  }

  function render(frame = {}) {
    if (destroyed || failed) return false;
    try {
      const size = syncSize();
      stats.drawn = 0;
      stats.skipped = 0;
      stats.selectionSuppressed = 0;
      stats.billboarded = 0;
      stats.farRouteAnchors = 0;
      frameNumber += 1;
      const nowMs = Number.isFinite(Number(frame.nowMs)) ? Number(frame.nowMs) : frameNumber * 50;
      if (frame.stage && applyThreeStageProjection(camera, frame.stage, {
        width: Number(frame.stage.stageWidth) || size.width,
        height: Number(frame.stage.stageHeight) || size.height,
      })) hasProjection = true;
      if (!hasProjection) return clearFrame("stage projection unavailable");

      const canPresent = Boolean(
        frame.stage
        && (frame.sync?.canDrawPrecise === true || frame.held === true),
      );
      const presentFrame = canPresent ? frame : { ...frame, signs: [] };
      stats.lastReason = canPresent
        ? ""
        : (frame.sync?.reasons?.[0] || "stage/sync unavailable");
      renderSurfaces(presentFrame, size, nowMs);
      renderSigns(presentFrame, size, nowMs);
      if (stats.drawn === 0 && !canPresent) return clearFrame(stats.lastReason);
      renderer.render(scene, camera);
      return canPresent;
    } catch (error) {
      fatal(error?.message || error);
      return false;
    }
  }

  function status() {
    return Object.freeze({
      backend: "three",
      destroyed,
      failed: failed || null,
      contextLost: stats.contextLost,
      drawn: stats.drawn,
      skipped: stats.skipped,
      selectionSuppressed: stats.selectionSuppressed,
      billboarded: stats.billboarded,
      farRouteAnchors: stats.farRouteAnchors,
      lastReason: stats.lastReason,
      textureCount: signCache.size + surfaceCache.size,
      canvas: `${lastSize.width}x${lastSize.height}`,
    });
  }

  function reset() {
    if (destroyed) return false;
    for (const entry of surfaceCache.values()) disposeThreeSignboardGroup(entry.group);
    surfaceCache.clear();
    for (const entry of signCache.values()) disposeThreeSignboardGroup(entry.group);
    signCache.clear();
    selectedSignKeys.clear();
    hasProjection = false;
    clearFrame("reset");
    return true;
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    canvas.removeEventListener?.("webglcontextlost", onContextLost);
    for (const entry of surfaceCache.values()) disposeThreeSignboardGroup(entry.group);
    surfaceCache.clear();
    for (const entry of signCache.values()) disposeThreeSignboardGroup(entry.group);
    signCache.clear();
    selectedSignKeys.clear();
    renderer.dispose?.();
    renderer.forceContextLoss?.();
    return true;
  }

  return Object.freeze({ canvas, render, reset, status, destroy });
}
