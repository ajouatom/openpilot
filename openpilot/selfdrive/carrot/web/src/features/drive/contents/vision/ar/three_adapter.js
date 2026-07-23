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
  markerSupportHeightM,
  markerForm,
} from "./tokens.js";
import {
  AR_LEGIBILITY,
  AR_OPACITY,
  AR_SHAPE,
} from "./design_tokens.js";
import { paintSignboard, textureSizeFor } from "./signboard.js";
import {
  scaledBandFacePosition,
  signboardLayout,
} from "./three_signboard_layout.js";
import { markerWorldMatrix, stageProjectionMatrix } from "./three_projection.js";
import { projectRouteFluPoint, routeFluProjectionDepth } from "./projection.js";
import { planMarker, selectVisibleMarkers } from "./responsive.js";
import { markerIdentity, markerLifecycleSlot } from "./marker_identity.js";
import {
  AR_MARKER_VISIBILITY_STATE,
  markerStateReason,
  markerViewportState,
} from "./marker_state.js";
import {
  constrainedBillboardAnchor,
  faceFrameForAnchor,
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
  supportHeightM,
  surfacePitchRad,
} = {}) {
  const layout = signboardLayout(descriptor, { supportHeightM, surfacePitchRad });
  const group = new Group();
  group.name = `signboard:${descriptor.shape}`;

  const faceTexture = configureTexture(
    textureFactory
      ? textureFactory(descriptor)
      : paintedTexture(descriptor, canvasFactory),
    anisotropy,
    // Canvas pixels start at the top-left while PlaneGeometry uses v=1 at its
    // visual top edge. Keep Three's upload flip so the canvas top row remains
    // at the sign top. markerWorldMatrix already preserves horizontal order.
    { flipY: true, flipX: false },
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

  // One FrontSide plane per viewing direction. A single DoubleSide plane makes
  // its back texture mirrored; a single FrontSide plane disappears at a
  // hairpin. The paired plane has reversed winding and therefore exposes the
  // same readable canvas orientation from the opposite route direction.
  if (descriptor.shape !== AR_SHAPE.BAND) {
    const faceBack = new Mesh(face.geometry, faceMaterial);
    faceBack.name = "signboard:face-back";
    faceBack.position.copy(face.position);
    faceBack.rotation.x = layout.face.rotationXRad;
    faceBack.rotation.y = Math.PI;
    group.add(faceBack);
  }

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
    layout,
    coordinateSystem: layout.coordinateSystem,
    baseOpacity: descriptor.opacity,
    presentationScale: 1,
  };
  return group;
}

/** Dispose geometries, materials and owned texture maps in a signboard group. */
export function disposeThreeSignboardGroup(group) {
  if (!group) return false;
  const textures = new Set();
  const geometries = new Set();
  const materialsToDispose = new Set();
  group.traverse?.((node) => {
    if (node.geometry) geometries.add(node.geometry);
    const materials = Array.isArray(node.material) ? node.material : [node.material];
    for (const material of materials) {
      if (!material) continue;
      if (material.map) textures.add(material.map);
      materialsToDispose.add(material);
    }
  });
  geometries.forEach((geometry) => geometry.dispose?.());
  materialsToDispose.forEach((material) => material.dispose?.());
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
  presentationScale = null,
  presentationWidthScale = null,
  alpha = 1,
  held = false,
} = {}) {
  const values = markerWorldMatrix(anchor, scale);
  if (!group || !values) return false;
  if (
    presentationScale !== null
    && presentationScale !== undefined
    && Number.isFinite(Number(presentationScale))
  ) {
    setThreeSignboardPresentationScale(group, presentationScale, {
      widthScale: presentationWidthScale,
    });
  }
  group.matrixAutoUpdate = false;
  group.matrix.set(...values);
  group.matrixWorldNeedsUpdate = true;

  setThreeSignboardOpacity(group, alpha, held);
  return true;
}

function descriptorCacheKey(item) {
  // One cache entry belongs to one semantic event occurrence. Presentation
  // fields must never participate: phase, size and text change while the
  // vehicle approaches the same physical marker.
  return String(item?.markerId || item?.eventKey || markerIdentity(item));
}

function descriptorContentKey(descriptor = {}) {
  return [
    descriptor.kind, descriptor.tone, descriptor.shape,
    descriptor.primary, descriptor.secondary,
    descriptor.turnSign, descriptor.chevronCount, descriptor.phase,
    descriptor.opacity, descriptor.radiusM, descriptor.widthM, descriptor.heightM,
  ].join("|");
}

function descriptorGeometryKey(descriptor = {}, options = {}) {
  return [
    descriptor.kind, descriptor.shape,
    descriptor.widthM, descriptor.heightM, descriptor.mountHeightM,
    descriptor.depthM, descriptor.radiusM,
    Number(options.supportHeightM) || 0,
    Number(options.surfacePitchRad) || 0,
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

function sameDimension(a, b) {
  return Math.abs((Number(a) || 0) - (Number(b) || 0)) <= 1e-6;
}

function replacePlaneGeometry(nodes, widthM, heightM) {
  const usable = nodes.filter(Boolean);
  if (!usable.length) return false;
  const previous = usable[0].geometry;
  const parameters = previous?.parameters || {};
  if (sameDimension(parameters.width, widthM) && sameDimension(parameters.height, heightM)) {
    return false;
  }
  const geometry = new PlaneGeometry(widthM, heightM);
  for (const node of usable) node.geometry = geometry;
  previous?.dispose?.();
  return true;
}

/** Update mutable layout inside one stable marker Group. */
function updateThreeSignboardGroupGeometry(group, descriptor, options = {}) {
  const previous = group?.userData?.descriptor;
  if (!group || previous?.shape !== descriptor?.shape) return null;
  const layout = signboardLayout(descriptor, {
    supportHeightM: options.supportHeightM,
    surfacePitchRad: options.surfacePitchRad,
  });
  const face = group.getObjectByName?.("signboard:face");
  const faceBack = group.getObjectByName?.("signboard:face-back");
  const shadow = group.getObjectByName?.("signboard:shadow");
  const pole = group.getObjectByName?.("signboard:pole")
    || group.getObjectByName?.("signboard:pin-stem");
  if (!face || Boolean(faceBack) !== (descriptor.shape !== AR_SHAPE.BAND)) return null;
  if (Boolean(shadow) !== Boolean(layout.shadow) || Boolean(pole) !== Boolean(layout.pole)) return null;

  let changed = replacePlaneGeometry([face, faceBack], layout.face.widthM, layout.face.heightM);
  face.position.set(layout.face.xM, layout.face.yM, layout.face.zM);
  face.rotation.x = layout.face.rotationXRad;
  if (faceBack) {
    faceBack.position.copy(face.position);
    faceBack.rotation.x = layout.face.rotationXRad;
    faceBack.rotation.y = Math.PI;
  }
  if (shadow && layout.shadow) {
    changed = replacePlaneGeometry([shadow], layout.shadow.widthM, layout.shadow.heightM) || changed;
    shadow.position.set(layout.shadow.xM, layout.shadow.yM, layout.shadow.zM);
    shadow.rotation.x = layout.shadow.rotationXRad;
  }
  if (pole && layout.pole) {
    changed = replacePlaneGeometry([pole], layout.pole.widthM, layout.pole.heightM) || changed;
    pole.position.set(layout.pole.xM, layout.pole.yM, layout.pole.zM);
    const poleColor = new Color(descriptor.palette.surface).multiplyScalar(layout.pole.colorMultiplier);
    pole.material?.color?.copy?.(poleColor);
  }
  group.userData.descriptor = descriptor;
  group.userData.layout = layout;
  group.userData.coordinateSystem = layout.coordinateSystem;
  group.userData.baseOpacity = descriptor.opacity;
  setThreeSignboardPresentationScale(group, group.userData.presentationScale, {
    widthScale: group.userData.presentationWidthScale,
  });
  return changed;
}

/**
 * Scale only the approved visual component around its support, never the
 * world-anchor transform. The face bottom therefore continues to meet the
 * pole top while far-legibility and near-view bounds change its readable size.
 */
export function setThreeSignboardPresentationScale(group, value = 1, options = {}) {
  const layout = group?.userData?.layout;
  const descriptor = group?.userData?.descriptor;
  if (!group || !layout || !descriptor) return false;
  const scale = Math.max(0.01, Number(value) || 1);
  const widthScale = Math.max(0.01, Number(options.widthScale) || scale);
  const face = group.getObjectByName?.("signboard:face");
  const faceBack = group.getObjectByName?.("signboard:face-back");
  const shadow = group.getObjectByName?.("signboard:shadow");
  const pole = group.getObjectByName?.("signboard:pole")
    || group.getObjectByName?.("signboard:pin-stem");
  if (!face) return false;

  for (const node of [face, faceBack].filter(Boolean)) {
    node.scale.set(widthScale, scale, 1);
    node.position.x = layout.face.xM;
  }
  const isBand = descriptor.shape === AR_SHAPE.BAND;
  const isPin = descriptor.shape === AR_SHAPE.PIN;
  const bandFace = isBand ? scaledBandFacePosition(layout.face, scale) : null;
  const faceY = isBand
    ? bandFace.yM
    : isPin
      ? layout.face.yM
    : (layout.pole?.heightM || 0) + layout.face.heightM * scale / 2;
  for (const node of [face, faceBack].filter(Boolean)) {
    node.position.y = faceY;
    node.position.z = bandFace?.zM ?? layout.face.zM;
  }

  if (shadow) {
    shadow.scale.set(widthScale, scale, 1);
    shadow.position.set(layout.shadow.xM, layout.shadow.yM, layout.shadow.zM);
  }
  if (pole) {
    pole.scale.set(1, 1, 1);
    pole.position.set(layout.pole.xM, layout.pole.yM, layout.pole.zM);
  }
  group.userData.presentationScale = scale;
  group.userData.presentationWidthScale = widthScale;
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
  const projected = point && projectRouteFluPoint(stage?.calibTransform, point.x, point.y, point.z);
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

/** Build an upright face frame without consulting or correcting screen pixels. */
export function uprightPresentationAnchor(anchor, maxYawRad = AR_BILLBOARD.maxYawRad) {
  return constrainedBillboardAnchor(anchor, maxYawRad);
}

export function markerProjectionState(descriptor, anchor, stage, options = {}) {
  const unavailable = () => Object.freeze({
    state: AR_MARKER_VISIBILITY_STATE.PROJECTION_UNAVAILABLE,
    metrics: null,
  });
  if (!descriptor || !anchor || !stage?.calibTransform) return unavailable();
  const frame = faceFrameForAnchor(anchor);
  const layout = signboardLayout(descriptor, {
    supportHeightM: options.supportHeightM,
    surfacePitchRad: options.surfacePitchRad,
  });
  const isBand = descriptor.shape === AR_SHAPE.BAND;
  const isPin = descriptor.shape === AR_SHAPE.PIN;
  const presentationScale = Math.max(0.01, Number(options.presentationScale) || 1);
  const presentationHeightScale = Math.max(
    0.01,
    Number(options.presentationHeightScale) || presentationScale,
  );
  const presentationWidthScale = Math.max(
    0.01,
    Number(options.presentationWidthScale) || presentationScale,
  );
  const opticalHeightM = layout.face.heightM * presentationHeightScale;
  const opticalWidthM = layout.face.widthM * presentationWidthScale;
  if (!(opticalHeightM > 0) || !(opticalWidthM > 0)) return unavailable();

  const bandFace = isBand
    ? scaledBandFacePosition(layout.face, presentationHeightScale)
    : null;
  const centerLocal = {
    x: layout.face.xM,
    y: isBand
      ? bandFace.yM
      : isPin
        ? layout.face.yM
      : (layout.pole?.heightM || 0) + layout.face.heightM * presentationHeightScale / 2,
    z: bandFace?.zM ?? layout.face.zM,
  };
  let heightStartLocal;
  let heightEndLocal;
  if (isBand) {
    // Measure the actual pitched PlaneGeometry Y axis. Product BAND uses a
    // readable road-relative pitch while the immutable preview stays flat.
    const halfHeightM = opticalHeightM / 2;
    heightStartLocal = {
      ...centerLocal,
      y: centerLocal.y - bandFace.cosPitch * halfHeightM,
      z: centerLocal.z - bandFace.sinPitch * halfHeightM,
    };
    heightEndLocal = {
      ...centerLocal,
      y: centerLocal.y + bandFace.cosPitch * halfHeightM,
      z: centerLocal.z + bandFace.sinPitch * halfHeightM,
    };
  } else {
    heightStartLocal = {
      ...centerLocal,
      y: centerLocal.y - opticalHeightM / 2,
    };
    heightEndLocal = {
      ...centerLocal,
      y: centerLocal.y + opticalHeightM / 2,
    };
  }
  const widthStartLocal = {
    ...centerLocal,
    x: centerLocal.x - opticalWidthM / 2,
  };
  const widthEndLocal = {
    ...centerLocal,
    x: centerLocal.x + opticalWidthM / 2,
  };
  const worldPoints = [
    centerLocal, heightStartLocal, heightEndLocal, widthStartLocal, widthEndLocal,
  ].map((local) => worldFromMarkerLocal(anchor, frame, local));
  const depths = worldPoints.map((point) => routeFluProjectionDepth(
    stage.calibTransform, point.x, point.y, point.z,
  ));
  if (depths.some((depth) => depth === null)) return unavailable();
  const centerDepth = depths[0];
  const nearestDepth = Math.min(...depths);
  if (!(centerDepth > 1e-3)) return Object.freeze({
    state: AR_MARKER_VISIBILITY_STATE.BEHIND_CAMERA,
    metrics: null,
  });
  // 프리뷰의 face-above/pole-below 배치를 바꾸지 않는다. 대신 표지가 카메라
  // 근평면을 통과해 원근이 폭발하기 전에 해당 앵커의 수명을 끝낸다.
  const nearDepthM = isBand
    ? AR_RENDER.nearMarkerDepthM
    : AR_RENDER.nearUprightMarkerDepthM;
  if (!(nearestDepth >= nearDepthM)) return Object.freeze({
    state: AR_MARKER_VISIBILITY_STATE.NEAR_PLANE,
    metrics: null,
  });
  const projected = worldPoints.map((point) => stagePoint(stage, point));
  if (projected.some((point) => !point)) return unavailable();
  const [center, heightStart, heightEnd, widthStart, widthEnd] = projected;
  const heightPx = pixelDistance(heightStart, heightEnd);
  const widthPx = pixelDistance(widthStart, widthEnd);
  if (!(heightPx > 0.5)) return unavailable();
  return Object.freeze({
    state: AR_MARKER_VISIBILITY_STATE.ACTIVE_CANDIDATE,
    metrics: Object.freeze({
      centerX: center.x,
      centerY: center.y,
      heightPx,
      widthPx,
      depth: centerDepth,
      nearestDepth,
      opticalHeightM,
      opticalWidthM,
      focalPx: heightPx * Math.max(0.5, Number(center.depth) || 0.5)
        / opticalHeightM,
    }),
  });
}

export function markerScreenMetrics(descriptor, anchor, stage, options = {}) {
  return markerProjectionState(descriptor, anchor, stage, options).metrics;
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
  let pendingSelectionSignature = "";
  let pendingSelectionAtMs = 0;
  let destroyed = false;
  let failed = "";
  let frameNumber = 0;
  let lastSize = { width: 0, height: 0 };
  let hasProjection = false;
  let collectMarkerDiagnostics = false;
  const stats = {
    drawn: 0,
    skipped: 0,
    selectionSuppressed: 0,
    billboarded: 0,
    farRouteAnchors: 0,
    cacheCreates: 0,
    cacheDisposes: 0,
    contentRepaints: 0,
    geometryUpdates: 0,
    groupRebuilds: 0,
    lifecycleTransitions: 0,
    lifecycleReplacements: 0,
    trackingAlpha: 0,
    lastReason: "",
    contextLost: false,
    markers: [],
    visibilityStates: {},
  };

  function recordMarkerDiagnostic(item, anchor, metrics, plan, state, reason = "") {
    const visibilityState = String(state || AR_MARKER_VISIBILITY_STATE.PROJECTION_UNAVAILABLE);
    stats.visibilityStates[visibilityState] = (stats.visibilityStates[visibilityState] || 0) + 1;
    if (!collectMarkerDiagnostics) return;
    if (stats.markers.length >= 8) return;
    const number = (value) => Number.isFinite(Number(value)) ? Number(value) : null;
    stats.markers.push(Object.freeze({
      markerId: String(item?.markerId || item?.eventKey || ""),
      lifecycleSlot: String(item?.lifecycleSlot || markerLifecycleSlot(item) || ""),
      source: String(item?.source || "unknown"),
      kind: String(item?.descriptor?.kind || ""),
      distanceM: number(item?.distanceM),
      visible: visibilityState === AR_MARKER_VISIBILITY_STATE.ACTIVE_VISIBLE,
      visibilityState,
      reason: String(reason || markerStateReason(visibilityState)),
      phase: String(plan?.phase || ""),
      confidence: String(item?.confidence || "high"),
      alpha: number(plan?.alpha),
      worldScale: number(plan?.worldScale),
      anchor: anchor ? Object.freeze({
        x: number(anchor.x), y: number(anchor.y), z: number(anchor.z),
        headingRad: number(anchor.headingRad), billboardYawRad: number(anchor.billboardYawRad),
        roadGrade: number(anchor.roadGrade),
        heightSource: String(anchor.heightSource || ""),
        heightConfidence: number(anchor.heightConfidence),
        heightLimited: anchor.heightLimited === true,
        routeDerived: anchor.routeDerived === true,
      }) : null,
      screen: metrics ? Object.freeze({
        centerX: number(metrics.centerX), centerY: number(metrics.centerY),
        widthPx: number(metrics.widthPx),
        heightPx: number(metrics.heightPx),
        depth: number(metrics.depth),
      }) : null,
    }));
  }

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
    stats.markers = [];
    stats.visibilityStates = {};
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

  function retainPassingEntry(cache, used, key, item, anchor, projectionState, nowMs) {
    const entry = cache.get(key);
    if (!entry) return false;
    const wasPresented = entry.presentationState === AR_MARKER_VISIBILITY_STATE.ACTIVE_VISIBLE
      || entry.presentationState === AR_MARKER_VISIBILITY_STATE.PASSING;
    if (!wasPresented) {
      entry.presentationState = projectionState;
      entry.group.visible = false;
      entry.lastSeenAt = nowMs;
      return false;
    }
    used.add(key);
    entry.lastSeenAt = nowMs;
    entry.presentationState = AR_MARKER_VISIBILITY_STATE.PASSING;
    const alpha = transitionOpacity(entry, false, nowMs);
    entry.group.visible = alpha > 0;
    if (entry.group.visible) {
      setThreeSignboardOpacity(entry.group, entry.lastPlanAlpha * alpha, entry.lastHeld);
      stats.drawn += 1;
    }
    recordMarkerDiagnostic(
      item,
      anchor,
      null,
      null,
      AR_MARKER_VISIBILITY_STATE.PASSING,
      `${markerStateReason(projectionState)}; passing fade`,
    );
    return true;
  }

  function markPresentEntryHidden(cache, key, state, nowMs) {
    const entry = cache.get(key);
    if (!entry) return;
    entry.presentationState = state;
    entry.group.visible = false;
    entry.lastSeenAt = nowMs;
  }

  function hideCrossedUprightEntry(cache, key, state, nowMs) {
    const entry = cache.get(key);
    if (!entry) return;
    entry.presentationState = state;
    entry.group.visible = false;
    entry.lastSeenAt = nowMs;
    // Drain the lifecycle alpha while the stale transform is hidden so the
    // cache can retire normally after its grace period.
    transitionOpacity(entry, false, nowMs);
  }

  function selectionSignature(candidates) {
    return candidates.map((candidate) => String(candidate.key || "")).sort().join("|");
  }

  function stableMarkerSelection(candidates, canvas, nowMs) {
    const proposed = selectVisibleMarkers(candidates, canvas, {
      preferredKeys: selectedSignKeys,
    });
    const previousAvailable = candidates.some((candidate) => selectedSignKeys.has(candidate.key));
    if (!previousAvailable) {
      pendingSelectionSignature = "";
      pendingSelectionAtMs = 0;
      return proposed;
    }
    const continuous = selectVisibleMarkers(candidates, canvas, {
      preferredKeys: selectedSignKeys,
      preferContinuity: true,
    });
    const proposedSignature = selectionSignature(proposed);
    if (proposedSignature === selectionSignature(continuous)) {
      pendingSelectionSignature = "";
      pendingSelectionAtMs = 0;
      return proposed;
    }
    if (pendingSelectionSignature !== proposedSignature) {
      pendingSelectionSignature = proposedSignature;
      pendingSelectionAtMs = nowMs;
      return continuous;
    }
    if (nowMs - pendingSelectionAtMs < AR_RENDER.selectionHysteresisMs) return continuous;
    pendingSelectionSignature = "";
    pendingSelectionAtMs = 0;
    return proposed;
  }

  function useEntry(cache, root, key, item, factories, nowMs) {
    const descriptor = item.descriptor;
    const contentKey = descriptorContentKey(descriptor);
    const geometryKey = descriptorGeometryKey(descriptor, factories);
    const lifecycleSlot = item.lifecycleSlot || markerLifecycleSlot(item);
    let entry = cache.get(key);
    if (!entry) {
      const group = createThreeSignboardGroup(descriptor, factories);
      entry = {
        group,
        contentKey,
        geometryKey,
        lastSeenAt: nowMs,
        opacityAt: nowMs,
        visibilityAlpha: 0.01,
        lastPlanAlpha: 1,
        lastHeld: false,
        lifecycleSlot,
        presentationState: null,
        presentation: createMarkerPresentationFilter(),
      };
      cache.set(key, entry);
      root.add(group);
      stats.cacheCreates += 1;
    } else {
      let groupRebuilt = false;
      if (entry.lifecycleSlot !== lifecycleSlot) stats.lifecycleTransitions += 1;
      if (entry.geometryKey !== geometryKey) {
        const updated = updateThreeSignboardGroupGeometry(entry.group, descriptor, factories);
        if (updated === null) {
          disposeThreeSignboardGroup(entry.group);
          stats.cacheDisposes += 1;
          stats.groupRebuilds += 1;
          entry.group = createThreeSignboardGroup(descriptor, factories);
          stats.cacheCreates += 1;
          root.add(entry.group);
          groupRebuilt = true;
        } else {
          stats.geometryUpdates += 1;
        }
        entry.geometryKey = geometryKey;
      }
      if (
        entry.contentKey !== contentKey
        && !groupRebuilt
        && !repaintThreeSignboardGroup(entry.group, descriptor)
      ) {
        disposeThreeSignboardGroup(entry.group);
        stats.cacheDisposes += 1;
        stats.groupRebuilds += 1;
        entry.group = createThreeSignboardGroup(descriptor, factories);
        stats.cacheCreates += 1;
        root.add(entry.group);
      } else if (entry.contentKey !== contentKey && !groupRebuilt) {
        stats.contentRepaints += 1;
      }
      entry.contentKey = contentKey;
    }
    entry.lifecycleSlot = lifecycleSlot;
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
        stats.cacheDisposes += 1;
        stats.lifecycleReplacements += 1;
        cache.delete(key);
        continue;
      }
      const alpha = transitionOpacity(entry, false, nowMs);
      const wasPresented = entry.presentationState === AR_MARKER_VISIBILITY_STATE.ACTIVE_VISIBLE
        || entry.presentationState === AR_MARKER_VISIBILITY_STATE.PASSING;
      if (wasPresented) entry.presentationState = AR_MARKER_VISIBILITY_STATE.PASSING;
      entry.group.visible = wasPresented && alpha > 0;
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
        stats.cacheDisposes += 1;
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
      const surfacePlacement = {
        surfacePitchRad: AR_RENDER.laneBandPresentation.viewPitchRad,
      };
      const projection = markerProjectionState(
        descriptor,
        anchor,
        frame.stage,
        surfacePlacement,
      );
      const metrics = projection.metrics;
      if (!anchor || !metrics) {
        const passing = anchor && (
          projection.state === AR_MARKER_VISIBILITY_STATE.NEAR_PLANE
          || projection.state === AR_MARKER_VISIBILITY_STATE.BEHIND_CAMERA
        ) && retainPassingEntry(
          surfaceCache, used, key, item, anchor, projection.state, nowMs,
        );
        if (!passing) {
          markPresentEntryHidden(surfaceCache, key, projection.state, nowMs);
          stats.skipped += 1;
          recordMarkerDiagnostic(item, anchor, metrics, null, projection.state);
        }
        continue;
      }
      const plan = planMarker({
        distanceM: item.distanceM,
        egoSpeedMps: frame.egoSpeedMps,
        // BAND는 수평면이므로 폭을 화면 크기 clamp의 광학 높이로 사용한다.
        worldHeightM: metrics.opticalHeightM,
        // Only road-longitudinal length receives far-legibility scaling. Lane
        // width stays physical and is checked from the target projection.
        worldWidthM: null,
        focalPx: metrics.focalPx,
        centerX: null,
        projectedHeightPx: metrics.heightPx,
        projectedWidthPx: null,
        descriptorScale: descriptor.scale,
        confidence: item.confidence || "high",
        minimumWorldScale: AR_RENDER.presentation.nearWorldScaleMin,
        maximumWorldScale: AR_RENDER.laneBandPresentation.farLengthScaleMax,
        canvas: {
          width: Number(frame.stage?.stageWidth) || size.width,
          height: Number(frame.stage?.stageHeight) || size.height,
        },
      });
      if (!plan.visible) {
        markPresentEntryHidden(
          surfaceCache, key, AR_MARKER_VISIBILITY_STATE.PLAN_REJECTED, nowMs,
        );
        stats.skipped += 1;
        recordMarkerDiagnostic(
          item, anchor, metrics, plan, AR_MARKER_VISIBILITY_STATE.PLAN_REJECTED,
        );
        continue;
      }

      const targetProjection = markerProjectionState(descriptor, anchor, frame.stage, {
        ...surfacePlacement,
        presentationHeightScale: plan.worldScale,
        presentationWidthScale: plan.geometryScale,
      });
      const targetMetrics = targetProjection.metrics;
      if (!targetMetrics) {
        const passing = (
          targetProjection.state === AR_MARKER_VISIBILITY_STATE.NEAR_PLANE
          || targetProjection.state === AR_MARKER_VISIBILITY_STATE.BEHIND_CAMERA
        ) && retainPassingEntry(
          surfaceCache, used, key, item, anchor, targetProjection.state, nowMs,
        );
        if (!passing) {
          markPresentEntryHidden(surfaceCache, key, targetProjection.state, nowMs);
          stats.skipped += 1;
          recordMarkerDiagnostic(item, anchor, null, plan, targetProjection.state);
        }
        continue;
      }
      const canvas = {
        width: Number(frame.stage?.stageWidth) || size.width,
        height: Number(frame.stage?.stageHeight) || size.height,
      };
      const halfWidth = targetMetrics.widthPx / 2;
      const halfHeight = targetMetrics.heightPx / 2;
      const screenBounds = Object.freeze({
        left: targetMetrics.centerX - halfWidth,
        right: targetMetrics.centerX + halfWidth,
        top: targetMetrics.centerY - halfHeight,
        bottom: targetMetrics.centerY + halfHeight,
      });
      const targetTooLarge = targetMetrics.heightPx > canvas.height
          * AR_RENDER.laneBandPresentation.maxScreenHeightRatio
        || targetMetrics.widthPx > canvas.width
          * AR_RENDER.laneBandPresentation.maxScreenWidthRatio;
      const viewportState = markerViewportState(screenBounds, canvas, {
        paddingPx: surfaceCache.has(key)
          ? AR_RENDER.viewportExitPaddingPx
          : AR_RENDER.viewportEnterPaddingPx,
      });
      if (targetTooLarge || viewportState === AR_MARKER_VISIBILITY_STATE.ACTIVE_OFFSCREEN) {
        const state = targetTooLarge
          ? AR_MARKER_VISIBILITY_STATE.NEAR_PLANE
          : viewportState;
        const passing = targetTooLarge && retainPassingEntry(
          surfaceCache, used, key, item, anchor, state, nowMs,
        );
        if (!passing) {
          markPresentEntryHidden(surfaceCache, key, state, nowMs);
          stats.skipped += 1;
          recordMarkerDiagnostic(item, anchor, targetMetrics, plan, state);
        }
        continue;
      }

      const entry = useEntry(surfaceCache, surfaceRoot, key, item, {
        canvasFactory: options.canvasFactory,
        textureFactory: options.textureFactory,
        anisotropy: options.anisotropy || 8,
        ...surfacePlacement,
      }, nowMs);
      used.add(key);
      entry.group.visible = true;
      entry.presentationState = AR_MARKER_VISIBILITY_STATE.ACTIVE_VISIBLE;
      const trackingAlpha = clamp01(frame.tracking?.alpha ?? 1);
      const legacyHeld = frame.held === true && !frame.tracking;
      entry.lastPlanAlpha = plan.alpha * trackingAlpha;
      entry.lastHeld = legacyHeld;
      const presentation = entry.presentation.update(anchor, plan.presentationScale, nowMs);
      const descriptorScale = Math.max(1e-6, Number(descriptor.scale) || 1);
      const lengthScale = presentation.scale / descriptorScale;
      const widthScale = plan.distanceScale / descriptorScale;
      placeThreeSignboardGroup(entry.group, presentation.anchor, {
        scale: 1,
        presentationScale: lengthScale,
        presentationWidthScale: widthScale,
        alpha: plan.alpha * trackingAlpha * transitionOpacity(entry, true, nowMs),
        held: legacyHeld,
      });
      if (collectMarkerDiagnostics) {
        recordMarkerDiagnostic(
          item,
          presentation.anchor,
          markerScreenMetrics(descriptor, presentation.anchor, frame.stage, {
            ...surfacePlacement,
            presentationHeightScale: lengthScale,
            presentationWidthScale: widthScale,
          }) || targetMetrics,
          plan,
          AR_MARKER_VISIBILITY_STATE.ACTIVE_VISIBLE,
        );
      }
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
      const anchor = uprightPresentationAnchor(item.anchor, AR_BILLBOARD.maxYawRad);
      const placement = { supportHeightM: markerSupportHeightM(item.descriptor) };
      const projection = markerProjectionState(item.descriptor, anchor, frame.stage, placement);
      const metrics = projection.metrics;
      if (!anchor || !metrics) {
        // An upright marker that crossed the camera near plane must leave the
        // view with its world point. Retaining its last pre-clip transform
        // leaves a viewport-sized stale sign stuck to one edge for 420 ms.
        // Cache the object for continuity, but never draw that stale pose.
        hideCrossedUprightEntry(signCache, key, projection.state, nowMs);
        stats.skipped += 1;
        recordMarkerDiagnostic(item, anchor, metrics, null, projection.state);
        continue;
      }
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
        confidence: item.confidence || "high",
        minimumWorldScale: AR_RENDER.presentation.nearWorldScaleMin,
        canvas,
      });
      if (!plan.visible) {
        markPresentEntryHidden(
          signCache, key, AR_MARKER_VISIBILITY_STATE.PLAN_REJECTED, nowMs,
        );
        stats.skipped += 1;
        recordMarkerDiagnostic(
          item, anchor, metrics, plan, AR_MARKER_VISIBILITY_STATE.PLAN_REJECTED,
        );
        continue;
      }

      const targetProjection = markerProjectionState(
        item.descriptor,
        anchor,
        frame.stage,
        { ...placement, presentationScale: plan.worldScale },
      );
      const targetMetrics = targetProjection.metrics || metrics;

      const halfWidth = targetMetrics.widthPx / 2;
      const halfHeight = targetMetrics.heightPx / 2;
      const screenBounds = Object.freeze({
        left: targetMetrics.centerX - halfWidth,
        right: targetMetrics.centerX + halfWidth,
        top: targetMetrics.centerY - halfHeight,
        bottom: targetMetrics.centerY + halfHeight,
      });
      const viewportState = markerViewportState(screenBounds, canvas, {
        paddingPx: signCache.has(key)
          ? AR_RENDER.viewportExitPaddingPx
          : AR_RENDER.viewportEnterPaddingPx,
      });
      if (viewportState === AR_MARKER_VISIBILITY_STATE.ACTIVE_OFFSCREEN) {
        markPresentEntryHidden(signCache, key, viewportState, nowMs);
        stats.skipped += 1;
        recordMarkerDiagnostic(item, anchor, metrics, plan, viewportState);
        continue;
      }
      candidates.push({
        item,
        key,
        anchor,
        metrics: targetMetrics,
        plan,
        placement,
        distanceM: item.distanceM,
        screenBounds,
      });
    }

    const selected = stableMarkerSelection(candidates, canvas, nowMs);
    if (candidates.length > 0) {
      selectedSignKeys.clear();
      for (const candidate of selected) selectedSignKeys.add(candidate.key);
    }
    stats.selectionSuppressed += candidates.length - selected.length;
    stats.skipped += candidates.length - selected.length;
    const selectedCandidates = new Set(selected);
    for (const candidate of candidates) {
      if (selectedCandidates.has(candidate)) continue;
      markPresentEntryHidden(
        signCache, candidate.key, AR_MARKER_VISIBILITY_STATE.ACTIVE_SUPPRESSED, nowMs,
      );
      recordMarkerDiagnostic(
        candidate.item,
        candidate.anchor,
        candidate.metrics,
        candidate.plan,
        AR_MARKER_VISIBILITY_STATE.ACTIVE_SUPPRESSED,
      );
    }
    for (const candidate of selected) {
      const { item, key, anchor, plan, placement } = candidate;
      if (Math.abs(Number(anchor.billboardYawRad) || 0) > 1e-4) stats.billboarded += 1;
      if (item.anchor?.routeDerived === true) stats.farRouteAnchors += 1;

      const entry = useEntry(signCache, signRoot, key, item, {
        canvasFactory: options.canvasFactory,
        textureFactory: options.textureFactory,
        shadowTextureFactory: options.shadowTextureFactory,
        anisotropy: options.anisotropy || 8,
        ...placement,
      }, nowMs);
      used.add(key);
      entry.group.visible = true;
      entry.presentationState = AR_MARKER_VISIBILITY_STATE.ACTIVE_VISIBLE;
      const trackingAlpha = clamp01(frame.tracking?.alpha ?? 1);
      const legacyHeld = frame.held === true && !frame.tracking;
      entry.lastPlanAlpha = plan.alpha * trackingAlpha;
      entry.lastHeld = legacyHeld;
      const presentation = entry.presentation.update(
        anchor,
        plan.presentationScale,
        nowMs,
      );
      const descriptorScale = Math.max(1e-6, Number(item.descriptor?.scale) || 1);
      const componentScale = presentation.scale / descriptorScale;
      placeThreeSignboardGroup(entry.group, presentation.anchor, {
        scale: 1,
        presentationScale: componentScale,
        alpha: plan.alpha * trackingAlpha * transitionOpacity(entry, true, nowMs),
        held: legacyHeld,
      });
      if (collectMarkerDiagnostics) {
        recordMarkerDiagnostic(
          item,
          presentation.anchor,
          markerScreenMetrics(item.descriptor, presentation.anchor, frame.stage, {
            ...placement,
            presentationScale: componentScale,
          }) || candidate.metrics,
          plan,
          AR_MARKER_VISIBILITY_STATE.ACTIVE_VISIBLE,
        );
      }
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
      stats.trackingAlpha = clamp01(frame.tracking?.alpha ?? 1);
      stats.markers = [];
      stats.visibilityStates = {};
      collectMarkerDiagnostics = frame.diagnosticsEnabled === true;
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
    const activeEntries = [...signCache.values(), ...surfaceCache.values()]
      .filter((entry) => entry.group.visible);
    const minimumVisibilityAlpha = activeEntries.length
      ? Math.min(...activeEntries.map((entry) => clamp01(entry.visibilityAlpha)))
      : 0;
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
      cacheCreates: stats.cacheCreates,
      cacheDisposes: stats.cacheDisposes,
      contentRepaints: stats.contentRepaints,
      geometryUpdates: stats.geometryUpdates,
      groupRebuilds: stats.groupRebuilds,
      lifecycleTransitions: stats.lifecycleTransitions,
      lifecycleReplacements: stats.lifecycleReplacements,
      trackingAlpha: stats.trackingAlpha,
      minimumVisibilityAlpha,
      lastReason: stats.lastReason,
      markers: Object.freeze(stats.markers.slice()),
      visibilityStates: Object.freeze({ ...stats.visibilityStates }),
      textureCount: signCache.size + surfaceCache.size,
      canvas: `${lastSize.width}x${lastSize.height}`,
    });
  }

  function reset() {
    if (destroyed) return false;
    for (const entry of surfaceCache.values()) {
      disposeThreeSignboardGroup(entry.group);
      stats.cacheDisposes += 1;
    }
    surfaceCache.clear();
    for (const entry of signCache.values()) {
      disposeThreeSignboardGroup(entry.group);
      stats.cacheDisposes += 1;
    }
    signCache.clear();
    selectedSignKeys.clear();
    pendingSelectionSignature = "";
    pendingSelectionAtMs = 0;
    hasProjection = false;
    clearFrame("reset");
    return true;
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    canvas.removeEventListener?.("webglcontextlost", onContextLost);
    for (const entry of surfaceCache.values()) {
      disposeThreeSignboardGroup(entry.group);
      stats.cacheDisposes += 1;
    }
    surfaceCache.clear();
    for (const entry of signCache.values()) {
      disposeThreeSignboardGroup(entry.group);
      stats.cacheDisposes += 1;
    }
    signCache.clear();
    selectedSignKeys.clear();
    renderer.dispose?.();
    renderer.forceContextLoss?.();
    return true;
  }

  return Object.freeze({ canvas, render, reset, status, destroy });
}
