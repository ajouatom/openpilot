/* Carrot Vision AR 오버레이 — 공개 파사드.
 *
 * 이 폴더의 공개 표면은 여기 하나다. 다른 기능은 내부 모듈을 직접 import 하지
 * 않는다(drive_insights 와 같은 규칙).
 *
 * 계층
 *   design_tokens / tokens   토큰 (색·배율·투명도·거리단계)
 *   signboard                표지 컴포넌트 (descriptor + 캔버스 페인터)
 *   tmap_catalog             TMap 필드 → 마커 descriptor
 *   projection / frame_sync  정합 (도로→화면 투영, 동기화 게이트)
 *   responsive               화면 크기 적응
 *   three_signboard_layout   승인 프리뷰의 renderer-independent 3D 배치
 *   three_adapter/worker     Three-only OffscreenCanvas 렌더 경계
 *   activation_gate/runtime 설정 opt-in·생명주기·데이터 임대·프레임 구독
 */

export {
  VISION_AR_SETTING_KEY,
  VISION_AR_AVAILABLE,
  readVisionArEnabled,
  createVisionArActivationGate,
} from "./activation_gate.js";

export {
  createArRuntime,
  getOrCreateArRuntime,
  installArRuntimeFacade,
} from "./runtime.js";

export {
  AR_CLOCK_DOMAIN,
  AR_TIMELINE_DISCONTINUITY,
  createArTimelineTracker,
} from "./timeline.js";

export {
  CAMERA_ODOMETRY_POSE_DELAY_MS,
  AR_PRESENTED_CLOCK_CONFIDENCE,
  cameraOdometryObservationTimestampNs,
  createCameraOdometryTimeline,
  createPresentedFrameClockMapper,
} from "./pose_timeline.js";

export {
  AR_TRACKING_STATE,
  AR_TRACKING_LIMITS,
  createArTrackingState,
} from "./tracking_state.js";

export {
  AR_WORLD_POSE_LIMITS,
  createDeviceWorldPose,
  normalizeQuaternion,
  multiplyQuaternions,
  quaternionFromRotationVector,
  rotateVectorByQuaternion,
  quaternionToRotationMatrix,
  worldFromDeviceMatrix,
  devicePointToWorld,
  worldPointToDevice,
} from "./world_pose.js";


export {
  AR_HOLD_STATE,
  AR_HOLD_LIMITS,
  createAnchorHold,
  advanceAnchor,
  driftIncrement,
} from "./anchor.js";

export {
  AR_ANCHOR_SOURCE_MODE,
  AR_ANCHOR_LIFECYCLE_STATE,
  AR_CONTINUOUS_LIMITS,
  naviFixIdentity,
  naviSourceIdentity,
  createContinuousAnchorStore,
} from "./anchor_store.js";

export {
  AR_SYNC_STATE,
  AR_SYNC_LIMITS,
  evaluateFrameSync,
} from "./frame_sync.js";

export {
  projectPoint,
  projectRouteFluPoint,
  routeFluProjectionDepth,
  pointOnPath,
  gateCorners,
  projectCorners,
} from "./projection.js";

export {
  AR_MARKER_VISIBILITY_STATE,
  markerStateReason,
  markerViewportState,
} from "./marker_state.js";

export { geoAnchor, routeDistanceAnchor, routeTangentHeading } from "./geo.js";
export {
  AR_DISCOVERY_POLICY,
  discoveryRangeM,
  isWithinDiscoveryRange,
} from "./discovery.js";
export {
  AR_ANCHOR_HANDOFF_POLICY,
  handoffProgress,
  planWorldAnchorHandoff,
  publicHandoffState,
  sampleWorldAnchorHandoff,
} from "./anchor_handoff.js";
export {
  AR_ROAD_HEIGHT_SOURCE,
  AR_ROAD_OBSERVATION_LIMITS,
  UNKNOWN_ROAD_HEIGHT,
  modelRoadObservation,
  limitedRoadLateralCorrection,
} from "./road_observation.js";
export {
  AR_ROUTE_MATCH_LIMITS,
  createRouteMatcher,
  matchRoutePosition,
} from "./route_matcher.js";

export {
  blendRoadFrames,
  constrainedBillboardAnchor,
  roadFrameFields,
  roadFrameForAnchor,
  roadFrameFromForward,
  roadFrameFromHeading,
  uprightRoadFrameForAnchor,
  rotateByRotationVector,
  rotateRoadFrame,
} from "./road_frame.js";

export {
  describeSignboard,
  signboardFromMarker,
  paintSignboard,
  textureSizeFor,
  formatDistance,
} from "./signboard.js";

export { describeMarkers, classifyTurnType, classifySdiType } from "./tmap_catalog.js";

export {
  AR_MARKER_IDENTITY_LIMITS,
  createMarkerIdentityTracker,
  markerIdentity,
  markerLifecycleSlot,
} from "./marker_identity.js";

export { deviceFromCalibMatrix, odometryInDeviceFrame } from "./odometry.js";

export {
  AR_COORDINATE_FRAME,
  AR_LIVE_POSE_FRAMES,
  assertCoordinateFrame,
  deviceFrdVectorToRouteFlu,
  routeFluVectorToDeviceFrd,
  modelPositionFrdToRouteFlu,
  deviceOdometryFrdToRouteFlu,
} from "./coordinate_frames.js";

export { AR_TRACE_LIMITS, createArTrace } from "./trace.js";

export {
  AR_TOKEN_PREVIEW_CONTRACT,
  AR_TONE,
  AR_SHAPE,
  AR_SCALE,
  AR_SIGN_HEIGHT_M,
  AR_BADGE_M,
  AR_OPACITY,
  paletteFor,
  shapeFor,
  assertScaleConsistency,
} from "./design_tokens.js";

export {
  AR_BILLBOARD,
  AR_PHASE,
  AR_MARKER_KIND,
  AR_MARKER_FORM,
  markerForm,
  phaseForDistance,
  viewportTier,
} from "./tokens.js";
export {
  distanceEmphasisScale,
  planMarker,
  resolveArCanvasSize,
  resolveWorldScale,
  selectVisibleMarkers,
} from "./responsive.js";
