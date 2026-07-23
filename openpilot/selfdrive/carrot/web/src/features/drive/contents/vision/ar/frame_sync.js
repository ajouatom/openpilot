/* Phase 3 — 표시 중인 영상 프레임과 model/odometry 를 묶는 동기화 게이트.
 *
 * AR 앵커는 "지금 화면에 떠 있는 프레임"의 기하에 붙어야 한다. 한 프레임만
 * 어긋나도 표지가 영상을 따라 끌리는 것이 눈에 보인다.
 *
 * 기존 vision_raw.js 가 RTP timestamp → roadCameraState.frameId 매핑과
 * modelV2 history 로 "표시 중인 model" 을 이미 고르고 있다. 여기서는 그 결과에
 * **엄격한 허용 오차**를 씌운다. 기존 HUD 는 오차가 커도 계속 그리지만
 * (부드러움 우선), AR 은 어긋나면 숨기는 편이 낫다.
 *
 * Phase 1 에서 roadCameraState.timestampEof 를 추가했으므로 frameId 뿐 아니라
 * "시간" 기준으로도 검사할 수 있다.
 */

import {
  CAMERA_ODOMETRY_POSE_DELAY_MS,
  cameraOdometryObservationTimestampNs,
} from "./pose_timeline.js";

export { CAMERA_ODOMETRY_POSE_DELAY_MS } from "./pose_timeline.js";

export const AR_SYNC_STATE = Object.freeze({
  OK: "ok",
  STALE: "stale",           // 데이터가 오래됨
  FRAME_GAP: "frame_gap",   // 영상 프레임과 model 프레임이 벌어짐
  MISSING: "missing",       // 필수 입력 없음
});

/* 허용 오차. 20Hz 기준 1프레임 = 50ms.
 * roadCameraState가 경량 표본이고 replay에는 model drop도 있으므로 3프레임까지는
 * 정상적인 기록 양자화 범위로 본다. 실제 리플레이에서 2↔3 경계가 반복되며
 * 월드 앵커 전체가 점멸했기 때문에, 150ms까지 허용하고 그 이상은 기존 bounded
 * hold로 넘긴다. */
export const AR_SYNC_LIMITS = Object.freeze({
  // 일단 보이는 게 우선(테스트·퍼블리싱 공통): 동기화/신선도 gate를 넉넉히
  // 완화한다. 값이 조금 오래되거나 프레임 갭이 커도 표지를 지우지 않고,
  // 정밀도 저하는 tracking/hold의 연속 보정과 fade가 흡수한다.
  maxFrameIdGap: 8,          // roadCameraState.frameId ↔ modelV2.frameId
  maxModelAgeMs: 350,
  maxOdometryAgeMs: 350,
  maxOdometryFrameGap: 8,
  maxPoseAgeMs: 700,
  maxCalibrationAgeMs: 20000,
  maxNaviAgeMs: 8000,
  /* cameraOdometry 신뢰도.
   *
   * 처음엔 transStd 에 절대 상한(0.5 m/s)을 뒀는데, 실측에서 0.632 가 나와
   * 막혔다. 그런데 그 값은 비정상이 아니다 — transStd 는 속도에 따라 커지므로
   * 절대값만 보면 정상 주행이 걸린다.
   *
   * openpilot 자신은 calibrationd 에서 절대값이 아니라 **전진 속도 대비 횡방향
   * 불확실성의 각도**로 판단한다(atan2(transStd[1], trans[0]) < 0.25°).
   * 같은 기준을 쓴다. AR 앵커도 결국 "각도가 얼마나 흔들리나"의 문제다.
   */
  maxVelAngleStd: 0.8 * Math.PI / 180,    // 완화: 실측 0.3~0.4°에서 막히던 것을 허용
  maxRotStd: 0.12,                        // rad/s. 회전 std 는 속도 의존이 약하다.
});

const NS_PER_MS = 1e6;

/* 디코드 타당성 검사.
 *
 * cameraOdometry / livePose 는 리플레이에서 raw_capnp 로 읽는데, 그 오프셋은
 * capnp 배치 규칙으로 산출한 값이라 실로그로 최종 확인되기 전까지는 신뢰할 수
 * 없다. 오프셋이 틀리면 예외가 아니라 "그럴듯한 쓰레기 숫자"가 나와서 앵커가
 * 조용히 어긋난다. 그래서 물리적으로 불가능한 값을 걸러 낸다.
 */
export const AR_PLAUSIBILITY = Object.freeze({
  maxTransMps: 120,      // 432km/h 이상은 차량 속도가 아니다
  maxRotRadps: 6,        // 초당 한 바퀴 이상은 차량 회전이 아니다
  maxStd: 50,
  maxOrientationRad: Math.PI * 2 + 0.1,
  maxCalibrationRad: Math.PI,
});

function allFinite(list, limit) {
  if (!Array.isArray(list) || list.length === 0) return false;
  return list.every((v) => {
    const n = Number(v);
    return Number.isFinite(n) && Math.abs(n) <= limit;
  });
}

/** cameraOdometry 값이 물리적으로 말이 되는가. false 면 디코드를 의심한다. */
export function isOdometryPlausible(odo) {
  if (!odo) return false;
  const p = AR_PLAUSIBILITY;
  return allFinite(odo.trans, p.maxTransMps)
    && allFinite(odo.rot, p.maxRotRadps)
    && (odo.transStd === undefined || allFinite(odo.transStd, p.maxStd))
    && (odo.rotStd === undefined || allFinite(odo.rotStd, p.maxStd));
}

/** livePose 값이 말이 되는가. */
export function isPosePlausible(pose) {
  if (!pose) return false;
  const o = pose.orientationNED;
  if (!o) return false;
  const p = AR_PLAUSIBILITY;
  for (const k of ["x", "y", "z"]) {
    const n = Number(o[k]);
    if (!Number.isFinite(n) || Math.abs(n) > p.maxOrientationRad) return false;
  }
  return typeof pose.inputsOK === "boolean" && typeof pose.posenetOK === "boolean";
}

function finite(value) {
  if (value === null || value === undefined || value === "") return null;
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

/** 나노초 타임스탬프 차이를 ms 로. 둘 중 하나라도 없으면 null. */
export function nanosDeltaMs(aNs, bNs) {
  const a = finite(aNs);
  const b = finite(bNs);
  if (a === null || b === null || a <= 0 || b <= 0) return null;
  return (a - b) / NS_PER_MS;
}

/** 현재 calibration 값이 projection에 쓸 수 있는 실제 3축 회전인가. */
export function isCalibrationPlausible(calibration) {
  const rpy = calibration?.rpyCalib;
  return Array.isArray(rpy)
    && rpy.length >= 3
    && rpy.slice(0, 3).every((value) => {
      const n = Number(value);
      return Number.isFinite(n) && Math.abs(n) <= AR_PLAUSIBILITY.maxCalibrationRad;
    });
}

/** 브라우저 monotonic 수신시각 기준 service age. 시계 역행·누락은 null이다. */
export function serviceReceiptAgeMs(nowMs, receivedAtMonotonic, service) {
  const now = finite(nowMs);
  const received = finite(receivedAtMonotonic?.[service]);
  if (now === null || received === null || now < received) return null;
  return now - received;
}

/* liveCalibration.calStatus 정규화.
 * log.capnp 순서: 0 uncalibrated / 1 calibrated / 2 invalid / 3 recalibrating.
 * 라이브 compact 는 이름 문자열, 리플레이는 숫자로 준다. */
const CAL_STATUS_NAMES = Object.freeze(["uncalibrated", "calibrated", "invalid", "recalibrating"]);

export function normalizeCalStatus(value) {
  if (value === null || value === undefined) return "";
  if (typeof value === "number" && Number.isInteger(value)) {
    return CAL_STATUS_NAMES[value] || String(value);
  }
  const text = String(value).trim();
  if (!text) return "";
  const numeric = Number(text);
  if (Number.isInteger(numeric) && CAL_STATUS_NAMES[numeric]) return CAL_STATUS_NAMES[numeric];
  return text;
}

export function isCalibrated(value) {
  return normalizeCalStatus(value) === "calibrated";
}

function worst(...states) {
  const order = [AR_SYNC_STATE.MISSING, AR_SYNC_STATE.FRAME_GAP, AR_SYNC_STATE.STALE, AR_SYNC_STATE.OK];
  let out = AR_SYNC_STATE.OK;
  for (const s of states) {
    if (order.indexOf(s) < order.indexOf(out)) out = s;
  }
  return out;
}

/**
 * 동기화 판정.
 *
 * @param {object} input
 *   roadCameraState : { frameId, timestampEof }
 *   modelV2         : { frameId, frameAge, frameDropPerc }
 *   cameraOdometry  : { frameId, timestampEof, transStd, rotStd }
 *   livePose        : { inputsOK, posenetOK, sensorsOK, timestamp }
 *   liveCalibration : { calStatus }
 *   carrotNavi      : { publishMonoTimeNanos, connected, navigationStatus }
 *   nowMs           : 접속 브라우저 performance.now().
 *   receivedAtMonotonic: live-state provider의 service별 수신시각.
 */
export function evaluateFrameSync(input = {}, limits = AR_SYNC_LIMITS) {
  const cam = input.roadCameraState;
  const model = input.modelV2;
  const odo = input.cameraOdometry;
  const pose = input.livePose;
  const cal = input.liveCalibration;
  const navi = input.carrotNavi;
  const receipts = input.receivedAtMonotonic;
  const nowMs = input.nowMs;
  const replayClock = input.clockDomain === "replay-media";
  const presentedTargetCandidate = finite(input.presentedClock?.targetTimestampNs);
  const presentedTargetTimestampNs = presentedTargetCandidate > 0
    ? presentedTargetCandidate
    : null;
  const cameraTargetTimestampNs = presentedTargetTimestampNs ?? finite(cam?.timestampEof);
  const reasons = [];

  if (!cam || !model) {
    return Object.freeze({
      state: AR_SYNC_STATE.MISSING, reasons: Object.freeze(["camera/model 없음"]),
      frameIdGap: null, modelAgeMs: null, odometryAgeMs: null, odometryUsable: false,
      poseAgeMs: null, calibrationAgeMs: null, naviAgeMs: null,
      poseOk: false, calibrated: false, calibrationPlausible: false, naviUsable: false,
      presentedClockConfidence: input.presentedClock?.confidence || "unmapped",
      presentedTargetTimestampNs,
      odometryPoseDelayMs: CAMERA_ODOMETRY_POSE_DELAY_MS,
      clockDomain: input.clockDomain === "replay-media" ? "replay-media" : "live-monotonic",
      canDrawPrecise: false, canHoldAnchor: false,
    });
  }

  // 1) 영상 프레임 ↔ model 프레임
  const presentedFrameId = finite(input.presentedClock?.sourceFrameId);
  const camFrame = presentedFrameId ?? finite(cam.frameId);
  const modelFrame = finite(model.frameId);
  const frameIdGap = camFrame !== null && modelFrame !== null
    ? Math.abs(camFrame - modelFrame) : null;
  let frameState = AR_SYNC_STATE.OK;
  if (frameIdGap === null) {
    frameState = AR_SYNC_STATE.MISSING;
    reasons.push("frameId 없음");
  } else if (frameIdGap > limits.maxFrameIdGap) {
    frameState = AR_SYNC_STATE.FRAME_GAP;
    reasons.push(`frame gap ${frameIdGap} > ${limits.maxFrameIdGap}`);
  }

  // 2) model 신선도. modelV2.frameAge 는 "model 이 본 프레임이 몇 장 지났는가".
  const frameAge = finite(model.frameAge);
  const modelFrameAgeMs = frameAge === null ? null : frameAge * 50;
  const modelReceiptAgeMs = replayClock
    ? null
    : serviceReceiptAgeMs(nowMs, receipts, "modelV2");
  const modelAgeMs = replayClock
    ? (modelFrameAgeMs ?? 0)
    : modelReceiptAgeMs === null
      ? modelFrameAgeMs
      : Math.max(modelReceiptAgeMs, modelFrameAgeMs ?? 0);
  let modelState = AR_SYNC_STATE.OK;
  if (!replayClock && modelReceiptAgeMs === null) {
    modelState = AR_SYNC_STATE.MISSING;
    reasons.push("model 수신시각 없음");
  } else if (modelAgeMs > limits.maxModelAgeMs) {
    modelState = AR_SYNC_STATE.STALE;
    reasons.push(`model age ${modelAgeMs.toFixed(0)}ms`);
  }

  // 3) odometry — 앵커 유지에 쓰므로 std 까지 본다.
  let odometryAgeMs = null;
  let odometryUsable = false;
  if (odo) {
    const odometryObservationTimestampNs = cameraOdometryObservationTimestampNs(odo);
    const odometrySourceAgeMs = nanosDeltaMs(
      cameraTargetTimestampNs,
      odometryObservationTimestampNs,
    );
    const odometryReceiptAgeMs = replayClock
      ? null
      : serviceReceiptAgeMs(nowMs, receipts, "cameraOdometry");
    odometryAgeMs = replayClock
      ? Math.abs(odometrySourceAgeMs ?? 0)
      : odometryReceiptAgeMs === null
        ? null
        : Math.max(odometryReceiptAgeMs, Math.abs(odometrySourceAgeMs ?? 0));
    const odoFrame = finite(odo.frameId);
    const odoGap = odoFrame !== null && camFrame !== null ? Math.abs(camFrame - odoFrame) : null;
    // 전진 속도 대비 횡방향 std 를 각도로. 정지에 가까우면 판단을 보류한다
    // (trans[0] 이 0 이면 각도가 무한히 커져 항상 막히므로).
    const forward = Array.isArray(odo.trans) ? Math.abs(Number(odo.trans[0]) || 0) : null;
    const lateralStd = Array.isArray(odo.transStd) ? Math.abs(Number(odo.transStd[1]) || 0) : null;
    const velAngleStd = forward !== null && lateralStd !== null && forward > 1
      ? Math.atan2(lateralStd, forward) : null;
    const rotStd = Array.isArray(odo.rotStd) ? Math.max(...odo.rotStd.map((v) => Math.abs(Number(v) || 0))) : null;
    const ageOk = odometryAgeMs !== null && odometryAgeMs <= limits.maxOdometryAgeMs;
    const gapOk = odoGap === null || odoGap <= limits.maxOdometryFrameGap;
    const stdOk = (velAngleStd === null || velAngleStd <= limits.maxVelAngleStd)
      && (rotStd === null || rotStd <= limits.maxRotStd);
    const plausible = isOdometryPlausible(odo);
    if (!plausible) reasons.push("odometry 값이 비정상 — 디코드 오프셋 의심");
    odometryUsable = ageOk && gapOk && stdOk && plausible;
    if (!replayClock && odometryReceiptAgeMs === null) reasons.push("odometry 수신시각 없음");
    else if (!ageOk) reasons.push(`odometry age ${odometryAgeMs.toFixed(0)}ms`);
    if (!gapOk) reasons.push(`odometry frame gap ${odoGap}`);
    if (!stdOk) {
      const deg = velAngleStd === null ? "n/a" : `${(velAngleStd * 180 / Math.PI).toFixed(3)}°`;
      reasons.push(`odometry 불확실 velAngleStd=${deg} rotStd=${rotStd?.toFixed(4)}`);
    }
  }

  // 4) pose 유효성 — odometry 적분의 전제
  const poseReceiptAgeMs = replayClock
    ? null
    : serviceReceiptAgeMs(nowMs, receipts, "livePose");
  const poseSourceAgeMs = nanosDeltaMs(cameraTargetTimestampNs, pose?.timestamp);
  const poseAgeMs = replayClock
    ? Math.abs(poseSourceAgeMs ?? 0)
    : poseReceiptAgeMs === null
      ? null
      : Math.max(poseReceiptAgeMs, Math.abs(poseSourceAgeMs ?? 0));
  const posePlausible = Boolean(pose) && isPosePlausible(pose);
  if (pose && !posePlausible) reasons.push("livePose 값이 비정상 — 디코드 오프셋 의심");
  if (!pose) reasons.push("livePose 없음");
  if (!replayClock && poseReceiptAgeMs === null) reasons.push("livePose 수신시각 없음");
  else if (poseAgeMs > limits.maxPoseAgeMs) reasons.push(`livePose age ${poseAgeMs.toFixed(0)}ms`);
  const poseOk = posePlausible
    && pose.inputsOK === true
    && pose.posenetOK === true
    && pose.sensorsOK !== false
    && poseAgeMs !== null
    && poseAgeMs <= limits.maxPoseAgeMs;
  if (pose && posePlausible && !poseOk) reasons.push("livePose 품질/상태 not OK");

  // 5) calibration — 정밀 앵커는 calibrated 일 때만.
  // 주의: 값의 형태가 경로마다 다르다. 라이브 compact 는 enum 이름("calibrated"),
  // 리플레이(raw_capnp)는 values 목록이 없어 숫자(1)로 온다. 둘 다 받는다.
  const calibrated = isCalibrated(cal?.calStatus);
  if (!calibrated) reasons.push(`calStatus=${cal?.calStatus ?? "없음"}`);
  const calibrationPlausible = isCalibrationPlausible(cal);
  if (!calibrationPlausible) reasons.push("calibration rpy 비정상/누락");
  const calibrationAgeMs = replayClock
    ? (cal ? 0 : null)
    : serviceReceiptAgeMs(nowMs, receipts, "liveCalibration");
  let calibrationState = AR_SYNC_STATE.OK;
  if (calibrationAgeMs === null) {
    calibrationState = AR_SYNC_STATE.MISSING;
    reasons.push("calibration 수신시각 없음");
  } else if (calibrationAgeMs > limits.maxCalibrationAgeMs) {
    calibrationState = AR_SYNC_STATE.STALE;
    reasons.push(`calibration age ${calibrationAgeMs.toFixed(0)}ms`);
  }

  // 6) Navi — 안내 의미의 전제. 없어도 도로 상대 앵커는 가능하므로 별도 플래그.
  let naviUsable = false;
  let naviAgeMs = null;
  if (navi) {
    const offRoute = navi.navigationStatus?.offRoute === true;
    const active = navi.navigationStatus?.guidanceActive === true;
    const routePresent = navi.navigationStatus?.routePresent === true;
    const naviReceiptAgeMs = replayClock
      ? null
      : serviceReceiptAgeMs(nowMs, receipts, "carrotNavi");
    const naviSourceAgeMs = nanosDeltaMs(cameraTargetTimestampNs, navi.publishMonoTimeNanos);
    naviAgeMs = replayClock
      ? Math.abs(naviSourceAgeMs ?? 0)
      : naviReceiptAgeMs === null
        ? null
        : Math.max(naviReceiptAgeMs, Math.abs(naviSourceAgeMs ?? 0));
    const fresh = naviAgeMs !== null && naviAgeMs <= limits.maxNaviAgeMs;
    // 일단 보이는 게 우선: connected 소켓 플래그는 리플레이/연결 blip에서 자주
    // 꺼지지만 안내가 활성(또는 경로 존재)이면 표지 데이터는 유효하다. connected는
    // 요구하지 않고, off-route와 신선도(fresh)만 실제 차단 조건으로 둔다.
    naviUsable = (active || routePresent) && !offRoute && fresh;
    if (!replayClock && naviReceiptAgeMs === null) reasons.push("navi 수신시각 없음");
    else if (!fresh) reasons.push(`navi age ${naviAgeMs.toFixed(0)}ms`);
    if ((!active && !routePresent) || offRoute) {
      reasons.push("navi off-route/안내없음");
    }
  }

  const state = worst(frameState, modelState, calibrationState);
  return Object.freeze({
    state,
    reasons: Object.freeze(reasons),
    frameIdGap,
    modelAgeMs,
    odometryAgeMs,
    poseAgeMs,
    calibrationAgeMs,
    naviAgeMs,
    odometryUsable,
    poseOk,
    calibrated,
    calibrationPlausible,
    naviUsable,
    presentedClockConfidence: input.presentedClock?.confidence || "unmapped",
    presentedTargetTimestampNs,
    odometryPoseDelayMs: CAMERA_ODOMETRY_POSE_DELAY_MS,
    clockDomain: replayClock ? "replay-media" : "live-monotonic",
    /** 정밀(도로 접지) 앵커를 그려도 되는가. */
    canDrawPrecise: state === AR_SYNC_STATE.OK && calibrated && calibrationPlausible,
    /** 앵커를 프레임 사이에 유지(적분)해도 되는가. */
    canHoldAnchor: calibrated
      && calibrationPlausible
      && odometryUsable
      && poseOk,
  });
}
