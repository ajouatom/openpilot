/* Phase 4 — 월드 앵커와 "제한적 유지(bounded hold)".
 *
 * 문제: 동기화 게이트(frame_sync)는 어긋난 프레임을 정확히 막아 준다. 그런데
 * 막을 때마다 표지가 사라진다. model 이 한두 프레임 늦거나 frame gap 이 잠깐
 * 벌어지는 것은 주행 중 흔한 일이라, 그대로 두면 표지가 깜빡인다.
 * 깜빡이는 표지는 어긋난 표지만큼이나 나쁘다.
 *
 * 해법: 표지는 도로 위 한 점에 붙어 있고 그 점은 **세상에 고정**되어 있다.
 * 그러니 새 model 이 없어도 차가 얼마나 움직였는지만 알면 그 점이 지금 어디
 * 있는지 계산할 수 있다. cameraOdometry 가 그 속도를 준다.
 *
 * 왜 "제한적"인가: 적분은 오차가 쌓인다. 무한히 유지하면 표지가 슬금슬금
 * 엉뚱한 곳으로 간다. 그래서 두 가지 예산 중 하나라도 소진되면 즉시 놓는다.
 *   - 시간 예산: 유지 시작 후 최대 1.5s
 *   - 오차 예산: 누적 추정 횡방향 드리프트 1.0m
 * 예산을 넘으면 숨긴다. 틀린 위치의 표지보다 없는 표지가 낫다(Phase 3 와 같은 원칙).
 */

import {
  roadFrameFields,
  roadFrameForAnchor,
  rotateByRotationVector,
  rotateRoadFrame,
} from "./road_frame.js";
import { AR_COORDINATE_FRAME, assertCoordinateFrame } from "./coordinate_frames.js";

export const AR_HOLD_LIMITS = Object.freeze({
  // 일단 보이는 게 우선: 새 model fix 없이도 더 오래(odometry로) 유지하고,
  // 더 큰 drift까지 그린 채로 둔다.
  maxHoldMs: 3500,
  maxDriftM: 2.5,
  /* 적분 자체를 신뢰할 수 없는 dt. 탭 전환·끊김 후 되돌아온 경우
   * dt 가 크게 튀는데, 그걸 그대로 적분하면 표지가 순간이동한다. */
  maxStepMs: 200,
});

export const AR_HOLD_STATE = Object.freeze({
  LIVE: "live",       // 새 model 로 갱신 중
  HELD: "held",       // odometry 로 유지 중
  DROPPED: "dropped", // 예산 소진 — 그리지 않음
});

function finite(v, fallback = null) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

/**
 * 자차가 dt 동안 움직였을 때, 세상에 고정된 점의 자차 기준 좌표를 갱신한다.
 *
 * 좌표계는 AR route-local FLU: x 전방, y 좌측, z 상방.
 * cameraOdometry.trans 는 자차 속도(m/s), rot 은 각속도(rad/s).
 *
 * 자차가 앞으로 가면 점은 가까워지고(x 감소), 자차가 좌회전(+yaw)하면
 * 점은 자차 기준으로 우측으로 도는 것처럼 보인다(-yaw 회전).
 */
export function advanceAnchor(anchor, odometry, dtSeconds) {
  const dt = finite(dtSeconds, 0);
  if (!anchor || !odometry || !(dt > 0)) return anchor || null;
  // Pure legacy fixtures may omit metadata; any declared frame is strict.
  if (odometry.coordinateFrame !== undefined) {
    assertCoordinateFrame(odometry, AR_COORDINATE_FRAME.ROUTE_FLU, "anchor odometry");
  }

  const trans = Array.isArray(odometry.trans) ? odometry.trans : [];
  const rot = Array.isArray(odometry.rot) ? odometry.rot : [];
  const vx = finite(trans[0], 0);
  const vy = finite(trans[1], 0);
  const vz = finite(trans[2], 0);
  const rotationVector = [
    -finite(rot[0], 0) * dt,
    -finite(rot[1], 0) * dt,
    -finite(rot[2], 0) * dt,
  ];

  // A static world point seen from the next route-local frame is the inverse ego
  // motion: first remove translation, then apply the inverse 3D rotation.
  const position = rotateByRotationVector([
    finite(anchor.x, 0) - vx * dt,
    finite(anchor.y, 0) - vy * dt,
    finite(anchor.z, 0) - vz * dt,
  ], rotationVector);
  const frame = rotateRoadFrame(roadFrameForAnchor(anchor), rotationVector);

  return Object.freeze({
    ...anchor,
    x: position[0],
    y: position[1],
    z: position[2],
    ...roadFrameFields(frame),
  });
}

/**
 * 이번 스텝에서 늘어난 횡방향 오차 추정치(m).
 *
 * 두 갈래로 쌓인다.
 *   - 속도 오차: transStd[1] 을 dt 만큼 적분
 *   - 각속도 오차: rotStd[2] * dt 만큼 방향이 틀어지면, 거리 x 에서
 *     횡방향으로 x * (rotStd * dt) 만큼 벌어진다. 먼 표지일수록 빨리 나빠진다.
 *
 * 정확한 공분산 전파가 아니라 상한에 가까운 1차 근사다. 유지 시간을 짧게
 * 끊는 것이 목적이므로 보수적인 쪽이 맞다.
 */
export function driftIncrement(anchor, odometry, dtSeconds) {
  const dt = finite(dtSeconds, 0);
  if (!odometry || !(dt > 0)) return 0;
  const transStd = Array.isArray(odometry.transStd) ? odometry.transStd : [];
  const rotStd = Array.isArray(odometry.rotStd) ? odometry.rotStd : [];
  const translationStd = Math.hypot(
    finite(transStd[0], 0),
    finite(transStd[1], 0),
    finite(transStd[2], 0),
  );
  const rotationStd = Math.hypot(
    finite(rotStd[0], 0),
    finite(rotStd[1], 0),
    finite(rotStd[2], 0),
  );
  const distance = Math.hypot(
    finite(anchor?.x, 0),
    finite(anchor?.y, 0),
    finite(anchor?.z, 0),
  );
  return Math.abs(translationStd) * dt + Math.abs(rotationStd) * dt * distance;
}

/**
 * 앵커 유지기.
 *
 * 사용법 — 매 프레임 update() 한 번.
 *   canDrawPrecise 면 fresh 앵커를 넘긴다 → 그대로 통과시키고 예산을 되돌린다.
 *   아니면 fresh 없이 부른다 → 예산이 남아 있는 동안 적분해서 돌려준다.
 */
export function createAnchorHold(options = {}) {
  const limits = { ...AR_HOLD_LIMITS, ...(options.limits || {}) };
  let anchors = null;      // 마지막으로 믿을 수 있었던 앵커 목록
  let holdMs = 0;
  let driftM = 0;
  let lastAtMs = null;
  let state = AR_HOLD_STATE.DROPPED;
  let reason = "아직 유효한 앵커 없음";

  function reset() {
    anchors = null;
    holdMs = 0;
    driftM = 0;
    lastAtMs = null;
    state = AR_HOLD_STATE.DROPPED;
    reason = "초기화됨";
  }

  /**
   * @param input.nowMs      단조 증가 시각
   * @param input.fresh      canDrawPrecise 일 때의 앵커 배열. 아니면 null.
   * @param input.canHold    sync.canHoldAnchor (odometry 를 믿어도 되는가)
   * @param input.odometry   route-local FLU로 변환된 cameraOdometry
   */
  function update(input = {}) {
    const now = finite(input.nowMs, null);
    const dtMs = lastAtMs === null || now === null ? 0 : now - lastAtMs;
    if (now !== null) lastAtMs = now;

    // 1) 새 model 이 있으면 그대로 쓴다. 예산을 전부 되돌린다.
    if (Array.isArray(input.fresh) && input.fresh.length) {
      anchors = input.fresh;
      holdMs = 0;
      driftM = 0;
      state = AR_HOLD_STATE.LIVE;
      reason = "";
      return { state, anchors, holdMs, driftM, reason };
    }

    if (!anchors) {
      state = AR_HOLD_STATE.DROPPED;
      reason = "유지할 앵커 없음";
      return { state, anchors: null, holdMs, driftM, reason };
    }

    // 2) 유지 가능 조건. odometry 를 믿을 수 없으면 적분 자체가 무의미하다.
    if (input.canHold !== true) {
      reset();
      state = AR_HOLD_STATE.DROPPED;
      reason = "odometry 신뢰 불가 — 유지 포기";
      return { state, anchors: null, holdMs, driftM, reason };
    }

    // 3) 큰 scheduling gap은 seek가 아니다. 앵커는 유지하고 이 구간만
    // 적분하지 않는다. 실제 seek/domain/session 경계 reset은 runtime 계약이 맡는다.
    if (dtMs > limits.maxStepMs) {
      state = AR_HOLD_STATE.HELD;
      reason = `프레임 간격 ${Math.round(dtMs)}ms — 앵커 유지, 적분 생략`;
      return { state, anchors, holdMs, driftM, reason };
    }

    const dt = Math.max(0, dtMs) / 1000;
    holdMs += Math.max(0, dtMs);

    // 4) 예산 검사는 적분 **전에**. 넘은 상태로 한 프레임이라도 그리면 안 된다.
    if (holdMs > limits.maxHoldMs) {
      reset();
      state = AR_HOLD_STATE.DROPPED;
      reason = `유지 시간 초과 (${limits.maxHoldMs}ms)`;
      return { state, anchors: null, holdMs, driftM, reason };
    }

    const next = [];
    let worstDrift = driftM;
    for (const item of anchors) {
      const moved = advanceAnchor(item.anchor, input.odometry, dt);
      const grown = driftM + driftIncrement(item.anchor, input.odometry, dt);
      worstDrift = Math.max(worstDrift, grown);
      next.push({ ...item, anchor: moved });
    }
    driftM = worstDrift;

    if (driftM > limits.maxDriftM) {
      // reset() 이 값을 지우므로 진단 문구를 먼저 만든다.
      const exceeded = driftM;
      reset();
      state = AR_HOLD_STATE.DROPPED;
      reason = `누적 드리프트 ${exceeded.toFixed(2)}m > ${limits.maxDriftM}m`;
      return { state, anchors: null, holdMs, driftM: exceeded, reason };
    }

    anchors = next;
    state = AR_HOLD_STATE.HELD;
    reason = `유지 중 ${Math.round(holdMs)}ms / 드리프트 ${driftM.toFixed(2)}m`;
    return { state, anchors, holdMs, driftM, reason };
  }

  function status() {
    return Object.freeze({ state, holdMs: Math.round(holdMs), driftM: +driftM.toFixed(3), reason });
  }

  return Object.freeze({ update, reset, status, limits: Object.freeze(limits) });
}
