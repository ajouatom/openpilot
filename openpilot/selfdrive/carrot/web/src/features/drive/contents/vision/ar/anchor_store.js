/* Phase 5B — 저주기 Navi fix 사이를 잇는 연속 월드 앵커 store.
 *
 * Navi는 약 2Hz지만 road video/model/odometry는 훨씬 자주 온다. 같은 Navi
 * snapshot을 매 presented frame의 새 위치로 취급하면 앵커가 0.5초 동안 멈췄다가
 * 다음 fix에서 점프한다. 이 store는 source identity가 바뀔 때만 절대 기준을
 * 갱신하고, 그 사이에는 cameraOdometry로 차량 상대 좌표를 전파한다.
 *
 * model path가 현재 전방거리를 덮으면 매 frame path에 다시 붙여 횡방향 drift를
 * 없앤다. path가 끊긴 구간만 기존 Phase 4 시간/드리프트 예산으로 제한한다.
 */

import {
  AR_HOLD_LIMITS,
  AR_HOLD_STATE,
  advanceAnchor,
  driftIncrement,
} from "./anchor.js";
import { pointOnPath } from "./projection.js";
import { formatDistance } from "./signboard.js";
import { markerIdentity } from "./marker_identity.js";
import {
  roadFrameFields,
  roadFrameForAnchor,
  roadFrameFromHeading,
} from "./road_frame.js";

export const AR_ANCHOR_SOURCE_MODE = Object.freeze({
  GEO_PATH: "geo+path",
  GEO_ONLY: "geo-only",
  PATH_ONLY: "path-only",
  HELD: "held",
  SURFACE: "surface",
});

export const AR_CONTINUOUS_LIMITS = Object.freeze({
  ...AR_HOLD_LIMITS,
  /** 좌표 없는 동일 안내가 이만큼 앞으로 점프하면 다음 event로 판정한다. */
  eventForwardJumpM: 35,
});

function finite(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function fixed(value, digits) {
  const number = finite(value);
  return number === null ? "" : number.toFixed(digits);
}

/** 동일 decoded object가 아니라 Navi가 실제로 publish된 시점을 식별한다. */
export function naviFixIdentity(navi) {
  if (!navi) return null;
  const sessionId = String(navi.sessionId || "");
  const generationNumber = finite(navi.generation, 0);
  const generation = String(generationNumber);
  const publish = finite(navi.publishMonoTimeNanos, 0);
  if (sessionId || generationNumber > 0 || publish > 0) return `${sessionId}|${generation}|${publish}`;

  // 오래된 replay/fixture에는 source identity가 없을 수 있다. 그때만 실제 위치와
  // 안내거리로 만든 보수적인 signature를 동등 source identity로 사용한다.
  const vehicle = navi.vehicle || {};
  const current = navi.guidanceCurrent || {};
  const next = navi.guidanceNext || {};
  const speed = navi.speed || {};
  return [
    fixed(vehicle.latitude, 7), fixed(vehicle.longitude, 7), fixed(vehicle.headingDeg, 2),
    current.present === false ? 0 : 1, finite(current.turnType, 0), finite(current.distanceM, 0),
    next.present === false ? 0 : 1, finite(next.turnType, 0), finite(next.distanceM, 0),
    finite(speed.sdiType, -1), finite(speed.sdiDistanceM, 0),
  ].join("|");
}

/** session/generation은 publish fix보다 강한 경계다. 바뀌면 이전 event를 이어 붙이지 않는다. */
export function naviSourceIdentity(navi) {
  if (!navi) return null;
  const sessionId = String(navi.sessionId || "");
  const generation = finite(navi.generation, 0);
  return sessionId || generation > 0 ? `${sessionId}|${generation}` : "legacy";
}

function anchorKey(item) {
  if (item?.markerId) return String(item.markerId);
  if (item?.eventKey) return String(item.eventKey);
  return markerIdentity(item);
}

function isSurface(record) {
  return record?.descriptor?.surface === true;
}

function sourceMode(record, snapped) {
  if (isSurface(record)) return AR_ANCHOR_SOURCE_MODE.SURFACE;
  if (!snapped) return AR_ANCHOR_SOURCE_MODE.HELD;
  return record.placedBy === "geo" ? AR_ANCHOR_SOURCE_MODE.GEO_PATH : AR_ANCHOR_SOURCE_MODE.PATH_ONLY;
}

function pathSnap(record, modelPosition, options = {}) {
  if (!record?.anchor) return null;
  const distanceM = isSurface(record) ? 0 : finite(record.anchor.x);
  if (distanceM === null || distanceM < 0) return null;
  const path = pointOnPath(modelPosition, distanceM);
  if (!path) return null;

  let roadOffsetM = finite(record.roadOffsetM);
  if (roadOffsetM === null || options.captureOffset === true) {
    roadOffsetM = isSurface(record) ? 0 : finite(record.anchor.y, 0) - finite(path.y, 0);
  }
  const pathFrame = roadFrameForAnchor(path);
  const frame = record.placedBy === "geo"
    ? roadFrameFromHeading(
      finite(record.anchor.headingRad, path.headingRad),
      Math.atan2(
        pathFrame.forward[2],
        Math.hypot(pathFrame.forward[0], pathFrame.forward[1]),
      ),
    )
    : pathFrame;
  return {
    ...record,
    roadOffsetM,
    sourceMode: sourceMode(record, true),
    anchor: Object.freeze({
      ...path,
      y: finite(path.y, 0) + roadOffsetM,
      z: finite(path.z, 0),
      ...roadFrameFields(frame),
    }),
    driftM: 0,
  };
}

function prepareCandidate(item, modelPosition) {
  if (!item?.anchor) return null;
  const base = {
    ...item,
    anchorKey: anchorKey(item),
    roadOffsetM: null,
    sourceMode: item.placedBy === "geo"
      ? AR_ANCHOR_SOURCE_MODE.GEO_ONLY
      : AR_ANCHOR_SOURCE_MODE.PATH_ONLY,
    driftM: 0,
  };
  return pathSnap(base, modelPosition, { captureOffset: true }) || base;
}

function reconcile(previous, candidate, limits) {
  if (!previous?.anchor || !candidate?.anchor) return candidate;

  const previousX = finite(previous.anchor.x, previous.distanceM);
  const candidateX = finite(candidate.anchor.x, candidate.distanceM);
  const forwardJumpM = previousX === null || candidateX === null
    ? 0
    : candidateX - previousX;

  // Navi has no durable per-item sequence. A large forward reset in the same
  // semantic slot is the safe indication that a coordinate-less event was
  // replaced by the next occurrence. Start that occurrence at its fresh pose.
  if (forwardJumpM > limits.eventForwardJumpM) return candidate;

  // Same event: keep the propagated world pose intact. New snapshots may
  // repaint distance/text/phase, but must not relocate or rotate the object.
  return {
    ...candidate,
    placedBy: previous.placedBy,
    roadOffsetM: previous.roadOffsetM,
    sourceMode: previous.sourceMode,
    anchor: Object.freeze({ ...previous.anchor }),
    driftM: previous.driftM,
  };
}

function publicAnchors(records, nowMs, lastFixAtMs) {
  const ageMs = lastFixAtMs === null ? 0 : Math.max(0, nowMs - lastFixAtMs);
  return records.map((record) => {
    const currentDistanceM = isSurface(record)
      ? finite(record.distanceM, 0)
      : Math.max(0, finite(record.anchor?.x, record.distanceM) || 0);
    const descriptor = record.source === "calibrationProbe"
      ? Object.freeze({
          ...record.descriptor,
          primary: formatDistance(currentDistanceM),
        })
      : record.descriptor;
    return Object.freeze({
      ...record,
      descriptor,
      distanceM: currentDistanceM,
      anchorAgeMs: Math.round(ageMs),
    });
  });
}

export function createContinuousAnchorStore(options = {}) {
  const limits = Object.freeze({ ...AR_CONTINUOUS_LIMITS, ...(options.limits || {}) });
  let records = [];
  let fixId = null;
  let sourceId = null;
  let lastAtMs = null;
  let lastFixAtMs = null;
  let holdMs = 0;
  let driftM = 0;
  let state = AR_HOLD_STATE.DROPPED;
  let mode = null;
  let reason = "아직 유효한 앵커 없음";

  function reset(nextReason = "초기화됨") {
    records = [];
    fixId = null;
    sourceId = null;
    lastAtMs = null;
    lastFixAtMs = null;
    holdMs = 0;
    driftM = 0;
    state = AR_HOLD_STATE.DROPPED;
    mode = null;
    reason = nextReason;
  }

  function result(nowMs) {
    return {
      state,
      anchors: records.length ? publicAnchors(records, nowMs, lastFixAtMs) : null,
      holdMs,
      driftM,
      sourceMode: mode,
      fixId,
      sourceId,
      fixAgeMs: lastFixAtMs === null ? null : Math.max(0, nowMs - lastFixAtMs),
      reason,
    };
  }

  function acceptFix(candidates, nextFixId, nextSourceId, nowMs, modelPosition, reconcilePrevious) {
    const previous = reconcilePrevious
      ? new Map(records.map((record) => [record.anchorKey, record]))
      : new Map();
    const next = [];
    for (let index = 0; index < candidates.length; index += 1) {
      const candidate = prepareCandidate(candidates[index], modelPosition);
      if (!candidate) continue;
      next.push(reconcile(previous.get(candidate.anchorKey), candidate, limits));
    }
    records = next;
    fixId = nextFixId;
    sourceId = nextSourceId;
    lastFixAtMs = nowMs;
    holdMs = 0;
    driftM = 0;
    state = records.length ? AR_HOLD_STATE.LIVE : AR_HOLD_STATE.DROPPED;
    mode = records.length && records.every((record) => record.sourceMode === AR_ANCHOR_SOURCE_MODE.SURFACE)
      ? AR_ANCHOR_SOURCE_MODE.SURFACE
      : records.some((record) => record.sourceMode === AR_ANCHOR_SOURCE_MODE.GEO_ONLY)
        ? AR_ANCHOR_SOURCE_MODE.GEO_ONLY
        : records.some((record) => record.sourceMode === AR_ANCHOR_SOURCE_MODE.GEO_PATH)
          ? AR_ANCHOR_SOURCE_MODE.GEO_PATH
          : AR_ANCHOR_SOURCE_MODE.PATH_ONLY;
    reason = records.length ? "" : "새 fix에 배치 가능한 앵커 없음";
  }

  function update(input = {}) {
    const nowMs = finite(input.nowMs, 0);
    let dtMs = lastAtMs === null ? 0 : nowMs - lastAtMs;
    if (dtMs < 0) {
      reset("시간 역행/replay seek");
      dtMs = 0;
    }
    lastAtMs = nowMs;

    const sourceValid = input.valid !== false;
    const candidates = Array.isArray(input.candidates) ? input.candidates : [];
    const isProbe = input.probe === true
      || (candidates.length > 0 && candidates.every((item) => item?.source === "calibrationProbe"));
    if (sourceValid) {
      const nextFixId = isProbe ? "calibrationProbe" : naviFixIdentity(input.navi);
      const nextSourceId = isProbe ? "calibrationProbe" : naviSourceIdentity(input.navi);
      const sourceChanged = !isProbe && sourceId !== null && nextSourceId !== sourceId;
      // 새 fix identity만 먼저 도착하고 정확 투영 후보가 아직 없을 수 있다.
      // 후보 없는 fix로 기존 앵커를 비우지 말고, 배치 가능한 프레임까지 갱신을 미룬다.
      const newFix = candidates.length > 0
        && (nextFixId !== fixId || !records.length);
      if (newFix) {
        acceptFix(candidates, nextFixId, nextSourceId, nowMs, input.modelPosition, !sourceChanged);
        if (sourceChanged) reason = records.length ? "session/generation 변경 — 이전 앵커 reset" : reason;
        return result(nowMs);
      }
    }

    if (!records.length) {
      if (!sourceValid) reason = input.reason || "source 무효 — 유지할 앵커 없음";
      return result(nowMs);
    }
    if (dtMs > limits.maxStepMs) {
      reset(`프레임 간격 ${Math.round(dtMs)}ms — 적분 불가`);
      return result(nowMs);
    }
    // Fresh model/calibration이 있는 프레임은 canHold가 보수적으로 false여도
    // 현재 odometry 속도와 경로 관측을 함께 쓸 수 있다. frame gap 하나 때문에
    // store를 비우고 다시 40m에 생성하면 그 순간이 화면에서 큰 점프로 보인다.
    const canPropagate = Boolean(input.odometry)
      && (input.canHold === true || input.precise === true);
    if (!canPropagate) {
      reset("odometry 신뢰 불가 — 연속 전파 포기");
      return result(nowMs);
    }

    const dt = Math.max(0, dtMs) / 1000;
    // precise가 아닌 프레임은 model path가 남아 있더라도 그 경로를 새 관측으로
    // 취급하지 않는다. 마지막 월드 앵커를 odometry로만 전파하고 HELD로 명시해
    // renderer가 한 프레임짜리 stale 판정에 화면을 지우지 않게 한다.
    let anyHeld = input.precise !== true || !sourceValid;
    let worstDrift = 0;
    const nextRecords = [];
    for (const record of records) {
      if (isSurface(record)) {
        nextRecords.push(input.precise === true
          ? (pathSnap(record, input.modelPosition) || record)
          : record);
        continue;
      }

      const moved = advanceAnchor(record.anchor, input.odometry, dt);
      const advanced = {
        ...record,
        anchor: moved,
      };
      if (input.precise === true && sourceValid) {
        // Once placed, an upright sign is an immutable world object. Fresh
        // model paths validate tracking but never pull or rotate the object.
        nextRecords.push(advanced);
        continue;
      }

      anyHeld = true;
      const recordDrift = finite(record.driftM, 0)
        + driftIncrement(record.anchor, input.odometry, dt);
      worstDrift = Math.max(worstDrift, recordDrift);
      nextRecords.push({
        ...advanced,
        sourceMode: AR_ANCHOR_SOURCE_MODE.HELD,
        driftM: recordDrift,
      });
    }
    records = nextRecords;

    if (anyHeld) {
      holdMs += Math.max(0, dtMs);
      driftM = worstDrift;
      if (holdMs > limits.maxHoldMs || driftM > limits.maxDriftM) {
        const exceeded = holdMs > limits.maxHoldMs
          ? `유지 시간 초과 (${limits.maxHoldMs}ms)`
          : `누적 드리프트 ${driftM.toFixed(2)}m > ${limits.maxDriftM}m`;
        reset(exceeded);
        return result(nowMs);
      }
      state = AR_HOLD_STATE.HELD;
      mode = AR_ANCHOR_SOURCE_MODE.HELD;
      reason = `유지 중 ${Math.round(holdMs)}ms / 드리프트 ${driftM.toFixed(2)}m`;
    } else {
      holdMs = 0;
      driftM = 0;
      state = AR_HOLD_STATE.LIVE;
      mode = records.some((record) => record.sourceMode === AR_ANCHOR_SOURCE_MODE.GEO_PATH)
        ? AR_ANCHOR_SOURCE_MODE.GEO_PATH
        : records.some((record) => record.sourceMode === AR_ANCHOR_SOURCE_MODE.GEO_ONLY)
          ? AR_ANCHOR_SOURCE_MODE.GEO_ONLY
          : records.every((record) => record.sourceMode === AR_ANCHOR_SOURCE_MODE.SURFACE)
            ? AR_ANCHOR_SOURCE_MODE.SURFACE
            : AR_ANCHOR_SOURCE_MODE.PATH_ONLY;
      reason = "odometry world lock";
    }
    return result(nowMs);
  }

  function status(nowMs = lastAtMs ?? 0) {
    const current = result(nowMs);
    return Object.freeze({
      state: current.state,
      sourceMode: current.sourceMode,
      anchorCount: records.length,
      fixId: current.fixId,
      sourceId: current.sourceId,
      fixAgeMs: current.fixAgeMs === null ? null : Math.round(current.fixAgeMs),
      holdMs: Math.round(current.holdMs),
      driftM: +current.driftM.toFixed(3),
      reason: current.reason,
    });
  }

  return Object.freeze({ update, reset, status, limits });
}
