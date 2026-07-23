/* Phase 5B — 저주기 Navi fix 사이를 잇는 연속 월드 앵커 store.
 *
 * Navi는 약 2Hz지만 road video/model/odometry는 훨씬 자주 온다. 같은 Navi
 * snapshot을 매 presented frame의 새 위치로 취급하면 앵커가 0.5초 동안 멈췄다가
 * 다음 fix에서 점프한다. 이 store는 source identity가 바뀔 때만 절대 기준을
 * 갱신하고, 그 사이에는 route-local FLU odometry로 차량 상대 좌표를 전파한다.
 *
 * model path는 최초 승인 시 road height와 제한된 lateral observation으로만 쓴다.
 * 이후 marker는 불변 world anchor로 유지하며 매 frame path에 hard snap하지 않는다.
 * 노면 SURFACE만 현재 road geometry를 직접 따라간다.
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
import {
  createImmutableWorldAnchor,
  isUsableWorldPose,
  projectWorldAnchorToRoute,
} from "./world_anchor.js";
import { limitedRoadLateralCorrection } from "./road_observation.js";
import {
  AR_ANCHOR_HANDOFF_POLICY,
  handoffProgress,
  planWorldAnchorHandoff,
  publicHandoffState,
  sampleWorldAnchorHandoff,
} from "./anchor_handoff.js";

export const AR_ANCHOR_SOURCE_MODE = Object.freeze({
  GEO_PATH: "geo+path",
  GEO_ONLY: "geo-only",
  PATH_ONLY: "path-only",
  HELD: "held",
  SURFACE: "surface",
  WORLD: "world-anchor",
});

export const AR_CONTINUOUS_LIMITS = Object.freeze({
  ...AR_HOLD_LIMITS,
  /** Keep the last verified pose briefly, but never integrate an untrusted pose.
   *  Navi fix는 약 2Hz라, 빈 스냅샷이 끼어드는 구간을 넘기려면 그보다 길어야 한다. */
  maxUntrackedHoldMs: 1000,
  /** 좌표 없는 동일 안내가 이만큼 앞으로 점프하면 다음 event로 판정한다. */
  eventForwardJumpM: 35,
  /** An active Navi marker may temporarily lack a route/model placement. */
  unresolvedGraceMs: 3500,
  unresolvedGraceFixes: 8,
  /** A new geo anchor must not freeze an intermediate world-frame correction. */
  maxCreationPositionInnovationM: 15,
  maxCreationYawInnovationRad: 12 * Math.PI / 180,
});

export const AR_ANCHOR_LIFECYCLE_STATE = Object.freeze({
  ACTIVE: "active",
  ACTIVE_UNRESOLVED: "active-unresolved",
});

function finite(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function fixed(value, digits) {
  const number = finite(value);
  return number === null ? "" : number.toFixed(digits);
}

/**
 * Decide whether the current world frame is stable enough to freeze a new
 * geographic marker into it. Existing anchors are never invalidated by this
 * gate; only a genuinely new occurrence waits for the next Navi fix.
 */
export function worldAnchorCreationGate(
  worldPose,
  limits = AR_CONTINUOUS_LIMITS,
  context = {},
) {
  if (context.trackingState === "lost" || context.trackingState === "initializing") {
    return Object.freeze({ accepted: false, reason: "pose-propagation-settling" });
  }
  if (!isUsableWorldPose(worldPose)) {
    // Path-only compatibility remains available until a world pose exists.
    // The tracking state machine separately controls whether such an anchor
    // can be propagated; this gate only guards an active geo correction.
    return Object.freeze({ accepted: true, reason: "not-applicable" });
  }
  const correction = worldPose.geoCorrection;
  // Older fixtures and non-geographic pose sources have no correction state.
  if (!correction) return Object.freeze({ accepted: true, reason: "not-required" });
  if (correction.referenceReady !== true) {
    return Object.freeze({ accepted: false, reason: "geo-reference-unavailable" });
  }
  if (finite(correction.accepted, 0) < 1) {
    return Object.freeze({ accepted: false, reason: "geo-reference-settling" });
  }
  const positionInnovationM = finite(correction.lastPositionInnovationM);
  if (positionInnovationM !== null
      && positionInnovationM > limits.maxCreationPositionInnovationM) {
    return Object.freeze({
      accepted: false,
      reason: "geo-position-settling",
      positionInnovationM,
    });
  }
  const yawInnovationRad = finite(correction.lastYawInnovationRad);
  if (yawInnovationRad !== null
      && Math.abs(yawInnovationRad) > limits.maxCreationYawInnovationRad) {
    return Object.freeze({
      accepted: false,
      reason: "geo-yaw-settling",
      yawInnovationRad,
    });
  }
  return Object.freeze({
    accepted: true,
    reason: "stable",
    positionInnovationM,
    yawInnovationRad,
  });
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

/** sessionId만 경로 경계다. generation은 Navi 상태 변경 횟수이므로 새 fix 판별에만 쓴다. */
export function naviSourceIdentity(navi) {
  if (!navi) return null;
  const sessionId = String(navi.sessionId || "");
  return sessionId ? `session:${sessionId}` : "legacy";
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
  // A lane BAND is a surface, not an ego-attached carpet. Snap it at the
  // explicit TMap event distance; snapping every surface at x=0 created a
  // world anchor under the camera that immediately moved behind the vehicle.
  const distanceM = isSurface(record)
    ? finite(record.distanceM, finite(record.anchor.x))
    : finite(record.anchor.x);
  if (distanceM === null || distanceM < 0) return null;
  const path = pointOnPath(modelPosition, distanceM);
  if (!path) return null;

  let roadOffsetM = finite(record.roadOffsetM);
  let lateralCorrectionM = finite(record.lateralCorrectionM, 0);
  if (roadOffsetM === null || options.captureOffset === true) {
    const lateral = limitedRoadLateralCorrection(record.anchor.y, path.y, {
      // Only route-derived centers receive a small model innovation. A real
      // geo point and a turn branch must not be dragged onto the ego path.
      enabled: record.placedBy === "geo" && record.anchor.routeDerived === true,
    });
    lateralCorrectionM = lateral.correctionM;
    roadOffsetM = isSurface(record) ? 0 : lateral.y - finite(path.y, 0);
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
    lateralCorrectionM,
    sourceMode: sourceMode(record, true),
    anchor: Object.freeze({
      // Preserve route provenance/ordered-route diagnostics while replacing
      // only the observed road geometry.
      ...record.anchor,
      ...path,
      y: finite(path.y, 0) + roadOffsetM,
      z: finite(path.z, 0),
      lateralCorrectionM,
      ...roadFrameFields(frame),
    }),
    driftM: 0,
  };
}

function prepareCandidate(item, modelPosition, worldPose) {
  if (!item?.anchor) return null;
  const base = {
    ...item,
    anchorKey: anchorKey(item),
    lifecycleState: AR_ANCHOR_LIFECYCLE_STATE.ACTIVE,
    unresolvedSinceMs: null,
    unresolvedFixes: 0,
    roadOffsetM: null,
    sourceMode: item.placedBy === "geo"
      ? AR_ANCHOR_SOURCE_MODE.GEO_ONLY
      : AR_ANCHOR_SOURCE_MODE.PATH_ONLY,
    driftM: 0,
  };
  const prepared = pathSnap(base, modelPosition, { captureOffset: true }) || base;
  const worldAnchor = createImmutableWorldAnchor(prepared.anchor, worldPose);
  return worldAnchor
    ? { ...prepared, worldAnchor }
    : prepared;
}

function routeModelHandoff(previous, candidate, nowMs, worldPose, handoffPolicy) {
  if (
    previous?.placedBy !== "geo"
    || previous.sourceMode !== AR_ANCHOR_SOURCE_MODE.GEO_ONLY
    || candidate?.sourceMode !== AR_ANCHOR_SOURCE_MODE.GEO_PATH
    || !previous.worldAnchor
    || !candidate.worldAnchor
    || !isUsableWorldPose(worldPose)
  ) return null;

  const previousProjected = projectWorldAnchorToRoute(previous.worldAnchor, worldPose, previous.anchor);
  if (!previousProjected) return null;
  const preservesGeoPlacement = previous.anchor?.routeDerived !== true
    && candidate.anchor?.routeDerived !== true;
  const candidateFrame = roadFrameForAnchor(candidate.anchor);
  const targetFrame = roadFrameFromHeading(
    finite(previousProjected.headingRad, finite(previous.anchor?.headingRad, 0)),
    Math.atan2(
      candidateFrame.forward[2],
      Math.hypot(candidateFrame.forward[0], candidateFrame.forward[1]),
    ),
  );
  // A real TMap point already owns its world X/Y and branch yaw. The model
  // path may only contribute bounded road height and pitch here. Comparing the
  // raw path candidate's full XYZ first rejected valid height handoffs whenever
  // the ego-lane model and the turn branch were laterally separated.
  const targetAnchor = preservesGeoPlacement
    ? Object.freeze({
        ...candidate.anchor,
        x: finite(previousProjected.x, finite(previous.anchor?.x, 0)),
        y: finite(previousProjected.y, finite(previous.anchor?.y, 0)),
        z: finite(candidate.anchor.z, finite(previousProjected.z, 0)),
        ...roadFrameFields(targetFrame),
      })
    : candidate.anchor;
  const targetWorldAnchor = preservesGeoPlacement
    ? createImmutableWorldAnchor(targetAnchor, worldPose)
    : candidate.worldAnchor;
  if (!targetWorldAnchor) return null;
  const decision = planWorldAnchorHandoff(previous.worldAnchor, targetWorldAnchor, nowMs, {
    policy: handoffPolicy,
    lateralInnovationM: finite(candidate.anchor.y, 0) - finite(previousProjected?.y, 0),
    heightInnovationM: finite(candidate.anchor.z, 0) - finite(previousProjected?.z, 0),
  });
  if (!decision.accepted) {
    return Object.freeze({
      accepted: false,
      diagnostic: Object.freeze({
        state: "rejected",
        reason: decision.reason,
        positionInnovationM: decision.positionInnovationM,
        orientationInnovationRad: decision.orientationInnovationRad,
        lateralInnovationM: decision.lateralInnovationM,
        heightInnovationM: decision.heightInnovationM,
      }),
    });
  }
  return Object.freeze({
    accepted: true,
    transition: Object.freeze({
      ...decision.transition,
      targetAnchor,
      targetSourceMode: candidate.sourceMode,
    }),
  });
}

function reconcile(previous, candidate, limits, nowMs, worldPose, handoffPolicy) {
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

  // The marker identity and lifecycle stay unchanged. Only its observation
  // source advances from far route geometry to bounded near model geometry.
  if (!previous.handoff) {
    const handoff = routeModelHandoff(previous, candidate, nowMs, worldPose, handoffPolicy);
    if (handoff?.accepted) {
      return {
        ...candidate,
        placedBy: previous.placedBy,
        sourceMode: previous.sourceMode,
        anchor: Object.freeze({ ...previous.anchor }),
        worldAnchor: previous.worldAnchor,
        handoff: handoff.transition,
        handoffDiagnostic: null,
        driftM: previous.driftM,
      };
    }
    if (handoff) candidate = { ...candidate, handoffDiagnostic: handoff.diagnostic };
  }

  // Same event: keep the propagated world pose intact. New snapshots may
  // repaint distance/text/phase, but must not relocate or rotate the object.
  return {
    ...candidate,
    placedBy: previous.placedBy,
    roadOffsetM: previous.roadOffsetM,
    lateralCorrectionM: previous.lateralCorrectionM,
    sourceMode: previous.sourceMode,
    anchor: Object.freeze({ ...previous.anchor }),
    ...(previous.worldAnchor ? { worldAnchor: previous.worldAnchor } : {}),
    ...(previous.handoff ? { handoff: previous.handoff } : {}),
    handoffDiagnostic: previous.handoffDiagnostic || candidate.handoffDiagnostic || null,
    driftM: previous.driftM,
  };
}

function settleHandoffs(records, nowMs) {
  return records.map((record) => {
    const handoff = record.handoff;
    if (!handoff || handoffProgress(handoff, nowMs) < 1) return record;
    return {
      ...record,
      anchor: handoff.targetAnchor,
      worldAnchor: handoff.to,
      sourceMode: handoff.targetSourceMode,
      handoff: null,
      handoffDiagnostic: publicHandoffState(handoff, nowMs, "complete"),
    };
  });
}

function publicAnchors(records, nowMs, lastFixAtMs, worldPose, bypassWorldAnchor = false) {
  const ageMs = lastFixAtMs === null ? 0 : Math.max(0, nowMs - lastFixAtMs);
  return records.map((record) => {
    // Diagnostic bypass (?ar_bypass_world=1): skip the world-anchor projection
    // and render from the stored route-relative anchor, i.e. the pre-worldPose
    // path. If signs reappear with this on, the fault is in the world-pose
    // projection (steps 11/12), not in anchor creation.
    const renderedWorldAnchor = record.handoff
      ? sampleWorldAnchorHandoff(record.handoff, nowMs)
      : record.worldAnchor;
    const anchorTemplate = record.handoff?.targetAnchor || record.anchor;
    const projectedAnchor = !bypassWorldAnchor && renderedWorldAnchor
      ? projectWorldAnchorToRoute(renderedWorldAnchor, worldPose, anchorTemplate)
      : null;
    const anchor = projectedAnchor || record.anchor;
    const currentDistanceM = isSurface(record)
      ? finite(record.distanceM, 0)
      : Math.max(0, finite(anchor?.x, record.distanceM) || 0);
    const descriptor = record.source === "calibrationProbe"
      ? Object.freeze({
          ...record.descriptor,
          primary: formatDistance(currentDistanceM),
        })
      : record.descriptor;
    return Object.freeze({
      ...record,
      worldAnchor: renderedWorldAnchor,
      handoff: record.handoff
        ? publicHandoffState(record.handoff, nowMs)
        : record.handoffDiagnostic || null,
      anchor,
      descriptor,
      distanceM: currentDistanceM,
      anchorAgeMs: Math.round(ageMs),
    });
  });
}

export function createContinuousAnchorStore(options = {}) {
  const limits = Object.freeze({ ...AR_CONTINUOUS_LIMITS, ...(options.limits || {}) });
  const handoffPolicy = Object.freeze({
    ...AR_ANCHOR_HANDOFF_POLICY,
    ...(options.handoffPolicy || {}),
  });
  // Diagnostic bypass may be a live source (function) so it can be toggled from
  // the console mid-replay without a reload; a plain boolean is also accepted.
  const resolveBypass = typeof options.bypassWorldAnchor === "function"
    ? () => options.bypassWorldAnchor() === true
    : (() => { const on = options.bypassWorldAnchor === true; return () => on; })();
  let records = [];
  let fixId = null;
  let sourceId = null;
  let lastAtMs = null;
  let lastFixAtMs = null;
  let holdMs = 0;
  let driftM = 0;
  let state = AR_HOLD_STATE.DROPPED;
  let mode = null;
  let trackingLost = false;
  let lastWorldPose = null;
  let worldEpoch = null;
  let creationDeferredCount = 0;
  let creationGateReason = null;
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
    trackingLost = false;
    lastWorldPose = null;
    worldEpoch = null;
    creationDeferredCount = 0;
    creationGateReason = null;
    reason = nextReason;
  }

  function result(nowMs) {
    return {
      state,
      anchors: records.length ? publicAnchors(records, nowMs, lastFixAtMs, lastWorldPose, resolveBypass()) : null,
      holdMs,
      driftM,
      sourceMode: mode,
      fixId,
      sourceId,
      fixAgeMs: lastFixAtMs === null ? null : Math.max(0, nowMs - lastFixAtMs),
      reason,
    };
  }

  function acceptFix(
    candidates,
    activeMarkers,
    nextFixId,
    nextSourceId,
    nowMs,
    modelPosition,
    reconcilePrevious,
    creationContext,
  ) {
    const previous = reconcilePrevious
      ? new Map(records.map((record) => [record.anchorKey, record]))
      : new Map();
    const active = new Map((Array.isArray(activeMarkers) ? activeMarkers : [])
      .map((item) => [anchorKey(item), item])
      .filter(([key]) => Boolean(key)));
    const next = [];
    const resolved = new Set();
    creationDeferredCount = 0;
    creationGateReason = null;
    for (let index = 0; index < candidates.length; index += 1) {
      const item = candidates[index];
      const key = anchorKey(item);
      const previousRecord = previous.get(key);
      const previousX = finite(previousRecord?.anchor?.x, previousRecord?.distanceM);
      const candidateX = finite(item?.anchor?.x, item?.distanceM);
      const startsNewOccurrence = !previousRecord || (
        previousX !== null
        && candidateX !== null
        && candidateX - previousX > limits.eventForwardJumpM
      );
      if (
        startsNewOccurrence
        && item?.placedBy === "geo"
        && !isSurface(item)
      ) {
        const gate = worldAnchorCreationGate(lastWorldPose, limits, creationContext);
        if (!gate.accepted) {
          resolved.add(key);
          creationDeferredCount += 1;
          creationGateReason = gate.reason;
          continue;
        }
      }
      const candidate = prepareCandidate(item, modelPosition, lastWorldPose);
      if (!candidate) continue;
      resolved.add(candidate.anchorKey);
      next.push(reconcile(
        previousRecord,
        candidate,
        limits,
        nowMs,
        lastWorldPose,
        handoffPolicy,
      ));
    }
    for (const [key, marker] of active) {
      if (resolved.has(key)) continue;
      const record = previous.get(key);
      if (!record) continue;
      const unresolvedSinceMs = record.unresolvedSinceMs === null
        || record.unresolvedSinceMs === undefined
        ? nowMs
        : finite(record.unresolvedSinceMs, nowMs);
      const unresolvedFixes = Math.max(0, finite(record.unresolvedFixes, 0)) + 1;
      if (
        nowMs - unresolvedSinceMs > limits.unresolvedGraceMs
        || unresolvedFixes > limits.unresolvedGraceFixes
      ) continue;
      next.push({
        ...record,
        source: marker.source || record.source,
        markerId: marker.markerId || record.markerId,
        eventKey: marker.eventKey || record.eventKey,
        lifecycleSlot: marker.lifecycleSlot || record.lifecycleSlot,
        descriptor: marker.descriptor || record.descriptor,
        distanceM: finite(marker.distanceM, record.distanceM),
        confidence: marker.confidence || record.confidence,
        lifecycleState: AR_ANCHOR_LIFECYCLE_STATE.ACTIVE_UNRESOLVED,
        unresolvedSinceMs,
        unresolvedFixes,
      });
    }
    records = next;
    fixId = nextFixId;
    sourceId = nextSourceId;
    lastFixAtMs = nowMs;
    holdMs = 0;
    driftM = 0;
    state = records.length ? AR_HOLD_STATE.LIVE : AR_HOLD_STATE.DROPPED;
    trackingLost = false;
    mode = records.length && records.every((record) => Boolean(record.worldAnchor))
      ? AR_ANCHOR_SOURCE_MODE.WORLD
      : records.length && records.every((record) => record.sourceMode === AR_ANCHOR_SOURCE_MODE.SURFACE)
        ? AR_ANCHOR_SOURCE_MODE.SURFACE
        : records.some((record) => record.sourceMode === AR_ANCHOR_SOURCE_MODE.GEO_ONLY)
          ? AR_ANCHOR_SOURCE_MODE.GEO_ONLY
          : records.some((record) => record.sourceMode === AR_ANCHOR_SOURCE_MODE.GEO_PATH)
            ? AR_ANCHOR_SOURCE_MODE.GEO_PATH
            : AR_ANCHOR_SOURCE_MODE.PATH_ONLY;
    reason = records.length
      ? ""
      : creationDeferredCount
        ? `world anchor creation deferred: ${creationGateReason}`
        : "새 fix에 배치 가능한 앵커 없음";
  }

  function update(input = {}) {
    const nowMs = finite(input.nowMs, 0);
    let dtMs = lastAtMs === null ? 0 : nowMs - lastAtMs;
    if (dtMs < 0) {
      reset("시간 역행/replay seek");
      dtMs = 0;
    }
    lastAtMs = nowMs;

    if (isUsableWorldPose(input.worldPose)) {
      const nextWorldEpoch = String(input.worldPose.epoch || "uninitialized");
      if (worldEpoch !== null && nextWorldEpoch !== worldEpoch) {
        reset("world pose epoch changed - anchors reset");
        lastAtMs = nowMs;
      }
      lastWorldPose = input.worldPose;
      worldEpoch = nextWorldEpoch;
    }
    records = settleHandoffs(records, nowMs);

    const sourceValid = input.valid !== false;
    const candidates = Array.isArray(input.candidates) ? input.candidates : [];
    const activeMarkers = Array.isArray(input.activeMarkers) ? input.activeMarkers : [];
    const lifecycleAuthoritative = input.lifecycleAuthoritative === true;
    const trackingRecovered = input.trackingRecovered === true;
    const isProbe = input.probe === true
      || (candidates.length > 0 && candidates.every((item) => item?.source === "calibrationProbe"));
    if (sourceValid) {
      const nextFixId = isProbe ? "calibrationProbe" : naviFixIdentity(input.navi);
      const nextSourceId = isProbe ? "calibrationProbe" : naviSourceIdentity(input.navi);
      const sourceChanged = !isProbe && sourceId !== null && nextSourceId !== sourceId;
      // 새 fix에 배치 후보가 없어도 authoritative marker 목록은 소비한다.
      // 여전히 활성인 marker는 bounded unresolved로 유지하고, 목록에서 끝난
      // marker는 즉시 retire해 오래된 world anchor가 다시 나타나지 않게 한다.
      const newFix = candidates.length > 0
        ? (trackingRecovered || nextFixId !== fixId || !records.length)
        : lifecycleAuthoritative && nextFixId !== fixId;
      if (newFix) {
        acceptFix(
          candidates,
          activeMarkers,
          nextFixId,
          nextSourceId,
          nowMs,
          input.modelPosition,
          // Coasting/recovery changes pose quality, not Navi event identity.
          // Reusing the same-session record prevents a fresh 2 Hz Navi sample
          // from rebuilding one marker at a different lateral route estimate.
          // The calibration probe intentionally keeps its resnap behavior.
          !sourceChanged && (!isProbe || (!trackingLost && !trackingRecovered)),
          { trackingState: input.trackingState, canHold: input.canHold },
        );
        if (sourceChanged) reason = records.length ? "session 변경 — 이전 앵커 reset" : reason;
        else if (trackingRecovered) reason = records.length ? "tracking 복구 — fresh fix 재정합" : reason;
        return result(nowMs);
      }
    }

    if (!records.length) {
      if (!sourceValid) reason = input.reason || "source 무효 — 유지할 앵커 없음";
      return result(nowMs);
    }
    /* tracking lost가 곧 "앵커 폐기"는 아니다. route/geo 앵커는 새 Navi fix마다
     * 현재 차량 좌표에서 다시 계산되므로, source가 살아 있는 동안은 유지하고
     * 아래 hold 예산이 소진될 때만 떨군다.
     *
     * 리플레이에서 presented-frame 클럭이 안 잡히면 tracking이 lost↔initializing을
     * 왕복하는데, 그때마다 여기서 리셋되어 만들자마자 지워지고 있었다. */
    if (input.trackingState === "lost" && !sourceValid) {
      reset(input.reason || "tracking lost — anchor fade-out");
      return result(nowMs);
    }
    const worldLocked = records.every((record) => Boolean(record.worldAnchor));
    if (worldLocked && lastWorldPose) {
      holdMs = input.trackingState === "tracking" ? 0 : holdMs + Math.max(0, dtMs);
      driftM = Math.max(0, finite(input.trackingUncertaintyM, 0));
      state = input.trackingState === "tracking" ? AR_HOLD_STATE.LIVE : AR_HOLD_STATE.HELD;
      mode = AR_ANCHOR_SOURCE_MODE.WORLD;
      reason = state === AR_HOLD_STATE.LIVE
        ? "immutable world anchor"
        : `${input.trackingState || "tracking"} world pose hold ${Math.round(holdMs)}ms`;
      trackingLost = state !== AR_HOLD_STATE.LIVE;
      return result(nowMs);
    }
    const canPropagate = Boolean(input.odometry) && input.canHold === true;
    if (dtMs > limits.maxStepMs && canPropagate) {
      state = AR_HOLD_STATE.HELD;
      mode = AR_ANCHOR_SOURCE_MODE.HELD;
      reason = `프레임 간격 ${Math.round(dtMs)}ms — 앵커 유지, 적분 생략`;
      return result(nowMs);
    }
    // Once ego motion is untrusted, integrating a later sample would skip the
    // missing interval and make the marker follow the car. Freeze briefly and
    // require a fresh Navi fix before world-lock propagation can resume.
    if (!canPropagate || trackingLost) {
      trackingLost = true;
      holdMs += Math.max(0, dtMs);
      // The tracking state machine owns the dynamic uncertainty/age budget.
      // COASTING/DEGRADED freezes the immutable anchor when motion is absent;
      // it must not fall through to the old fixed 150ms deletion threshold.
      if (input.retainAnchor === true) {
        state = AR_HOLD_STATE.HELD;
        mode = AR_ANCHOR_SOURCE_MODE.HELD;
        reason = `${input.trackingState || "tracking"} freeze ${Math.round(holdMs)}ms`;
        return result(nowMs);
      }
      if (holdMs > limits.maxUntrackedHoldMs) {
        // tracking이 앵커를 놓은 이유가 있으면 그대로 남긴다(진단 패널이 읽는다).
        const why = input.trackingState === "lost" && input.reason ? String(input.reason) : "";
        reset(why
          ? `${why} — ${Math.round(holdMs)}ms 후 앵커 해제`
          : `odometry untracked ${Math.round(holdMs)}ms - anchor dropped`);
        return result(nowMs);
      }
      state = AR_HOLD_STATE.HELD;
      mode = AR_ANCHOR_SOURCE_MODE.HELD;
      reason = `odometry untracked hold ${Math.round(holdMs)}ms`;
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
      worldEpoch,
      worldLockedCount: records.filter((record) => Boolean(record.worldAnchor)).length,
      unresolvedCount: records.filter((record) => (
        record.lifecycleState === AR_ANCHOR_LIFECYCLE_STATE.ACTIVE_UNRESOLVED
      )).length,
      creationDeferredCount,
      creationGateReason,
      handoffCount: records.filter((record) => Boolean(record.handoff)).length,
      handoff: current.anchors?.find((record) => record.handoff)?.handoff || null,
      // 첫 앵커의 route-FLU 좌표. "그렸다는데 화면에 없다"를 가릴 때 쓴다.
      sample: current.anchors?.[0]?.anchor
        ? Object.freeze({
            x: +Number(current.anchors[0].anchor.x).toFixed(1),
            y: +Number(current.anchors[0].anchor.y).toFixed(1),
            z: +Number(current.anchors[0].anchor.z ?? 0).toFixed(1),
            markerId: current.anchors[0].markerId || current.anchors[0].eventKey || null,
            lifecycleSlot: current.anchors[0].lifecycleSlot || null,
            source: current.anchors[0].source || null,
            heightSource: current.anchors[0].anchor.heightSource || null,
            heightConfidence: +Number(current.anchors[0].anchor.heightConfidence ?? 0).toFixed(2),
            lateralCorrectionM: +Number(current.anchors[0].lateralCorrectionM ?? 0).toFixed(2),
          })
        : null,
    });
  }

  return Object.freeze({ update, reset, status, limits });
}
