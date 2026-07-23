/* AR tracking lifecycle.
 *
 * A stale bit is not a lifecycle. Camera/model/odometry samples arrive at
 * different cadences, so a single missing frame must reduce confidence without
 * destroying the world anchor. This state machine owns that distinction and
 * keeps its time/uncertainty budgets explicit for trace and replay tests.
 */

export const AR_TRACKING_STATE = Object.freeze({
  INITIALIZING: "initializing",
  TRACKING: "tracking",
  COASTING: "coasting",
  DEGRADED: "degraded",
  LOST: "lost",
});

export const AR_TRACKING_LIMITS = Object.freeze({
  // 일단 보이는 게 우선: coasting/degraded 유지시간과 횡방향 예산을 넉넉히 늘려
  // 추적이 잠깐 흔들려도 표지를 떨구지 않는다.
  minHealthySamples: 1,
  expectedFrameHoldMs: 90,
  maxCoastingMs: 1200,
  maxDegradedMs: 4000,
  initializingTimeoutMs: 2000,
  minLateralBudgetM: 2.5,
  maxLateralBudgetM: 12.0,
  lateralBudgetDistanceRatio: 0.015,
  coastingBudgetRatio: 0.55,
  noOdometryTranslationStdMps: 0.12,
  noOdometrySpeedStdRatio: 0.01,
  noOdometryRotationStdRadps: 0.002,
});

function finite(value, fallback = null) {
  if (value === null || value === undefined || value === "") return fallback;
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function vectorNorm(values) {
  if (!Array.isArray(values) || values.length === 0) return null;
  const finiteValues = values.map((value) => finite(value, 0));
  return Math.hypot(...finiteValues);
}

function sampleIdentity(input) {
  if (input.sampleId !== null && input.sampleId !== undefined && input.sampleId !== "") {
    return String(input.sampleId);
  }
  const targetNs = finite(input.presentedTimestampNs);
  return targetNs !== null && targetNs > 0 ? `time:${targetNs}` : null;
}

function stateAlpha(state, predictionAgeMs, limits) {
  if (state === AR_TRACKING_STATE.TRACKING) return 1;
  if (state === AR_TRACKING_STATE.COASTING) {
    const progress = clamp(
      (predictionAgeMs - limits.expectedFrameHoldMs)
        / Math.max(1, limits.maxCoastingMs - limits.expectedFrameHoldMs),
      0,
      1,
    );
    return 1 - progress * 0.22;
  }
  if (state === AR_TRACKING_STATE.DEGRADED) {
    const progress = clamp(
      (predictionAgeMs - limits.maxCoastingMs)
        / Math.max(1, limits.maxDegradedMs - limits.maxCoastingMs),
      0,
      1,
    );
    return 0.72 - progress * 0.30;
  }
  return 0;
}

function publicResult(values) {
  return Object.freeze({
    ...values,
    uncertainty: Object.freeze({ ...values.uncertainty }),
    reasons: Object.freeze([...values.reasons]),
  });
}

export function createArTrackingState(options = {}) {
  const limits = Object.freeze({ ...AR_TRACKING_LIMITS, ...(options.limits || {}) });
  let state = AR_TRACKING_STATE.INITIALIZING;
  let startedAtMs = null;
  let lastUpdateAtMs = null;
  let lastHealthyAtMs = null;
  let lastSampleId = null;
  let healthySamples = 0;
  let translationSigmaM = 0;
  let rotationSigmaRad = 0;
  let lastResult = null;

  function reset(reason = "reset") {
    state = AR_TRACKING_STATE.INITIALIZING;
    startedAtMs = null;
    lastUpdateAtMs = null;
    lastHealthyAtMs = null;
    lastSampleId = null;
    healthySamples = 0;
    translationSigmaM = 0;
    rotationSigmaRad = 0;
    lastResult = publicResult({
      state,
      previousState: state,
      transition: null,
      recovered: false,
      canCreateAnchor: false,
      canPropagateAnchor: false,
      retainAnchor: false,
      alpha: 0,
      predictionAgeMs: null,
      uncertainty: {
        translationSigmaM: 0,
        rotationSigmaRad: 0,
        lateralM: 0,
        budgetM: limits.minLateralBudgetM,
        utilization: 0,
      },
      reasons: [String(reason || "reset")],
    });
    return lastResult;
  }

  function update(input = {}) {
    const nowMs = Math.max(0, finite(input.nowMs, 0));
    if (startedAtMs === null) startedAtMs = nowMs;
    let dtMs = lastUpdateAtMs === null ? 0 : nowMs - lastUpdateAtMs;
    if (dtMs < 0) dtMs = 0;
    dtMs = Math.min(dtMs, limits.maxDegradedMs);
    lastUpdateAtMs = nowMs;

    const previousState = state;
    const presented = finite(input.presentedTimestampNs, null);
    const spatialClockReady = input.spatialClockReady === true && presented !== null && presented > 0;
    const odometry = input.odometry || null;
    const precise = input.sync?.canDrawPrecise === true;
    const motionUsable = input.sync?.canHoldAnchor === true && Boolean(odometry);
    const healthy = spatialClockReady && precise && motionUsable;
    const nextSampleId = sampleIdentity(input);
    const distinctSample = nextSampleId !== null && nextSampleId !== lastSampleId;
    if (distinctSample) lastSampleId = nextSampleId;

    const reasons = [];
    if (input.discontinuity === true) {
      state = AR_TRACKING_STATE.LOST;
      healthySamples = 0;
      startedAtMs = nowMs;
      lastHealthyAtMs = null;
      lastSampleId = null;
      translationSigmaM = 0;
      rotationSigmaRad = 0;
      reasons.push(String(input.discontinuityReason || "timeline discontinuity"));
    } else if (healthy && distinctSample) {
      healthySamples += 1;
      lastHealthyAtMs = nowMs;
      translationSigmaM = 0;
      rotationSigmaRad = 0;
      state = healthySamples >= limits.minHealthySamples
        ? AR_TRACKING_STATE.TRACKING
        : AR_TRACKING_STATE.INITIALIZING;
    } else {
      if (healthy && !distinctSample) reasons.push("presented sample not advanced");
      if (!spatialClockReady) reasons.push("presented frame clock unmapped");
      if (!precise) reasons.push(...(input.sync?.reasons || ["precise sync unavailable"]));
      if (!motionUsable) reasons.push("odometry/pose propagation unavailable");

      const dtSeconds = dtMs / 1000;
      const transStd = vectorNorm(odometry?.transStd);
      const rotStd = vectorNorm(odometry?.rotStd);
      const speedMps = Math.abs(finite(input.egoSpeedMps, 0));
      translationSigmaM += (transStd === null
        ? limits.noOdometryTranslationStdMps + speedMps * limits.noOdometrySpeedStdRatio
        : transStd) * dtSeconds;
      rotationSigmaRad += (rotStd === null
        ? limits.noOdometryRotationStdRadps
        : rotStd) * dtSeconds;

      const predictionAgeMs = lastHealthyAtMs === null
        ? Math.max(0, nowMs - startedAtMs)
        : Math.max(0, nowMs - lastHealthyAtMs);
      const distanceM = clamp(Math.abs(finite(input.anchorDistanceM, 40)), 0, 300);
      const budgetM = clamp(
        limits.minLateralBudgetM + distanceM * limits.lateralBudgetDistanceRatio,
        limits.minLateralBudgetM,
        limits.maxLateralBudgetM,
      );
      const lateralM = translationSigmaM + distanceM * rotationSigmaRad;

      if (lastHealthyAtMs === null) {
        state = predictionAgeMs <= limits.initializingTimeoutMs
          ? AR_TRACKING_STATE.INITIALIZING
          : AR_TRACKING_STATE.LOST;
      } else if (
        previousState === AR_TRACKING_STATE.TRACKING
        && predictionAgeMs <= limits.expectedFrameHoldMs
      ) {
        // A 20Hz camera frame is legitimately presented over several 30Hz UI
        // ticks. Do not oscillate into COASTING between normal video frames.
        state = AR_TRACKING_STATE.TRACKING;
      } else if (
        predictionAgeMs <= limits.maxCoastingMs
        && lateralM <= budgetM * limits.coastingBudgetRatio
      ) {
        state = AR_TRACKING_STATE.COASTING;
      } else if (predictionAgeMs <= limits.maxDegradedMs && lateralM <= budgetM) {
        state = AR_TRACKING_STATE.DEGRADED;
      } else {
        state = AR_TRACKING_STATE.LOST;
      }
    }

    const predictionAgeMs = lastHealthyAtMs === null
      ? Math.max(0, nowMs - startedAtMs)
      : Math.max(0, nowMs - lastHealthyAtMs);
    const distanceM = clamp(Math.abs(finite(input.anchorDistanceM, 40)), 0, 300);
    const budgetM = clamp(
      limits.minLateralBudgetM + distanceM * limits.lateralBudgetDistanceRatio,
      limits.minLateralBudgetM,
      limits.maxLateralBudgetM,
    );
    const lateralM = translationSigmaM + distanceM * rotationSigmaRad;
    if (state === AR_TRACKING_STATE.DEGRADED) {
      reasons.push(
        `tracking degraded age=${Math.round(predictionAgeMs)}ms uncertainty=${lateralM.toFixed(2)}/${budgetM.toFixed(2)}m`,
      );
    } else if (state === AR_TRACKING_STATE.LOST && input.discontinuity !== true) {
      reasons.push(
        `tracking budget exceeded age=${Math.round(predictionAgeMs)}ms uncertainty=${lateralM.toFixed(2)}/${budgetM.toFixed(2)}m`,
      );
    }
    const recovered = state === AR_TRACKING_STATE.TRACKING
      && previousState !== AR_TRACKING_STATE.TRACKING
      && previousState !== AR_TRACKING_STATE.INITIALIZING;
    const retainAnchor = state === AR_TRACKING_STATE.TRACKING
      || state === AR_TRACKING_STATE.COASTING
      || state === AR_TRACKING_STATE.DEGRADED;
    lastResult = publicResult({
      state,
      previousState,
      transition: state === previousState ? null : `${previousState}->${state}`,
      recovered,
      /* 앵커 "생성"은 presented-frame 공간 클럭 락을 요구하지 않는다.
       * route/geo 앵커는 carrotNavi 좌표와 model path만으로 만들어진다. 공간
       * 클럭과 odometry는 만들어진 앵커를 world에 고정하고 프레임 사이에
       * 유지(전파)할 때 필요한 것이므로 canPropagateAnchor가 계속 책임진다.
       *
       * 리플레이에서 presented-frame 매핑이 안 되면 tracking이 영원히 LOST에
       * 머물러 표지가 아예 안 나오던 원인이 여기였다. 일단 보이는 게 우선이므로
       * 생성은 데이터 신선도 gate(sync.canDrawPrecise)만 본다. */
      canCreateAnchor: precise,
      canPropagateAnchor: retainAnchor && motionUsable,
      retainAnchor,
      /* alpha는 "오래된 앵커를 서서히 지우는" 값이다. 그런데 precise 프레임의
       * 앵커는 지금 입력으로 새로 계산된 것이라 지울 이유가 없다.
       *
       * 리플레이에서 presented-frame 클럭이 안 잡히면 tracking이 LOST에 머물고,
       * LOST의 alpha는 0이라 표지가 "그려지긴 하는데 완전투명"으로 나왔다.
       * (drawn>0, 스킵 0인데 화면에는 아무것도 없던 원인) */
      alpha: precise ? 1 : +stateAlpha(state, predictionAgeMs, limits).toFixed(3),
      predictionAgeMs: Math.round(predictionAgeMs),
      uncertainty: {
        translationSigmaM: +translationSigmaM.toFixed(4),
        rotationSigmaRad: +rotationSigmaRad.toFixed(6),
        lateralM: +lateralM.toFixed(4),
        budgetM: +budgetM.toFixed(3),
        utilization: +(budgetM > 0 ? lateralM / budgetM : 1).toFixed(3),
      },
      reasons,
    });
    return lastResult;
  }

  function status() {
    return lastResult;
  }

  reset("not started");
  return Object.freeze({ update, reset, status, limits });
}
