/* 접속 브라우저 Worker의 AR 처리시간 정책.
 *
 * 단일 느린 frame에는 반응하지 않는다. 일정 window에서 frame budget을 넘긴 비율이
 * 충분히 높을 때만 30→15fps로 강등한다. 복구는 target 30fps budget에 대한 충분한
 * headroom이 긴 window 동안 확인될 때만 허용해 30/15 왕복 떨림을 막는다. 정상
 * WebGL context는 성능 부족만으로 종료하지 않고 최신 프레임 우선 백프레셔가
 * 흡수한다. 실제 renderer 오류와 context loss만 fail-closed한다.
 */

import { AR_RENDER } from "./tokens.js";

function positiveNumber(value, fallback) {
  const n = Number(value);
  return Number.isFinite(n) && n > 0 ? n : fallback;
}

function positiveInteger(value, fallback) {
  return Math.max(1, Math.round(positiveNumber(value, fallback)));
}

export function createRenderPerformancePolicy(options = {}) {
  const defaults = AR_RENDER.performance;
  const targetFps = positiveNumber(options.targetFps, AR_RENDER.targetFps);
  const degradedFps = Math.min(
    targetFps,
    positiveNumber(options.degradedFps, AR_RENDER.degradedFps),
  );
  const slowBudgetRatio = positiveNumber(options.slowBudgetRatio, defaults.slowBudgetRatio);
  const degradeWindowFrames = positiveInteger(
    options.degradeWindowFrames,
    defaults.degradeWindowFrames,
  );
  const degradeSlowFrames = Math.min(
    degradeWindowFrames,
    positiveInteger(options.degradeSlowFrames, defaults.degradeSlowFrames),
  );
  const recoverBudgetRatio = positiveNumber(options.recoverBudgetRatio, defaults.recoverBudgetRatio);
  const recoverWindowFrames = positiveInteger(
    options.recoverWindowFrames,
    defaults.recoverWindowFrames,
  );
  const recoverFastFrames = Math.min(
    recoverWindowFrames,
    positiveInteger(options.recoverFastFrames, defaults.recoverFastFrames),
  );
  let level = "target";
  let fps = targetFps;
  let samples = [];
  let recoverySamples = [];
  let totalFrames = 0;
  let transitions = 0;
  let lastWorkMs = 0;
  let averageWorkMs = 0;

  function snapshot() {
    return Object.freeze({
      level,
      fps,
      targetFps,
      degradedFps,
      frameBudgetMs: 1000 / fps,
      lastWorkMs,
      averageWorkMs,
      sampledFrames: samples.length,
      slowFrames: samples.reduce((count, slow) => count + Number(slow), 0),
      recoverySampledFrames: recoverySamples.length,
      fastFrames: recoverySamples.reduce((count, fast) => count + Number(fast), 0),
      totalFrames,
      transitions,
      failed: false,
    });
  }

  function observe(workMs) {
    const duration = Number(workMs);
    if (!Number.isFinite(duration) || duration < 0) return snapshot();

    totalFrames += 1;
    lastWorkMs = duration;
    averageWorkMs = totalFrames === 1
      ? duration
      : averageWorkMs + (duration - averageWorkMs) * 0.2;

    const slow = duration > (1000 / fps) * slowBudgetRatio;
    samples.push(slow);
    if (samples.length > degradeWindowFrames) samples.shift();

    const fastForTarget = duration <= (1000 / targetFps) * recoverBudgetRatio;
    recoverySamples.push(fastForTarget);
    if (recoverySamples.length > recoverWindowFrames) recoverySamples.shift();

    const slowFrames = samples.reduce((count, value) => count + Number(value), 0);
    if (level === "target" && samples.length === degradeWindowFrames
        && slowFrames >= degradeSlowFrames && degradedFps < targetFps) {
      level = "degraded";
      fps = degradedFps;
      samples = [];
      recoverySamples = [];
      transitions += 1;
    } else if (level === "degraded" && recoverySamples.length === recoverWindowFrames) {
      const fastFrames = recoverySamples.reduce((count, value) => count + Number(value), 0);
      if (fastFrames >= recoverFastFrames) {
        level = "target";
        fps = targetFps;
        samples = [];
        recoverySamples = [];
        transitions += 1;
      }
    }

    return snapshot();
  }

  return Object.freeze({ observe, status: snapshot });
}
