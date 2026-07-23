export const CAMERA_ODOMETRY_POSE_DELAY_MS = 100;

export const AR_PRESENTED_CLOCK_CONFIDENCE = Object.freeze({
  EXACT_FRAME: "exact-frame",
  ESTIMATED_FRAME: "estimated-frame",
  MEDIA_FRAME: "media-frame",
  UNMAPPED: "unmapped",
});

const NS_PER_MS = 1e6;
const NS_PER_SECOND = 1e9;

/** media↔cereal offset을 다시 잡는 불일치 한계.
 *  정상 지터(cereal 프레임 간격 약 50ms + pose 지연 100ms)보다 크고,
 *  odometry 보간이 감당하는 범위(150ms)를 넘는 값으로 둔다. */
const MEDIA_CLOCK_RESYNC_NS = 250 * NS_PER_MS;

function finite(value, fallback = null) {
  if (value === null || value === undefined || value === "") return fallback;
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function positive(value) {
  const number = finite(value);
  return number !== null && number > 0 ? number : null;
}

export function cameraOdometryObservationTimestampNs(
  odometry,
  poseDelayMs = CAMERA_ODOMETRY_POSE_DELAY_MS,
) {
  const timestampEof = positive(odometry?.timestampEof);
  const delayMs = Math.max(0, finite(poseDelayMs, CAMERA_ODOMETRY_POSE_DELAY_MS));
  return timestampEof === null ? null : timestampEof - delayMs * NS_PER_MS;
}

/**
 * Converts the metadata of the frame actually shown by the video element into
 * a spatial target clock. Missing mapping stays explicitly unmapped; it never
 * falls back to performance.now().
 *
 * @param reference  같은 tick에 디코드된 cereal 카메라 상태
 *                   `{ cameraTimestampEof }`. replay 영상은 `cameraTimestampEof`를
 *                   프레임 metadata로 싣지 않으므로, media↔cereal offset을 세울
 *                   유일한 근거가 이 동시 관측이다. replay 엔진이 cereal과 영상을
 *                   같은 미디어 시각으로 재생하므로 둘의 차이가 곧 offset이다.
 *                   offset이 한 번 서면 이후 프레임은 mediaTime만으로 cereal
 *                   시각을 얻는다(ESTIMATED_FRAME).
 */
export function createPresentedFrameClockMapper() {
  let last = null;
  let mediaToCerealOffsetNs = null;

  function map(presented = null, timeline = null, reference = null) {
    const sourceMatches = presented?.source === "replay"
      ? timeline?.domain === "replay-media"
      : presented?.source === "live"
        ? timeline?.domain !== "replay-media"
        : false;
    const cameraTimestampNs = positive(presented?.cameraTimestampEof);
    const mediaTimeSeconds = finite(presented?.mediaTime);
    const mediaTimeMs = mediaTimeSeconds === null ? null : mediaTimeSeconds * 1000;
    const sourceFrameId = finite(presented?.frameId);
    const suppliedConfidence = String(presented?.clockMappingConfidence || "");

    if (sourceMatches && cameraTimestampNs !== null) {
      const confidence = suppliedConfidence === AR_PRESENTED_CLOCK_CONFIDENCE.ESTIMATED_FRAME
        ? AR_PRESENTED_CLOCK_CONFIDENCE.ESTIMATED_FRAME
        : AR_PRESENTED_CLOCK_CONFIDENCE.EXACT_FRAME;
      last = Object.freeze({
        domain: "cereal-monotonic",
        targetTimeMs: cameraTimestampNs / NS_PER_MS,
        targetTimestampNs: cameraTimestampNs,
        mediaTimeMs,
        mediaToCerealOffsetNs: mediaTimeMs === null
          ? null
          : cameraTimestampNs - mediaTimeSeconds * NS_PER_SECOND,
        sourceFrameId,
        sequence: presented?.sequence ?? null,
        confidence,
        mapped: true,
      });
      if (last.mediaToCerealOffsetNs !== null) mediaToCerealOffsetNs = last.mediaToCerealOffsetNs;
      return last;
    }

    if (sourceMatches && timeline?.domain === "replay-media" && mediaTimeMs !== null) {
      /* replay 영상 프레임에는 cereal timestamp가 실려 오지 않는다. 그래서 위의
       * exact 분기가 한 번도 성립하지 않고 offset이 영원히 비어 있었다(= spatial
       * clock 없음 → world pose 미초기화 → tracking LOST). 같은 tick에 디코드된
       * roadCameraState.timestampEof로 offset을 한 번 세워 그 사슬을 끊는다. */
      const referenceTimestampNs = positive(reference?.cameraTimestampEof);
      if (referenceTimestampNs !== null) {
        const impliedOffsetNs = referenceTimestampNs - mediaTimeSeconds * NS_PER_SECOND;
        /* offset은 같은 영상 프레임 동안 target을 고정하려고 유지하는 값이라
         * 매 tick 다시 쓰지 않는다. 다만 seek 직후처럼 cereal이 영상보다 크게
         * 뒤처진 순간에 씨앗을 잡으면 그 오차가 영구히 고정되어 sampleAt이
         * 계속 null을 반환한다(= odometry propagation unavailable). 그래서
         * 정상 지터를 넘는 불일치에서만 한 번 재동기한다. */
        if (mediaToCerealOffsetNs === null
          || Math.abs(impliedOffsetNs - mediaToCerealOffsetNs) > MEDIA_CLOCK_RESYNC_NS) {
          mediaToCerealOffsetNs = impliedOffsetNs;
        }
      }
      if (mediaToCerealOffsetNs !== null) {
        const targetTimestampNs = mediaTimeSeconds * NS_PER_SECOND + mediaToCerealOffsetNs;
        last = Object.freeze({
          domain: "cereal-monotonic",
          targetTimeMs: targetTimestampNs / NS_PER_MS,
          targetTimestampNs,
          mediaTimeMs,
          mediaToCerealOffsetNs,
          sourceFrameId,
          sequence: presented?.sequence ?? null,
          confidence: AR_PRESENTED_CLOCK_CONFIDENCE.ESTIMATED_FRAME,
          mapped: true,
        });
        return last;
      }
      last = Object.freeze({
        domain: "replay-media",
        targetTimeMs: mediaTimeMs,
        targetTimestampNs: null,
        mediaTimeMs,
        mediaToCerealOffsetNs: null,
        sourceFrameId,
        sequence: presented?.sequence ?? null,
        confidence: AR_PRESENTED_CLOCK_CONFIDENCE.MEDIA_FRAME,
        mapped: true,
      });
      return last;
    }

    /* presented 이벤트가 media 시각을 싣지 않는 경우가 있다(리플레이 컨트롤러가
     * 항상 넣지는 않고, live 경로에는 애초에 없다). 그때도 같은 tick에 디코드된
     * cereal 카메라 시각이 있으면 그것이 곧 현재 영상 프레임의 시각이므로 그대로
     * spatial target으로 쓴다. 같은 카메라 프레임 동안 값이 고정되므로 30Hz 표시
     * tick 사이에서 target이 흔들리지 않는다. */
    const referenceTimestampNs = positive(reference?.cameraTimestampEof);
    if (sourceMatches && referenceTimestampNs !== null) {
      last = Object.freeze({
        domain: "cereal-monotonic",
        targetTimeMs: referenceTimestampNs / NS_PER_MS,
        targetTimestampNs: referenceTimestampNs,
        mediaTimeMs,
        mediaToCerealOffsetNs,
        sourceFrameId,
        sequence: presented?.sequence ?? null,
        confidence: AR_PRESENTED_CLOCK_CONFIDENCE.ESTIMATED_FRAME,
        mapped: true,
      });
      return last;
    }

    last = Object.freeze({
      domain: timeline?.domain || null,
      targetTimeMs: null,
      targetTimestampNs: null,
      mediaTimeMs,
      mediaToCerealOffsetNs: null,
      sourceFrameId,
      sequence: presented?.sequence ?? null,
      confidence: AR_PRESENTED_CLOCK_CONFIDENCE.UNMAPPED,
      mapped: false,
    });
    return last;
  }

  function reset() {
    last = null;
    mediaToCerealOffsetNs = null;
  }

  function status() {
    return last;
  }

  return Object.freeze({ map, reset, status });
}

function vector3(value) {
  if (!Array.isArray(value) || value.length < 3) return null;
  const values = value.slice(0, 3).map(Number);
  return values.every(Number.isFinite) ? values : null;
}

function interpolateVector(a, b, alpha) {
  const left = vector3(a);
  const right = vector3(b);
  if (!left || !right) return null;
  return Object.freeze(left.map((value, index) => value + (right[index] - value) * alpha));
}

function alignedSample(source, fields) {
  return Object.freeze({
    ...source,
    ...fields,
    temporalAlignment: Object.freeze(fields.temporalAlignment),
  });
}

/** Retains delayed 20Hz odometry and resolves the sample for a video frame. */
export function createCameraOdometryTimeline(options = {}) {
  const poseDelayMs = Math.max(
    0,
    finite(options.poseDelayMs, CAMERA_ODOMETRY_POSE_DELAY_MS),
  );
  const capacity = Math.max(2, Math.round(finite(options.capacity, 64)));
  const maxExtrapolationMs = Math.max(0, finite(options.maxExtrapolationMs, 150));
  let samples = [];

  function push(odometry) {
    const observationTimestampNs = cameraOdometryObservationTimestampNs(odometry, poseDelayMs);
    if (observationTimestampNs === null || !vector3(odometry?.trans) || !vector3(odometry?.rot)) {
      return false;
    }
    const sample = Object.freeze({ odometry, observationTimestampNs });
    const previous = samples[samples.length - 1];
    if (previous?.observationTimestampNs === observationTimestampNs) samples[samples.length - 1] = sample;
    else {
      samples.push(sample);
      samples.sort((a, b) => a.observationTimestampNs - b.observationTimestampNs);
      if (samples.length > capacity) samples = samples.slice(-capacity);
    }
    return true;
  }

  function sampleAt(targetTimestampNs) {
    const target = positive(targetTimestampNs);
    if (target === null || !samples.length) return null;

    const upperIndex = samples.findIndex((sample) => sample.observationTimestampNs >= target);
    if (upperIndex === -1) {
      const latest = samples[samples.length - 1];
      const deltaMs = (target - latest.observationTimestampNs) / NS_PER_MS;
      if (deltaMs > maxExtrapolationMs) return null;
      return alignedSample(latest.odometry, {
        observationTimestampNs: latest.observationTimestampNs,
        targetTimestampNs: target,
        temporalAlignment: {
          mode: deltaMs === 0 ? "exact" : "bounded-extrapolation",
          poseDelayMs,
          targetDeltaMs: deltaMs,
        },
      });
    }

    const upper = samples[upperIndex];
    if (upper.observationTimestampNs === target || upperIndex === 0) {
      const deltaMs = (upper.observationTimestampNs - target) / NS_PER_MS;
      if (deltaMs > maxExtrapolationMs) return null;
      return alignedSample(upper.odometry, {
        observationTimestampNs: upper.observationTimestampNs,
        targetTimestampNs: target,
        temporalAlignment: {
          mode: deltaMs === 0 ? "exact" : "bounded-lookahead",
          poseDelayMs,
          targetDeltaMs: -deltaMs,
        },
      });
    }

    const lower = samples[upperIndex - 1];
    const span = upper.observationTimestampNs - lower.observationTimestampNs;
    const alpha = span > 0 ? (target - lower.observationTimestampNs) / span : 0;
    const trans = interpolateVector(lower.odometry.trans, upper.odometry.trans, alpha);
    const rot = interpolateVector(lower.odometry.rot, upper.odometry.rot, alpha);
    const transStd = interpolateVector(lower.odometry.transStd, upper.odometry.transStd, alpha);
    const rotStd = interpolateVector(lower.odometry.rotStd, upper.odometry.rotStd, alpha);
    return alignedSample(lower.odometry, {
      ...(trans ? { trans } : {}),
      ...(rot ? { rot } : {}),
      ...(transStd ? { transStd } : {}),
      ...(rotStd ? { rotStd } : {}),
      observationTimestampNs: target,
      targetTimestampNs: target,
      temporalAlignment: {
        mode: "interpolated",
        poseDelayMs,
        targetDeltaMs: 0,
        alpha,
      },
    });
  }

  function reset() {
    samples = [];
  }

  function status() {
    return Object.freeze({
      samples: samples.length,
      poseDelayMs,
      maxExtrapolationMs,
      firstObservationTimestampNs: samples[0]?.observationTimestampNs ?? null,
      lastObservationTimestampNs: samples.at(-1)?.observationTimestampNs ?? null,
    });
  }

  return Object.freeze({ push, sampleAt, reset, status });
}
