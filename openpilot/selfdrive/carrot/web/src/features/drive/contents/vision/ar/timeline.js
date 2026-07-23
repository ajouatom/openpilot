export const AR_CLOCK_DOMAIN = Object.freeze({
  LIVE_MONOTONIC: "live-monotonic",
  REPLAY_MEDIA: "replay-media",
});

export const AR_TIMELINE_DISCONTINUITY = Object.freeze({
  CLOCK_DOMAIN_CHANGE: "clock-domain-change",
  SESSION_EPOCH_CHANGE: "session-epoch-change",
  TIME_REVERSAL: "time-reversal",
  REPLAY_SEEK: "replay-seek",
});

function finite(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function normalizeEpoch(value) {
  if (value === null || value === undefined || value === "") return null;
  return String(value);
}

/**
 * Tracks the selected clock without treating ordinary browser scheduling delay
 * as a seek. A discontinuity is a semantic boundary, not a large frame dt.
 */
export function createArTimelineTracker() {
  let previous = null;

  function update(sample = {}) {
    const domain = sample.domain === AR_CLOCK_DOMAIN.REPLAY_MEDIA
      ? AR_CLOCK_DOMAIN.REPLAY_MEDIA
      : AR_CLOCK_DOMAIN.LIVE_MONOTONIC;
    const nowMs = Math.max(0, finite(sample.nowMs, 0));
    const epoch = normalizeEpoch(sample.epoch);
    const deltaMs = previous?.domain === domain ? nowMs - previous.nowMs : null;
    let discontinuityReason = null;

    if (previous) {
      if (sample.explicitDiscontinuity === true) {
        discontinuityReason = String(
          sample.discontinuityReason || AR_TIMELINE_DISCONTINUITY.REPLAY_SEEK,
        );
      } else if (previous.domain !== domain) {
        discontinuityReason = AR_TIMELINE_DISCONTINUITY.CLOCK_DOMAIN_CHANGE;
      } else if (previous.epoch !== epoch) {
        discontinuityReason = AR_TIMELINE_DISCONTINUITY.SESSION_EPOCH_CHANGE;
      } else if (deltaMs < 0) {
        discontinuityReason = AR_TIMELINE_DISCONTINUITY.TIME_REVERSAL;
      }
    }

    previous = Object.freeze({ domain, nowMs, epoch });
    return Object.freeze({
      domain,
      nowMs,
      epoch,
      deltaMs,
      discontinuity: discontinuityReason !== null,
      discontinuityReason,
    });
  }

  function reset() {
    previous = null;
  }

  function status() {
    return previous;
  }

  return Object.freeze({ update, reset, status });
}
