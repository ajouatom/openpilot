import assert from "node:assert/strict";
import test from "node:test";

import {
  AR_CLOCK_DOMAIN,
  AR_TIMELINE_DISCONTINUITY,
  createArTimelineTracker,
} from "../src/features/drive/contents/vision/ar/timeline.js";

test("ordinary forward scheduling gaps never become timeline discontinuities", () => {
  const tracker = createArTimelineTracker();
  tracker.update({ domain: AR_CLOCK_DOMAIN.LIVE_MONOTONIC, nowMs: 100, epoch: "live" });
  const delayed = tracker.update({
    domain: AR_CLOCK_DOMAIN.LIVE_MONOTONIC,
    nowMs: 500,
    epoch: "live",
  });

  assert.equal(delayed.deltaMs, 400);
  assert.equal(delayed.discontinuity, false);
  assert.equal(delayed.discontinuityReason, null);
});

test("clock domain, session epoch, time reversal, and explicit seek are boundaries", () => {
  const tracker = createArTimelineTracker();
  tracker.update({ domain: AR_CLOCK_DOMAIN.LIVE_MONOTONIC, nowMs: 100, epoch: "live" });
  const domain = tracker.update({
    domain: AR_CLOCK_DOMAIN.REPLAY_MEDIA,
    nowMs: 1_000,
    epoch: "segment-a",
  });
  assert.equal(domain.discontinuityReason, AR_TIMELINE_DISCONTINUITY.CLOCK_DOMAIN_CHANGE);

  const epoch = tracker.update({
    domain: AR_CLOCK_DOMAIN.REPLAY_MEDIA,
    nowMs: 1_050,
    epoch: "segment-b",
  });
  assert.equal(epoch.discontinuityReason, AR_TIMELINE_DISCONTINUITY.SESSION_EPOCH_CHANGE);

  const reverse = tracker.update({
    domain: AR_CLOCK_DOMAIN.REPLAY_MEDIA,
    nowMs: 900,
    epoch: "segment-b",
  });
  assert.equal(reverse.discontinuityReason, AR_TIMELINE_DISCONTINUITY.TIME_REVERSAL);

  const seek = tracker.update({
    domain: AR_CLOCK_DOMAIN.REPLAY_MEDIA,
    nowMs: 1_200,
    epoch: "segment-b",
    explicitDiscontinuity: true,
    discontinuityReason: "replay-seek",
  });
  assert.equal(seek.discontinuityReason, AR_TIMELINE_DISCONTINUITY.REPLAY_SEEK);
});

test("reset starts a fresh timeline without inventing a boundary", () => {
  const tracker = createArTimelineTracker();
  tracker.update({ domain: AR_CLOCK_DOMAIN.REPLAY_MEDIA, nowMs: 1_000, epoch: "segment-a" });
  tracker.reset();
  const first = tracker.update({
    domain: AR_CLOCK_DOMAIN.LIVE_MONOTONIC,
    nowMs: 5_000,
    epoch: "live",
  });
  assert.equal(first.deltaMs, null);
  assert.equal(first.discontinuity, false);
});
