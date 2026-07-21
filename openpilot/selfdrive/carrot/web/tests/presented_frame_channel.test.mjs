import assert from "node:assert/strict";
import test from "node:test";

import {
  createPresentedFrameChannel,
  installDriveVisionPresentedFrameChannelFacade,
} from "../src/features/drive/contents/vision/presented_frame_channel.js";

test("presented-frame channel publishes one ordered live/replay contract", () => {
  const received = [];
  const errors = [];
  const channel = createPresentedFrameChannel({ reportError: (error) => errors.push(error) });
  const unsubscribe = channel.subscribe((frame) => received.push(frame));
  channel.subscribe(() => { throw new Error("listener failure"); });

  const live = channel.publish({ source: "live", frameId: 17 });
  const replay = channel.publish({ source: "replay", frameId: 18, mediaTime: 1.5 });

  assert.deepEqual(received, [live, replay]);
  assert.equal(live.sequence, 1);
  assert.equal(replay.sequence, 2);
  assert.equal(replay.source, "replay");
  assert.equal(errors.length, 2);
  assert.equal(channel.status().listeners, 2);
  assert.equal(channel.status().last, replay);
  assert.equal(unsubscribe(), true);
  assert.equal(unsubscribe(), false);
});

test("presented-frame facade is one channel per target", () => {
  const target = { console: { error() {} } };
  const first = installDriveVisionPresentedFrameChannelFacade(target);
  const second = installDriveVisionPresentedFrameChannelFacade(target);
  assert.equal(first, second);
  assert.equal(target.DriveVisionPresentedFrames, first);
});
