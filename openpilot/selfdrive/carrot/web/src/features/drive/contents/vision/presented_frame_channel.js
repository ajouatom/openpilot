/**
 * Shared notification channel for frames that have actually reached the
 * visible Vision surface. AR deliberately follows this channel instead of
 * owning another requestAnimationFrame loop, so the camera and AR canvases
 * advance together in both live and replay modes.
 */
export function createPresentedFrameChannel(options = {}) {
  const listeners = new Set();
  const reportError = typeof options.reportError === "function"
    ? options.reportError
    : () => {};
  let sequence = 0;
  let last = null;

  function subscribe(listener) {
    if (typeof listener !== "function") {
      throw new TypeError("presented-frame listener must be a function");
    }
    listeners.add(listener);
    let subscribed = true;
    return () => {
      if (!subscribed) return false;
      subscribed = false;
      return listeners.delete(listener);
    };
  }

  function publish(detail = {}) {
    const source = detail?.source === "replay" ? "replay" : "live";
    last = Object.freeze({
      ...detail,
      source,
      sequence: ++sequence,
    });
    for (const listener of [...listeners]) {
      try {
        listener(last);
      } catch (error) {
        reportError(error);
      }
    }
    return last;
  }

  function status() {
    return Object.freeze({
      listeners: listeners.size,
      sequence,
      last,
    });
  }

  return Object.freeze({ subscribe, publish, status });
}

const installedChannels = new WeakMap();

export function installDriveVisionPresentedFrameChannelFacade(target = globalThis) {
  const existing = installedChannels.get(target);
  if (existing) return existing;
  const channel = createPresentedFrameChannel({
    reportError(error) {
      target?.console?.error?.("[drive vision] presented-frame listener failed", error);
    },
  });
  installedChannels.set(target, channel);
  target.DriveVisionPresentedFrames = channel;
  return channel;
}

export const DriveVisionPresentedFrameChannel = Object.freeze({
  create: createPresentedFrameChannel,
  install: installDriveVisionPresentedFrameChannelFacade,
});
