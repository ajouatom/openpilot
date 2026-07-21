const DRIVE_DATA_CHANNELS = Object.freeze(["hud", "overlay", "tracks", "ar"]);
// "tracks" (radar track list) and "ar" (20Hz odometry + pose) are far heavier
// than the base channels, so an unqualified lease never opts into them.
// Only a caller that names the channel gets it.
const DRIVE_DATA_DEFAULT_CHANNELS = Object.freeze(["hud", "overlay"]);
const DRIVE_DATA_ACTIVITY_EVENT = "carrot:drivedataactivitychange";

class DriveDataActivityError extends Error {
  constructor(code, message) {
    super(message);
    this.name = "DriveDataActivityError";
    this.code = code;
  }
}

function normalizeChannels(request = {}) {
  if (request === "hud+overlay") return DRIVE_DATA_DEFAULT_CHANNELS;
  if (typeof request === "string") request = { [request]: true };
  if (Array.isArray(request)) {
    request = Object.fromEntries(request.map((channel) => [channel, true]));
  }
  if (!request || typeof request !== "object") {
    throw new DriveDataActivityError("INVALID_LEASE", "drive data lease options must be an object");
  }

  const hasExplicitChannel = DRIVE_DATA_CHANNELS.some((channel) => (
    Object.prototype.hasOwnProperty.call(request, channel)
  ));
  const channels = hasExplicitChannel
    ? DRIVE_DATA_CHANNELS.filter((channel) => request[channel] === true)
    : DRIVE_DATA_DEFAULT_CHANNELS;
  if (!channels.length) {
    throw new DriveDataActivityError("EMPTY_LEASE", "drive data lease must request at least one channel");
  }
  return Object.freeze(channels);
}

function dispatchActivityEvent(target, snapshot) {
  if (typeof target?.dispatchEvent !== "function" || typeof target?.CustomEvent !== "function") return;
  target.dispatchEvent(new target.CustomEvent(DRIVE_DATA_ACTIVITY_EVENT, { detail: snapshot }));
}

function createDriveDataActivity(options = {}) {
  const target = options.target || globalThis;
  const eventName = String(options.eventName || DRIVE_DATA_ACTIVITY_EVENT);
  const leases = new Map();
  const listeners = new Set();
  const channelCounts = Object.fromEntries(DRIVE_DATA_CHANNELS.map((channel) => [channel, 0]));
  let nextLeaseId = 0;
  let revision = 0;
  let destroyed = false;
  let currentSnapshot = Object.freeze({
    revision,
    active: false,
    leaseCount: 0,
    counts: Object.freeze({ ...channelCounts }),
  });

  function assertAlive(operation) {
    if (destroyed) {
      throw new DriveDataActivityError("ACTIVITY_DESTROYED", `drive data activity cannot ${operation} after destroy`);
    }
  }

  function publish() {
    revision += 1;
    currentSnapshot = Object.freeze({
      revision,
      active: leases.size > 0,
      leaseCount: leases.size,
      counts: Object.freeze({ ...channelCounts }),
    });
    for (const listener of Array.from(listeners)) {
      try {
        listener(currentSnapshot);
      } catch (error) {
        target?.console?.error?.("[drive data activity] listener failed", error);
      }
    }
    if (eventName === DRIVE_DATA_ACTIVITY_EVENT) {
      dispatchActivityEvent(target, currentSnapshot);
    } else if (typeof target?.dispatchEvent === "function" && typeof target?.CustomEvent === "function") {
      target.dispatchEvent(new target.CustomEvent(eventName, { detail: currentSnapshot }));
    }
    return currentSnapshot;
  }

  function releaseById(id) {
    const entry = leases.get(id);
    if (!entry) return false;
    leases.delete(id);
    for (const channel of entry.channels) channelCounts[channel] -= 1;
    publish();
    return true;
  }

  function acquire(request = {}) {
    assertAlive("acquire");
    const channels = normalizeChannels(request);
    const id = `drive-data-${++nextLeaseId}`;
    const owner = String(request?.owner || "anonymous");
    const entry = Object.freeze({ id, owner, channels });
    leases.set(id, entry);
    for (const channel of channels) channelCounts[channel] += 1;
    publish();

    let released = false;
    return Object.freeze({
      id,
      owner,
      channels,
      get active() {
        return !released && leases.has(id);
      },
      release() {
        if (released) return false;
        released = true;
        return releaseById(id);
      },
    });
  }

  function release(leaseOrId) {
    const id = typeof leaseOrId === "string" ? leaseOrId : leaseOrId?.id;
    if (!id) return false;
    return releaseById(id);
  }

  function isActive(channel = null) {
    if (channel == null) return leases.size > 0;
    if (!DRIVE_DATA_CHANNELS.includes(channel)) return false;
    return channelCounts[channel] > 0;
  }

  function counts() {
    return Object.freeze({ ...channelCounts });
  }

  function snapshot() {
    return currentSnapshot;
  }

  function subscribe(listener) {
    assertAlive("subscribe");
    if (typeof listener !== "function") {
      throw new DriveDataActivityError("INVALID_LISTENER", "drive data activity listener must be a function");
    }
    listeners.add(listener);
    let subscribed = true;
    return () => {
      if (!subscribed) return false;
      subscribed = false;
      return listeners.delete(listener);
    };
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    const changed = leases.size > 0;
    leases.clear();
    for (const channel of DRIVE_DATA_CHANNELS) channelCounts[channel] = 0;
    if (changed) publish();
    listeners.clear();
    return true;
  }

  return Object.freeze({
    acquire,
    release,
    isActive,
    counts,
    snapshot,
    subscribe,
    destroy,
  });
}

function installDriveDataActivityFacade(target = globalThis, options = {}) {
  const existing = target?.CarrotDriveDataActivity;
  if (existing && typeof existing.acquire === "function" && typeof existing.isActive === "function") {
    return existing;
  }
  const activity = createDriveDataActivity({ ...options, target });
  target.CarrotDriveDataActivity = activity;
  return activity;
}

export {
  DRIVE_DATA_ACTIVITY_EVENT,
  DRIVE_DATA_CHANNELS,
  DRIVE_DATA_DEFAULT_CHANNELS,
  DriveDataActivityError,
  createDriveDataActivity,
  installDriveDataActivityFacade,
};
