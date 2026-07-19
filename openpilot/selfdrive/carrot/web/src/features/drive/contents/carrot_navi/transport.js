export const SESSION_CONTROL_TYPE = "carrotNaviSession";
export const SESSION_ACCEPTED_CODE = "carrot_navi_accepted";
export const SESSION_BUSY_CODE = "carrot_navi_busy";
export const SESSION_REPLACED_CODE = "carrot_navi_replaced";
export const SESSION_BUSY_CLOSE_CODE = 4409;
export const SESSION_REPLACED_CLOSE_CODE = 4401;
export const NAVIGATION_MEDIA_WORKER_ID = "navigation.media.worker";

function resolveNavigationWorkerUrl(options, target) {
  const resolveAssetUrl = options.resolveAssetUrl
    || ((logicalId) => globalThis.CarrotAssetUrl.resolve(logicalId));
  const url = String(resolveAssetUrl(NAVIGATION_MEDIA_WORKER_ID) || "");
  if (!url) throw new Error("Carrot Navi media worker asset URL is empty");
  return url;
}

export function createTransport(options = {}) {
  const target = options.target || globalThis;
  const statePath = String(options.statePath || "/ws/carrot_navi/state");
  const mediaPath = String(options.mediaPath || "/ws/carrot_navi/media");
  const workerUrl = resolveNavigationWorkerUrl(options, target);
  const reconnectMs = Math.max(100, Number(options.reconnectMs) || 500);
  const clientId = String(target.CarrotStreamIdentity?.clientId
    || `carrot-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`);
  let active = false;
  let ownershipBlocked = false;
  let ownershipCode = "";
  let takeoverArmed = true;
  let stateAccepted = false;
  let stateSocket = null;
  let mediaSocket = null;
  let worker = null;
  let reconnectTimer = 0;

  function websocketUrl(path, takeover = false) {
    const url = new target.URL(path, target.location.href);
    url.protocol = target.location.protocol === "https:" ? "wss:" : "ws:";
    url.searchParams.set("client_id", clientId);
    if (takeover) url.searchParams.set("takeover", "1");
    return url.href;
  }

  function snapshot() {
    return {
      active,
      ownershipBlocked,
      ownershipCode,
      takeoverArmed,
      stateAccepted,
      stateSocketState: stateSocket?.readyState ?? -1,
      mediaSocketState: mediaSocket?.readyState ?? -1,
      stateOpen: stateSocket?.readyState === target.WebSocket.OPEN,
      mediaOpen: mediaSocket?.readyState === target.WebSocket.OPEN,
      workerActive: Boolean(worker),
    };
  }

  function notifyConnection() {
    options.onConnectionChange?.(snapshot());
  }

  function closeSocket(socket) {
    if (!socket) return;
    try { socket.close(); } catch (_) {}
  }

  function ensureWorker() {
    if (worker) return worker;
    worker = new Worker(workerUrl);
    worker.addEventListener("message", (event) => options.onWorkerMessage?.(event.data || {}));
    worker.addEventListener("error", (event) => options.onError?.(event.message || "Carrot Navi worker error"));
    worker.postMessage({ type: "configure" });
    return worker;
  }

  function markOwnershipBusy(code = SESSION_BUSY_CODE) {
    const normalizedCode = String(code || SESSION_BUSY_CODE);
    const changed = !ownershipBlocked || ownershipCode !== normalizedCode;
    ownershipBlocked = true;
    ownershipCode = normalizedCode;
    takeoverArmed = false;
    stateAccepted = false;
    target.clearTimeout(reconnectTimer);
    reconnectTimer = 0;
    const state = stateSocket;
    const media = mediaSocket;
    stateSocket = null;
    mediaSocket = null;
    closeSocket(state);
    closeSocket(media);
    worker?.postMessage({ type: "reset-media" });
    if (changed) options.onOwnershipBusy?.(normalizedCode);
    notifyConnection();
  }

  function handleSessionMessage(data, socket, kind) {
    if (typeof data !== "string") return false;
    try {
      const message = JSON.parse(data);
      if (message?.type !== SESSION_CONTROL_TYPE) return false;
      if (message.status === "busy") {
        markOwnershipBusy(message.code || SESSION_BUSY_CODE);
        return true;
      }
      if (message.status !== "accepted" || message.code !== SESSION_ACCEPTED_CODE) return true;
      const current = kind === "state" ? stateSocket === socket : mediaSocket === socket;
      if (!current) return true;
      ownershipBlocked = false;
      ownershipCode = "";
      if (kind === "state") {
        stateAccepted = true;
        connectMedia();
      }
      notifyConnection();
      return true;
    } catch (_) {
      return false;
    }
  }

  function handleSocketClose(event, socket, kind) {
    const isState = kind === "state";
    const wasCurrent = isState ? stateSocket === socket : mediaSocket === socket;
    if (!wasCurrent) return;
    if (isState) {
      stateSocket = null;
      stateAccepted = false;
    } else mediaSocket = null;
    if (event.code === SESSION_BUSY_CLOSE_CODE || event.code === SESSION_REPLACED_CLOSE_CODE) {
      markOwnershipBusy(event.code === SESSION_REPLACED_CLOSE_CODE ? SESSION_REPLACED_CODE : SESSION_BUSY_CODE);
      return;
    }
    if (isState && active) options.onStateDisconnected?.();
    notifyConnection();
    scheduleReconnect();
  }

  function scheduleReconnect() {
    target.clearTimeout(reconnectTimer);
    reconnectTimer = 0;
    if (!active || ownershipBlocked) return;
    reconnectTimer = target.setTimeout(() => {
      reconnectTimer = 0;
      connect();
    }, reconnectMs);
  }

  function connectState() {
    if (!active || ownershipBlocked || stateSocket) return;
    stateAccepted = false;
    const socket = new target.WebSocket(websocketUrl(statePath, takeoverArmed));
    stateSocket = socket;
    socket.addEventListener("open", () => {
      if (stateSocket !== socket) return;
      takeoverArmed = false;
      notifyConnection();
    });
    socket.addEventListener("message", (event) => {
      if (handleSessionMessage(event.data, socket, "state")) return;
      if (typeof event.data !== "string") return;
      try {
        const message = JSON.parse(event.data);
        if (message?.type === "carrotNaviState" && message.state) options.onState?.(message.state);
      } catch (_) {}
    });
    socket.addEventListener("close", (event) => handleSocketClose(event, socket, "state"));
    socket.addEventListener("error", () => closeSocket(socket));
  }

  function connectMedia() {
    if (!active || ownershipBlocked || !stateAccepted || mediaSocket) return;
    const decoderWorker = ensureWorker();
    const socket = new target.WebSocket(websocketUrl(mediaPath, false));
    socket.binaryType = "arraybuffer";
    mediaSocket = socket;
    socket.addEventListener("open", () => {
      if (mediaSocket !== socket) return;
      notifyConnection();
    });
    socket.addEventListener("message", (event) => {
      if (handleSessionMessage(event.data, socket, "media")) return;
      if (!(event.data instanceof ArrayBuffer) || worker !== decoderWorker) return;
      decoderWorker.postMessage({ type: "media", buffer: event.data }, [event.data]);
    });
    socket.addEventListener("close", (event) => handleSocketClose(event, socket, "media"));
    socket.addEventListener("error", () => closeSocket(socket));
  }

  function connect() {
    if (!active || ownershipBlocked) return;
    connectState();
    if (stateAccepted) connectMedia();
  }

  function start() {
    if (!active) active = true;
    connect();
    notifyConnection();
  }

  function recoverMedia() {
    if (!active || ownershipBlocked) return;
    const socket = mediaSocket;
    mediaSocket = null;
    closeSocket(socket);
    worker?.postMessage({ type: "reset-media" });
    scheduleReconnect();
    notifyConnection();
  }

  function requestOwnershipTakeover() {
    const state = stateSocket;
    const media = mediaSocket;
    stateSocket = null;
    mediaSocket = null;
    stateAccepted = false;
    closeSocket(state);
    closeSocket(media);
    ownershipBlocked = false;
    ownershipCode = "";
    takeoverArmed = true;
    target.clearTimeout(reconnectTimer);
    reconnectTimer = 0;
    if (active) connect();
    notifyConnection();
  }

  function stop() {
    active = false;
    ownershipBlocked = false;
    ownershipCode = "";
    takeoverArmed = false;
    stateAccepted = false;
    target.clearTimeout(reconnectTimer);
    reconnectTimer = 0;
    const state = stateSocket;
    const media = mediaSocket;
    stateSocket = null;
    mediaSocket = null;
    closeSocket(state);
    closeSocket(media);
    if (worker) {
      worker.postMessage({ type: "reset" });
      worker.terminate();
      worker = null;
    }
    notifyConnection();
  }

  return Object.freeze({ start, stop, recoverMedia, requestOwnershipTakeover, snapshot });
}

export const CarrotNaviTransport = Object.freeze({
  SESSION_BUSY_CODE,
  SESSION_REPLACED_CODE,
  create: createTransport,
});

export function installCarrotNaviTransportGlobal(target = globalThis) {
  target.CarrotNaviTransport = CarrotNaviTransport;
  return CarrotNaviTransport;
}
