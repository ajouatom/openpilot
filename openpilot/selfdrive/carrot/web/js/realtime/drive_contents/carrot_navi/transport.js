"use strict";

globalThis.CarrotNaviTransport = (() => {
  function create(options = {}) {
    const statePath = String(options.statePath || "/ws/carrot_navi/state");
    const mediaPath = String(options.mediaPath || "/ws/carrot_navi/media");
    const workerUrl = String(options.workerUrl || "/js/realtime/drive_contents/carrot_navi/worker.js");
    const reconnectMs = Math.max(100, Number(options.reconnectMs) || 500);
    let active = false;
    let stateSocket = null;
    let mediaSocket = null;
    let worker = null;
    let reconnectTimer = 0;

    function websocketUrl(path) {
      const protocol = location.protocol === "https:" ? "wss:" : "ws:";
      return `${protocol}//${location.host}${path}`;
    }

    function snapshot() {
      return {
        active,
        stateSocketState: stateSocket?.readyState ?? -1,
        mediaSocketState: mediaSocket?.readyState ?? -1,
        stateOpen: stateSocket?.readyState === WebSocket.OPEN,
        mediaOpen: mediaSocket?.readyState === WebSocket.OPEN,
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

    function scheduleReconnect() {
      globalThis.clearTimeout(reconnectTimer);
      reconnectTimer = 0;
      if (!active) return;
      reconnectTimer = globalThis.setTimeout(() => {
        reconnectTimer = 0;
        connect();
      }, reconnectMs);
    }

    function connectState() {
      if (!active || stateSocket) return;
      const socket = new WebSocket(websocketUrl(statePath));
      stateSocket = socket;
      socket.addEventListener("open", notifyConnection);
      socket.addEventListener("message", (event) => {
        if (typeof event.data !== "string") return;
        try {
          const message = JSON.parse(event.data);
          if (message?.type === "carrotNaviState" && message.state) options.onState?.(message.state);
        } catch (_) {}
      });
      socket.addEventListener("close", () => {
        const wasCurrent = stateSocket === socket;
        if (wasCurrent) stateSocket = null;
        if (active && wasCurrent) options.onStateDisconnected?.();
        notifyConnection();
        scheduleReconnect();
      });
      socket.addEventListener("error", () => closeSocket(socket));
    }

    function connectMedia() {
      if (!active || mediaSocket) return;
      const decoderWorker = ensureWorker();
      const socket = new WebSocket(websocketUrl(mediaPath));
      socket.binaryType = "arraybuffer";
      mediaSocket = socket;
      socket.addEventListener("open", notifyConnection);
      socket.addEventListener("message", (event) => {
        if (!(event.data instanceof ArrayBuffer) || worker !== decoderWorker) return;
        decoderWorker.postMessage({ type: "media", buffer: event.data }, [event.data]);
      });
      socket.addEventListener("close", () => {
        if (mediaSocket === socket) mediaSocket = null;
        notifyConnection();
        scheduleReconnect();
      });
      socket.addEventListener("error", () => closeSocket(socket));
    }

    function connect() {
      if (!active) return;
      connectState();
      connectMedia();
    }

    function start() {
      if (!active) active = true;
      connect();
      notifyConnection();
    }

    function recoverMedia() {
      if (!active) return;
      const socket = mediaSocket;
      mediaSocket = null;
      closeSocket(socket);
      worker?.postMessage({ type: "reset-media" });
      scheduleReconnect();
      notifyConnection();
    }

    function stop() {
      active = false;
      globalThis.clearTimeout(reconnectTimer);
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

    return Object.freeze({ start, stop, recoverMedia, snapshot });
  }

  return Object.freeze({ create });
})();
