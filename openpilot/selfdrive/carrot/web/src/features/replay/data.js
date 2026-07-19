"use strict";

window.CarrotReplayInsightsData = window.CarrotReplayInsightsData || (() => {
  const ADVANCED_WORKER_URL = "/js/realtime/replay_advanced_worker.js?v=2607-02";
  const POLICY = Object.freeze({
    chunkSize: 240,
    numericSampleIntervalMs: 75,
    maxStateSamples: 6000,
    rawGraphPointLimit: 1200,
  });

  const GROUPS = Object.freeze([
    { key: "control", labelKey: "replay_group_control" },
    { key: "vehicle", labelKey: "replay_group_vehicle" },
    { key: "navigation", labelKey: "replay_group_navigation" },
    { key: "planning", labelKey: "replay_group_planning" },
    { key: "perception", labelKey: "replay_group_perception" },
    { key: "location", labelKey: "replay_group_location" },
    { key: "sensors", labelKey: "replay_group_sensors" },
    { key: "camera", labelKey: "replay_group_camera" },
    { key: "device", labelKey: "replay_group_device" },
    { key: "connectivity", labelKey: "replay_group_connectivity" },
    { key: "diagnostics", labelKey: "replay_group_diagnostics" },
    { key: "raw", labelKey: "replay_group_raw" },
  ]);
  const SERVICE_GROUPS = Object.freeze({
    carState: "vehicle",
    controlsState: "control",
    deviceState: "device",
    peripheralState: "device",
    carrotMan: "navigation",
    selfdriveState: "control",
    gpsLocationExternal: "location",
    longitudinalPlan: "planning",
    modelV2: "perception",
    liveCalibration: "sensors",
    roadCameraState: "camera",
    lateralPlan: "planning",
    radarState: "perception",
    carControl: "control",
    liveDelay: "diagnostics",
    liveTorqueParameters: "sensors",
    liveParameters: "vehicle",
  });

  function scheduleWork(callback) {
    if (typeof window.requestIdleCallback === "function") {
      window.requestIdleCallback(callback, { timeout: 80 });
    } else {
      window.setTimeout(() => callback({ timeRemaining: () => 8 }), 0);
    }
  }

  function normalizedGroup(group) {
    const key = String(group || "");
    return GROUPS.some((entry) => entry.key === key) ? key : "raw";
  }

  function clientRawSource(manifest) {
    const rlog = manifest?.rlog;
    if (!rlog || typeof rlog !== "object") return null;
    const url = String(rlog.url || "");
    const compression = String(rlog.compression || "");
    if (!url || !["zstd", "bzip2", "none"].includes(compression)) return null;
    return {
      url,
      compression,
      expectedSegment: Number(manifest?.segmentIndex) || 0,
      baseMonoTime: String(manifest?.rawFirstMonoTimeNanos || ""),
    };
  }

  function canQueryClientRaw(manifest) {
    return typeof Worker === "function" && Boolean(clientRawSource(manifest));
  }

  function runClientWorker(type, options = {}) {
    const source = clientRawSource(options.manifest);
    if (!source || typeof Worker !== "function") return Promise.resolve(null);
    return new Promise((resolve, reject) => {
      const worker = new Worker(ADVANCED_WORKER_URL);
      const signal = options.signal;
      let settled = false;
      const finish = (callback, value) => {
        if (settled) return;
        settled = true;
        signal?.removeEventListener?.("abort", onAbort);
        worker.terminate();
        callback(value);
      };
      const onAbort = () => {
        try { worker.postMessage({ type: "abort" }); } catch {}
        finish(reject, new DOMException("Aborted", "AbortError"));
      };
      worker.onmessage = (event) => {
        const payload = event?.data || {};
        if (payload.type === "progress") {
          const loaded = Math.max(0, Number(payload.loaded) || 0);
          const total = Math.max(0, Number(payload.total) || 0);
          options.onProgress?.(total > 0 ? Math.min(1, loaded / total) : 0);
        } else if (payload.type === "complete") {
          finish(resolve, payload.result || null);
        } else if (payload.type === "error") {
          finish(reject, new Error(String(payload.error || "Client field processing failed")));
        }
      };
      worker.onerror = (event) => finish(reject, new Error(event?.message || "Client field worker failed"));
      if (signal?.aborted) return onAbort();
      signal?.addEventListener?.("abort", onAbort, { once: true });
      worker.postMessage({
        type,
        ...source,
        service: String(options.service || ""),
        field: options.field || null,
        timeMs: Math.max(0, Number(options.timeMs) || 0),
        maxPoints: Math.max(50, Math.min(3000, Number(options.maxPoints) || POLICY.rawGraphPointLimit)),
      });
    });
  }

  function queryClientTrack(options = {}) {
    if (!options.service || !options.field?.path) return Promise.resolve(null);
    return runClientWorker("query", options);
  }

  function queryClientSnapshot(options = {}) {
    if (!options.service) return Promise.resolve(null);
    return runClientWorker("snapshot", options);
  }

  function catalog(availableServices) {
    const filtered = availableServices instanceof Set || Array.isArray(availableServices);
    const available = availableServices instanceof Set ? availableServices : new Set(availableServices || []);
    const source = window.CarrotVisionCompact?.catalog?.() || [];
    return source
      .filter((entry) => !filtered || available.has(entry.service))
      .map((entry) => ({ ...entry, group: normalizedGroup(entry.group || SERVICE_GROUPS[entry.service]) }));
  }

  function valueAtPath(object, path) {
    let value = object;
    for (const key of String(path || "").split(".")) {
      if (!key || value == null || typeof value !== "object") return undefined;
      value = value[key];
    }
    return value;
  }

  function normalizedValue(value, type) {
    if (typeof value === "number" && Number.isFinite(value)) return { kind: "number", value };
    if (typeof value === "boolean") return { kind: "state", value };
    if (typeof value === "string") return { kind: "state", value };
    if (Array.isArray(value)) {
      const numeric = value.filter((item) => typeof item === "number" && Number.isFinite(item));
      const average = numeric.length ? numeric.reduce((sum, item) => sum + item, 0) / numeric.length : null;
      return {
        kind: "summary",
        value: { type, length: value.length, average },
      };
    }
    if (value && typeof value === "object") {
      return {
        kind: "summary",
        value: { type, keys: Object.keys(value).length },
      };
    }
    return null;
  }

  function valueSignature(value) {
    if (value == null) return "null";
    if (typeof value === "object") return JSON.stringify(value);
    return `${typeof value}:${String(value)}`;
  }

  function buildSummarySeries(options = {}) {
    const records = Array.isArray(options.records) ? options.records : [];
    const decodeRecord = typeof options.decodeRecord === "function" ? options.decodeRecord : () => [];
    const shouldCancel = typeof options.shouldCancel === "function" ? options.shouldCancel : () => false;
    const onProgress = typeof options.onProgress === "function" ? options.onProgress : null;
    const series = { speed: [], steer: [], accel: [], lead: [] };
    const lastSampleAt = { speed: -Infinity, steer: -Infinity, accel: -Infinity, lead: -Infinity };
    let index = 0;

    function push(name, timeMs, value) {
      const numeric = Number(value);
      if (!Number.isFinite(numeric) || timeMs - lastSampleAt[name] < POLICY.numericSampleIntervalMs) return;
      series[name].push({ timeMs, value: numeric });
      lastSampleAt[name] = timeMs;
    }

    return new Promise((resolve) => {
      const step = (deadline) => {
        if (shouldCancel()) return resolve(null);
        const end = Math.min(records.length, index + POLICY.chunkSize);
        const start = index;
        while (index < end && (index - start < 80 || deadline.timeRemaining() > 1)) {
          const record = records[index];
          const timeMs = Number(record?.timeMs || 0);
          try {
            for (const frame of decodeRecord(record)) {
              if (frame?.service === "carState") {
                push("speed", timeMs, Number(frame.decoded?.vEgo) * 3.6);
                push("steer", timeMs, frame.decoded?.steeringAngleDeg);
                push("accel", timeMs, frame.decoded?.aEgo);
              } else if (frame?.service === "radarState" && frame.decoded?.leadOne?.status) {
                push("lead", timeMs, frame.decoded.leadOne.dRel);
              }
            }
          } catch {}
          index += 1;
        }
        onProgress?.(records.length ? index / records.length : 1);
        if (index < records.length) return scheduleWork(step);
        resolve(series);
      };
      scheduleWork(step);
    });
  }

  function buildTrack(options = {}) {
    const records = Array.isArray(options.records) ? options.records : [];
    const decodeRecord = typeof options.decodeRecord === "function" ? options.decodeRecord : () => [];
    const service = String(options.service || "");
    const field = options.field || {};
    const path = String(field.path || "");
    const shouldCancel = typeof options.shouldCancel === "function" ? options.shouldCancel : () => false;
    const onProgress = typeof options.onProgress === "function" ? options.onProgress : null;
    const samples = [];
    let kind = "state";
    let index = 0;
    let lastSampleTimeMs = -Infinity;
    let lastSignature = "";
    let count = 0;
    let sum = 0;
    let min = Infinity;
    let max = -Infinity;

    return new Promise((resolve) => {
      const step = (deadline) => {
        if (shouldCancel()) return resolve(null);
        const end = Math.min(records.length, index + POLICY.chunkSize);
        const start = index;
        while (index < end && (index - start < 80 || deadline.timeRemaining() > 1)) {
          const record = records[index];
          const timeMs = Number(record?.timeMs || 0);
          try {
            for (const frame of decodeRecord(record)) {
              if (frame?.service !== service) continue;
              const normalized = normalizedValue(valueAtPath(frame.decoded, path), field.type);
              if (!normalized) continue;
              kind = normalized.kind;
              const signature = valueSignature(normalized.value);
              const numericReady = kind === "number" && timeMs - lastSampleTimeMs >= POLICY.numericSampleIntervalMs;
              const stateReady = kind !== "number" && signature !== lastSignature;
              if ((numericReady || stateReady) && samples.length < POLICY.maxStateSamples) {
                samples.push({ timeMs, value: normalized.value });
                lastSampleTimeMs = timeMs;
                lastSignature = signature;
              }
              if (kind === "number") {
                count += 1;
                sum += normalized.value;
                min = Math.min(min, normalized.value);
                max = Math.max(max, normalized.value);
              }
            }
          } catch {}
          index += 1;
        }
        onProgress?.(records.length ? index / records.length : 1);
        if (index < records.length) return scheduleWork(step);
        resolve({
          service,
          field,
          kind,
          renderer: String(field.recommendedView || (kind === "number" ? "graph" : "state-band")),
          statisticsPolicy: String(field.statisticsPolicy || ""),
          samples,
          stats: kind === "number" && count
            ? { count, min, max, average: sum / count }
            : { count: samples.length },
        });
      };
      scheduleWork(step);
    });
  }

  function sampleAt(samples, timeMs) {
    if (!samples.length) return null;
    let low = 0;
    let high = samples.length - 1;
    while (low < high) {
      const middle = Math.ceil((low + high) / 2);
      if (samples[middle].timeMs <= timeMs) low = middle;
      else high = middle - 1;
    }
    return samples[low];
  }

  return {
    POLICY,
    GROUPS,
    catalog,
    canQueryClientRaw,
    queryClientTrack,
    queryClientSnapshot,
    buildSummarySeries,
    buildTrack,
    sampleAt,
  };
})();
