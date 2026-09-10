const API_ROOT = "/xiaoge/api";

export const ONNX_VISION_STATUS_ENDPOINT = `${API_ROOT}/status`;
export const ONNX_VISION_CONFIG_ENDPOINT = `${API_ROOT}/config`;
export const ONNX_VISION_SETTINGS_ENDPOINT = `${API_ROOT}/settings`;
export const ONNX_VISION_SNAPSHOT_ENDPOINT = `${API_ROOT}/snapshot`;

function requestError(response, payload) {
  const error = new Error(String(payload?.error || response?.statusText || "ONNX service is unavailable"));
  error.code = String(payload?.code || "ONNX_SERVICE_UNAVAILABLE");
  error.status = Number(response?.status || 0);
  return error;
}

function fetchClient(target) {
  if (typeof target?.fetch !== "function") throw new Error("ONNX client is unavailable");
  return target.fetch.bind(target);
}

async function requestJson(target, endpoint, options = {}) {
  const request = { cache: "no-store", signal: options.signal, method: options.method || "GET" };
  if (options.body !== undefined) {
    request.headers = { "Content-Type": "application/json" };
    request.body = JSON.stringify(options.body);
  }
  const response = await fetchClient(target)(endpoint, request);
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) throw requestError(response, payload);
  return payload && typeof payload === "object" ? payload : {};
}

/** All browser access stays on the Carrot Web origin; port 8082 is never exposed here. */
export function loadOnnxVisionStatus(target = globalThis, options = {}) {
  return requestJson(target, ONNX_VISION_STATUS_ENDPOINT, options);
}

export function loadOnnxVisionConfig(target = globalThis, options = {}) {
  return requestJson(target, ONNX_VISION_CONFIG_ENDPOINT, options);
}

export function saveOnnxVisionConfig(config, target = globalThis, options = {}) {
  return requestJson(target, ONNX_VISION_CONFIG_ENDPOINT, { ...options, method: "POST", body: config });
}

export function restoreOnnxVisionConfig(target = globalThis, options = {}) {
  return requestJson(target, ONNX_VISION_CONFIG_ENDPOINT, { ...options, method: "DELETE" });
}

export function updateOnnxVisionSettings(settings, target = globalThis, options = {}) {
  return requestJson(target, ONNX_VISION_SETTINGS_ENDPOINT, { ...options, method: "POST", body: settings });
}

export async function loadOnnxVisionSnapshot(stream, target = globalThis, options = {}) {
  const normalizedStream = stream === "road" ? "road" : "wide";
  const response = await fetchClient(target)(
    `${ONNX_VISION_SNAPSHOT_ENDPOINT}?stream=${encodeURIComponent(normalizedStream)}`,
    { cache: "no-store", signal: options.signal },
  );
  if (!response.ok) {
    const payload = await response.json().catch(() => ({}));
    throw requestError(response, payload);
  }
  const contentType = String(response.headers?.get?.("Content-Type") || "").toLowerCase();
  if (!contentType.startsWith("image/jpeg")) {
    throw new Error("ONNX camera returned an invalid image");
  }
  return response.blob();
}
