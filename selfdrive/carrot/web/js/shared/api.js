"use strict";

/* ── Params helpers ───────────────────────────────────── */
async function bulkGet(names) {
  const q = encodeURIComponent(names.join(","));
  const j = await getJson("/api/params_bulk?names=" + q);
  return j.values || {};
}

async function setParam(name, value) {
  const j = await postJson("/api/param_set", { name, value });
  window.dispatchEvent(new CustomEvent("carrot:paramchange", {
    detail: { name, value: j.value ?? value },
  }));
  return true;
}


/* ── Generic fetch wrappers ─────────────────────────── */
function buildApiError(response, payload) {
  const friendly = (typeof friendlyError === "function") ? friendlyError(payload) : null;
  const message = friendly || payload?.error || payload?.out || `HTTP ${response.status}`;
  const error = new Error(message);
  error.status = response.status;
  error.payload = payload || null;
  error.errorCode = payload?.error_code || null;
  error.errorDetail = payload?.error_detail || null;
  return error;
}

async function readJsonResponse(response) {
  const text = await response.text();
  if (!text) return {};
  try {
    return JSON.parse(text);
  } catch (e) {
    return {
      ok: false,
      error: text.slice(0, 500) || `HTTP ${response.status}`,
      error_code: "INVALID_JSON_RESPONSE",
    };
  }
}

async function requestJson(url, options = {}) {
  const response = await fetch(url, options);
  const payload = await readJsonResponse(response);
  if (!response.ok || payload?.ok === false) {
    throw buildApiError(response, payload);
  }
  return payload;
}

async function postJson(url, bodyObj) {
  return requestJson(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(bodyObj || {})
  });
}

async function getJson(url) {
  return requestJson(url);
}

function waitMs(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}
