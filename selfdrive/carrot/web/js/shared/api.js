"use strict";

/* ── Params helpers ───────────────────────────────────── */
async function bulkGet(names) {
  const q = encodeURIComponent(names.join(","));
  const r = await fetch("/api/params_bulk?names=" + q);
  const j = await r.json();
  if (!j.ok) throw new Error(j.error || "bulk failed");
  return j.values || {};
}

async function setParam(name, value) {
  const r = await fetch("/api/param_set", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name, value })
  });
  const j = await r.json();
  if (!j.ok) throw new Error(j.error || "set failed");
  window.dispatchEvent(new CustomEvent("carrot:paramchange", {
    detail: { name, value: j.value ?? value },
  }));
  return true;
}


/* ── Generic fetch wrappers ─────────────────────────── */
async function postJson(url, bodyObj) {
  const r = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(bodyObj || {})
  });
  return r.json();
}

async function getJson(url) {
  const r = await fetch(url);
  return r.json();
}

function waitMs(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}
