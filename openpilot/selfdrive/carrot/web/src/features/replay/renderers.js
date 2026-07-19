"use strict";

window.CarrotReplayRendererRegistry = window.CarrotReplayRendererRegistry || (() => {
  const DEFINITIONS = Object.freeze({
    graph: Object.freeze({ key: "graph", mode: "line", interactiveSeries: true }),
    "state-band": Object.freeze({ key: "state-band", mode: "bands", interactiveSeries: true }),
    "text-transitions": Object.freeze({ key: "text-transitions", mode: "bands", interactiveSeries: true }),
    "event-marker": Object.freeze({ key: "event-marker", mode: "markers", interactiveSeries: true }),
    "numeric-audit": Object.freeze({ key: "numeric-audit", mode: "summary", interactiveSeries: false }),
    "exact-values": Object.freeze({ key: "exact-values", mode: "summary", interactiveSeries: false }),
    "list-snapshot": Object.freeze({ key: "list-snapshot", mode: "snapshot", interactiveSeries: false }),
    "binary-metadata": Object.freeze({ key: "binary-metadata", mode: "snapshot", interactiveSeries: false }),
    raw: Object.freeze({ key: "raw", mode: "summary", interactiveSeries: false }),
  });

  function resolve(key, valueKind = "") {
    const requested = String(key || "");
    if (DEFINITIONS[requested]) return DEFINITIONS[requested];
    return valueKind === "number" ? DEFINITIONS.graph : DEFINITIONS["state-band"];
  }

  return { DEFINITIONS, resolve };
})();
