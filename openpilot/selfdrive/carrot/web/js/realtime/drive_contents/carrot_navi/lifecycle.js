"use strict";

globalThis.CarrotNaviLifecycle = (() => {
  const PHASES = Object.freeze(["idle", "connecting", "buffering", "live", "recovering"]);
  const PHASE_SET = new Set(PHASES);

  function create(options = {}) {
    let phase = "idle";
    let reason = "";
    let changedAt = performance.now();

    function transition(nextValue, nextReason = "") {
      const next = String(nextValue || "");
      if (!PHASE_SET.has(next)) throw new Error(`Invalid Carrot Navi phase: ${next}`);
      const normalizedReason = String(nextReason || "").slice(0, 128);
      if (phase === next && reason === normalizedReason) return false;
      const previous = phase;
      phase = next;
      reason = normalizedReason;
      changedAt = performance.now();
      const detail = snapshot();
      options.onChange?.({ ...detail, previous });
      globalThis.dispatchEvent?.(new CustomEvent("carrot:navigationphasechange", { detail: { ...detail, previous } }));
      return true;
    }

    function snapshot() {
      return {
        phase,
        reason,
        changedAt,
        ageMs: Math.max(0, Math.round(performance.now() - changedAt)),
      };
    }

    return Object.freeze({ transition, snapshot });
  }

  return Object.freeze({ PHASES, create });
})();
