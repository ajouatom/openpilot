const PHASE_VALUES = ["idle", "connecting", "buffering", "live", "recovering", "busy"];

export const PHASES = Object.freeze(PHASE_VALUES);
const PHASE_SET = new Set(PHASES);

export function createLifecycle(options = {}) {
  const target = options.target || globalThis;
  const now = options.now || (() => target.performance.now());
  let phase = "idle";
  let reason = "";
  let changedAt = now();

  function snapshot() {
    return {
      phase,
      reason,
      changedAt,
      ageMs: Math.max(0, Math.round(now() - changedAt)),
    };
  }

  function transition(nextValue, nextReason = "") {
    const next = String(nextValue || "");
    if (!PHASE_SET.has(next)) throw new Error(`Invalid Carrot Navi phase: ${next}`);
    const normalizedReason = String(nextReason || "").slice(0, 128);
    if (phase === next && reason === normalizedReason) return false;
    const previous = phase;
    phase = next;
    reason = normalizedReason;
    changedAt = now();
    const detail = snapshot();
    options.onChange?.({ ...detail, previous });
    target.dispatchEvent?.(new target.CustomEvent("carrot:navigationphasechange", {
      detail: { ...detail, previous },
    }));
    return true;
  }

  return Object.freeze({ transition, snapshot });
}

export const CarrotNaviLifecycle = Object.freeze({ PHASES, create: createLifecycle });

export function installCarrotNaviLifecycleGlobal(target = globalThis) {
  target.CarrotNaviLifecycle = CarrotNaviLifecycle;
  return CarrotNaviLifecycle;
}
