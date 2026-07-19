import { createFooterState } from "./footer_state.js";

const DEFAULT_CLEAR_GRACE_MS = 220;

function createOverlayStore(renderer, options = {}) {
  if (!renderer) return null;

  const clearGraceMs = Math.max(0, Number(options.clearGraceMs) || DEFAULT_CLEAR_GRACE_MS);
  const footerState = createFooterState(options) || null;
  const sequences = new Map();
  const clearTimers = new Map();
  const pending = new Map();
  let pendingState = null;
  let statePending = false;
  let pendingFooterState = null;
  let footerPending = false;
  let generation = 0;

  function normalize(name, sequence) {
    return {
      name: String(name || ""),
      sequence: Math.max(0, Number(sequence) || 0),
    };
  }

  function isCurrent(name, sequence, expectedGeneration = generation) {
    return expectedGeneration === generation && sequences.get(name) === sequence;
  }

  function acceptSequence(name, sequence) {
    if (!name) return false;
    const previous = sequences.get(name);
    if (previous !== undefined && sequence < previous) return false;
    sequences.set(name, sequence);
    return true;
  }

  function cancelClear(name) {
    const timer = clearTimers.get(name);
    if (!timer) return;
    globalThis.clearTimeout(timer);
    clearTimers.delete(name);
  }

  function cancelClears() {
    for (const timer of clearTimers.values()) globalThis.clearTimeout(timer);
    clearTimers.clear();
  }

  function discardPending(name) {
    const previous = pending.get(name);
    if (previous?.source) previous.source.close?.();
    pending.delete(name);
  }

  function present(nameValue, source, sequenceValue = 0, sourceTimestampMillis = 0) {
    const { name, sequence } = normalize(nameValue, sequenceValue);
    if (!source || !acceptSequence(name, sequence)) {
      source?.close?.();
      return false;
    }
    cancelClear(name);
    discardPending(name);
    pending.set(name, {
      name,
      source,
      sequence,
      sourceTimestampMillis: Math.max(0, Number(sourceTimestampMillis) || 0),
    });
    return true;
  }

  function clear(nameValue, sequenceValue = 0, sourceTimestampMillis = 0) {
    const { name, sequence } = normalize(nameValue, sequenceValue);
    if (!acceptSequence(name, sequence)) return false;
    cancelClear(name);
    const expectedGeneration = generation;
    const timer = globalThis.setTimeout(() => {
      clearTimers.delete(name);
      if (!isCurrent(name, sequence, expectedGeneration)) return;
      discardPending(name);
      pending.set(name, {
        name,
        clear: true,
        sequence,
        sourceTimestampMillis: Math.max(0, Number(sourceTimestampMillis) || 0),
      });
    }, clearGraceMs);
    clearTimers.set(name, timer);
    return true;
  }

  async function decode(message) {
    if (!(message?.buffer instanceof ArrayBuffer)) return false;
    const { name, sequence } = normalize(message.name, message.sequence);
    if (!acceptSequence(name, sequence)) return false;
    const expectedGeneration = generation;
    const blob = new Blob([message.buffer], { type: String(message.mime || "image/png") });

    try {
      if (typeof globalThis.createImageBitmap === "function") {
        const bitmap = await globalThis.createImageBitmap(blob);
        if (!isCurrent(name, sequence, expectedGeneration)) {
          bitmap.close?.();
          return false;
        }
        return present(name, bitmap, sequence, message.sourceTimestampMillis);
      }

      if (typeof globalThis.Image !== "function" || !globalThis.URL?.createObjectURL) return false;
      const url = globalThis.URL.createObjectURL(blob);
      return await new Promise((resolve) => {
        const image = new globalThis.Image();
        image.onload = () => {
          globalThis.URL.revokeObjectURL(url);
          if (!isCurrent(name, sequence, expectedGeneration)) {
            resolve(false);
            return;
          }
          resolve(present(name, image, sequence, message.sourceTimestampMillis));
        };
        image.onerror = () => {
          globalThis.URL.revokeObjectURL(url);
          resolve(false);
        };
        image.src = url;
      });
    } catch (_) {
      return false;
    }
  }

  function setState(state) {
    pendingState = state && typeof state === "object" ? state : null;
    statePending = true;
    pendingFooterState = footerState?.update(pendingState) || { route: null, vehicle: null };
    footerPending = true;
  }

  function flush() {
    if (!pending.size && !statePending && !footerPending) return false;
    const updates = Array.from(pending.values());
    pending.clear();
    if (updates.length) renderer.applyBatch(updates);
    if (statePending) {
      renderer.setState(pendingState);
      pendingState = null;
      statePending = false;
    }
    if (footerPending) {
      renderer.setFooterState?.(pendingFooterState);
      pendingFooterState = null;
      footerPending = false;
    }
    renderer.renderNow();
    return true;
  }

  function clearFooterState() {
    footerState?.reset();
    pendingFooterState = null;
    footerPending = false;
  }

  function clearPending() {
    for (const update of pending.values()) update.source?.close?.();
    pending.clear();
  }

  function resetImages() {
    generation += 1;
    cancelClears();
    clearPending();
    sequences.clear();
    renderer.clearImages();
    renderer.renderNow();
  }

  function reset() {
    generation += 1;
    cancelClears();
    clearPending();
    sequences.clear();
    pendingState = null;
    statePending = false;
    clearFooterState();
    renderer.reset();
    renderer.renderNow();
  }

  return Object.freeze({ present, clear, decode, setState, flush, resetImages, reset });
}

const overlayStore = Object.freeze({ create: createOverlayStore });

function installCarrotNaviOverlayStoreGlobal(target = globalThis) {
  target.CarrotNaviOverlayStore = overlayStore;
  return overlayStore;
}

export {
  createOverlayStore,
  overlayStore,
  installCarrotNaviOverlayStoreGlobal,
};
