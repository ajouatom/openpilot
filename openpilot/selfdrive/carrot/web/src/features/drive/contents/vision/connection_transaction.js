"use strict";

export const VISION_CONNECTION_ABORT_CODE = "vision-connection-stale";

export class VisionConnectionAbortError extends Error {
  constructor(message = "Carrot Vision connection attempt is no longer current") {
    super(message);
    this.name = "AbortError";
    this.code = VISION_CONNECTION_ABORT_CODE;
  }
}

function createAbortController(factory) {
  if (typeof factory === "function") return factory();
  if (typeof AbortController === "function") return new AbortController();
  return null;
}

function abortController(controller, reason) {
  if (!controller || controller.signal?.aborted) return;
  try {
    controller.abort(new VisionConnectionAbortError(reason));
  } catch (_) {
    try { controller.abort(); } catch (_) {}
  }
}

export function createVisionConnectionTransactionManager(options = {}) {
  const abortControllerFactory = options.abortControllerFactory;
  const now = typeof options.now === "function" ? options.now : () => Date.now();
  let generation = 0;
  let current = null;
  let currentController = null;
  let phase = "idle";
  let lastCancelReason = "";

  function isCurrent(transaction) {
    return Boolean(
      transaction
      && current === transaction
      && transaction.generation === generation
      && transaction.signal?.aborted !== true
    );
  }

  function assertCurrent(transaction, message = "") {
    if (!isCurrent(transaction)) {
      throw new VisionConnectionAbortError(message || "Carrot Vision connection attempt was superseded");
    }
    return transaction;
  }

  function cancel(reason = "connection cancelled", expected = null) {
    if (expected && current !== expected) return false;
    const controller = currentController;
    const hadCurrent = Boolean(current);
    generation += 1;
    current = null;
    currentController = null;
    phase = "idle";
    lastCancelReason = String(reason || "connection cancelled");
    abortController(controller, lastCancelReason);
    return hadCurrent;
  }

  function begin(metadata = {}) {
    cancel(metadata.supersedeReason || "connection superseded");
    generation += 1;
    const controller = createAbortController(abortControllerFactory);
    const transaction = Object.freeze({
      generation,
      attemptId: String(metadata.attemptId || ""),
      signal: controller?.signal || null,
      startedAtMs: now(),
    });
    current = transaction;
    currentController = controller;
    phase = "connecting";
    lastCancelReason = "";
    return transaction;
  }

  function markActive(transaction) {
    assertCurrent(transaction);
    phase = "active";
    return true;
  }

  function matches(transaction, identity = {}) {
    if (!isCurrent(transaction)) return false;
    if (identity.generation != null && Number(identity.generation) !== transaction.generation) return false;
    if (identity.attemptId != null && String(identity.attemptId) !== transaction.attemptId) return false;
    return true;
  }

  function runIfCurrent(transaction, callback) {
    if (!isCurrent(transaction) || typeof callback !== "function") return false;
    callback(transaction);
    return true;
  }

  function snapshot() {
    return Object.freeze({
      generation,
      phase,
      active: Boolean(current),
      attemptId: current?.attemptId || "",
      startedAtMs: current?.startedAtMs || 0,
      aborted: Boolean(current?.signal?.aborted),
      lastCancelReason,
    });
  }

  return Object.freeze({
    begin,
    cancel,
    assertCurrent,
    isCurrent,
    markActive,
    matches,
    runIfCurrent,
    current: () => current,
    snapshot,
  });
}

const CONNECTION_TRANSACTION_API = Object.freeze({
  abortCode: VISION_CONNECTION_ABORT_CODE,
  AbortError: VisionConnectionAbortError,
  create: createVisionConnectionTransactionManager,
});

export function installDriveVisionConnectionTransactionFacade(target = globalThis) {
  if (!target) return CONNECTION_TRANSACTION_API;
  if (target.DriveVisionConnectionTransactions) return target.DriveVisionConnectionTransactions;
  target.DriveVisionConnectionTransactions = CONNECTION_TRANSACTION_API;
  return target.DriveVisionConnectionTransactions;
}

export const DriveVisionConnectionTransactions = CONNECTION_TRANSACTION_API;
