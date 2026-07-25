import assert from "node:assert/strict";
import test from "node:test";

import {
  VISION_CONNECTION_ABORT_CODE,
  VisionConnectionAbortError,
  createVisionConnectionTransactionManager,
} from "../src/features/drive/contents/vision/index.js";

test("a newer connection aborts and invalidates the previous transaction", () => {
  const manager = createVisionConnectionTransactionManager();
  const first = manager.begin({ attemptId: "first" });
  const second = manager.begin({ attemptId: "second" });

  assert.equal(first.signal.aborted, true);
  assert.equal(manager.isCurrent(first), false);
  assert.equal(manager.isCurrent(second), true);
  assert.ok(second.generation > first.generation);
});

test("explicit cancellation invalidates an in-flight request", () => {
  const manager = createVisionConnectionTransactionManager();
  const transaction = manager.begin({ attemptId: "connect" });

  assert.equal(manager.cancel("page hidden"), true);
  assert.equal(transaction.signal.aborted, true);
  assert.equal(manager.isCurrent(transaction), false);
  assert.equal(manager.snapshot().lastCancelReason, "page hidden");
});

test("cancelling an old transaction cannot cancel its replacement", () => {
  const manager = createVisionConnectionTransactionManager();
  const first = manager.begin({ attemptId: "first" });
  const second = manager.begin({ attemptId: "second" });

  assert.equal(manager.cancel("late cleanup", first), false);
  assert.equal(manager.isCurrent(second), true);
  assert.equal(second.signal.aborted, false);
});

test("stale async continuations fail with an AbortError", async () => {
  const manager = createVisionConnectionTransactionManager();
  const first = manager.begin({ attemptId: "first" });
  manager.begin({ attemptId: "second" });

  await assert.rejects(
    Promise.resolve().then(() => manager.assertCurrent(first)),
    (error) => error instanceof VisionConnectionAbortError
      && error.name === "AbortError"
      && error.code === VISION_CONNECTION_ABORT_CODE,
  );
});

test("a delayed response from an older attempt cannot commit", async () => {
  const manager = createVisionConnectionTransactionManager();
  let resolveFirst;
  const firstResponse = new Promise((resolve) => {
    resolveFirst = resolve;
  });
  const committed = [];
  const first = manager.begin({ attemptId: "first" });
  const firstContinuation = firstResponse.then(() => {
    manager.assertCurrent(first);
    committed.push("first");
  });

  const second = manager.begin({ attemptId: "second" });
  manager.assertCurrent(second);
  committed.push("second");
  resolveFirst();

  await assert.rejects(firstContinuation, { name: "AbortError" });
  assert.deepEqual(committed, ["second"]);
});

test("the transaction signal cancels an in-flight request", async () => {
  const manager = createVisionConnectionTransactionManager();
  const transaction = manager.begin({ attemptId: "request" });
  const request = new Promise((resolve, reject) => {
    transaction.signal.addEventListener("abort", () => {
      reject(transaction.signal.reason || new VisionConnectionAbortError());
    }, { once: true });
  });

  manager.cancel("backgrounded");

  await assert.rejects(request, { name: "AbortError" });
});

test("generation and attempt identity must both match", () => {
  const manager = createVisionConnectionTransactionManager();
  const transaction = manager.begin({ attemptId: "attempt-7" });

  assert.equal(manager.matches(transaction, {
    generation: transaction.generation,
    attemptId: "attempt-7",
  }), true);
  assert.equal(manager.matches(transaction, {
    generation: transaction.generation,
    attemptId: "attempt-6",
  }), false);
  assert.equal(manager.matches(transaction, {
    generation: transaction.generation + 1,
    attemptId: "attempt-7",
  }), false);
});

test("late peer callbacks are ignored after stop or replacement", () => {
  const manager = createVisionConnectionTransactionManager();
  const calls = [];
  const first = manager.begin({ attemptId: "first" });
  manager.runIfCurrent(first, () => calls.push("first-live"));
  manager.cancel("user stop");
  manager.runIfCurrent(first, () => calls.push("first-late"));

  const second = manager.begin({ attemptId: "second" });
  manager.runIfCurrent(first, () => calls.push("first-after-replace"));
  manager.runIfCurrent(second, () => calls.push("second-live"));

  assert.deepEqual(calls, ["first-live", "second-live"]);
});

test("only the current transaction can become active", () => {
  const manager = createVisionConnectionTransactionManager();
  const first = manager.begin({ attemptId: "first" });
  const second = manager.begin({ attemptId: "second" });

  assert.throws(() => manager.markActive(first), { name: "AbortError" });
  assert.equal(manager.markActive(second), true);
  assert.equal(manager.snapshot().phase, "active");
});
