import assert from "node:assert/strict";
import test from "node:test";
import { createDialogController } from "../src/ui/components/dialog/dialog.js";

function harness(pageState = { page: "setting", screen: "items", group: "driving" }) {
  const timers = new Map();
  const frames = new Map();
  const listeners = [];
  const traversals = [];
  const routed = [];
  const entries = [{ page: "carrot" }, pageState];
  let index = 1;
  let serial = 0;
  const history = {
    get state() { return entries[index]; },
    pushState(state) { entries.splice(++index, Infinity, structuredClone(state)); },
    replaceState(state) { entries[index] = structuredClone(state); },
    back() { traversals.push(-1); },
    forward() { traversals.push(1); },
  };
  const nodes = new Map();
  const makeNode = () => ({
    hidden: true, value: "", style: { removeProperty() {}, setProperty() {} },
    classList: { add() {}, remove() {}, toggle() {} },
    setAttribute() {}, removeAttribute() {}, focus() {}, select() {},
    querySelector() { return null; },
  });
  for (const id of ["appDialog", "appDialogBackdrop", "appDialogTitle", "appDialogBody",
    "appDialogConfirm", "appDialogCancel", "appDialogInput", "appDialogInputError"]) {
    nodes.set(id, makeNode());
  }
  const document = {
    getElementById: (id) => nodes.get(id),
    activeElement: makeNode(), body: makeNode(), addEventListener() {},
  };
  const target = {
    history,
    addEventListener(type, callback, capture) {
      if (type === "popstate") listeners.push({ callback, capture });
    },
  };
  const dialog = createDialogController({
    target, document,
    setTimeout: (callback) => { timers.set(++serial, callback); return serial; },
    requestAnimationFrame: (callback) => { frames.set(++serial, callback); return serial; },
    cancelAnimationFrame: (id) => frames.delete(id),
    createFocusTrap: () => ({ activate() {}, deactivate() {} }),
  });
  target.addEventListener("popstate", (event) => routed.push(event.state));
  function flushHistory() {
    while (traversals.length) {
      const destination = index + traversals.shift();
      if (destination < 0 || destination >= entries.length) continue;
      index = destination;
      let stopped = false;
      const event = { state: history.state, stopImmediatePropagation() { stopped = true; } };
      for (const listener of [...listeners].sort((a, b) => Number(Boolean(b.capture)) - Number(Boolean(a.capture)))) {
        listener.callback(event);
        if (stopped) break;
      }
    }
  }
  function finishAnimation() {
    for (const [id, callback] of timers) { timers.delete(id); callback(); }
  }
  return { dialog, history, nodes, entries, routed, target, flushHistory, finishAnimation,
    back() { history.back(); flushHistory(); finishAnimation(); } };
}

for (const [mode, expected] of [["alert", true], ["confirm", false], ["prompt", null], ["choice", null], ["form", null]]) {
  test(`mobile Back dismisses ${mode} without routing the underlying page`, async () => {
    const h = harness();
    const page = h.history.state;
    const completion = h.dialog.openAppDialog({ mode, message: "Example" });
    assert.equal(h.entries.length, 3);
    h.back();
    assert.equal(await completion, expected);
    assert.equal(h.nodes.get("appDialog").hidden, true);
    assert.deepEqual(h.history.state, page);
    assert.deepEqual(h.routed, []);
    h.back();
    assert.deepEqual(h.routed, [{ page: "carrot" }]);
  });
}

for (const action of ["confirmAppDialog", "cancelAppDialog"]) {
  test(`${action} removes the dialog entry without routing or adding an extra Back step`, async () => {
    const h = harness();
    const completion = h.dialog.appConfirm("Continue?");
    await h.dialog[action]();
    h.flushHistory();
    h.finishAnimation();
    assert.equal(await completion, action === "confirmAppDialog");
    assert.deepEqual(h.routed, []);
    h.back();
    assert.deepEqual(h.routed, [{ page: "carrot" }]);
  });
}

test("replacing a dialog reuses its Back entry", async () => {
  const h = harness();
  const first = h.dialog.appConfirm("First");
  const second = h.dialog.appPrompt("Second");
  h.finishAnimation();
  assert.equal(await first, false);
  assert.equal(h.nodes.get("appDialog").hidden, false);
  assert.equal(h.entries.length, 3);
  h.back();
  assert.equal(await second, null);
  assert.deepEqual(h.routed, []);
  h.back();
  assert.equal(h.routed[0].page, "carrot");
});

test("a dialog opened before asynchronous history cleanup still owns one Back step", async () => {
  const h = harness();
  const first = h.dialog.appConfirm("First");
  await h.dialog.confirmAppDialog();
  const second = h.dialog.appConfirm("Second");
  h.flushHistory();
  h.finishAnimation();
  assert.equal(await first, true);
  assert.equal(h.nodes.get("appDialog").hidden, false);
  assert.equal(h.entries.length, 3);
  assert.deepEqual(h.routed, []);
  h.back();
  assert.equal(await second, false);
  h.back();
  assert.equal(h.routed[0].page, "carrot");
});

test("a dialog over a search or tools panel consumes Back before panel handlers", async () => {
  for (const page of [{ page: "setting", search: true }, { page: "tools", toolsLogPanel: true }]) {
    const h = harness(page);
    let panelBacks = 0;
    h.target.addEventListener("popstate", () => { panelBacks++; }, true);
    const completion = h.dialog.appAlert("Details");
    h.back();
    await completion;
    assert.equal(panelBacks, 0);
    assert.deepEqual(h.history.state, page);
    h.back();
    assert.equal(panelBacks, 1);
  }
});

test("Back during form submission preserves the dialog and never submits twice", async () => {
  const h = harness();
  let finishSubmit;
  let submits = 0;
  const completion = h.dialog.appForm("Save", { onSubmit: () => {
    submits++;
    return new Promise((resolve) => { finishSubmit = resolve; });
  } });
  const submitted = h.dialog.confirmAppDialog();
  h.back();
  h.back();
  assert.equal(h.nodes.get("appDialog").hidden, false);
  assert.equal(submits, 1);
  assert.deepEqual(h.routed, []);
  finishSubmit();
  await submitted;
  h.flushHistory();
  h.finishAnimation();
  await completion;
  h.back();
  assert.equal(h.routed[0].page, "carrot");
});

test("progress Back requests cancellation once and guards the page until the operation closes", async () => {
  const h = harness();
  let finishCancel;
  let cancels = 0;
  const progress = h.dialog.openAppProgressDialog({ onCancel: () => {
    cancels++;
    return new Promise((resolve) => { finishCancel = resolve; });
  } });
  h.back();
  h.back();
  assert.equal(cancels, 1);
  assert.equal(h.nodes.get("appDialog").hidden, false);
  assert.deepEqual(h.routed, []);
  finishCancel();
  await Promise.resolve();
  progress.close();
  h.flushHistory();
  h.finishAnimation();
  await progress.completion;
  h.back();
  assert.equal(h.routed[0].page, "carrot");
});

test("Forward after dismissal does not reopen a confirmation", async () => {
  const h = harness();
  const completion = h.dialog.appConfirm("Run action?");
  h.back();
  assert.equal(await completion, false);
  h.history.forward();
  h.flushHistory();
  assert.equal(h.nodes.get("appDialog").hidden, true);
  assert.equal(h.history.state.appDialog, undefined);
});
