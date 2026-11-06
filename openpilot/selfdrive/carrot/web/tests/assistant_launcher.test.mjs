import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';
import * as policy from '../src/features/tools/assistant_policy.js';

const source = readFileSync(new URL('../src/features/tools/assistant.js', import.meta.url), 'utf8')
  .replace(/^import .*;\r?\n/gm, '').replace('export function openAssistantDialog', 'function openAssistantDialog');

function mount(connected) {
  const handlers = {}, launcher = { classList: { toggle() {} }, addEventListener(name, fn) { handlers[name] = fn; } };
  let dialogs = 0;
  const session = connected === null ? null : { browser_token: 'a'.repeat(43), expires_at: null, connected };
  const values = new Map([[policy.ASSISTANT_SESSION_KEY, JSON.stringify(session)]]);
  vm.runInNewContext(source, {
    ...policy, copyPairingMessage() {}, navigator: { userAgent: 'Android' },
    localStorage: { getItem: key => values.get(key) ?? null }, sessionStorage: { getItem: () => null },
    document: { readyState: 'complete', hidden: false, getElementById: id => id === 'assistantLauncher' ? launcher : null, addEventListener() {} },
    setTimeout: () => 1, clearTimeout() {}, appAlert: () => { dialogs++; },
  });
  return { launcher, click(shiftKey = false) {
    let prevented = false;
    handlers.click({ shiftKey, preventDefault() { prevented = true; } });
    return { prevented, dialogs };
  } };
}

test('paired Android click launches even without an optional saved conversation URL', () => {
  const ui = mount(true);
  assert.equal(ui.launcher.target, '_self');
  assert.match(ui.launcher.href, /^intent:\/\/chatgpt.com\//);
  assert.deepEqual(ui.click(), { prevented: false, dialogs: 0 });
  assert.deepEqual(ui.click(true), { prevented: true, dialogs: 1 });
});

test('unpaired and waiting browsers still open connection setup', () => {
  for (const connected of [false, null]) assert.deepEqual(mount(connected).click(), { prevented: true, dialogs: 1 });
});
