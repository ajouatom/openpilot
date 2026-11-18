import test from 'node:test';
import assert from 'node:assert/strict';

test('Tools commands are bounded, hidden pages do not execute, local credentials never leave the origin, retries do not re-run', async () => {
  const stored = new Map([['carrot-assistant-tools-v1', 'private-local-grant']]);
  globalThis.localStorage = { getItem: key => stored.get(key), setItem: (key, value) => stored.set(key, value), removeItem: key => stored.delete(key) };
  globalThis.document = { hidden: true };
  const calls = [];
  globalThis.fetch = async (url, options) => {
    calls.push({ url, options });
    return { ok: true, json: async () => ({ ok: true, status: 'running', action: 'git_pull', job_id: 'a'.repeat(12) }) };
  };
  globalThis.CarrotToolsRuntime = { logNotice() { throw Error('Unrelated UI failure'); } };
  const module = await import('../src/features/tools/assistant_tools.js');
  const command = { id: 'c'.repeat(43), operation: 'commit', proposal_id: 'p'.repeat(43), expires_at: Date.now() + 20000 };
  assert.equal(module.validToolsCommand({ ...command, operation: 'shell' }), false);
  assert.equal(module.validToolsCommand({ ...command, expires_at: Date.now() - 1 }), false);
  assert.equal(module.validToolsCommand({ ...command, expires_at: Date.now() + 60000 }), false);
  assert.equal(await module.executeToolsCommand(command), null);
  document.hidden = false;
  const result = await module.executeToolsCommand(command);
  assert.equal(result.result.status, 'running');
  assert.deepEqual(await module.executeToolsCommand(command), result);
  assert.equal(calls.length, 1);
  assert.equal(calls[0].url, '/api/assistant/tools/execute');
  assert.equal(calls[0].options.headers.Authorization, 'Bearer private-local-grant');
  assert.equal(JSON.stringify(result).includes('private-local-grant'), false);
  await module.revokeToolsAccess();
  assert.equal((await module.executeToolsCommand({ ...command, id: 'd'.repeat(43) })).result.status, 'failed');
  assert.equal(calls.length, 2);
});

test('manual-only panels do not synthesize destructive clicks and an unrelated open dialog is preserved', async () => {
  const module = await import('../src/features/tools/assistant_tools.js');
  let clicks = 0, existing = false;
  const button = { hidden: false, disabled: false, closest: () => null, scrollIntoView() {}, focus() {}, click() { clicks++; } };
  globalThis.document = { hidden: false, querySelector: () => existing ? {} : null,
    getElementById: id => id === 'pageTools' ? { hidden: false } : button };
  globalThis.showPage = () => {};
  for (const action of ['git_reset_repo', 'restore_settings', 'copy_settings', 'qr_restore_settings']) {
    assert.equal(await module.openToolPanel(action), 'needs_input');
  }
  assert.equal(clicks, 0);
  assert.equal(await module.openToolPanel('tools'), 'opened');
  existing = true;
  await assert.rejects(module.openToolPanel('bluetooth'), /Close the current dialog/);
  assert.equal(clicks, 0);
});

test('Naver Map launches only for the Android wrapper and reports the matching native result', async () => {
  globalThis.localStorage = { getItem: () => 'local-grant' };
  globalThis.document = { hidden: false };
  globalThis.fetch = async () => ({ ok: true, json: async () => ({ ok: true, status: 'open_requested', action: 'naver_map' }) });
  const module = await import('../src/features/tools/assistant_tools.js?phone');
  const command = id => ({ id: id.repeat(43), operation: 'open', action: 'naver_map', expires_at: Date.now() + 20000 });
  const browser = await module.executeToolsCommand(command('a'));
  assert.equal(browser.result.status, 'failed');
  Object.defineProperty(globalThis, 'navigator', { configurable: true, value: { userAgent: 'CarrotAssistPhone/1' } });
  globalThis.window = new EventTarget();
  let launches = 0, outcome = 'requested';
  window.location = { assign(value) {
    launches++;
    const url = new URL(value);
    assert.equal(url.protocol, 'carrotassist:');
    assert.equal(url.hostname, 'phone');
    const id = url.pathname.split('/')[2];
    for (const key of ['unrelated', id]) window.dispatchEvent(new CustomEvent('carrot-phone-result', { detail: { id: key, status: outcome } }));
  } };
  const request = command('b');
  const result = await module.executeToolsCommand(request);
  assert.equal(result.result.status, 'requested');
  assert.deepEqual(await module.executeToolsCommand(request), result);
  assert.equal(launches, 1);
  outcome = 'not_installed';
  assert.match((await module.executeToolsCommand(command('c'))).result.error, /not installed/);
});


test('model panel opens the model dialog through the compact change button', async () => {
  const module = await import('../src/features/tools/assistant_tools.js');
  let open = false;
  const button = { hidden: false, closest: () => null, scrollIntoView() {}, focus() {}, click() { open = true; } };
  globalThis.document = { hidden: false, querySelector: () => open ? {} : null,
    getElementById: id => id === 'pageTools' ? { hidden: false } : id === 'btnChangeModel' ? button : null };
  globalThis.showPage = () => {};
  assert.equal(await module.openToolPanel('egpu'), 'opened');
  assert.equal(open, true);
});
