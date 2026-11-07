import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createSettingValueCache } from '../src/features/settings/value_cache.js';

test('voice settings reject stale/unknown work and never send the local grant to the relay', async () => {
  const stored = new Map([['carrot-assistant-settings-v1', 'local-secret']]);
  globalThis.localStorage = { getItem: key => stored.get(key), setItem: (key, value) => stored.set(key, value), removeItem: key => stored.delete(key) };
  const calls = [];
  const cache = createSettingValueCache();
  cache.primeGroup('display', { ShowCustomBrightness: 50 });
  globalThis.CarrotSettingsRuntime = { values: cache };
  let rendered = 50;
  globalThis.applyRestoredSettingValuesToRenderedItems = values => { rendered = values.ShowCustomBrightness; };
  globalThis.fetch = async (url, options) => {
    calls.push({ url, options });
    return { ok: true, json: async () => ({ ok: true, status: 'applied', name: 'ShowCustomBrightness', previous: 50, current: 55 }) };
  };
  globalThis.openAssistantSetting = async () => { throw new Error('Navigation unavailable'); };
  const module = await import('../src/features/tools/assistant_settings.js');
  const command = { id: 'a'.repeat(43), operation: 'commit', proposal_id: 'p'.repeat(43), expires_at: Date.now()+20000 };
  assert.equal(module.validSettingCommand({ ...command, operation: 'shell' }), false);
  assert.equal(module.validSettingCommand({ ...command, expires_at: Date.now()-1 }), false);
  assert.equal(module.validSettingCommand({ ...command, expires_at: Date.now()+60000 }), false);
  const result = await module.executeSettingCommand(command);
  assert.equal(result.result.current, 55);
  assert.equal(result.result.status, 'applied');
  assert.equal(rendered, 55);
  assert.equal(cache.peekValues(['ShowCustomBrightness']).values.ShowCustomBrightness, 55);
  // A second voice change on the already-open row must replace the prior value.
  globalThis.fetch = async (url, options) => {
    calls.push({ url, options });
    return { ok: true, json: async () => ({ ok: true, status: 'applied', name: 'ShowCustomBrightness', previous: 55, current: 50 }) };
  };
  await module.executeSettingCommand(command);
  assert.equal(calls.length, 1);
  assert.equal(calls[0].url, '/api/assistant/settings/execute');
  assert.equal(calls[0].options.headers.Authorization, 'Bearer local-secret');
  assert.equal(JSON.stringify(result).includes('local-secret'), false);
  await module.executeSettingCommand({ ...command, id: 'c'.repeat(43) });
  assert.equal(rendered, 50);
  assert.equal(cache.peekValues(['ShowCustomBrightness']).values.ShowCustomBrightness, 50);
  await module.revokeSettingAccess();
  assert.equal(calls[2].url, '/api/assistant/settings/disable');
  assert.equal((await module.executeSettingCommand({ ...command, id: 'b'.repeat(43) })).result.status, 'failed');
  assert.equal(calls.length, 3);
  // Same-origin permission restored elsewhere after this module was loaded.
  // A new request uses it; the earlier failed request must never replay.
  stored.set('carrot-assistant-settings-v1', 'restored-secret');
  assert.equal((await module.executeSettingCommand({ ...command, id: 'b'.repeat(43) })).result.status, 'failed');
  assert.equal(calls.length, 3);
  await module.executeSettingCommand({ ...command, id: 'd'.repeat(43) });
  assert.equal(calls[3].options.headers.Authorization, 'Bearer restored-secret');
  stored.delete('carrot-assistant-settings-v1');
  assert.equal((await module.executeSettingCommand({ ...command, id: 'e'.repeat(43) })).result.status, 'failed');
  assert.equal(calls.length, 4);
  // Read-only code fields do not block commands; reads never interrupt editing.
  assert.equal(module.settingCommandBlocked(command, { tagName: 'INPUT', readOnly: true }), false);
  assert.equal(module.settingCommandBlocked(command, { tagName: 'INPUT', readOnly: false }), true);
  assert.equal(module.settingCommandBlocked({ operation: 'read' }, { tagName: 'INPUT' }), false);
});
