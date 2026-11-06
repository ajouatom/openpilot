import { test } from 'node:test';
import assert from 'node:assert/strict';

test('voice settings reject stale/unknown work and never send the local grant to the relay', async () => {
  const stored = new Map([['carrot-assistant-settings-v1', 'local-secret']]);
  globalThis.localStorage = { getItem: key => stored.get(key), setItem: (key, value) => stored.set(key, value), removeItem: key => stored.delete(key) };
  const calls = [];
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
  await module.executeSettingCommand(command);
  assert.equal(calls.length, 1);
  assert.equal(calls[0].url, '/api/assistant/settings/execute');
  assert.equal(calls[0].options.headers.Authorization, 'Bearer local-secret');
  assert.equal(JSON.stringify(result).includes('local-secret'), false);
  await module.revokeSettingAccess();
  assert.equal(calls[1].url, '/api/assistant/settings/disable');
  assert.equal((await module.executeSettingCommand({ ...command, id: 'b'.repeat(43) })).result.status, 'failed');
  assert.equal(calls.length, 2);
});
