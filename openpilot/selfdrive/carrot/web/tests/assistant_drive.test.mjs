import { test } from 'node:test';
import assert from 'node:assert/strict';

test('drive commands require a visible web and local grant; credentials stay on the device', async () => {
  const stored = new Map([['carrot-assistant-drive-v1', 'device-only']]);
  globalThis.localStorage = { getItem: k => stored.get(k), removeItem: k => stored.delete(k) };
  globalThis.document = { hidden: false };
  const calls = [];
  globalThis.fetch = async (url, options) => {
    calls.push({ url, options });
    const command = JSON.parse(options.body).command;
    return { ok: true, json: async () => ({ ok: true, id: command?.id, status: 'waiting' }) };
  };
  const m = await import('../src/features/tools/assistant_drive.js');
  const command = { id: 'a'.repeat(43), action: 'lane_left', operation: 'execute', expires_at: Date.now() + 4000 };
  assert.equal(await m.executeDriveCommand({ ...command, expires_at: Date.now() - 1 }), null);
  assert.equal(await m.executeDriveCommand({ ...command, expires_at: Date.now() + 10000 }), null);
  assert.equal(await m.executeDriveCommand({ ...command, operation: 'shell' }), null);
  const result = await m.executeDriveCommand(command);
  assert.equal(result.result.status, 'waiting');
  assert.equal(calls[0].url, '/api/assistant/drive/execute');
  assert.equal(calls[0].options.headers.Authorization, 'Bearer device-only');
  assert.equal(JSON.stringify(result).includes('device-only'), false);
  globalThis.fetch = async (url, options) => {
    assert.equal(url, '/api/assistant/drive/speeds');
    assert.equal(options.headers.Authorization, 'Bearer device-only');
    return { ok: true, json: async () => ({ ok: true, status: 'ok', vehicle_speed_kph: 64.8, set_speed_kph: 60, sample_age_ms: 10 }) };
  };
  assert.deepEqual(await m.readDriveSpeeds(), { status: 'ok', vehicle_speed_kph: 64.8, set_speed_kph: 60, sample_age_ms: 10 });
  document.hidden = true;
  assert.equal((await m.executeDriveCommand(command)).result.status, 'unverified');
  assert.equal(calls.length, 1);
  document.hidden = false;
  globalThis.fetch = async (url, options) => {
    calls.push({ url, options });
    return { ok: true, json: async () => ({ ok: true, revoked: true }) };
  };
  await m.revokeDriveAccess();
  assert.equal((await m.readDriveSpeeds()).reason, 'device_permission');
  assert.equal(m.driveAccessEnabled(), false);
  assert.equal((await m.executeDriveCommand(command)).result.status, 'unverified');
  assert.equal(calls.length, 2);
});
