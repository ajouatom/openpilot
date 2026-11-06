import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createDriveBridge } from './drive_bridge.mjs';
import { createBridge } from './bridge.mjs';

test('driving delivery deduplicates, separates owners and never replays after expiry', () => {
  let time = 10000;
  const b = createDriveBridge(() => time);
  const first = b.queue('a', { action: 'lane_left' });
  assert.equal(b.queue('a', { action: 'lane_left' }).request_id, first.request_id);
  assert.throws(() => b.queue('a', { action: 'lane_right' }));
  assert.equal(b.poll('b'), null);
  assert.equal(b.poll('a').operation, 'execute');
  assert.throws(() => b.ack('b', first.request_id, { id: first.request_id, status: 'completed', direction: 'left' }));
  time += 5001;
  assert.equal(b.poll('a'), null);
  b.ack('a', first.request_id, { id: first.request_id, status: 'completed', direction: 'left' });
  assert.equal(b.status('a').result.status, 'expired');
});

test('vehicle acknowledgement switches to observation and requires credible action-specific results', () => {
  let time = 10000;
  const b = createDriveBridge(() => time);
  const id = b.queue('a', { action: 'lane_left' }).request_id;
  for (const result of [{ status: 'applied' }, { status: 'completed' }, { status: 'completed', direction: 'right' }]) {
    assert.throws(() => b.ack('a', id, { id, ...result }));
  }
  b.ack('a', id, { id, status: 'running', direction: 'left' });
  assert.equal(b.poll('a').operation, 'status');
  time += 21000;
  assert.equal(b.poll('a'), null);
  assert.equal(b.status('a').result.status, 'unverified');
  const speed = b.queue('b', { action: 'speed_set', value: 100 }).request_id;
  assert.throws(() => b.ack('b', speed, { id: speed, status: 'applied' }));
  assert.throws(() => b.ack('b', speed, { id: speed, status: 'completed', current: 100 }));
  b.ack('b', speed, { id: speed, status: 'applied', previous: 90, current: 100 });
  assert.equal(b.poll('b'), null);
  assert.equal(b.status('b').result.current, 100);
  b.remove('b');
  assert.equal(b.status('b').status, 'none');
});

test('waiting browser wakes for a command and one tool call returns its acknowledgement', async () => {
  const origin = 'http://192.168.0.178:7000';
  const b = createBridge();
  const device = b.create(origin);
  const code = b.browser(device.browser_token, origin, 'account_code').account_code;
  const account = b.claimAccountCode(code);
  const wait = b.waitForBrowser(device.browser_token, origin);
  await assert.rejects(b.waitForBrowser(device.browser_token, origin));
  const started = Date.now();
  const request = b.accountDriveAction(account.id, { action: 'speed_set', value: 100 });
  const delivered = await wait;
  assert.ok(Date.now() - started < 1000);
  assert.equal(delivered.drive_command.value, 100);
  b.browser(device.browser_token, origin, 'drive_ack', { id: delivered.drive_command.id,
    result: { id: delivered.drive_command.id, status: 'applied', previous: 90, current: 100 } });
  assert.equal((await request).result.status, 'applied');
});
