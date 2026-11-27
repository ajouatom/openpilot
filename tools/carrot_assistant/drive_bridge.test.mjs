import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createDriveBridge } from './drive_bridge.mjs';
import { createBridge } from './bridge.mjs';

test('driving delivery deduplicates, separates owners and never replays after expiry', () => {
  let time = 10000;
  const b = createDriveBridge(() => time);
  const first = b.queue('a', { action: 'lane_left' });
  assert.equal(b.queue('a', { action: 'lane_left' }).request_id, first.request_id);
  const blocked = b.queue('a', { action: 'lane_right' });
  assert.equal(blocked.status, 'blocked');
  assert.equal(blocked.submitted, false);
  assert.equal(blocked.retry_allowed, false);
  assert.equal(b.queue('a', { action: 'lane_left' }).reason, 'duplicate');
  assert.equal(b.poll('a').id, first.request_id);
  assert.equal(b.poll('b'), null);
  assert.equal(b.poll('a').operation, 'execute');
  assert.throws(() => b.ack('b', first.request_id, { id: first.request_id, status: 'completed', direction: 'left' }));
  time += 5001;
  assert.equal(b.poll('a'), null);
  b.ack('a', first.request_id, { id: first.request_id, status: 'completed', direction: 'left' });
  assert.equal(b.status('a').result.status, 'expired');
});

test('speed reads cannot replace an active command, cross owners, or survive disconnect', async () => {
  let time = 10000;
  const origin = 'http://localhost:7000';
  const bridge = createBridge({ now: () => time });
  function connect() {
    const device = bridge.create(origin);
    const browser = (operation, body) => bridge.browser(device.browser_token, origin, operation, body);
    const owner = bridge.claimAccountCode(browser('account_code').account_code).id;
    return { owner, browser };
  }
  const a = connect(), b = connect();
  assert.equal(bridge.accountDriveAction(a.owner, { action: 'speed_up' }).reason, 'browser_offline');
  a.browser('poll');
  const receipt = bridge.accountDriveAction(a.owner, { action: 'speed_up' });
  const read = bridge.accountSpeeds(a.owner);
  assert.equal(bridge.accountSpeeds(a.owner), read);
  time += 1000;
  const polled = a.browser('poll');
  assert.equal(polled.drive_command.id, receipt.request_id);
  const reply = { id: polled.speed_read.id, result: { status: 'unavailable', reason: 'no_fresh_data' } };
  assert.throws(() => b.browser('speed_read_ack', reply));
  a.browser('speed_read_ack', reply);
  assert.deepEqual(await read, reply.result);
  assert.equal(bridge.accountDriveStatus(a.owner).request_id, receipt.request_id);
  const interrupted = bridge.accountSpeeds(a.owner);
  a.browser('disconnect');
  assert.equal((await interrupted).reason, 'connection_failed');
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

test('timed-out speed replies cannot complete a later read or occupy the driving slot', async (t) => {
  t.mock.timers.enable({ apis: ['setTimeout'] });
  let time = 10000;
  const origin = 'http://localhost:7000';
  const bridge = createBridge({ now: () => time });
  const device = bridge.create(origin);
  const browser = (operation, body) => bridge.browser(device.browser_token, origin, operation, body);
  const owner = bridge.claimAccountCode(browser('account_code').account_code).id;
  browser('poll');
  const expired = bridge.accountSpeeds(owner);
  time += 1000;
  const oldId = browser('poll').speed_read.id;
  t.mock.timers.tick(4000);
  assert.equal((await expired).reason, 'read_timeout');
  const pending = bridge.accountSpeeds(owner);
  const drive = bridge.accountDriveAction(owner, { action: 'speed_set', value: 60 });
  time += 1000;
  const poll = browser('poll');
  assert.equal(poll.drive_command.id, drive.request_id);
  assert.notEqual(poll.speed_read.id, oldId);
  const result = { status: 'ok', vehicle_speed_kph: 64.8, set_speed_kph: 60, sample_age_ms: 0 };
  assert.throws(() => browser('speed_read_ack', { id: oldId, result }), /Unknown speed read/);
  assert.throws(() => browser('speed_read_ack', { id: poll.speed_read.id, result: { ...result, sample_age_ms: 1001 } }));
  browser('speed_read_ack', { id: poll.speed_read.id, result });
  assert.equal((await pending).vehicle_speed_kph, 64.8);
  assert.equal(bridge.accountDriveStatus(owner).request_id, drive.request_id);
});

test('changing the selected device invalidates its in-flight speed sample', async () => {
  let time = 10000;
  const origin = 'http://192.168.0.178:7000', next = 'http://192.168.0.179:7000';
  const bridge = createBridge({ now: () => time });
  const device = bridge.create(origin);
  const browser = (operation, body) => bridge.browser(device.browser_token, origin, operation, body);
  const owner = bridge.claimAccountCode(browser('account_code').account_code).id;
  browser('poll');
  const pending = bridge.accountSpeeds(owner);
  time += 1000;
  const id = browser('poll').speed_read.id;
  bridge.browser(device.browser_token, next, 'relocate', { previous_origin: origin });
  assert.equal((await pending).reason, 'connection_failed');
  assert.throws(() => bridge.browser(device.browser_token, next, 'speed_read_ack', {
    id, result: { status: 'ok', vehicle_speed_kph: 50, set_speed_kph: 60, sample_age_ms: 0 },
  }), /Unknown speed read/);
});

test('transport delay cannot turn an old speed sample into a current reading', async () => {
  let time = 10000;
  const origin = 'http://localhost:7000';
  const bridge = createBridge({ now: () => time });
  const device = bridge.create(origin);
  const browser = (operation, body) => bridge.browser(device.browser_token, origin, operation, body);
  const owner = bridge.claimAccountCode(browser('account_code').account_code).id;
  browser('poll');
  const pending = bridge.accountSpeeds(owner);
  time += 1000;
  const id = browser('poll').speed_read.id;
  await new Promise(resolve => setTimeout(resolve, 1050));
  browser('speed_read_ack', { id, result: { status: 'ok', vehicle_speed_kph: 50, set_speed_kph: 60, sample_age_ms: 1000 } });
  assert.equal((await pending).reason, 'no_fresh_data');
});

test('waiting browser wakes immediately; the receipt precedes the controller result', async () => {
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
  assert.equal((await request).status, 'queued');
  assert.equal(b.accountDriveStatus(account.id).result.status, 'applied');
});

test('different commands follow verified completion, while A/B/A, in-flight and uncertain outcomes are protected', () => {
  let time = 10000;
  const b = createDriveBridge(() => time);
  const speed = { action: 'speed_set', value: 60 }, lane = { action: 'lane_right' };
  const first = b.queue('a', speed).request_id;
  assert.equal(b.queue('a', lane).reason, 'request_in_progress');
  time += 120;
  b.ack('a', first, { id: first, status: 'applied', previous: 80, current: 60 });
  const second = b.queue('a', lane).request_id;
  assert.notEqual(first, second);
  assert.equal(b.poll('a').id, second);
  assert.throws(() => b.ack('a', first, { id: first, status: 'unchanged', current: 60 }));
  b.ack('a', second, { id: second, status: 'completed', direction: 'right' });
  assert.equal(b.queue('a', speed).request_id, first);
  assert.equal(b.queue('a', speed).reason, 'duplicate');
  time = 20000;
  assert.equal(b.queue('a', speed).status, 'queued');
  time += 5100;
  assert.equal(b.queue('a', lane).reason, 'previous_outcome_uncertain');
  assert.equal(b.poll('a'), null);
});

// Without a controller reply, submission must finish promptly and the same
// request must still accept a later acknowledgement. Exercise both first states.
test('lane receipt is independent of controller feedback', { timeout: 2500 }, async () => {
  for (const [side, acknowledge] of [['left', false], ['right', true]]) {
    const origin = 'http://carrot:7000';
    let time = 100000;
    const bridge = createBridge({ now: () => time });
    const device = bridge.create(origin);
    const browser = (operation, body) => bridge.browser(device.browser_token, origin, operation, body);
    const owner = bridge.claimAccountCode(browser('account_code').account_code).id;
    browser('poll');
    const pending = bridge.accountDriveAction(owner, { action: `lane_${side}` });
    time += 1000;
    const { id } = browser('poll').drive_command;
    if (acknowledge) browser('drive_ack', { id, result: { id, status: 'running', direction: side } });
    const first = await pending;
    assert.equal(first.request_id, id);
    assert.equal(first.status, 'queued');
    assert.equal(first.submitted, true);
    assert.equal(bridge.accountDriveStatus(owner).status, acknowledge ? 'active' : 'queued');
    time += 1000;
    if (acknowledge) assert.equal(browser('poll').drive_command.operation, 'status');
    browser('drive_ack', { id, result: { id, status: 'rejected', reason: 'vehicle_unavailable' } });
    assert.equal(bridge.accountDriveStatus(owner).result.status, 'rejected');
    time += 1000;
    assert.equal(browser('poll').drive_command, null);
  }
});
