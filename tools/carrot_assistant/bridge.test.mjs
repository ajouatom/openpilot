import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createBridge, allowedWebOrigin } from './bridge.mjs';
import { createApp } from './server.mjs';
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StreamableHTTPClientTransport } from '@modelcontextprotocol/sdk/client/streamableHttp.js';
import { mkdtempSync, readFileSync, writeFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

test('DHCP migration keeps the account session, drops stale commands and persists the new origin', () => {
  const dir = mkdtempSync(join(tmpdir(), 'carrot-move-'));
  const path = join(dir, 'connections.json');
  let time = 1000;
  try {
    const old = 'http://10.225.84.19:7000', next = 'http://192.168.0.178:7000';
    const b = createBridge({ statePath: path, now: () => time, isAccountLinked: () => true, accountActivity: () => 123 });
    const browser = b.create(old), token = browser.browser_token;
    const account = b.claimAccountCode(b.browser(token, old, 'account_code').account_code);
    assert.equal(b.browser(token, old, 'inspect').account_activity_at, 123);
    assert.equal(b.accountStatus(account.id).browser_online, false); // inspect is not a heartbeat
    const pendingCode = b.browser(token, old, 'account_code');
    b.browser(token, old, 'poll');
    b.accountAction(account.id, 'open', 'AutoBlinkerControl');
    const result = b.browser(token, next, 'relocate', { previous_origin: old });
    assert.equal(result.account_linked, true); assert.equal(result.browser_online, false);
    assert.equal(result.command, null); assert.equal(result.label, '192.168.0.178:7000');
    assert.throws(() => b.claimAccountCode(pendingCode.account_code));
    assert.throws(() => b.browser(token, old, 'poll'), /origin mismatch/);
    assert.throws(() => b.browser('z'.repeat(43), next, 'relocate', { previous_origin: old }));
    assert.throws(() => b.browser(token, 'https://evil.example', 'relocate', { previous_origin: next }));
    b.browser(token, next, 'relocate', { previous_origin: old }); // lost response is retryable
    const restored = createBridge({ statePath: path, now: () => time });
    assert.equal(restored.accountStatus(account.id).label, '192.168.0.178:7000');
    assert.equal(restored.browser(token, next, 'poll').command, null);
    time += 1000;
    restored.browser(token, old, 'relocate', { previous_origin: next });
    assert.equal(restored.browser(token, old, 'poll').command, null);
  } finally { rmSync(dir, { recursive: true }); }
});

test('unpaired sessions cannot relocate and failed persistence leaves the old origin intact', () => {
  const dir = mkdtempSync(join(tmpdir(), 'carrot-move-failure-'));
  const old = 'http://10.0.0.2:7000', next = 'http://192.168.0.178:7000';
  const b = createBridge({ statePath: join(dir, 'connections.json') });
  const browser = b.create(old), token = browser.browser_token;
  assert.throws(() => b.browser(token, next, 'relocate', { previous_origin: old }));
  b.claimAccountCode(b.browser(token, old, 'account_code').account_code);
  rmSync(dir, { recursive: true });
  assert.throws(() => b.browser(token, next, 'relocate', { previous_origin: old }));
  assert.equal(b.browser(token, old, 'inspect').connected, true);
});

test('address selection discards pending settings, driving and Tools delivery', async () => {
  const b = createBridge(), old = 'http://10.0.0.2:7000', next = 'http://192.168.0.178:7000';
  const browser = b.create(old), token = browser.browser_token;
  const account = b.claimAccountCode(b.browser(token, old, 'account_code').account_code);
  b.browser(token, old, 'poll');
  const requests = [b.accountSettingAction(account.id, 'read', { name: 'ShowCustomBrightness' }),
    b.accountDriveAction(account.id, { action: 'speed_up' }),
    b.accountToolsAction(account.id, 'prepare', { action: 'backup_settings' })];
  b.browser(token, next, 'relocate', { previous_origin: old });
  const poll = b.browser(token, next, 'poll');
  for (const field of ['settings_command', 'drive_command', 'tools_command']) assert.equal(poll[field], null);
  await Promise.all(requests);
});

test('fresh account codes retain their full lifetime beyond the original browser lease', () => {
  let time = 1000;
  const b = createBridge({ now: () => time });
  const origin = 'http://carrot:7000';
  const browser = b.create(origin);
  time += 299000;
  const old = b.browser(browser.browser_token, origin, 'account_code');
  time += 2000;
  assert.equal(b.browser(browser.browser_token, origin, 'poll').expires_at, old.expires_at);
  // Renewing account approval must not extend the old legacy chat-pairing code.
  assert.throws(() => b.pair(browser.pairing_code));
  const fresh = b.browser(browser.browser_token, origin, 'account_code');
  assert.throws(() => b.claimAccountCode(old.account_code));
  time += 299999;
  const account = b.claimAccountCode(fresh.account_code);
  assert.equal(b.accountStatus(account.id).connected, true);
  assert.throws(() => b.claimAccountCode(fresh.account_code));
});

test('an unclaimed renewed account code still expires at its own deadline', () => {
  let time = 1000;
  const b = createBridge({ now: () => time });
  const origin = 'http://carrot:7000';
  const browser = b.create(origin);
  time += 299000;
  const code = b.browser(browser.browser_token, origin, 'account_code');
  time = code.expires_at;
  assert.throws(() => b.claimAccountCode(code.account_code));
  assert.throws(() => b.browser(browser.browser_token, origin, 'poll'));
});

test('pairing is single-use, origin-bound, isolated, expiring and revocable', () => {
  let time = 1000;
  const bridge = createBridge({ now: () => time, maxSessions: 2 });
  const origin = 'http://192.168.0.178:7000';
  const a = bridge.create(origin), b = bridge.create(origin);
  assert.throws(() => bridge.create(origin));
  const control = bridge.pair(a.pairing_code).connection_id;
  assert.throws(() => bridge.pair(a.pairing_code));
  assert.throws(() => bridge.status(a.browser_token));
  assert.throws(() => bridge.browser(control, origin, 'poll'));
  assert.throws(() => bridge.browser(a.browser_token, 'http://192.168.0.179:7000', 'poll'));
  const command = bridge.open(control, 'AutoBlinkerControl').command;
  assert.equal(bridge.browser(b.browser_token, origin, 'poll').command, null);
  assert.equal(bridge.browser(a.browser_token, origin, 'poll').command.id, command.id);
  assert.throws(() => bridge.open(control, 'CruiseGap1'));
  assert.equal(bridge.open(control, 'AutoBlinkerControl').command.id, command.id);
  assert.throws(() => bridge.browser(a.browser_token, origin, 'ack', { id: command.id, status: 'arbitrary' }));
  bridge.browser(a.browser_token, origin, 'ack', { id: command.id, status: 'opened' });
  assert.equal(bridge.status(control).command.status, 'opened');
  bridge.open(control, 'CruiseGap1'); time += 90001;
  assert.equal(bridge.status(control).command.status, 'expired');
  bridge.open(control, 'CruiseGap1'); bridge.cancel(control);
  assert.equal(bridge.status(control).command.status, 'cancelled');
  bridge.browser(a.browser_token, origin, 'disconnect');
  assert.throws(() => bridge.status(control));
  time += 300001; assert.throws(() => bridge.pair(b.pairing_code));
  const c = bridge.create(origin), d = bridge.pair(c.pairing_code);
  time += 365 * 24 * 3600000; assert.equal(bridge.status(d.connection_id).connected, true);
});

test('paired hashes survive restart without queued commands; disconnect is durable', () => {
  const dir = mkdtempSync(join(tmpdir(), 'carrot-bridge-'));
  const statePath = join(dir, 'connections.json'), origin = 'http://192.168.0.178:7000';
  try {
    let bridge = createBridge({ statePath });
    const browser = bridge.create(origin), control = bridge.pair(browser.pairing_code).connection_id;
    bridge.open(control, 'AutoBlinkerControl');
    const disk = readFileSync(statePath, 'utf8');
    for (const secret of [browser.browser_token, browser.pairing_code, control, 'AutoBlinkerControl']) assert.equal(disk.includes(secret), false);
    bridge = createBridge({ statePath });
    assert.equal(bridge.status(control).connected, true);
    assert.equal(bridge.status(control).command, null);
    assert.equal(bridge.browser(browser.browser_token, origin, 'poll').expires_at, null);
    bridge.browser(browser.browser_token, origin, 'disconnect');
    bridge = createBridge({ statePath });
    assert.throws(() => bridge.status(control));
    writeFileSync(statePath, '{broken');
    assert.throws(() => createBridge({ statePath }));
  } finally { rmSync(dir, { recursive: true, force: true }); }
});

test('failed persistence does not issue a control capability', () => {
  const dir = mkdtempSync(join(tmpdir(), 'carrot-bridge-'));
  try {
    const bridge = createBridge({ statePath: join(dir, 'missing', 'state.json') });
    const browser = bridge.create('http://localhost:7000');
    assert.throws(() => bridge.pair(browser.pairing_code));
    assert.equal(bridge.browser(browser.browser_token, 'http://localhost:7000', 'poll').connected, false);
  } finally { rmSync(dir, { recursive: true, force: true }); }
});

test('CORS permits local web origins, not null or lookalike/public origins', () => {
  for (const origin of ['null', 'https://evil.example', 'http://192.168.0.1.evil.example:7000', 'ftp://localhost', 'http://localhost:7000/path']) {
    assert.equal(allowedWebOrigin(origin), false, origin);
  }
  assert.equal(allowedWebOrigin('http://192.168.0.178:7000'), true);
});

test('HTTP browser pairing to actual MCP call and browser acknowledgement end to end', async () => {
  const http = createApp().listen(0, '127.0.0.1');
  await new Promise(r => http.once('listening', r));
  const base = `http://127.0.0.1:${http.address().port}`, origin = 'http://192.168.0.178:7000';
  const client = new Client({ name: 'bridge-test', version: '1' });
  const browser = (op, key, body = {}) => fetch(`${base}/bridge/${op}`, {
    method: 'POST', headers: { Origin: origin, 'Content-Type': 'application/json', ...(key ? { Authorization: `Bearer ${key}` } : {}) }, body: JSON.stringify(body),
  });
  try {
    const preflight = await fetch(`${base}/bridge/create`, { method: 'OPTIONS', headers: { Origin: origin } });
    assert.equal(preflight.status, 204); assert.equal(preflight.headers.get('access-control-allow-origin'), origin);
    assert.equal((await fetch(`${base}/bridge/create`, { method: 'POST' })).status, 403);
    const created = await browser('create'); assert.equal(created.status, 200);
    const pair = await created.json();
    await client.connect(new StreamableHTTPClientTransport(new URL(`${base}/mcp`)));
    const connected = await client.callTool({ name: 'connect_carrot_web', arguments: { pairing_code: pair.pairing_code } });
    const connection_id = connected.structuredContent.connection_id;
    const bad = await client.callTool({ name: 'open_carrot_setting', arguments: { connection_id, name: 'NotARealSetting' } });
    assert.equal(bad.isError, true);
    const opened = await client.callTool({ name: 'open_carrot_setting', arguments: { connection_id, name: 'AutoBlinkerControl' } });
    assert.equal(opened.structuredContent.command.status, 'queued');
    const poll = await (await browser('poll', pair.browser_token)).json();
    assert.equal(poll.command.name, 'AutoBlinkerControl');
    await browser('ack', pair.browser_token, { id: poll.command.id, status: 'opened' });
    const status = await client.callTool({ name: 'get_carrot_web_status', arguments: { connection_id } });
    assert.equal(status.structuredContent.command.status, 'opened');
    assert.equal((await browser('poll', 'x'.repeat(43))).status, 401);
    await browser('disconnect', pair.browser_token);
    assert.equal((await client.callTool({ name: 'get_carrot_web_status', arguments: { connection_id } })).isError, true);
  } finally { await client.close(); await new Promise(r => http.close(r)); }
});
