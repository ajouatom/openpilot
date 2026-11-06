import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createBridge, allowedWebOrigin } from './bridge.mjs';
import { createApp } from './server.mjs';
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StreamableHTTPClientTransport } from '@modelcontextprotocol/sdk/client/streamableHttp.js';

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
  time += 8 * 3600000; assert.throws(() => bridge.status(d.connection_id));
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
