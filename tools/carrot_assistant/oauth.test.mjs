import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { createHash, randomBytes } from 'node:crypto';
import { mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StreamableHTTPClientTransport } from '@modelcontextprotocol/sdk/client/streamableHttp.js';
import { createApp } from './server.mjs';
import { createBridge } from './bridge.mjs';
import { createAccountAuth } from './oauth.mjs';

const origin = 'http://192.168.0.178:7000';
const callback = 'https://chatgpt.com/connector_platform_oauth_redirect';
const digest = value => createHash('sha256').update(value).digest('base64url');
async function start(dir, port = 0) {
  const http = createServer().listen(port, '127.0.0.1');
  await new Promise(r => http.once('listening', r));
  const base = `http://127.0.0.1:${http.address().port}`;
  http.on('request', createApp({ oauthIssuer: base, ...(dir ? { statePath: join(dir, 'bridge.json'), oauthStatePath: join(dir, 'oauth.json') } : {}) }));
  const post = (path, body, headers = {}) => fetch(`${base}${path}`, { method: 'POST', redirect: 'manual', headers: { 'Content-Type': 'application/json', ...headers }, body: JSON.stringify(body) });
  const form = (path, body, headers = {}) => fetch(`${base}${path}`, { method: 'POST', redirect: 'manual', headers: { 'Content-Type': 'application/x-www-form-urlencoded', ...headers }, body: new URLSearchParams(body) });
  const browser = (op, key, body = {}) => post(`/bridge/${op}`, body, { Origin: origin, ...(key ? { Authorization: `Bearer ${key}` } : {}) });
  async function register(redirect = callback) {
    return post('/register', { redirect_uris: [redirect], token_endpoint_auth_method: 'none', grant_types: ['authorization_code', 'refresh_token'], response_types: ['code'] });
  }
  async function begin(client) {
    const verifier = randomBytes(32).toString('base64url');
    const response = await fetch(`${base}/authorize?${new URLSearchParams({ client_id: client.client_id, response_type: 'code', redirect_uri: callback,
      code_challenge_method: 'S256', code_challenge: digest(verifier), state: 'test-state', resource: `${base}/account/mcp`, scope: 'carrot.screen' })}`, { redirect: 'manual' });
    assert.equal(response.status, 200);
    const flow = (await response.text()).match(/name="flow" value="([^"]+)"/)[1];
    return { verifier, flow, cookie: response.headers.get('set-cookie').split(';')[0] };
  }
  async function link(client, key) {
    const auth = await begin(client);
    const device = await (await browser('account_code', key)).json();
    const approved = await form('/oauth/approve', { flow: auth.flow, code: device.account_code }, { Cookie: auth.cookie, Origin: base });
    assert.equal(approved.status, 303);
    const destination = new URL(approved.headers.get('location'));
    assert.equal(destination.searchParams.get('state'), 'test-state');
    const exchange = { grant_type: 'authorization_code', client_id: client.client_id, code: destination.searchParams.get('code'),
      code_verifier: auth.verifier, redirect_uri: callback, resource: `${base}/account/mcp` };
    return { auth, device, exchange };
  }
  async function mcp(access) {
    const client = new Client({ name: 'new-voice-chat', version: '1' });
    await client.connect(new StreamableHTTPClientTransport(new URL(`${base}/account/mcp`), { requestInit: { headers: { Authorization: `Bearer ${access}` } } }));
    return client;
  }
  return { http, base, post, form, browser, register, begin, link, mcp, close: () => new Promise(r => http.close(r)) };
}

test('OAuth discovery, callback allowlist, consent CSRF, PKCE, resource and code replay', async () => {
  const env = await start();
  try {
    const unauth = await env.post('/account/mcp', {});
    assert.equal(unauth.status, 401); assert.match(unauth.headers.get('www-authenticate'), /oauth-protected-resource\/account\/mcp/);
    const discovery = await (await fetch(`${env.base}/.well-known/oauth-authorization-server`)).json();
    assert.deepEqual(discovery.code_challenge_methods_supported, ['S256']);
    for (const bad of ['https://evil.example/callback', 'https://chatgpt.com.evil.example/connector_platform_oauth_redirect', 'http://chatgpt.com/connector_platform_oauth_redirect', `${callback}?leak=1`]) assert.equal((await env.register(bad)).status, 400);
    const client = await (await env.register()).json(), other = await (await env.register()).json();
    const device = await (await env.browser('create')).json();
    const auth = await env.begin(client);
    const code = await (await env.browser('account_code', device.browser_token)).json();
    assert.equal((await env.form('/oauth/approve', { flow: auth.flow, code: code.account_code }, { Origin: env.base })).status, 403);
    assert.equal((await env.form('/oauth/approve', { flow: auth.flow, code: code.account_code }, { Cookie: auth.cookie, Origin: 'https://evil.example' })).status, 403);
    const linked = await env.link(client, device.browser_token);
    assert.equal((await env.form('/token', { ...linked.exchange, code_verifier: 'x'.repeat(43) })).status, 400);
    assert.equal((await env.form('/token', { ...linked.exchange, resource: `${env.base}/other` })).status, 400);
    assert.equal((await env.form('/token', { ...linked.exchange, client_id: other.client_id })).status, 400);
    assert.equal((await env.form('/token', { ...linked.exchange, redirect_uri: 'https://evil.example' })).status, 400);
    const tokenResponse = await env.form('/token', linked.exchange);
    assert.equal(tokenResponse.status, 200);
    assert.equal((await env.form('/token', linked.exchange)).status, 400);
    assert.equal((await env.form('/oauth/approve', { flow: linked.auth.flow, code: linked.device.account_code }, { Cookie: linked.auth.cookie, Origin: env.base })).status, 403);
  } finally { await env.close(); }
});

test('fresh MCP conversations resolve only the authenticated browser, survive restart and revoke', async () => {
  const dir = mkdtempSync(join(tmpdir(), 'carrot-oauth-'));
  let env = await start(dir);
  const sdk = [];
  try {
    const client = await (await env.register()).json();
    const a = await (await env.browser('create')).json(), b = await (await env.browser('create')).json();
    const la = await env.link(client, a.browser_token), lb = await env.link(client, b.browser_token);
    const ta = await (await env.form('/token', la.exchange)).json(), tb = await (await env.form('/token', lb.exchange)).json();
    const chatA = await env.mcp(ta.access_token), chatB = await env.mcp(tb.access_token); sdk.push(chatA, chatB);
    const listed = (await chatA.listTools()).tools;
    assert.equal(listed.length, 6); assert.equal(listed.some(t => t.name === 'connect_carrot_web'), false);
    assert.deepEqual(Object.keys(listed.find(t => t.name === 'open_carrot_setting').inputSchema.properties), ['name']);
    const opened = await chatA.callTool({ name: 'open_carrot_setting', arguments: { name: 'AutoBlinkerControl' } });
    assert.equal(opened.structuredContent.command.status, 'queued');
    assert.equal((await chatB.callTool({ name: 'get_carrot_web_status', arguments: {} })).structuredContent.command, null);
    const poll = await (await env.browser('poll', a.browser_token)).json();
    assert.equal(poll.account_linked, true); assert.equal(poll.command.name, 'AutoBlinkerControl');
    await env.browser('ack', a.browser_token, { id: poll.command.id, status: 'opened' });
    // A new conversation has no connection_id or old chat history.
    const fresh = await env.mcp(ta.access_token); sdk.push(fresh);
    assert.equal((await fresh.callTool({ name: 'get_carrot_web_status', arguments: {} })).structuredContent.command.status, 'opened');
    assert.equal((await fresh.callTool({ name: 'open_carrot_setting', arguments: { name: 'UnknownSetting' } })).isError, true);
    const disk = readFileSync(join(dir, 'oauth.json'), 'utf8') + readFileSync(join(dir, 'bridge.json'), 'utf8');
    for (const secret of [a.browser_token, la.device.account_code, la.exchange.code, ta.access_token, ta.refresh_token, tb.access_token]) assert.equal(disk.includes(secret), false);
    await Promise.all(sdk.splice(0).map(c => c.close()));
    const port = env.http.address().port; await env.close(); env = await start(dir, port);
    const resumed = await env.mcp(ta.access_token); sdk.push(resumed);
    assert.equal((await resumed.callTool({ name: 'get_carrot_web_status', arguments: {} })).structuredContent.command, null);
    const refreshBody = { grant_type: 'refresh_token', client_id: client.client_id, refresh_token: ta.refresh_token, resource: `${env.base}/account/mcp` };
    const refreshed = await (await env.form('/token', refreshBody)).json(); assert.ok(refreshed.access_token);
    assert.equal((await env.form('/token', refreshBody)).status, 400); // replay revokes that grant only
    assert.equal((await env.post('/account/mcp', {}, { Authorization: `Bearer ${refreshed.access_token}` })).status, 401);
    const bchat = await env.mcp(tb.access_token); sdk.push(bchat);
    assert.equal((await bchat.callTool({ name: 'get_carrot_web_status', arguments: {} })).structuredContent.connected, true);
    await env.browser('disconnect', b.browser_token);
    assert.equal((await env.post('/account/mcp', {}, { Authorization: `Bearer ${tb.access_token}` })).status, 401);
    assert.equal((await env.form('/token', { ...refreshBody, refresh_token: tb.refresh_token })).status, 400);
  } finally { await Promise.all(sdk.map(c => c.close())); await env.close(); rmSync(dir, { recursive: true, force: true }); }
});

test('device authorization code expiry, single use and legacy pairing preservation', () => {
  let time = 1000;
  const bridge = createBridge({ now: () => time });
  const a = bridge.create(origin), legacy = bridge.pair(a.pairing_code);
  const code = bridge.browser(a.browser_token, origin, 'account_code').account_code;
  time += 300001; assert.throws(() => bridge.claimAccountCode(code));
  const next = bridge.browser(a.browser_token, origin, 'account_code').account_code;
  const identity = bridge.claimAccountCode(next);
  assert.equal(bridge.status(legacy.connection_id).connected, true);
  assert.equal(bridge.accountStatus(identity.id).connected, true);
  assert.throws(() => bridge.claimAccountCode(next));
  assert.throws(() => bridge.accountStatus(a.browser_token));
  bridge.browser(a.browser_token, origin, 'disconnect');
  assert.throws(() => bridge.accountStatus(identity.id));
});

test('corrupt OAuth state and failed client persistence fail closed', async () => {
  const dir = mkdtempSync(join(tmpdir(), 'carrot-oauth-invalid-'));
  try {
    const path = join(dir, 'oauth.json'); writeFileSync(path, '{bad');
    assert.throws(() => createAccountAuth({ issuer: 'https://carrot.example', bridge: createBridge(), statePath: path }));
    const provider = createAccountAuth({ issuer: 'https://carrot.example', bridge: createBridge(), statePath: join(dir, 'missing', 'oauth.json') });
    await assert.rejects(provider.clientsStore.registerClient({ client_id: 'test', redirect_uris: [callback], token_endpoint_auth_method: 'none' }));
    assert.equal(await provider.clientsStore.getClient('test'), undefined);
  } finally { rmSync(dir, { recursive: true, force: true }); }
});
