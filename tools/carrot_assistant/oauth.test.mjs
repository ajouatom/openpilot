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
  const app = createApp({ oauthIssuer: base, ...(dir ? { statePath: join(dir, 'bridge.json'), oauthStatePath: join(dir, 'oauth.json') } : {}) });
  // The restart test reuses a port. Do not reuse fetch sockets from its previous server.
  http.on('request', (req, res) => { res.setHeader('Connection', 'close'); app(req, res); });
  const post = (path, body, headers = {}) => fetch(`${base}${path}`, { method: 'POST', redirect: 'manual', headers: { 'Content-Type': 'application/json', ...headers }, body: JSON.stringify(body) });
  const form = (path, body, headers = {}) => fetch(`${base}${path}`, { method: 'POST', redirect: 'manual', headers: { 'Content-Type': 'application/x-www-form-urlencoded', ...headers }, body: new URLSearchParams(body) });
  const browser = (op, key, body = {}) => post(`/bridge/${op}`, body, { Origin: origin, ...(key ? { Authorization: `Bearer ${key}` } : {}) });
  async function register(redirect = callback) {
    return post('/register', { redirect_uris: [redirect], token_endpoint_auth_method: 'none', grant_types: ['authorization_code', 'refresh_token'], response_types: ['code'] });
  }
  async function begin(client, settings = false, drive = false, tools = false) {
    const verifier = randomBytes(32).toString('base64url');
    const response = await fetch(`${base}/authorize?${new URLSearchParams({ client_id: client.client_id, response_type: 'code', redirect_uri: callback,
      code_challenge_method: 'S256', code_challenge: digest(verifier), state: 'test-state', resource: `${base}/account/mcp`, scope: ['carrot.screen', ...(settings ? ['carrot.settings'] : []), ...(drive ? ['carrot.drive'] : []), ...(tools ? ['carrot.tools'] : [])].join(' ') })}`, { redirect: 'manual' });
    assert.equal(response.status, 200);
    // A real browser must permit the consent POST's cross-origin 303 callback.
    const policy = response.headers.get('content-security-policy');
    assert.equal(policy.split(';').map(s => s.trim()).find(s => s.startsWith('form-action')), `form-action 'self' ${callback}`);
    assert.match(policy, /frame-ancestors 'none'/);
    assert.equal(response.headers.get('referrer-policy'), 'same-origin');
    const flow = (await response.text()).match(/name="flow" value="([^"]+)"/)[1];
    return { verifier, flow, cookie: response.headers.get('set-cookie').split(';')[0] };
  }
  async function link(client, key, settings = false, drive = false, approveDrive = drive) {
    const auth = await begin(client, settings, drive);
    const device = await (await browser('account_code', key)).json();
    const approved = await form('/oauth/approve', { flow: auth.flow, code: device.account_code, ...(settings ? { settings_access: 'yes' } : {}), ...(approveDrive ? { drive_access: 'yes' } : {}) }, { Cookie: auth.cookie, Origin: base });
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

test('English OAuth consent and errors retain the same fields and security requirements', async () => {
  const env = await start();
  try {
    const client = await (await env.register()).json();
    const query = new URLSearchParams({ client_id: client.client_id, response_type: 'code', redirect_uri: callback,
      code_challenge_method: 'S256', code_challenge: digest(randomBytes(32)), state: 'language-test',
      resource: `${env.base}/account/mcp`, scope: 'carrot.screen carrot.settings carrot.drive carrot.tools' });
    const page = async language => {
      const response = await fetch(`${env.base}/authorize?${query}`, { headers: { 'Accept-Language': language } });
      assert.equal(response.status, 200);
      assert.match(response.headers.get('vary'), /Accept-Language/i);
      return response.text();
    };
    const en = await page('en-US,en;q=0.9'), ko = await page('ko-KR,ko;q=0.9');
    assert.match(en, /<html lang="en">/); assert.doesNotMatch(en, /[가-힣]/);
    assert.match(en, /Authorize this connection/); assert.match(en, /diagnostic log uploads/);
    assert.match(ko, /이 기기 연결 승인/);
    assert.deepEqual([...en.matchAll(/name="([^"]+)"/g)].map(x => x[1]), [...ko.matchAll(/name="([^"]+)"/g)].map(x => x[1]));
    const error = await env.form('/oauth/approve', { flow: 'missing' }, { 'Accept-Language': 'en' });
    assert.equal(error.status, 403); assert.match(await error.text(), /connection request expired/);
  } finally { await env.close(); }
});

test('selecting another Carrot Web keeps ChatGPT authorization without a new code or broader scopes', async () => {
  const env = await start();
  let mcp;
  try {
    const client = await (await env.register()).json();
    const browser = await (await env.browser('create')).json();
    const linked = await env.link(client, browser.browser_token);
    const tokens = await (await env.form('/token', linked.exchange)).json();
    mcp = await env.mcp(tokens.access_token);
    const target = 'http://10.225.84.19:7000';
    const moved = await env.post('/bridge/relocate', { previous_origin: origin }, { Origin: target, Authorization: `Bearer ${browser.browser_token}` });
    assert.equal(moved.status, 200);
    assert.equal((await moved.json()).account_linked, true);
    assert.equal((await env.browser('inspect', browser.browser_token)).status, 403);
    const info = await mcp.callTool({ name: 'get_carrot_web_status', arguments: {} });
    assert.equal(info.isError, undefined);
    assert.equal(JSON.parse(info.content[0].text).label, '10.225.84.19:7000');
    const refreshed = await (await env.form('/token', { grant_type: 'refresh_token', client_id: client.client_id,
      refresh_token: tokens.refresh_token, resource: `${env.base}/account/mcp` })).json();
    assert.equal(refreshed.scope, 'carrot.screen');
  } finally { if (mcp) await mcp.close(); await env.close(); }
});

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
    const opaqueOrigin = await env.form('/oauth/approve', { flow: auth.flow, code: code.account_code }, { Cookie: auth.cookie, Origin: 'null' });
    assert.equal(opaqueOrigin.status, 403);
    assert.match(await opaqueOrigin.text(), /보안 확인/);
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

test('Tools OAuth requires its own checkbox, preserves old grants and delivers bounded commands', async () => {
  const env = await start();
  const clients = [];
  try {
    const client = await (await env.register()).json();
    const device = await (await env.browser('create')).json();
    const oldLink = await env.link(client, device.browser_token, true);
    const oldToken = await (await env.form('/token', oldLink.exchange)).json();
    for (const consent of [false, true]) {
      const flow = await env.begin(client, true, false, true);
      const code = await (await env.browser('account_code', device.browser_token)).json();
      const approved = await env.form('/oauth/approve', { flow: flow.flow, code: code.account_code,
        settings_access: 'yes', ...(consent ? { tools_access: 'yes' } : {}) }, { Cookie: flow.cookie, Origin: env.base });
      assert.equal(approved.status, 303);
      const token = await (await env.form('/token', { grant_type: 'authorization_code', client_id: client.client_id,
        code: new URL(approved.headers.get('location')).searchParams.get('code'), code_verifier: flow.verifier,
        redirect_uri: callback, resource: `${env.base}/account/mcp` })).json();
      assert.equal(token.scope.includes('carrot.tools'), consent);
      const chat = await env.mcp(token.access_token); clients.push(chat);
      const listed = await chat.callTool({ name: 'list_carrot_tool_actions', arguments: {} });
      assert.equal(Boolean(listed.isError), !consent);
      if (consent) {
        await env.browser('poll', device.browser_token);
        const pending = chat.callTool({ name: 'prepare_carrot_tool_action', arguments: { action: 'reboot' } });
        await new Promise(r => setTimeout(r, 1000));
        const poll = await (await env.browser('poll', device.browser_token)).json();
        assert.equal(poll.tools_command.action, 'reboot');
        assert.equal(poll.tools_command.operation, 'prepare');
        const result = { status: 'prepared', action: 'reboot', title: 'Reboot', effect: 'Connection stops',
          confirmation_required: true, proposal_id: 'p'.repeat(43), expires_in_seconds: 60 };
        assert.equal((await env.browser('tools_ack', device.browser_token, { id: poll.tools_command.id, result })).status, 200);
        assert.deepEqual((await pending).structuredContent.result, result);
      }
    }
    const refreshed = await (await env.form('/token', { grant_type: 'refresh_token', client_id: client.client_id,
      refresh_token: oldToken.refresh_token, resource: `${env.base}/account/mcp` })).json();
    assert.equal(refreshed.scope.includes('carrot.tools'), false);
  } finally { for (const client of clients) await client.close(); await env.close(); }
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
    const context = (await chatA.callTool({ name: 'get_carrot_context', arguments: {} })).structuredContent;
    assert.equal(context.vehicle_connected, null);
    assert.equal(context.vehicle_state, 'not_observed');
    assert.equal(context.can_drive, false);
    const listed = (await chatA.listTools()).tools;
    assert.equal(listed.length, 21); assert.ok(listed.some(t => t.name === 'open_carrot_phone_app')); assert.equal(listed.some(t => t.name === 'connect_carrot_web'), false);
    assert.equal((await chatA.callTool({ name: 'request_carrot_drive', arguments: { action: 'lane_left' } })).isError, true);
    assert.equal((await chatA.callTool({ name: 'get_carrot_drive_request', arguments: {} })).isError, true);
    assert.equal((await chatA.callTool({ name: 'read_carrot_setting_value', arguments: { name: 'AutoBlinkerControl' } })).isError, true);
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

test('driving scope needs explicit consent and cannot be added by refreshing a settings grant', async () => {
  const env = await start(); const clients = [];
  try {
    const registered = await (await env.register()).json();
    const browser = await (await env.browser('create')).json();
    const declined = await env.link(registered, browser.browser_token, true, true, false);
    const noDrive = await (await env.form('/token', declined.exchange)).json();
    assert.equal(noDrive.scope, 'carrot.screen carrot.settings');
    assert.equal((await env.form('/token', { grant_type: 'refresh_token', client_id: registered.client_id,
      refresh_token: noDrive.refresh_token, resource: `${env.base}/account/mcp`, scope: 'carrot.screen carrot.settings carrot.drive' })).status, 400);
    const oldChat = await env.mcp(noDrive.access_token); clients.push(oldChat);
    const approved = await env.link(registered, browser.browser_token, true, true);
    const token = await (await env.form('/token', approved.exchange)).json();
    assert.equal(token.scope, 'carrot.screen carrot.settings carrot.drive');
    assert.equal((await oldChat.callTool({ name: 'request_carrot_drive', arguments: { action: 'speed_up' } })).isError, true);
    const chat = await env.mcp(token.access_token); clients.push(chat);
    const listed = (await chat.listTools()).tools;
    const context = (await chat.callTool({ name: 'get_carrot_context', arguments: {} })).structuredContent;
    assert.equal(context.can_drive, true);
    assert.equal(context.vehicle_connected, null);
    assert.equal(context.vehicle_state, 'not_observed');
    assert.equal(context.web.browser_online, false);
    const annotation = listed.find(t => t.name === 'request_carrot_drive').annotations;
    assert.equal(annotation.readOnlyHint, false);
    assert.equal(annotation.destructiveHint, true);
    const waitingBrowser = env.browser('poll_wait', browser.browser_token);
    await new Promise(resolve => setTimeout(resolve, 20));
    const online = (await chat.callTool({ name: 'get_carrot_context', arguments: {} })).structuredContent;
    assert.equal(online.web.browser_online, true);
    assert.equal(online.vehicle_connected, null);
    const pending = chat.callTool({ name: 'request_carrot_drive', arguments: { action: 'speed_set', value: 100 } });
    const poll = await (await waitingBrowser).json();
    assert.equal(poll.drive_command.action, 'speed_set');
    assert.equal(poll.drive_command.value, 100);
    await env.browser('drive_ack', browser.browser_token, { id: poll.drive_command.id,
      result: { id: poll.drive_command.id, status: 'rejected', reason: 'cruise_inactive_or_driver_override' } });
    assert.equal((await pending).structuredContent.result.status, 'rejected');
  } finally { await Promise.all(clients.map(c => c.close())); await env.close(); }
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

test('explicit settings OAuth scope delivers only to its browser and returns verified result', async () => {
  const env = await start(); const clients = [];
  try {
    const registered = await (await env.register()).json();
    const browser = await (await env.browser('create')).json();
    const old = await env.link(registered, browser.browser_token);
    const oldTokens = await (await env.form('/token', old.exchange)).json();
    assert.equal((await env.form('/token', { grant_type: 'refresh_token', client_id: registered.client_id, refresh_token: oldTokens.refresh_token,
      resource: `${env.base}/account/mcp`, scope: 'carrot.screen carrot.settings' })).status, 400);
    const linked = await env.link(registered, browser.browser_token, true);
    const tokens = await (await env.form('/token', linked.exchange)).json();
    assert.equal(tokens.scope, 'carrot.screen carrot.settings');
    const chat = await env.mcp(tokens.access_token); clients.push(chat);
    const pending = chat.callTool({ name: 'prepare_carrot_setting_change', arguments: { name: 'ShowCustomBrightness', mode: 'percent', value: 10 } });
    await new Promise(resolve => setTimeout(resolve, 80));
    const poll = await (await env.browser('poll', browser.browser_token)).json();
    assert.equal(poll.settings_command.operation, 'prepare');
    assert.equal(poll.settings_command.name, 'ShowCustomBrightness');
    assert.equal(poll.settings_command.value, 10);
    assert.equal((await env.browser('settings_ack', browser.browser_token, { id: poll.settings_command.id, result: {
      status: 'prepared', name: 'ShowCustomBrightness', current: 50, proposed: 55, proposal_id: 'p'.repeat(43), confirmation_required: true,
    } })).status, 200);
    assert.equal((await pending).structuredContent.result.proposed, 55);
    const ready = (await chat.callTool({ name: 'get_carrot_setting_request', arguments: {} })).structuredContent.result;
    assert.equal(ready.confirmation_required, false);
    assert.equal(ready.commit_required, true);
    assert.match(chat.getInstructions(), /명확한 사용자 변경 명령 자체가 승인/);
    const committing = chat.callTool({ name: 'commit_carrot_setting_change', arguments: { proposal_id: ready.proposal_id, confirmed: true } });
    await new Promise(resolve => setTimeout(resolve, 950));
    const commitPoll = await (await env.browser('poll', browser.browser_token)).json();
    assert.equal(commitPoll.settings_command.operation, 'commit');
    assert.equal(commitPoll.settings_command.proposal_id, ready.proposal_id);
    await env.browser('settings_ack', browser.browser_token, { id: commitPoll.settings_command.id,
      result: { status: 'applied', name: 'ShowCustomBrightness', previous: 50, current: 55 } });
    assert.equal((await committing).structuredContent.result.current, 55);
    const oldChat = await env.mcp(oldTokens.access_token); clients.push(oldChat);
    assert.equal((await oldChat.callTool({ name: 'get_carrot_setting_request', arguments: {} })).isError, true);
    const screen = (await oldChat.callTool({ name: 'get_carrot_web_status', arguments: {} })).structuredContent;
    assert.equal('settings_command' in screen, false);
    assert.equal('result' in screen, false);
    const invalid = await chat.callTool({ name: 'commit_carrot_setting_change', arguments: { proposal_id: 'p'.repeat(43), confirmed: false } });
    assert.equal(invalid.isError, true);
  } finally { for (const client of clients) await client.close(); await env.close(); }
});
