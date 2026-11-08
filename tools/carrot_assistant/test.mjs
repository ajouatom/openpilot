import { test } from 'node:test';
import assert from 'node:assert/strict';
import { get } from 'node:http';
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { StreamableHTTPClientTransport } from '@modelcontextprotocol/sdk/client/streamableHttp.js';
import { createApp } from './server.mjs';
import { searchSettings, getSetting } from './catalog.mjs';

test('Korean questions resolve real catalog entries, with no invented current value', () => {
  for (const query of ['깜박이 제어에 관련된 설정을 찾아줘', '앞차간격 설정', '화면이 너무 어두워', '가속이 너무 빨라']) {
    const result = searchSettings(query, 3);
    assert.ok(result.matches.length > 0, query);
    result.matches.forEach(item => { assert.equal(item.current_value, null); assert.ok(getSetting(item.name).found); });
  }
  assert.ok(searchSettings('깜박이').matches.some(item => item.name === 'AutoBlinkerControl'));
  assert.equal(searchSettings('zzzz-no-setting-zzzz').total, 0);
  assert.equal(getSetting('UnknownSetting').found, false);
});

test('English requests resolve stable parameter IDs and include English descriptions', () => {
  for (const query of ['turn signal settings', 'screen brightness', 'following gap', 'acceleration settings']) {
    const result = searchSettings(query, 3);
    assert.ok(result.matches.length > 0, query);
    for (const item of result.matches) {
      assert.ok(item.etitle); assert.ok(item.edescr);
      assert.equal(item.current_value, null);
      assert.ok(getSetting(item.name).found);
    }
  }
  assert.equal(searchSettings('ShowCustomBrightness',1).matches[0].name, 'ShowCustomBrightness');
});

test('public deployment restricts hostnames and bounds request traffic', async () => {
  const http = createApp({ host: '0.0.0.0', publicHostname: 'carrot-mcp.shind0.synology.me', requestsPerMinute: 2 }).listen(0, '127.0.0.1');
  await new Promise(resolve => http.once('listening', resolve));
  const base = `http://127.0.0.1:${http.address().port}`;
  const hostStatus = hostname => new Promise((resolve, reject) => {
    get(`${base}/health`, { headers: { Host: hostname } }, response => {
      response.resume(); resolve(response.statusCode);
    }).on('error', reject);
  });
  try {
    assert.equal(await hostStatus('carrot-mcp.shind0.synology.me'), 200);
    assert.equal(await hostStatus('evil.example'), 403);
    for (let i = 0; i < 2; i++) assert.equal((await fetch(`${base}/mcp`)).status, 405);
    const limited = await fetch(`${base}/mcp`);
    assert.equal(limited.status, 429);
    assert.equal(limited.headers.get('retry-after'), '60');
    assert.equal((await fetch(`${base}/health`)).status, 200);
  } finally {
    await new Promise(resolve => http.close(resolve));
  }
});

test('real MCP handshake, bounded tools, concurrent clients, invalid and write calls', async () => {
  const http = createApp().listen(0, '127.0.0.1');
  await new Promise(resolve => http.once('listening', resolve));
  const url = new URL(`http://127.0.0.1:${http.address().port}/mcp`);
  const clients = [];
  try {
    await Promise.all([1, 2].map(async n => {
      const client = new Client({ name: `test-${n}`, version: '1' });
      clients.push(client);
      await client.connect(new StreamableHTTPClientTransport(url));
      assert.match(client.getInstructions(), /Reply in the user's language/);
      assert.doesNotMatch(client.getInstructions(), /한국어로 짧게/);
      const tools = (await client.listTools()).tools;
      assert.equal(tools.length, 7);
      assert.equal(tools.filter(tool => tool.annotations.readOnlyHint === true).length, 4);
      const context = await client.callTool({ name: 'get_carrot_context', arguments: {} });
      assert.equal(context.structuredContent.vehicle_connected, false);
      const found = await client.callTool({ name: 'search_carrot_settings', arguments: { query: '깜박이', limit: 3 } });
      assert.equal(found.structuredContent.matches.length, 3);
      const exact = await client.callTool({ name: 'get_carrot_setting', arguments: { name: found.structuredContent.matches[0].name } });
      assert.equal(exact.structuredContent.setting.current_value, null);
      const invalid = await client.callTool({ name: 'search_carrot_settings', arguments: { query: 'x', limit: 999 } });
      assert.equal(invalid.isError, true);
      const write = await client.callTool({ name: 'set_setting', arguments: { name: 'AutoBlinkerControl', value: 2 } });
      assert.equal(write.isError, true);
    }));
    assert.equal((await fetch(url)).status, 405);
    assert.equal((await fetch(url, { method: 'POST', headers: { Origin: 'https://evil.example' } })).status, 403);
    assert.equal((await fetch(new URL('/catalog.json', url))).status, 404);
  } finally {
    await Promise.all(clients.map(client => client.close()));
    await new Promise(resolve => http.close(resolve));
  }
});
