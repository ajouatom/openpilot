import test from 'node:test';
import assert from 'node:assert/strict';
import { createToolsBridge, toolActions, toolPanels } from './tools_bridge.mjs';
import { createServer } from './server.mjs';
import { InMemoryTransport } from '@modelcontextprotocol/sdk/inMemory.js';
import { Client } from '@modelcontextprotocol/sdk/client/index.js';

test('Tools requests expire, isolate owners, validate results and freeze terminal acknowledgements', () => {
  let now = 1000;
  const b = createToolsBridge(() => now);
  const r = b.queue('one', 'prepare', { action: 'reboot' });
  assert.equal(b.poll('two'), null);
  assert.throws(() => b.ack('two', r.request_id, { status: 'prepared' }));
  assert.throws(() => b.queue('one', 'prepare', { action: 'git_pull' }));
  assert.throws(() => b.ack('one', r.request_id, { status: 'completed', action: 'reboot' }));
  assert.throws(() => b.ack('one', r.request_id, { status: 'prepared', action: 'reboot' }));
  const result = { status: 'prepared', action: 'reboot', title: 'Reboot', effect: 'Connection stops', confirmation_required: true, proposal_id: 'p'.repeat(43) };
  assert.throws(() => b.ack('one', r.request_id, { ...result, log: 'private' }));
  assert.equal(b.ack('one', r.request_id, result).result.status, 'prepared');
  assert.deepEqual(b.ack('one', r.request_id, { status: 'failed' }).result, result);
  const next = b.queue('one', 'commit', { proposal_id: result.proposal_id });
  now += 30000;
  assert.equal(b.poll('one'), null);
  assert.equal(b.ack('one', next.request_id, { status: 'running', job_id: 'a'.repeat(12) }).status, 'expired');
  b.remove('one'); assert.equal(b.status('one').status, 'none');
});

test('MCP existing screen/settings/drive grants cannot run Tools; separately granted requests use the fixed catalog', async () => {
  for (const allowed of [false, true]) {
    const calls = [];
    const bridge = { accountToolsAction: (...args) => { calls.push(args); return { status: 'queued' }; } };
    const server = createServer(() => {}, bridge, 'owner', ['carrot.screen', 'carrot.settings', 'carrot.drive', ...(allowed ? ['carrot.tools'] : [])]);
    const client = new Client({ name: 'test', version: '1' });
    const [a, b] = InMemoryTransport.createLinkedPair();
    await server.connect(a); await client.connect(b);
    try {
      const result = await client.callTool({ name: 'prepare_carrot_tool_action', arguments: { action: 'git_pull' } });
      assert.equal(Boolean(result.isError), !allowed);
      assert.equal(calls.length, allowed ? 1 : 0);
      if (allowed) assert.deepEqual(calls[0], ['owner', 'prepare', { action: 'git_pull' }]);
      const bad = await client.callTool({ name: 'prepare_carrot_tool_action', arguments: { action: 'shell_cmd' } });
      assert.equal(bad.isError, true);
      const phone = await client.callTool({ name: 'open_carrot_phone_app', arguments: { app: 'naver_map' } });
      assert.equal(Boolean(phone.isError), !allowed);
      if (allowed) assert.deepEqual(calls.at(-1), ['owner', 'open', { action: 'naver_map' }]);
      const invalid = await client.callTool({ name: 'open_carrot_phone_app', arguments: { app: 'any_package' } });
      assert.equal(invalid.isError, true);
      const tools = await client.listTools();
      assert.equal(tools.tools.find(t => t.name === 'commit_carrot_tool_action').annotations.destructiveHint, true);
      assert.equal(toolActions.includes('git_checkout'), false);
      assert.equal(toolPanels.includes('restore_settings'), true);
    } finally { await client.close(); await server.close(); }
  }
});
