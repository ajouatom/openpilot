import { test } from 'node:test';
import assert from 'node:assert/strict';
import { screenRequest, sessionValid } from '../src/features/tools/assistant_policy.js';

test('only fresh, queued, bounded screen requests are eligible for delivery', () => {
  const command = { id: 'x'.repeat(43), name: 'AutoBlinkerControl', status: 'queued', expires_at: 100 };
  assert.equal(screenRequest(command, 50), true);
  assert.equal(screenRequest(command, 101), false);
  assert.equal(screenRequest({ ...command, status: 'cancelled' }, 50), false);
  assert.equal(screenRequest({ ...command, name: '../api/params' }, 50), false);
  assert.equal(sessionValid({ browser_token: 'x'.repeat(43), expires_at: 100 }, 50), true);
  assert.equal(sessionValid({ browser_token: 'x'.repeat(43), expires_at: 100 }, 100), true);
  assert.equal(sessionValid({ browser_token: 'x'.repeat(43), expires_at: null }, 100), true);
  assert.equal(sessionValid({ browser_token: 'x'.repeat(43) }, 100), false);
});
