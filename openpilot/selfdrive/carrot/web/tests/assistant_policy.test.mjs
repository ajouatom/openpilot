import { test } from 'node:test';
import assert from 'node:assert/strict';
import { chatGptUrl, screenRequest, sessionValid } from '../src/features/tools/assistant_policy.js';

test('assistant launcher accepts own ChatGPT URLs, rejects other origins and executable URLs', () => {
  assert.equal(chatGptUrl(''), 'https://chatgpt.com/');
  assert.equal(chatGptUrl('https://chatgpt.com/c/abc-123'), 'https://chatgpt.com/c/abc-123');
  for (const url of ['javascript:alert(1)', 'https://chatgpt.com.evil.example/c/x', 'https://chatgpt.com@evil.example/', 'https://u:p@chatgpt.com/', 'http://chatgpt.com/', 'https://chatgpt.com/c/x?token=secret', '//chatgpt.com/']) {
    assert.equal(chatGptUrl(url), null, url);
  }
});
test('only fresh, queued, bounded screen requests are eligible for delivery', () => {
  const command = { id: 'x'.repeat(43), name: 'AutoBlinkerControl', status: 'queued', expires_at: 100 };
  assert.equal(screenRequest(command, 50), true);
  assert.equal(screenRequest(command, 101), false);
  assert.equal(screenRequest({ ...command, status: 'cancelled' }, 50), false);
  assert.equal(screenRequest({ ...command, name: '../api/params' }, 50), false);
  assert.equal(sessionValid({ browser_token: 'x'.repeat(43), expires_at: 100 }, 50), true);
  assert.equal(sessionValid({ browser_token: 'x'.repeat(43), expires_at: 100 }, 100), false);
});
