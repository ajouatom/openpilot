import { test } from 'node:test';
import assert from 'node:assert/strict';
import { copyPairingMessage } from '../src/features/tools/assistant_clipboard.js';

function fixture({ secure = false, write, command = () => true } = {}) {
  const calls = [];
  const field = { value: 'Connect this test code: ABC123',
    focus: () => calls.push('focus'), select: () => calls.push('select'),
    setSelectionRange: (start, end) => calls.push([start, end]) };
  const environment = { isSecureContext: secure, navigator: { clipboard: write ? {writeText:write} : undefined },
    document: { execCommand: name => { calls.push(name); return command(); } } };
  return {field, environment, calls};
}

test('LAN HTTP copies the selected field synchronously without navigator.clipboard', async () => {
  const h = fixture();
  const result = copyPairingMessage(h.field, h.environment);
  assert.deepEqual(h.calls, ['focus', 'select', [0, h.field.value.length], 'copy']);
  assert.equal(await result, true);
});

test('secure clipboard success avoids legacy selection and denied access falls back', async () => {
  let copied;
  const secure = fixture({secure:true, write:async value => { copied = value; }});
  assert.equal(await copyPairingMessage(secure.field, secure.environment), true);
  assert.equal(copied, secure.field.value);
  assert.deepEqual(secure.calls, []);
  const denied = fixture({secure:true, write:async () => { throw Error('NotAllowedError'); }});
  assert.equal(await copyPairingMessage(denied.field, denied.environment), true);
  assert.equal(denied.calls.at(-1), 'copy');
});

test('rejected copy leaves text selected and never claims success; empty fields do not copy', async () => {
  for (const command of [() => false, () => { throw Error('blocked'); }]) {
    const h = fixture({command});
    assert.equal(await copyPairingMessage(h.field, h.environment), false);
    assert.ok(h.calls.includes('select'));
  }
  const h = fixture(); h.field.value = '';
  assert.equal(await copyPairingMessage(h.field, h.environment), false);
  assert.deepEqual(h.calls, []);
});
