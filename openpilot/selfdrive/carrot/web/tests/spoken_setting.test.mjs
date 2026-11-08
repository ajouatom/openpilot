import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { resolveSpokenSetting, assignSpokenSetting } from '../src/features/tools/spoken_setting.js';
import { createLocalVoice } from '../src/features/tools/local_voice.js';

const { params } = JSON.parse(readFileSync(new URL('../../../carrot_settings.json', import.meta.url)));
const target = 'CruiseMaxVals0';
const id = 'a'.repeat(43);
globalThis.document = { hidden: false };

test('spoken writes require one exact catalog match', () => {
  for (const name of ['가속도 설정 0', '가속도 설정 영', 'acceleration setting zero', target])
    assert.equal(resolveSpokenSetting(params, name)?.name, target);
  assert.equal(resolveSpokenSetting(params, '가속도 설정 1')?.name, 'CruiseMaxVals1');
  assert.equal(resolveSpokenSetting(params, '가속도 설정'), null);
  assert.equal(resolveSpokenSetting(params, '가속도 설정 0 말고'), null);
  assert.equal(resolveSpokenSetting([{ name: 'A', title: '밝기' }, { name: 'B', title: '밝기' }], '밝기'), null);
});

function fixture(mode) {
  let token = 'local', shown = null;
  const calls = [];
  const deps = {
    catalog: async () => params, grant: () => token,
    display: result => { shown = result.current; },
    send: async (command, granted) => {
      calls.push(command);
      assert.equal(granted, 'local');
      if (command.operation === 'prepare') {
        if (mode === 'same') throw new Error('No integer value change; choose another value');
        if (mode === 'revoke') token = null;
        return { status: 'prepared', name: target, proposed: command.value, proposal_id: 'b'.repeat(43) };
      }
      if (command.operation === 'read') return { status: 'read', name: target, current: 240 };
      return { status: 'applied', name: target, current: mode === 'wrong' ? 241 : 240 };
    },
  };
  return { calls, deps, shown: () => shown };
}

test('absolute write confirms device readback and keeps a stable commit identity', async () => {
  const h = fixture();
  const result = await assignSpokenSetting('가속도 설정 0', 240, id, Date.now()+5000, h.deps);
  assert.equal(result.current, 240);
  assert.equal(result.status, 'applied');
  assert.deepEqual(h.calls.map(x => x.operation), ['prepare','commit']);
  assert.equal(h.calls[0].value, 240);
  assert.equal(h.calls[0].name, target);
  assert.notEqual(h.calls[0].id, h.calls[1].id);
  assert.equal(h.calls[1].id, id);
  assert.equal(h.shown(), 240);
});

test('range, permission, revocation, expiry, unchanged and readback mismatch', async () => {
  for (const [mode, value, expected, calls] of [
    ['range',251,'range',0], ['denied',240,'permission',0], ['revoke',240,'unverified',1],
    ['expired',240,'unverified',0], ['same',240,'unchanged',2], ['wrong',240,'unverified',2],
  ]) {
    const h = fixture(mode);
    if (mode === 'denied') h.deps.grant = () => null;
    const result = await assignSpokenSetting('가속도 설정 0', value, id, Date.now()+(mode === 'expired' ? -1 : 5000), h.deps);
    assert.equal(result.status, expected);
    assert.equal(h.calls.length, calls);
    assert.equal(h.shown(), null);
  }
});

test('search has no write path, accepts no duplicates and reads the displayed result count', async () => {
  const searched = [];
  globalThis.findPhoneSettings = async query => { searched.push(query); return { status:'found', total:2 }; };
  const voice = createLocalVoice(null, null, null, () => 1000, () => { throw Error('unexpected write'); });
  assert.equal(voice.findSettings('가속도', id, 5000), true);
  await new Promise(r => setImmediate(r));
  assert.equal(voice.result(id).total, 2);
  assert.equal(voice.findSettings('가속도', id, 5000), true);
  assert.deepEqual(searched, ['가속도']);
  assert.equal(voice.findSettings(' ', id, 5000), false);
});

test('ordinal writes bind the displayed row to its canonical setting and report its actual title', async () => {
  const searchId = 's'.repeat(43);
  globalThis.phoneSearchResultName = (search, position) => search === searchId && position === 1 ? target : null;
  const h = fixture();
  const voice = createLocalVoice(null, null, null, Date.now, (name, value, request, expires, selection) =>
    assignSpokenSetting(name, value, request, expires, { ...h.deps, ...selection }));
  const until = Date.now() + 5000;
  const run = async (search, row, identity) => {
    assert.ok(voice.setResultValue(search, row, 240, 'ko', identity, until));
    await new Promise(r => setImmediate(r));
    return voice.result(identity);
  };
  assert.deepEqual(await run(searchId, 1, id), {status:'applied', label:'가속도설정 0', current:240});
  assert.equal((await run(searchId, 1, id)).status, 'applied');
  assert.equal((await run(searchId, 2, 'b'.repeat(43))).status, 'no_result');
  assert.equal((await run('x'.repeat(43), 1, 'c'.repeat(43))).status, 'no_result');
  assert.deepEqual(h.calls.map(x => x.operation), ['prepare', 'commit']);
});

test('changing the result selection during prepare cannot commit an ordinal write', async () => {
  const h = fixture();
  let row = target;
  const send = async (...args) => { const result = await h.deps.send(...args); row = null; return result; };
  const result = await assignSpokenSetting(target, 240, id, Date.now()+5000,
    { ...h.deps, send, language:'en', selected:() => row });
  assert.equal(result.status, 'unverified');
  assert.deepEqual(h.calls.map(c => c.operation), ['prepare']);
});
