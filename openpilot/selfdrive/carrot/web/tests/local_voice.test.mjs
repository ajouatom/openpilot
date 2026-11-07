import test from 'node:test';
import assert from 'node:assert/strict';
import { createLocalVoice } from '../src/features/tools/local_voice.js';

test('phone commands need no relay session and make one origin-bound request', async () => {
  globalThis.document = { hidden: false };
  let time = 1000, allowed = false;
  const sent = [], opened = [];
  const voice = createLocalVoice(async name => { opened.push(name); return 'opened'; }, () => allowed,
    async command => { sent.push(command); return { result: { status: 'waiting' } }; }, () => time);
  const run = async (action, key) => {
    const id = key.repeat(43);
    assert.equal(voice.start(action,id,time+3500), true);
    await new Promise(resolve => setImmediate(resolve));
    return voice.result(id);
  };
  assert.equal(await run('blinker_settings','a'), 'opened');
  assert.deepEqual(opened,['AutoBlinkerControl']);
  assert.equal(await run('lane_left','b'), 'permission');
  assert.equal(sent.length,0);
  allowed = true;
  assert.equal(await run('lane_left','c'), 'waiting');
  assert.equal(await run('lane_left','c'), 'waiting');
  assert.equal(await run('lane_right','d'), 'duplicate');
  assert.equal(sent.length,1);
  time += 11000;
  assert.equal(await run('lane_right','e'), 'waiting');
  assert.deepEqual(sent.map(x=>x.action), ['lane_left','lane_right']);
  document.hidden = true;
  assert.equal(voice.start('lane_left','x'.repeat(43),time+50), false);
  document.hidden = false;
  assert.equal(voice.start('lane_left','x'.repeat(43),time-1), false);
  assert.equal(voice.start('shell','x'.repeat(43),time+50), false);
});

test('late callbacks and competing commands do not replace an in-flight request', async () => {
  globalThis.document = {hidden:false};
  let finish, calls = 0;
  const voice = createLocalVoice(null, () => true, () => { calls++; return new Promise(r => {finish=r;}); }, () => 1000);
  assert.ok(voice.start('lane_left','a'.repeat(43),2500));
  assert.ok(voice.start('lane_left','a'.repeat(43),2500));
  assert.equal(voice.start('lane_right','b'.repeat(43),2500),false);
  assert.equal(calls,1);
  finish(null);
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(voice.result('a'.repeat(43)),'unverified');
});
