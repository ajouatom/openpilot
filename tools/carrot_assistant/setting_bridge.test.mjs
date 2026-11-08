import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createSettingBridge } from './setting_bridge.mjs';

test('setting relay isolates owners, expires work and validates acknowledgements', () => {
  let time = 1000;
  const relay = createSettingBridge(() => time);
  const first = relay.queue('a', 'prepare', { name: 'ShowCustomBrightness', mode: 'percent', value: 10 });
  assert.equal(relay.status('b').status, 'none');
  assert.equal(relay.queue('a', 'prepare', { name: 'ShowCustomBrightness', mode: 'percent', value: 10 }).request_id, first.request_id);
  assert.throws(() => relay.queue('a', 'read', { name: 'ShowCustomBrightness' }));
  assert.throws(() => relay.ack('b', first.request_id, { status: 'failed' }));
  assert.throws(() => relay.ack('a', first.request_id, { status: 'applied', current: 66 }));
  assert.throws(() => relay.ack('a', first.request_id, { status: 'prepared', name: 'ShowCustomBrightness', current: 60 }));
  assert.throws(() => relay.ack('a', first.request_id, { status: 'failed', secret: 'not allowed' }));
  const result = { status: 'prepared', name: 'ShowCustomBrightness', current: 60, proposed: 66, proposal_id: 'p'.repeat(43), confirmation_required: true,
    step_note: 'step is the web button increment, not a required multiple or an offset from min. Any in-range integer may be specified.' };
  relay.ack('a', first.request_id, result);
  assert.deepEqual(relay.status('a').result, { ...result, confirmation_required: false, commit_required: true });
  // Compatibility with existing devices must not turn preflight into a write.
  assert.equal(relay.status('a').result.status, 'prepared');
  assert.equal(relay.poll('a'), null);
  relay.queue('a', 'commit', { proposal_id: result.proposal_id });
  time += 30000;
  assert.equal(relay.status('a').status, 'expired');
  assert.equal(relay.poll('a'), null);
  relay.remove('a');
  assert.equal(relay.status('a').status, 'none');
});
