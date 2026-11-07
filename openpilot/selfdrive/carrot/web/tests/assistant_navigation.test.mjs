import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';

const page = readFileSync(new URL('../js/pages/setting.js', import.meta.url), 'utf8');
const source = page.slice(page.indexOf('async function openAssistantSetting('), page.indexOf('// Overlay panel DOM state'));
function harness({ available = true, visible = true, detail = false, leave = false } = {}) {
  const events = [];
  const entry = { name: 'AutoBlinkerControl', group: 'Steer', source: 'carrot', ...(detail ? { detailParent: 'Parent' } : {}) };
  const context = vm.createContext({
    CURRENT_PAGE: 'tools', CURRENT_SETTING_DETAIL: null,
    requestAnimationFrame: cb => setImmediate(cb),
    showPage: () => { context.CURRENT_PAGE = 'setting'; },
    loadSettings: async () => { setImmediate(() => { events.push('initial-reset'); if (leave) context.CURRENT_PAGE = 'tools'; }); },
    switchSettingTab: async () => {},
    rebuildSettingSearchEntries: () => available ? [entry] : [],
    selectSettingSearchEntry: async () => { events.push('select'); setImmediate(() => events.push('group-reset')); },
    selectSettingDetail: async () => { events.push('detail'); context.CURRENT_SETTING_DETAIL = 'Parent'; },
    document: { querySelectorAll: () => [{ dataset: {settingName: entry.name}, offsetParent: visible ? {} : null }] },
    focusSettingItem: (_name, behavior) => { events.push(`focus-${behavior}`); return true; },
  });
  vm.runInContext(source, context);
  return { open: context.openAssistantSetting, events };
}

test('assistant navigation waits for initial/group scroll resets before confirming the visible row', async () => {
  const h = harness({ detail: true });
  assert.equal(await h.open('AutoBlinkerControl'), 'opened');
  assert.deepEqual(h.events, ['initial-reset', 'select', 'detail', 'group-reset', 'focus-instant']);
});

test('assistant does not confirm hidden/missing settings or force a jump after the user leaves', async () => {
  for (const options of [{visible:false}, {leave:true}]) {
    const h = harness(options);
    assert.equal(await h.open('AutoBlinkerControl'), 'failed');
    assert.equal(h.events.some(e => e.startsWith('focus')), false);
  }
  assert.equal(await harness({available:false}).open('AutoBlinkerControl'), 'unavailable');
  const h = harness();
  assert.equal(await h.open('../param_set'), 'unavailable');
  assert.deepEqual(h.events, []);
});
