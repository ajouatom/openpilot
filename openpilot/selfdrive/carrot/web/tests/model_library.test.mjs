import assert from 'node:assert/strict';
import test from 'node:test';
import { renderLibrary } from '../src/features/tools/model_library.js';
class Element {
  constructor(tag) { this.tag = tag; this.children = []; this.dataset = {}; this.events = {}; }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children = children; }
  setAttribute() {}
  showModal() { this.open = true; }
  close() { this.open = false; }
  addEventListener(name, callback) { this.events[name] = callback; }
  querySelector(tag) { return this.children.find(e => e.tag === tag) || this.children.map(e => e.querySelector?.(tag)).find(Boolean); }
}
function setupDocument() {
  const elements = new Map();
  for (const id of ['externalModelLibrary', 'modelLibraryDialog', 'modelLibraryEditor',
    'modelLibraryTitle', 'btnCloseModelLibrary', 'egpuModelCard', 'egpuModelState', 'egpuModelAmount']) {
    elements.set(id, new Element('div'));
  }
  elements.get('egpuModelCard').hidden = true;
  globalThis.document = { getElementById: id => elements.get(id), createElement: tag => new Element(tag) };
  return { root: elements.get('externalModelLibrary'), editor: elements.get('modelLibraryEditor'), elements };
}
const data = { effective: 'n16', selected: 'flip-flop', reboot_required: true, models: [
  { id: 'n16', name: 'N16 Nav', noo: true, hidden: false, protected: true, egpu_bytes: 100, jetson_bytes: null },
  { id: 'flip-flop', name: 'Flip Flop', noo: false, hidden: false, protected: true, egpu_bytes: 200, jetson_bytes: 300, jetson_sha256: 'f'.repeat(64) },
] };
test('model choice distinguishes current/pending/NOO and preserves in-flight choice across polls', t => {
  const old = globalThis.document;
  t.after(() => { globalThis.document = old; });
  const { root, editor, elements } = setupDocument();
  renderLibrary(data);
  const select = editor.querySelector('select');
  assert.equal(select.value, 'flip-flop');
  assert.deepEqual(select.children.map(x => x.value), ['n16', 'flip-flop']);
  assert.match(select.children[0].textContent, /NOO supported/);
  assert.match(select.children[1].textContent, /NOO not supported/);
  assert.match(root.querySelector('p').textContent, /eGPU this boot N16/);
  assert.equal(root.querySelector('strong').textContent, 'Flip Flop');
  assert.equal(root.querySelector('select'), undefined);
  assert.equal(root.querySelector('button').textContent, 'Change model');
  assert.equal(root.children.length, 2);
  root.querySelector('button').events.click();
  assert.equal(elements.get('modelLibraryDialog').open, true);
  elements.get('btnCloseModelLibrary').events.click();
  assert.equal(elements.get('modelLibraryDialog').open, false);
  assert.equal(root.children.some(x => x.className === 'model-library__row'), false);
  select.value = 'n16';
  select.disabled = true;
  renderLibrary({ ...data, jetson_inventory_fresh: true });
  assert.equal(editor.querySelector('select').value, 'n16');
  renderLibrary(data, true);
  assert.equal(editor.querySelector('select').value, 'flip-flop');
});

test('selection alone saves next boot and exposes no deletion action', async t => {
  const old = { document: globalThis.document, postJson: globalThis.postJson };
  t.after(() => Object.assign(globalThis, old));
  const { root, editor, elements } = setupDocument();
  const calls = [];
  globalThis.postJson = async (url, body) => { calls.push({url, body}); return {library: {...data, selected: body.model}}; };
  renderLibrary(data, true);
  const select = editor.querySelector('select');
  select.value = 'n16';
  await select.events.change();
  assert.deepEqual(calls, [{url: '/api/egpu/model/library', body: {action:'select',model:'n16'}}]);
  assert.equal(root.querySelector('button').textContent, 'Change model');
  assert.equal(editor.querySelector('select').value, 'n16');
});

test('failed selection restores the saved choice', async t => {
  const old = { document: globalThis.document, postJson: globalThis.postJson };
  t.after(() => Object.assign(globalThis, old));
  const { root, editor, elements } = setupDocument();
  globalThis.postJson = async () => { throw new Error('offline'); };
  renderLibrary(data, true);
  const select = editor.querySelector('select');
  select.value = 'n16';
  await select.events.change();
  assert.equal(editor.querySelector('select').value, 'flip-flop');
  assert.ok(!editor.querySelector('select').disabled);
});


test('status updates preserve the open dialog and focused selection across inventory polls', t => {
  const old = globalThis.document;
  t.after(() => { globalThis.document = old; });
  const { root, editor, elements } = setupDocument();
  const card = elements.get('egpuModelCard');
  card.hidden = false;
  card.dataset.state = 'downloading';
  elements.get('egpuModelState').textContent = 'Downloading';
  elements.get('egpuModelAmount').textContent = '25%';
  renderLibrary({ ...data, selected: 'n16', reboot_required: false });
  root.querySelector('button').events.click();
  const button = root.querySelector('button');
  const select = editor.querySelector('select');
  assert.equal(root.querySelector('p').textContent, 'Downloading · 25%');
  card.dataset.state = 'error';
  elements.get('egpuModelState').textContent = 'eGPU unavailable';
  elements.get('egpuModelAmount').textContent = '';
  renderLibrary({ ...data, selected: 'n16', reboot_required: false, jetson_inventory_fresh: true });
  assert.equal(editor.querySelector('select'), select);
  assert.equal(root.querySelector('button'), button);
  assert.equal(elements.get('modelLibraryDialog').open, true);
  assert.equal(root.dataset.state, 'error');
  assert.equal(root.querySelector('p').textContent, 'eGPU unavailable');
});

test('shows the model a Jetlink host runs or prepares and the pick sent to it', t => {
  const old = globalThis.document;
  t.after(() => { globalThis.document = old; });
  const { root, editor } = setupDocument();
  const host = { sha256: 'f'.repeat(64), preparation: null };
  renderLibrary({ ...data, app_pick: { model: 'n16' } }, true, host);
  const lines = editor.children.map(x => x.textContent || '');
  assert.ok(lines.some(x => x === 'Jetlink host running: Flip Flop · NOO not supported'));
  assert.ok(lines.some(x => x === 'Sent to Jetlink host: N16 Nav'));
  renderLibrary(data, true, { ...host, preparation: { stage: 'upload', fraction: 0.5, message: 'Uploading' } });
  assert.ok(editor.children.some(x => x.textContent === 'Jetlink host preparing: Flip Flop · NOO not supported · Uploading · 50%'));
  // A phone or tablet switches while parked: its state shows without opening the dialog.
  renderLibrary({ ...data, reboot_required: false }, true, host);
  assert.equal(root.querySelector('p').textContent, 'Internal model: CD210 · Jetlink host running: Flip Flop · NOO not supported');
});

test('internal model choice lists CD210 and Cheddar and saves with the internal action', async t => {
  const old = { document: globalThis.document, postJson: globalThis.postJson };
  t.after(() => Object.assign(globalThis, old));
  const { root, editor } = setupDocument();
  const internal_model = { selected: 'cd210', models: [{ id: 'cd210', name: 'CD210' }, { id: 'cheddar', name: 'Cheddar' }] };
  const calls = [];
  globalThis.postJson = async (url, body) => {
    calls.push([url, body]);
    return { library: { ...data, internal_model: { ...internal_model, selected: body.model } } };
  };
  renderLibrary({ ...data, reboot_required: false, internal_model });
  const internal = editor.querySelector('select');
  assert.deepEqual(internal.children.map(x => x.value), ['cd210', 'cheddar']);
  assert.equal(internal.value, 'cd210');
  assert.match(root.querySelector('p').textContent, /^Internal model: CD210/);
  internal.value = 'cheddar';
  await internal.events.change();
  assert.deepEqual(calls, [['/api/egpu/model/library', { action: 'internal', model: 'cheddar' }]]);
  assert.equal(editor.querySelector('select').value, 'cheddar');
  assert.match(root.querySelector('p').textContent, /Internal model: Cheddar/);
  assert.equal(editor.dataset.busy, undefined);
});

test('internal model shows what the last drive ran, including a Cheddar load failure', t => {
  const old = globalThis.document;
  t.after(() => { globalThis.document = old; });
  const models = [{ id: 'cd210', name: 'CD210' }, { id: 'cheddar', name: 'Cheddar' }];
  const texts = editor => editor.children.filter(e => e.tag === 'p').map(e => e.textContent);
  let { root, editor } = setupDocument();
  renderLibrary({ ...data, reboot_required: false, internal_model: { selected: 'cheddar', models,
    last: { selected: 'cheddar', running: 'cd210', error: 'model files: missing' } } });
  assert.match(root.querySelector('p').textContent, /^Internal model: CD210 · Cheddar failed/);
  assert.ok(texts(editor).includes('Last drive ran: CD210 · Cheddar could not be loaded; the drive ran on CD210 (model files: missing)'));
  ({ root, editor } = setupDocument());
  renderLibrary({ ...data, reboot_required: false, internal_model: { selected: 'cheddar', models,
    last: { selected: 'cheddar', running: 'cheddar', error: null } } });
  assert.match(root.querySelector('p').textContent, /^Internal model: Cheddar/);
  assert.ok(texts(editor).includes('Last drive ran: Cheddar'));
  ({ root, editor } = setupDocument());
  renderLibrary({ ...data, reboot_required: false, internal_model: { selected: 'cheddar', models,
    last: { selected: 'cd210', running: 'cd210', error: null } } });
  assert.ok(texts(editor).includes('Last drive ran: CD210 · Cheddar applies at the next ignition'));
});
