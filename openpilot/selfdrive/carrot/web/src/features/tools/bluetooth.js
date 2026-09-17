const WORDS = {
  ko: {
    title: "블루투스 리모컨", close: "닫기", scan: "장치 검색 (30초)", on: "블루투스 켜기", off: "블루투스 끄기",
    help: "정차하고 크루즈를 해제한 상태에서 설정하세요. 매핑을 켜면 해당 장치의 원래 터치·키보드·볼륨 입력을 차단합니다. 차로변경은 기존 속도·측후방·조향 조건을 따릅니다. 휴대폰도 페어링할 수 있지만 통화·오디오·네트워크 기능은 제공하지 않습니다.",
    empty: "장치를 페어링 모드로 놓고 검색하세요. 기존 휴대폰에 연결되어 있다면 그 연결을 해제하세요.",
    pair: "페어링", connect: "연결", disconnect: "연결 해제", forget: "등록 해제", paired: "페어링됨", connected: "연결됨",
    profile: "입력 종류", generic: "일반 키보드 / HID", enabled: "캐럿 매핑 사용 (원래 HID 입력 차단)",
    save: "매핑 저장", saved: "저장했습니다", test: "키 등록 / 시험 시작 (120초)", stop: "시험 종료", waiting: "시험 준비 중…",
    testing: "시험 중: 차량 명령은 보내지 않습니다. 버튼을 누르면 아래에 표시됩니다. 새 키는 자동으로 목록에 추가됩니다.",
    idle: "입력 대기", blocked: "정차·크루즈 해제 상태 또는 입력 서비스가 확인되지 않아 설정할 수 없습니다.",
    grabbed: "HID 입력 독점 중", inactive: "입력 독점 대기 (연결·매핑 사용 여부 확인)", learnHelp: "먼저 입력 종류와 매핑을 저장한 뒤 시험을 시작하세요. 일반 HID는 시험 중 버튼을 한 번 눌렀다 떼어 등록합니다. 변경한 기능은 다시 저장해야 적용됩니다. 키보드 자동 반복은 무시합니다. Yiser-J6는 짧게 한 번씩 눌러 사용하세요.",
    none: "사용 안 함", accelCruise: "크루즈 + (accelCruise)", decelCruise: "크루즈 − (decelCruise)",
    laneLeft: "왼쪽 차로변경", laneRight: "오른쪽 차로변경", paddleDecel: "패들 감속", gapAdjustCruise: "크루즈 갭",
    up: "위", down: "아래", left: "왼쪽", right: "오른쪽", center: "중앙", "1": "1", "2": "2",
    cancel: "취소", confirm: "확인", pin: "PIN / 패스키", prompt: "페어링 확인", display: "상대 장치에 표시된 번호를 입력하거나 아래 번호를 확인하세요.",
    scanning: "검색 중", ready: "준비", unavailable: "블루투스 어댑터를 사용할 수 없습니다", noService: "입력 서비스가 실행되지 않았습니다",
    unsaved: "변경 내용을 저장하세요", sent: "차량 명령 전달", testEvent: "시험 입력", ignored: "명령 미전달", battery: "배터리",
  },
  en: {
    title: "Bluetooth remotes", close: "Close", scan: "Scan devices (30s)", on: "Enable Bluetooth", off: "Disable Bluetooth",
    help: "Configure while stationary with cruise disengaged. Enabling a mapping captures this device's original touch, keyboard and volume input. Lane changes retain existing speed, blind-spot and steering conditions. Phones can pair; calls, audio and networking are not provided.",
    empty: "Put your device in pairing mode and scan. Disconnect it from its previous phone if needed.",
    pair: "Pair", connect: "Connect", disconnect: "Disconnect", forget: "Forget", paired: "Paired", connected: "Connected",
    profile: "Input profile", generic: "Generic keyboard / HID", enabled: "Use Carrot mapping (capture original HID input)",
    save: "Save mapping", saved: "Saved", test: "Learn / test keys (120s)", stop: "End test", waiting: "Preparing test…",
    testing: "Test mode: no vehicle commands. Press and release a button to see it below. New keys are added to the list automatically.",
    idle: "Waiting for input", blocked: "Setup requires a fresh stationary, disengaged state and the input service.",
    grabbed: "Exclusive HID capture active", inactive: "Waiting for HID capture (check connection and mapping)",
    learnHelp: "Save the input profile and mapping before testing. For generic HID devices, press and release each button during the test to learn it. Save again to apply edited actions. Keyboard auto-repeat is ignored. Use short individual presses on Yiser-J6.",
    none: "No action", accelCruise: "Cruise + (accelCruise)", decelCruise: "Cruise − (decelCruise)", laneLeft: "Lane change left", laneRight: "Lane change right",
    paddleDecel: "Paddle deceleration", gapAdjustCruise: "Cruise gap", up: "Up", down: "Down", left: "Left", right: "Right", center: "Center", "1": "1", "2": "2",
    cancel: "Cancel", confirm: "Confirm", pin: "PIN / passkey", prompt: "Pairing confirmation", display: "Enter the code shown on the other device, or verify the code below.",
    scanning: "Scanning", ready: "Ready", unavailable: "Bluetooth adapter unavailable", noService: "Input service is not running",
    unsaved: "Save your changes", sent: "Vehicle command sent", testEvent: "Test input", ignored: "Command not sent", battery: "Battery",
  },
};
const DEFAULTS = { up: "accelCruise", down: "decelCruise", left: "laneLeft", right: "laneRight", center: "paddleDecel", "1": "gapAdjustCruise", "2": "none" };
const ACTIONS = ["none", "accelCruise", "decelCruise", "laneLeft", "laneRight", "paddleDecel", "gapAdjustCruise"];
let dialog, state, selected, draft, timer, devicesSignature, busy = false, lastEvent, promptId, dirty = false;
const t = (key) => WORDS[typeof LANG !== "undefined" && LANG === "ko" ? "ko" : "en"][key] || key;
const element = (tag, text, className) => { const e = document.createElement(tag); if (text != null) e.textContent = text; if (className) e.className = className; return e; };
const part = (id) => dialog.querySelector(`[data-bt="${id}"]`);
const region = (id, tag = "div") => { const e = element(tag); e.dataset.bt = id; return e; };
function button(label, action, mutation = true) {
  const e = element("button", t(label), "btn"); e.type = "button";
  if (mutation) e.dataset.mutation = "true";
  e.onclick = mutation ? () => run(action) : action; return e;
}
async function api(operation, data) {
  const response = await fetch(`/api/bluetooth${operation ? `/${operation}` : ""}`, operation ? {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(data || {}),
  } : { cache: "no-store" });
  if (!response.ok) throw new Error(await response.text());
  return response.json();
}
async function run(action) {
  if (busy) return;
  busy = true; lock(); part("error").textContent = "";
  try { await action(); await refresh(); } catch (error) { part("error").textContent = error.message; }
  finally { busy = false; lock(); }
}
function lock() {
  dialog?.querySelectorAll('[data-mutation="true"]').forEach(e => { e.disabled = busy || !state?.runtime?.stationary; });
}
function selectDevice(device) {
  selected = device.address;
  const stored = state.config.devices[selected];
  draft = stored ? structuredClone(stored) : { name: device.name, profile: /yiser-j6/i.test(device.name) ? "yiser-j6" : "generic", enabled: false, mapping: {} };
  if (!stored && draft.profile === "yiser-j6") draft.mapping = { ...DEFAULTS };
  dirty = false; lastEvent = state.runtime.last_event?.id; renderEditor();
}
function renderEditor() {
  const editor = part("editor"); editor.replaceChildren(); if (!draft) return;
  editor.append(element("h3", `${draft.name || selected} · ${selected}`));
  const label = element("label", t("profile")), select = element("select");
  for (const [value, text] of [["yiser-j6", "Yiser-J6"], ["generic", t("generic")]]) { const option = element("option", text); option.value = value; select.append(option); }
  select.value = draft.profile; select.dataset.mutation = "true";
  select.onchange = () => { draft.profile = select.value; draft.mapping = select.value === "yiser-j6" ? { ...DEFAULTS } : {}; dirty = true; renderEditor(); };
  label.append(select); editor.append(label);
  const enabledLabel = element("label"), enabled = element("input"); enabled.type = "checkbox"; enabled.checked = draft.enabled; enabled.dataset.mutation = "true";
  enabled.onchange = () => { draft.enabled = enabled.checked; dirty = true; }; enabledLabel.append(enabled, document.createTextNode(t("enabled"))); editor.append(enabledLabel);
  const rows = region("mapping"); editor.append(rows); renderMappings();
  const actions = element("div", null, "bt-actions");
  actions.append(button("save", async () => {
    await api("config", { version: 1, devices: { ...state.config.devices, [selected]: draft } });
    dirty = false; part("notice").textContent = t("saved");
  }), button("test", async () => {
    if (dirty || !state.config.devices[selected]) throw new Error(t("unsaved"));
    lastEvent = state.runtime.last_event?.id;
    await api("learn", { address: selected, enabled: true }); part("notice").textContent = t("waiting");
  }), button("stop", async () => { await api("learn", { address: selected, enabled: false }); }));
  editor.append(actions, element("p", t("learnHelp")), region("notice"), region("capture"), region("event", "output")); lock();
}
function renderMappings() {
  const rows = part("mapping"); rows.replaceChildren();
  for (const [token, action] of Object.entries(draft.mapping)) {
    const label = element("label", t(token)), select = element("select"); select.dataset.mutation = "true";
    for (const value of ACTIONS) { const option = element("option", t(value)); option.value = value; select.append(option); }
    select.value = action; select.onchange = () => { draft.mapping[token] = select.value; dirty = true; };
    label.append(select); rows.append(label);
  }
  lock();
}
function renderDevices() {
  const devices = [...state.devices].sort((a, b) => Number(b.paired) - Number(a.paired) || a.address.localeCompare(b.address));
  const signature = JSON.stringify(devices.map(({ address, name, paired, connected, battery }) => ({ address, name, paired, connected, battery })));
  if (signature === devicesSignature) return;
  devicesSignature = signature;
  const list = part("devices"); list.replaceChildren();
  if (!state.devices.length) list.append(element("p", t("empty")));
  for (const device of devices) {
    const card = element("div", null, "bt-device"), name = element("strong", device.name || device.address);
    card.append(name, element("div", `${device.address} · ${device.connected ? t("connected") : device.paired ? t("paired") : ""}${device.battery != null ? ` · ${t("battery")} ${device.battery}%` : ""}`));
    const actions = element("div", null, "bt-actions");
    if (!device.paired) actions.append(button("pair", () => api("pair", { address: device.address })));
    else {
      actions.append(button(device.connected ? "disconnect" : "connect", () => api(device.connected ? "disconnect" : "connect", { address: device.address })),
        button("forget", async () => {
          await api("forget", { address: device.address });
          if (selected === device.address) { selected = draft = null; renderEditor(); }
        }), button("profile", () => selectDevice(device), false));
    }
    card.append(actions); list.append(card);
  }
}
function renderPrompt() {
  const prompt = state.prompt;
  if ((prompt?.id || null) === promptId) return;
  promptId = prompt?.id || null; const box = part("prompt"); box.replaceChildren();
  if (!prompt) return;
  box.append(element("h3", t("prompt")), element("p", t("display")), element("strong", prompt.value));
  let input;
  if (["RequestPinCode", "RequestPasskey"].includes(prompt.kind)) {
    input = element("input"); input.placeholder = t("pin"); input.setAttribute("aria-label", t("pin")); input.maxLength = prompt.kind === "RequestPasskey" ? 6 : 16; box.append(input);
  }
  if (!prompt.kind.startsWith("Display")) box.append(button("confirm", () => api("answer", { id: prompt.id, value: input ? input.value : true })));
  box.append(button("cancel", () => api("cancel")));
}
async function refresh() {
  if (!dialog?.open) return;
  state = await api();
  part("status").textContent = !state.runtime.stationary ? t("blocked") : state.available ? (state.adapters.some(a => a.discovering) ? t("scanning") : t("ready")) : t("unavailable");
  part("radio").textContent = t(state.radioEnabled ? "off" : "on");
  part("service").textContent = [state.error, !state.runtime.alive && t("noService"), ...Object.values(state.runtime.errors || {}), state.pair?.error].filter(Boolean).join(" · ");
  renderDevices(); renderPrompt();
  const learning = state.runtime.learning;
  if (draft) {
    part("capture").textContent = (state.runtime.grabbed || []).includes(selected) ? t("grabbed") : t("inactive");
    part("notice").textContent = learning?.address === selected ? `${t("testing")} (${Math.max(0, Math.ceil(learning.until - state.runtime.time))}s)` : dirty ? t("unsaved") : "";
    const event = state.runtime.last_event;
    if (event?.address === selected && event.id !== lastEvent) {
      lastEvent = event.id;
      part("event").textContent = `${t(event.button)} → ${t(event.action)} · ${t(event.reason === "test" ? "testEvent" : event.emitted ? "sent" : "ignored")}`;
      if (event.reason === "test" && learning?.address === selected && !(event.button in draft.mapping)) {
        draft.mapping[event.button] = "none"; dirty = true; renderMappings();
      }
    }
  }
  lock();
}
function open() {
  if (dialog?.open) return;
  dialog?.remove(); state = selected = draft = promptId = devicesSignature = null; dirty = false;
  dialog = element("dialog", null, "carrot-bt-dialog"); dialog.setAttribute("aria-label", t("title"));
  const header = element("div", null, "bt-actions"); header.append(element("h2", t("title")), button("close", () => dialog.close(), false));
  const actions = element("div", null, "bt-actions"), radio = button("on", () => api("radio", { enabled: !state.radioEnabled })); radio.dataset.bt = "radio";
  actions.append(radio, button("scan", () => api("scan")), button("cancel", () => api("cancel")));
  dialog.append(header, element("p", t("help")), region("status"), region("service"), region("error"), actions, region("prompt"), region("devices"), region("editor"));
  part("error").setAttribute("role", "alert"); part("status").setAttribute("role", "status");
  dialog.addEventListener("close", () => { clearTimeout(timer); /* Test expiry is explicit; closing never silently re-enables commands. */ });
  document.body.append(dialog); dialog.showModal(); lock();
  let pollFailed = false;
  const poll = async () => {
    try { await refresh(); if (pollFailed) part("error").textContent = ""; pollFailed = false; }
    catch (error) { pollFailed = true; part("error").textContent = error.message; }
    finally { if (dialog.open) timer = setTimeout(poll, 1000); }
  }; poll();
}
globalThis.CarrotBluetooth = { init() { const button = document.getElementById("btnToolsBluetooth"); if (button) { button.textContent = t("title"); button.onclick = open; } } };
