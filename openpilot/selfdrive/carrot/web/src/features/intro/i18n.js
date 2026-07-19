"use strict";

/* 인트로 문구
 *
 * 웹 UI 언어는 ko/en/zh 3종 (web_settings.py: WEB_LANGUAGES).
 * 줄바꿈(\n)은 언어마다 자연스러운 지점에 직접 넣는다.
 * 한 문장을 3개 언어에 기계적으로 맞추지 않는다.
 *
 * 설명은 한 줄로 끝낸다. 선택 화면에서 문단을 읽게 하지 않는다 —
 * 각 선택지가 답해야 하는 건 두 가지뿐이다: 하네스가 어디에 붙었나,
 * 가감속은 누가 하나.
 *
 * 예외는 legalBody 하나다. 이건 훑는 글이 아니라 읽는 글이라
 * 문단 배열로 둔다 (한 문자열에 \n\n 을 넣지 않는다 — 구조가 곧 문단). */

globalThis.CarrotIntroI18n = {
  ko: {
    restore: "기존 설정 불러오기",
    back: "이전",
    preview: "미리보기 · 저장되지 않음",
    previewClose: "닫기",

    carTitle: "차량을 선택하세요",
    carBackToList: "목록으로",
    carEmpty: "지원 차량 목록을 불러오지 못했어요.",

    hdaTitle: "HDA 종류",
    hda1: "HDA1",
    hda1d: "대부분의 CAN 차량",
    hda2: "HDA2",
    hda2d: "CAN-FD 차량",

    ctrlTitle: "주행제어 방식",
    ctrlSub: "하네스를 어디에 연결하셨나요?",

    /* ① 하네스가 ADAS/레이더에 있으면 코너레이더까지 쓴다 */
    p1: "ADAS · 레이더 롱컨",
    p1d: "ADAS·레이더에 연결 · 코너레이더까지",

    /* ② 당근파일럿 전용.
       근거 — opendbc/car/hyundai/interface.py:179 의 `# carrot` 분기:
       순정 openpilot 은 alpha_long 이 켜져야만 롱컨을 하는데,
       당근파일럿은 CAMERA_SCC 가 켜지면 그 게이트를 무시하고
       openpilotLongitudinalControl 을 강제로 True 로 만든다.
       SCC 메시지는 카메라 버스에서 읽는다 (carstate.py:228). */
    p2: "카메라 롱컨",
    p2d: "SCC 신호가 카메라로 가는 차량",

    /* ③ HyundaiCameraSCC=0 이라 롱컨 강제가 없고,
       SpeedFromPCM=2 로 커브/카메라 감속만 버튼으로 넣는다. */
    p3: "순정",
    p3d: "가감속은 순정 SCC · 조향만",

    legalTitle: "법적 고지",
    legalBody: [
      "대한민국 자동차관리법 개정안에 따라, 본 소프트웨어를 실제 차량에 장착하거나 주행에 사용하는 것은 법률에 위배될 수 있습니다.",
      "이 저장소에 있는 모든 소프트웨어는 연구, 실험, 시뮬레이션 목적으로만 제공됩니다.",
      "개발자는 본 소프트웨어의 실제 사용으로 인해 발생하는 모든 법적 책임을 지지 않습니다.",
    ],
    legalAck: "확인했습니다",

    outTitle: "준비됐어요",
    outSub: "궁금한 점은 언제든\n당근서버에 물어보세요.",
    start: "시작하기",
    previewDone: "미리보기 닫기",

    rsTitle: "설정 불러오기",
    rsSub: "이전 기기의 백업에서 그대로.",
    rsFile: "백업 파일 선택",
    rsFiled: "params_backup.json",
    rsDone: "설정을 가져왔어요.",
  },

  en: {
    restore: "Restore my settings",
    back: "Back",
    preview: "Preview · Not saved",
    previewClose: "Close",

    carTitle: "Choose your car",
    carBackToList: "Back to list",
    carEmpty: "Could not load the supported car list.",

    hdaTitle: "HDA generation",
    hda1: "HDA1",
    hda1d: "Most CAN cars",
    hda2: "HDA2",
    hda2d: "CAN-FD cars",

    ctrlTitle: "Driving control",
    ctrlSub: "Where is your harness connected?",

    p1: "ADAS · radar long",
    p1d: "On ADAS or radar · corner radar too",

    p2: "Camera long",
    p2d: "On the camera only · CarrotPilot drives accel",

    p3: "Stock",
    p3d: "Stock SCC drives accel · steering only",

    legalTitle: "Legal notice",
    legalBody: [
      "Under the amended Motor Vehicle Management Act of the Republic of Korea, installing this software in an actual vehicle or using it while driving may violate the law.",
      "All software in this repository is provided for research, experimentation, and simulation purposes only.",
      "The developers accept no legal liability whatsoever arising from actual use of this software.",
    ],
    legalAck: "I understand",

    outTitle: "You're all set",
    outSub: "Questions? Just ask\non the Carrot server.",
    start: "Start",
    previewDone: "Close preview",

    rsTitle: "Restore settings",
    rsSub: "Straight from a previous device.",
    rsFile: "Choose backup file",
    rsFiled: "params_backup.json",
    rsDone: "Settings restored.",
  },

  zh: {
    restore: "导入已有设置",
    back: "返回",
    preview: "预览 · 不保存",
    previewClose: "关闭",

    carTitle: "请选择车辆",
    carBackToList: "返回列表",
    carEmpty: "无法加载支持的车辆列表。",

    hdaTitle: "HDA 代次",
    hda1: "HDA1",
    hda1d: "多数 CAN 车型",
    hda2: "HDA2",
    hda2d: "CAN-FD 车型",

    ctrlTitle: "驾驶控制方式",
    ctrlSub: "线束连接在哪里？",

    p1: "ADAS · 雷达纵向",
    p1d: "接至 ADAS 或雷达 · 含角雷达",

    p2: "摄像头纵向",
    p2d: "仅接至摄像头 · 当根领航负责加减速",

    p3: "原厂",
    p3d: "加减速交给原厂 SCC · 仅转向",

    legalTitle: "法律声明",
    legalBody: [
      "根据大韩民国《汽车管理法》修正案，将本软件安装于实际车辆或用于行驶可能违反法律。",
      "本仓库中的所有软件仅供研究、实验与仿真用途。",
      "开发者对因实际使用本软件而产生的一切法律责任概不负责。",
    ],
    legalAck: "我已知悉",

    outTitle: "准备就绪",
    outSub: "有任何疑问\n欢迎在当根服务器提问。",
    start: "开始",
    previewDone: "关闭预览",

    rsTitle: "导入设置",
    rsSub: "直接从旧设备的备份导入。",
    rsFile: "选择备份文件",
    rsFiled: "params_backup.json",
    rsDone: "设置已导入。",
  },
};

/* 0단계에서 순환하며 보여주고, 그대로 선택지가 되는 목록 */
globalThis.CarrotIntroLangs = [
  ["en", "English", "Welcome!"],
  ["ko", "한국어",  "오셨군요!"],
  ["zh", "中文",    "欢迎!"],
];

/* 브라우저 언어 감지 → 기본값. 못 알아보면 영어.
 * navigator.languages 를 순서대로 훑어 처음 맞는 것을 쓴다.
 *   ko, ko-KR        → ko
 *   zh, zh-CN, zh-TW → zh
 *   그 외             → en  */
globalThis.detectCarrotIntroLang = function detectCarrotIntroLang() {
  const supported = new Set(CarrotIntroLangs.map(([code]) => code));
  const tags = [
    ...(Array.isArray(navigator.languages) ? navigator.languages : []),
    navigator.language || "",
  ];
  for (const tag of tags) {
    const base = String(tag).toLowerCase().split("-")[0];
    if (supported.has(base)) return base;
  }
  return "en";
};
