"use strict";

/* 인트로 레지스트리
 *
 * step 파일들이 여기에 자기를 등록한다. 셸(intro.js)이 마지막에 로드되어
 * 등록된 것들을 굴린다. app.js 가 마지막에 도는 기존 패턴과 같다.
 *
 * step 계약:
 *   {
 *     id:      "car",                 고유 id. 이동은 전부 이 id 로 한다.
 *     file:    "step2_car.js",        (개발 표시용)
 *     flow:    true,                  본 흐름에 속하는가 (restore 는 false)
 *     hkgOnly: true,                  현대·기아·제네시스일 때만 (선택)
 *     render(el) -> cleanup?          el 은 .intro-col. 정리 함수를 반환할 수 있다.
 *   }
 *
 * 일관성은 각 step 이 아니라 셸이 강제한다: 카드/전환/타이포/버튼은
 * 전부 셸과 intro.css 소유. step 은 컨텐츠만 그린다.
 */

globalThis.CarrotIntro = (() => {
  const steps = [];

  /* 마법사가 모으는 값. 마지막에 서버로 넘어간다.
     lang 기본값은 브라우저 언어 감지 결과 — 못 알아보면 영어.
     (detectCarrotIntroLang — intro_i18n.js) */
  const ctx = {
    lang: detectCarrotIntroLang(),
    preview: false, // 터미널에서 연 실제 흐름의 dry-run. 장치/브라우저 상태를 쓰지 않는다.
    car: "",
    maker: "",
    hda: null,     // 0 = HDA1, 1 = HDA2, null = 미선택/비HKG
    preset: null,  // "radar_long" | "camera_long" | "stock"
    restored: false, // 백업 복원으로 들어왔나 (프리셋 적용을 건너뛴다)
  };

  function reset() {
    Object.assign(ctx, {
      lang: detectCarrotIntroLang(),
      preview: false,
      car: "", maker: "", hda: null, preset: null, restored: false,
    });
  }

  /* 현대·기아·제네시스 판정.
     /api/cars 의 makers 키는 car_docs 이름의 첫 토큰이다
     (features/cars.py + utils.js:7 주석 참조).
     ⚠ 디바이스에서 GET /api/cars 응답으로 실제 키를 확인해야 확정된다. */
  const HKG_MAKERS = new Set(["Hyundai", "Kia", "Genesis"]);
  const isHkg = () => HKG_MAKERS.has(ctx.maker);

  /* 프리셋 값은 여기 없다. server/features/intro/presets.py 가 단일 진실이고
     클라이언트는 이름("radar_long" | "camera_long" | "stock")만 보낸다.
     값을 두 곳에 두면 캐시된 구버전 JS 가 낡은 값을 쓰게 된다. */
  const PRESET_NAMES = ["radar_long", "camera_long", "stock"];

  function register(step) {
    if (!step || !step.id) throw new TypeError("CarrotIntro.register: id 필요");
    if (typeof step.render !== "function") throw new TypeError(`CarrotIntro.register: ${step.id}.render 필요`);
    if (steps.some((s) => s.id === step.id)) throw new Error(`CarrotIntro.register: id 중복 — ${step.id}`);
    steps.push(step);
  }

  const get = (id) => steps.find((s) => s.id === id) || null;

  /* 진행 표시용 — 현재 조건에서 실제로 거치게 될 단계들 */
  const flow = () => steps.filter((s) => s.flow && (!s.hkgOnly || isHkg()));

  /* 문구 */
  const t = (key) => {
    const pack = CarrotIntroI18n[ctx.lang] || CarrotIntroI18n.ko;
    return pack[key] != null ? pack[key] : (CarrotIntroI18n.ko[key] || key);
  };
  const br = (s) => String(s).replace(/\n/g, "<br>");

  /* 로고는 selfdrive/assets/img_spinner_comma.png 다.
     config.SOUND_ASSETS_DIR 가 그 디렉터리를 가리키고 static.py 가
     /sound-assets/ 로 서빙하므로 신규 라우트가 필요 없다. */
  const assetUrl = (name) => `/sound-assets/${name}`;

  /* 선택한 언어를 문서에 반영한다. 안 하면 <html lang> 이 index.html 의
     값으로 고정되어, 영어를 골라도 보조기술이 한국어 발음 규칙으로 읽는다. */
  function applyLang() {
    try {
      document.documentElement.lang = ctx.lang;
    } catch (_) {}
  }

  return { steps, register, get, flow, ctx, reset, isHkg, PRESET_NAMES, t, br, assetUrl, applyLang };
})();

/* ═══════════════════════════════════════════════════════════
   아이콘 — SVG 신규 제작.
   레포에 자동차 일러스트가 없어 새로 그렸다. 로고만 원본 PNG 를 쓴다
   (손그림 캐릭터라 SVG 재현 불가, 이미 /sound-assets/ 로 서빙 중).

   ── 공통 규칙 (일관성의 근거) ──
   · viewBox 0 0 120 120, 모든 선은 stroke-width 5 / round cap·join
   · 내용은 x·y 20~100 안에 두고, 광학 중심을 (60, 60~63) 에 맞춘다
   · .stroke = currentColor(--md-primary), .soft = --md-outline-var(맥락)
   · 채우기(fill) 금지 — 선 도형과 면 도형을 섞으면 무게가 안 맞는다
   · 떠 있는 조각 금지 — 모든 선은 다른 선에 닿거나 명확한 의미를 갖는다

   ── 2026-07-17 전면 재작업. 이전 버전의 실제 결함 ──
   car     바퀴(x42·78)와 다른 위치(x30·90)에 다리가 매달려 다리가 4개였고,
           차대선이 바퀴를 관통했다.
   hda     화살표 위에 아무데도 안 붙은 사각형이 떠 있었다.
   ctrl    스포크가 림에도 허브에도 닿지 않고 공중에 떠 있었다
           (34,78 은 림 안쪽 / 48,70 은 허브 바깥).
   chill   6px 짜리 눈이 stroke 5 라 뭉개졌고, 주행 앱에 스마일은 톤이 안 맞았다.
   restore 채운 삼각형과 선 도형을 섞어 무게가 어긋났고,
           undo 와 history 아이콘을 섞어놓아 뜻이 흐렸다.
   ═══════════════════════════════════════════════════════════ */
globalThis.CarrotIntroIcons = {
  /* 차량 선택 — 측면 실루엣.
     차대선은 바퀴 자리(34~50, 70~86)를 비워 바퀴가 그 틈을 메운다. */
  car: `<svg class="intro-icon" viewBox="0 0 120 120" aria-hidden="true">
    <path class="stroke" d="M22 72V60l10-3 9-13h38l9 13 10 3v12"/>
    <path class="stroke" d="M22 72h12M50 72h20M86 72h12"/>
    <circle class="stroke" cx="42" cy="72" r="8"/>
    <circle class="stroke" cx="78" cy="72" r="8"/>
    <path class="soft" d="M32 57h56"/></svg>`,

  /* HDA — 원근으로 좁아지는 차로 + 차로 한가운데로 나아가는 진행선.
     차로유지보조가 하는 일 그대로. */
  hda: `<svg class="intro-icon" viewBox="0 0 120 120" aria-hidden="true">
    <path class="soft" d="M30 104L44 26"/>
    <path class="soft" d="M90 104L76 26"/>
    <path class="stroke" d="M60 98V50"/>
    <path class="stroke" d="M48 62l12-12 12 12"/></svg>`,

  /* 주행제어 — 내 차(아래)가 앞차(위)를 감지해 차간거리를 잡는 모습.
     이 단계는 가감속(롱컨) 방식을 고르는 자리이므로 조향(핸들)이 아니라
     차간거리로 그린다. */
  ctrl: `<svg class="intro-icon" viewBox="0 0 120 120" aria-hidden="true">
    <rect class="soft" x="46" y="20" width="28" height="20" rx="5"/>
    <path class="stroke" d="M46 80A14 14 0 0 1 74 80"/>
    <path class="stroke" d="M36 80A24 24 0 0 1 84 80"/>
    <rect class="stroke" x="44" y="82" width="32" height="24" rx="6"/></svg>`,

  /* 마무리 — 확인. 스마일 대신 체크. */
  chill: `<svg class="intro-icon" viewBox="0 0 120 120" aria-hidden="true">
    <circle class="stroke" cx="60" cy="60" r="34"/>
    <path class="stroke" d="M45 61l11 11 21-25"/></svg>`,

  /* 설정 불러오기 — 백업에서 받아 담는다. 선 도형만. */
  restore: `<svg class="intro-icon" viewBox="0 0 120 120" aria-hidden="true">
    <path class="stroke" d="M60 24v42"/>
    <path class="stroke" d="M43 50l17 16 17-16"/>
    <path class="stroke" d="M26 80v10c0 3 3 6 6 6h56c3 0 6-3 6-6V80"/></svg>`,

  /* 법적 고지 — 문서 + 느낌표. 모서리 접힘은 문서 외곽선과 맞물린다.
     경고 삼각형을 쓰지 않은 이유: 삼각형은 "위험"이고 이건 "고지"다. */
  legal: `<svg class="intro-icon" viewBox="0 0 120 120" aria-hidden="true">
    <path class="stroke" d="M34 22h32l24 24v52a4 4 0 0 1-4 4H34a4 4 0 0 1-4-4V26a4 4 0 0 1 4-4z"/>
    <path class="soft" d="M66 22v24h24"/>
    <path class="stroke" d="M60 58v18"/>
    <path class="stroke" d="M60 88v.01"/></svg>`,
};
