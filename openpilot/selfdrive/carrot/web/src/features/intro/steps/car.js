"use strict";

/* 2단계 — 차량 선택 (제조사 → 모델)
 *
 * 기존 웹의 차량 선택과 같은 코드를 쓴다. 복제본이 아니다.
 * car.js 는 index.html 에서 인트로보다 먼저 로드되므로 아래를 그대로 쓴다:
 *
 *   ensureCarsLoaded()      car.js:355 — /api/cars 를 받아 CARS 전역에 채운다.
 *                           인트로가 미리 받아두면 나중에 사용자가 차량 페이지를
 *                           열 때도 이미 캐시돼 있다 (같은 CARS 를 공유).
 *   CARS                    shared/utils.js:7 — { ok, sources, makers }
 *   stripMaker(full, mk)    car.js:476 — "Hyundai Sonata 2020" → "Sonata 2020"
 *   applyCurrentCarLabel()  car.js:38 — 앱의 차량 라벨과 localStorage 캐시 갱신
 *   setParam()              shared/api.js:10 — carrot:paramchange 이벤트까지 발행
 *   escapeHtml()            shared/utils.js:35
 *
 * onSelectCar()(car.js:484)는 쓰지 않는다. 그건 차량 피커 전용이라 확인
 * 대화상자를 띄우고 곧바로 재부팅을 묻는다. 인트로는 재부팅을 마지막에
 * 한 번만 다룬다. 대신 그 함수가 하는 param 쓰기와 라벨 갱신은 같은
 * 함수로 그대로 수행한다.
 *
 * 목록 형식은 features/cars.py 가 주는 그대로다:
 *   { "Hyundai": ["Hyundai Elantra 2017-18", ...], "Kia": [...], ... }
 * 제조사 = 이름의 첫 토큰. 연식이 범위인 항목("2023-2024")도 그대로 나온다.
 *
 * 현대·기아·제네시스면 3단계(HDA)로, 아니면 4단계(주행제어)로. */

/* 0단계에서 불러 미리 받아둔다. 사용자가 언어를 고르는 동안 끝나므로
   2단계 진입 때 목록이 뒤늦게 채워지며 튀지 않는다. */
globalThis.prefetchCarrotIntroCars = () => {
  try {
    return Promise.resolve(ensureCarsLoaded()).catch(() => {});
  } catch (_) {
    return Promise.resolve();
  }
};

CarrotIntro.register({
  id: "car",
  file: "step2_car.js",
  flow: true,

  render(el) {
    const { t, ctx, isHkg } = CarrotIntro;

    /* 뼈대는 한 번만 만든다. 이후 제목 텍스트와 목록 내용만 바꾼다 —
       목록이 도착했다고 아이콘·제목까지 다시 그리면 진입 애니메이션 중에
       화면이 통째로 갈아끼워져 튄다. */
    el.innerHTML = `
      ${CarrotIntroIcons.car}
      <h2 class="intro-title" data-title>${t("carTitle")}</h2>
      <div class="intro-list" data-list></div>`;

    const title = el.querySelector("[data-title]");
    const list = el.querySelector("[data-list]");

    let mode = "makers";   // "makers" | "models"
    let alive = true;

    const makersOf = () => (CARS && CARS.makers) || {};

    function drawMakers() {
      mode = "makers";
      title.textContent = t("carTitle");

      const makers = makersOf();
      const names = Object.keys(makers).sort((a, b) => a.localeCompare(b));
      if (!names.length) {
        list.innerHTML = `<div class="intro-empty">${t("carEmpty")}</div>`;
        return;
      }

      list.innerHTML = names.map((mk) => `
        <button class="intro-opt" type="button" data-maker="${escapeHtml(mk)}">
          <span>${escapeHtml(mk)}</span>
          <span class="intro-list__count">${(makers[mk] || []).length}</span>
        </button>`).join("");

      list.querySelectorAll("[data-maker]").forEach((opt) => {
        opt.addEventListener("click", () => drawModels(opt.dataset.maker));
      });
      list.scrollTop = 0;
    }

    function drawModels(mk) {
      mode = "models";
      title.textContent = mk;

      /* 목록 첫 줄에 되돌아가는 행을 둔다. 상단 "이전" 은 화면 구석에
         고정이라 목록을 스크롤하고 있으면 손가락이 닿지 않는다. */
      const models = makersOf()[mk] || [];
      list.innerHTML = `
        <button class="intro-opt intro-opt--up" type="button" data-up>
          ${t("carBackToList")}
        </button>` + models.map((full) => `
        <button class="intro-opt" type="button" data-car="${escapeHtml(full)}">
          <span>${escapeHtml(stripMaker(full, mk))}</span>
        </button>`).join("");

      list.querySelector("[data-up]").addEventListener("click", drawMakers);

      list.querySelectorAll("[data-car]").forEach((opt) => {
        opt.addEventListener("click", () => {
          const full = opt.dataset.car;
          ctx.car = full;
          ctx.maker = mk;
          if (!isHkg()) ctx.hda = null;   // 비HKG 는 CanfdHDA2 를 건드리지 않는다

          /* 정상 설치에서는 기존 공용 setParam/라벨 갱신을 그대로 쓰고,
             터미널 프리뷰에서는 같은 API 파사드가 장치 쓰기를 차단한다. */
          CarrotIntroApi.setCar(full, stripMaker(full, mk)).catch(() => {});

          CarrotIntroShell.pick(opt, isHkg() ? "hda" : "control");
        });
      });
      list.scrollTop = 0;
    }

    Promise.resolve(ensureCarsLoaded())
      .then(() => { if (alive) drawMakers(); })
      .catch((err) => {
        if (!alive) return;
        list.innerHTML = `<div class="intro-empty">${t("carEmpty")}<br>${escapeHtml(err?.message || "")}</div>`;
      });

    return {
      cleanup: () => { alive = false; },

      /* 모델 목록에서는 제조사 목록으로 돌아간다.
         true 를 돌려주면 셸이 단계 이동을 하지 않는다. */
      onBack: () => {
        if (mode === "models") { drawMakers(); return true; }
        return false;
      },
    };
  },
});
