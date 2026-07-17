"use strict";

/* 인트로 셸
 *
 * step 파일이 전부 등록된 뒤 마지막에 로드된다 (app.js 와 같은 패턴).
 *
 * 셸이 소유: pane 생성/파괴, 전환, 개막/폐막, 선택 피드백, 이전 이동.
 * step 이 소유: 컨텐츠뿐.
 * → 일관성이 각 step 의 성의에 달리지 않는다.
 *
 * 이동 규칙:
 *   앞으로 — 선택했을 때만. 임의 전진 없음.
 *   뒤로   — 방문 기록(history)을 되짚어 언제든. step 이 onBack() 으로
 *            자기 안에서 먼저 처리할 수 있다 (예: 모델 목록 → 제조사 목록).
 *
 * ⚠ 이동은 전부 id 기반이다. 인덱스 산술 금지 —
 *   restore 가 배열 중간에 있어서 인덱스 +1 이 그리로 새는 버그가 있었다.
 */

globalThis.CarrotIntroShell = (() => {
  const deck = document.getElementById("introDeck");
  const curtain = document.getElementById("introCurtain");

  const CURTAIN_MS = 900;

  /* ── 전환: Material 3 "fade through" ──
     이전 판(2026-07-17)은 새 화면(520ms 진입)과 옛 화면(340ms 퇴장)을
     동시에 돌렸다. 둘 다 position:absolute 로 겹쳐 있어서 겹치는 구간에
     반투명한 두 벌의 글자가 서로 비쳐 보였다 — "전환이 이상한" 원인.

     이제 순서를 지킨다: 옛 화면이 완전히 사라지고 DOM 에서 빠진 뒤에야
     새 화면을 만든다. 어느 순간에도 화면에는 판이 하나뿐이다. */
  const OUT_MS = 150;
  const IN_MS = 330;

  const PICK_HOLD_MS = 130;
  const EASE_IN = "cubic-bezier(0.2, 0, 0, 1)";        /* --ease-emphasized */
  const EASE_OUT = "cubic-bezier(0.3, 0, 0.8, 0.15)";  /* --ease-emphasized-accelerate */

  const BACK_ICON = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor"
    stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
    <path d="M15 18l-6-6 6-6"/></svg>`;

  let currentId = "welcome";
  let history = [];        // 방문한 step id 스택 (현재 것은 안 들어있다)
  let pane = null;
  let cleanup = null;
  let backHandler = null;  // 현재 step 의 onBack (있으면)
  let busy = false;
  let closing = false;
  let previewDocumentLang = null;

  /* ── pane 만들기 ──────────────────────────────────────── */
  function build() {
    if (cleanup) { cleanup(); cleanup = null; }
    backHandler = null;

    const step = CarrotIntro.get(currentId);
    if (!step) throw new Error(`CarrotIntroShell: 없는 step — ${currentId}`);

    const p = document.createElement("div");
    p.className = "intro-pane";
    p.dataset.introStep = currentId;

    const col = document.createElement("div");
    col.className = "intro-col";
    /* 중국어는 word-break 규칙이 달라 lang 을 심는다 (intro.css 참조) */
    if (CarrotIntro.ctx.lang === "zh") col.setAttribute("lang", "zh");
    p.appendChild(col);

    const back = document.createElement("button");
    back.type = "button";
    back.className = "intro-back";
    back.innerHTML = `${BACK_ICON}<span>${CarrotIntro.t("back")}</span>`;
    back.addEventListener("click", goBack);
    p.appendChild(back);

    if (CarrotIntro.ctx.preview) {
      const note = document.createElement("div");
      note.className = "intro-preview-note";
      note.textContent = CarrotIntro.t("preview");
      p.appendChild(note);

      const close = document.createElement("button");
      close.type = "button";
      close.className = "intro-close";
      close.textContent = CarrotIntro.t("previewClose");
      close.addEventListener("click", finish);
      p.appendChild(close);
    }

    const result = step.render(col) || null;
    /* render 는 cleanup 함수만 반환하거나, { cleanup, onBack } 을 반환할 수 있다 */
    if (typeof result === "function") {
      cleanup = result;
    } else if (result && typeof result === "object") {
      cleanup = result.cleanup || null;
      backHandler = result.onBack || null;
    }

    /* 첫 화면에서는 돌아갈 데가 없다 */
    back.hidden = history.length === 0 && !backHandler;

    deck.appendChild(p);
    pane = p;
    return p;
  }

  /* ── 이동 ─────────────────────────────────────────────── */
  function transition(nextId, { push = true, reverse = false } = {}) {
    if (busy) return;
    if (!CarrotIntro.get(nextId)) {
      console.error(`CarrotIntroShell: 없는 id — ${nextId}`);
      return;
    }

    busy = true;
    const old = pane;

    /* 2단계: 옛 화면을 지우고 → 새 화면을 만든다. 겹치지 않는다. */
    const swap = () => {
      if (old) old.remove();
      if (push) history.push(currentId);
      currentId = nextId;
      const next = build();

      next.animate(
        [{ opacity: 0, transform: `translateY(${reverse ? -14 : 14}px)` },
         { opacity: 1, transform: "translateY(0)" }],
        { duration: IN_MS, easing: EASE_IN, fill: "both" },
      ).finished.then(release).catch(release);
    };
    const release = () => { busy = false; };

    if (!old) { swap(); return; }

    old.animate(
      [{ opacity: 1, transform: "translateY(0)" },
       { opacity: 0, transform: `translateY(${reverse ? 10 : -10}px)` }],
      { duration: OUT_MS, easing: EASE_OUT, fill: "both" },
    ).finished.then(swap).catch(swap);
  }

  /* 앞으로 — 선택했을 때만 불린다 */
  const goTo = (nextId) => transition(nextId, { push: true, reverse: false });

  /* 뒤로 — step 이 자기 안에서 먼저 처리할 기회를 준다 */
  function goBack() {
    if (busy) return;
    if (backHandler && backHandler() === true) return;  // step 이 삼켰다
    if (!history.length) return;
    const prev = history.pop();
    transition(prev, { push: false, reverse: true });
  }

  /* 선택 = 진행. 고른 것을 잠깐 보여준 뒤 넘어간다. */
  function pick(el, nextId) {
    if (busy) return;
    busy = true;
    pane.querySelectorAll(".intro-opt").forEach((x) => x.classList.remove("is-picked"));
    el.classList.add("is-picked");
    setTimeout(() => { busy = false; goTo(nextId); }, PICK_HOLD_MS);
  }

  /* ── 개막 / 폐막 ──────────────────────────────────────── */
  function open({ preview = false } = {}) {
    if (!deck.hidden) return false;

    deck.hidden = false;
    document.body.dataset.introOpen = "1";

    CarrotIntro.reset();
    CarrotIntro.ctx.preview = preview === true;
    if (CarrotIntro.ctx.preview) {
      previewDocumentLang = document.documentElement.lang || "";
      document.body.dataset.introPreview = "1";
    } else {
      previewDocumentLang = null;
      delete document.body.dataset.introPreview;
    }
    currentId = "welcome";
    history = [];
    deck.innerHTML = "";
    pane = null;
    cleanup = null;
    backHandler = null;
    busy = false;
    closing = false;

    /* 첫 판에만 순차 등장(stagger)을 붙인다. 판 자체에 표시하므로
       타이머로 클래스를 걷어낼 필요가 없다 — 이전 판은 deck 에 걸어두고
       1300ms 뒤 제거했는데, 그 전에 사용자가 언어를 고르면 다음 판까지
       stagger 가 딸려가 전환이 엉켰다. */
    build().classList.add("is-first");

    curtain.className = "intro-curtain is-opening";
    return true;
  }

  function finish() {
    if (closing || deck.hidden) return;
    closing = true;
    const wasPreview = CarrotIntro.ctx.preview === true;
    if (cleanup) { cleanup(); cleanup = null; }
    curtain.className = "intro-curtain is-closing";
    setTimeout(() => {
      if (cleanup) { cleanup(); cleanup = null; }
      deck.innerHTML = "";
      deck.hidden = true;
      delete document.body.dataset.introOpen;
      delete document.body.dataset.introPreview;
      curtain.className = "intro-curtain";
      closing = false;

      if (wasPreview) {
        if (previewDocumentLang) document.documentElement.lang = previewDocumentLang;
        previewDocumentLang = null;
        CarrotIntro.reset();
        return;
      }
      /* 인트로가 끝나고 나서야 앱의 첫 페이지를 띄운다.
         커튼이 덮인 동안 붙으므로 전환이 튀지 않는다. */
      try {
        window.bootstrapWebStartPage?.("intro");
      } catch (err) {
        console.error("[intro] bootstrapWebStartPage failed", err);
      }
    }, CURTAIN_MS);
  }

  /* 키보드 — 보이는 크롬이 없으므로 접근성용 최소한만.
     앞으로 가는 키는 없다 (임의 전진 금지). */
  document.addEventListener("keydown", (e) => {
    if (deck.hidden) return;
    if (e.key === "Escape" && CarrotIntro.ctx.preview && history.length === 0 && !backHandler) {
      e.preventDefault();
      finish();
      return;
    }
    if (e.key === "Escape" || e.key === "ArrowLeft") { e.preventDefault(); goBack(); }
  });

  /* 오류는 인트로 안에 띄운다.
     기존 토스트(.app-toast-host)는 z-index 150 인데 인트로 무대는
     --z-overlay(220) 라 인트로 뒤에 가려서 보이지 않는다. */
  function error(message) {
    const col = pane && pane.querySelector(".intro-col");
    if (!col) return;
    let box = col.querySelector(".intro-error");
    if (!box) {
      box = document.createElement("p");
      box.className = "intro-error";
      box.setAttribute("role", "alert");
      col.appendChild(box);
    }
    box.textContent = message;
  }

  return { goTo, goBack, pick, open, finish, error };
})();

/* 여기서 열지 않는다. 게이트는 shared/ui/navigation.js 의
 * bootstrapWebStartPage() 안에 있다 — app.js 와 app_realtime.js 두 경로가
 * 모두 그 함수를 지나므로 게이트를 한 곳에만 두면 된다.
 *
 * 인트로가 끝나면 finish() 가 bootstrapWebStartPage("intro") 를 불러
 * 앱의 첫 페이지를 띄운다. */
