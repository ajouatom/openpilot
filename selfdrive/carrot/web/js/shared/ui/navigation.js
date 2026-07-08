"use strict";

let pageTransitionTimer = null;
let pageTransitionToken = 0;
let CURRENT_PAGE = "carrot";
let settingScreenHideTimer = null;
let settingScreenTransitionToken = 0;
let carScreenHideTimer = null;
let carScreenTransitionToken = 0;


function disableViewportZoomGestures() {
  const preventGesture = (e) => e.preventDefault();

  ["gesturestart", "gesturechange", "gestureend"].forEach((type) => {
    document.addEventListener(type, preventGesture, { passive: false });
  });

  document.addEventListener("touchmove", (e) => {
    if (e.touches && e.touches.length > 1) e.preventDefault();
  }, { passive: false });

  let lastTouchEnd = 0;
  document.addEventListener("touchend", (e) => {
    const now = Date.now();
    if (now - lastTouchEnd <= 300) e.preventDefault();
    lastTouchEnd = now;
  }, { passive: false });
}

disableViewportZoomGestures();


/* ── Page transition primitives ─────────────────────────── */
function clearPageTransitionClasses(el) {
  if (!el) return;
  el.classList.remove(...PAGE_TRANSITION_CLASSES);
}

function resetPageRuntimeStyles(el) {
  if (!el) return;
  el.style.transition = "";
  el.style.transform = "";
  el.style.opacity = "";
  el.style.zIndex = "";
  el.style.willChange = "";
  el.style.position = "";
  el.style.top = "";
  el.style.left = "";
  el.style.right = "";
  el.style.width = "";
  el.style.visibility = "";
  el.style.pointerEvents = "";
  el.style.boxShadow = "";
}

function setPageRendered(el, rendered) {
  if (!el) return;
  el.hidden = !rendered;
  if (rendered) {
    el.removeAttribute("aria-hidden");
    el.style.display = "";
  } else {
    el.setAttribute("aria-hidden", "true");
    el.style.display = "none";
  }
}

function setDisplayedPage(page) {
  Object.entries(PAGE_ELEMENTS).forEach(([name, el]) => {
    if (!el) return;
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    setPageRendered(el, name === page);
  });
  if (settingScreenHost) settingScreenHost.style.minHeight = "";
  if (swipeContainer) swipeContainer.style.minHeight = "";
}

function clearPendingScreenHide(timerRef) {
  if (timerRef) {
    window.clearTimeout(timerRef);
  }
  return null;
}

function getSwipeTransition(fromPage, toPage) {
  const fromIdx = SWIPE_PAGES.indexOf(fromPage);
  const toIdx = SWIPE_PAGES.indexOf(toPage);
  if (fromIdx < 0 || toIdx < 0 || fromIdx === toIdx) return null;
  return toIdx > fromIdx ? "forward" : "backward";
}

function getSwipeViewportMetrics(host = swipeContainer) {
  if (!host) {
    return { host: null, top: 0, left: 0, width: window.innerWidth || 1 };
  }

  const styles = window.getComputedStyle(host);
  const paddingTop = parseFloat(styles.paddingTop) || 0;
  const paddingLeft = parseFloat(styles.paddingLeft) || 0;
  const paddingRight = parseFloat(styles.paddingRight) || 0;
  const width = Math.max((host.clientWidth || window.innerWidth || 1) - paddingLeft - paddingRight, 1);

  return { host, top: paddingTop, left: paddingLeft, width };
}

function pinSwipeLayer(el, metrics) {
  if (!el) return;
  el.style.position = "absolute";
  el.style.top = `${metrics.top}px`;
  el.style.left = `${metrics.left}px`;
  el.style.right = "auto";
  el.style.width = `${metrics.width}px`;
}

function updateSwipeFrameHeight(frame) {
  if (!frame?.host) return;
  const heights = [frame.fromEl, frame.toEl]
    .filter(Boolean)
    .map((el) => el.offsetHeight || 0);
  const maxHeight = Math.max(...heights, 0);
  frame.host.style.minHeight = maxHeight > 0 ? `${maxHeight}px` : "";
}

function prepareSwipeFrame(host, fromEl, toEl = null) {
  if (!host || !fromEl) return null;

  const metrics = getSwipeViewportMetrics(host);
  fromEl.style.display = "";
  fromEl.classList.add("page-transitioning", "page-active");
  fromEl.style.transition = "none";
  fromEl.style.willChange = "transform, opacity";
  pinSwipeLayer(fromEl, metrics);

  if (toEl) {
    toEl.style.display = "";
    toEl.classList.add("page-transitioning");
    toEl.style.transition = "none";
    toEl.style.willChange = "transform, opacity";
    pinSwipeLayer(toEl, metrics);
  }

  const frame = { host, fromEl, toEl, metrics, width: metrics.width };
  updateSwipeFrameHeight(frame);
  return frame;
}

function animatePageTransition(fromPage, toPage, transition, onComplete = null) {
  const fromEl = PAGE_ELEMENTS[fromPage];
  const toEl = PAGE_ELEMENTS[toPage];
  if (!swipeContainer || !fromEl || !toEl || fromEl === toEl || !transition) {
    setDisplayedPage(toPage);
    if (typeof onComplete === "function") onComplete();
    return null;
  }

  pageTransitionToken += 1;
  const token = pageTransitionToken;

  if (pageTransitionTimer) {
    clearTimeout(pageTransitionTimer);
    pageTransitionTimer = null;
  }

  const metrics = getSwipeViewportMetrics(swipeContainer);
  const frame = { host: swipeContainer, fromEl, toEl, metrics, width: metrics.width };

  // Both pages render simultaneously during the transition. Pin them
  // to the same host box before animation so page-specific gutters or
  // full-bleed rules can't resize the visible layer mid-transition.
  Object.values(PAGE_ELEMENTS).forEach((el) => {
    if (!el) return;
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    setPageRendered(el, el === fromEl || el === toEl);
  });

  fromEl.classList.add("page-transitioning", "page-active");
  fromEl.style.willChange = "transform, opacity";
  pinSwipeLayer(fromEl, metrics);

  toEl.classList.add(
    "page-transitioning",
    transition === "forward" ? "page-enter-from-right" : "page-enter-from-left",
  );
  toEl.style.willChange = "transform, opacity";
  pinSwipeLayer(toEl, metrics);
  updateSwipeFrameHeight(frame);

  // Force the initial frame so the browser commits the "from" state
  // before the transition rule kicks in.
  void toEl.offsetWidth;

  requestAnimationFrame(() => {
    requestAnimationFrame(() => {
      if (token !== pageTransitionToken) return;
      toEl.classList.add("page-active");
      toEl.classList.remove(
        transition === "forward" ? "page-enter-from-right" : "page-enter-from-left",
      );
      fromEl.classList.add(
        transition === "forward" ? "page-exit-to-left" : "page-exit-to-right",
      );
    });
  });

  pageTransitionTimer = setTimeout(() => {
    if (token !== pageTransitionToken) return;
    setDisplayedPage(toPage);
    pageTransitionTimer = null;
    if (typeof onComplete === "function") onComplete(token);
  }, PAGE_TRANSITION_MS);

  return token;
}

function stopPageTransition() {
  if (pageTransitionTimer) {
    clearTimeout(pageTransitionTimer);
    pageTransitionTimer = null;
  }
  pageTransitionToken += 1;
  setDisplayedPage(CURRENT_PAGE);
}

function setElementClass(el, className, enabled) {
  if (!el) return;
  if (el.classList.contains(className) !== enabled) {
    el.classList.toggle(className, enabled);
  }
}

function syncPageDataset(page) {
  if (document.documentElement.dataset.page !== page) {
    document.documentElement.dataset.page = page;
  }
  if (document.body.dataset.page !== page) {
    document.body.dataset.page = page;
  }
}

function syncNavActivePage(page) {
  setElementClass(btnHome, "active", page === "carrot");
  setElementClass(btnSetting, "active", page === "setting");
  setElementClass(btnTools, "active", page === "tools");
  setElementClass(btnLogs, "active", page === "logs");
  setElementClass(btnTuner, "active", page === "tuner");
  setElementClass(btnTerminal, "active", page === "terminal");
}

function shouldUseSettingSplitLayout(page = CURRENT_PAGE) {
  return Boolean(
    page === "setting" &&
    typeof isCompactLandscapeMode === "function" &&
    isCompactLandscapeMode()
  );
}

function syncSettingSplitLayoutClass(enabled = shouldUseSettingSplitLayout()) {
  const pageEl = PAGE_ELEMENTS.setting;
  if (pageEl) pageEl.classList.toggle("setting-layout-split", enabled);
  if (settingScreenHost) settingScreenHost.classList.toggle("setting-screen-host--split", enabled);

  if (pageEl) {
    if (enabled) pageEl.dataset.settingLayout = "split";
    else delete pageEl.dataset.settingLayout;
  }
  if (settingScreenHost) {
    if (enabled) settingScreenHost.dataset.settingLayout = "split";
    else delete settingScreenHost.dataset.settingLayout;
  }

  if (!enabled) return;

  settingScreenHideTimer = clearPendingScreenHide(settingScreenHideTimer);

  [screenGroups, screenItems].forEach((el) => {
    if (!el) return;
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    el.style.display = "";
    el.style.opacity = "";
    el.style.transform = "";
    el.style.pointerEvents = "";
    el.classList.remove("hidden");
    el.removeAttribute("aria-hidden");
  });
}

function getPageHistoryState(page) {
  if (page === "setting") {
    return shouldUseSettingSplitLayout("setting")
      ? { page: "setting", screen: "items", group: CURRENT_GROUP || null }
      : { page: "setting", screen: "groups", group: null };
  }
  if (page === "car") return { page: "car", screen: "makers", maker: null };
  if (page === "tools") return { page: "tools" };
  if (page === "logs") return { page: "logs" };
  if (page === "terminal") return { page: "terminal" };
  if (page === "carrot") return { page: "carrot" };
  if (page === "branch") return { page: "branch" };
  return { page: "carrot" };
}

function resolveWebStartPage() {
  const fallback = "carrot";
  let page = fallback;
  try {
    page = typeof window.getWebStartPage === "function" ? window.getWebStartPage() : fallback;
  } catch {
    page = fallback;
  }
  return PAGE_ELEMENTS[page] ? page : fallback;
}

function markWebStartPageBootstrapped() {
  document.documentElement.dataset.carrotBootstrapped = "1";
}

function bootstrapWebStartPage(source = "app") {
  if (window.__CARROT_WEB_HAS_BOOTSTRAPPED_PAGE) {
    markWebStartPageBootstrapped();
    return window.__CARROT_WEB_INITIAL_PAGE || CURRENT_PAGE || "carrot";
  }

  const startPage = resolveWebStartPage();
  const prevPage = CURRENT_PAGE;
  window.__CARROT_WEB_BOOTSTRAPPING = true;
  try {
    showPage(startPage, false);
    history.replaceState(getPageHistoryState(startPage), "");
  } finally {
    window.__CARROT_WEB_BOOTSTRAPPING = false;
  }

  window.__CARROT_WEB_HAS_BOOTSTRAPPED_PAGE = true;
  window.__CARROT_WEB_INITIAL_PAGE = startPage;
  window.__CARROT_WEB_BOOTSTRAP_SOURCE = source;
  markWebStartPageBootstrapped();

  if (startPage === prevPage && PAGE_ELEMENTS[startPage] && !PAGE_ELEMENTS[startPage].hidden) {
    runPageEnter(startPage, prevPage, false);
  }
  return startPage;
}

window.bootstrapWebStartPage = bootstrapWebStartPage;

function runPageEnter(page, prevPage, pushHistory) {
  if (page === "setting") {
    const animateOnEnter = pushHistory || prevPage !== "setting";
    if (!SETTINGS && typeof loadSettings === "function") loadSettings();
    else if (typeof syncSettingViewportLayout === "function" && shouldUseSettingSplitLayout("setting")) {
      syncSettingViewportLayout({
        animateChrome: animateOnEnter,
        animateItems: animateOnEnter,
      }).catch(() => {});
    } else if (pushHistory || !CURRENT_GROUP) {
      if (animateOnEnter) {
        if (typeof getCurrentSettingTab === "function" && getCurrentSettingTab() === "device") {
          if (typeof renderDeviceTab === "function") renderDeviceTab({ animateGroups: true, animateItems: true }).catch(() => {});
        } else if (typeof renderGroups === "function") {
          renderGroups({ animateGroups: true });
        }
      }
      showSettingScreen("groups", false);
    }

    if (typeof loadCurrentCar === "function") loadCurrentCar().catch(() => {});
    return;
  }

  if (page === "carrot" && window.HomeDrive && typeof window.HomeDrive.refresh === "function") {
    window.HomeDrive.refresh();
  }

  if (page === "car") {
    showCarScreen("makers", false);
    if (!CARS && typeof loadCars === "function") loadCars();
    if (typeof loadCurrentCar === "function") loadCurrentCar().catch(() => {});
  }

  if (page === "tools") {
    if (typeof initToolsPage === "function") initToolsPage();
    if (typeof updateQuickLink === "function") updateQuickLink().catch(() => {});
  }

  if (page === "logs" && typeof initLogsPage === "function") {
    initLogsPage();
  }

  if (page === "tuner" && typeof initTunerPage === "function") {
    initTunerPage();
  }

  if (page === "terminal" && typeof initTerminalPage === "function") {
    initTerminalPage();
  }

  if (page === "carrot") {
    if (typeof loadRecordState === "function") loadRecordState().catch(() => {});
  }
}

function commitPageChange(page, prevPage, pushHistory, options = {}) {
  CURRENT_PAGE = page;
  if (!window.__CARROT_WEB_BOOTSTRAPPING && typeof window.recordWebLastPage === "function") window.recordWebLastPage(page);
  syncPageDataset(page);
  syncNavActivePage(page);
  syncSettingSplitLayoutClass(shouldUseSettingSplitLayout(page));

  if (typeof updateAppViewportMetrics === "function") {
    updateAppViewportMetrics();
  }

  if (!options.displaySettled) setDisplayedPage(page);

  window.dispatchEvent(new CustomEvent("carrot:pagechange", { detail: { page, prevPage } }));

  if (page !== "setting" && typeof closeSettingSearchPanel === "function") {
    closeSettingSearchPanel({ clear: false });
  }

  if (prevPage !== "terminal" && page !== "terminal") {
    window.scrollTo(0, 0);
  }

  runPageEnter(page, prevPage, pushHistory);

  if (!options.deferTerminalTeardown && prevPage === "terminal" && page !== "terminal" && typeof teardownTerminalPage === "function") {
    teardownTerminalPage();
  }

  const state = getPageHistoryState(page);
  if (pushHistory) history.pushState(state, "");
  else history.replaceState(state, "");
}

function applySwipeDrag(frame, dx, direction, withResistance = false, options = {}) {
  if (!frame) return;
  const { fromEl, toEl, width } = frame;
  const fade = options.fade !== false;
  const hierarchy = options.hierarchy === true;
  const parallax = Number.isFinite(options.parallax) ? options.parallax : 0.18;
  const dragX = withResistance ? dx * SWIPE_EDGE_RESISTANCE : dx;
  const progress = Math.min(Math.abs(dragX) / width, 1);
  const targetBase = direction === "forward" ? width : -width;

  if (hierarchy && direction === "backward") {
    fromEl.style.transform = `translateX(${dragX}px)`;
    fromEl.style.zIndex = "2";
  } else {
    fromEl.style.transform = `translateX(${dragX}px)`;
    fromEl.style.zIndex = "1";
  }
  fromEl.style.opacity = fade ? `${1 - (progress * 0.14)}` : "1";

  if (toEl) {
    const targetX = hierarchy && direction === "backward"
      ? (-width * parallax) + (dragX * parallax)
      : targetBase + dragX;
    toEl.style.transform = `translateX(${targetX}px)`;
    toEl.style.opacity = fade ? `${0.82 + (progress * 0.18)}` : "1";
    toEl.style.zIndex = hierarchy && direction === "backward" ? "1" : "2";
  }

  if (hierarchy) {
    const foreground = direction === "backward" ? fromEl : toEl;
    const background = direction === "backward" ? toEl : fromEl;
    if (foreground) foreground.style.boxShadow = "-14px 0 30px rgba(0, 0, 0, 0.22)";
    if (background) background.style.boxShadow = "none";
  }

  updateSwipeFrameHeight(frame);
}

function settleSwipe(frame, direction, commit, done, options = {}) {
  if (!frame) {
    done();
    return;
  }
  const { fromEl, toEl, width } = frame;
  const durationMs = Number.isFinite(options.durationMs) ? options.durationMs : SWIPE_SETTLE_MS;
  const easing = options.easing || "cubic-bezier(0.22, 1, 0.36, 1)";
  const fade = options.fade !== false;
  const hierarchy = options.hierarchy === true;
  const parallax = Number.isFinite(options.parallax) ? options.parallax : 0.18;
  const outX = hierarchy && direction === "forward"
    ? -(width * parallax)
    : (direction === "forward" ? -width : width);
  const inX = direction === "forward" ? width : -width;
  const transition = `transform ${durationMs}ms ${easing}, opacity ${durationMs}ms ease`;

  fromEl.style.transition = transition;
  if (toEl) toEl.style.transition = transition;

  void fromEl.offsetWidth;

  // 전환이 '실제로 끝난' 시점에 정리한다. setTimeout(durationMs) 만 쓰면 rAF 로
  // 1프레임 늦게 시작된 transition 보다 먼저 발화해, 마지막에 살짝 튀는 잔상이 생긴다.
  let settleDone = false;
  const finishSettle = () => {
    if (settleDone) return;
    settleDone = true;
    fromEl.removeEventListener("transitionend", onTransitionEnd);
    done();
  };
  const onTransitionEnd = (event) => {
    if (event.target === fromEl && event.propertyName === "transform") finishSettle();
  };
  fromEl.addEventListener("transitionend", onTransitionEnd);

  requestAnimationFrame(() => {
    fromEl.style.transform = commit ? `translateX(${outX}px)` : "translateX(0px)";
    fromEl.style.opacity = fade && commit ? "0" : "1";

    if (toEl) {
      toEl.style.transform = commit ? "translateX(0px)" : `translateX(${inX}px)`;
      toEl.style.opacity = fade ? (commit ? "1" : "0.82") : "1";
    }
  });

  // transitionend 누락(예: 값 변화 없음) 대비 폴백.
  window.setTimeout(finishSettle, durationMs + 80);
}


/* ── showPage / showSettingScreen / showCarScreen ───────── */
function showPage(page, pushHistory = false, transition = null) {
  const prevPage = CURRENT_PAGE;
  if (prevPage === page) {
    if (!window.__CARROT_WEB_BOOTSTRAPPING && typeof window.recordWebLastPage === "function") window.recordWebLastPage(page);
    syncPageDataset(page);
    syncNavActivePage(page);
    syncSettingSplitLayoutClass(shouldUseSettingSplitLayout(page));
    if (typeof updateAppViewportMetrics === "function") {
      updateAppViewportMetrics();
    }
    if (PAGE_ELEMENTS[page]?.hidden || PAGE_ELEMENTS[page]?.style.display === "none") {
      setDisplayedPage(page);
      runPageEnter(page, prevPage, pushHistory);
    }
    return;
  }

  if (page !== "setting" && typeof closeSettingSearchPanel === "function") {
    closeSettingSearchPanel({ clear: false });
  }

  if (pageTransitionTimer) {
    stopPageTransition();
  }

  const shouldTransition = Boolean(transition && prevPage !== page);
  if (shouldTransition) {
    animatePageTransition(prevPage, page, transition, () => {
      if (prevPage === "terminal" && page !== "terminal" && typeof teardownTerminalPage === "function") {
        teardownTerminalPage();
      }
    });
    commitPageChange(page, prevPage, pushHistory, {
      displaySettled: true,
      deferTerminalTeardown: true,
    });
    return;
  }

  commitPageChange(page, prevPage, pushHistory);
}

/* One UI 기본 모션 곡선(공식): 빠르게 가속 후 천천히 감속.
   전체 화면 좌우 슬라이드라 제스처 릴리스(기본 220ms ease-out)보다 살짝
   길게(320ms) 잡아 또렷하고 의도적으로 보이게 한다. */
const SETTING_SCREEN_SLIDE_MS = 320;
const SETTING_SCREEN_SLIDE_EASE = "cubic-bezier(0.22, 0.25, 0, 1)";

/* groups ↔ items 단계 전환을 탭으로 들어갈 때 One UI 식 좌우 슬라이드로 재생.
   방향: 들어갈 때(items) forward, 나올 때(groups) backward. */
function runSettingScreenSlide(showEl, hideEl, direction, token) {
  if (!settingScreenHost || !showEl || !hideEl) return false;

  // 단계 전환에서는 세로 stagger 를 재생하지 않고 좌우 슬라이드만 보인다(들어오는·나가는 화면 모두).
  [showEl, hideEl].forEach((scr) => {
    scr.querySelectorAll(".ui-stagger-item").forEach((el) => el.classList.remove("ui-stagger-item"));
  });

  // 세로 구조: 두 화면은 CSS 로 host 안에서 position:absolute inset:0 로 '항상' 겹쳐 있다.
  // 따라서 전환은 순수 transform 만으로 끝난다 — pin/unpin·minHeight 가 없어, 끝날 때
  // 재배치(흔들)나 재페인트(깜박)가 구조적으로 생기지 않는다.
  [showEl, hideEl].forEach((el) => {
    clearPageTransitionClasses(el);
    el.style.transition = "none";
    el.style.transform = "";
    el.style.display = "";
    el.classList.remove("hidden");
  });

  settingScreenHost.classList.add("setting-screen-transitioning");
  document.getElementById("pageSetting")?.classList.add("setting-screen-transitioning");

  const enterFrom = direction === "forward" ? "100%" : "-100%";
  const exitTo = direction === "forward" ? "-100%" : "100%";
  showEl.style.zIndex = "2";
  showEl.style.willChange = "transform";
  showEl.style.transform = `translateX(${enterFrom})`;
  hideEl.style.zIndex = "1";
  hideEl.style.willChange = "transform";
  hideEl.style.transform = "translateX(0)";

  void showEl.offsetWidth;

  let settled = false;
  const finalize = () => {
    if (settled) return;
    settled = true;
    showEl.removeEventListener("transitionend", onTransitionEnd);
    if (token !== settingScreenTransitionToken) return;
    // transition 을 끈 채로 정리한다 — 그래야 transform 을 지울 때 .screen 의 .16s 전환이
    // 재가동돼 끝에 살짝 움직이거나 깜박이지 않는다. 정리를 즉시 커밋한 뒤 CSS 전환을 복구한다.
    showEl.style.transition = "none";
    hideEl.style.transition = "none";
    hideEl.style.display = "none";
    hideEl.classList.add("hidden");
    showEl.style.transform = "";
    hideEl.style.transform = "";
    showEl.style.zIndex = "";
    hideEl.style.zIndex = "";
    void showEl.offsetWidth;
    showEl.style.transition = "";
    hideEl.style.transition = "";
    showEl.style.willChange = "";
    hideEl.style.willChange = "";
    settingScreenHost.classList.remove("setting-screen-transitioning");
    document.getElementById("pageSetting")?.classList.remove("setting-screen-transitioning");
  };
  const onTransitionEnd = (event) => {
    if (event.target === showEl && event.propertyName === "transform") finalize();
  };
  showEl.addEventListener("transitionend", onTransitionEnd);

  requestAnimationFrame(() => {
    if (token !== settingScreenTransitionToken) { finalize(); return; }
    const slide = `transform ${SETTING_SCREEN_SLIDE_MS}ms ${SETTING_SCREEN_SLIDE_EASE}`;
    showEl.style.transition = slide;
    hideEl.style.transition = slide;
    showEl.style.transform = "translateX(0)";
    hideEl.style.transform = `translateX(${exitTo})`;
  });

  window.setTimeout(finalize, SETTING_SCREEN_SLIDE_MS + 80);
  return true;
}

function showSettingScreen(which, pushHistory = false) {
  const isGroups = (which === "groups");
  const showEl = isGroups ? screenGroups : screenItems;
  const hideEl = isGroups ? screenItems : screenGroups;
  const isProfileItems = !isGroups && typeof isSettingProfileGroup === "function" && isSettingProfileGroup(CURRENT_GROUP);
  const currentGroupLabel = (!isGroups && CURRENT_GROUP && typeof getSettingGroupLabel === "function")
    ? getSettingGroupLabel(CURRENT_GROUP)
    : (CURRENT_GROUP || "");
  const splitLandscape = (CURRENT_PAGE === "setting" && typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode());
  const transitionToken = ++settingScreenTransitionToken;

  settingScreenHideTimer = clearPendingScreenHide(settingScreenHideTimer);
  if (settingScreenHost?.classList.contains("setting-screen-transitioning") && !settingGroupTransitionLock) {
    [screenGroups, screenItems].forEach((el) => {
      clearPageTransitionClasses(el);
      resetPageRuntimeStyles(el);
    });
    settingScreenHost.classList.remove("setting-screen-transitioning");
    document.getElementById("pageSetting")?.classList.remove("setting-screen-transitioning");
    settingScreenHost.style.minHeight = "";
  }
  syncSettingSplitLayoutClass(splitLandscape);
  document.getElementById("pageSetting")?.classList.toggle("setting-profile-active", isProfileItems);

  if (splitLandscape) {
    settingTitle.textContent = UI_STRINGS[LANG].setting || "Setting";
    if (pushHistory) {
      history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
    }
    if (settingScreenHost) settingScreenHost.style.minHeight = "";
    return;
  }

  if (btnBackGroups) btnBackGroups.style.display = "none";
  settingTitle.textContent = isGroups ? (UI_STRINGS[LANG].setting || "Setting") : ((UI_STRINGS[LANG].setting || "Setting") + " - " + currentGroupLabel);

  const reduceMotion = Boolean(window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches);
  const canSlide =
    !window.__CARROT_WEB_BOOTSTRAPPING &&
    !reduceMotion &&
    settingScreenHost &&
    showEl !== hideEl &&
    hideEl.style.display !== "none" &&
    !hideEl.classList.contains("hidden");
  const didSlide = canSlide && runSettingScreenSlide(showEl, hideEl, isGroups ? "backward" : "forward", transitionToken);

  if (!didSlide) {
    showEl.style.display = "";
    requestAnimationFrame(() => {
      if (transitionToken !== settingScreenTransitionToken) return;
      showEl.classList.remove("hidden");
    });

    hideEl.classList.add("hidden");
    settingScreenHideTimer = window.setTimeout(() => {
      if (transitionToken !== settingScreenTransitionToken) return;
      hideEl.style.display = "none";
      settingScreenHideTimer = null;
    }, 170);
  }

  if (pushHistory) {
    history.pushState({ page: "setting", screen: which, group: CURRENT_GROUP || null }, "");
  }

  if (settingScreenHost && !didSlide) settingScreenHost.style.minHeight = "";
  if (isGroups && typeof setSettingItemsScrollTop === "function") {
    requestAnimationFrame(() => setSettingItemsScrollTop(0));
  }
}

function resetSettingPageToRoot() {
  if (typeof closeSettingSearchPanel === "function") {
    closeSettingSearchPanel({ clear: false, syncHistory: false });
  }

  if (shouldUseSettingSplitLayout("setting")) {
    if (typeof syncSettingViewportLayout === "function") {
      syncSettingViewportLayout({ animateChrome: true, animateItems: true }).catch(() => {});
    }
    history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
    return;
  }

  showSettingScreen("groups", false);
  if (typeof setSettingItemsScrollTop === "function") {
    setSettingItemsScrollTop(0);
  }
  window.scrollTo(0, 0);
  history.replaceState({ page: "setting", screen: "groups", group: null }, "");
}

function showCarScreen(which, pushHistory = false) {
  const isMakers = (which === "makers");
  const showEl = isMakers ? carScreenMakers : carScreenModels;
  const hideEl = isMakers ? carScreenModels : carScreenMakers;
  const transitionToken = ++carScreenTransitionToken;

  carScreenHideTimer = clearPendingScreenHide(carScreenHideTimer);

  showEl.style.display = "";
  requestAnimationFrame(() => {
    if (transitionToken !== carScreenTransitionToken) return;
    showEl.classList.remove("hidden");
  });

  hideEl.classList.add("hidden");
  carScreenHideTimer = window.setTimeout(() => {
    if (transitionToken !== carScreenTransitionToken) return;
    hideEl.style.display = "none";
    carScreenHideTimer = null;
  }, 170);

  if (pushHistory) {
    history.pushState({ page: "car", screen: which, maker: CURRENT_MAKER || null }, "");
  }
}


/* ── Setting back-swipe support helpers ─────────────────── */
function isLandscapeRailMode() {
  return window.matchMedia("(orientation: landscape)").matches;
}

function isSettingItemsScreenActive() {
  return Boolean(
    CURRENT_PAGE === "setting" &&
    screenItems &&
    screenItems.style.display !== "none" &&
    !screenItems.classList.contains("hidden")
  );
}

function prepareSettingBackFrame() {
  if (!settingScreenHost || !screenItems || !screenGroups) return null;

  [screenItems, screenGroups].forEach((el) => {
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    el.classList.remove("hidden");
  });

  const frame = prepareSwipeFrame(settingScreenHost, screenItems, screenGroups);
  if (!frame) return null;
  settingScreenHost.classList.add("setting-back-swiping");
  screenItems.style.zIndex = "2";
  screenGroups.style.zIndex = "1";
  return frame;
}

function cleanupSettingBackFrame() {
  if (!settingScreenHost || !screenItems || !screenGroups) return;
  settingScreenHost.style.minHeight = "";
  settingScreenHost.classList.remove("setting-back-swiping");
  [screenItems, screenGroups].forEach((el) => {
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
  });
}


/* ── Page navigation bindings ───────────────────────────── */
btnTools.onclick = () => showPage("tools", true, getSwipeTransition(CURRENT_PAGE, "tools"));
btnHome.onclick = () => showPage("carrot", true, getSwipeTransition(CURRENT_PAGE, "carrot"));
btnRecordToggle.onclick = () => { if (typeof toggleRecord === "function") toggleRecord(); };
btnSetting.onclick = () => {
  if (CURRENT_PAGE === "setting") {
    resetSettingPageToRoot();
    return;
  }
  showPage("setting", true, getSwipeTransition(CURRENT_PAGE, "setting"));
};
if (btnLogs) btnLogs.onclick = () => showPage("logs", true, getSwipeTransition(CURRENT_PAGE, "logs"));
if (btnTuner) btnTuner.onclick = () => showPage("tuner", true, getSwipeTransition(CURRENT_PAGE, "tuner"));
btnTerminal.onclick = () => showPage("terminal", true, getSwipeTransition(CURRENT_PAGE, "terminal"));

if (settingCarRow) {
  settingCarRow.onclick = () => {
    if (typeof window.openCarPickerFlow === "function") window.openCarPickerFlow();
    else showPage("car", true);
  };
  settingCarRow.onkeydown = (e) => {
    if (e.key === "Enter" || e.key === " ") {
      e.preventDefault();
      if (typeof window.openCarPickerFlow === "function") window.openCarPickerFlow();
      else showPage("car", true);
    }
  };
}
btnBackCar.onclick = () => history.back();
carTitle.onclick = () => history.back();
modelTitle.onclick = () => showCarScreen("makers");

function goBackUnlessSettingSplit() {
  if (
    CURRENT_PAGE === "setting" &&
    typeof isCompactLandscapeMode === "function" &&
    isCompactLandscapeMode()
  ) {
    return;
  }
  history.back();
}

if (btnBackGroups) btnBackGroups.onclick = goBackUnlessSettingSplit;
settingTitle.onclick = goBackUnlessSettingSplit;
if (itemsTitle) itemsTitle.onclick = goBackUnlessSettingSplit;

btnBackBranch.onclick = () => history.back();
branchTitle.onclick = () => history.back();

if (btnQuickLinkWeb) {
  btnQuickLinkWeb.onclick = (e) => {
    e.preventDefault();
    openQuickLink().catch(() => {});
  };
}


/* Touch-swipe navigation is intentionally removed; menu buttons keep lightweight page transitions. */


// Final initialization (originally at end of app_core.js)
syncHomeUtilityButtons();
renderQuickLinkUI();
