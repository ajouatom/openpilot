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
  el.style.width = "";
}

function setDisplayedPage(page) {
  Object.entries(PAGE_ELEMENTS).forEach(([name, el]) => {
    if (!el) return;
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    el.style.display = (name === page) ? "" : "none";
  });
  if (swipeContainer) swipeContainer.style.minHeight = "";
  if (settingScreenHost) settingScreenHost.style.minHeight = "";
}

function clearPendingScreenHide(timerRef) {
  if (timerRef) {
    window.clearTimeout(timerRef);
  }
  return null;
}

function getSwipeTransition(fromPage, toPage) {
  if (fromPage === "terminal" || toPage === "terminal") return null;
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
  el.style.top = `${metrics.top}px`;
  el.style.left = `${metrics.left}px`;
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

function animatePageTransition(fromPage, toPage, transition) {
  const fromEl = PAGE_ELEMENTS[fromPage];
  const toEl = PAGE_ELEMENTS[toPage];
  if (!swipeContainer || !fromEl || !toEl || fromEl === toEl || !transition) {
    setDisplayedPage(toPage);
    return;
  }

  pageTransitionToken += 1;
  const token = pageTransitionToken;

  if (pageTransitionTimer) {
    clearTimeout(pageTransitionTimer);
    pageTransitionTimer = null;
  }

  Object.values(PAGE_ELEMENTS).forEach((el) => {
    if (!el) return;
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    if (el !== fromEl && el !== toEl) el.style.display = "none";
  });

  const frame = prepareSwipeFrame(swipeContainer, fromEl, toEl);
  if (!frame) {
    setDisplayedPage(toPage);
    return;
  }

  toEl.classList.add(transition === "forward" ? "page-enter-from-right" : "page-enter-from-left");

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
  }, PAGE_TRANSITION_MS);
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
  setElementClass(btnTerminal, "active", page === "terminal");
}

function prepareSwipePages(fromPage, toPage) {
  const fromEl = PAGE_ELEMENTS[fromPage];
  const toEl = PAGE_ELEMENTS[toPage];
  if (!swipeContainer || !fromEl || !toEl) return null;

  stopPageTransition();

  Object.values(PAGE_ELEMENTS).forEach((el) => {
    if (!el) return;
    clearPageTransitionClasses(el);
    resetPageRuntimeStyles(el);
    if (el !== fromEl && el !== toEl) el.style.display = "none";
  });

  return prepareSwipeFrame(swipeContainer, fromEl, toEl);
}

function applySwipeDrag(frame, dx, direction, withResistance = false) {
  if (!frame) return;
  const { fromEl, toEl, width } = frame;
  const dragX = withResistance ? dx * SWIPE_EDGE_RESISTANCE : dx;
  const progress = Math.min(Math.abs(dragX) / width, 1);
  const targetBase = direction === "forward" ? width : -width;

  fromEl.style.transform = `translateX(${dragX}px)`;
  fromEl.style.opacity = `${1 - (progress * 0.14)}`;

  if (toEl) {
    toEl.style.transform = `translateX(${targetBase + dragX}px)`;
    toEl.style.opacity = `${0.82 + (progress * 0.18)}`;
    toEl.style.zIndex = "2";
  }

  updateSwipeFrameHeight(frame);
}

function settleSwipe(frame, direction, commit, done) {
  if (!frame) {
    done();
    return;
  }
  const { fromEl, toEl, width } = frame;
  const outX = direction === "forward" ? -width : width;
  const inX = direction === "forward" ? width : -width;
  const transition = `transform ${SWIPE_SETTLE_MS}ms cubic-bezier(0.22, 1, 0.36, 1), opacity ${SWIPE_SETTLE_MS}ms ease`;

  fromEl.style.transition = transition;
  if (toEl) toEl.style.transition = transition;

  requestAnimationFrame(() => {
    fromEl.style.transform = commit ? `translateX(${outX}px)` : "translateX(0px)";
    fromEl.style.opacity = commit ? "0" : "1";

    if (toEl) {
      toEl.style.transform = commit ? "translateX(0px)" : `translateX(${inX}px)`;
      toEl.style.opacity = commit ? "1" : "0.82";
    }
  });

  window.setTimeout(done, SWIPE_SETTLE_MS);
}


/* ── showPage / showSettingScreen / showCarScreen ───────── */
function showPage(page, pushHistory = false, transition = null) {
  const prevPage = CURRENT_PAGE;
  if (prevPage === page) {
    syncPageDataset(page);
    syncNavActivePage(page);
    return;
  }

  if (prevPage === "terminal" && page !== "terminal" && typeof teardownTerminalPage === "function") {
    teardownTerminalPage();
  }
  CURRENT_PAGE = page;
  syncPageDataset(page);
  syncNavActivePage(page);

  if (typeof updateAppViewportMetrics === "function") {
    updateAppViewportMetrics();
  }

  if (page === "setting" && SETTINGS) {
    if (typeof syncSettingViewportLayout === "function") {
      syncSettingViewportLayout().catch(() => {});
    } else if (pushHistory || !CURRENT_GROUP) {
      showSettingScreen("groups", false);
    }
  }

  if (page === "carrot" && window.HomeDrive && typeof window.HomeDrive.refresh === "function") {
    window.HomeDrive.refresh();
  }

  if (transition && prevPage !== page) animatePageTransition(prevPage, page, transition);
  else setDisplayedPage(page);

  window.dispatchEvent(new CustomEvent("carrot:pagechange", { detail: { page, prevPage } }));

  if (page !== "setting" && typeof closeSettingSearchPanel === "function") {
    closeSettingSearchPanel({ clear: false });
  }

  // Terminal uses its own fixed viewport layout. Resetting the window scroll
  // while entering/leaving it causes visible jumps on mobile.
  if (prevPage !== "terminal" && page !== "terminal") {
    window.scrollTo(0, 0);
  }

  if (page === "setting") {
    if (!SETTINGS && typeof loadSettings === "function") loadSettings();
    else if (typeof syncSettingViewportLayout === "function" && typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode()) {
      syncSettingViewportLayout().catch(() => {});
    } else if (pushHistory || !CURRENT_GROUP) showSettingScreen("groups", false);
    if (typeof loadCurrentCar === "function") loadCurrentCar().catch(() => {});
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
  if (page === "terminal" && typeof initTerminalPage === "function") {
    initTerminalPage();
  }
  if (page === "carrot") {
    if (typeof loadRecordState === "function") loadRecordState().catch(() => {});
  }

  const state =
    (page === "setting") ? { page: "setting", screen: "groups", group: null } :
    (page === "car") ? { page: "car", screen: "makers", maker: null } :
    (page === "tools") ? { page: "tools" } :
    (page === "logs") ? { page: "logs" } :
    (page === "terminal") ? { page: "terminal" } :
    (page === "carrot") ? { page: "carrot" } :
    (page === "branch") ? { page: "branch" } :
    { page: "carrot" };

  if (pushHistory) history.pushState(state, "");
  else history.replaceState(state, "");
}

function showSettingScreen(which, pushHistory = false) {
  const isGroups = (which === "groups");
  const showEl = isGroups ? screenGroups : screenItems;
  const hideEl = isGroups ? screenItems : screenGroups;
  const currentGroupLabel = (!isGroups && CURRENT_GROUP && typeof getSettingGroupLabel === "function")
    ? getSettingGroupLabel(CURRENT_GROUP)
    : (CURRENT_GROUP || "");
  const splitLandscape = (CURRENT_PAGE === "setting" && typeof isCompactLandscapeMode === "function" && isCompactLandscapeMode());
  const transitionToken = ++settingScreenTransitionToken;

  settingScreenHideTimer = clearPendingScreenHide(settingScreenHideTimer);

  if (splitLandscape) {
    settingTitle.textContent = UI_STRINGS[LANG].setting || "Setting";
    if (settingSubnavWrap) settingSubnavWrap.style.display = "none";
    if (showEl) {
      showEl.style.display = "";
      showEl.classList.remove("hidden");
    }
    if (hideEl) {
      hideEl.style.display = "";
      hideEl.classList.remove("hidden");
    }
    if (pushHistory) {
      history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
    }
    if (settingScreenHost) settingScreenHost.style.minHeight = "";
    if (typeof syncSettingSubnavFixedOffset === "function") {
      requestAnimationFrame(syncSettingSubnavFixedOffset);
    }
    return;
  }

  if (btnBackGroups) btnBackGroups.style.display = "none";
  settingTitle.textContent = isGroups ? (UI_STRINGS[LANG].setting || "Setting") : ((UI_STRINGS[LANG].setting || "Setting") + " - " + currentGroupLabel);
  if (settingSubnavWrap) settingSubnavWrap.style.display = isGroups ? "none" : "";

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

  if (pushHistory) {
    history.pushState({ page: "setting", screen: which, group: CURRENT_GROUP || null }, "");
  }

  if (settingScreenHost) settingScreenHost.style.minHeight = "";
  if (typeof syncSettingSubnavFixedOffset === "function") {
    requestAnimationFrame(syncSettingSubnavFixedOffset);
  }
  if (isGroups && typeof setSettingItemsScrollTop === "function") {
    requestAnimationFrame(() => setSettingItemsScrollTop(0));
  }
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
  if (typeof stopSettingSubnavMotion === "function") stopSettingSubnavMotion();

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
btnSetting.onclick = () => showPage("setting", true, getSwipeTransition(CURRENT_PAGE, "setting"));
if (btnLogs) btnLogs.onclick = () => showPage("logs", true, getSwipeTransition(CURRENT_PAGE, "logs"));
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

if (btnBackGroups) btnBackGroups.onclick = () => history.back();
settingTitle.onclick = () => history.back();
if (itemsTitle) itemsTitle.onclick = () => history.back();

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
