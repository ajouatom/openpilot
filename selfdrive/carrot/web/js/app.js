"use strict";

// App bootstrap — runs LAST, after all shared/* and pages/* modules are loaded.
// Owns:
//   1. browser back/forward (popstate) → page restoration
//   2. initial page show on first load

window.addEventListener("popstate", async (ev) => {
  const st = ev.state || { page: "carrot" };

  if (settingSearchPanel && !settingSearchPanel.hidden && !st.search) {
    closeSettingSearchPanel({ clear: false, fromHistory: true });
  }

  if (st.page === "home" || st.page === "carrot") {
    CURRENT_GROUP = null;
    CURRENT_MAKER = null;
    showPage("carrot", false);
    return;
  }

  if (st.page === "setting") {
    const screen = st.screen || "groups";
    const previousDetail = CURRENT_SETTING_DETAIL;
    const itemsWereActive = typeof isSettingItemsScreenActive === "function" && isSettingItemsScreenActive();
    CURRENT_GROUP = st.group || null;
    showPage("setting", false);

    if (isCompactLandscapeMode()) {
      const targetGroup = CURRENT_GROUP || getLandscapeDefaultSettingGroup();
      if (targetGroup) {
        CURRENT_GROUP = targetGroup;
        if (screen === "detail" && st.settingName) {
          showSettingScreen("items", false);
          await renderItems(targetGroup, {
            detailName: st.settingName,
            scrollMode: "restore",
            animateItems: false,
          });
        } else {
          await activateSettingGroup(targetGroup, false, { scrollMode: "restore", animateGroups: false, animateItems: false });
        }
      } else {
        showSettingScreen("groups", false);
      }
      if (st.search) {
        openSettingSearchPanel({ pushHistory: false }).catch(() => {});
      }
      return;
    }

    if (screen === "detail" && CURRENT_GROUP && st.settingName) {
      await transitionSettingItemsContent(() => renderItems(CURRENT_GROUP, {
        detailName: st.settingName,
        scrollMode: "restore",
        animateItems: false,
      }), previousDetail ? "backward" : "forward");
    } else if (screen === "items" && CURRENT_GROUP) {
      if (itemsWereActive && previousDetail) {
        await transitionSettingItemsContent(
          () => renderItems(CURRENT_GROUP, { scrollMode: "restore", animateItems: false }),
          "backward",
        );
      } else {
        await activateSettingGroup(CURRENT_GROUP, false, {
          scrollMode: "restore",
          animateGroups: false,
          animateItems: false,
        });
      }
    } else {
      CURRENT_SETTING_DETAIL = null;
      showSettingScreen("groups", false);
    }
    if (st.search) {
      openSettingSearchPanel({ pushHistory: false }).catch(() => {});
    }
    return;
  }

  if (st.page === "car") {
    showPage("car", false);
    if (!CARS) await loadCars();

    const screen = st.screen || "makers";
    CURRENT_MAKER = st.maker || null;

    if (screen === "models" && CURRENT_MAKER) {
      renderModels(CURRENT_MAKER);
      showCarScreen("models", false);
    } else {
      showCarScreen("makers", false);
    }
    return;
  }

  if (st.page == "tools") {
    showPage("tools", false);
    return;
  }

  if (st.page === "logs") {
    showPage("logs", false);
    return;
  }

  if (st.page === "terminal") {
    showPage("terminal", false);
    return;
  }

  if (st.page === "carrot") {
    showPage("carrot", false);
    return;
  }

  if (st.page === "branch") {
    showPage("branch", false);
    if (!BRANCHES || !BRANCHES.length) {
      loadBranchesAndShow().catch(() => {});
    }
    return;
  }
});

// Initial page render after all scripts are loaded.
if (typeof window.bootstrapWebStartPage === "function") {
  window.bootstrapWebStartPage("app");
} else {
  showPage("carrot", false);
}
