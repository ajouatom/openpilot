"use strict";
(function () {
  var menuWraps = Array.prototype.slice.call(document.querySelectorAll(".rc-menu-wrap"));
  function closeMenus() {
    menuWraps.forEach(function (wrap) {
      var m = wrap.querySelector(".rc-menu");
      var b = wrap.querySelector("button[aria-haspopup]");
      if (m) m.hidden = true;
      if (b) b.setAttribute("aria-expanded", "false");
    });
  }
  menuWraps.forEach(function (wrap) {
    var b = wrap.querySelector("button[aria-haspopup]");
    var m = wrap.querySelector(".rc-menu");
    if (!b || !m) return;
    b.addEventListener("click", function (e) {
      e.stopPropagation();
      var willOpen = m.hidden;
      closeMenus();
      m.hidden = !willOpen;
      b.setAttribute("aria-expanded", String(willOpen));
    });
  });
  document.addEventListener("click", function (e) {
    if (!e.target.closest(".rc-menu-wrap")) closeMenus();
  });

  function toast(msg, tone) {
    if (typeof showAppToast === "function") showAppToast(msg, tone ? { tone: tone } : {});
  }

  async function callAction(action, payload) {
    var body = Object.assign({ action: action }, payload || {});
    var r = await fetch("/api/action", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body),
    });
    return r.json();
  }
  var cwpPush = document.getElementById("rcCwpPush");
  var cwpState = document.getElementById("rcCwpState");
  function showCwp(data) {
    if (!cwpPush || !cwpState) return;
    cwpPush.checked = !!(data && data.enabled);
    cwpPush.disabled = !(data && data.ok && data.registered);
    cwpState.hidden = !!(data && data.registered);
    cwpState.textContent = data && data.state === "unregistered" ? "Not registered" : "Unavailable";
  }
  async function refreshCwp() {
    try {
      showCwp(await CarrotRecoveryApi.getJson("/api/recovery/cwp"));
    } catch (err) {
      showCwp((err && err.payload) || {});
    }
  }
  if (cwpPush) cwpPush.addEventListener("change", async function (event) {
    event.stopPropagation();
    var enabled = cwpPush.checked;
    cwpPush.disabled = true;
    try {
      showCwp(await CarrotRecoveryApi.postJson("/api/recovery/cwp", { enabled: enabled }));
      toast("CWP Push " + (enabled ? "on" : "off"), "success");
    } catch (err) {
      toast((err && err.message) || "CWP failed", "error");
      await refreshCwp();
    }
  });
  async function runInTerminal(command) {
    await fetch("/api/terminal/input", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ data: command }),
    });
  }
  async function dispatchGit(action, payload) {
    var data = await callAction(action, payload);
    if (data && data.rebooting) return data;
    if (data && data.ok === false) { toast(data.error || "failed", "error"); return data; }
    if (data && data.command) await runInTerminal(data.command);
    return data;
  }

  var CONFIRMS = {
    git_pull:    { title: "git pull",  message: "Pull latest commits from remote.\nLocal changes will be reset first.", confirmLabel: "Pull" },
    git_sync:    { title: "git sync",  message: "Delete all local branches except the current one, then fetch from remote.", confirmLabel: "Sync" },
    git_reset:   { title: "git reset", message: "Reset the current branch to the last commit.\nAll local changes will be lost.", confirmLabel: "Reset" },
    git_rebuild: { title: "rebuild",   message: "Clean all build cache. Will rebuild on next boot.", confirmLabel: "Clean" },
  };

  async function pickBranch(title, message, selected, confirmLabel) {
    var data = await callAction("git_branches");
    if (!data || !data.ok) { toast((data && data.error) || "failed to list branches", "error"); return null; }
    var want = selected || data.current;
    var choices = data.branches.map(function (b) {
      return { label: (b.kind === "remote" ? "\u2197 " : "   ") + b.name, value: b.name, current: b.name === want };
    });
    return openAppDialog({ mode: "choice", title: title, message: message, choices: choices, confirmLabel: confirmLabel });
  }

  async function onGit(action) {
    closeMenus();
    if (CONFIRMS[action]) {
      var c = CONFIRMS[action];
      if (await appConfirm(c.message, { title: c.title, confirmLabel: c.confirmLabel })) await dispatchGit(action);
      return;
    }
    if (action === "git_log") {
      var log = await callAction("git_log");
      if (!log || !log.ok) { toast((log && log.error) || "failed", "error"); return; }
      var commits = log.commits || [];
      if (!commits.length) { appAlert("(no commits)", { title: "git log" }); return; }
      var logChoices = commits.map(function (c) {
        return { label: c.hash + "  " + (c.message || ""), value: c.hash, current: !!(log.current && c.hash.indexOf(log.current) === 0) };
      });
      var sel = await openAppDialog({ mode: "choice", title: "git log", message: "Select a commit to checkout.", choices: logChoices, confirmLabel: "Checkout" });
      if (!sel) return;
      if (log.current && sel.indexOf(log.current) === 0) return;
      if (await appConfirm("Move to the selected commit.\n\n" + sel, { title: "Checkout commit", confirmLabel: "Checkout" })) {
        await dispatchGit("git_checkout_commit", { commit: sel });
      }
      return;
    }
    if (action === "git_branches") {
      var branch = await pickBranch("Change branch", "Select a branch to switch to.", null, "Switch");
      if (branch) await dispatchGit("git_checkout", { branch: branch });
      return;
    }
    if (action === "git_reset_repo") {
      var target = await pickBranch("Reset repo", "Fetch the selected branch fresh.\nAll local changes and untracked files will be lost.", "c3-wip", "Next");
      if (!target) return;
      if (await appConfirm("Branch: " + target + "\nRemote: origin\n\nThis cannot be undone.", { title: "Confirm reset", confirmLabel: "Reset" })) {
        await dispatchGit("git_reset_repo", { branch: target });
      }
      return;
    }
    if (action === "git_reboot") {
      if (!(await appConfirm("The device will restart immediately.", { title: "Reboot", confirmLabel: "Next" }))) return;
      if (!(await appConfirm("Really reboot the device?", { title: "Confirm reboot", confirmLabel: "Reboot" }))) return;
      await dispatchGit("git_reboot");
      return;
    }
  }

  async function onTool(action) {
    closeMenus();
    if (action === "rebuild_all") {
      if (await appConfirm("Clean the build cache and reboot.\n\n\u2022 scons -c\n\u2022 rm -rf prebuilt\n\u2022 sudo reboot", { title: "rebuild", confirmLabel: "Rebuild" })) {
        await dispatchGit("rebuild_all");
      }
      return;
    }
    if (action === "send_tmux_log") {
      var r = await callAction("send_tmux_log");
      if (r && r.ok) { toast("tmux log captured", "success"); window.open(r.file || "/download/tmux.log", "_blank"); }
      else toast((r && r.error) || "capture failed", "error");
      return;
    }
    if (action === "server_tmux_log") {
      var r2 = await callAction("server_tmux_log");
      if (r2 && r2.ok) toast(r2.partial ? "tmux log sent to server (partial)" : "tmux log sent to server", "success");
      else toast((r2 && r2.error) || "failed", "error");
      return;
    }
  }

  document.querySelectorAll("[data-act]").forEach(function (btn) {
    btn.addEventListener("click", function (e) { e.stopPropagation(); onGit(btn.dataset.act); });
  });
  document.querySelectorAll("[data-tool]").forEach(function (btn) {
    btn.addEventListener("click", function (e) { e.stopPropagation(); onTool(btn.dataset.tool); });
  });
  refreshCwp();

  // Boot the reused generated terminal runtime.
  function boot() {
    if (typeof initTerminalPage === "function") initTerminalPage();
    else setTimeout(boot, 30);
  }
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", boot);
  else boot();
})();
