// Model Selector — carrot web page.
//
// Loaded on-demand when the user navigates to #pageModels.  Keeps its own
// polling loop for the currently active install job.

(function () {
  "use strict";

  const POLL_INTERVAL_MS = 320;

  let currentJobId = null;
  let pollTimer = null;

  const $ = (id) => document.getElementById(id);

  function fmtMB(bytes) {
    return (bytes / (1024 * 1024)).toFixed(1) + " MB";
  }

  function showError(msg) {
    const el = $("msError");
    if (!el) return;
    el.textContent = msg || "";
    el.style.display = msg ? "block" : "none";
  }

  function showProgress(show, percent, message) {
    const wrap = $("msProgressWrap");
    if (!wrap) return;
    wrap.style.display = show ? "block" : "none";
    if (show) {
      $("msProgressFill").style.width = Math.max(0, Math.min(100, percent || 0)) + "%";
      $("msProgressMsg").textContent = message || "";
    }
  }

  async function fetchJSON(url, opts) {
    const resp = await fetch(url, opts || {});
    const data = await resp.json().catch(() => ({ ok: false, error: "bad json" }));
    if (!resp.ok || data.ok === false) {
      const err = data.error || ("HTTP " + resp.status);
      throw new Error(err);
    }
    return data;
  }

  async function refreshStatus() {
    try {
      const s = await fetchJSON("/api/models/status");
      $("msCurrent").textContent = s.current_model || "Default Model";
      $("msPending").textContent = s.pending_model || "—";
      $("msDisk").textContent = s.disk_free_mb >= 0 ? s.disk_free_mb : "?";
      $("msDescription").textContent = s.description || "";
      $("msApplyBtn").disabled = !s.pending_model;
    } catch (e) {
      showError("status: " + e.message);
    }
  }

  function rowHtml(m) {
    const arch = m.has_off_policy
      ? (m.has_on_policy ? "on+off" : "policy+off")
      : (m.has_on_policy ? "on_policy" : "policy");
    return `
      <tr>
        <td>${escapeHtml(m.name)}</td>
        <td>${arch}</td>
        <td>${fmtMB(m.total_size)}</td>
        <td>${escapeHtml(m.added_at || "")}</td>
        <td><button class="btn ms-install-btn" data-id="${escapeHtml(m.id)}">Install</button></td>
      </tr>`;
  }

  function escapeHtml(s) {
    return String(s == null ? "" : s)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  async function refreshList() {
    const body = $("msModelsBody");
    body.innerHTML = "<tr><td colspan='5'>Loading…</td></tr>";
    try {
      const d = await fetchJSON("/api/models/list");
      if (!d.models || !d.models.length) {
        body.innerHTML = "<tr><td colspan='5'>No models available.</td></tr>";
        return;
      }
      body.innerHTML = d.models.map(rowHtml).join("");
      Array.from(body.querySelectorAll(".ms-install-btn")).forEach((btn) => {
        btn.addEventListener("click", () => onInstall(btn.dataset.id));
      });
    } catch (e) {
      body.innerHTML = "<tr><td colspan='5'>Failed to load.</td></tr>";
      showError("list: " + e.message);
    }
  }

  async function onInstall(modelId) {
    if (currentJobId) {
      showError("another install is already running");
      return;
    }
    showError("");
    try {
      const d = await fetchJSON("/api/models/install", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ id: modelId }),
      });
      currentJobId = d.id;
      showProgress(true, 0, "starting…");
      startPoll();
    } catch (e) {
      showError("install: " + e.message);
    }
  }

  function startPoll() {
    stopPoll();
    pollTimer = setInterval(pollOnce, POLL_INTERVAL_MS);
  }

  function stopPoll() {
    if (pollTimer) {
      clearInterval(pollTimer);
      pollTimer = null;
    }
  }

  async function pollOnce() {
    if (!currentJobId) return;
    try {
      const snap = await fetchJSON("/api/models/job?id=" + encodeURIComponent(currentJobId));
      showProgress(true, snap.progress || 0, snap.message || "");
      if (snap.done) {
        stopPoll();
        currentJobId = null;
        if (snap.status === "done") {
          showProgress(true, 100, "downloaded — ready to apply");
          refreshStatus();
        } else {
          showProgress(false, 0, "");
          showError(snap.error || "install failed");
        }
      }
    } catch (e) {
      stopPoll();
      currentJobId = null;
      showProgress(false, 0, "");
      showError("poll: " + e.message);
    }
  }

  async function onApply() {
    showError("");
    try {
      const d = await fetchJSON("/api/models/apply", { method: "POST" });
      showProgress(true, 100, "rebooting in " + (d.reboot_in || 3) + "s…");
    } catch (e) {
      showError("apply: " + e.message);
    }
  }

  async function onReset() {
    if (!confirm("Remove custom model and revert to built-in?")) return;
    showError("");
    try {
      await fetchJSON("/api/models/reset", { method: "POST" });
      refreshStatus();
    } catch (e) {
      showError("reset: " + e.message);
    }
  }

  function bind() {
    const refreshBtn = $("msRefreshBtn");
    if (refreshBtn && !refreshBtn.dataset.bound) {
      refreshBtn.dataset.bound = "1";
      refreshBtn.addEventListener("click", () => {
        refreshStatus();
        refreshList();
      });
    }
    const applyBtn = $("msApplyBtn");
    if (applyBtn && !applyBtn.dataset.bound) {
      applyBtn.dataset.bound = "1";
      applyBtn.addEventListener("click", onApply);
    }
    const resetBtn = $("msResetBtn");
    if (resetBtn && !resetBtn.dataset.bound) {
      resetBtn.dataset.bound = "1";
      resetBtn.addEventListener("click", onReset);
    }
  }

  // Entry point — call from the page router when #pageModels becomes visible.
  window.ModelSelector = {
    onShow: function () {
      bind();
      refreshStatus();
      refreshList();
    },
    onHide: function () {
      stopPoll();
    },
  };

  // Self-bootstrap: pull the page fragment into the DOM and wire up the nav
  // button that index.html provides.  Idempotent.
  async function bootstrap() {
    if (!document.getElementById("pageModels")) {
      try {
        const r = await fetch("/models/page_models.html");
        if (r.ok) {
          const html = await r.text();
          const host = document.getElementById("swipeContainer") || document.body;
          host.insertAdjacentHTML("beforeend", html);
        }
      } catch (_) { /* ignore */ }
    }
    const navBtn = document.getElementById("btnModels");
    if (navBtn && !navBtn.dataset.bound) {
      navBtn.dataset.bound = "1";
      navBtn.addEventListener("click", () => {
        document.querySelectorAll(".page").forEach((p) => { p.style.display = "none"; });
        const page = document.getElementById("pageModels");
        if (page) page.style.display = "block";
        document.querySelectorAll(".nav-btn").forEach((b) => b.classList.remove("active"));
        navBtn.classList.add("active");
        window.ModelSelector.onShow();
      });
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", bootstrap);
  } else {
    bootstrap();
  }
})();
