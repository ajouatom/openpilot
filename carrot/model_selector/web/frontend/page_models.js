// Model Selector — carrot web page.
//
// Loaded on-demand when the user navigates to #pageModels.  Keeps its own
// polling loop for the currently active install job.

(function () {
  "use strict";

  const POLL_INTERVAL_MS = 320;

  let currentJobId = null;
  let pollTimer = null;
  let currentModel = "";
  let pendingModel = "";
  let selectedModelId = "";
  let modelsCache = [];

  const $ = (id) => document.getElementById(id);

  async function confirmPopup(message, opts) {
    if (typeof window.appConfirm === "function") {
      try {
        return await window.appConfirm(message, opts || {});
      } catch (_) { /* fall through to native */ }
    }
    return window.confirm(message);
  }

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
    const merged = Object.assign({ cache: "no-store" }, opts || {});
    const resp = await fetch(url, merged);
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
      currentModel = s.current_model || "";
      pendingModel = s.pending_model || "";
      $("msCurrent").textContent = currentModel || "기본 (내장)";
      $("msDescription").textContent = s.description || "커스텀 모델 없음";
      $("msDisk").textContent = s.disk_free_mb >= 0 ? s.disk_free_mb : "?";
      const pendingChip = $("msPendingChip");
      if (pendingModel) {
        $("msPending").textContent = pendingModel;
        pendingChip.style.display = "";
      } else {
        pendingChip.style.display = "none";
      }
    } catch (e) {
      showError("status: " + e.message);
    }
  }

  function hasFile(m, fname) {
    return Array.isArray(m.files) && m.files.some((f) => f && f.name === fname);
  }

  // has_* 플래그가 없는 구버전 백엔드와 조합돼도 files 목록으로 판별한다.
  function archLabel(m) {
    if (m.has_supercombo || hasFile(m, "driving_supercombo.onnx")) return "supercombo";
    const off = m.has_off_policy || hasFile(m, "driving_off_policy.onnx");
    const on = m.has_on_policy || hasFile(m, "driving_on_policy.onnx");
    if (off) return on ? "on+off policy" : "policy + off";
    return on ? "on-policy" : "policy";
  }

  function sortModels(list) {
    const opts = { numeric: true, sensitivity: "base" };
    return list.slice().sort((a, b) => {
      const da = a.added_at || "";
      const db = b.added_at || "";
      if (da !== db) return db.localeCompare(da);
      return (b.name || "").localeCompare(a.name || "", undefined, opts);
    });
  }

  function rowHtml(m) {
    const isCurrent = m.id === currentModel || m.name === currentModel;
    const isPending = m.id === pendingModel || m.name === pendingModel;
    const isSelected = m.id === selectedModelId;
    const cls = [
      "ms-row",
      isCurrent ? "is-current" : "",
      isPending ? "is-pending" : "",
      isSelected ? "is-selected" : "",
    ].filter(Boolean).join(" ");

    const badges = [];
    if (isCurrent) badges.push(`<span class="ms-chip ms-chip--current">현재</span>`);
    else if (isPending) badges.push(`<span class="ms-chip ms-chip--pending">설치중</span>`);

    const btnLabel = isCurrent ? "재설치" : (isPending ? "다운로드됨" : "설치");
    const btnDisabled = isPending ? "disabled" : "";
    const btnClass = isCurrent ? "btn" : "btn btn--filled";

    return `
      <div class="${cls}" data-id="${escapeHtml(m.id)}">
        <div class="ms-row__main">
          <div class="ms-row__name">
            <span>${escapeHtml(m.name)}</span>
            ${badges.join("")}
          </div>
          <div class="ms-row__meta">
            <span>${escapeHtml(m.added_at || "—")}</span>
            <span>•</span>
            <span>${escapeHtml(archLabel(m))}</span>
            <span>•</span>
            <span>${fmtMB(m.total_size)}</span>
          </div>
        </div>
        <div class="ms-row__actions">
          <button class="${btnClass} ms-install-btn" data-id="${escapeHtml(m.id)}" ${btnDisabled}>${btnLabel}</button>
        </div>
      </div>`;
  }

  function escapeHtml(s) {
    return String(s == null ? "" : s)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  function renderList() {
    const body = $("msModelsBody");
    if (!modelsCache.length) {
      body.innerHTML = `<div class="ms-empty">사용 가능한 모델이 없습니다.</div>`;
      return;
    }
    body.innerHTML = modelsCache.map(rowHtml).join("");
    Array.from(body.querySelectorAll(".ms-install-btn")).forEach((btn) => {
      btn.addEventListener("click", (e) => {
        e.stopPropagation();
        onInstall(btn.dataset.id);
      });
    });
    Array.from(body.querySelectorAll(".ms-row")).forEach((row) => {
      row.addEventListener("click", () => {
        selectedModelId = row.dataset.id;
        renderList();
      });
    });
  }

  async function refreshList() {
    const body = $("msModelsBody");
    body.innerHTML = `<div class="ms-loading">불러오는 중…</div>`;
    try {
      const d = await fetchJSON("/api/models/list");
      modelsCache = sortModels(d.models || []);
      renderList();
    } catch (e) {
      body.innerHTML = `<div class="ms-empty">불러오기 실패</div>`;
      showError("list: " + e.message);
    }
  }

  async function onInstall(modelId) {
    if (currentJobId) {
      showError("이미 설치가 진행 중입니다");
      return;
    }
    const m = modelsCache.find((x) => x.id === modelId);
    const name = m ? m.name : modelId;
    const sizeStr = m ? fmtMB(m.total_size) : "";
    const isCurrent = m && (m.id === currentModel || m.name === currentModel);
    const verb = isCurrent ? "재설치" : "설치";
    const msg = `"${name}"${sizeStr ? " (" + sizeStr + ")" : ""} 모델을 ${verb}합니다.\n다운로드 후 자동 재부팅됩니다.`;
    const ok = await confirmPopup(msg, { title: `모델 ${verb}`, confirmLabel: verb });
    if (!ok) return;
    showError("");
    try {
      const d = await fetchJSON("/api/models/install", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ id: modelId }),
      });
      currentJobId = d.id;
      showProgress(true, 0, "시작 중…");
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

  function rebootCountdown(seconds, label) {
    const total = seconds;
    let elapsed = 0;
    showProgress(true, 0, `${label} — ${total}초 후 재부팅…`);
    const t = setInterval(() => {
      elapsed += 1;
      const remaining = total - elapsed;
      if (remaining <= 0) {
        clearInterval(t);
        showProgress(true, 100, `${label} — 재부팅 중…`);
        return;
      }
      const pct = (elapsed / total) * 100;
      showProgress(true, pct, `${label} — ${remaining}초 후 재부팅…`);
    }, 1000);
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
          try {
            const d = await fetchJSON("/api/models/apply", { method: "POST" });
            await refreshStatus();
            renderList();
            rebootCountdown(d.reboot_in || 5, "다운로드 완료");
          } catch (e) {
            showError("apply: " + e.message);
          }
        } else {
          showProgress(false, 0, "");
          showError(snap.error || "설치 실패");
        }
      }
    } catch (e) {
      stopPoll();
      currentJobId = null;
      showProgress(false, 0, "");
      showError("poll: " + e.message);
    }
  }

  async function onReset() {
    const ok = await confirmPopup(
      "기본(내장) 모델로 되돌립니다.\n자동 재부팅됩니다.",
      { title: "기본 모델 복원", confirmLabel: "복원" },
    );
    if (!ok) return;
    showError("");
    try {
      const d = await fetchJSON("/api/models/reset", { method: "POST" });
      await refreshStatus();
      renderList();
      rebootCountdown(d.reboot_in || 5, "기본 모델 복원");
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

  function integrateWithRouter() {
    const pageEl = document.getElementById("pageModels");
    if (!pageEl) return false;
    try {
      if (typeof PAGE_ELEMENTS === "object" && PAGE_ELEMENTS && !PAGE_ELEMENTS.models) {
        PAGE_ELEMENTS.models = pageEl;
      }
    } catch (_) { /* ignore */ }
    return true;
  }

  function activateModelsPage() {
    const pageEl = document.getElementById("pageModels");
    if (!pageEl) return;
    const integrated = integrateWithRouter();
    if (integrated && typeof window.showPage === "function") {
      window.showPage("models", true);
    } else {
      document.querySelectorAll(".page").forEach((p) => { p.style.display = "none"; });
      pageEl.style.display = "block";
      document.body.dataset.page = "models";
    }
    document.querySelectorAll(".nav-btn").forEach((b) => b.classList.remove("active"));
    const navBtn = document.getElementById("btnModels");
    if (navBtn) navBtn.classList.add("active");
    try { window.scrollTo(0, 0); } catch (_) { /* ignore */ }
    window.ModelSelector.onShow();
  }

  window.addEventListener("carrot:pagechange", (e) => {
    const cur = e && e.detail && e.detail.page;
    const navBtn = document.getElementById("btnModels");
    if (navBtn) navBtn.classList.toggle("active", cur === "models");
    if (cur && cur !== "models") {
      window.ModelSelector.onHide();
    }
  });

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
    integrateWithRouter();

    const navBtn = document.getElementById("btnModels");
    if (navBtn && !navBtn.dataset.bound) {
      navBtn.dataset.bound = "1";
      navBtn.addEventListener("click", activateModelsPage);
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", bootstrap);
  } else {
    bootstrap();
  }
})();
