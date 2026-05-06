#!/usr/bin/env python3
"""Tiny recovery web server.

This intentionally avoids importing openpilot modules. It is meant to keep a
small Git/terminal surface available when the main web stack is broken.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 6999
TIMEOUT_SEC = 45
TMUX_SESSION = "carrot-recovery"
TMUX_CAPTURE_LINES = 1600

GIT_ACTIONS = {
  "git_pull",
  "git_sync",
  "git_reset",
  "git_log",
  "git_branches",
  "git_checkout",
  "git_checkout_commit",
  "git_reset_repo",
  "git_rebuild",
  "git_reboot",
}

HTML_PAGE = """<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Recovery</title>
<style>
:root {
  --md-surface: #090c10;
  --md-surface-cont: #11161d;
  --md-surface-cont-h: #1b222c;
  --md-surface-cont-hh: #262f3b;
  --md-on-surface: #ffffff;
  --md-on-surface-var: #eef2f8;
  --md-outline: #cbd2de;
  --md-outline-var: #7c8594;
  --md-stroke-soft: #788191;
  --md-stroke-strong: #aeb7c8;
  --md-primary: #ffb06d;
  --md-on-primary: #3a1800;
  --md-primary-cont: #7b3e10;
  --md-on-primary-cont: #fff0e2;
  --md-error: #ff9d94;
  --md-on-error: #690005;
  --md-error-cont: #93000a;
  --md-on-error-cont: #ffdad6;
  --sp-xs: 4px;
  --sp-sm: 8px;
  --sp-md: 12px;
  --sp-lg: 16px;
  --r-sm: 8px;
  --r-md: 12px;
  --r-lg: 16px;
  --r-pill: 999px;
  --fs-body-sm: 14px;
  --fs-body-md: 16px;
  --fs-label-lg: 16px;
  --fs-title-sm: 16px;
  --shadow-2: 0 18px 44px rgba(0, 0, 0, 0.34);
  --font-sans: "Roboto", system-ui, -apple-system, sans-serif;
  --font-mono: ui-monospace, "Roboto Mono", SFMono-Regular, Menlo, monospace;
  --vv-height: 100dvh;
  --vv-top: 0px;
}
* { box-sizing: border-box; }
html, body { height: 100%; margin: 0; }
body {
  background: var(--md-surface);
  color: var(--md-on-surface);
  font-family: var(--font-sans);
  font-size: var(--fs-body-sm);
  line-height: 1.5;
  -webkit-tap-highlight-color: transparent;
  overflow: hidden;
}
.shell {
  position: fixed;
  left: 0;
  right: 0;
  top: var(--vv-top);
  height: var(--vv-height);
  display: flex;
  flex-direction: column;
  gap: var(--sp-sm);
  padding: max(var(--sp-md), env(safe-area-inset-top, 0px))
           max(var(--sp-md), env(safe-area-inset-right, 0px))
           max(var(--sp-sm), env(safe-area-inset-bottom, 0px))
           max(var(--sp-md), env(safe-area-inset-left, 0px));
}
.head {
  flex: 0 0 auto;
  margin: 0;
  padding: 2px 4px;
  font-size: var(--fs-title-sm);
  font-weight: 800;
  color: var(--md-primary);
  letter-spacing: 0.4px;
}
.menus {
  flex: 0 0 auto;
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: var(--sp-sm);
  position: relative;
  z-index: 50;
}
.menu-wrap { position: relative; }
.menu-trigger {
  width: 100%;
  min-width: 0;
  min-height: 46px;
  margin: 0;
  padding: 0 12px;
  border: 1px solid color-mix(in srgb, var(--md-outline-var) 46%, transparent);
  background: var(--md-surface-cont);
  color: var(--md-on-surface);
  border-radius: var(--r-sm);
  cursor: pointer;
  font-family: inherit;
  font-size: var(--fs-label-lg);
  font-weight: 750;
  letter-spacing: 0.1px;
  display: inline-flex;
  align-items: center;
  justify-content: space-between;
  gap: 10px;
  text-align: left;
  line-height: 1.18;
  transition: background 0.14s ease, border-color 0.14s ease, color 0.14s ease;
}
.menu-trigger:hover, .menu-trigger:focus-visible {
  border-color: color-mix(in srgb, var(--md-primary) 36%, var(--md-outline-var));
  background: color-mix(in srgb, var(--md-surface-cont-h) 92%, var(--md-primary));
  outline: none;
}
.menu-trigger.is-open {
  border-color: color-mix(in srgb, var(--md-primary) 56%, var(--md-outline-var));
  background: color-mix(in srgb, var(--md-surface-cont-h) 88%, var(--md-primary));
  color: var(--md-primary);
}
.menu-trigger__caret { font-size: 11px; opacity: 0.7; }
.menu {
  position: absolute;
  top: calc(100% + 4px);
  left: 0;
  right: 0;
  z-index: 60;
  display: flex;
  flex-direction: column;
  gap: 2px;
  padding: 4px;
  background: color-mix(in srgb, var(--md-surface-cont) 96%, #000);
  border: 1px solid color-mix(in srgb, var(--md-stroke-soft) 92%, transparent);
  border-radius: var(--r-md);
  box-shadow: var(--shadow-2);
}
.menu[hidden] { display: none; }
.menu button {
  text-align: left;
  background: transparent;
  color: var(--md-on-surface);
  border: 0;
  border-radius: var(--r-sm);
  padding: 10px 12px;
  font-family: inherit;
  font-size: var(--fs-body-sm);
  font-weight: 600;
  cursor: pointer;
  letter-spacing: 0.1px;
}
.menu button:hover {
  background: var(--md-surface-cont-h);
}
.menu button.danger {
  color: color-mix(in srgb, var(--md-error) 78%, var(--md-on-surface));
}
.menu button.danger:hover {
  background: color-mix(in srgb, var(--md-error-cont) 18%, var(--md-surface-cont-h));
}

.terminal-shell {
  flex: 1 1 auto;
  min-height: 0;
  display: flex;
  flex-direction: column;
  background: color-mix(in srgb, var(--md-surface) 92%, #000);
  border: 1px solid color-mix(in srgb, var(--md-stroke-soft) 42%, transparent);
  border-radius: var(--r-md);
  overflow: hidden;
}
.terminal-screen {
  flex: 1 1 auto;
  min-height: 0;
  overflow: auto;
  overscroll-behavior: contain;
  padding: var(--sp-md);
}
.terminal-output {
  margin: 0;
  background: transparent;
  color: var(--md-on-surface);
  font-family: var(--font-mono);
  font-size: 12px;
  line-height: 1.45;
  white-space: pre;
}
.terminal-output__promptHost {
  color: color-mix(in srgb, #67e27a 88%, #d7ffe0);
  font-weight: 700;
}
.terminal-form {
  flex: 0 0 auto;
  display: grid;
  grid-template-columns: auto minmax(0, 1fr) auto;
  gap: var(--sp-sm);
  align-items: center;
  margin: var(--sp-sm);
  padding-left: 14px;
  border: 1px solid color-mix(in srgb, var(--md-stroke-soft) 56%, transparent);
  border-radius: var(--r-pill);
  background: color-mix(in srgb, var(--md-surface-cont) 92%, #000);
  overflow: hidden;
}
.terminal-form:focus-within {
  border-color: color-mix(in srgb, var(--md-stroke-strong) 76%, transparent);
}
.terminal-form__prompt {
  color: var(--md-primary);
  font-family: var(--font-mono);
  font-size: var(--fs-body-sm);
  font-weight: 700;
}
.terminal-form__input {
  min-width: 0;
  min-height: 44px;
  padding: 10px 0;
  border: 0;
  border-radius: 0;
  background: transparent;
  color: var(--md-on-surface);
  font-size: var(--fs-body-sm);
  font-family: var(--font-mono);
  line-height: 1.25;
  outline: none;
  appearance: none;
}
.smallBtn {
  align-self: stretch;
  padding: 0 16px;
  border: 0;
  border-left: 1px solid color-mix(in srgb, var(--md-stroke-soft) 44%, transparent);
  border-radius: 0;
  background: color-mix(in srgb, var(--md-primary) 16%, transparent);
  color: var(--md-primary);
  font-family: inherit;
  font-size: 12px;
  font-weight: 700;
  cursor: pointer;
}
.smallBtn:hover {
  background: color-mix(in srgb, var(--md-primary) 24%, transparent);
}

.modal-bg {
  position: fixed;
  inset: 0;
  background: color-mix(in srgb, #000 56%, transparent);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 100;
  padding: var(--sp-lg);
}
.modal {
  background: color-mix(in srgb, var(--md-surface-cont) 96%, #000);
  border: 1px solid color-mix(in srgb, var(--md-stroke-soft) 92%, transparent);
  border-radius: var(--r-lg);
  padding: var(--sp-lg);
  width: min(100%, 460px);
  max-height: calc(100dvh - var(--sp-lg) * 2);
  display: flex;
  flex-direction: column;
  gap: var(--sp-md);
}
.modal h2 {
  margin: 0;
  font-size: var(--fs-title-sm);
  font-weight: 800;
  color: var(--md-on-surface);
}
.modal p {
  margin: 0;
  color: var(--md-on-surface-var);
  font-size: var(--fs-body-md);
  line-height: 1.55;
  white-space: pre-wrap;
  word-break: break-word;
}
.modal .list {
  flex: 1;
  min-height: 0;
  overflow: auto;
  display: flex;
  flex-direction: column;
  gap: var(--sp-sm);
  padding-right: 2px;
}
.modal .list button {
  width: 100%;
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: var(--sp-sm);
  text-align: left;
  background: var(--md-surface-cont-h);
  color: var(--md-on-surface);
  border: 1px solid color-mix(in srgb, var(--md-outline-var) 46%, transparent);
  border-radius: var(--r-pill);
  padding: 10px 14px;
  font-family: var(--font-mono);
  font-size: var(--fs-body-sm);
  font-weight: 600;
  min-height: 44px;
  cursor: pointer;
  white-space: nowrap;
  transition: background 0.14s ease, border-color 0.14s ease, color 0.14s ease;
}
.modal .list .label {
  min-width: 0;
  flex: 1 1 auto;
  overflow-x: auto;
  overflow-y: hidden;
  text-overflow: clip;
  scrollbar-width: none;
  -webkit-overflow-scrolling: touch;
}
.modal .list .label::-webkit-scrollbar { display: none; }
.modal .list button:hover {
  border-color: color-mix(in srgb, var(--md-outline) 60%, var(--md-outline-var));
  background: var(--md-surface-cont-hh);
}
.modal .list button.sel {
  border-color: var(--md-primary);
  box-shadow: inset 0 0 0 1px var(--md-primary);
}
.modal .viewer {
  flex: 1;
  min-height: 0;
  margin: 0;
  padding: var(--sp-md);
  background: color-mix(in srgb, var(--md-surface) 92%, #000);
  border: 1px solid color-mix(in srgb, var(--md-stroke-soft) 42%, transparent);
  border-radius: var(--r-sm);
  font-family: var(--font-mono);
  font-size: 12px;
  line-height: 1.45;
  color: var(--md-on-surface-var);
  white-space: pre;
  overflow: auto;
}
.modal .row {
  display: flex;
  justify-content: flex-end;
  gap: var(--sp-sm);
  flex-wrap: wrap;
  margin-top: var(--sp-sm);
}
.modal .row button {
  padding: 10px 20px;
  border: 1px solid color-mix(in srgb, var(--md-outline-var) 46%, transparent);
  background: var(--md-surface-cont-h);
  color: var(--md-on-surface);
  border-radius: var(--r-pill);
  font-family: inherit;
  font-size: var(--fs-label-lg);
  font-weight: 750;
  min-height: 44px;
  letter-spacing: 0.1px;
  cursor: pointer;
  transition: background 0.14s ease, border-color 0.14s ease, color 0.14s ease;
}
.modal .row button:hover {
  border-color: color-mix(in srgb, var(--md-primary) 38%, var(--md-outline-var));
  background: color-mix(in srgb, var(--md-surface-cont-h) 88%, var(--md-primary));
}
.modal .row button.primary {
  background: var(--md-primary);
  border-color: color-mix(in srgb, var(--md-primary) 76%, var(--md-outline-var));
  color: var(--md-on-primary);
}
.modal .row button.danger { color: var(--md-error); }

@media (orientation: landscape) and (max-height: 700px) {
  .shell { gap: var(--sp-xs); }
  .head { font-size: 14px; padding: 0 4px; }
  .menu-trigger {
    min-height: 40px;
    padding: 0 10px;
    font-size: var(--fs-body-sm);
  }
  .menu {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(110px, 1fr));
    gap: 2px;
    padding: 3px;
  }
  .menu button {
    padding: 7px 10px;
    font-size: 13px;
    min-height: 36px;
  }
  .terminal-form__input { min-height: 38px; padding: 6px 0; }
  .smallBtn { padding: 0 12px; }
}
</style>
</head>
<body>
<div class="shell">
  <h1 class="head">carrot recovery</h1>
  <div class="menus">
    <div class="menu-wrap">
      <button class="menu-trigger" data-menu="git">
        <span>Git</span><span class="menu-trigger__caret">&#x25BE;</span>
      </button>
      <div class="menu" id="gitMenu" hidden>
        <button data-act="git_pull">git pull</button>
        <button data-act="git_sync">git sync</button>
        <button data-act="git_reset">git reset</button>
        <button data-act="git_log">git log</button>
        <button data-act="git_branches">change branch</button>
        <button data-act="git_reset_repo">reset repo</button>
        <button data-act="git_rebuild">rebuild</button>
        <button data-act="git_reboot" class="danger">reboot</button>
      </div>
    </div>
    <div class="menu-wrap">
      <button class="menu-trigger" data-menu="term">
        <span>Terminal</span><span class="menu-trigger__caret">&#x25BE;</span>
      </button>
      <div class="menu" id="termMenu" hidden>
        <button data-tctrl="ctrl_c">Ctrl+C</button>
        <button data-tctrl="clear">Clear</button>
        <button data-tctrl="new_session">Reconnect</button>
      </div>
    </div>
  </div>
  <div class="terminal-shell">
    <div class="terminal-screen" id="terminalScreen">
      <pre class="terminal-output" id="terminalOutput"></pre>
    </div>
    <form class="terminal-form" id="terminalForm">
      <span class="terminal-form__prompt" aria-hidden="true">&gt;</span>
      <input class="terminal-form__input" id="terminalInput" type="text"
             autocomplete="off" autocapitalize="none" spellcheck="false" autocorrect="off">
      <button class="smallBtn" id="terminalSend" type="submit">Send</button>
    </form>
  </div>
</div>
<div id="modalRoot" hidden></div>
<script>
const termOut = document.getElementById("terminalOutput");
const termScreen = document.getElementById("terminalScreen");
const termInput = document.getElementById("terminalInput");
const termForm = document.getElementById("terminalForm");
const modalRoot = document.getElementById("modalRoot");
let terminalTimer = 0;
let terminalLast = "";

function updateVV() {
  const vv = window.visualViewport;
  const height = Math.max(320, Math.round((vv && vv.height) || window.innerHeight || 0));
  const top = Math.max(0, Math.round((vv && vv.offsetTop) || 0));
  const root = document.documentElement.style;
  root.setProperty("--vv-height", height + "px");
  root.setProperty("--vv-top", top + "px");
}
window.addEventListener("resize", updateVV, { passive: true });
window.addEventListener("orientationchange", updateVV, { passive: true });
if (window.visualViewport) {
  window.visualViewport.addEventListener("resize", updateVV, { passive: true });
  window.visualViewport.addEventListener("scroll", updateVV, { passive: true });
}
updateVV();

function showModal(node) { modalRoot.replaceChildren(node); modalRoot.hidden = false; }
function closeModal() { modalRoot.hidden = true; modalRoot.replaceChildren(); }
function buildModal(title, message) {
  const back = document.createElement("div");
  back.className = "modal-bg";
  const m = document.createElement("div");
  m.className = "modal";
  if (title) { const h = document.createElement("h2"); h.textContent = title; m.appendChild(h); }
  if (message) { const p = document.createElement("p"); p.textContent = message; m.appendChild(p); }
  back.appendChild(m);
  back.onclick = (e) => { if (e.target === back) closeModal(); };
  function addRow(actions) {
    const row = document.createElement("div");
    row.className = "row";
    for (const a of actions) {
      const b = document.createElement("button");
      b.textContent = a.label;
      if (a.kind) b.classList.add(a.kind);
      b.onclick = () => { closeModal(); if (a.onClick) a.onClick(); };
      row.appendChild(b);
    }
    m.appendChild(row);
  }
  return { back, body: m, addRow };
}
function confirmDialog(title, message, opts) {
  opts = opts || {};
  return new Promise((resolve) => {
    const { back, addRow } = buildModal(title, message);
    addRow([
      { label: "Cancel", onClick: () => resolve(false) },
      { label: opts.confirmLabel || "OK", kind: opts.danger ? "danger" : "primary", onClick: () => resolve(true) },
    ]);
    showModal(back);
  });
}
function viewerDialog(title, text) {
  return new Promise((resolve) => {
    const { back, body, addRow } = buildModal(title, "");
    const pre = document.createElement("pre");
    pre.className = "viewer";
    pre.textContent = text;
    body.appendChild(pre);
    addRow([
      { label: "Close", kind: "primary", onClick: () => resolve() },
    ]);
    showModal(back);
  });
}
function pickerDialog(title, items, opts) {
  opts = opts || {};
  return new Promise((resolve) => {
    const { back, body, addRow } = buildModal(title, opts.message || "");
    const list = document.createElement("div");
    list.className = "list";
    let chosen = opts.selected || null;
    const buttons = [];
    for (const item of items) {
      const b = document.createElement("button");
      const label = document.createElement("span");
      label.className = "label";
      label.textContent = item.label || item.value;
      b.appendChild(label);
      if (item.value === chosen) b.classList.add("sel");
      b.onclick = () => {
        chosen = item.value;
        for (const x of buttons) x.classList.remove("sel");
        b.classList.add("sel");
      };
      list.appendChild(b);
      buttons.push(b);
    }
    body.appendChild(list);
    addRow([
      { label: "Cancel", onClick: () => resolve(null) },
      { label: opts.confirmLabel || "OK", kind: opts.danger ? "danger" : "primary",
        onClick: () => resolve(chosen) },
    ]);
    showModal(back);
  });
}

function closeAllMenus() {
  for (const t of document.querySelectorAll(".menu-trigger")) t.classList.remove("is-open");
  for (const m of document.querySelectorAll(".menu")) m.hidden = true;
}
function openMenu(name) {
  closeAllMenus();
  const trigger = document.querySelector('.menu-trigger[data-menu="' + name + '"]');
  const menu = document.getElementById(name + "Menu");
  if (trigger) trigger.classList.add("is-open");
  if (menu) menu.hidden = false;
}
function toggleMenu(name) {
  const menu = document.getElementById(name + "Menu");
  if (menu && !menu.hidden) closeAllMenus();
  else openMenu(name);
}
document.addEventListener("click", (e) => {
  if (e.target.closest(".menu-wrap")) return;
  closeAllMenus();
});
document.querySelectorAll(".menu-trigger").forEach((btn) => {
  btn.onclick = (e) => { e.stopPropagation(); toggleMenu(btn.dataset.menu); };
});

async function callAction(action, payload) {
  const r = await fetch("/api/action", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(Object.assign({ action }, payload || {})),
  });
  return r.json();
}
async function terminalSend(data) {
  await fetch("/api/terminal/input", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ data }),
  });
  await terminalScreen(true);
}
async function terminalControl(action) {
  await fetch("/api/terminal/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action }),
  });
  await terminalScreen(true);
}

async function dispatchGit(action, payload) {
  const data = await callAction(action, payload);
  if (data.rebooting) return data;
  if (data.command) await terminalSend(data.command);
  return data;
}
const GIT_CONFIRMS = {
  git_pull: {
    title: "git pull",
    message: "Pull latest commits from remote.\\nLocal changes will be reset first.",
    confirmLabel: "Pull",
    danger: true,
  },
  git_sync: {
    title: "git sync",
    message: "Delete all local branches except the current one,\\nthen fetch from remote.",
    confirmLabel: "Sync",
    danger: true,
  },
  git_reset: {
    title: "git reset",
    message: "Reset the current branch to the last commit.\\nAll local changes will be lost.",
    confirmLabel: "Reset",
    danger: true,
  },
  git_rebuild: {
    title: "rebuild",
    message: "Clean all build cache.\\nWill rebuild on next boot.\\n\\n• scons -c\\n• remove .sconsign.dblite, /tmp/scons_cache, prebuilt",
    confirmLabel: "Clean",
    danger: true,
  },
};

async function clickGit(action) {
  closeAllMenus();
  if (action === "git_branches") return clickChangeBranch();
  if (action === "git_reset_repo") return clickResetRepo();
  if (action === "git_reboot") return clickReboot();
  if (action === "git_log") return clickGitLog();
  const c = GIT_CONFIRMS[action];
  if (c) {
    const ok = await confirmDialog(c.title, c.message, { confirmLabel: c.confirmLabel, danger: c.danger });
    if (!ok) return;
  }
  await dispatchGit(action);
}
async function clickGitLog() {
  const data = await callAction("git_log");
  if (!data.ok) {
    await viewerDialog("git log", data.error || "failed to read log");
    return;
  }
  const commits = data.commits || [];
  if (!commits.length) {
    await viewerDialog("git log", "(no commits)");
    return;
  }
  const items = commits.map((c) => ({
    value: c.hash,
    label: c.hash + "  " + (c.message || ""),
  }));
  const selected = await pickerDialog("git log", items, {
    selected: data.current,
    message: "Select a commit to checkout.",
    confirmLabel: "Checkout",
  });
  if (!selected) return;
  if (data.current && selected.startsWith(data.current)) return;
  const ok = await confirmDialog(
    "Checkout commit",
    "Move to the selected commit.\\n\\n" + selected,
    { confirmLabel: "Checkout" },
  );
  if (!ok) return;
  await dispatchGit("git_checkout_commit", { commit: selected });
}
async function clickChangeBranch() {
  const data = await callAction("git_branches");
  if (!data.ok) return;
  const items = data.branches.map((b) => ({
    value: b.name,
    label: (b.kind === "remote" ? "↗ " : "  ") + b.name,
  }));
  const choice = await pickerDialog("Change branch", items, {
    selected: data.current,
    message: "Select a branch to switch to.",
    confirmLabel: "Switch",
  });
  if (!choice) return;
  if (choice === data.current) return;
  await dispatchGit("git_checkout", { branch: choice });
}
async function clickResetRepo() {
  const data = await callAction("git_branches");
  if (!data.ok) return;
  const items = data.branches.map((b) => ({
    value: b.name,
    label: (b.kind === "remote" ? "↗ " : "  ") + b.name,
  }));
  const choice = await pickerDialog("Reset repo", items, {
    selected: "c3-wip",
    message: "Fetch the selected branch fresh.\\nAll local changes and untracked files will be lost.",
    confirmLabel: "Next",
    danger: true,
  });
  if (!choice) return;
  const ok = await confirmDialog(
    "Confirm reset",
    "Branch: " + choice + "\\nRemote: ajouatom/openpilot.git\\n\\nThis cannot be undone.",
    { confirmLabel: "Reset", danger: true },
  );
  if (!ok) return;
  await dispatchGit("git_reset_repo", { branch: choice });
}
async function clickReboot() {
  const ok1 = await confirmDialog("Reboot", "The device will restart immediately.", { confirmLabel: "Next", danger: true });
  if (!ok1) return;
  const ok2 = await confirmDialog("Confirm reboot", "Really reboot the device?", { confirmLabel: "Reboot", danger: true });
  if (!ok2) return;
  await dispatchGit("git_reboot");
}

document.querySelectorAll("[data-act]").forEach((btn) => {
  btn.onclick = (e) => { e.stopPropagation(); clickGit(btn.dataset.act); };
});
document.querySelectorAll("[data-tctrl]").forEach((btn) => {
  btn.onclick = async (e) => {
    e.stopPropagation();
    closeAllMenus();
    await terminalControl(btn.dataset.tctrl);
  };
});

async function terminalScreen(force) {
  try {
    const r = await fetch("/api/terminal/screen");
    const j = await r.json();
    if (!j.ok) { termOut.textContent = j.error || "terminal unavailable"; return; }
    if (force || j.text !== terminalLast) {
      const stick = termScreen.scrollHeight - termScreen.scrollTop - termScreen.clientHeight < 32;
      terminalLast = j.text || " ";
      termOut.textContent = terminalLast;
      if (stick || force) termScreen.scrollTop = termScreen.scrollHeight;
    }
  } catch (e) { termOut.textContent = e.message; }
}
function startTerminal() {
  clearInterval(terminalTimer);
  terminalTimer = setInterval(() => terminalScreen(false), 250);
  terminalScreen(true);
}

termForm.addEventListener("submit", (e) => {
  e.preventDefault();
  const v = termInput.value;
  termInput.value = "";
  terminalSend(v);
});

startTerminal();
</script>
</body>
</html>
"""


def _json_response(handler: BaseHTTPRequestHandler, status: int, payload: dict) -> None:
  data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
  handler.send_response(status)
  handler.send_header("Content-Type", "application/json; charset=utf-8")
  handler.send_header("Content-Length", str(len(data)))
  handler.send_header("Cache-Control", "no-store")
  handler.end_headers()
  handler.wfile.write(data)


def _html_response(handler: BaseHTTPRequestHandler) -> None:
  data = HTML_PAGE.encode("utf-8")
  handler.send_response(200)
  handler.send_header("Content-Type", "text/html; charset=utf-8")
  handler.send_header("Content-Length", str(len(data)))
  handler.send_header("Cache-Control", "no-store")
  handler.end_headers()
  handler.wfile.write(data)


def _read_json(handler: BaseHTTPRequestHandler) -> dict:
  length = int(handler.headers.get("Content-Length", "0"))
  if length > 4096:
    raise ValueError("request too large")
  body = handler.rfile.read(length).decode("utf-8")
  return json.loads(body or "{}")


def _run_exec(args: list[str], timeout: float = TIMEOUT_SEC) -> tuple[int, str]:
  proc = subprocess.run(
    args,
    cwd=str(REPO_ROOT),
    text=True,
    capture_output=True,
    timeout=timeout,
    check=False,
  )
  return proc.returncode, (proc.stdout or "") + (proc.stderr or "")


def _git_branches() -> dict:
  _run_exec(["git", "fetch", "--all", "--prune"], 180)
  current = (_run_exec(["git", "branch", "--show-current"], 15)[1] or "").strip()
  rc, out = _run_exec(
    ["git", "for-each-ref",
     "--format=%(refname:short)\t%(refname)\t%(upstream:short)",
     "refs/heads/", "refs/remotes/"],
    30,
  )
  if rc != 0:
    return {"ok": False, "error": out}
  branches: list[dict] = []
  for line in out.splitlines():
    parts = line.split("\t")
    if len(parts) < 2:
      continue
    name = parts[0]
    fullref = parts[1]
    tracking = parts[2] if len(parts) > 2 else ""
    if name.endswith("/HEAD"):
      continue
    if fullref.startswith("refs/heads/"):
      kind = "local"
    elif fullref.startswith("refs/remotes/"):
      kind = "remote"
    else:
      continue
    branches.append({"name": name, "kind": kind, "tracking": tracking})
  return {"ok": True, "current": current, "branches": branches}


def _git_reboot() -> dict:
  for cmd in (["sudo", "reboot"], ["reboot"]):
    try:
      subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
      return {"ok": True, "rebooting": True}
    except FileNotFoundError:
      continue
    except Exception as exc:
      return {"ok": False, "error": str(exc)}
  return {"ok": False, "error": "reboot not available"}


def _git_command(action: str, payload: dict) -> str | None:
  """Returns shell command string to inject into tmux. None if invalid."""
  if action == "git_pull":
    return "git reset --hard && git pull"
  if action == "git_sync":
    current = (_run_exec(["git", "branch", "--show-current"], 15)[1] or "").strip()
    rc, out = _run_exec(["git", "branch", "--format=%(refname:short)"], 30)
    parts: list[str] = []
    if rc == 0:
      for line in (out or "").splitlines():
        b = line.strip()
        if b and b != current:
          parts.append(f"git branch -D {shlex.quote(b)}")
    parts.append("git fetch --all --prune")
    return " && ".join(parts)
  if action == "git_reset":
    return "git reset --hard HEAD"
  if action == "git_log":
    return "git log --oneline --decorate -30"
  if action == "git_checkout":
    branch = str(payload.get("branch") or "").strip()
    if not branch:
      return None
    if branch.startswith("origin/"):
      local = branch.split("/", 1)[1]
      return (
        "git fetch --all --prune && "
        f"git switch -C {shlex.quote(local)} --track {shlex.quote(branch)}"
      )
    return f"git fetch --all --prune && git switch {shlex.quote(branch)}"
  if action == "git_checkout_commit":
    commit = str(payload.get("commit") or "").strip()
    if not commit:
      return None
    return f"git checkout {shlex.quote(commit)}"
  if action == "git_reset_repo":
    branch = str(payload.get("branch") or "").strip()
    if not branch:
      return None
    return (
      "git remote set-url origin https://github.com/ajouatom/openpilot.git && "
      "git fetch origin --prune && "
      f"git checkout -B {shlex.quote(branch)} origin/{shlex.quote(branch)} && "
      f"git reset --hard origin/{shlex.quote(branch)} && "
      "git clean -xfd"
    )
  if action == "git_rebuild":
    return (
      "cd /data/openpilot && "
      "scons -c && "
      "rm -f .sconsign.dblite && "
      "rm -rf /tmp/scons_cache && "
      "rm -f prebuilt"
    )
  return None


def _git_log_output() -> dict:
  rc, out = _run_exec(["git", "log", "--oneline", "-30"], 30)
  if rc != 0:
    return {"ok": False, "error": out}
  rc_head, out_head = _run_exec(["git", "rev-parse", "--short", "HEAD"], 10)
  current = out_head.strip() if rc_head == 0 else ""
  commits: list[dict] = []
  for line in (out or "").splitlines():
    line = line.strip()
    if not line:
      continue
    parts = line.split(" ", 1)
    commits.append({"hash": parts[0], "message": parts[1] if len(parts) > 1 else ""})
  return {"ok": True, "commits": commits, "current": current}


def _git_action(action: str, payload: dict) -> dict:
  if action not in GIT_ACTIONS:
    return {"ok": False, "error": f"unknown action: {action}"}
  if action == "git_branches":
    return _git_branches()
  if action == "git_reboot":
    return _git_reboot()
  if action == "git_log":
    return _git_log_output()
  cmd = _git_command(action, payload)
  if cmd is None:
    return {"ok": False, "error": "invalid action or missing parameters"}
  return {"ok": True, "command": cmd}


def _tmux_run(args: list[str], timeout: float = 5.0, check: bool = False) -> subprocess.CompletedProcess:
  return subprocess.run(args, cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=timeout, check=check)


def _tmux_start_command() -> str:
  if os.name == "posix":
    return f"cd {shlex.quote(str(REPO_ROOT))} && exec bash -il"
  return "powershell -NoLogo"


def _tmux_ensure() -> None:
  if shutil.which("tmux") is None:
    raise RuntimeError("tmux not available")
  p = _tmux_run(["tmux", "has-session", "-t", TMUX_SESSION], timeout=2.5)
  if p.returncode == 0:
    return
  _tmux_run(["tmux", "new-session", "-d", "-s", TMUX_SESSION, _tmux_start_command()], timeout=5.0, check=True)


def _tmux_capture() -> str:
  _tmux_ensure()
  p = _tmux_run(
    ["tmux", "capture-pane", "-p", "-J", "-t", TMUX_SESSION, "-S", f"-{TMUX_CAPTURE_LINES}"],
    timeout=4.0,
  )
  if p.returncode != 0:
    raise RuntimeError((p.stderr or p.stdout or "tmux capture failed").strip())
  return (p.stdout or "").rstrip() or " "


def _tmux_send_line(line: str) -> None:
  _tmux_ensure()
  if line:
    _tmux_run(["tmux", "send-keys", "-t", TMUX_SESSION, "-l", line], timeout=4.0, check=True)
  _tmux_run(["tmux", "send-keys", "-t", TMUX_SESSION, "Enter"], timeout=4.0, check=True)


def _tmux_control(action: str) -> None:
  _tmux_ensure()
  if action == "ctrl_c":
    _tmux_run(["tmux", "send-keys", "-t", TMUX_SESSION, "C-c"], timeout=4.0, check=True)
  elif action == "clear":
    _tmux_send_line("clear")
    _tmux_run(["tmux", "clear-history", "-t", TMUX_SESSION], timeout=4.0)
  elif action == "new_session":
    _tmux_run(["tmux", "kill-session", "-t", TMUX_SESSION], timeout=3.0)
    _tmux_ensure()
  else:
    raise ValueError(f"unknown control: {action}")


class RecoveryHandler(BaseHTTPRequestHandler):
  server_version = "CarrotRecovery/1.0"

  def log_message(self, fmt: str, *args) -> None:
    print("[recovery]", self.address_string(), fmt % args)

  def do_GET(self) -> None:
    path = urlparse(self.path).path
    if path in ("/", "/index.html"):
      _html_response(self)
      return
    if path == "/api/terminal/screen":
      try:
        _json_response(self, 200, {"ok": True, "session": TMUX_SESSION, "text": _tmux_capture()})
      except Exception as exc:
        _json_response(self, 200, {"ok": False, "error": str(exc), "session": TMUX_SESSION, "text": ""})
      return
    _json_response(self, 404, {"ok": False, "error": "not found"})

  def do_POST(self) -> None:
    path = urlparse(self.path).path
    try:
      payload = _read_json(self)
    except Exception as exc:
      _json_response(self, 400, {"ok": False, "error": str(exc)})
      return

    if path == "/api/action":
      _json_response(self, 200, _git_action(str(payload.get("action") or "").strip(), payload))
      return
    if path == "/api/terminal/input":
      try:
        _tmux_send_line(str(payload.get("data") or ""))
        _json_response(self, 200, {"ok": True})
      except Exception as exc:
        _json_response(self, 200, {"ok": False, "error": str(exc)})
      return
    if path == "/api/terminal/control":
      try:
        _tmux_control(str(payload.get("action") or "").strip())
        _json_response(self, 200, {"ok": True})
      except Exception as exc:
        _json_response(self, 200, {"ok": False, "error": str(exc)})
      return

    _json_response(self, 404, {"ok": False, "error": "not found"})


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--host", default=DEFAULT_HOST)
  parser.add_argument("--port", type=int, default=DEFAULT_PORT)
  args = parser.parse_args()

  httpd = ThreadingHTTPServer((args.host, args.port), RecoveryHandler)
  print(f"[recovery] serving http://{args.host}:{args.port} cwd={REPO_ROOT}")
  try:
    httpd.serve_forever()
  except KeyboardInterrupt:
    pass
  finally:
    httpd.server_close()


if __name__ == "__main__":
  main()
