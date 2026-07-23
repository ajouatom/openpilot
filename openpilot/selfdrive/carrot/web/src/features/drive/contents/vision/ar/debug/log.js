"use strict";

/* 진단 이력 버퍼와 클립보드. DOM을 모른다(문서는 복사에만 쓴다).
 *
 * 상태가 실제로 바뀐 순간만 남긴다. 고속으로 왕복하는 구간도 누락 없이 보이고,
 * 변화 없는 프레임으로 버퍼가 낭비되지 않는다.
 */

import { timestamp } from "./format.js";

export const AR_DEBUG_LOG_LIMIT = 400;

export function createArDebugLog(options = {}) {
  const limit = Number(options.limit) > 0 ? Number(options.limit) : AR_DEBUG_LOG_LIMIT;
  const now = typeof options.now === "function" ? options.now : () => Date.now();
  const startedAt = now();
  const lines = [];
  let lastSignature = null;

  /** 서명이 바뀌었을 때만 기록한다. 기록했으면 true. */
  function record(signature, text) {
    if (signature === lastSignature) return false;
    lastSignature = signature;
    lines.push(`${timestamp(now() - startedAt)} ${text}`);
    if (lines.length > limit) lines.splice(0, lines.length - limit);
    return true;
  }

  return Object.freeze({
    record,
    get size() { return lines.length; },
    text: () => lines.join("\n"),
    entries: () => lines.slice(),
    clear() { lines.length = 0; lastSignature = null; },
  });
}

/**
 * 클립보드 복사. 비보안 컨텍스트(http)에서는 navigator.clipboard가 아예 없으므로
 * textarea+execCommand로 떨어진다. 둘 다 실패하면 false를 준다.
 */
export function copyToClipboard(target, text) {
  const documentRoot = target?.document;
  const viaSelection = () => {
    if (!documentRoot?.createElement) return false;
    try {
      const field = documentRoot.createElement("textarea");
      field.value = text;
      field.setAttribute("readonly", "");
      field.style.cssText = "position:fixed;top:0;left:0;width:1px;height:1px;opacity:0;";
      (documentRoot.body || documentRoot.documentElement).appendChild(field);
      field.select();
      field.setSelectionRange(0, text.length);
      const ok = documentRoot.execCommand?.("copy") === true;
      field.remove();
      return ok;
    } catch { return false; }
  };
  const write = target?.navigator?.clipboard?.writeText;
  if (typeof write !== "function") return Promise.resolve(viaSelection());
  return target.navigator.clipboard.writeText(text)
    .then(() => true)
    .catch(() => viaSelection());
}
