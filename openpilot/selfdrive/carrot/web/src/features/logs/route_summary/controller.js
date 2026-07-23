import { fetchRouteSummarySource } from "./api.js";
import { getCachedRouteSummary, setCachedRouteSummary } from "./cache.js";
import { createSummaryState } from "./components.js";
import { createRouteSummaryViewModel } from "./model.js";
import { mountRouteSummary } from "./view.js";
import { analyzeRouteSummary } from "./worker_client.js";

let activeOperation = null;
let operationSerial = 0;

function text(key, fallback, params) {
  return typeof getUIText === "function" ? getUIText(key, fallback, params) : fallback;
}

function mountId(serial, kind) {
  return `routeSummary${kind}${serial}`;
}

function staticMountMarkup(id) {
  return `<div id="${id}"></div>`;
}

function errorMessage(error) {
  if (error?.code === "summary-worker-unsupported") {
    return text("summary_worker_unsupported", "이 브라우저에서는 주행요약 분석을 지원하지 않습니다.");
  }
  if (error?.code === "summary-source-empty") {
    return text("report_no_data", "분석할 주행 데이터가 없습니다.");
  }
  return text("report_failed", "주행요약을 불러오지 못했습니다.");
}

export async function openRouteSummary(route) {
  if (!route) return;
  activeOperation?.abort();
  const serial = ++operationSerial;
  const controller = new AbortController();
  activeOperation = controller;
  let phase = "loading";
  const loadingId = mountId(serial, "Loading");
  const loadingDialog = openAppDialog({
    mode: "choice",
    html: true,
    title: text("route_summary", "주행요약"),
    messageHtml: staticMountMarkup(loadingId),
    cancelLabel: text("cancel", "취소"),
  });
  const state = createSummaryState(text("report_analyzing", "주행 데이터를 분석하는 중…"));
  document.getElementById(loadingId)?.append(state.element);
  loadingDialog.then(() => {
    if (phase === "loading" && activeOperation === controller) controller.abort();
  });

  try {
    const source = await fetchRouteSummarySource(route, { signal: controller.signal });
    let result = getCachedRouteSummary(source);
    if (result) {
      state.update(1, text("summary_cached", "저장된 주행요약을 불러오는 중…"));
    } else {
      result = await analyzeRouteSummary(source, {
        signal: controller.signal,
        onProgress(progress) {
          const total = Math.max(0, Number(progress.total) || 0);
          const completed = Math.max(0, Number(progress.completed) || 0);
          const totalBytes = Math.max(0, Number(progress.totalBytes) || 0);
          const loadedBytes = Math.max(0, Number(progress.loadedBytes) || 0);
          const ratio = totalBytes > 0 ? loadedBytes / totalBytes : (total > 0 ? completed / total : NaN);
          const percent = Number.isFinite(ratio) ? Math.round(Math.max(0, Math.min(1, ratio)) * 100) : 0;
          const description = totalBytes > 0
            ? text("summary_progress_percent", "세그먼트 {completed}/{total} · {percent}%", { completed, total, percent })
            : text("summary_progress", "세그먼트 {completed}/{total}", { completed, total });
          state.update(ratio, description);
        },
      });
      setCachedRouteSummary(source, result);
    }
    if (controller.signal.aborted || activeOperation !== controller) return;
    const model = createRouteSummaryViewModel(result);
    const contentId = mountId(serial, "Content");
    phase = "result";
    openAppDialog({
      mode: "alert",
      html: true,
      title: text("route_summary", "주행요약"),
      messageHtml: staticMountMarkup(contentId),
      confirmLabel: text("close", "닫기"),
    });
    const root = document.getElementById(contentId);
    if (root) mountRouteSummary(root, model);
  } catch (error) {
    if (error?.name === "AbortError" || controller.signal.aborted) return;
    phase = "error";
    openAppDialog({
      mode: "alert",
      title: text("route_summary", "주행요약"),
      message: errorMessage(error),
      confirmLabel: text("close", "닫기"),
    });
  } finally {
    if (activeOperation === controller) activeOperation = null;
  }
}
