"use strict";

window.CarrotReplayClient = (() => {
  const LOG_WORKER_URL = "/js/realtime/replay_log_worker.js?v=2607-12";
  const VIDEO_WORKER_URL = "/js/realtime/replay_video_worker.js?v=2607-02";
  const MSE_BASELINE = 'video/mp4; codecs="avc1.42e01e, mp4a.40.2"';
  const MSE_TYPES = [
    'video/mp4; codecs="avc1.42c015, mp4a.40.2"',
    MSE_BASELINE,
    'video/mp4; codecs="avc1.4d401f, mp4a.40.2"',
    'video/mp4; codecs="avc1.64001f, mp4a.40.2"',
  ];

  function capabilities() {
    const primitives = typeof Promise === "function"
      && typeof fetch === "function"
      && typeof Worker === "function"
      && typeof ArrayBuffer === "function"
      && typeof Uint8Array === "function"
      && typeof TextDecoder === "function"
      && typeof Blob === "function"
      && typeof URL !== "undefined"
      && typeof URL.createObjectURL === "function";
    const mediaSourceCandidates = [window.MediaSource, window.ManagedMediaSource]
      .filter((candidate, index, values) => candidate && values.indexOf(candidate) === index);
    let MediaSourceCtor = null;
    let mse = false;
    if (primitives) {
      for (const candidate of mediaSourceCandidates) {
        if (typeof candidate.isTypeSupported !== "function") continue;
        try {
          if (MSE_TYPES.some((type) => candidate.isTypeSupported(type))) {
            MediaSourceCtor = candidate;
            mse = true;
            break;
          }
        } catch {}
      }
    }
    return {
      client: primitives,
      streamFetch: typeof ReadableStream === "function",
      mse,
      MediaSourceCtor,
      mode: !primitives ? "unsupported" : (mse ? "client-stream" : "client-buffered"),
    };
  }

  async function fetchSource(segment, signal) {
    const response = await fetch(`/api/dashcam/replay-source/${encodeURIComponent(segment)}`, {
      cache: "no-store",
      signal,
    });
    let payload = null;
    try { payload = await response.json(); } catch {}
    if (!response.ok || payload?.ok === false) {
      throw new Error(String(payload?.error || `${response.status} ${response.statusText}`));
    }
    return payload;
  }

  function logTimeline(source, signal, session) {
    return new Promise((resolve, reject) => {
      const worker = new Worker(LOG_WORKER_URL);
      session.workers.add(worker);
      let settled = false;
      const onAbort = () => finish(reject, new DOMException("Aborted", "AbortError"));
      const finish = (callback, value) => {
        if (settled) return;
        settled = true;
        signal?.removeEventListener?.("abort", onAbort);
        session.workers.delete(worker);
        worker.terminate();
        callback(value);
      };
      worker.onmessage = (event) => {
        const payload = event?.data || {};
        if (payload.type === "complete") {
          finish(resolve, {
            durationMs: Number(payload.durationMs) || 0,
            records: Array.isArray(payload.records) ? payload.records : [],
            metadata: payload.metadata && typeof payload.metadata === "object" ? payload.metadata : {},
          });
        } else if (payload.type === "error") {
          finish(reject, new Error(String(payload.error || "Client replay log processing failed")));
        }
      };
      worker.onerror = (event) => finish(reject, new Error(event?.message || "Client replay log worker failed"));
      if (signal?.aborted) {
        finish(reject, new DOMException("Aborted", "AbortError"));
        return;
      }
      signal?.addEventListener?.("abort", onAbort, { once: true });
      worker.postMessage({
        type: "start",
        url: source.rlog.url,
        compression: source.rlog.compression,
        expectedSegment: source.segmentIndex,
      });
    });
  }

  function createMsePreviewSource(chunks, mime, durationSeconds, profile, session) {
    if (!profile.MediaSourceCtor || !chunks.length) return null;
    const mediaSource = new profile.MediaSourceCtor();
    const url = URL.createObjectURL(mediaSource);
    session.objectUrls.add(url);
    const ready = new Promise((resolve, reject) => {
      let settled = false;
      const finish = (error = null) => {
        if (settled) return;
        settled = true;
        if (error) reject(error);
        else resolve();
      };
      mediaSource.addEventListener("sourceopen", () => {
        let sourceBuffer = null;
        let index = 0;
        const appendNext = () => {
          if (sourceBuffer?.updating) return;
          if (index < chunks.length) {
            try { sourceBuffer.appendBuffer(chunks[index++]); } catch (error) { finish(error); }
            return;
          }
          try {
            let duration = Number(durationSeconds);
            const buffered = sourceBuffer?.buffered;
            if (buffered?.length) duration = Math.max(duration, Number(buffered.end(buffered.length - 1)) || 0);
            if (Number.isFinite(duration) && duration > 0) mediaSource.duration = duration;
          } catch {}
          try {
            if (mediaSource.readyState === "open") mediaSource.endOfStream();
            finish();
          } catch (error) { finish(error); }
        };
        try {
          sourceBuffer = mediaSource.addSourceBuffer(mime || MSE_BASELINE);
          sourceBuffer.addEventListener("updateend", appendNext);
          sourceBuffer.addEventListener("error", () => finish(new Error("Preview video buffer failed")));
          appendNext();
        } catch (error) {
          finish(error);
        }
      }, { once: true });
      mediaSource.addEventListener("sourceclose", () => {
        if (!settled) finish(new Error("Preview video source closed"));
      });
    });
    return { url, ready };
  }

  function videoFromMp4(source) {
    return {
      urlPromise: Promise.resolve(source.video.url),
      createPreviewSource: async () => ({ url: source.video.url, ready: Promise.resolve() }),
      complete: Promise.resolve(),
      streaming: false,
    };
  }

  function videoFromMpegTs(source, videoEl, profile, signal, session) {
    let resolveUrl;
    let rejectUrl;
    let resolveComplete;
    let rejectComplete;
    let resolvePreviewData;
    const urlPromise = new Promise((resolve, reject) => { resolveUrl = resolve; rejectUrl = reject; });
    const complete = new Promise((resolve, reject) => { resolveComplete = resolve; rejectComplete = reject; });
    const previewDataPromise = new Promise((resolve) => { resolvePreviewData = resolve; });
    const worker = new Worker(VIDEO_WORKER_URL);
    session.workers.add(worker);
    const packagedChunks = [];
    const appendQueue = [];
    let mediaSource = null;
    let sourceBuffer = null;
    let mediaOpen = false;
    let workerComplete = false;
    let mime = "video/mp4";
    let settled = false;
    let previewSettled = false;
    let useMse = null;

    const finishPreview = (data = null) => {
      if (previewSettled) return;
      previewSettled = true;
      resolvePreviewData(data);
    };

    const fail = (error) => {
      if (settled) return;
      settled = true;
      session.workers.delete(worker);
      worker.terminate();
      const failure = error instanceof Error ? error : new Error(String(error || "Client video packaging failed"));
      finishPreview();
      rejectUrl(failure);
      rejectComplete(failure);
    };

    const maybeEndStream = () => {
      if (!useMse || !workerComplete || appendQueue.length || sourceBuffer?.updating) return;
      try {
        if (mediaSource?.readyState === "open") mediaSource.endOfStream();
      } catch {}
      if (!settled) {
        settled = true;
        session.workers.delete(worker);
        worker.terminate();
        resolveComplete();
      }
    };

    const pump = () => {
      if (!useMse || !mediaOpen || sourceBuffer?.updating || !appendQueue.length) {
        maybeEndStream();
        return;
      }
      if (!sourceBuffer) {
        const detectedMime = mime || MSE_BASELINE;
        try {
          if (!profile.MediaSourceCtor.isTypeSupported(detectedMime)) {
            throw new Error(`Browser does not support ${detectedMime}`);
          }
          sourceBuffer = mediaSource.addSourceBuffer(detectedMime);
          sourceBuffer.addEventListener("updateend", pump);
          sourceBuffer.addEventListener("error", () => fail(new Error("Browser rejected packaged replay video")));
        } catch (error) {
          fail(error);
          return;
        }
      }
      const next = appendQueue.shift();
      try { sourceBuffer.appendBuffer(next); } catch (error) { fail(error); }
    };

    const startMediaSource = () => {
      let objectUrl = "";
      try {
        mediaSource = new profile.MediaSourceCtor();
        videoEl.disableRemotePlayback = true;
        objectUrl = URL.createObjectURL(mediaSource);
        session.objectUrls.add(objectUrl);
        mediaSource.addEventListener("sourceopen", () => {
          mediaOpen = true;
          pump();
        }, { once: true });
        resolveUrl(objectUrl);
        return true;
      } catch {
        if (objectUrl) {
          URL.revokeObjectURL(objectUrl);
          session.objectUrls.delete(objectUrl);
        }
        mediaSource = null;
        mediaOpen = false;
        useMse = false;
        return false;
      }
    };

    worker.onmessage = (event) => {
      const payload = event?.data || {};
      if (payload.type === "chunk") {
        mime = String(payload.mime || mime);
        const chunk = payload.data;
        if (!(chunk instanceof ArrayBuffer)) return;
        packagedChunks.push(chunk);
        if (useMse == null) {
          try {
            useMse = Boolean(
              profile.mse
              && profile.MediaSourceCtor?.isTypeSupported?.(mime)
              && startMediaSource(),
            );
          } catch {
            useMse = false;
          }
        }
        if (useMse) {
          appendQueue.push(chunk);
          pump();
        }
      } else if (payload.type === "complete") {
        workerComplete = true;
        if (useMse) {
          finishPreview({ chunks: [...packagedChunks], mime });
          maybeEndStream();
        } else {
          try {
            const objectUrl = URL.createObjectURL(new Blob(packagedChunks, { type: "video/mp4" }));
            session.objectUrls.add(objectUrl);
            resolveUrl(objectUrl);
            finishPreview({ url: objectUrl });
            if (!settled) {
              settled = true;
              session.workers.delete(worker);
              worker.terminate();
              resolveComplete();
            }
          } catch (error) {
            fail(error);
          }
        }
      } else if (payload.type === "error") {
        fail(new Error(String(payload.error || "Client video packaging failed")));
      }
    };
    worker.onerror = (event) => fail(new Error(event?.message || "Client replay video worker failed"));
    if (signal?.aborted) {
      fail(new DOMException("Aborted", "AbortError"));
    } else {
      signal?.addEventListener?.("abort", () => fail(new DOMException("Aborted", "AbortError")), { once: true });
      worker.postMessage({ type: "start", url: source.video.url });
    }
    const createPreviewSource = async (durationSeconds) => {
      const data = await previewDataPromise;
      if (data?.url) return { url: data.url, ready: Promise.resolve() };
      if (!data?.chunks?.length) return null;
      return createMsePreviewSource(data.chunks, data.mime, durationSeconds, profile, session);
    };
    return { urlPromise, createPreviewSource, complete, streaming: profile.mse };
  }

  function start(source, videoEl, signal) {
    const profile = capabilities();
    if (!profile.client) throw new Error("client-replay-unsupported");
    const session = {
      workers: new Set(),
      objectUrls: new Set(),
      disposed: false,
      dispose() {
        if (this.disposed) return;
        this.disposed = true;
        for (const worker of this.workers) {
          try { worker.postMessage({ type: "abort" }); } catch {}
          worker.terminate();
        }
        this.workers.clear();
        for (const url of this.objectUrls) URL.revokeObjectURL(url);
        this.objectUrls.clear();
      },
    };
    if (signal) signal.addEventListener("abort", () => session.dispose(), { once: true });
    const container = String(source?.video?.container || "");
    if (!source?.rlog?.url || !source?.video?.url || !["mp4", "mpegts"].includes(container)) {
      session.dispose();
      throw new Error("unsupported-client-replay-source");
    }
    const timelinePromise = logTimeline(source, signal, session);
    let video;
    try {
      video = container === "mp4"
        ? videoFromMp4(source)
        : videoFromMpegTs(source, videoEl, profile, signal, session);
    } catch (error) {
      session.dispose();
      throw error;
    }
    return { profile, session, timelinePromise, video };
  }

  return { capabilities, fetchSource, start };
})();
