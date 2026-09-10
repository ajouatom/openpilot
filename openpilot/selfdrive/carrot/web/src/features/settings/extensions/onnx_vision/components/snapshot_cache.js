const CACHE_NAME = "carrot-onnx-vision-snapshot-v1";
const CACHE_PATH = "/__carrot_cache__/onnx-wide-snapshot";

function isJpeg(blob) {
  return Boolean(blob && Number(blob.size) > 0 && String(blob.type || "").toLowerCase().startsWith("image/jpeg"));
}

function cacheKey(target) {
  const origin = String(target?.location?.origin || "").replace(/\/$/, "");
  return origin ? `${origin}${CACHE_PATH}` : CACHE_PATH;
}

export function createOnnxSnapshotCache() {
  let memorySnapshot = null;

  return Object.freeze({
    async load(target = globalThis) {
      if (isJpeg(memorySnapshot)) return memorySnapshot;
      if (typeof target?.caches?.open !== "function") return null;
      try {
        const cache = await target.caches.open(CACHE_NAME);
        const response = await cache.match(cacheKey(target));
        if (!response?.ok) return null;
        const blob = await response.blob();
        if (!isJpeg(blob)) return null;
        memorySnapshot = blob;
        return blob;
      } catch {
        return null;
      }
    },

    async store(blob, target = globalThis) {
      if (!isJpeg(blob)) return false;
      memorySnapshot = blob;
      if (typeof target?.caches?.open !== "function") return true;
      try {
        const cache = await target.caches.open(CACHE_NAME);
        const ResponseClass = target.Response || globalThis.Response;
        if (typeof ResponseClass !== "function") return true;
        await cache.put(cacheKey(target), new ResponseClass(blob, {
          status: 200,
          headers: { "Content-Type": "image/jpeg" },
        }));
      } catch {
        // The in-memory copy still keeps the latest frame for this Web session.
      }
      return true;
    },
  });
}

export const onnxSnapshotCache = createOnnxSnapshotCache();
