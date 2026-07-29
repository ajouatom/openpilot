"use strict";

const UPLOAD_ARTIFACT_KINDS = new Set(["qcamera", "rlog"]);

export function dashcamUploadArtifactKind(file) {
  const declared = String(file?.kind || "").trim().toLowerCase();
  if (UPLOAD_ARTIFACT_KINDS.has(declared)) return declared;
  const name = String(file?.name || "").trim().toLowerCase();
  if (name.startsWith("qcamera.")) return "qcamera";
  if (name === "rlog" || name.startsWith("rlog.")) return "rlog";
  return "";
}

export function dashcamUploadStats(items) {
  const list = Array.isArray(items) ? items : [];
  const segmentNames = [];
  const seenSegments = new Set();
  const stats = {
    segments: 0,
    segmentNames,
    files: 0,
    qcamera: 0,
    rlog: 0,
    bytes: 0,
  };

  list.forEach((item) => {
    const segment = String(item?.segment || "").trim();
    if (segment && !seenSegments.has(segment)) {
      seenSegments.add(segment);
      segmentNames.push(segment);
    }
    const files = Array.isArray(item?.files) ? item.files : [];
    const totalSize = Number(item?.totalSize)
      || files.reduce((sum, file) => sum + (Number(file?.size) || 0), 0);
    stats.segments += 1;
    stats.files += files.length;
    stats.bytes += totalSize;
    files.forEach((file) => {
      const kind = dashcamUploadArtifactKind(file);
      if (kind) stats[kind] += 1;
    });
  });

  return stats;
}
