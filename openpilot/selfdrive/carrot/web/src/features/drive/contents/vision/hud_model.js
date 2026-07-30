import { resolveLaneModeState } from "./lane_mode.js";

function finiteNumber(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function optionalNumber(value) {
  if (value == null || value === "") return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function formatRtcNumber(value, suffix = "", digits = 0) {
  return value == null ? `-${suffix}` : `${value.toFixed(digits)}${suffix}`;
}

function getRtcCodecLabel(codec, codecParams) {
  const rawCodec = String(codec || "").split("/").pop().trim().toUpperCase();
  if (!rawCodec) return "-";
  if (rawCodec !== "H264") return rawCodec;
  const match = /profile-level-id=([0-9a-fA-F]{6})/.exec(String(codecParams || ""));
  const profilePrefix = match?.[1]?.slice(0, 2)?.toLowerCase() || "";
  const profile = profilePrefix === "42"
    ? "Baseline"
    : profilePrefix === "4d"
      ? "Main"
      : profilePrefix === "64"
        ? "High"
        : "";
  return profile ? `${rawCodec} ${profile}` : rawCodec;
}

function isLongActive(overlayState) {
  return Boolean(overlayState?.carControl?.longActive);
}

function isLaneMode(hudState) {
  return resolveLaneModeState(hudState).pathActive;
}

function buildRtcPerf(perf) {
  if (!perf?.active) {
    return { visible: false, tone: "offline", title: "VISION OFFLINE", glance: "" };
  }

  const network = perf.network || {};
  const inbound = perf.inbound || {};
  const video = perf.video || {};
  const resolution = String(network.resolutionLabel || "").trim() || "-";
  const bitrateMbps = optionalNumber(network.bitrateMbps);
  const fps = optionalNumber(inbound.framesPerSecond);
  const rttMs = optionalNumber(network.rttMs);
  const lossPct = optionalNumber(network.lossPct);
  const jitterMs = optionalNumber(network.jitterMs);
  const freezeCount = optionalNumber(inbound.freezeCount);
  const framesDecoded = optionalNumber(inbound.framesDecoded);
  const framesReceived = optionalNumber(inbound.framesReceived);
  const readyState = optionalNumber(video.readyState);
  const reconnecting = (
    perf.connectionState === "connecting"
    || perf.iceConnectionState === "checking"
    || perf.iceConnectionState === "disconnected"
  );
  const stalled = (
    (framesReceived != null && framesReceived > 0 && framesDecoded != null && framesDecoded <= 0)
    || (freezeCount != null && freezeCount > 0 && readyState != null && readyState < 3)
  );
  const degraded = (
    (lossPct != null && lossPct >= 0.5)
    || (jitterMs != null && jitterMs >= 30)
    || (rttMs != null && rttMs >= 150)
  );
  const bitrateLabel = formatRtcNumber(bitrateMbps, "Mbps", bitrateMbps != null && bitrateMbps < 10 ? 2 : 1);
  const fpsLabel = fps == null ? "-fps" : `${Math.round(fps)}fps`;
  const rttLabel = rttMs == null ? "-ms" : `${Math.round(rttMs)}ms`;
  const jitterLabel = jitterMs == null ? "-ms" : `${Math.round(jitterMs)}ms`;
  const lossLabel = lossPct == null ? "-%" : `${lossPct.toFixed(lossPct >= 10 ? 0 : 1)}%`;
  const codecLabel = getRtcCodecLabel(perf.codec, perf.codecParams);
  const protocol = String(network.protocol || "-").toUpperCase();
  const localType = String(network.localCandidateType || "-");
  const remoteType = String(network.remoteCandidateType || "-");
  let tone = "normal";
  let title = "VISION OK";
  let glance = `${resolution} | ${fpsLabel} | ${bitrateLabel} | ${rttLabel}`;

  if (perf.error) {
    tone = "offline";
    title = "VISION OFFLINE";
    glance = "OFFLINE | stats unavailable";
  } else if (reconnecting) {
    tone = "reconnecting";
    title = "VISION RECONNECTING";
    glance = "RECONNECTING | waiting stream";
  } else if (stalled) {
    tone = "stall";
    title = "VISION STALL";
    glance = `STALL | decode ${framesDecoded ?? "-"} | recv ${framesReceived ?? "-"}`;
  } else if (degraded) {
    tone = "degraded";
    title = "VISION DEGRADED";
    const warnings = [];
    if (lossPct != null && lossPct >= 0.5) warnings.push(`loss ${lossLabel}`);
    if (rttMs != null && rttMs >= 150) warnings.push(`RTT ${rttLabel}`);
    if (jitterMs != null && jitterMs >= 30) warnings.push(`jitter ${jitterLabel}`);
    glance = `DEGRADED | ${warnings.slice(0, 2).join(" | ")}`;
  }

  return {
    visible: true,
    tone,
    title,
    glance,
    video: `${resolution} | ${fpsLabel}`,
    codec: codecLabel,
    bitrate: bitrateLabel,
    rtt: rttLabel,
    jitter: jitterLabel,
    loss: lossLabel,
    freeze: freezeCount == null ? "-" : String(Math.round(freezeCount)),
    path: `${protocol} ${localType} -> ${remoteType}`,
  };
}

function formatRtcPerfLabel(perf) {
  return buildRtcPerf(perf).glance || "";
}

export const DriveVisionHudModel = Object.freeze({
  isLongActive,
  isLaneMode,
  resolveLaneModeState,
  buildRtcPerf,
  formatRtcPerfLabel,
});

export {
  isLongActive,
  isLaneMode,
  resolveLaneModeState,
  buildRtcPerf,
  formatRtcPerfLabel,
};

export function installDriveVisionHudModelFacade(target = globalThis) {
  target.DriveVisionHudModel = DriveVisionHudModel;
  return DriveVisionHudModel;
}
