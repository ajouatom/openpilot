import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { readFileSync } from "node:fs";
import test from "node:test";

import {
  AR_BADGE_M,
  AR_BAR_ASPECT,
  AR_BASE_M,
  AR_EMPHASIS,
  AR_FORM,
  AR_LEGIBILITY,
  AR_OPACITY,
  AR_PALETTE,
  AR_RATIO,
  AR_SCALE,
  AR_SHAPE,
  AR_SIGN_HEIGHT_M,
  AR_TEXTURE,
  AR_TOKEN_PREVIEW_CONTRACT,
  AR_TONE,
  AR_TONE_SHAPE,
  AR_TYPE,
} from "../src/features/drive/contents/vision/ar/design_tokens.js";
import { AR_MARKER_KIND } from "../src/features/drive/contents/vision/ar/tokens.js";
import {
  describeSignboard,
  signboardFromMarker,
} from "../src/features/drive/contents/vision/ar/signboard.js";
import {
  createThreeSignboardGroup,
  disposeThreeSignboardGroup,
} from "../src/features/drive/contents/vision/ar/three_adapter.js";
import { signboardLayout } from "../src/features/drive/contents/vision/ar/three_signboard_layout.js";

const U = Object.freeze({
  right: "\uc6b0\ud68c\uc804",
  left: "\uc88c\ud68c\uc804",
  straight: "\uc9c1\uc9c4",
  teheran: "\ud14c\ud5e4\ub780\ub85c",
  bangbae: "\ubc29\ubc30\ub3d9",
  gangnam: "\uac15\ub0a8\ub300\ub85c",
  camera: "\ub2e8\uc18d\uce74\uba54\ub77c",
  bump: "\uacfc\uc18d\ubc29\uc9c0\ud131",
  lane: "\uad8c\uc7a5 \ucc28\uc120",
  section: "\uad6c\uac04 \ud3c9\uade0 58",
  arrival: "\ub3c4\ucc29",
});

const APPROVED_SCENARIOS = Object.freeze([
  ["right-turn", { kind: AR_MARKER_KIND.TURN_GATE, distanceM: 80, turn: { direction: "right", sign: 1 }, label: U.teheran }],
  ["left-turn", { kind: AR_MARKER_KIND.TURN_GATE, distanceM: 197, turn: { direction: "left", sign: -1 }, label: U.bangbae }],
  ["straight", { kind: AR_MARKER_KIND.TURN_GATE, distanceM: 1200, turn: { direction: "straight", sign: 0 }, label: U.gangnam }],
  ["camera", { kind: AR_MARKER_KIND.CAUTION_SIGN, distanceM: 500, sdiFamily: "camera" }],
  ["bump", { kind: AR_MARKER_KIND.CAUTION_SIGN, distanceM: 120, sdiFamily: "bump" }],
  ["section", { kind: AR_MARKER_KIND.SECTION_GATE, distanceM: 1500, limitKph: 60, averageKph: 58 }],
  ["speed", { kind: AR_MARKER_KIND.SPEED_SIGN, speedLimitKph: 60 }],
  ["lane", { kind: AR_MARKER_KIND.LANE_BAND, distanceM: 160, laneWidthM: 3.5, laneOffsetM: 0 }],
  ["destination", { kind: AR_MARKER_KIND.DESTINATION_PIN, distanceM: 420 }],
]);

const SNAPSHOT_KEYS = Object.freeze([
  "kind", "tone", "shape", "phase", "primary", "secondary", "turnSign",
  "chevronCount", "distanceM", "widthM", "heightM", "mountHeightM",
  "depthM", "radiusM", "opacity", "scale", "surface",
]);

function canonicalize(value) {
  if (typeof value === "number") return +value.toFixed(6);
  if (Array.isArray(value)) return value.map(canonicalize);
  if (!value || typeof value !== "object") return value;
  return Object.fromEntries(
    Object.entries(value)
      .filter(([, item]) => item !== undefined && typeof item !== "function")
      .map(([key, item]) => [key, canonicalize(item)]),
  );
}

function descriptorSnapshot(descriptor) {
  return Object.fromEntries(
    SNAPSHOT_KEYS.filter((key) => descriptor[key] !== undefined)
      .map((key) => [key, descriptor[key]]),
  );
}

function componentSnapshot(shape) {
  const descriptor = describeSignboard({ shape, primary: "snapshot", phase: "precise" });
  const textureFactory = () => ({ dispose() {} });
  const group = createThreeSignboardGroup(descriptor, {
    textureFactory,
    shadowTextureFactory: textureFactory,
  });
  const snapshot = {
    layout: signboardLayout(descriptor),
    group: {
      kind: group.userData.kind,
      shape: group.userData.shape,
      children: group.children.map((child) => ({
        name: child.name,
        geometry: child.geometry.type,
        size: {
          width: child.geometry.parameters.width,
          height: child.geometry.parameters.height,
        },
        position: [child.position.x, child.position.y, child.position.z],
        rotation: [child.rotation.x, child.rotation.y, child.rotation.z],
        material: {
          transparent: child.material.transparent,
          depthWrite: child.material.depthWrite,
          opacity: child.material.opacity,
          side: child.material.side,
        },
      })),
    },
  };
  disposeThreeSignboardGroup(group);
  return snapshot;
}

function approvedPreviewSnapshot() {
  const shapes = [
    AR_SHAPE.BAR, AR_SHAPE.DIAMOND, AR_SHAPE.CIRCLE, AR_SHAPE.BAND, AR_SHAPE.PIN,
  ];
  return canonicalize({
    provenance: AR_TOKEN_PREVIEW_CONTRACT,
    tokens: {
      baseM: AR_BASE_M,
      ratio: AR_RATIO,
      scale: AR_SCALE,
      tone: AR_TONE,
      shape: AR_SHAPE,
      toneShape: AR_TONE_SHAPE,
      palette: AR_PALETTE,
      signHeightM: AR_SIGN_HEIGHT_M,
      barAspect: AR_BAR_ASPECT,
      badgeM: AR_BADGE_M,
      form: AR_FORM,
      texture: AR_TEXTURE,
      typography: AR_TYPE,
      opacity: AR_OPACITY,
      legibility: AR_LEGIBILITY,
      emphasis: AR_EMPHASIS,
    },
    scenarios: Object.fromEntries(APPROVED_SCENARIOS.map(([id, marker]) => [
      id,
      descriptorSnapshot(signboardFromMarker({ ...marker, phase: "precise" })),
    ])),
    components: Object.fromEntries(shapes.map((shape) => [shape, componentSnapshot(shape)])),
  });
}

const APPROVED_PREVIEW_GOLDEN = JSON.parse(readFileSync(
  new URL("./fixtures/ar_token_preview_snapshot.json", import.meta.url),
  "utf8",
));

test("approved self-contained preview identity and design token values stay fixed", () => {
  assert.deepEqual(AR_TOKEN_PREVIEW_CONTRACT, {
    file: "carrot_ar_token_preview.html",
    sha256: "f5395c5eabfeb6bfb55fb2a14e0e10665fee23076e3220187fac54abbe42336f",
    threeRevision: "185",
  });
  assert.deepEqual(AR_SCALE, {
    xxs: 0.2441, xs: 0.3906, sm: 0.625, md: 1,
    lg: 1.6, xl: 2.56, xxl: 4.096,
  });
  assert.deepEqual(AR_TONE_SHAPE, {
    guide: AR_SHAPE.BAR,
    caution: AR_SHAPE.DIAMOND,
    restrict: AR_SHAPE.CIRCLE,
    lane: AR_SHAPE.BAND,
    destination: AR_SHAPE.PIN,
  });
  assert.deepEqual(AR_PALETTE, {
    guide: {
      surface: "#0E7A46", surfaceTop: "#17A45E",
      ink: "#FFFFFF", inkMuted: "rgba(255,255,255,0.78)",
      edge: "#7DFFC0", accent: "#3BE38B",
    },
    caution: {
      surface: "#FFC53D", surfaceTop: "#FFD976",
      ink: "#1A1200", inkMuted: "rgba(26,18,0,0.72)",
      edge: "#FFE9A8", accent: "#FF8A00",
    },
    restrict: {
      surface: "#FFFFFF", surfaceTop: "#FFFFFF",
      ink: "#10141A", inkMuted: "rgba(16,20,26,0.68)",
      edge: "#FF5A5E", accent: "#E5262B",
    },
    lane: {
      surface: "#0B6E8C", surfaceTop: "#1394B5",
      ink: "#FFFFFF", inkMuted: "rgba(255,255,255,0.78)",
      edge: "#9BE8FF", accent: "#4DD2FF",
    },
    destination: {
      surface: "#8E2A6B", surfaceTop: "#B93C8D",
      ink: "#FFFFFF", inkMuted: "rgba(255,255,255,0.80)",
      edge: "#FFA8DC", accent: "#FF6BC4",
    },
  });
  assert.equal(AR_FORM[AR_SHAPE.BAR].widthM, 5.12);
  assert.equal(AR_FORM[AR_SHAPE.BAR].heightM, 2.56);
  assert.equal(AR_FORM[AR_SHAPE.BAR].radiusM, 0.3906);
  assert.equal(AR_FORM[AR_SHAPE.DIAMOND].widthM, 2.56);
  assert.equal(AR_FORM[AR_SHAPE.CIRCLE].diameterM, 2.56);
  assert.equal(AR_FORM[AR_SHAPE.BAND].lengthM, 8.192);
  assert.equal(AR_FORM[AR_SHAPE.BAND].liftM, 0.03);
  assert.equal(AR_FORM[AR_SHAPE.BAND].insetRatio, 0.72);
  assert.equal(AR_FORM[AR_SHAPE.PIN].heightM, 5.376);
  assert.equal(AR_FORM[AR_SHAPE.PIN].headDiameterM, 2.56);
  assert.deepEqual(AR_FORM.chevron, {
    countDefault: 3, widthRatio: 0.115, gapRatio: 0.028, insetRatio: 0.055,
  });
  assert.equal(AR_TEXTURE.barWidthPx, 1280);
  assert.equal(AR_TEXTURE.barHeightPx, 640);
  assert.equal(AR_TYPE.primaryPx, 218);
  assert.equal(AR_TYPE.secondaryPx, 85);
  const { held, ...previewOpacity } = AR_OPACITY;
  assert.deepEqual(previewOpacity, {
    surface: 0.86, surfaceTop: 0.9, edge: 0.92, ink: 0.98,
    inkMuted: 0.8, chevron: 0.88, pole: 0.72, shadow: 0.38,
    group: 0.94,
  });
  assert.equal(held, 0.72, "bounded-hold is a product adapter extension");
  assert.deepEqual(AR_EMPHASIS, {
    preview: { scale: 0.78, opacity: 0.62, chevrons: 0, showSecondary: false },
    approach: { scale: 0.9, opacity: 0.85, chevrons: 2, showSecondary: true },
    precise: { scale: 1, opacity: 1, chevrons: 3, showSecondary: true },
    commit: { scale: 1.12, opacity: 1, chevrons: 3, showSecondary: false },
  });
});

test("all nine approved preview scenarios resolve to the same content and silhouettes", () => {
  const scenarios = [
    [{ kind: AR_MARKER_KIND.TURN_GATE, distanceM: 80, turn: { direction: "right", sign: 1 }, label: U.teheran }, AR_TONE.GUIDE, AR_SHAPE.BAR, "80m", `${U.right} \u00b7 ${U.teheran}`],
    [{ kind: AR_MARKER_KIND.TURN_GATE, distanceM: 197, turn: { direction: "left", sign: -1 }, label: U.bangbae }, AR_TONE.GUIDE, AR_SHAPE.BAR, "200m", `${U.left} \u00b7 ${U.bangbae}`],
    [{ kind: AR_MARKER_KIND.TURN_GATE, distanceM: 1200, turn: { direction: "straight", sign: 0 }, label: U.gangnam }, AR_TONE.GUIDE, AR_SHAPE.BAR, "1.2km", `${U.straight} \u00b7 ${U.gangnam}`],
    [{ kind: AR_MARKER_KIND.CAUTION_SIGN, distanceM: 500, sdiFamily: "camera" }, AR_TONE.CAUTION, AR_SHAPE.DIAMOND, "500m", U.camera],
    [{ kind: AR_MARKER_KIND.CAUTION_SIGN, distanceM: 120, sdiFamily: "bump" }, AR_TONE.CAUTION, AR_SHAPE.DIAMOND, "120m", U.bump],
    [{ kind: AR_MARKER_KIND.SECTION_GATE, distanceM: 1500, limitKph: 60, averageKph: 58 }, AR_TONE.RESTRICT, AR_SHAPE.CIRCLE, "60", U.section],
    [{ kind: AR_MARKER_KIND.SPEED_SIGN, speedLimitKph: 60 }, AR_TONE.RESTRICT, AR_SHAPE.CIRCLE, "60", ""],
    [{ kind: AR_MARKER_KIND.LANE_BAND, distanceM: 160 }, AR_TONE.LANE, AR_SHAPE.BAND, "160m", U.lane],
    [{ kind: AR_MARKER_KIND.DESTINATION_PIN, distanceM: 420 }, AR_TONE.DESTINATION, AR_SHAPE.PIN, U.arrival, "420m"],
  ];

  for (const [marker, tone, shape, primary, secondary] of scenarios) {
    const descriptor = signboardFromMarker({ ...marker, phase: "precise" });
    assert.equal(descriptor.tone, tone);
    assert.equal(descriptor.shape, shape);
    assert.equal(descriptor.primary, primary);
    assert.equal(descriptor.secondary, secondary);
    assert.equal(descriptor.palette, AR_PALETTE[tone]);
  }
});

test("approved preview full token and Three component graph matches its golden snapshot", () => {
  const snapshot = approvedPreviewSnapshot();
  const digest = createHash("sha256").update(JSON.stringify(snapshot)).digest("hex");

  assert.deepEqual(snapshot.provenance, APPROVED_PREVIEW_GOLDEN.canonicalSource);
  assert.equal(
    digest,
    APPROVED_PREVIEW_GOLDEN.snapshotSha256,
    `approved preview snapshot changed: ${digest}`,
  );
});

test("approved lane scenario remains the exact BAND component in product metadata", () => {
  const lane = signboardFromMarker({
    kind: AR_MARKER_KIND.LANE_BAND,
    distanceM: 160,
    laneWidthM: 3.5,
    laneOffsetM: -0.25,
    phase: "precise",
  });
  assert.equal(lane.kind, AR_MARKER_KIND.LANE_BAND);
  assert.equal(lane.surface, true);
  assert.equal(lane.widthM, 3.5 * AR_FORM[AR_SHAPE.BAND].insetRatio);
  assert.equal(lane.heightM, AR_FORM[AR_SHAPE.BAND].lengthM);
  assert.equal(lane.laneOffsetM, -0.25);
});
