import assert from "node:assert/strict";
import test from "node:test";

import {
  AR_FORM,
  AR_OPACITY,
  AR_SHAPE,
} from "../src/features/drive/contents/vision/ar/design_tokens.js";
import { describeSignboard } from "../src/features/drive/contents/vision/ar/signboard.js";
import {
  createThreeSignboardGroup,
  disposeThreeSignboardGroup,
  setThreeSignboardPresentationScale,
} from "../src/features/drive/contents/vision/ar/three_adapter.js";
import {
  scaledBandFacePosition,
  signboardLayout,
} from "../src/features/drive/contents/vision/ar/three_signboard_layout.js";
import {
  AR_MARKER_KIND,
  markerSupportHeightM,
} from "../src/features/drive/contents/vision/ar/tokens.js";

function descriptor(shape) {
  return describeSignboard({ shape, primary: "test", phase: "precise" });
}

function textureTracker() {
  const textures = [];
  const factory = () => {
    const texture = {
      disposed: false,
      dispose() { this.disposed = true; },
    };
    textures.push(texture);
    return texture;
  };
  return { textures, factory };
}

test("approved preview BAR keeps face, shadow and pole layout", () => {
  const bar = descriptor(AR_SHAPE.BAR);
  const layout = signboardLayout(bar);
  const tracked = textureTracker();
  const group = createThreeSignboardGroup(bar, {
    textureFactory: tracked.factory,
    shadowTextureFactory: tracked.factory,
  });

  assert.equal(layout.coordinateSystem, "preview-y-up");
  assert.equal(layout.face.yM, bar.mountHeightM + bar.heightM / 2);
  assert.equal(group.children.length, 4);
  assert.equal(group.getObjectByName("signboard:face").material.opacity, AR_OPACITY.group);
  assert.equal(group.getObjectByName("signboard:face-back").rotation.y, Math.PI);
  assert.equal(
    group.getObjectByName("signboard:face-back").material,
    group.getObjectByName("signboard:face").material,
  );
  assert.equal(group.getObjectByName("signboard:pole").geometry.parameters.height, bar.mountHeightM);
  assert.ok(Math.abs(
    (layout.face.yM - layout.face.heightM / 2)
    - (layout.pole.yM + layout.pole.heightM / 2)
  ) < 1e-12, "the face bottom meets the pole top and can never swap vertically");

  assert.equal(disposeThreeSignboardGroup(group), true);
  assert.ok(tracked.textures.every((texture) => texture.disposed));
});

test("product support height is absolute and leaves approved face geometry unchanged", () => {
  const bar = descriptor(AR_SHAPE.BAR);
  const preview = signboardLayout(bar);
  const product = signboardLayout(bar, { supportHeightM: 1.28 });

  assert.equal(product.face.widthM, preview.face.widthM);
  assert.equal(product.face.heightM, preview.face.heightM);
  assert.equal(product.face.yM, 1.28 + product.face.heightM / 2);
  assert.equal(product.pole.heightM, 1.28);
  assert.equal(product.pole.yM, product.pole.heightM / 2);
});

test("product upright support is lower than the immutable preview mount", () => {
  const bar = descriptor(AR_SHAPE.BAR);
  const preview = signboardLayout(bar);
  const productSupport = markerSupportHeightM({ ...bar, kind: AR_MARKER_KIND.TURN_GATE });
  const product = signboardLayout(bar, { supportHeightM: productSupport });

  assert.equal(preview.pole.heightM, bar.mountHeightM);
  assert.equal(productSupport, 1.05);
  assert.ok(product.pole.heightM < preview.pole.heightM);
  assert.equal(product.face.widthM, preview.face.widthM);
  assert.equal(product.face.heightM, preview.face.heightM);
});

test("product presentation scale changes the face around a fixed support and anchor base", () => {
  const bar = descriptor(AR_SHAPE.BAR);
  const supportHeightM = markerSupportHeightM({ ...bar, kind: AR_MARKER_KIND.TURN_GATE });
  const tracked = textureTracker();
  const group = createThreeSignboardGroup(bar, {
    textureFactory: tracked.factory,
    shadowTextureFactory: tracked.factory,
    supportHeightM,
  });

  assert.equal(setThreeSignboardPresentationScale(group, 0.5), true);
  const face = group.getObjectByName("signboard:face");
  const pole = group.getObjectByName("signboard:pole");
  assert.equal(face.scale.x, 0.5);
  assert.equal(face.scale.y, 0.5);
  assert.equal(pole.scale.y, 1);
  assert.equal(pole.geometry.parameters.height, supportHeightM);
  assert.ok(Math.abs(
    (face.position.y - face.geometry.parameters.height * face.scale.y / 2)
    - (pole.position.y + pole.geometry.parameters.height / 2)
  ) < 1e-12);

  disposeThreeSignboardGroup(group);
});

test("approved preview BAND lies on the road without standing supports", () => {
  const band = descriptor(AR_SHAPE.BAND);
  const tracked = textureTracker();
  const group = createThreeSignboardGroup(band, { textureFactory: tracked.factory });

  assert.equal(group.children.length, 1);
  const face = group.getObjectByName("signboard:face");
  assert.equal(face.geometry.parameters.width, band.widthM);
  assert.equal(face.geometry.parameters.height, band.heightM);
  assert.equal(face.position.y, AR_FORM[AR_SHAPE.BAND].liftM);
  assert.equal(face.position.z, -band.heightM * 0.35);
  assert.equal(face.rotation.x, -Math.PI / 2);
  assert.equal(group.getObjectByName("signboard:shadow"), undefined);
  assert.equal(group.getObjectByName("signboard:pole"), undefined);
  assert.equal(group.getObjectByName("signboard:face-back"), undefined);

  disposeThreeSignboardGroup(group);
  assert.ok(tracked.textures.every((texture) => texture.disposed));
});

test("product BAND tilts toward the driver while its near edge stays road-anchored", () => {
  const band = descriptor(AR_SHAPE.BAND);
  const pitch = -Math.PI / 4;
  const preview = signboardLayout(band);
  const product = signboardLayout(band, { surfacePitchRad: pitch });
  const tracked = textureTracker();
  const group = createThreeSignboardGroup(band, {
    textureFactory: tracked.factory,
    surfacePitchRad: pitch,
  });
  const face = group.getObjectByName("signboard:face");

  assert.equal(preview.face.rotationXRad, -Math.PI / 2);
  assert.equal(product.face.rotationXRad, pitch);
  assert.equal(face.rotation.x, pitch);
  assert.ok(product.face.yM > AR_FORM[AR_SHAPE.BAND].liftM);
  assert.ok(Math.abs(
    product.face.yM - Math.cos(pitch) * product.face.heightM / 2
      - AR_FORM[AR_SHAPE.BAND].liftM
  ) < 1e-12);
  const baseFace = scaledBandFacePosition(product.face, 1);
  const extendedFace = scaledBandFacePosition(product.face, 5);
  const nearEdge = (face, scale) => ({
    y: face.yM - face.cosPitch * product.face.heightM * scale / 2,
    z: face.zM - face.sinPitch * product.face.heightM * scale / 2,
  });
  const baseNear = nearEdge(baseFace, 1);
  const extendedNear = nearEdge(extendedFace, 5);
  assert.ok(Math.abs(baseNear.y - extendedNear.y) < 1e-12);
  assert.ok(Math.abs(baseNear.z - extendedNear.z) < 1e-12);

  disposeThreeSignboardGroup(group);
});

test("approved preview PIN uses a square head plane and dedicated stem", () => {
  const pin = descriptor(AR_SHAPE.PIN);
  const layout = signboardLayout(pin);
  const tracked = textureTracker();
  const group = createThreeSignboardGroup(pin, {
    textureFactory: tracked.factory,
    shadowTextureFactory: tracked.factory,
  });

  assert.equal(group.children.length, 4);
  assert.equal(group.getObjectByName("signboard:face").geometry.parameters.height, pin.widthM);
  assert.equal(
    group.getObjectByName("signboard:pin-stem").geometry.parameters.height,
    layout.pole.heightM,
  );

  disposeThreeSignboardGroup(group);
  assert.ok(tracked.textures.every((texture) => texture.disposed));
});

test("destination product placement keeps one anchor-tip support across phases", () => {
  const preview = describeSignboard({ shape: AR_SHAPE.PIN, primary: "test", phase: "preview" });
  const commit = describeSignboard({ shape: AR_SHAPE.PIN, primary: "test", phase: "commit" });
  const previewHeight = markerSupportHeightM({
    ...preview,
    kind: AR_MARKER_KIND.DESTINATION_PIN,
  });
  const commitHeight = markerSupportHeightM({
    ...commit,
    kind: AR_MARKER_KIND.DESTINATION_PIN,
  });

  assert.ok(Math.abs(previewHeight - commitHeight) < 1e-12);
  assert.equal(signboardLayout(preview, { supportHeightM: previewHeight }).pole.heightM, previewHeight);
  assert.equal(signboardLayout(commit, { supportHeightM: commitHeight }).pole.heightM, commitHeight);
});
