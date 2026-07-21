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
} from "../src/features/drive/contents/vision/ar/three_adapter.js";
import { signboardLayout } from "../src/features/drive/contents/vision/ar/three_signboard_layout.js";

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
  assert.equal(group.children.length, 3);
  assert.equal(group.getObjectByName("signboard:face").material.opacity, AR_OPACITY.group);
  assert.equal(group.getObjectByName("signboard:pole").geometry.parameters.height, bar.mountHeightM);

  assert.equal(disposeThreeSignboardGroup(group), true);
  assert.ok(tracked.textures.every((texture) => texture.disposed));
});

test("product mount lift extends the support without changing preview face geometry", () => {
  const bar = descriptor(AR_SHAPE.BAR);
  const preview = signboardLayout(bar);
  const product = signboardLayout(bar, { mountLiftM: 0.45 });

  assert.equal(product.face.widthM, preview.face.widthM);
  assert.equal(product.face.heightM, preview.face.heightM);
  assert.equal(product.face.yM, preview.face.yM + 0.45);
  assert.equal(product.pole.heightM, preview.pole.heightM + 0.45);
  assert.equal(product.pole.yM, product.pole.heightM / 2);
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

  disposeThreeSignboardGroup(group);
  assert.ok(tracked.textures.every((texture) => texture.disposed));
});

test("approved preview PIN uses a square head plane and dedicated stem", () => {
  const pin = descriptor(AR_SHAPE.PIN);
  const layout = signboardLayout(pin);
  const tracked = textureTracker();
  const group = createThreeSignboardGroup(pin, {
    textureFactory: tracked.factory,
    shadowTextureFactory: tracked.factory,
  });

  assert.equal(group.children.length, 3);
  assert.equal(group.getObjectByName("signboard:face").geometry.parameters.height, pin.widthM);
  assert.equal(
    group.getObjectByName("signboard:pin-stem").geometry.parameters.height,
    layout.pole.heightM,
  );

  disposeThreeSignboardGroup(group);
  assert.ok(tracked.textures.every((texture) => texture.disposed));
});
