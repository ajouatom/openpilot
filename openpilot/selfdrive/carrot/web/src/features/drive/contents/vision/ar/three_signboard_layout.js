/* Renderer-independent layout for the approved Three.js signboard preview.
 *
 * Keep all dimensions here in the preview's local coordinate system:
 * X = horizontal, Y = up, -Z = forward. The runtime scene adapter is
 * responsible for mapping that local group into vehicle coordinates.
 */

import {
  AR_FORM,
  AR_LEGIBILITY,
  AR_SCALE,
  AR_SHAPE,
} from "./design_tokens.js";

function finitePositive(value, fallback) {
  const number = Number(value);
  return Number.isFinite(number) && number > 0 ? number : fallback;
}

export function signboardLayout(descriptor, options = {}) {
  if (!descriptor) throw new TypeError("signboard descriptor is required");

  const widthM = finitePositive(descriptor.widthM, 1);
  const heightM = finitePositive(descriptor.heightM, 1);

  if (descriptor.shape === AR_SHAPE.BAND) {
    return Object.freeze({
      coordinateSystem: "preview-y-up",
      face: Object.freeze({
        widthM,
        heightM,
        xM: 0,
        yM: AR_FORM[AR_SHAPE.BAND].liftM,
        zM: -heightM * 0.35,
        rotationXRad: -Math.PI / 2,
      }),
      shadow: null,
      pole: null,
    });
  }

  const isPin = descriptor.shape === AR_SHAPE.PIN;
  const faceHeightM = isPin ? widthM : heightM;
  const baseSupportHeightM = isPin
    ? Math.max(0, heightM - faceHeightM / 2)
    : finitePositive(descriptor.mountHeightM, AR_FORM.mountHeightM);
  const mountLiftM = Math.max(0, Number(options.mountLiftM) || 0);
  const supportHeightM = baseSupportHeightM + mountLiftM;

  return Object.freeze({
    coordinateSystem: "preview-y-up",
    face: Object.freeze({
      widthM,
      heightM: faceHeightM,
      xM: 0,
      yM: isPin ? supportHeightM : supportHeightM + faceHeightM / 2,
      zM: 0,
      rotationXRad: 0,
    }),
    shadow: Object.freeze({
      widthM: widthM * AR_LEGIBILITY.shadowPlaneScale,
      heightM: widthM * AR_LEGIBILITY.shadowPlaneScale * 0.45,
      xM: 0,
      yM: 0.02,
      zM: 0,
      rotationXRad: -Math.PI / 2,
    }),
    pole: Object.freeze({
      widthM: AR_SCALE.xxs * (isPin ? 0.22 : 0.28),
      heightM: supportHeightM,
      xM: 0,
      yM: supportHeightM / 2,
      zM: 0,
      colorMultiplier: isPin ? 1 : 0.55,
    }),
  });
}
