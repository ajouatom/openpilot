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

function bandPitch(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return -Math.PI / 2;
  return Math.max(-Math.PI / 2, Math.min(0, number));
}

function stableTrig(value, fn) {
  const result = fn(value);
  return Math.abs(result) < 1e-12 ? 0 : result;
}

/**
 * Keep the BAND's near edge at one road anchor while its readable length is
 * scaled. This is the surface equivalent of keeping an upright sign's pole
 * base fixed: far-legibility may change geometry, never the world anchor.
 */
export function scaledBandFacePosition(face, scale = 1) {
  const lengthScale = Math.max(0.01, Number(scale) || 1);
  const rotationXRad = bandPitch(face?.rotationXRad);
  const cosPitch = stableTrig(rotationXRad, Math.cos);
  const sinPitch = stableTrig(rotationXRad, Math.sin);
  const heightM = finitePositive(face?.heightM, 1);
  const baseHalfLengthM = heightM / 2;
  const scaledHalfLengthM = heightM * lengthScale / 2;
  const roadLiftM = (Number(face?.yM) || 0) - cosPitch * baseHalfLengthM;
  const nearEdgeZM = (Number(face?.zM) || 0) - sinPitch * baseHalfLengthM;
  return Object.freeze({
    xM: Number(face?.xM) || 0,
    yM: roadLiftM + cosPitch * scaledHalfLengthM,
    zM: nearEdgeZM + sinPitch * scaledHalfLengthM,
    rotationXRad,
    cosPitch,
    sinPitch,
  });
}

export function signboardLayout(descriptor, options = {}) {
  if (!descriptor) throw new TypeError("signboard descriptor is required");

  const widthM = finitePositive(descriptor.widthM, 1);
  const heightM = finitePositive(descriptor.heightM, 1);

  if (descriptor.shape === AR_SHAPE.BAND) {
    // The immutable preview remains a road-flat -pi/2 plane. Product runtime
    // may provide a bounded viewing pitch so a distant lane cue does not
    // collapse into a one-pixel horizon line. The near edge stays on the same
    // road anchor for both forms.
    const productPitch = Number(options.surfacePitchRad);
    const hasProductPitch = Number.isFinite(productPitch);
    const rotationXRad = hasProductPitch ? bandPitch(productPitch) : -Math.PI / 2;
    if (!hasProductPitch) {
      return Object.freeze({
        coordinateSystem: "preview-y-up",
        face: Object.freeze({
          widthM,
          heightM,
          xM: 0,
          yM: AR_FORM[AR_SHAPE.BAND].liftM,
          zM: -heightM * 0.35,
          rotationXRad,
        }),
        shadow: null,
        pole: null,
      });
    }
    const cosPitch = stableTrig(rotationXRad, Math.cos);
    const sinPitch = stableTrig(rotationXRad, Math.sin);
    const nearEdgeZM = heightM * 0.15;
    return Object.freeze({
      coordinateSystem: "preview-y-up",
      face: Object.freeze({
        widthM,
        heightM,
        xM: 0,
        yM: AR_FORM[AR_SHAPE.BAND].liftM + cosPitch * heightM / 2,
        zM: nearEdgeZM + sinPitch * heightM / 2,
        rotationXRad,
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
  const requestedSupportHeightM = Number(options.supportHeightM);
  const supportHeightM = Number.isFinite(requestedSupportHeightM) && requestedSupportHeightM >= 0
    ? requestedSupportHeightM
    : baseSupportHeightM;

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
