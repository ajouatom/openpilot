/* Immutable world anchors and their per-frame camera-relative projection.
 *
 * A marker is accepted once in route-local FLU, converted through Device FRD
 * into the current local world, and never moved again. Rendering performs the
 * inverse operation from the latest T_world_device. This is the central
 * distinction between a world marker and a HUD element that follows the car.
 */

import {
  AR_COORDINATE_FRAME,
  deviceFrdVectorToRouteFlu,
  routeFluVectorToDeviceFrd,
} from "./coordinate_frames.js";
import {
  faceFrameFields,
  faceFrameForAnchor,
  roadFrameFields,
  roadFrameForAnchor,
} from "./road_frame.js";
import {
  devicePointToWorld,
  normalizeQuaternion,
  rotateVectorByQuaternion,
  worldPointToDevice,
} from "./world_pose.js";

function finite(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function freezeVector(value) {
  return Object.freeze([...value]);
}

function inverseRotate(pose, vector) {
  const orientation = normalizeQuaternion(pose?.orientation);
  if (!orientation) return null;
  return rotateVectorByQuaternion(
    [orientation[0], -orientation[1], -orientation[2], -orientation[3]],
    vector,
  );
}

export function isUsableWorldPose(pose) {
  return pose?.initialized === true
    && pose.worldCoordinateFrame === AR_COORDINATE_FRAME.LOCAL_WORLD_DEVICE
    && pose.deviceCoordinateFrame === AR_COORDINATE_FRAME.DEVICE_FRD
    && Array.isArray(pose.position)
    && Array.isArray(pose.orientation)
    && Boolean(normalizeQuaternion(pose.orientation));
}

/** Capture route-relative marker position and orientation in fixed world axes. */
export function createImmutableWorldAnchor(anchor, pose) {
  if (!anchor || !isUsableWorldPose(pose)) return null;
  const routePosition = [finite(anchor.x), finite(anchor.y), finite(anchor.z)];
  if (routePosition.some((value) => value === null)) return null;

  const devicePosition = routeFluVectorToDeviceFrd(routePosition);
  const worldPosition = devicePointToWorld(pose, devicePosition);
  const roadFrame = roadFrameForAnchor(anchor);
  const faceFrame = faceFrameForAnchor(anchor);
  const worldRoadForward = rotateVectorByQuaternion(
    pose.orientation,
    routeFluVectorToDeviceFrd(roadFrame.forward),
  );
  const worldRoadUp = rotateVectorByQuaternion(
    pose.orientation,
    routeFluVectorToDeviceFrd(roadFrame.up),
  );
  const hasExplicitFaceFrame = Array.isArray(anchor.faceForward) || Array.isArray(anchor.faceUp);
  const worldFaceForward = hasExplicitFaceFrame
    ? rotateVectorByQuaternion(pose.orientation, routeFluVectorToDeviceFrd(faceFrame.forward))
    : null;
  const worldFaceUp = hasExplicitFaceFrame
    ? rotateVectorByQuaternion(pose.orientation, routeFluVectorToDeviceFrd(faceFrame.up))
    : null;
  if (!worldPosition || !worldRoadForward || !worldRoadUp) return null;

  return Object.freeze({
    coordinateFrame: AR_COORDINATE_FRAME.LOCAL_WORLD_DEVICE,
    epoch: String(pose.epoch || "uninitialized"),
    position: freezeVector(worldPosition),
    roadForward: freezeVector(worldRoadForward),
    roadUp: freezeVector(worldRoadUp),
    ...(worldFaceForward && worldFaceUp
      ? {
          faceForward: freezeVector(worldFaceForward),
          faceUp: freezeVector(worldFaceUp),
        }
      : {}),
    createdAtTimestampNs: finite(pose.timestampNs),
  });
}

/** Derive the current route-relative render pose from an unchanged world anchor. */
export function projectWorldAnchorToRoute(worldAnchor, pose, template = {}) {
  if (!worldAnchor || !isUsableWorldPose(pose)) return null;
  if (worldAnchor.coordinateFrame !== AR_COORDINATE_FRAME.LOCAL_WORLD_DEVICE) return null;
  if (String(worldAnchor.epoch) !== String(pose.epoch)) return null;

  const devicePosition = worldPointToDevice(pose, worldAnchor.position);
  const deviceRoadForward = inverseRotate(pose, worldAnchor.roadForward);
  const deviceRoadUp = inverseRotate(pose, worldAnchor.roadUp);
  if (!devicePosition || !deviceRoadForward || !deviceRoadUp) return null;

  const routePosition = deviceFrdVectorToRouteFlu(devicePosition);
  const routeRoadForward = deviceFrdVectorToRouteFlu(deviceRoadForward);
  const routeRoadUp = deviceFrdVectorToRouteFlu(deviceRoadUp);
  const roadFields = roadFrameFields({ forward: routeRoadForward, up: routeRoadUp });

  let faceFields = {};
  if (worldAnchor.faceForward && worldAnchor.faceUp) {
    const deviceFaceForward = inverseRotate(pose, worldAnchor.faceForward);
    const deviceFaceUp = inverseRotate(pose, worldAnchor.faceUp);
    if (deviceFaceForward && deviceFaceUp) {
      faceFields = faceFrameFields({
        forward: deviceFrdVectorToRouteFlu(deviceFaceForward),
        up: deviceFrdVectorToRouteFlu(deviceFaceUp),
      });
    }
  }

  return Object.freeze({
    ...template,
    x: routePosition[0],
    y: routePosition[1],
    z: routePosition[2],
    ...roadFields,
    ...faceFields,
    coordinateFrame: AR_COORDINATE_FRAME.ROUTE_FLU,
  });
}
