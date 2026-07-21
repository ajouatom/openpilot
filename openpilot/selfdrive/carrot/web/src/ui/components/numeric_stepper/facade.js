import { createCommitGesture } from "../commit_gesture.js";
import { createNumericStepper, nextStepValue } from "./numeric_stepper.js";

function isObject(value) {
  return (typeof value === "object" && value !== null) || typeof value === "function";
}

export function installNumericStepperFacade(target = globalThis, environment = {}) {
  if (!isObject(target)) throw new TypeError("A facade target is required");

  const createStepper = environment.createNumericStepper || createNumericStepper;
  const createGesture = environment.createCommitGesture || createCommitGesture;
  if (typeof createStepper !== "function") throw new TypeError("A numeric stepper factory is required");
  if (typeof createGesture !== "function") throw new TypeError("A commit gesture factory is required");

  const api = Object.freeze({
    create: createStepper,
    createGesture,
    nextStepValue: environment.nextStepValue || nextStepValue,
  });

  const currentNamespace = isObject(target.CarrotUI) ? target.CarrotUI : null;
  const namespace = currentNamespace && Object.isExtensible(currentNamespace)
    ? currentNamespace
    : Object.assign({}, currentNamespace || {});
  namespace.numericStepper = api;
  target.CarrotUI = namespace;
  target.CarrotNumericStepper = api;
  return api;
}
