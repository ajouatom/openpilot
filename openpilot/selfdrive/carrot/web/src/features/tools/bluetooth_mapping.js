export function splitGesture(token) {
  const [base, gesture = "single"] = token.split("@");
  return [base, gesture];
}

export function gestureToken(base, gesture) {
  return gesture === "single" ? base : `${base}@${gesture}`;
}

export function mappingButtons(mapping) {
  return [...new Set(Object.keys(mapping).map(token => splitGesture(token)[0]))];
}

export function learnEvents(mapping, events) {
  let changed = false;
  for (const event of events) {
    if (event.reason !== "test") continue;
    const base = splitGesture(event.button)[0];
    if (!(base in mapping)) { mapping[base] = "none"; changed = true; }
    if (!(event.button in mapping)) { mapping[event.button] = "none"; changed = true; }
  }
  return changed;
}
export const BUTTON_ACTIONS = ["accelCruise", "decelCruise", "gapAdjustCruise", "lfaButton", "cancel"];
export const REMOTE_ACTIONS = ["none", ...BUTTON_ACTIONS, ...BUTTON_ACTIONS.map(button => `${button}Long`),
  "laneLeft", "laneRight", "paddleDecel", "carrotCruise"];
