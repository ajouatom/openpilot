export function createDriveVisionHudCanvas(options = {}) {
  const context = options.context;
  if (!context) return null;

  function clear(stageWidth, stageHeight) {
    context.clearRect(0, 0, stageWidth, stageHeight);
  }

  function reset() {}

  function status() {
    return {};
  }

  return Object.freeze({
    clear,
    reset,
    status,
  });
}

export const DriveVisionHudCanvas = Object.freeze({ create: createDriveVisionHudCanvas });

export function installDriveVisionHudCanvasFacade(target = globalThis) {
  target.DriveVisionHudCanvas = DriveVisionHudCanvas;
  return DriveVisionHudCanvas;
}
