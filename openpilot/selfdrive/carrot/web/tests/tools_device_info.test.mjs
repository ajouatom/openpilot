import assert from "node:assert/strict";
import test from "node:test";

import {
  buildToolsDeviceInfoDialog,
  legacyToolsDeviceInfo,
} from "../src/features/tools/device_info.js";

test("device info dialog always presents IMEI and masks it in support copy", () => {
  const dialog = buildToolsDeviceInfoDialog({
    identity: {
      device_type: "tici",
      imei: "351234567890123",
      imei_available: true,
      dongle_id: "dongle-id",
      hardware_serial: "serial-id",
      position: "center",
    },
    software: { branch: "carrot-wip", commit: "2df34f5d", commit_date: "2026-09-10" },
    connectivity: { technology: "LTE", carrier: "Carrier", sim_state: "READY", modem_version: "RM500Q" },
    runtime: { boot_time: "2026-09-10T10:11:12Z" },
  });
  assert.match(dialog.html, /351234567890123/);
  assert.match(dialog.html, /data-tools-device-info-copy="imei"/);
  assert.match(dialog.supportCopyText, /3512••••••0123/);
  assert.doesNotMatch(dialog.supportCopyText, /351234567890123/);
  assert.equal(dialog.imeiCopyText, "351234567890123");
  assert.equal((dialog.html.match(/center/g) || []).length, 1);
  assert.match(dialog.html, /Booted/);
});

test("device info dialog preserves the IMEI row when the modem is unavailable", () => {
  const dialog = buildToolsDeviceInfoDialog(legacyToolsDeviceInfo({ DeviceType: "mici" }));
  assert.match(dialog.html, />IMEI</);
  assert.doesNotMatch(dialog.html, /data-tools-device-info-copy/);
});

test("device info values are escaped before insertion", () => {
  const dialog = buildToolsDeviceInfoDialog({ identity: { device_type: "<device>", imei: "123" } });
  assert.match(dialog.html, /&lt;device&gt;/);
  assert.doesNotMatch(dialog.html, /<device>/);
});
