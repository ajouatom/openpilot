/* Driving HUD widget (standalone)
 * - Exposes: window.DrivingHud.init(), window.DrivingHud.update(payload)
 * Payload (suggested):
 *  {
 *    cpuTempC, memPct, diskPct, diskLabel,
 *    vEgoKph, vSetKph,
 *    temp: { reason, speedKph, isDecel },   // optional
 *    redDot, tlight,  // tlight: "green"|"red"|"off"
 *    tfGap, gear, gpsOk,
 *    driveMode: { name, kind },             // kind: "normal"|"eco"|"safe"|"sport"
 *    speedLimitKph, speedLimitOver,
 *    apm, tfBars
 *  }
 */
(function () {
  function $(id) { return document.getElementById(id); }
  function setText(id, v) { const el=$(id); if (el) el.textContent = (v==null? "" : String(v)); }
  function show(id, on) { const el=$(id); if (el) el.style.display = on ? "" : "none"; }

  function clamp01(x){ x = Number(x); if (!isFinite(x)) return 0; return Math.max(0, Math.min(1, x)); }

  function setMini(elId, labelId, label, valueText, good=true){
    const el = $(elId);
    if (!el) return;
    if (labelId) setText(labelId, label);
    setText(elId, valueText);
  }

  function setSignalDot(kind){
    const el = $("hudSignalDot");
    if (!el) return;
    if (kind === "red") el.style.background = "#ff2a2a";
    else if (kind === "green") el.style.background = "#15d14b";
    else el.style.background = "rgba(255,255,255,0.18)";
  }

  function setGear(txt){
    const el = $("hudGear");
    if (!el) return;
    el.textContent = txt || "U";
    // green for D/number, gray for unknown, orange for R, blue for N (taste)
    const t = String(txt || "U").toUpperCase();
    if (t === "R") el.style.color = "#ff9c2a";
    else if (t === "N") el.style.color = "#7ec8ff";
    else if (t === "U") el.style.color = "rgba(255,255,255,0.7)";
    else el.style.color = "#1cff57";
  }

  function setDriveMode(name, kind){
    const el = $("hudDriveMode");
    if (!el) return;
    el.textContent = name || "일반";
    el.classList.remove("mode_normal","mode_eco","mode_safe","mode_sport");
    if (kind === "eco") el.classList.add("mode_eco");
    else if (kind === "safe") el.classList.add("mode_safe");
    else if (kind === "sport") el.classList.add("mode_sport");
    else el.classList.add("mode_normal");
  }

  function setGps(ok){
    const el = $("hudGps");
    if (!el) return;
    el.classList.toggle("off", !ok);
  }

  function setRoadLimit(speedKph, over){
    const box = $("hudRoadLimitVal");
    setText("hudRoadLimitVal", (speedKph==null || !isFinite(speedKph)) ? "--" : Math.round(speedKph));
    if (box) box.classList.toggle("over", !!over);
  }

  function setBars(n){
    const wrap = $("hudBars");
    if (!wrap) return;
    const bars = wrap.querySelectorAll(".hudBar");
    const k = Math.max(0, Math.min(bars.length, Number(n) || 0));
    const start = bars.length - k;
    bars.forEach((b, i) => b.classList.toggle("on", i >= start));
  }

  function setGapNum(n){
    const el = $("hudGapNum");
    if (!el) return;
    if (n == null || !isFinite(n)) el.textContent = "-";
    else el.textContent = String(Math.round(n));
  }

  function setTemp(temp){
    //const row = $("hudTemphudTempReason");
    if (!temp || temp.speed == null || !isFinite(temp.speed) || !temp.source) {
      //row.style.display = "none";
      return;
    }
    //row.style.display = "";
    $("hudTempReason").textContent = String(temp.source);
    $("hudTempSpeed").textContent = String(Math.round(temp.speed));

    const isDecel = !!temp.is_decel;
    const color = isDecel ? "#ff9c2a" : "#22ff61";
    $("hudTempReason").style.color = color;
    $("hudTempSpeed").style.color = color;
  }

  function setSpeed(vEgoKph){
    const el = $("hudSpeed");
    //console.log("[HUD] setSpeed", vEgoKph, "el?", !!el);
    if (!el) return;
    if (vEgoKph == null || !isFinite(vEgoKph)) { el.textContent = "--"; return; }
    el.textContent = String(Math.round(vEgoKph));
  }

  function setSetSpeed(vSetKph){
    const el = $("hudSetSpeed");
    if (!el) return;
    if (vSetKph == null || !isFinite(vSetKph)) { el.textContent = "--"; return; }
    el.textContent = String(Math.round(vSetKph));
  }

  function setRedDot(on){
    show("hudRedDot", !!on);
  }

  function setSys(cpuTempC, memPct, diskPct, diskLabel){
    setText("hudCpuVal", (cpuTempC==null || !isFinite(cpuTempC)) ? "--°C" : `${cpuTempC.toFixed(0)}°C`);
    setText("hudMemVal", (memPct==null || !isFinite(memPct)) ? "--%" : `${memPct.toFixed(0)}%`);
    setText("hudDiskVal", (diskPct==null || !isFinite(diskPct)) ? "--%" : `${diskPct.toFixed(0)}%`);
    if (diskLabel) setText("hudDiskLabel", diskLabel);
  }

  const DrivingHud = {
    init() {
      // default visuals
      setBars(0);
      setSignalDot("off");
      setGps(false);
      setDriveMode("일반","normal");
      setRoadLimit(null, false);
      setGapNum(null);
      setGear("U");
      setRedDot(false);
      setTemp(null);
    },

    update(p) {
      if (!p) return;

      setSys(p.cpuTempC, p.memPct, p.diskPct, p.diskLabel);
      setSpeed(p.vEgoKph);
      setSetSpeed(p.vSetKph);
      setSignalDot(p.tlight || "off");
      setRedDot(p.redDot);
      setTemp(p.temp);
      setGapNum(p.tfGap);
      setBars(p.tfBars != null ? p.tfBars : p.tfGap); // default same meaning
      setGear(p.gear);
      setGps(!!p.gpsOk);
      if (p.driveMode) setDriveMode(p.driveMode.name, p.driveMode.kind);
      setRoadLimit(p.speedLimitKph, p.speedLimitOver);
    }
  };

  window.DrivingHud = DrivingHud;
})();
