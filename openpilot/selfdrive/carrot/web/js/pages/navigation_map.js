"use strict";
(() => {
  const $ = (id) => document.getElementById(id);
  const ko = (localStorage.getItem("language") || navigator.language || "ko").startsWith("ko");
  const tr = (k, e) => ko ? k : e;
  $("title").textContent = tr("Carrot 내비게이션", "Carrot Navigation");
  $("searchTitle").textContent = tr("목적지 검색", "Find a destination");
  $("search").textContent = tr("검색", "Search");
  $("clear").textContent = tr("Mapbox 목적지 취소", "Clear Mapbox destination");
  $("note").textContent = tr("외부 내비의 TBT·경로가 우선입니다. Mapbox 목적지는 외부 내비 연결이 끝나면 사용합니다. 키는 Carrot Web 설정 → 네트워크에서 등록하세요.", "External navigation takes priority. Your Mapbox destination resumes after it disconnects. Register your Public key in Carrot Web Settings → Network.");
  async function api(path, options) {
    const response = await fetch(`/api/navigation/${path}`, options);
    const value = await response.json();
    if (!response.ok) throw new Error(value.error || `HTTP ${response.status}`);
    return value;
  }
  function current(value) { $("current").textContent = value.destination?.place_name || tr("저장된 목적지 없음", "No saved destination"); }
  $("form").addEventListener("submit", async (event) => {
    event.preventDefault(); $("search").disabled = true; $("results").replaceChildren();
    $("status").textContent = tr("검색 중…", "Searching…");
    try {
      const value = await api(`search?q=${encodeURIComponent($("query").value.trim())}`);
      for (const place of value.results) {
        const row = document.createElement("li"), button = document.createElement("button");
        button.textContent = place.place_name;
        button.addEventListener("click", async () => {
          button.disabled = true;
          try { current(await api("destination", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(place)}));
            $("status").textContent = tr("목적지를 저장했습니다. GPS와 지도 수신을 기다립니다.", "Destination saved. Waiting for GPS and map data.");
          } catch (error) { $("status").textContent = error.message; }
          finally { button.disabled = false; }
        }); row.append(button); $("results").append(row);
      }
      $("status").textContent = value.results.length ? tr("목적지를 선택하세요.", "Select a destination.") : tr("검색 결과 없음", "No results.");
    } catch (error) { $("status").textContent = error.message; }
    finally { $("search").disabled = false; }
  });
  $("clear").addEventListener("click", async () => { try { current(await api("destination", {method:"DELETE"})); $("status").textContent = ""; } catch (error) { $("status").textContent = error.message; } });
  api("destination").then(current).catch((error) => { $("status").textContent = error.message; });
  let imageUrl;
  async function refresh() {
    if (!document.hidden) {
      try {
        const response = await fetch("/api/navigation/map.png", {cache:"no-store"});
        if (response.status !== 200) throw new Error("waiting");
        const next = URL.createObjectURL(await response.blob());
        $("map").src = next; if (imageUrl) URL.revokeObjectURL(imageUrl); imageUrl = next;
        $("map").hidden = false;
        const reasons = {
          ready: tr("Nav 모델 지도 · 실시간 경로", "Nav model map · Live route"),
          tiles: tr("지도 데이터를 받지 못했습니다. Mapbox 키와 네트워크를 확인하세요.", "Map data unavailable. Check your Mapbox key and network."),
          route: tr("외부 네비 경로 또는 Mapbox 목적지를 기다리는 중입니다.", "Waiting for an external route or Mapbox destination."),
          position: tr("현재 GPS 위치를 기다리는 중입니다.", "Waiting for a current GPS position."),
          instruction: tr("현재 경로의 안내를 기다리는 중입니다.", "Waiting for current route instructions."),
          route_position: tr("현재 위치가 안내 경로에서 벗어나 있습니다.", "Current position is away from the route."),
          slow: tr("지도 로딩 중입니다. 잠시 기다려 주세요.", "Loading map data. Please wait."),
        };
        $("mapStatus").textContent = reasons[response.headers.get("X-Nav-Map-Status")] || tr("지도를 기다리는 중입니다.", "Waiting for map.");
      } catch (_) { $("map").hidden = true; $("map").removeAttribute("src"); $("mapStatus").textContent = tr("지도를 기다리는 중 · 주행 상태와 GPS를 확인하세요.", "Waiting for map · Check onroad state and GPS."); }
    }
    setTimeout(refresh, 500);
  }
  refresh();
})();
