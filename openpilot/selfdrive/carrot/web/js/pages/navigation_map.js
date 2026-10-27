"use strict";
(() => {
  const $ = (id) => document.getElementById(id);
  const ko = (localStorage.getItem("language") || navigator.language || "ko").startsWith("ko");
  const tr = (k, e) => ko ? k : e;
  $("title").textContent = tr("Carrot 내비게이션", "Carrot Navigation");
  $("searchTitle").textContent = tr("목적지 검색", "Find a destination");
  $("search").textContent = tr("검색", "Search");
  $("clear").textContent = tr("Mapbox 목적지 취소", "Clear Mapbox destination");
  $("note").textContent = tr("키 등록 후 목적지를 검색하고 선택하세요. 경로 지도는 차량의 주행 화면이 켜져 있고 현재 위치와 안내 경로가 있을 때 표시됩니다. 외부 내비 안내가 우선이며, 연결이 끝나면 Mapbox 목적지를 사용합니다.", "After saving your Public key, search for and select a destination. The route map needs the vehicle's driving screen to be active, a current position and route guidance. External navigation takes priority; your Mapbox destination resumes after it disconnects.");
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
  function waiting(message) {
    $("map").hidden = true; $("map").removeAttribute("src");
    if (imageUrl) URL.revokeObjectURL(imageUrl); imageUrl = undefined;
    $("mapStatus").hidden = false; $("mapStatus").textContent = message;
  }
  waiting(tr("현재 위치와 목적지를 확인하고 있습니다.", "Checking position and destination."));
  async function refresh() {
    if (!document.hidden) {
      try {
        const response = await fetch("/api/navigation/map.png", {cache:"no-store"});
        if (response.status !== 200 && response.status !== 204) throw new Error("waiting");
        const reasons = {
          ready: tr("Nav 모델 지도 · 실시간 경로", "Nav model map · Live route"),
          tiles: tr("지도 데이터를 받지 못했습니다. Mapbox 키와 네트워크를 확인하세요.", "Map data unavailable. Check your Mapbox key and network."),
          route: tr("아래에서 목적지를 검색·선택하거나 외부 네비의 안내를 시작하세요. 키 등록만으로는 경로 지도가 표시되지 않습니다.", "Search for and select a destination below, or start external navigation guidance. Saving keys alone does not display a route map."),
          offroad: tr("차량의 주행 화면이 켜지면 경로 지도를 표시합니다. 먼저 아래에서 목적지를 선택하세요.", "The route map is available when the vehicle's driving screen is active. Select a destination below first."),
          waiting: tr("지도 서비스를 기다리는 중입니다. 목적지와 현재 위치를 확인하세요.", "Waiting for the map service. Check your destination and current position."),
          position: tr("현재 GPS 위치를 기다리는 중입니다.", "Waiting for a current GPS position."),
          instruction: tr("현재 경로의 안내를 기다리는 중입니다.", "Waiting for current route instructions."),
          route_position: tr("현재 위치가 안내 경로에서 벗어나 있습니다.", "Current position is away from the route."),
          slow: tr("지도 로딩 중입니다. 잠시 기다려 주세요.", "Loading map data. Please wait."),
        };
        const reason = response.headers.get("X-Nav-Map-Status");
        if (response.status === 200 && reason === "ready") {
          const next = URL.createObjectURL(await response.blob());
          $("map").src = next; if (imageUrl) URL.revokeObjectURL(imageUrl); imageUrl = next;
          $("map").hidden = false; $("mapStatus").hidden = true;
        } else {
          waiting(reasons[reason] || reasons.waiting);
        }
      } catch (_) { waiting(tr("지도 상태를 받지 못했습니다. 차량과의 연결을 확인하세요.", "Unable to read map status. Check your connection to the vehicle.")); }
    }
    setTimeout(refresh, 500);
  }
  refresh();
})();
