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
  $("clear").addEventListener("click", async () => { try { current(await api("destination", {method:"DELETE"})); } catch (error) { $("status").textContent = error.message; } });
  api("destination").then(current).catch((error) => { $("status").textContent = error.message; });
  let imageUrl;
  async function refresh() {
    if (!document.hidden) {
      try {
        const response = await fetch("/api/navigation/map.png", {cache:"no-store"});
        if (response.status !== 200) throw new Error("waiting");
        const next = URL.createObjectURL(await response.blob());
        $("map").src = next; if (imageUrl) URL.revokeObjectURL(imageUrl); imageUrl = next;
        $("mapStatus").textContent = tr("Nav 모델 렌더링 · 경로/GPS/지도 데이터가 없으면 검은 화면입니다.", "Nav model rendering · Black when route, GPS or map data is unavailable.");
      } catch (_) { $("map").removeAttribute("src"); $("mapStatus").textContent = tr("지도를 기다리는 중 · 주행 상태와 GPS를 확인하세요.", "Waiting for map · Check onroad state and GPS."); }
    }
    setTimeout(refresh, 500);
  }
  refresh();
})();
