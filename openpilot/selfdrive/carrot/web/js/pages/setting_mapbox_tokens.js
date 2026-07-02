"use strict";

(function () {
  const MAPBOX_SECTION = "SYS_NET";
  const CARD_ID = "mapboxTokenCard";
  const TOKEN_TYPES = [
    {
      key: "public",
      param: "MapboxPublicKey",
      titleKey: "mapbox_token_public_title",
      titleFallback: "Public",
      placeholderKey: "mapbox_public_placeholder",
      placeholderFallback: "예: pk.eyJ...abc",
      inputType: "text",
    },
    {
      key: "secret",
      param: "MapboxSecretKey",
      titleKey: "mapbox_token_secret_title",
      titleFallback: "Secret",
      placeholderKey: "mapbox_secret_placeholder",
      placeholderFallback: "예: sk.eyJ...abc",
      inputType: "password",
    },
  ];

  const state = {
    loading: false,
    loaded: false,
    tokens: {},
  };

  function text(key, fallback, vars) {
    if (typeof getUIText === "function") return getUIText(key, fallback, vars);
    let value = fallback || key;
    Object.entries(vars || {}).forEach(([name, replacement]) => {
      value = value.replaceAll(`{${name}}`, String(replacement));
    });
    return value;
  }

  function toast(message, options) {
    if (typeof showAppToast === "function") showAppToast(message, options || undefined);
  }

  function escapeHtmlLocal(value) {
    if (typeof escapeHtml === "function") return escapeHtml(value);
    return String(value ?? "").replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;",
      "<": "&lt;",
      ">": "&gt;",
      '"': "&quot;",
      "'": "&#39;",
    }[ch]));
  }

  function itemsBox() {
    return document.getElementById("items");
  }

  function sectionRoot() {
    return itemsBox()?.querySelector(`[data-setting-section-id="${MAPBOX_SECTION}"]`) || null;
  }

  function appliedLabel(keyType) {
    const info = state.tokens?.[keyType] || {};
    if (state.loading && !state.loaded) return text("loading", "로딩 중...");
    return info.configured
      ? text("mapbox_token_applied", "적용됨")
      : text("mapbox_token_not_applied", "안됨");
  }

  function summaryStatus() {
    return text("mapbox_token_status_summary", "Public {public} · Secret {secret}", {
      public: appliedLabel("public"),
      secret: appliedLabel("secret"),
    });
  }

  function fieldValue(type) {
    const info = state.tokens?.[type.key] || {};
    return info.masked || text("mapbox_token_missing", "없음");
  }

  function fieldControl(type) {
    const info = state.tokens?.[type.key] || {};
    return renderSettingFormControl({
      label: text(type.titleKey, type.titleFallback),
      value: fieldValue(type),
      placeholder: text(type.placeholderKey, type.placeholderFallback),
      configured: Boolean(info.configured),
      editAction: "edit",
      actions: [
        { action: "clear", label: text("delete", "삭제"), disabled: !info.configured },
      ],
    });
  }

  function renderCardContents(card) {
    card.innerHTML = `
      <div class="mapbox-token-card__head">
        <div class="mapbox-token-card__title">${escapeHtmlLocal(text("mapbox_token_title", "Mapbox 키"))}</div>
        <div class="mapbox-token-card__status" data-role="summary-status">${escapeHtmlLocal(summaryStatus())}</div>
      </div>
      <div class="mapbox-token-list">
        ${TOKEN_TYPES.map((type) => `
          <div class="mapbox-token-field" data-mapbox-token-type="${escapeHtmlLocal(type.key)}">
            <div class="mapbox-token-field__meta">
              <div class="mapbox-token-field__title">${escapeHtmlLocal(text(type.titleKey, type.titleFallback))}</div>
            </div>
            ${fieldControl(type)}
          </div>
        `).join("")}
      </div>
    `;

    TOKEN_TYPES.forEach((type) => {
      const field = card.querySelector(`[data-mapbox-token-type="${type.key}"]`);
      field?.querySelector('[data-action="edit"]')?.addEventListener("click", () => editToken(type.key));
      field?.querySelector('[data-action="clear"]')?.addEventListener("click", () => clearToken(type.key));
    });
  }

  function rerenderCard() {
    const card = document.getElementById(CARD_ID);
    if (card) renderCardContents(card);
  }

  function syncStatusText() {
    const card = document.getElementById(CARD_ID);
    if (!card) return;
    const summary = card.querySelector('[data-role="summary-status"]');
    if (summary) summary.textContent = summaryStatus();
  }

  function ensureCard() {
    const root = sectionRoot();
    if (!root) return null;
    const settingsCard = Array.from(root.children).find((child) => child.classList?.contains("setting-group-card"));
    const settingsBody = settingsCard?.querySelector(".setting-group-card__body");
    if (!settingsBody) return null;

    let card = document.getElementById(CARD_ID);
    if (!card) {
      card = document.createElement("div");
      card.id = CARD_ID;
      card.className = "mapbox-token-card";
      renderCardContents(card);
    }

    const mapboxStyleRow = settingsBody.querySelector('[data-setting-name="MapboxStyle"]');
    const nextNode = mapboxStyleRow?.nextSibling || null;
    if (card.parentElement !== settingsBody || card.previousSibling !== mapboxStyleRow) {
      settingsBody.insertBefore(card, nextNode);
    }
    return card;
  }

  async function loadTokens() {
    if (state.loading) return;
    state.loading = true;
    syncStatusText();
    try {
      const result = await getJson("/api/mapbox/tokens");
      state.tokens = {
        public: result.public || {},
        secret: result.secret || {},
      };
      state.loaded = true;
      rerenderCard();
    } catch (err) {
      toast(err?.message || String(err), { tone: "error" });
    } finally {
      state.loading = false;
      syncStatusText();
    }
  }

  async function editToken(keyType) {
    const info = TOKEN_TYPES.find((type) => type.key === keyType);
    if (!info) return;
    await openSettingFormDialog({
      title: text(info.titleKey, info.titleFallback),
      placeholder: text(info.placeholderKey, info.placeholderFallback),
      inputType: info.inputType,
      onSave: (value) => saveToken(keyType, value),
    });
  }

  async function saveToken(keyType, value) {
    const token = String(value || "").trim();
    try {
      await postJson("/api/mapbox/token/validate", { key_type: keyType, token });
    } catch (err) {
      const payload = err?.payload || null;
      throw new Error((payload ? friendlyValidationMessage(payload, keyType) : "") || err?.message || String(err));
    }

    try {
      const result = await postJson("/api/mapbox/token", { key_type: keyType, token });
      state.tokens = {
        public: result.public || state.tokens.public || {},
        secret: result.secret || state.tokens.secret || {},
      };
      state.loaded = true;
      rerenderCard();
      toast(text("mapbox_token_saved", "저장했습니다."));
    } catch (err) {
      const payload = err?.payload || null;
      throw new Error((payload ? friendlyValidationMessage(payload, keyType) : "") || err?.message || String(err));
    }
  }

  async function clearToken(keyType) {
    const info = TOKEN_TYPES.find((type) => type.key === keyType);
    const name = text(info?.titleKey, info?.titleFallback || keyType);
    const ok = await appConfirm(
      text("mapbox_token_clear_confirm", "{name} 키를 삭제할까요?", { name }),
      {
        title: text("delete", "삭제"),
        confirmLabel: text("delete", "삭제"),
        cancelLabel: text("cancel", "취소"),
      },
    );
    if (!ok) return;
    try {
      const result = await requestJson(`/api/mapbox/token?key_type=${encodeURIComponent(keyType)}`, { method: "DELETE" });
      state.tokens = {
        public: result.public || state.tokens.public || {},
        secret: result.secret || state.tokens.secret || {},
      };
      state.loaded = true;
      rerenderCard();
      toast(text("mapbox_token_cleared", "Mapbox 토큰 삭제됨"));
    } catch (err) {
      toast(err?.message || String(err), { tone: "error" });
    }
  }

  function friendlyValidationMessage(result, keyType) {
    if (!result?.format_ok) {
      const reason = String(result?.reason || "").toLowerCase();
      if (reason === "required") return text("mapbox_token_need_input", "토큰을 입력해 주세요.");
      if (reason === "space") return text("mapbox_token_no_space", "토큰에 공백이 들어가면 안 됩니다.");
      if (reason === "prefix") return keyType === "public"
        ? text("mapbox_public_prefix_hint", "Public key는 pk.로 시작해야 합니다.")
        : text("mapbox_secret_prefix_hint", "Secret key는 sk.로 시작해야 합니다.");
      if (reason === "short") return text("mapbox_token_too_short", "토큰이 너무 짧습니다.");
      const message = String(result?.message || "").toLowerCase();
      if (message.includes("required")) return text("mapbox_token_need_input", "토큰을 입력해 주세요.");
      if (message.includes("space")) return text("mapbox_token_no_space", "토큰에 공백이 들어가면 안 됩니다.");
      if (message.includes("start")) return keyType === "public"
        ? text("mapbox_public_prefix_hint", "Public key는 pk.로 시작해야 합니다.")
        : text("mapbox_secret_prefix_hint", "Secret key는 sk.로 시작해야 합니다.");
      if (message.includes("short")) return text("mapbox_token_too_short", "토큰이 너무 짧습니다.");
      return text("mapbox_token_format_bad", "토큰 형식을 확인해 주세요.");
    }

    if (keyType === "secret") {
      return text("mapbox_secret_format_ok", "Secret key 형식은 정상입니다. 현재 앱에서는 Public key만 실제 지도에 사용합니다.");
    }

    if (result?.online_ok) {
      return text("mapbox_public_ready", "사용 가능합니다. Mapbox 서버 확인까지 완료했습니다.");
    }

    return text("mapbox_public_check_failed", "토큰 형식은 맞지만 Mapbox 서버 확인에 실패했습니다. 토큰 또는 인터넷 연결을 확인해 주세요.");
  }

  function sync() {
    const card = ensureCard();
    if (!card) return;
    if (!state.loaded) loadTokens();
    else syncStatusText();
  }

  window.addEventListener("carrot:paramchange", (event) => {
    const name = event?.detail?.name;
    if (name === "MapboxPublicKey" || name === "MapboxSecretKey") {
      state.loaded = false;
      sync();
    }
  });

  window.CarrotMapboxTokenSettings = Object.freeze({ sync });
})();
