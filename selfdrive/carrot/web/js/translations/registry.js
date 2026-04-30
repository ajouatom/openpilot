"use strict";

(function initCarrotTranslations(global) {
  const api = global.CarrotTranslations || {};
  const packs = api.packs || {};
  const order = api.order || ["en", "ko", "zh", "ja", "fr"];
  const strings = api.strings || {};
  const actionLabels = api.actionLabels || {};
  const errorMessages = api.errorMessages || {};
  const driveModes = api.driveModes || {};

  function rebuild() {
    const fallback = packs.en || packs.ko || {};
    order.forEach((lang) => {
      const pack = packs[lang] || {};
      strings[lang] = Object.assign({}, fallback.strings || {}, pack.strings || {});
      actionLabels[lang] = Object.assign({}, fallback.actionLabels || {}, pack.actionLabels || {});
      errorMessages[lang] = Object.assign({}, fallback.errorMessages || {}, pack.errorMessages || {});
      driveModes[lang] = Object.assign({}, fallback.driveModes || {}, pack.driveModes || {});
    });
  }

  api.packs = packs;
  api.order = order;
  api.strings = strings;
  api.actionLabels = actionLabels;
  api.errorMessages = errorMessages;
  api.driveModes = driveModes;
  api.register = function register(lang, pack) {
    if (!lang || !pack) return;
    packs[lang] = Object.assign({}, packs[lang] || {}, pack);
    if (!order.includes(lang)) order.push(lang);
    rebuild();
  };
  api.getPack = function getPack(lang) {
    return packs[lang] || packs.en || packs.ko || {};
  };

  global.CarrotTranslations = api;
})(window);
