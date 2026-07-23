"use strict";

function element(tag, className = "", text = "") {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== "") node.textContent = String(text);
  return node;
}

function formatLabel(template, values = {}) {
  let result = String(template || "");
  for (const [key, value] of Object.entries(values)) {
    result = result.replaceAll(`{${key}}`, String(value));
  }
  return result;
}

const SEGMENT_STATUS_STATES = new Set(["reading", "recent"]);

export function normalizeLogsPlayerGroup(value) {
  if (!value || typeof value !== "object") return null;
  const rawCount = Number(value.segmentCount);
  return Object.freeze({
    route: String(value.route || "").trim(),
    title: String(value.title || value.route || "").trim(),
    dateLabel: String(value.dateLabel || "").trim(),
    timeRange: String(value.timeRange || "").trim(),
    segmentCount: Number.isFinite(rawCount) ? Math.max(0, Math.trunc(rawCount)) : 0,
  });
}

export function normalizeLogsPlayerSegments(values) {
  const seen = new Set();
  const segments = [];
  for (const value of Array.isArray(values) ? values : []) {
    if (!value || typeof value !== "object") continue;
    const id = String(value.id || "").trim();
    if (!id || seen.has(id)) continue;
    seen.add(id);
    segments.push(Object.freeze({
      id,
      name: String(value.name || id).trim(),
      timeLabel: String(value.timeLabel || "").trim(),
      title: String(value.title || value.name || id).trim(),
      subtitle: String(value.subtitle || "").trim(),
      src: String(value.src || "").trim(),
      thumbnailSrc: String(value.thumbnailSrc || "").trim(),
    }));
  }
  return Object.freeze(segments);
}

export function createLogsSegmentStatusTag(state, labels = {}) {
  const normalizedState = String(state || "").trim().toLowerCase();
  if (!SEGMENT_STATUS_STATES.has(normalizedState)) return null;

  const tag = element(
    "span",
    "dashcam-player-status-tag",
    normalizedState === "reading" ? labels.reading : labels.recent,
  );
  tag.dataset.state = normalizedState;
  return tag;
}

export function createLogsPlayerGroupSummary(groupValue, labels = {}) {
  const group = normalizeLogsPlayerGroup(groupValue);
  if (!group) return null;

  const section = element("section", "dashcam-player-group-summary");
  const heading = element("h2", "dashcam-player-section-title", labels.currentGroup);
  const body = element("div", "dashcam-player-group-summary__body");
  const main = element("div", "dashcam-player-group-summary__main");
  const title = element("strong", "dashcam-player-group-summary__title", group.title || group.route);
  const count = element(
    "span",
    "dashcam-player-group-summary__count",
    formatLabel(labels.segmentCount, { count: group.segmentCount }),
  );
  const time = element(
    "span",
    "dashcam-player-group-summary__time",
    [group.dateLabel, group.timeRange].filter(Boolean).join(" · "),
  );

  heading.id = "dashcamPlayerCurrentGroupTitle";
  section.setAttribute("aria-labelledby", heading.id);
  main.append(title, count);
  body.append(main, time);
  section.append(heading, body);

  return Object.freeze({ element: section, group });
}

export function createLogsPlayerSegmentList(segmentValues, options = {}) {
  const region = element("div", "dashcam-player-segment-browser");
  const heading = element("h2", "dashcam-player-section-title", options.labels?.segments);
  const list = element("div", "dashcam-player-segment-list");
  let segments = Object.freeze([]);
  let statusFor = typeof options.statusFor === "function" ? options.statusFor : null;

  heading.id = "dashcamPlayerSegmentListTitle";
  list.setAttribute("aria-labelledby", heading.id);
  region.append(heading, list);

  const syncStatuses = () => {
    list.querySelectorAll(".dashcam-player-segment-item").forEach((button) => {
      const statusHost = button.querySelector(".dashcam-player-segment-status");
      const state = statusFor?.(button.dataset.segmentId) || "";
      const tag = createLogsSegmentStatusTag(state, options.labels);
      statusHost?.replaceChildren(...(tag ? [tag] : []));
      if (statusHost) statusHost.hidden = !tag;
      button.setAttribute(
        "aria-label",
        [button.dataset.segmentLabel, tag?.textContent].filter(Boolean).join(" · "),
      );
    });
  };

  const setActive = (segmentId, scroll = false) => {
    const activeId = String(segmentId || "");
    let activeButton = null;
    list.querySelectorAll(".dashcam-player-segment-item").forEach((button) => {
      const active = button.dataset.segmentId === activeId;
      button.classList.toggle("is-active", active);
      if (active) {
        button.setAttribute("aria-current", "true");
        activeButton = button;
      } else {
        button.removeAttribute("aria-current");
      }
    });
    syncStatuses();
    if (scroll && activeButton) activeButton.scrollIntoView({ block: "nearest" });
    return activeButton;
  };

  const setStatusFor = (resolver) => {
    statusFor = typeof resolver === "function" ? resolver : null;
    syncStatuses();
  };

  const render = (values, activeSegmentId = "") => {
    segments = normalizeLogsPlayerSegments(values);
    const fragment = document.createDocumentFragment();
    for (const segment of segments) {
      const button = element("button", "dashcam-player-segment-item");
      const thumbnail = element("span", "dashcam-player-segment-thumb");
      const copy = element("span", "dashcam-player-segment-copy");
      const meta = element("span", "dashcam-player-segment-meta");
      const time = element("span", "dashcam-player-segment-time", segment.timeLabel);
      const status = element("span", "dashcam-player-segment-status");
      const name = element("span", "dashcam-player-segment-name", segment.name);
      const segmentLabel = [segment.timeLabel, segment.name].filter(Boolean).join(" · ");

      button.type = "button";
      button.dataset.segmentId = segment.id;
      button.dataset.segmentLabel = segmentLabel;
      button.setAttribute("aria-label", segmentLabel);
      if (segment.thumbnailSrc) {
        const image = element("img");
        image.src = segment.thumbnailSrc;
        image.alt = "";
        image.loading = "lazy";
        image.decoding = "async";
        thumbnail.append(image);
      } else {
        thumbnail.classList.add("is-empty");
      }
      status.hidden = true;
      meta.append(name, status);
      copy.append(time, meta);
      button.append(thumbnail, copy);
      button.addEventListener("click", () => options.onSelect?.(segment.id));
      fragment.append(button);
    }
    list.replaceChildren(fragment);
    setActive(activeSegmentId);
    return segments;
  };

  render(segmentValues, options.activeSegmentId);
  return Object.freeze({
    element: region,
    get segments() {
      return segments;
    },
    render,
    setActive,
    setStatusFor,
  });
}

export function createLogsPlayerDialog(options = {}) {
  const dialog = element("div", "dashcam-player-dialog");
  dialog.setAttribute("role", "dialog");
  dialog.setAttribute("aria-modal", "true");

  const layout = element("div", "dashcam-player-layout");
  const frame = element("div", "dashcam-player-frame");
  const summary = createLogsPlayerGroupSummary(options.currentGroup, options.labels);
  let browser = null;
  let segmentHost = null;

  layout.append(frame);
  if (summary) {
    dialog.classList.add("dashcam-player-dialog--browsable");
    browser = element("aside", "dashcam-player-browser");
    browser.setAttribute("aria-label", String(options.labels?.browser || ""));
    segmentHost = element("section", "dashcam-player-segment-region");
    segmentHost.hidden = true;
    browser.append(summary.element, segmentHost);
    layout.append(browser);
  }

  dialog.append(layout);
  return Object.freeze({
    browser,
    dialog,
    frame,
    group: summary?.group || null,
    layout,
    segmentHost,
  });
}
