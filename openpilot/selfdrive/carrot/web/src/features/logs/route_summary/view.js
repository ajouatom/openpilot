import {
  createCompositionDistribution,
  createCompositionStat,
  createDetailRow,
  createEventRow,
  createSummaryMetric,
  createSummarySection,
} from "./components.js";

const METRIC_LABELS = Object.freeze({
  duration: ["summary_duration", "주행시간"],
  distance: ["summary_distance", "주행거리"],
  averageSpeed: ["summary_average_speed", "평균속도"],
  autoRatio: ["summary_auto_ratio", "자동주행 비율"],
});
const DISTANCE_LABELS = Object.freeze({
  totalDistance: ["summary_total_distance", "총 거리"],
  autoDistance: ["summary_auto_distance", "자동"],
  manualDistance: ["summary_manual_distance", "수동"],
  averageSpeed: ["summary_average_speed", "평균속도"],
  maxSpeed: ["summary_max_speed", "최고속도"],
});
const COMPOSITION_LABELS = Object.freeze({
  auto: ["report_auto", "자동주행"],
  manual: ["report_manual", "수동주행"],
  gas: ["summary_gas", "가속 페달 사용"],
  brake: ["summary_brake", "브레이크 사용"],
});
const EVENT_LABELS = Object.freeze({
  hardAccel: ["report_hard_accel", "급가속"],
  overAccel: ["report_over_accel", "과가속"],
  hardDecel: ["report_hard_decel", "급감속"],
  overDecel: ["report_over_decel", "과감속"],
});
const DETAIL_LABELS = Object.freeze({
  stops: ["report_stops", "정차"],
  disengagements: ["summary_disengagements", "운전자 개입"],
  steerOverrides: ["report_steer_ovr", "조향 개입"],
  hardCorners: ["report_corner", "하드코너링"],
  maxLateral: ["report_max_lat", "최대 횡가속"],
  stopTime: ["summary_stop_time", "정차 시간"],
});

function text(key, fallback) {
  return typeof getUIText === "function" ? getUIText(key, fallback) : fallback;
}

function label(table, key) {
  const entry = table[key] || [key, key];
  return text(entry[0], entry[1]);
}

function element(tag, className = "", content = "") {
  const result = document.createElement(tag);
  if (className) result.className = className;
  if (content !== "") result.textContent = String(content);
  return result;
}

export function mountRouteSummary(root, model) {
  root.replaceChildren();
  const article = element("article", "route-summary");

  if (!model.hasData) {
    const empty = element("div", "route-summary__empty", text("report_no_data", "분석할 주행 데이터가 없습니다."));
    empty.setAttribute("role", "status");
    article.append(empty);
    root.append(article);
    return;
  }

  const duration = model.metrics.find((metric) => metric.key === "duration") || model.metrics[0];
  const overview = createSummarySection(text("summary_duration", "주행시간"));
  overview.section.classList.add("route-summary__section--hero");
  const hero = element("div", "route-summary-hero");
  const heroTime = element("div", "route-summary-hero__time");
  heroTime.append(createSummaryMetric(label(METRIC_LABELS, "duration"), duration, { variant: "hero", hideLabel: true }));
  if (model.date || model.timeRange) {
    const subtitle = element("p", "route-summary-hero__subtitle");
    if (model.date) subtitle.append(element("span", "route-summary-hero__date", model.date));
    if (model.timeRange) subtitle.append(element("span", "route-summary-hero__range", model.timeRange));
    heroTime.append(subtitle);
  }
  const compositionList = element("div", "route-summary-composition");
  for (const item of model.composition) {
    compositionList.append(createCompositionStat(label(COMPOSITION_LABELS, item.key), item));
  }
  hero.append(heroTime, compositionList);
  const distributionItems = model.composition
    .map((item) => ({ ...item, label: label(COMPOSITION_LABELS, item.key) }));
  overview.section.append(hero, createCompositionDistribution(distributionItems));

  const dashboard = element("div", "route-summary__dashboard");

  const distance = createSummarySection(text("summary_distance_speed", "주행 거리 / 속도"));
  distance.section.classList.add("route-summary__section--distance");
  const distanceMetrics = element("dl", "route-summary__metrics route-summary__metrics--distance");
  for (const metric of model.distanceMetrics) {
    distanceMetrics.append(createSummaryMetric(label(DISTANCE_LABELS, metric.key), metric, { variant: "compact" }));
  }
  distance.section.append(distanceMetrics);

  const events = createSummarySection(text("summary_events", "주행 이벤트"));
  events.section.classList.add("route-summary__section--events");
  const eventList = element("div", "route-summary-events");
  for (const [key, event] of Object.entries(model.events)) {
    eventList.append(createEventRow(label(EVENT_LABELS, key), event, {
      critical: key.startsWith("hard"),
      kind: key,
    }));
  }
  events.section.append(eventList);

  const details = createSummarySection(text("summary_additional_metrics", "추가 지표"));
  details.section.classList.add("route-summary__section--details");
  const detailList = element("dl", "route-summary-details");
  const countLabel = text("report_unit_times", "회");
  for (const item of model.details) detailList.append(createDetailRow(label(DETAIL_LABELS, item.key), item, countLabel));
  details.section.append(detailList);

  const warningTotal = model.warnings.fcw + model.warnings.ldw + model.warnings.driverDistracted;
  if (warningTotal > 0) {
    const warnings = element("p", "route-summary__warning");
    warnings.textContent = `${text("report_warns", "경고")} · FCW ${model.warnings.fcw} · LDW ${model.warnings.ldw} · ${text("report_warn_dm", "운전자 주의")} ${model.warnings.driverDistracted}`;
    details.section.append(warnings);
  }

  const metadata = document.createElement("details");
  metadata.className = "route-summary-metadata";
  metadata.append(element("summary", "route-summary-metadata__summary", text("summary_analysis_info", "분석 정보")));
  const metadataList = element("dl", "route-summary-metadata__list");
  const metadataRows = [
    [text("report_source", "분석 소스"), model.metadata.source],
    [text("report_segments", "세그먼트"), `${model.metadata.processed}/${model.metadata.requested}`],
    [text("summary_partial_segments", "부분 분석"), String(model.metadata.partial)],
    [text("summary_policy_version", "정책 버전"), String(model.metadata.policyVersion)],
    [text("summary_route_id", "경로 ID"), model.route],
  ];
  for (const [term, value] of metadataRows) {
    const row = element("div", "route-summary-metadata__row");
    row.append(element("dt", "", term), element("dd", "", value));
    metadataList.append(row);
  }
  metadata.append(metadataList);

  dashboard.append(distance.section, details.section, events.section);
  article.append(overview.section, dashboard, metadata);
  root.append(article);
}
