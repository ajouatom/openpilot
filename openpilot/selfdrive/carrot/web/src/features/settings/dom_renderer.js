function joinClassNames(...names) {
  return names.flat().filter(Boolean).join(" ");
}

function isFiniteCount(value) {
  return Number.isFinite(Number(value));
}

function isProfileGroup(group, profileGroupPrefix) {
  return Boolean(profileGroupPrefix) && String(group || "").startsWith(profileGroupPrefix);
}

function isCategoryDivider(group, categoryDividerPrefix) {
  return Boolean(categoryDividerPrefix) && String(group || "").startsWith(categoryDividerPrefix);
}

export function createSettingsGroupRenderPlan(options = {}) {
  const groups = Array.isArray(options.groups) ? options.groups : [];
  const currentGroup = String(options.currentGroup || "");
  const ids = options.ids || {};
  const getGroupLabel = typeof options.getGroupLabel === "function"
    ? options.getGroupLabel
    : (group) => String(group || "");
  const profilesLabel = String(options.profilesLabel || "Profiles");

  const entries = groups.map((entry, index) => {
    const group = String(entry?.group || "");
    const categoryDivider = isCategoryDivider(group, ids.categoryDividerPrefix || "");
    const divider = group === ids.profilesDivider || categoryDivider;
    const label = divider
      ? String(entry?.label || profilesLabel)
      : String(getGroupLabel(group));

    if (divider) {
      return Object.freeze({
        kind: "divider",
        key: group || `divider:${index}`,
        label,
        categoryDivider,
        index,
      });
    }

    const count = isFiniteCount(entry?.count) ? Number(entry.count) : null;
    return Object.freeze({
      kind: "group",
      key: group,
      group,
      label,
      count,
      title: label,
      favorite: group === ids.favoritesGroup,
      profile: isProfileGroup(group, ids.profileGroupPrefix || ""),
      active: group === currentGroup,
      index,
    });
  });

  // The active group is intentionally excluded: selecting another group can reuse
  // the existing semantic elements and only update their state.
  const signature = JSON.stringify(entries.map((entry) => (
    entry.kind === "divider"
      ? [entry.kind, entry.key, entry.label, entry.categoryDivider]
      : [entry.kind, entry.key, entry.label, entry.count, entry.favorite, entry.profile]
  )));

  return Object.freeze({ entries: Object.freeze(entries), signature });
}

function replaceDividerContent(element, label) {
  const doc = element.ownerDocument;
  const before = doc.createElement("span");
  const strong = doc.createElement("strong");
  const after = doc.createElement("span");
  strong.textContent = label;
  element.replaceChildren(before, strong, after);
}

function replaceGroupContent(element, title) {
  const label = element.ownerDocument.createElement("span");
  label.className = "setting-group-label";
  label.textContent = title;
  element.replaceChildren(label);
}

function applyGroupElement(element, entry, animate) {
  element.onclick = null;
  if (entry.kind === "divider") {
    element.className = joinClassNames(
      "setting-profile-divider",
      entry.categoryDivider && "setting-category-divider",
      animate && "ui-stagger-item",
    );
    element.removeAttribute("data-group");
    element.removeAttribute("title");
    replaceDividerContent(element, entry.label);
  } else {
    element.className = joinClassNames(
      "btn",
      "groupBtn",
      entry.favorite && "groupBtn--favorites",
      entry.profile && "groupBtn--profile",
      entry.active && "active",
      animate && "ui-stagger-item",
    );
    element.dataset.group = entry.group;
    element.title = entry.title;
    replaceGroupContent(element, entry.title);
  }

  if (animate) element.style.setProperty("--i", String(entry.index));
  else element.style.removeProperty("--i");
}

export function renderSettingsGroupList(root, plan, options = {}) {
  if (!root || !plan) return Object.freeze({ reused: false, count: 0 });
  const animate = options.animate === true;
  const entries = Array.isArray(plan.entries) ? plan.entries : [];
  const canReuse = !animate
    && root.dataset.groupsSignature === plan.signature
    && root.children.length === entries.length;

  if (canReuse) {
    entries.forEach((entry, index) => applyGroupElement(root.children[index], entry, false));
    return Object.freeze({ reused: true, count: entries.length });
  }

  const fragment = root.ownerDocument.createDocumentFragment();
  entries.forEach((entry) => {
    const element = root.ownerDocument.createElement(entry.kind === "divider" ? "div" : "button");
    if (entry.kind === "group") element.type = "button";
    applyGroupElement(element, entry, animate);
    fragment.appendChild(element);
  });
  root.replaceChildren(fragment);
  root.dataset.groupsSignature = plan.signature;
  return Object.freeze({ reused: false, count: entries.length });
}

export function createSettingsItemLayoutPlan(options = {}) {
  const entries = Array.isArray(options.entries) ? options.entries : [];
  const group = String(options.group || "");
  const detailMode = options.detailMode === true;
  const profile = options.profile || null;
  const favoriteMode = !detailMode && options.favoriteMode === true;
  const getSectionLabel = typeof options.getSectionLabel === "function"
    ? options.getSectionLabel
    : () => "";
  const getGroupLabel = typeof options.getGroupLabel === "function"
    ? options.getGroupLabel
    : (value) => String(value || "");
  const getProfileSectionExpanded = typeof options.getProfileSectionExpanded === "function"
    ? options.getProfileSectionExpanded
    : () => true;

  const sectionCounts = new Map();
  if (profile) {
    entries.forEach((entry) => {
      const originGroup = String(entry?.group || group);
      sectionCounts.set(originGroup, (sectionCounts.get(originGroup) || 0) + 1);
    });
  }

  let lastCategorySectionKey = null;
  let lastProfileGroup = "";
  const rows = entries.map((entry, index) => {
    const item = entry?.item || null;
    const originGroup = String(entry?.group || group);
    let section = null;

    if (index === 0 && detailMode) {
      section = { kind: "detail", staggerIndex: 1 };
    } else if (index === 0 && favoriteMode) {
      section = { kind: "favorites", staggerIndex: 1 };
    } else if (!detailMode && profile && originGroup !== lastProfileGroup) {
      lastProfileGroup = originGroup;
      const stateKey = `${profile.id}:${originGroup}`;
      section = {
        kind: "profile",
        key: stateKey,
        label: getGroupLabel(originGroup),
        count: sectionCounts.get(originGroup) || 0,
        expanded: getProfileSectionExpanded(stateKey) !== false,
        staggerIndex: Math.min(index + 1, 14),
      };
    } else if (!detailMode && !profile && !favoriteMode && item?.__section) {
      const key = String(item.__section.id || "");
      if (key !== lastCategorySectionKey) {
        lastCategorySectionKey = key;
        section = {
          kind: "category",
          key,
          label: String(getSectionLabel(item.__section) || ""),
          staggerIndex: Math.min(index + 1, 14),
        };
      }
    }

    return Object.freeze({ entry, item, originGroup, index, section: section ? Object.freeze(section) : null });
  });

  return Object.freeze({ rows: Object.freeze(rows) });
}

export function renderSettingsEmptyState(root, options = {}) {
  const empty = root.ownerDocument.createElement("div");
  empty.className = "setting-favorites-empty";
  const title = root.ownerDocument.createElement("div");
  title.className = "setting-favorites-empty__title";
  title.textContent = String(options.title || "");
  const description = root.ownerDocument.createElement("div");
  description.className = "setting-favorites-empty__desc";
  description.textContent = String(options.description || "");
  empty.append(title, description);
  root.appendChild(empty);
  return empty;
}

function createCardSection(root, descriptor, animate) {
  const doc = root.ownerDocument;
  const section = doc.createElement("section");
  section.className = joinClassNames("setting-section-block", animate && "ui-stagger-item");
  if (descriptor.kind === "category") section.dataset.settingSectionId = descriptor.key;
  if (animate) section.style.setProperty("--i", String(descriptor.staggerIndex));

  if (descriptor.label) {
    const title = doc.createElement("div");
    title.className = "setting-group-card__title";
    title.textContent = descriptor.label;
    section.appendChild(title);
  }

  const card = doc.createElement("div");
  card.className = "setting-group-card";
  const body = doc.createElement("div");
  body.className = "setting-group-card__body";
  card.appendChild(body);
  section.appendChild(card);
  root.appendChild(section);
  return Object.freeze({ section, body, header: null });
}

function createProfileSection(root, descriptor, animate) {
  const doc = root.ownerDocument;
  const section = doc.createElement("div");
  section.className = joinClassNames(
    "setting-section-block",
    "setting-profile-section",
    animate && "ui-stagger-item",
    !descriptor.expanded && "is-collapsed",
  );
  if (animate) section.style.setProperty("--i", String(descriptor.staggerIndex));

  const header = doc.createElement("button");
  header.type = "button";
  header.className = "setting-profile-section__header";
  header.setAttribute("aria-expanded", descriptor.expanded ? "true" : "false");
  const label = doc.createElement("span");
  label.className = "setting-profile-section__label";
  label.textContent = descriptor.label;
  const count = doc.createElement("span");
  count.className = "setting-profile-section__count";
  count.textContent = String(descriptor.count);
  const svg = doc.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.classList.add("setting-profile-section__chevron");
  svg.setAttribute("viewBox", "0 0 24 24");
  svg.setAttribute("aria-hidden", "true");
  svg.setAttribute("focusable", "false");
  const path = doc.createElementNS("http://www.w3.org/2000/svg", "path");
  path.setAttribute("d", "m6 9 6 6 6-6");
  svg.appendChild(path);
  header.append(label, count, svg);

  const bodyShell = doc.createElement("div");
  bodyShell.className = "setting-profile-section__body";
  const body = doc.createElement("div");
  body.className = "setting-group-card setting-group-card__body setting-profile-section__bodyInner";
  bodyShell.appendChild(body);
  section.append(header, bodyShell);
  root.appendChild(section);
  return Object.freeze({ section, body, header });
}

export function appendSettingsItemSection(root, descriptor, options = {}) {
  if (!root || !descriptor) return null;
  const animate = options.animate === true;
  return descriptor.kind === "profile"
    ? createProfileSection(root, descriptor, animate)
    : createCardSection(root, descriptor, animate);
}
