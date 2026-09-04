import json
import os
from copy import deepcopy
from typing import Any, Dict, List, Tuple

from ..config import DEFAULT_SETTINGS_PATH


# mtime-based cache for carrot_settings.json
# - "path" is mutated by carrot_server.py at startup if --settings is passed
# - "mtime" tracks the last loaded file mtime so reload happens only on change
settings_cache: dict = {
  "path": DEFAULT_SETTINGS_PATH,
  "mtime": 0,
  "data": None,        # full json
  "groups": None,      # {group: [param,...]}
  "by_name": None,     # {name: param}
  "groups_list": None, # [{group, egroup, count}, ...]
  "categories": None,  # 대>중>소 트리 ([{id,ko,en,zh,groups:[...]}]) or None when no "menu"
}


def _param_text(value) -> str:
  if isinstance(value, bytes):
    return value.decode("utf-8", errors="ignore").strip()
  return str(value or "").strip()


def current_vehicle_brand(params=None) -> str:
  """Return the opendbc brand for the currently fingerprinted vehicle."""
  if params is None:
    from .params import HAS_PARAMS, Params
    if not HAS_PARAMS or Params is None:
      return ""
    params = Params()

  try:
    cp_bytes = params.get("CarParamsPersistent")
    if cp_bytes:
      from openpilot.cereal import car, messaging
      brand = _param_text(messaging.log_from_bytes(cp_bytes, car.CarParams).brand).lower()
      if brand:
        return brand
  except Exception:
    pass

  # CarName is available after fingerprinting and keeps the settings UI safe
  # if a damaged or older CarParamsPersistent value cannot be decoded.
  try:
    car_name = _param_text(params.get("CarName")).upper()
  except Exception:
    car_name = ""
  if car_name.startswith(("HYUNDAI", "KIA", "GENESIS")):
    return "hyundai"
  return ""


def read_settings_file(path: str) -> Dict[str, Any]:
  with open(path, "r", encoding="utf-8") as f:
    return json.load(f)


def group_index(settings: Dict[str, Any]) -> Tuple[Dict[str, list], Dict[str, Dict[str, Any]], List[Dict[str, Any]]]:
  groups: Dict[str, list] = {}
  by_name: Dict[str, Dict[str, Any]] = {}
  groups_list: List[Dict[str, Any]] = []

  params = settings.get("params", [])
  for p in params:
    g = p.get("group", "기타")
    if g == "기타":
        if "egroup" not in p: p["egroup"] = "Other"
        if "cgroup" not in p: p["cgroup"] = "其他"

    groups.setdefault(g, []).append(p)
    n = p.get("name")
    if n:
      by_name[n] = p

  # group list with egroup/cgroup guess
  for g, items in groups.items():
    egroup = None
    cgroup = None
    for it in items:
      if not egroup and it.get("egroup"):
        egroup = it.get("egroup")
      if not cgroup and it.get("cgroup"):
        cgroup = it.get("cgroup")
      if egroup and cgroup:
        break
    groups_list.append({"group": g, "egroup": egroup, "cgroup": cgroup, "count": len(items)})

  return groups, by_name, groups_list


def _label(node: Dict[str, Any]) -> Dict[str, Any]:
  return {"ko": node.get("ko"), "en": node.get("en"), "zh": node.get("zh")}


def build_menu_categories(data: Dict[str, Any], by_name: Dict[str, Dict[str, Any]]) -> List[Dict[str, Any]] | None:
  """Build the 대>중>소 tree from the optional top-level "menu" block.

  Returns None when no "menu" is present so the frontend falls back to the
  flat group_index view. Leaf "items" carry param *names* only; the frontend
  resolves definitions from items_by_group to avoid duplicating param data.

  Shape:
    [ {id, ko, en, zh,
       groups: [ {id, ko, en, zh, count,
                  sections: [ {id, ko, en, zh, items:[name,...]} ]} ]} ]

  A 중-group whose menu node holds params directly (no nested "groups") is
  normalized to a single label-less section.
  """
  menu = data.get("menu")
  if not menu:
    return None

  def join_labels(nodes: List[Dict[str, Any]], key: str) -> str | None:
    parts = [str(n.get(key) or "").strip() for n in nodes]
    parts = [p for p in parts if p]
    return " · ".join(parts) if parts else None

  def sections_from(nodes: List[Dict[str, Any]], parents: List[Dict[str, Any]] | None = None) -> List[Dict[str, Any]]:
    sections: List[Dict[str, Any]] = []
    parents = parents or []
    for node in nodes:
      path = parents + [node]
      children = node.get("groups") or []
      if children:
        sections.extend(sections_from(children, path))
        continue
      items = [n for n in node.get("params", []) if n in by_name]
      if not items:
        continue
      sections.append({
        "id": "__".join(str(n.get("id") or "") for n in path if n.get("id")),
        "ko": join_labels(path, "ko"),
        "en": join_labels(path, "en"),
        "zh": join_labels(path, "zh"),
        "items": items,
      })
    return sections

  cats: List[Dict[str, Any]] = []
  for cat in menu:
    groups_out: List[Dict[str, Any]] = []
    for grp in cat.get("groups", []):
      if "groups" in grp:
        sections = sections_from(grp["groups"])
      else:
        # params directly under the 중-group → single label-less section
        sections = [{"id": grp.get("id"), "ko": None, "en": None, "zh": None,
                     "items": [n for n in grp.get("params", []) if n in by_name]}]
      count = sum(len(s["items"]) for s in sections)
      groups_out.append({**_label(grp), "id": grp.get("id"), "count": count, "sections": sections})
    cats.append({**_label(cat), "id": cat.get("id"), "groups": groups_out})
  return cats


def filter_settings_catalog_for_brand(
  groups: Dict[str, list],
  groups_list: List[Dict[str, Any]],
  categories: List[Dict[str, Any]] | None,
  brand: str,
) -> tuple[Dict[str, list], List[Dict[str, Any]], List[Dict[str, Any]] | None, set[str]]:
  """Return a catalog view with brand-restricted settings removed.

  The process cache remains brand-neutral because the same server helpers are
  also used by tests and tools. Filtering copies only the response structures
  that need changes and never mutates the cached source catalog.
  """
  normalized_brand = str(brand or "").strip().lower()
  hidden_names = {
    str(item.get("name"))
    for items in groups.values()
    for item in items
    if normalized_brand and normalized_brand in {
      str(hidden_brand).strip().lower()
      for hidden_brand in item.get("hidden_brands", [])
    }
  }
  if not hidden_names:
    return dict(groups), list(groups_list), categories, set()

  filtered_groups = {
    group: [item for item in items if item.get("name") not in hidden_names]
    for group, items in groups.items()
  }
  filtered_groups_list = [
    {**group, "count": len(filtered_groups.get(group.get("group"), []))}
    for group in groups_list
  ]

  filtered_categories = deepcopy(categories) if categories is not None else None
  if filtered_categories is not None:
    for category in filtered_categories:
      visible_groups = []
      for group in category.get("groups", []):
        visible_sections = []
        for section in group.get("sections", []):
          section["items"] = [name for name in section.get("items", []) if name not in hidden_names]
          if section["items"]:
            visible_sections.append(section)
        group["sections"] = visible_sections
        group["count"] = sum(len(section["items"]) for section in visible_sections)
        if group["count"]:
          visible_groups.append(group)
      category["groups"] = visible_groups

  return filtered_groups, filtered_groups_list, filtered_categories, hidden_names


def get_settings_cached() -> Tuple[Dict[str, Any], Dict[str, list], Dict[str, Dict[str, Any]], List[Dict[str, Any]]]:
  path = settings_cache["path"]
  st = os.stat(path)
  mtime = int(st.st_mtime)
  if settings_cache["data"] is None or settings_cache["mtime"] != mtime:
    data = read_settings_file(path)
    groups, by_name, groups_list = group_index(data)
    settings_cache.update({
      "mtime": mtime,
      "data": data,
      "groups": groups,
      "by_name": by_name,
      "groups_list": groups_list,
      "categories": build_menu_categories(data, by_name),
    })
  return (
    settings_cache["data"],
    settings_cache["groups"],
    settings_cache["by_name"],
    settings_cache["groups_list"],
  )
