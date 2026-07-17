from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class StatisticsPolicy:
  key: str
  metrics: tuple[str, ...]
  series_mode: str
  supports_window: bool = True

  def as_dict(self) -> dict[str, object]:
    return {
      "key": self.key,
      "metrics": list(self.metrics),
      "seriesMode": self.series_mode,
      "supportsWindow": self.supports_window,
    }


@dataclass(frozen=True, slots=True)
class ServiceSemantics:
  service: str
  group: str
  group_label_key: str
  domain: str
  purpose: str
  recommended_view: str
  privacy: str
  source: str

  def as_dict(self) -> dict[str, object]:
    return {
      "group": self.group,
      "groupLabelKey": self.group_label_key,
      "domain": self.domain,
      "purposeKo": self.purpose,
      "serviceRecommendedView": self.recommended_view,
      "servicePrivacy": self.privacy,
      "semanticSource": self.source,
    }


@dataclass(frozen=True, slots=True)
class FieldSemantics:
  service: str
  path: str
  capnp_type: str
  group: str
  unit: str
  unit_status: str
  unit_basis: str
  privacy: str
  recommended_view: str
  statistics_policy: str
  sequence_like: bool
  display_precision: int | None
  label_key: str
  source: str
  semantic_version: int
  statistics_version: int

  def as_dict(self) -> dict[str, object]:
    return {
      "group": self.group,
      "unit": self.unit,
      "unitStatus": self.unit_status,
      "unitBasis": self.unit_basis,
      "privacy": self.privacy,
      "recommendedView": self.recommended_view,
      "statisticsPolicy": self.statistics_policy,
      "sequenceLike": self.sequence_like,
      "displayPrecision": self.display_precision,
      "labelKey": self.label_key,
      "semanticSource": self.source,
      "semanticVersion": self.semantic_version,
      "statisticsVersion": self.statistics_version,
    }
