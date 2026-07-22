import csv
import gzip

from openpilot.selfdrive.carrot.radar.tools.radar_lead_anticipatory_dataset import augment_file, augment_rows, compact_rows


def row(time_s: float, cutin: float = 0.0, source: str = "teacher", **values: float | str) -> dict[str, str]:
  sample = {
    "time_s": str(time_s), "object_id": "front:40", "aliases": "front:40;corner:1003",
    "cutin_label": str(cutin), "cutin_weight": "1.0" if cutin else "0.08", "cutin_source": source,
    "d_rel": "20", "v_lead": "15", "track_age": "12", "d_path": "2.4", "future_d_path": "1.8",
    "h8_present": "1", "h8_dt": "0.4", "h8_d_path": "2.7",
    "h12_present": "1", "h12_dt": "0.6", "h12_d_path": "2.9",
    "lane1_prob": "0.9", "lane2_prob": "0.9",
  }
  sample.update({name: str(value) for name, value in values.items()})
  return sample


def test_anticipatory_label_requires_later_same_identity_confirmation() -> None:
  rows = [row(0.2), row(0.6), row(1.0, 1.0)]

  stats = augment_rows(rows)

  assert rows[0]["cutin_source"] == "anticipatory"
  assert rows[1]["cutin_source"] == "anticipatory"
  assert 0.5 < float(rows[0]["cutin_label"]) < float(rows[1]["cutin_label"]) < 1.0
  assert stats.anticipatory_positives == 2
  assert stats.earliest_lead_s == 0.8


def test_anticipatory_label_rejects_parallel_motion() -> None:
  rows = [row(0.5, h8_d_path=2.41, h12_d_path=2.42), row(1.0, 1.0)]

  augment_rows(rows)

  assert rows[0]["cutin_label"] == "0.0"
  assert rows[0]["cutin_source"] == "teacher"


def test_anticipatory_label_never_overwrites_manual_negative() -> None:
  rows = [row(0.5, source="manual"), row(1.0, 1.0)]

  stats = augment_rows(rows)

  assert rows[0]["cutin_label"] == "0.0"
  assert rows[0]["cutin_source"] == "manual"
  assert stats.manual_negatives_preserved == 1


def test_anticipatory_label_tracks_sensor_alias_across_fusion_transition() -> None:
  before = row(0.5)
  before["object_id"] = "corner:1003"
  before["aliases"] = "corner:1003"
  confirmed = row(1.0, 1.0)

  augment_rows([before, confirmed])

  assert before["cutin_source"] == "anticipatory"


def test_anticipatory_labels_do_not_cascade_when_processed_again() -> None:
  rows = [row(0.0), row(0.8), row(1.6, 1.0)]

  augment_rows(rows)
  assert rows[0]["cutin_source"] == "teacher"
  assert rows[1]["cutin_source"] == "anticipatory"

  augment_rows(rows)
  assert rows[0]["cutin_source"] == "teacher"


def test_augment_file_keeps_gzip_output(tmp_path) -> None:
  source = tmp_path / "source.fused.csv.gz"
  destination = tmp_path / "output.fused.csv.gz"
  rows = [row(0.5), row(1.0, 1.0)]
  with gzip.open(source, "wt", newline="", encoding="utf-8") as stream:
    writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
    writer.writeheader()
    writer.writerows(rows)

  augment_file(source, destination)

  assert destination.read_bytes()[:2] == b"\x1f\x8b"
  with gzip.open(destination, "rt", newline="", encoding="utf-8") as stream:
    output = list(csv.DictReader(stream))
  assert output[0]["cutin_source"] == "anticipatory"


def test_compact_rows_keeps_positives_hard_negatives_and_samples_easy_rows() -> None:
  positive = row(0.0, 1.0)
  hard = row(0.1, d_path=3.0)
  easy = [row(0.2 + index * 0.1, h8_present=0) for index in range(10)]

  output = compact_rows([positive, hard, *easy], easy_negative_limit=3)

  assert positive in output
  assert hard in output
  assert len(output) == 5
