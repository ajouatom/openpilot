import pytest

from opendbc.car.vin import decode_vin


@pytest.mark.parametrize("raw_vin", [
  b"1HGCM82633A004352",
  b"1HGCM82633A004352" + (b"\x00" * 7),
  b"\xff1HGCM82633A004352\xff\xff",
  b"\x111HGCM82633A004352",
])
def test_decode_vin(raw_vin):
  assert decode_vin(raw_vin) == "1HGCM82633A004352"


@pytest.mark.parametrize("raw_vin", [
  b"\xff" * 17,
  b"1HGCM82633A00435\xff2",
  b"1HGCM82633A00435",
])
def test_decode_vin_invalid(raw_vin):
  assert decode_vin(raw_vin) is None
