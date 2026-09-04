from openpilot.system.ui.setup_download import format_download_detail, format_download_size, format_http_error


def test_format_download_size() -> None:
  assert format_download_size(512 * 1024) == "512 KiB"
  assert format_download_size(1_270_680) == "1.2 MiB"


def test_format_download_detail_with_content_length() -> None:
  assert format_download_detail(0, 1_270_680) == (0, "0 KiB / 1.2 MiB")
  assert format_download_detail(635_340, 1_270_680) == (50, "620 KiB / 1.2 MiB")
  assert format_download_detail(2_000_000, 1_270_680)[0] == 100


def test_format_download_detail_without_content_length() -> None:
  assert format_download_detail(0, 0) == (0, "Waiting for server response...")
  assert format_download_detail(8192, 0) == (0, "8 KiB received")


def test_format_http_error_preserves_409_details() -> None:
  assert format_http_error(409, "Conflict", "  Unsupported\nAGNOS version.  ") == "Unsupported AGNOS version."
  assert format_http_error(409, "Conflict") == "Incompatible openpilot version."


def test_format_http_error_reports_other_statuses() -> None:
  assert format_http_error(403, "Forbidden") == "Server returned HTTP 403 (Forbidden). Check the URL or try again."
  assert format_http_error(500, "") == "Server returned HTTP 500. Check the URL or try again."
