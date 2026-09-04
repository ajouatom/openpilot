def format_download_size(size: int) -> str:
  if size < 1024 * 1024:
    return f"{size / 1024:.0f} KiB"
  return f"{size / (1024 * 1024):.1f} MiB"


def format_download_detail(downloaded: int, total: int) -> tuple[int, str]:
  if total > 0:
    progress = min(100, int(downloaded * 100 / total))
    return progress, f"{format_download_size(downloaded)} / {format_download_size(total)}"
  if downloaded > 0:
    return 0, f"{format_download_size(downloaded)} received"
  return 0, "Waiting for server response..."


def format_http_error(code: int, reason: str, details: str = "") -> str:
  if code == 409:
    details = " ".join(details.split())
    return details[:500] or "Incompatible openpilot version."

  reason_suffix = f" ({reason})" if reason else ""
  return f"Server returned HTTP {code}{reason_suffix}. Check the URL or try again."
