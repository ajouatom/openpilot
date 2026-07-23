import os
import shutil

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)

# Web assets
WEB_DIR = os.path.join(ROOT_DIR, "web")
CSS_DIR = os.path.join(WEB_DIR, "css")
JS_DIR = os.path.join(WEB_DIR, "js")
ASSETS_DIR = os.path.join(WEB_DIR, "assets")
PAGES_DIR = os.path.join(WEB_DIR, "pages")
TRAINING_ASSETS_DIR = os.path.join(os.path.dirname(ROOT_DIR), "assets", "training")
OFFROAD_ASSETS_DIR = os.path.join(os.path.dirname(ROOT_DIR), "assets", "offroad")
SELFDRIVE_ASSETS_DIR = os.path.join(os.path.dirname(ROOT_DIR), "assets")
SOUND_ASSETS_DIR = SELFDRIVE_ASSETS_DIR

# Settings file
DEFAULT_SETTINGS_PATH = os.environ.get(
  "CARROT_SETTINGS_PATH",
  os.path.join(os.path.dirname(ROOT_DIR), "carrot_settings.json"),
)

# Carrot data dirs.
# Kept OUTSIDE the git working tree so user state (web settings, YouTube stream
# key, favorites/profiles) survives `git reset --hard` + `git clean -xfd`, which
# the reset/sync tools run and which wipes untracked files. The old location was
# inside the repo (selfdrive/carrot/data) and got erased on every reset; files
# there are migrated once by migrate_legacy_carrot_state().
CARROT_DATA_DIR = os.environ.get("CARROT_DATA_DIR", "/data/carrot")
CARROT_STATE_DIR = os.path.join(CARROT_DATA_DIR, "state")
# Pre-relocation in-repo path (what the old hardcoded CARROT_DATA_DIR pointed at).
CARROT_LEGACY_STATE_DIR = "/data/openpilot/openpilot/selfdrive/carrot/data/state"
CARROT_GIT_STATE_PATH = os.path.join(CARROT_STATE_DIR, "git.json")
CARROT_TOOL_JOBS_STATE_PATH = os.path.join(CARROT_STATE_DIR, "tool_jobs.json")
CARROT_WEB_SETTINGS_PATH = os.path.join(CARROT_STATE_DIR, "web_settings.json")
CARROT_DASHCAM_READ_STATE_PATH = os.path.join(CARROT_STATE_DIR, "dashcam_read_state.json")
CARROT_SETTING_FAVORITES_PATH = os.path.join(CARROT_STATE_DIR, "setting_favorites.json")
CARROT_SETTING_PROFILES_PATH = os.path.join(CARROT_STATE_DIR, "setting_profiles.json")
CARROT_PARAM_CHANGES_PATH = os.path.join(CARROT_STATE_DIR, "param_changes.jsonl")
CARROT_SETTING_UNIT_INDEX_PATH = os.path.join(CARROT_STATE_DIR, "setting_unit_index.json")
CARROT_FINGERPRINT_BASELINE_PATH = os.path.join(CARROT_STATE_DIR, "fingerprint_baseline.json")
CARROT_YOUTUBE_LIVE_STATE_PATH = os.path.join(CARROT_STATE_DIR, "youtube_live.json")
CARROT_YOUTUBE_LIVE_SECRET_PATH = os.path.join(CARROT_STATE_DIR, "youtube_live_secret.json")

# Dashcam
DASHCAM_ROOT = "/data/media/0/realdata"
DASHCAM_CACHE_DIR = os.path.join(CARROT_DATA_DIR, "cache", "dashcam")

# Screen recording
SCREEN_RECORDING_DIRS = (
  "/data/media/0/videos",
  "/data/media/0/screenrecord",
  "/data/media/0/screen_recordings",
  "/data/media/0/screenrecords",
  "/data/media/0/ScreenRecords",
  "/data/media/0/Movies",
  "/sdcard/Movies",
)
SCREEN_RECORDING_EXTS = (".mp4", ".mkv", ".avi", ".mov", ".ts", ".hevc")

# Discord webhook (obfuscated)
DASHCAM_DEFAULT_DISCORD_WEBHOOK = (
  "CxUGAhxOAkMLDhACHQALWk4DAkgCERtdGBFPBAAICBJdQ1tNFV1aU1ZSS0JeRhVY"
  "Vl9RVV0WHD8eGyw3CCkTJQoeGyVCJTosGiEfMhgPVwJbCwEQVxBqCQBXJQk4BB9Z"
  "RUEoVxYELSNfWUgCOBUiF0s4HBpsIjcyLw"
)
DASHCAM_DEFAULT_DISCORD_KEY = "carrot-log"
VISION_DIAG_DEFAULT_DISCORD_WEBHOOK = (
  "CxUGAhxOAlkNGhoMAV8IQQQMDF0THx0CAQwRAQABRh9AVlZQQ0NXTBREWENRW1ga"
  "VF1WVU4gPB8OYgQtQyIEDWc8CBUFNCMGFjl5OT8XJCpeQThCFgY-JRk9GWICNhsr"
  "JwcbIgoCEAQtFDgfSyI9Ql89KWRcIxM1BQ"
)
VISION_DIAG_DEFAULT_DISCORD_KEY = "carrot-vision-log"

# Internal services
WEBRTCD_URL = "http://127.0.0.1:5001/stream"

# Tmux
# Keep the interactive web terminal in its own tmux session. Reusing service
# session names such as "carrot-web" makes Ctrl+C/exit affect the web service
# itself instead of behaving like an independent SSH login shell.
TMUX_WEB_SESSION = os.environ.get("CARROT_TMUX_WEB_SESSION", "carrot-terminal")
TMUX_CAPTURE_LINES = 160
TMUX_START_DIR = "/data/openpilot"

# Params backup file (referenced by params.backup, tools.actions.settings, features.params.routes)
PARAMS_BACKUP_PATH = "/data/media/params_backup.json"

# UI
UNIT_CYCLE = [1, 2, 5, 10, 50, 100]


# Files holding USER state that must survive a repo reset. Migrated once from the
# legacy in-repo location to CARROT_STATE_DIR under /data.
_LEGACY_STATE_FILES = (
  "web_settings.json",
  "youtube_live.json",
  "youtube_live_secret.json",
  "setting_favorites.json",
  "setting_profiles.json",
  "git.json",
  "tool_jobs.json",
)


def migrate_legacy_carrot_state() -> None:
  """One-time copy of user state from the old in-repo path (which `git clean
  -xfd` wiped on reset/sync) to CARROT_STATE_DIR under /data. Idempotent and
  best-effort: copies a file only when the destination is missing, so it never
  clobbers newer settings and is safe to call on every startup."""
  try:
    if os.path.abspath(CARROT_LEGACY_STATE_DIR) == os.path.abspath(CARROT_STATE_DIR):
      return
    if not os.path.isdir(CARROT_LEGACY_STATE_DIR):
      return
    os.makedirs(CARROT_STATE_DIR, exist_ok=True)
    for name in _LEGACY_STATE_FILES:
      src = os.path.join(CARROT_LEGACY_STATE_DIR, name)
      dst = os.path.join(CARROT_STATE_DIR, name)
      if os.path.isfile(src) and not os.path.exists(dst):
        shutil.copy2(src, dst)
  except Exception:
    pass
