import os

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

# Settings file
DEFAULT_SETTINGS_PATH = "/data/openpilot/selfdrive/carrot_settings.json"

# Carrot data dirs
CARROT_DATA_DIR = "/data/openpilot/selfdrive/carrot/data"
CARROT_STATE_DIR = os.path.join(CARROT_DATA_DIR, "state")
CARROT_GIT_STATE_PATH = os.path.join(CARROT_STATE_DIR, "git.json")
CARROT_TOOL_JOBS_STATE_PATH = os.path.join(CARROT_STATE_DIR, "tool_jobs.json")

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

# Internal services
WEBRTCD_URL = "http://127.0.0.1:5001/stream"

# Tmux
TMUX_WEB_SESSION = "carrot-web"
TMUX_CAPTURE_LINES = 160
TMUX_START_DIR = "/data/openpilot"

# Params backup file (referenced by params.backup, tools.actions.settings, features.params.routes)
PARAMS_BACKUP_PATH = "/data/media/params_backup.json"

# UI
UNIT_CYCLE = [1, 2, 5, 10, 50, 100]
