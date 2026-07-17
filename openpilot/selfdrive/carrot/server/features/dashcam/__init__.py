from .routes import register as register_dashcam


def register(app) -> None:
  # Replay is client-computed. The device exposes only the read-only source
  # routes registered by routes.py; legacy scan/query/cache handlers stay off.
  register_dashcam(app)

__all__ = ["register"]
