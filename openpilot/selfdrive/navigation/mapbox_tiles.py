"""Mapbox Streets v8 tiles for the same CPU road renderer; no extra GL/GPU work."""
import time

import requests


class MapboxTiles:
  min_zoom, max_zoom = 0, 16

  def __init__(self, token):
    self.token = token
    self.session = requests.Session()

  def get(self, zoom, x, y):
    url = f'https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/{zoom}/{x}/{y}.mvt'
    try:
      started = time.monotonic()
      with self.session.get(url, params={'access_token': self.token}, timeout=(3, 5), stream=True) as response:
        response.raise_for_status()
        body = bytearray()
        for chunk in response.iter_content(65536):
          body.extend(chunk)
          if len(body) > 1024 * 1024 or time.monotonic() - started > 8:
            raise OSError('Map tile response exceeds limits')
        return bytes(body)
    except requests.RequestException:
      # requests errors can contain the private access-token query string.
      raise OSError('Mapbox tiles unavailable') from None

  def close(self):
    self.session.close()
