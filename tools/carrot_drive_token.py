#!/usr/bin/env python3
"""Issue the Google Drive refresh token for Carrot dashcam uploads (run once on a PC, standard library only).

  python tools/carrot_drive_token.py client_secret.json
    -> opens the Google sign-in page, then writes google_drive.json next to this run;
       copy that file to the device as /data/carrot/google_drive.json (see docs/carrot_drive_upload.md).

client_secret.json is the "Desktop app" OAuth client downloaded from Google Cloud Console.
The scope is drive.file: the device can only see files and folders it created itself.
"""
import http.server
import json
import secrets
import sys
import urllib.parse
import urllib.request
import webbrowser

SCOPE = "https://www.googleapis.com/auth/drive.file"
AUTH_URL = "https://accounts.google.com/o/oauth2/v2/auth"
TOKEN_URL = "https://oauth2.googleapis.com/token"


def main() -> int:
  if len(sys.argv) != 2:
    print(__doc__)
    return 2
  with open(sys.argv[1], encoding="utf-8") as f:
    client = json.load(f)
  client = client.get("installed") or client.get("web") or client
  client_id, client_secret = client["client_id"], client["client_secret"]
  state = secrets.token_urlsafe(16)
  result: dict[str, str] = {}

  class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
      query = dict(urllib.parse.parse_qsl(urllib.parse.urlparse(self.path).query))
      if query.get("state") == state:
        result.update(query)
      self.send_response(200)
      self.send_header("Content-Type", "text/plain; charset=utf-8")
      self.end_headers()
      self.wfile.write("Done. You can close this tab.".encode())

    def log_message(self, *args):
      pass

  server = http.server.HTTPServer(("127.0.0.1", 0), Handler)
  redirect = f"http://127.0.0.1:{server.server_port}"
  url = AUTH_URL + "?" + urllib.parse.urlencode({
    "client_id": client_id, "redirect_uri": redirect, "response_type": "code", "scope": SCOPE,
    "access_type": "offline", "prompt": "consent", "state": state,
  })
  print("Opening the Google sign-in page. If it does not open, visit:\n" + url)
  webbrowser.open(url)
  while "code" not in result and "error" not in result:
    server.handle_request()
  if "error" in result:
    print("Sign-in failed: " + result["error"])
    return 1

  data = urllib.parse.urlencode({
    "code": result["code"], "client_id": client_id, "client_secret": client_secret,
    "redirect_uri": redirect, "grant_type": "authorization_code",
  }).encode()
  with urllib.request.urlopen(urllib.request.Request(TOKEN_URL, data=data)) as resp:
    token = json.load(resp)
  if not token.get("refresh_token"):
    print("Google returned no refresh token; remove the app's access at myaccount.google.com/permissions and retry")
    return 1
  with open("google_drive.json", "w", encoding="utf-8") as f:
    json.dump({"client_id": client_id, "client_secret": client_secret, "refresh_token": token["refresh_token"]}, f)
  print("Wrote google_drive.json - copy it to the device as /data/carrot/google_drive.json")
  return 0


if __name__ == "__main__":
  sys.exit(main())
