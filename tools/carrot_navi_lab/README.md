# Carrot Navi Live Pipeline Preview

This preview subscribes directly to the running cluster's production
`/ws/carrot_navi/media` WebSocket. It does not generate or capture video, does
not reconnect the TMAP app, and does not change the cluster/openpilot process.

```text
running TMAP app
  -> existing cluster CarrotNaviReceiver :7714
  -> carrotNaviMedia IPC
  -> existing production web fMP4 bridge :7000
  -> PC live preview :8765
```

The exact same live fMP4 is shown through the production worker in panels 1 and
2. The PC also extracts the AVC sample without re-encoding for WebCodecs panel
3 and decodes that sample to a diagnostic JPEG for panel 4.

Run from the repository root:

```powershell
python tools/carrot_navi_lab/server.py
```

Open `http://127.0.0.1:8765/lab` on the PC or
`http://<PC-LAN-IP>:8765/lab` on a phone.

The production-renderer stream inspector is available at
`http://127.0.0.1:8765/lab/streams`. It shows the actual composed web output,
the complete 28-stream TMAP catalog, current presence, every production layout
slot, and forced previews for rare JSON fallback and failure states. Both pages
are read-only consumers and do not change the cluster or openpilot processes.
