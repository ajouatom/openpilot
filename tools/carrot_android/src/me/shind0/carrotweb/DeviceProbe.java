package me.shind0.carrotweb;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/** Read-only probes, no redirects, no public addresses, bounded responses/time. */
public final class DeviceProbe {
  public interface Connections { HttpURLConnection open(URL url) throws Exception; }
  private final Connections connections;
  private final Set<HttpURLConnection> active = ConcurrentHashMap.newKeySet();
  private volatile boolean closed;
  public DeviceProbe(Connections connections) { this.connections = connections; }
  public String get(String address, String path, int limit) throws Exception {
    if (closed || Thread.currentThread().isInterrupted() || !DiscoveryPolicy.local(address)) throw new IllegalStateException("Probe cancelled or invalid address");
    HttpURLConnection c = connections.open(new URL("http://" + address + ":7000" + path));
    active.add(c);
    try {
      if (closed) throw new IllegalStateException("Probe cancelled");
      c.setInstanceFollowRedirects(false); c.setConnectTimeout(400); c.setReadTimeout(900);
      c.setRequestProperty("Accept", "application/json,text/html");
      c.setRequestProperty("User-Agent", "CarrotWebAndroid/0.1");
      c.setUseCaches(false);
      if (c.getResponseCode() != 200) throw new IllegalStateException("Not available");
      try (InputStream in = c.getInputStream(); ByteArrayOutputStream out = new ByteArrayOutputStream()) {
        byte[] buffer = new byte[4096]; int n;
        long deadline = System.nanoTime() + 1500000000L;
        while (out.size() < limit && !closed && System.nanoTime() < deadline && (n = in.read(buffer, 0, Math.min(buffer.length, limit - out.size()))) > 0) out.write(buffer, 0, n);
        if (closed) throw new IllegalStateException("Probe cancelled");
        return new String(out.toByteArray(), StandardCharsets.UTF_8);
      }
    } finally { active.remove(c); c.disconnect(); }
  }
  public boolean verify(String address) {
    try {
      String capabilities = get(address, "/api/carrot_navi/capabilities", 16384);
      return DiscoveryPolicy.carrot(capabilities, get(address, "/", 65536));
    } catch (Exception e) { return false; }
  }
  public void close() { closed = true; for (HttpURLConnection c : active) c.disconnect(); }
}
