package me.shind0.carrotweb;

import java.net.URI;

public final class UpdatePolicy {
  public static final String MANIFEST = "https://github.com/ajouatom/openpilot/releases/download/carrot-assist-updates/update.json";
  public static boolean valid(long current, long next, String version, String url, String hash) {
    if (next <= current || next > Integer.MAX_VALUE || version == null || !version.matches("[0-9]+\\.[0-9]+\\.[0-9]+") || hash == null || !hash.matches("[a-fA-F0-9]{64}")) return false;
    return ("https://github.com/ajouatom/openpilot/releases/download/carrot-assist-v"+version+"/CarrotAssist-"+version+".apk").equals(url);
  }
  public static boolean downloadHost(String url) {
    try {
      URI uri = new URI(url); String host = uri.getHost();
      return "https".equals(uri.getScheme()) && uri.getUserInfo() == null && (uri.getPort() == -1 || uri.getPort() == 443) &&
        ("github.com".equals(host) || "release-assets.githubusercontent.com".equals(host) || "objects.githubusercontent.com".equals(host));
    } catch (Exception error) { return false; }
  }
}
