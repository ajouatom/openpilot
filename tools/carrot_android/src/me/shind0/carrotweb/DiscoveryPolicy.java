package me.shind0.carrotweb;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

/** Platform-independent, bounded discovery and selection rules. */
public final class DiscoveryPolicy {
  public static final long QUICK_MS = 5000, LIMIT_MS = 60000;
  public enum Decision { WAIT, OPEN_ONLY, CHOOSE, EMPTY }
  public static Decision decide(long elapsed, int found) {
    if (found > 1) return Decision.CHOOSE;
    if (found == 1 && elapsed >= QUICK_MS) return Decision.OPEN_ONLY;
    return elapsed >= LIMIT_MS ? Decision.EMPTY : Decision.WAIT;
  }
  public static long ipv4(String value) {
    if (value == null || !value.matches("[0-9]{1,3}(\\.[0-9]{1,3}){3}")) return -1;
    long result = 0;
    for (String part : value.split("\\.")) {
      int n = Integer.parseInt(part);
      if (n > 255 || (part.length() > 1 && part.startsWith("0"))) return -1;
      result = (result << 8) | n;
    }
    return result;
  }
  public static boolean local(String value) {
    long n = ipv4(value);
    return n >= 0 && ((n >>> 24) == 10 || (n >>> 20) == 0xac1 || (n >>> 16) == 0xc0a8 || (n >>> 16) == 0xa9fe);
  }
  public static String address(long n) { return (n >>> 24) + "." + ((n >>> 16) & 255) + "." + ((n >>> 8) & 255) + "." + (n & 255); }
  /** A nearby /24 first, then up to 1024 hosts inside the actual interface subnet. */
  public static List<String> targets(String self, int prefix, String preferred) {
    LinkedHashSet<String> out = new LinkedHashSet<>();
    long ip = ipv4(self);
    if (!local(self) || prefix < 8 || prefix > 30) return new ArrayList<>();
    long mask = (0xffffffffL << (32 - prefix)) & 0xffffffffL, network = ip & mask, broadcast = network | (~mask & 0xffffffffL);
    long prior = ipv4(preferred);
    if (local(preferred) && prior > network && prior < broadcast && prior != ip) out.add(preferred);
    long nearby = ip & 0xffffff00L;
    for (long host = Math.max(network + 1, nearby); host <= Math.min(broadcast - 1, nearby + 255); host++) if (host != ip && local(address(host))) out.add(address(host));
    for (long host = network + 1; host < broadcast && out.size() < 1024; host++) if (host != ip && local(address(host))) out.add(address(host));
    return new ArrayList<>(out);
  }
  public static boolean carrot(String capabilities, String html) {
    return capabilities.matches("(?s).*\\\"feature\\\"\\s*:\\s*\\\"carrotNavi\\\".*")
      && capabilities.matches("(?s).*\\\"ok\\\"\\s*:\\s*true(?:\\s*[,}]).*")
      && html.matches("(?s).*<title>CarrotPilot</title>.*") && html.contains("carrotAssetManifest");
  }
  private DiscoveryPolicy() {}
}
