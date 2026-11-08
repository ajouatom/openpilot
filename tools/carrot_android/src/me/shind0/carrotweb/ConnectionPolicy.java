package me.shind0.carrotweb;

import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.SecureRandom;
import java.util.Base64;
import javax.crypto.Mac;
import javax.crypto.spec.SecretKeySpec;

/** Device proof uses a public double hash; bearer credentials never go to an unproven IP. */
public final class ConnectionPolicy {
  public static boolean origin(String value) {
    try {
      URI u = new URI(value);
      return DiscoveryPolicy.local(u.getHost()) && value.equals("http://" + u.getHost() + ":7000");
    } catch (Exception e) { return false; }
  }
  public static boolean token(String value) { return value != null && value.matches("[A-Za-z0-9_-]{43}"); }
  public static String nonce() { byte[] bytes = new byte[32]; new SecureRandom().nextBytes(bytes); return Base64.getUrlEncoder().withoutPadding().encodeToString(bytes); }
  private static String hex(byte[] bytes) {
    StringBuilder out = new StringBuilder();
    for (byte b : bytes) out.append(String.format(java.util.Locale.ROOT, "%02x", b & 255));
    return out.toString();
  }
  public static String digest(String value) throws Exception { return hex(MessageDigest.getInstance("SHA-256").digest(value.getBytes(StandardCharsets.UTF_8))); }
  public static String identifier(String token) throws Exception { return digest(digest(token)); }
  public static String proof(String token, String nonce, String previous, String next) throws Exception {
    Mac mac = Mac.getInstance("HmacSHA256");
    mac.init(new SecretKeySpec(digest(token).getBytes(StandardCharsets.UTF_8), "HmacSHA256"));
    return hex(mac.doFinal((nonce + "\n" + previous + "\n" + next).getBytes(StandardCharsets.UTF_8)));
  }
  public static boolean verify(String token, String nonce, String previous, String next, String proof) throws Exception {
    return token(token) && token(nonce) && origin(previous) && origin(next) && proof != null && proof.matches("[a-f0-9]{64}")
      && MessageDigest.isEqual(proof(token, nonce, previous, next).getBytes(StandardCharsets.UTF_8), proof.getBytes(StandardCharsets.UTF_8));
  }
  private ConnectionPolicy() {}
}
