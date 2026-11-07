package me.shind0.carrotweb;

import java.io.File;
import java.io.FileInputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Best-effort address hints for upgrades that predate the app's own origin history.
 * Reads only this app's WebView local-storage files, never changes database files,
 * and returns only local port-7000 origins. Tokens are still read by WebView and
 * authenticated by the relay; a stale/deleted database string is not authority.
 */
public final class LegacyOriginHints {
  private static final Pattern ORIGIN = Pattern.compile("(?:META:|METAACCESS:|_)(http://[0-9]{1,3}(?:\\.[0-9]{1,3}){3}:7000)(?![0-9])");
  public static List<String> read(File webViewRoot) {
    LinkedHashSet<String> result = new LinkedHashSet<>();
    try {
      String root = webViewRoot.getCanonicalPath() + File.separator;
      List<File> files = new ArrayList<>();
      for (String relative : new String[]{"Default/Local Storage/leveldb", "Local Storage/leveldb"}) {
        File dir = new File(webViewRoot, relative);
        if (!dir.getCanonicalPath().startsWith(root)) continue;
        File[] entries = dir.listFiles(); if (entries == null) continue;
        for (File f : entries) if (f.isFile() && f.getName().matches("[0-9]+\\.(log|ldb|sst)") && f.getCanonicalPath().startsWith(root)) files.add(f);
      }
      files.sort((a,b) -> Long.compare(b.lastModified(),a.lastModified()));
      int remaining = 8 * 1024 * 1024, visited = 0;
      byte[] buffer = new byte[32768];
      for (File file : files) {
        if (++visited > 32 || remaining <= 0 || result.size() >= 16 || Thread.currentThread().isInterrupted()) break;
        try (FileInputStream in = new FileInputStream(file)) {
          String tail = ""; int n;
          while (remaining > 0 && (n = in.read(buffer,0,Math.min(buffer.length,remaining))) > 0) {
            remaining -= n;
            String chunk = tail + new String(buffer,0,n,StandardCharsets.ISO_8859_1);
            Matcher matcher = ORIGIN.matcher(chunk);
            while (matcher.find() && result.size() < 16) if (ConnectionPolicy.origin(matcher.group(1))) result.add(matcher.group(1));
            tail = chunk.substring(Math.max(0,chunk.length()-96));
            if (result.size() >= 16 || Thread.currentThread().isInterrupted()) break;
          }
        } catch (Exception ignored) { /* A database rotation cannot block connecting. */ }
      }
    } catch (Exception ignored) {}
    return new ArrayList<>(result);
  }
  private LegacyOriginHints() {}
}
