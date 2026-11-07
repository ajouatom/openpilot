import me.shind0.carrotweb.LegacyOriginHints;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.charset.StandardCharsets;
import java.util.List;

public class LegacyOriginHintsTest {
  private static int checks;
  private static void check(boolean value) { checks++; if (!value) throw new AssertionError("Origin hint check " + checks); }
  public static void main(String[] args) throws Exception {
    Path root=Files.createTempDirectory("carrot-origin-hints"), db=Files.createDirectories(root.resolve("Default/Local Storage/leveldb"));
    try {
      String old="http://10.225.84.19:7000", next="http://192.168.0.178:7000";
      String padding=new String(new char[32760]).replace('\0','x');
      Files.write(db.resolve("000003.log"),(padding+"META:"+old+"\0value\0_"+next+"\0key\0META:http://8.8.8.8:7000\0META:http://10.0.0.2:70000\0").getBytes(StandardCharsets.ISO_8859_1));
      Files.write(db.resolve("ignored.txt"),"META:http://10.1.1.1:7000".getBytes(StandardCharsets.ISO_8859_1));
      List<String> hints=LegacyOriginHints.read(root.toFile());
      check(hints.contains(old)); check(hints.contains(next)); check(hints.size()==2);
      check(LegacyOriginHints.read(root.resolve("missing").toFile()).isEmpty());
      StringBuilder many=new StringBuilder();
      for(int i=1;i<=30;i++) many.append("META:http://10.0.0.").append(i).append(":7000\0");
      Files.write(db.resolve("000004.ldb"),many.toString().getBytes(StandardCharsets.ISO_8859_1));
      check(LegacyOriginHints.read(root.toFile()).size()==16);
      check(Files.readAllBytes(db.resolve("000003.log")).length==(padding+"META:"+old+"\0value\0_"+next+"\0key\0META:http://8.8.8.8:7000\0META:http://10.0.0.2:70000\0").length());
      System.out.println("Legacy origin hint checks passed: "+checks);
    } finally {
      try (java.util.stream.Stream<Path> paths=Files.walk(root)) { paths.sorted(java.util.Comparator.reverseOrder()).forEach(p -> { try { Files.delete(p); } catch(Exception e) { throw new RuntimeException(e); } }); }
    }
  }
}
