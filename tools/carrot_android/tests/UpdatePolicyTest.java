import me.shind0.carrotweb.UpdatePolicy;

public class UpdatePolicyTest {
  private static int checks;
  private static void check(boolean value) { checks++; if (!value) throw new AssertionError("Update check "+checks); }
  public static void main(String[] args) {
    String url="https://github.com/ajouatom/openpilot/releases/download/carrot-assist-v0.2.0/CarrotAssist-0.2.0.apk";
    String hash=new String(new char[64]).replace('\0','a');
    check(UpdatePolicy.valid(5,6,"0.2.0",url,hash));
    check(!UpdatePolicy.valid(6,6,"0.2.0",url,hash));
    check(!UpdatePolicy.valid(7,6,"0.2.0",url,hash));
    check(!UpdatePolicy.valid(5,6,"0.2.0",url.replace("ajouatom","other"),hash));
    check(!UpdatePolicy.valid(5,6,"0.2.0",url+"?redirect=evil",hash));
    check(!UpdatePolicy.valid(5,6,"0.2.0",url,"bad"));
    check(!UpdatePolicy.valid(5,6,"0.2.0/../x",url,hash));
    check(UpdatePolicy.downloadHost(url));
    check(UpdatePolicy.downloadHost("https://release-assets.githubusercontent.com/a?signature=b"));
    for(String bad:new String[]{"http://github.com/a","https://github.com.evil.test/a","https://evil@github.com/a","https://github.com:7000/a","file:///tmp/a","https://127.0.0.1/a"}) check(!UpdatePolicy.downloadHost(bad));
    System.out.println("Update policy checks passed: "+checks);
  }
}
