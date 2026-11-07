import me.shind0.carrotweb.DiscoveryPolicy;
import me.shind0.carrotweb.DeviceProbe;
import java.util.List;
import java.net.InetSocketAddress;
import java.net.HttpURLConnection;
import java.net.URL;
import com.sun.net.httpserver.HttpServer;

public final class DiscoveryTest {
  static int assertions;
  static void check(boolean condition) { assertions++; if (!condition) throw new AssertionError("Check " + assertions); }
  public static void main(String[] args) throws Exception {
    check(DiscoveryPolicy.decide(4999,1)==DiscoveryPolicy.Decision.WAIT);
    check(DiscoveryPolicy.decide(5000,1)==DiscoveryPolicy.Decision.OPEN_ONLY);
    check(DiscoveryPolicy.decide(5000,0)==DiscoveryPolicy.Decision.WAIT);
    check(DiscoveryPolicy.decide(59999,0)==DiscoveryPolicy.Decision.WAIT);
    check(DiscoveryPolicy.decide(60000,0)==DiscoveryPolicy.Decision.EMPTY);
    check(DiscoveryPolicy.decide(1000,2)==DiscoveryPolicy.Decision.CHOOSE);
    check(DiscoveryPolicy.decide(60000,2)==DiscoveryPolicy.Decision.CHOOSE);
    for (String bad : new String[]{"8.8.8.8","127.0.0.1","192.168.1.999","192.168.001.2","192.168.1.2:7000","10.0.0.1@evil.example","172.32.0.1","100.64.0.1"}) check(!DiscoveryPolicy.local(bad));
    for (String ip : new String[]{"192.168.1.2","10.0.0.2","172.16.0.1","172.31.255.254","169.254.4.2"}) check(DiscoveryPolicy.local(ip));
    List<String> lan = DiscoveryPolicy.targets("192.168.1.50",24,"192.168.1.178");
    check(lan.size()==253); check(lan.get(0).equals("192.168.1.178"));
    check(!lan.contains("192.168.1.0") && !lan.contains("192.168.1.255") && !lan.contains("192.168.1.50"));
    List<String> narrow = DiscoveryPolicy.targets("192.168.1.6",30,"192.168.2.2");
    check(narrow.size()==1 && narrow.get(0).equals("192.168.1.5"));
    List<String> wide = DiscoveryPolicy.targets("10.120.50.8",8,"");
    check(wide.size()==1024); check(wide.get(0).equals("10.120.50.0"));
    check(DiscoveryPolicy.targets("8.8.8.8",24,"").isEmpty());
    String html = "<html><title>CarrotPilot</title><script id=\"carrotAssetManifest\"></script></html>";
    String caps = "{\"ok\":true,\"feature\":\"carrotNavi\"}";
    check(DiscoveryPolicy.carrot(caps,html));
    check(!DiscoveryPolicy.carrot("{\"ok\":true,\"feature\":\"other\"}",html));
    check(!DiscoveryPolicy.carrot(caps,"<title>Other service</title>"));
    HttpServer server = HttpServer.create(new InetSocketAddress("127.0.0.1",0),0);
    server.createContext("/", exchange -> {
      byte[] body = (exchange.getRequestURI().getPath().contains("capabilities") ? caps : html).getBytes(java.nio.charset.StandardCharsets.UTF_8);
      exchange.sendResponseHeaders(200,body.length); exchange.getResponseBody().write(body); exchange.close();
    });
    server.start();
    try {
      DeviceProbe probe = new DeviceProbe(url -> (HttpURLConnection)new URL("http://127.0.0.1:"+server.getAddress().getPort()+url.getFile()).openConnection());
      check(probe.verify("192.168.1.2"));
      check(!probe.verify("8.8.8.8"));
      probe.close(); check(!probe.verify("192.168.1.2"));
    } finally { server.stop(0); }
    System.out.println("Discovery checks passed: "+assertions);
  }
}
