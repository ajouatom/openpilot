import me.shind0.carrotweb.ConnectionPolicy;

public class ConnectionPolicyTest {
  private static int checks;
  private static void check(boolean value) { checks++; if (!value) throw new AssertionError("Connection check " + checks); }
  public static void main(String[] args) throws Exception {
    String token=ConnectionPolicy.nonce(), nonce=ConnectionPolicy.nonce();
    String old="http://10.225.84.19:7000", next="http://192.168.0.178:7000";
    check(ConnectionPolicy.token(token)); check(!token.equals(nonce));
    check(ConnectionPolicy.origin(old)); check(ConnectionPolicy.origin(next));
    for (String bad : new String[]{"http://8.8.8.8:7000","http://192.168.0.178:7000/","http://user@192.168.0.178:7000","https://192.168.0.178:7000","http://192.168.0.178:7001","http://192.168.0.178.evil:7000","file:///a"}) check(!ConnectionPolicy.origin(bad));
    String proof=ConnectionPolicy.proof(token,nonce,old,next);
    check(ConnectionPolicy.verify(token,nonce,old,next,proof));
    check(!ConnectionPolicy.verify(token,ConnectionPolicy.nonce(),old,next,proof));
    check(!ConnectionPolicy.verify(token,nonce,next,old,proof));
    check(!ConnectionPolicy.verify(ConnectionPolicy.nonce(),nonce,old,next,proof));
    check(!ConnectionPolicy.verify(token,nonce,old,next,"bad"));
    check(!ConnectionPolicy.identifier(token).equals(ConnectionPolicy.digest(token)));
    // Shared Python/Java protocol vector, including UTF-8 encoding of the hex digest.
    check(ConnectionPolicy.identifier("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa").equals("eb64db39233b0ababf39976672ee3877bde4d18a79cbb8ae1dd2c0da96659bfb"));
    System.out.println("Connection policy checks passed: "+checks);
  }
}
