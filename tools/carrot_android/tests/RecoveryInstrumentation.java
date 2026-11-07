package me.shind0.carrotweb;

import android.app.Instrumentation;
import android.os.Bundle;
import android.webkit.WebStorage;
import org.json.JSONObject;
import java.lang.reflect.Method;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/** Runs against an emulator WebView with an in-memory transport; never a vehicle or real account. */
public final class RecoveryInstrumentation extends Instrumentation {
  private int checks;
  private final String old="http://10.225.84.19:7000", next="http://192.168.0.178:7000";
  private final String browser=ConnectionPolicy.nonce(), grant=ConnectionPolicy.nonce();
  @Override public void onCreate(Bundle arguments) { super.onCreate(arguments); start(); }
  private void check(boolean value) { checks++; if (!value) throw new AssertionError("Recovery check " + checks); }
  private JSONObject storage(ConnectionRecovery r,String origin,JSONObject values) throws Exception {
    Method method=ConnectionRecovery.class.getDeclaredMethod("storage",String.class,JSONObject.class); method.setAccessible(true);
    return (JSONObject)method.invoke(r,origin,values);
  }
  private JSONObject bundle(String token) throws Exception {
    JSONObject result=new JSONObject().put(ConnectionRecovery.SESSION,new JSONObject().put("browser_token",token).put("expires_at",JSONObject.NULL).toString());
    for (String role:ConnectionRecovery.ROLES) result.put(ConnectionRecovery.key(role),grant);
    return result;
  }
  private String run(ConnectionRecovery r) throws Exception {
    CountDownLatch done=new CountDownLatch(1); String[] message=new String[1];
    runOnMainSync(() -> r.start(next,old,value -> { message[0]=value; done.countDown(); }));
    check(done.await(30,TimeUnit.SECONDS)); return message[0];
  }
  @Override public void onStart() {
    Bundle result=new Bundle();
    try {
      runOnMainSync(() -> WebStorage.getInstance().deleteAllData());
      for (int scenario=0;scenario<4;scenario++) {
        final int mode=scenario; int[] moves={0};
        ConnectionRecovery r=new ConnectionRecovery(getTargetContext(),(target,origin,token,body) -> {
          if (target.endsWith("/inspect")) {
            check(ConnectionPolicy.token(token));
            return new JSONObject().put("account_linked",true).put("account_activity_at",token.equals(browser)?200:100);
          }
          if (target.endsWith("/proof")) {
            check(token==null); check(!body.toString().contains(grant)); check(!body.toString().contains(browser));
            JSONObject proofs=new JSONObject();
            if (mode!=1) for (String role:ConnectionRecovery.ROLES) if (!role.equals("drive"))
              proofs.put(role,ConnectionPolicy.proof(grant,body.getString("nonce"),old,next));
            return new JSONObject().put("proofs",proofs);
          }
          if (target.endsWith("/relocate")) { check(token.equals(browser)); moves[0]++; return new JSONObject(); }
          if (target.endsWith("/move")) {
            check(!body.getJSONObject("credentials").has("drive"));
            check(body.getJSONObject("credentials").getString("settings").equals(grant)); moves[0]++; return new JSONObject();
          }
          throw new AssertionError("Unexpected request");
        });
        JSONObject empty=new JSONObject().put(ConnectionRecovery.SESSION,JSONObject.NULL);
        for(String role:ConnectionRecovery.ROLES) empty.put(ConnectionRecovery.key(role),JSONObject.NULL);
        storage(r,old,bundle(browser)); storage(r,next, mode==2 ? bundle(ConnectionPolicy.nonce()) : empty);
        if (mode==3) storage(r,next,bundle(browser));
        String message=run(r);
        JSONObject saved=storage(r,next,null);
        if (mode==1) {
          check(message.isEmpty()); check(moves[0]==1); // a different device still becomes the target
          check(new JSONObject(saved.getString(ConnectionRecovery.SESSION)).getString("browser_token").equals(browser));
          check(saved.isNull(ConnectionRecovery.key("settings")));
        }
        else if (mode==3) { check(message.isEmpty()); check(moves[0]==0); }
        else {
          check(message.isEmpty()); check(moves[0]==2);
          check(new JSONObject(saved.getString(ConnectionRecovery.SESSION)).getString("browser_token").equals(browser));
          check(saved.getString(ConnectionRecovery.key("settings")).equals(grant));
          check(mode==2 ? saved.getString(ConnectionRecovery.key("drive")).equals(grant) : saved.isNull(ConnectionRecovery.key("drive")));
          check(storage(r,old,null).getString(ConnectionRecovery.SESSION).contains(browser));
        }
        runOnMainSync(r::cancel);
      }
      result.putString("stream","Recovery WebView checks passed: " + checks + "\n"); finish(-1,result);
    } catch (Throwable error) { result.putString("stream","FAILED: " + android.util.Log.getStackTraceString(error)); finish(0,result); }
  }
}
