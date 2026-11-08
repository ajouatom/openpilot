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
  private String run(ConnectionRecovery r,String preferred) throws Exception {
    CountDownLatch done=new CountDownLatch(1); String[] message=new String[1];
    runOnMainSync(() -> r.start(next,preferred,value -> { message[0]=value; done.countDown(); }));
    check(done.await(30,TimeUnit.SECONDS)); return message[0];
  }
  @Override public void onStart() {
    Bundle result=new Bundle();
    try {
      java.util.Locale.setDefault(java.util.Locale.ENGLISH);
      android.app.Activity screen=startActivitySync(new android.content.Intent(getTargetContext(),MainActivity.class).addFlags(android.content.Intent.FLAG_ACTIVITY_NEW_TASK));
      runOnMainSync(() -> {
        screen.getWindow().addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        WebStorage.getInstance().deleteAllData();
      });
      for (int scenario=0;scenario<5;scenario++) {
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
        // Reproduce 0.2.1: last_address is already the new IP, with no native origin history.
        getTargetContext().getSharedPreferences("carrot-web",0).edit().remove("connection_origins_v1").commit();
        if (mode==4) {
          // Chromium batches its disk writes; this fixture represents a prior app session.
          java.io.File root=new java.io.File(getTargetContext().getApplicationInfo().dataDir,"app_webview");
          long until=System.currentTimeMillis()+10000;
          while (!LegacyOriginHints.read(root).contains(old) && System.currentTimeMillis()<until) Thread.sleep(100);
          if (!LegacyOriginHints.read(root).contains(old)) throw new AssertionError("Legacy origin fixture not persisted: " + java.util.Arrays.toString(root.list()));
        }
        String message=run(r,mode==4 ? next : old);
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
      for (boolean knownDevice : new boolean[]{true, false}) {
        int[] moves = {0};
        ConnectionRecovery direct = new ConnectionRecovery(getTargetContext(), (target,origin,token,body) -> {
          check(target.startsWith(next + "/api/assistant/connection/")); // never contact the relay
          if (target.endsWith("/proof")) {
            JSONObject proofs = new JSONObject();
            if (knownDevice) for (String role : new String[]{"drive","settings"})
              proofs.put(role,ConnectionPolicy.proof(grant,body.getString("nonce"),old,next));
            return new JSONObject().put("proofs",proofs);
          }
          check(target.endsWith("/move")); moves[0]++; return new JSONObject();
        });
        storage(direct,old,bundle(browser).put(ConnectionRecovery.SESSION,JSONObject.NULL));
        storage(direct,next,bundle(browser).put(ConnectionRecovery.SESSION,JSONObject.NULL).put(ConnectionRecovery.key("drive"),JSONObject.NULL).put(ConnectionRecovery.key("settings"),JSONObject.NULL));
        check(run(direct,old).isEmpty());
        JSONObject restored = storage(direct,next,null);
        check(restored.isNull(ConnectionRecovery.SESSION));
        check(knownDevice ? grant.equals(restored.optString(ConnectionRecovery.key("drive"))) : restored.isNull(ConnectionRecovery.key("drive")));
        check(knownDevice ? grant.equals(restored.optString(ConnectionRecovery.key("settings"))) : restored.isNull(ConnectionRecovery.key("settings")));
        check(moves[0] == (knownDevice ? 1 : 0));
        runOnMainSync(direct::cancel);
      }
      // The startup check must authenticate this origin, never relocate or execute a command.
      for (int scenario = 0; scenario < 4; scenario++) {
        final int mode = scenario;
        int[] calls = {0};
        ConnectionRecovery inspector = new ConnectionRecovery(getTargetContext(), (target,origin,token,body) -> {
          check(target.endsWith("/bridge/inspect") && next.equals(origin));
          check(browser.equals(token) && body.length() == 0);
          calls[0]++;
          return new JSONObject().put("connected", true).put("account_linked", mode != 2).put("browser_online", mode != 3);
        });
        storage(inspector, next, mode == 0 ? new JSONObject().put(ConnectionRecovery.SESSION,JSONObject.NULL) : bundle(browser));
        CountDownLatch checked = new CountDownLatch(1);
        String[] outcome = new String[1];
        android.webkit.WebView[] currentPage = new android.webkit.WebView[1];
        runOnMainSync(() -> {
          android.webkit.WebView page = new android.webkit.WebView(getTargetContext()); currentPage[0] = page;
          page.getSettings().setJavaScriptEnabled(true); page.getSettings().setDomStorageEnabled(true);
          page.getSettings().setBlockNetworkLoads(true);
          page.setWebViewClient(new android.webkit.WebViewClient() {
            @Override public void onPageFinished(android.webkit.WebView w,String url) {
              inspector.checkForVoice(w,next,message -> { outcome[0] = message; checked.countDown(); });
            }
          });
          page.loadDataWithBaseURL(next+"/","<html></html>","text/html","UTF-8",null);
        });
        check(checked.await(15,TimeUnit.SECONDS));
        runOnMainSync(currentPage[0]::destroy);
        check(outcome[0].isEmpty() == (mode == 1));
        check(calls[0] == (mode == 0 ? 0 : 1));
        runOnMainSync(inspector::cancel);
      }
      JSONObject git = new JSONObject().put("ok",true).put("available",true).put("state","ok").put("behind",1).put("ahead",0)
        .put("head","1234567890123456789012345678901234567890").put("target_head","9876543210987654321098765432109876543210");
      check(VoiceWelcome.verifiedUpdate(git));
      check(!VoiceWelcome.verifiedUpdate(git.put("ahead",1)));
      check(!VoiceWelcome.verifiedUpdate(git.put("ahead",0).put("behind",0)));
      check(!VoiceWelcome.verifiedUpdate(git.put("behind",1).put("available",false)));
      android.content.SharedPreferences mode = getTargetContext().getSharedPreferences("test-local-voice-mode",0);
      mode.edit().clear().commit(); MainActivity.initializeVoiceMode(mode);
      check(mode.getBoolean("voice_local_first",false));
      mode.edit().clear().putString("last_address","192.168.1.1").commit(); MainActivity.initializeVoiceMode(mode);
      check(!mode.getBoolean("voice_local_first",true));
      mode.edit().putBoolean("voice_local_first",true).commit(); MainActivity.initializeVoiceMode(mode);
      check(mode.getBoolean("voice_local_first",false));
      runOnMainSync(this::checkFloatingButton);
      runOnMainSync(() -> {
        String visible=visibleText(screen.findViewById(android.R.id.content));
        check(visible.contains("Looking for Carrot")); check(visible.contains("Search again"));
        check(visible.contains("Enter IP address")); check(!visible.matches("(?s).*[가-힣].*"));
      });
      checkInstructionsCopy(screen, java.util.Locale.KOREAN);
      checkInstructionsCopy(screen, java.util.Locale.ENGLISH);
      runOnMainSync(screen::finish);
      result.putString("stream","Recovery WebView checks passed: " + checks + "\nEnglish native search screen verified.\n"); finish(-1,result);
    } catch (Throwable error) { result.putString("stream","FAILED: " + android.util.Log.getStackTraceString(error)); finish(0,result); }
  }
  private android.view.View windowText(String label, int flags) {
    java.util.ArrayList<android.view.View> found = new java.util.ArrayList<>();
    for (android.view.View root : android.view.inspector.WindowInspector.getGlobalWindowViews()) {
      root.findViewsWithText(found, label, flags);
    }
    for (android.view.View view : found) if (view.isShown()) return view;
    throw new AssertionError("Missing visible view: " + label);
  }
  private void checkInstructionsCopy(android.app.Activity screen, java.util.Locale locale) {
    boolean korean = locale.equals(java.util.Locale.KOREAN);
    runOnMainSync(() -> {
      java.util.Locale.setDefault(locale);
      windowText("Device name", android.view.View.FIND_VIEWS_WITH_CONTENT_DESCRIPTION).performClick();
    });
    waitForIdleSync();
    runOnMainSync(() -> windowText(korean ? "ChatGPT 맞춤 지침 복사" : "Copy ChatGPT instructions",
      android.view.View.FIND_VIEWS_WITH_TEXT).performClick());
    waitForIdleSync();
    runOnMainSync(() -> {
      android.content.ClipboardManager manager = screen.getSystemService(android.content.ClipboardManager.class);
      android.content.ClipData clip = manager.getPrimaryClip();
      check(clip != null && clip.getItemCount() == 1);
      String prompt = clip.getItemAt(0).getText().toString();
      check(prompt.contains("CarrotAssist") && prompt.contains(korean ? "왼쪽으로 차로 변경해" : "change lanes left"));
      android.widget.TextView message = (android.widget.TextView)windowText(korean ? "‘성격’" : "Personality",
        android.view.View.FIND_VIEWS_WITH_TEXT);
      check(message.getText().toString().contains(prompt));
      check(message.getText().toString().contains(korean ? "사용자 지정 지침" : "Custom instructions"));
      windowText(korean ? "확인" : "Done", android.view.View.FIND_VIEWS_WITH_TEXT).performClick();
    });
    waitForIdleSync();
  }
  private void checkFloatingButton() {
    android.content.Context context = getTargetContext();
    android.content.SharedPreferences prefs = context.getSharedPreferences("voice-position-test", 0);
    prefs.edit().clear().commit();
    android.widget.FrameLayout frame = new android.widget.FrameLayout(context);
    FloatingVoiceButton mic = new FloatingVoiceButton(context,prefs);
    frame.addView(mic, new android.widget.FrameLayout.LayoutParams(80,80));
    layout(frame,700,900); mic.place();
    int[] clicks = {0}; mic.setOnClickListener(v -> clicks[0]++);
    float initialX = mic.getX(), initialY = mic.getY();
    touch(mic,0,30,30); touch(mic,2,-170,240); touch(mic,1,-170,240);
    check(clicks[0] == 0 && mic.getX() < initialX && mic.getY() > initialY);
    float x = mic.getX(), y = mic.getY();
    touch(mic,0,30,30); touch(mic,1,30,30); check(clicks[0] == 1);
    frame.removeView(mic);
    FloatingVoiceButton restored = new FloatingVoiceButton(context,prefs);
    frame.addView(restored,new android.widget.FrameLayout.LayoutParams(80,80)); layout(frame,700,900); restored.place();
    check(Math.abs(restored.getX()-x)<1 && Math.abs(restored.getY()-y)<1);
    layout(frame,400,360); restored.place();
    check(restored.getX() >= 0 && restored.getY() >= 0 && restored.getX()+80<=400 && restored.getY()+80<=360);
  }
  private static void layout(android.widget.FrameLayout frame, int width, int height) {
    int exact = android.view.View.MeasureSpec.EXACTLY;
    frame.measure(android.view.View.MeasureSpec.makeMeasureSpec(width,exact), android.view.View.MeasureSpec.makeMeasureSpec(height,exact));
    frame.layout(0,0,width,height);
  }
  private static void touch(FloatingVoiceButton view, int action, float x, float y) {
    long now = android.os.SystemClock.uptimeMillis();
    android.view.MotionEvent event = android.view.MotionEvent.obtain(now,now,action,x,y,0);
    try { view.onTouchEvent(event); } finally { event.recycle(); }
  }
  private String visibleText(android.view.View view) {
    StringBuilder out=new StringBuilder();
    if (view instanceof android.widget.TextView) out.append(((android.widget.TextView)view).getText()).append('\n');
    if (view instanceof android.view.ViewGroup) {
      android.view.ViewGroup group=(android.view.ViewGroup)view;
      for(int i=0;i<group.getChildCount();i++) out.append(visibleText(group.getChildAt(i)));
    }
    return out.toString();
  }
}
