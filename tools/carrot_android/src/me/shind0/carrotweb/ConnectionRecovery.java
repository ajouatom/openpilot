package me.shind0.carrotweb;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.webkit.WebStorage;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.webkit.WebResourceRequest;
import org.json.JSONObject;
import org.json.JSONTokener;
import org.json.JSONArray;
import java.io.File;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.LinkedHashSet;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/** App-owned WebView storage recovery. No JavaScript bridge, external scripts or arbitrary URLs. */
final class ConnectionRecovery {
  interface Done { void finish(String message); }
  interface Transport { JSONObject post(String target, String origin, String token, JSONObject body) throws Exception; }
  static final String SESSION = "carrot.assistant.session.v1";
  static final String[] ROLES = {"settings", "drive", "tools"};
  private static final String RELAY = "https://carrot-mcp.shind0.synology.me";
  private final Context activity;
  private final Handler ui = new Handler(Looper.getMainLooper());
  private volatile boolean cancelled;
  private volatile HttpURLConnection request;
  private Thread worker;
  private WebView storage;
  private final Transport transport;
  ConnectionRecovery(Context activity) { this(activity, null); }
  ConnectionRecovery(Context activity, Transport transport) { this.activity = activity; this.transport = transport; }
  static String key(String role) { return "carrot-assistant-" + role + "-v1"; }
  static void remember(Context context,String origin) {
    if (!ConnectionPolicy.origin(origin)) return;
    android.content.SharedPreferences prefs=context.getSharedPreferences("carrot-web",Context.MODE_PRIVATE);
    LinkedHashSet<String> all=new LinkedHashSet<>(); all.add(origin);
    try {
      JSONArray saved=new JSONArray(prefs.getString("connection_origins_v1","[]"));
      for(int i=0;i<saved.length() && all.size()<16;i++) if(ConnectionPolicy.origin(saved.optString(i))) all.add(saved.optString(i));
    } catch(Exception ignored) {}
    prefs.edit().putString("connection_origins_v1",new JSONArray(all).toString()).apply();
  }
  void cancel() {
    cancelled = true;
    if (worker != null) worker.interrupt();
    HttpURLConnection active = request; if (active != null) active.disconnect();
    if (storage != null) { storage.stopLoading(); storage.destroy(); storage = null; }
  }
  void start(String origin, String preferred, Done done) {
    WebStorage.getInstance().getOrigins(origins -> {
      if (cancelled) return;
      LinkedHashSet<String> candidates = new LinkedHashSet<>(); candidates.add(origin);
      if (ConnectionPolicy.origin(preferred)) candidates.add(preferred);
      try {
        JSONArray saved=new JSONArray(activity.getSharedPreferences("carrot-web",Context.MODE_PRIVATE).getString("connection_origins_v1","[]"));
        for(int i=0;i<saved.length() && candidates.size()<16;i++) if(ConnectionPolicy.origin(saved.optString(i))) candidates.add(saved.optString(i));
      } catch(Exception ignored) {}
      for (Object entry : origins.keySet()) {
        String saved = String.valueOf(entry);
        if (ConnectionPolicy.origin(saved) && candidates.size() < 16) candidates.add(saved);
      }
      worker = new Thread(() -> {
        String message;
        try {
          // getOrigins() may omit localStorage-only origins. Old apps had only last_address.
          for (String saved : LegacyOriginHints.read(new File(activity.getApplicationInfo().dataDir,"app_webview")))
            if (candidates.size()<16) candidates.add(saved);
          message = recover(origin, new ArrayList<>(candidates));
        }
        catch (InterruptedException e) { return; }
        catch (Exception e) { message = UiText.get("ChatGPT 연결 복구를 마치지 못했어요. 인터넷과 당근 기기를 확인한 뒤 다시 검색해 주세요.", "Could not restore the ChatGPT connection. Check the Internet connection and Carrot device, then search again."); }
        final String result = message;
        ui.post(() -> { if (!cancelled) done.finish(result); });
      }, "carrot-connection-recovery");
      worker.start();
    });
  }
  // Inspect only the selected origin. Never relocate, issue a code or poll for commands here.
  void checkForVoice(WebView page, String origin, Done done) {
    if (!ConnectionPolicy.origin(origin)) { done.finish(voiceUnavailable()); return; }
    // Read the page already on screen. Creating a second WebView here can stall a cold launch.
    String key = JSONObject.quote(SESSION);
    String script = "(()=>{try{if(location.origin!==" + JSONObject.quote(origin) + ")return null;"
      + "const raw=localStorage.getItem(" + key + ")||sessionStorage.getItem(" + key + ");"
      + "const t=raw&&JSON.parse(raw).browser_token;return typeof t==='string'&&t.length===43?t:'';"
      + "}catch(e){return null}})()";
    page.evaluateJavascript(script, value -> {
      if (cancelled) return;
      try {
        Object result = new JSONTokener(value).nextValue();
        if (result instanceof String) inspectForVoice(origin, (String)result, done);
        else done.finish(voiceUnavailable());
      } catch (Exception e) { done.finish(voiceUnavailable()); }
    });
  }
  private void inspectForVoice(String origin, String token, Done done) {
    worker = new Thread(() -> {
      String message;
      try {
        if (!ConnectionPolicy.token(token)) message = voiceUnlinked();
        else {
          JSONObject state = post(RELAY + "/bridge/inspect", origin, token, new JSONObject());
          if (!state.optBoolean("connected") || !state.optBoolean("account_linked")) message = voiceUnlinked();
          else if (!state.optBoolean("browser_online")) message = UiText.get("당근웹이 아직 응답하지 않아요. 웹이 열린 뒤 다시 눌러 주세요.", "Carrot Web is not responding yet. Wait for the web page to open and try again.");
          else message = "";
        }
      } catch (InterruptedException stopped) { return; }
      catch (HttpFailure e) { message = (e.status == 401 || e.status == 403) ? voiceUnlinked() : voiceUnavailable(); }
      catch (Exception e) { message = voiceUnavailable(); }
      final String result = message;
      ui.post(() -> { if (!cancelled) done.finish(result); });
    }, "carrot-voice-connection-check");
    worker.start();
  }
  static boolean temporaryVoiceFailure(String message) { return !message.isEmpty() && !voiceUnlinked().equals(message); }
  private static String voiceUnlinked() {
    return UiText.get("이 당근웹의 ChatGPT 계정 연결을 확인하지 못했어요. 도구의 ChatGPT 설정 도우미를 확인해 주세요.", "No ChatGPT account link was verified for this Carrot Web. Check the ChatGPT assistant in Tools.");
  }
  static String voiceUnavailable() {
    return UiText.get("당근웹과 ChatGPT 연결을 확인하지 못했어요. 잠시 후 다시 확인해 주세요.", "Could not verify the Carrot Web connection to ChatGPT. Try again in a moment.");
  }
  private static final class Candidate {
    final String origin, token; final JSONObject saved, state;
    Candidate(String origin, String token, JSONObject saved, JSONObject state) { this.origin=origin; this.token=token; this.saved=saved; this.state=state; }
  }
  private String recover(String next, List<String> origins) throws Exception {
    List<Candidate> candidates = new ArrayList<>(), localOnly = new ArrayList<>(); boolean savedConnection = false;
    JSONObject destination = null;
    // Relay requests only go to the fixed HTTPS service, never to a discovered device.
    for (String origin : origins) {
      check(); JSONObject saved = storage(origin, null);
      if (origin.equals(next)) destination = saved;
      String token = "";
      try { token = new JSONObject(saved.optString(SESSION)).optString("browser_token"); } catch (Exception ignored) {}
      if (!ConnectionPolicy.token(token)) {
        if (!origin.equals(next)) localOnly.add(new Candidate(origin, "", saved, null));
        continue;
      }
      remember(activity,origin);
      savedConnection = true;
      try {
        JSONObject state;
        try { state = post(RELAY + "/bridge/inspect", origin, token, new JSONObject()); }
        catch (HttpFailure error) {
          if (error.status != 403 || origin.equals(next)) throw error;
          state = post(RELAY + "/bridge/inspect", next, token, new JSONObject()); // retry an interrupted move
        }
        if (state.optBoolean("account_linked")) candidates.add(new Candidate(origin, token, saved, state));
      } catch (HttpFailure error) { if (error.status != 401 && error.status != 403) throw error; }
    }
    // For direct voice, move only missing local grants the destination proves it already owns.
    if (destination != null) {
      for (Candidate local : localOnly) {
        JSONObject missing = new JSONObject();
        for (String role : new String[]{"settings", "drive"}) {
          String token = local.saved.optString(key(role));
          if (!ConnectionPolicy.token(destination.optString(key(role))) && ConnectionPolicy.token(token))
            missing.put(key(role), token);
        }
        if (missing.length() == 0) continue;
        JSONObject restored = restorePermissions(next, new Candidate(local.origin, "", missing, null));
        if (restored.length() == 0) continue;
        for (String role : ROLES)
          if (restored.has(role)) destination.put(key(role), restored.getString(role));
        storage(next, destination);
      }
    }
    candidates.sort((a,b) -> Long.compare(b.state.optLong("account_activity_at"), a.state.optLong("account_activity_at")));
    if (candidates.isEmpty()) return savedConnection ? UiText.get("저장된 ChatGPT 연결을 확인하지 못했어요. 인터넷과 ChatGPT 연결 설정을 확인해 주세요.", "Could not verify the saved ChatGPT connection. Check your Internet connection and ChatGPT connection settings.") : "";
    Candidate chosen = candidates.get(0);
    if (chosen.origin.equals(next)) return "";
    check();
    // The selected Carrot Web is always the account's target, regardless of device identity.
    // Same session ID and OAuth grants survive; pending commands are discarded by the relay.
    post(RELAY + "/bridge/relocate", next, chosen.token, new JSONObject().put("previous_origin",chosen.origin));
    check();
    // Restore only already allowed local permissions. This is optional and never gates connection.
    JSONObject credentials = restorePermissions(next, chosen);
    check();
    JSONObject moved = new JSONObject();
    moved.put(SESSION, new JSONObject().put("browser_token",chosen.token).put("expires_at",JSONObject.NULL)
      .put("connected",true).put("account_linked",true).toString());
    for (String role : ROLES) moved.put(key(role),credentials.has(role) ? credentials.getString(role) : destination.opt(key(role)));
    storage(next, moved); // retain the old copy until the new origin has been written and verified
    return "";
  }
  private JSONObject restorePermissions(String next, Candidate chosen) throws InterruptedException {
    try {
      String nonce = ConnectionPolicy.nonce(); JSONObject identifiers = new JSONObject();
      for (String role : ROLES) {
        String token = chosen.saved.optString(key(role));
        if (ConnectionPolicy.token(token)) identifiers.put(role, ConnectionPolicy.identifier(token));
      }
      if (identifiers.length() == 0) return new JSONObject();
      JSONObject response = post(next + "/api/assistant/connection/proof", next, null,
        new JSONObject().put("previous_origin",chosen.origin).put("nonce",nonce).put("identifiers",identifiers));
      JSONObject proofs = response.optJSONObject("proofs"), credentials = new JSONObject();
      if (proofs == null) return credentials;
      for (String role : ROLES) {
        String token = chosen.saved.optString(key(role));
        if (ConnectionPolicy.verify(token,nonce,chosen.origin,next,proofs.optString(role))) credentials.put(role,token);
      }
      if (credentials.length()>0) post(next + "/api/assistant/connection/move", next, null,
        new JSONObject().put("previous_origin",chosen.origin).put("credentials",credentials));
      return credentials;
    } catch (InterruptedException e) { throw e; }
    catch (Exception ignored) { return new JSONObject(); } // old firmware or a device without these permissions
  }
  private void check() throws InterruptedException { if (cancelled || Thread.currentThread().isInterrupted()) throw new InterruptedException(); }
  private static final class HttpFailure extends Exception {
    final int status; HttpFailure(int status) { super("HTTP " + status); this.status=status; }
  }
  private JSONObject post(String target, String origin, String token, JSONObject body) throws Exception {
    check();
    if (transport != null) return transport.post(target,origin,token,body);
    HttpURLConnection connection = (HttpURLConnection)new URL(target).openConnection(); request=connection;
    try {
      connection.setInstanceFollowRedirects(false); connection.setConnectTimeout(3500); connection.setReadTimeout(5000);
      connection.setRequestMethod("POST"); connection.setRequestProperty("Origin",origin);
      connection.setRequestProperty("Content-Type","application/json");
      if (token != null) connection.setRequestProperty("Authorization","Bearer " + token);
      connection.setDoOutput(true);
      byte[] data=body.toString().getBytes(StandardCharsets.UTF_8); connection.setFixedLengthStreamingMode(data.length);
      try (java.io.OutputStream out=connection.getOutputStream()) { out.write(data); }
      int status=connection.getResponseCode(); if (status!=200) throw new HttpFailure(status);
      ByteArrayOutputStream bytes=new ByteArrayOutputStream();
      try (InputStream in=connection.getInputStream()) {
        byte[] buffer=new byte[2048]; int count;
        while ((count=in.read(buffer))!=-1) { if (bytes.size()+count>16384) throw new Exception("Oversized response"); bytes.write(buffer,0,count); }
      }
      return new JSONObject(new String(bytes.toByteArray(),StandardCharsets.UTF_8));
    } finally { connection.disconnect(); request=null; }
  }
  @android.annotation.SuppressLint("SetJavaScriptEnabled")
  private JSONObject storage(String origin, JSONObject write) throws Exception {
    check(); CountDownLatch ready=new CountDownLatch(1); JSONObject[] result=new JSONObject[1];
    StringBuilder keys=new StringBuilder("[").append(JSONObject.quote(SESSION));
    for (String role : ROLES) keys.append(',').append(JSONObject.quote(key(role)));
    keys.append(']');
    final String script="(()=>{const keys="+keys+",values="+(write==null ? "null" : write.toString())+";"
      +"if(values)for(const k of keys){if(values[k]===null)localStorage.removeItem(k);else localStorage.setItem(k,values[k]);}"
      +"const out={};for(const k of keys)out[k]=localStorage.getItem(k);return JSON.stringify(out);})()";
    ui.post(() -> {
      if (cancelled) { ready.countDown(); return; }
      WebView view=new WebView(activity); storage=view;
      view.getSettings().setJavaScriptEnabled(true); view.getSettings().setDomStorageEnabled(true);
      view.getSettings().setBlockNetworkLoads(true); view.getSettings().setAllowFileAccess(false); view.getSettings().setAllowContentAccess(false);
      view.setWebViewClient(new WebViewClient() {
        boolean finished;
        @Override public boolean shouldOverrideUrlLoading(WebView w, WebResourceRequest r) { return true; }
        @Override public void onPageFinished(WebView w,String url) {
          if (finished || cancelled || !url.equals(origin+"/")) return; finished=true;
          w.evaluateJavascript(script,value -> {
            try { Object parsed=new JSONTokener(value).nextValue(); if (parsed instanceof String) result[0]=new JSONObject((String)parsed); }
            catch (Exception ignored) {}
            ready.countDown();
          });
        }
      });
      view.loadDataWithBaseURL(origin+"/","<!doctype html><html><head><meta charset=utf-8></head><body></body></html>","text/html","UTF-8",null);
    });
    boolean completed;
    try { completed=ready.await(6,TimeUnit.SECONDS); }
    finally { ui.post(() -> { if (storage!=null) { storage.destroy(); storage=null; } }); }
    check(); if (!completed || result[0]==null) throw new Exception("Web storage unavailable");
    if (write!=null) for (java.util.Iterator<String> it=write.keys();it.hasNext();) {
      String key=it.next(); if (!write.opt(key).equals(result[0].opt(key))) throw new Exception("Web storage verification failed");
    }
    return result[0];
  }
}
