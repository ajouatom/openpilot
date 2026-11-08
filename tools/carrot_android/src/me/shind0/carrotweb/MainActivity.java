package me.shind0.carrotweb;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.assist.AssistContent;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.net.ConnectivityManager;
import android.net.LinkAddress;
import android.net.LinkProperties;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.provider.Settings;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowInsets;
import android.webkit.CookieManager;
import android.webkit.WebChromeClient;
import android.webkit.WebResourceRequest;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.FrameLayout;
import android.widget.ProgressBar;
import android.widget.ScrollView;
import android.widget.TextView;
import org.json.JSONObject;
import java.net.HttpURLConnection;
import java.net.Inet4Address;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;

public final class MainActivity extends Activity {
  private final Handler ui = new Handler(Looper.getMainLooper());
  private final Map<String, Device> found = new LinkedHashMap<>();
  private final List<DeviceProbe> probes = new ArrayList<>();
  private final int background = Color.rgb(16,21,28), foreground = Color.rgb(238,241,246), accent = Color.rgb(237,145,68);
  private LinearLayout root, content, devices;
  private TextView title, status;
  private ProgressBar progress;
  private WebView web;
  private ExecutorService workers;
  private volatile int generation;
  private long started;
  private boolean scanning, choosing, foregroundActivity;
  private String currentAddress;
  private SharedPreferences preferences;
  private AppUpdater updater;
  private ConnectionRecovery recovery;
  private boolean recovering;
  private ConnectionRecovery voiceCheck;
  private long voiceVerifiedUntil;
  private VoiceWelcome welcome;
  private PhoneVoice phoneVoice;
  private boolean phoneSession, waitingForPermission, resumePhoneVoice, permissionGreeting;
  private int phoneGeneration, permissionDevice;
  private String phoneSearchId;
  private final LocalCommand.DirectionCheck directionCheck = new LocalCommand.DirectionCheck();
  private static final int PHONE_MIC = 42;

  private String lastPhoneRequest = "", lastPhoneResult = "failed";
  private boolean voiceStarting, voiceFromAuto, pageReady;
  private int voiceAttempt, autoVoiceGeneration = -1;
  private final Runnable retryVoice = this::beginVoiceAttempt;
  private final Runnable tryAutoVoice = this::maybeStartAutoVoice;
  private CharSequence titleBeforeVoiceCheck;
  private final Runnable voiceTimeout = () -> finishVoiceCheck(ConnectionRecovery.voiceUnavailable());
  private boolean checkAfterBackground;
  private DeviceProbe resumeProbe;
  private static final class Device {
    final String address, name;
    Device(String address, String name) { this.address = address; this.name = name; }
  }
  private static final class Target {
    final String address; final DeviceProbe probe;
    Target(String address, DeviceProbe probe) { this.address = address; this.probe = probe; }
  }
  @Override public void onCreate(Bundle saved) {
    super.onCreate(saved);
    preferences = getSharedPreferences("carrot-web", MODE_PRIVATE);
    initializeVoiceMode(preferences);
    updater = new AppUpdater(this, preferences);
    welcome = new VoiceWelcome(this);
    phoneVoice = new PhoneVoice(this, new PhoneVoice.Listener() {
      public void status(String text) { title.setText(text); if (!phoneVoice.listening()) phoneSession = false; }
      public void heard(LocalCommand command) { handlePhoneCommand(command); }
      public void missed(String heard, String reason) {
        directionCheck.clear();
        String text = heard == null ? "" : heard.trim();
        if (text.length() > 48) text = text.substring(0, 48) + "…";
        String prefix = text.isEmpty() ? "" : UiText.get("“" + text + "”로 들었어요. ", "I heard “" + text + "”. ");
        replyAndListen(phoneGeneration, prefix + reason);
      }
      public void heardSetting(SpokenSetting setting) { handleSpokenSetting(setting); }
      public void findSettings(String query) { startSettingAction("findSettings", JSONObject.quote(query)); }
    });
    root = column(); root.setBackgroundColor(background);
    if (android.os.Build.VERSION.SDK_INT >= 30) root.setOnApplyWindowInsetsListener((v,insets) -> {
      android.graphics.Insets b = insets.getInsets(WindowInsets.Type.systemBars() | WindowInsets.Type.displayCutout() | WindowInsets.Type.ime());
      v.setPadding(b.left,b.top,b.right,b.bottom);
      // The native host already reserves these insets; WebView must not reserve them again.
      return WindowInsets.CONSUMED;
    });
    // API 26-29 use the legacy system-window inset accessors.
    if (android.os.Build.VERSION.SDK_INT < 30) root.setOnApplyWindowInsetsListener((v,insets) -> {
      v.setPadding(insets.getSystemWindowInsetLeft(),insets.getSystemWindowInsetTop(),insets.getSystemWindowInsetRight(),insets.getSystemWindowInsetBottom()); return insets.consumeSystemWindowInsets();
    });
    LinearLayout bar = new LinearLayout(this); bar.setGravity(Gravity.CENTER_VERTICAL);
    Button search = button("⌕", v -> startSearch()); search.setContentDescription(UiText.get("당근 기기 찾기", "Find Carrot"));
    bar.addView(search,new LinearLayout.LayoutParams(dp(52),dp(48)));
    title = text("CarrotAssist",18); title.setSingleLine(true); title.setEllipsize(android.text.TextUtils.TruncateAt.END);
    title.setOnClickListener(v -> updater.info()); title.setContentDescription(UiText.get("기기 이름 · 앱 정보와 업데이트 확인", "Device name · App information and updates"));
    bar.addView(title,new LinearLayout.LayoutParams(0,dp(48),1)); title.setGravity(Gravity.CENTER_VERTICAL);
    root.addView(bar);
    FrameLayout page = new FrameLayout(this);
    root.addView(page, new LinearLayout.LayoutParams(-1, 0, 1));
    content = column();
    page.addView(content, new FrameLayout.LayoutParams(-1, -1));
    FloatingVoiceButton assistant = new FloatingVoiceButton(this, preferences);
    assistant.setOnClickListener(v -> invokeAssistant());
    page.addView(assistant, new FrameLayout.LayoutParams(dp(68), dp(68)));
    page.addOnLayoutChangeListener((v,l,t,r,b,oldL,oldT,oldR,oldB) -> assistant.place());
    setContentView(root);
    startSearch();
    updater.check(false);
  }
  static void initializeVoiceMode(SharedPreferences preferences) {
    // Preserve the launch mode of an existing installation before discovery records its first address.
    if (!preferences.contains("voice_local_first")) {
      boolean alreadyUsed = preferences.contains("last_address") || preferences.contains("voice_auto_start");
      preferences.edit().putBoolean("voice_local_first", !alreadyUsed).apply();
    }
  }
  private int dp(int value) { return Math.round(value * getResources().getDisplayMetrics().density); }
  private LinearLayout column() { LinearLayout v = new LinearLayout(this); v.setOrientation(LinearLayout.VERTICAL); return v; }
  private TextView text(String value,int size) { TextView v = new TextView(this); v.setText(value); v.setTextSize(size); v.setTextColor(foreground); return v; }
  private Button button(String label,View.OnClickListener listener) { Button b = new Button(this); b.setText(label); b.setAllCaps(false); b.setTextColor(accent); b.setOnClickListener(listener); return b; }
  private void stopSearch() {
    stopPhoneVoice(); resumePhoneVoice = false;
    voiceVerifiedUntil = 0;
    if (welcome != null) welcome.reset();
    pageReady = false;
    ui.removeCallbacks(tryAutoVoice);
    cancelVoiceCheck();
    if (resumeProbe != null) { resumeProbe.close(); resumeProbe = null; }
    scanning = false; generation++; ui.removeCallbacks(tick);
    if (progress != null) progress.setVisibility(View.GONE);
    if (workers != null) { workers.shutdownNow(); workers = null; }
    for (DeviceProbe p : probes) p.close(); probes.clear();
  }
  private void startSearch() {
    if (recovery != null) { recovery.cancel(); recovery = null; }
    recovering = false;
    stopSearch(); choosing = false; found.clear(); currentAddress = null;
    if (web != null) { web.stopLoading(); web.loadUrl("about:blank"); }
    content.removeAllViews(); title.setText(UiText.get("당근 기기 찾기", "Find Carrot"));
    LinearLayout panel = column(); panel.setPadding(dp(22),dp(28),dp(22),dp(16));
    panel.addView(text(UiText.get("당근을 찾고 있어요", "Looking for Carrot"),26));
    TextView help = text(UiText.get("같은 Wi-Fi 또는 휴대폰 핫스팟에 연결해 주세요.\n5초 동안 한 대만 찾으면 자동으로 열립니다.", "Connect to the same Wi-Fi or phone hotspot.\nOne device found within five seconds opens automatically."),16); help.setPadding(0,dp(14),0,dp(16)); panel.addView(help);
    status = text(UiText.get("검색 준비 중…", "Preparing search…"),16); panel.addView(status);
    progress = new ProgressBar(this,null,android.R.attr.progressBarStyleHorizontal); progress.setIndeterminate(true); panel.addView(progress);
    devices = column(); ScrollView list = new ScrollView(this); list.addView(devices); panel.addView(list,new LinearLayout.LayoutParams(-1,0,1));
    LinearLayout actions = new LinearLayout(this);
    actions.addView(button(UiText.get("다시 검색", "Search again"),v -> startSearch()),new LinearLayout.LayoutParams(0,dp(52),1));
    actions.addView(button(UiText.get("주소 직접 입력", "Enter IP address"),v -> manualAddress()),new LinearLayout.LayoutParams(0,dp(52),1)); panel.addView(actions);
    panel.addView(button(UiText.get("검색 중단", "Stop search"),v -> { stopSearch(); status.setText(UiText.get("검색을 중단했습니다. 발견된 기기를 선택할 수 있어요.", "Search stopped. You can select a device already found.")); progress.setVisibility(View.GONE); }));
    content.addView(panel,new LinearLayout.LayoutParams(-1,-1));
    List<Target> targets = targets();
    if (targets.isEmpty()) { status.setText(UiText.get("검색할 Wi-Fi를 찾지 못했어요. 연결을 확인하거나 주소를 입력해 주세요.", "No Wi-Fi network found. Check the connection or enter an IP address.")); progress.setVisibility(View.GONE); return; }
    started = SystemClock.elapsedRealtime(); scanning = true; final int token = generation;
    workers = Executors.newFixedThreadPool(32); AtomicInteger cursor = new AtomicInteger();
    for (int w=0;w<32;w++) workers.execute(() -> {
      while (token == generation && SystemClock.elapsedRealtime()-started < DiscoveryPolicy.LIMIT_MS) {
        int index = cursor.getAndIncrement();
        if (index >= targets.size()) break;
        Target target = targets.get(index);
        if (target.probe.verify(target.address)) {
          String name = UiText.get("당근파일럿", "CarrotPilot");
          try { JSONObject response = new JSONObject(target.probe.get(target.address,"/api/params_bulk?names=CarName",8192));
            String car = response.optJSONObject("values") == null ? "" : response.getJSONObject("values").optString("CarName","");
            if (!car.trim().isEmpty() && !car.equals("0")) name = car.trim().substring(0,Math.min(car.trim().length(),80));
          } catch (Exception ignored) {}
          final Device device = new Device(target.address,name);
          ui.post(() -> { if (token == generation && scanning) addDevice(device); });
        }
      }
    });
    ui.post(tick);
    // Repeat a finite sweep when nothing was found; newly booted devices can appear.
    scheduleRetry(token,targets,5000);
  }
  private void scheduleRetry(int token,List<Target> targets,long delay) {
    ui.postDelayed(() -> {
      if (token != generation || !scanning || SystemClock.elapsedRealtime()-started >= DiscoveryPolicy.LIMIT_MS) return;
      if (workers instanceof java.util.concurrent.ThreadPoolExecutor && ((java.util.concurrent.ThreadPoolExecutor)workers).getActiveCount() == 0) {
        for (Target target : targets) {
          if (found.containsKey(target.address)) continue;
          workers.execute(() -> {
            if (token != generation || SystemClock.elapsedRealtime()-started >= DiscoveryPolicy.LIMIT_MS) return;
            if (target.probe.verify(target.address)) ui.post(() -> { if (token == generation && scanning) addDevice(new Device(target.address,UiText.get("당근파일럿", "CarrotPilot"))); });
          });
        }
      }
      scheduleRetry(token,targets,5000);
    },delay);
  }
  private final Runnable tick = new Runnable() {
    @Override public void run() {
      if (!scanning) return;
      long elapsed = SystemClock.elapsedRealtime()-started;
      DiscoveryPolicy.Decision decision = DiscoveryPolicy.decide(elapsed,found.size());
      if (decision == DiscoveryPolicy.Decision.OPEN_ONLY && !choosing) { openDevice(found.values().iterator().next()); return; }
      if (elapsed >= DiscoveryPolicy.LIMIT_MS) {
        stopSearch(); status.setText(found.isEmpty() ? UiText.get("1분 동안 기기를 찾지 못했어요. 같은 Wi-Fi인지 확인한 뒤 다시 검색해 주세요.", "No device found within one minute. Check that you are on the same Wi-Fi and search again.") : UiText.get("검색을 마쳤어요. 연결할 기기를 선택해 주세요.", "Search complete. Select a device to connect.")); return;
      }
      if (found.size() > 1) { choosing = true; status.setText(found.size()+UiText.get("대를 찾았어요. 연결할 기기를 선택해 주세요. · ", " devices found. Select one. · ")+Math.max(0,(60-elapsed/1000))+UiText.get("초 남음", "s remaining")); }
      else if (elapsed < DiscoveryPolicy.QUICK_MS) status.setText(UiText.get("빠른 검색 · ", "Quick search · ")+Math.max(1,(5-elapsed/1000))+UiText.get("초 · 발견 ", "s · Found ")+found.size()+UiText.get("대", ""));
      else status.setText(UiText.get("계속 검색 중 · ", "Searching · ")+Math.max(0,(60-elapsed/1000))+UiText.get("초 남음", "s remaining"));
      ui.postDelayed(this,200);
    }
  };
  private void addDevice(Device device) {
    if (found.containsKey(device.address)) return;
    found.put(device.address,device);
    if (found.size()>1) choosing = true;
    Button row = button(device.name+"\n"+device.address+":7000",v -> openDevice(device));
    row.setGravity(Gravity.START | Gravity.CENTER_VERTICAL); row.setPadding(dp(14),dp(12),dp(14),dp(12));
    devices.addView(row,new LinearLayout.LayoutParams(-1,dp(82)));
  }
  private List<Target> targets() {
    LinkedHashMap<String,Target> out = new LinkedHashMap<>();
    ConnectivityManager manager = (ConnectivityManager)getSystemService(CONNECTIVITY_SERVICE);
    try {
      for (Network network : manager.getAllNetworks()) {
        NetworkCapabilities caps = manager.getNetworkCapabilities(network); LinkProperties props = manager.getLinkProperties(network);
        if (caps == null || props == null || !caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) || caps.hasTransport(NetworkCapabilities.TRANSPORT_VPN)) continue;
        DeviceProbe probe = new DeviceProbe(url -> (HttpURLConnection)network.openConnection(url)); probes.add(probe);
        for (LinkAddress address : props.getLinkAddresses()) if (address.getAddress() instanceof Inet4Address) appendTargets(out,address.getAddress().getHostAddress(),address.getPrefixLength(),probe);
      }
      // A phone hosting a hotspot has no Wi-Fi client Network for its AP interface.
      for (NetworkInterface iface : Collections.list(NetworkInterface.getNetworkInterfaces())) {
        if (!iface.isUp() || iface.isLoopback() || !iface.getName().matches("(?i)(wlan|wifi|swlan|ap|softap|eth).*")) continue;
        DeviceProbe probe = new DeviceProbe(url -> (HttpURLConnection)url.openConnection()); probes.add(probe);
        for (InterfaceAddress address : iface.getInterfaceAddresses()) if (address.getAddress() instanceof Inet4Address) appendTargets(out,address.getAddress().getHostAddress(),address.getNetworkPrefixLength(),probe);
      }
    } catch (Exception ignored) {}
    return new ArrayList<>(out.values());
  }
  private void appendTargets(Map<String,Target> out,String self,int prefix,DeviceProbe probe) {
    for (String ip : DiscoveryPolicy.targets(self,prefix,preferences.getString("last_address",""))) if (!out.containsKey(ip)) out.put(ip,new Target(ip,probe));
  }
  private void manualAddress() {
    stopSearch(); EditText input = new EditText(this); input.setSingleLine(true); input.setHint("192.168.0.178"); input.setText(preferences.getString("last_address",""));
    input.setInputType(android.text.InputType.TYPE_CLASS_PHONE);
    new AlertDialog.Builder(this).setTitle(UiText.get("당근 기기 IP 주소", "Carrot device IP address")).setView(input).setNegativeButton(UiText.get("취소", "Cancel"),null).setPositiveButton(UiText.get("확인", "OK"),(dialog,which) -> {
      String address = input.getText().toString().trim();
      if (!DiscoveryPolicy.local(address)) { notice(UiText.get("같은 네트워크의 IP 주소만 입력해 주세요. 포트는 7000번을 사용합니다.", "Enter an IP address on your local network. Carrot Web uses port 7000.")); return; }
      status.setText(UiText.get("당근웹인지 확인 중…", "Checking for Carrot Web…")); final int token = generation;
      DeviceProbe probe = new DeviceProbe(url -> (HttpURLConnection)url.openConnection()); probes.add(probe);
      workers = Executors.newSingleThreadExecutor(); workers.execute(() -> {
        boolean valid = probe.verify(address);
        ui.post(() -> { if (token != generation) return; if (valid) openDevice(new Device(address,UiText.get("당근파일럿", "CarrotPilot"))); else { stopSearch(); status.setText(UiText.get("해당 주소에서 당근웹을 확인하지 못했어요. IP와 Wi-Fi 연결을 확인해 주세요.", "Carrot Web was not found at that address. Check the IP address and Wi-Fi connection.")); } });
      });
    }).show();
  }
  @android.annotation.SuppressLint("SetJavaScriptEnabled")
  private void openDevice(Device device) {
    String previous = "http://" + preferences.getString("last_address", "") + ":7000";
    ConnectionRecovery.remember(this,previous);
    ConnectionRecovery.remember(this,"http://"+device.address+":7000");
    if (recovery != null) recovery.cancel();
    stopSearch(); currentAddress = device.address; preferences.edit().putString("last_address",device.address).apply(); title.setText(device.name+" · "+device.address);
    content.removeAllViews();
    if (web == null) {
      web = new WebView(this); web.setBackgroundColor(background);
      WebSettings settings = web.getSettings(); settings.setJavaScriptEnabled(true); settings.setDomStorageEnabled(true);
      // Match the existing browser page's CSS sizing. WebView otherwise adds
      // Android font_scale (130% on the test phone) to the page's own typography.
      settings.setTextZoom(100);
      settings.setAllowFileAccess(false); settings.setAllowContentAccess(false); settings.setMixedContentMode(WebSettings.MIXED_CONTENT_NEVER_ALLOW);
      settings.setMediaPlaybackRequiresUserGesture(true); settings.setSupportMultipleWindows(false);
      settings.setUserAgentString(settings.getUserAgentString()+" CarrotWebAndroid/0.1 CarrotAssistPhone/1");
      CookieManager.getInstance().setAcceptThirdPartyCookies(web,false);
      web.setWebChromeClient(new WebChromeClient());
      web.setWebViewClient(new WebViewClient() {
        @Override public boolean shouldOverrideUrlLoading(WebView view,WebResourceRequest request) {
          Uri uri = request.getUrl();
          if ("carrotassist".equals(uri.getScheme())) {
            if (request.isForMainFrame() && view.getUrl() != null && isCurrent(Uri.parse(view.getUrl()))) openPhoneApp(uri, view);
            return true;
          }
          if (isCurrent(uri)) return false;
          if (!request.isForMainFrame() || !request.hasGesture()) return true;
          if ("https".equals(uri.getScheme()) || "http".equals(uri.getScheme())) {
            try { startActivity(new Intent(Intent.ACTION_VIEW,uri).addCategory(Intent.CATEGORY_BROWSABLE)); } catch (Exception e) { notice(UiText.get("이 링크를 열 수 있는 브라우저가 필요합니다.", "A browser is required to open this link.")); }
          }
          return true;
        }
        @Override public void onPageStarted(WebView view,String url,android.graphics.Bitmap icon) {
          pageReady = false; stopPhoneVoice(); cancelVoiceCheck();
          super.onPageStarted(view,url,icon);
        }
        @Override public void onPageFinished(WebView view,String url) {
          if (isCurrent(Uri.parse(url))) {
            pageReady = true;
            scheduleAutoVoice();
          }
        }
        @Override public void onReceivedError(WebView view,WebResourceRequest request,android.webkit.WebResourceError error) {
          if (request.isForMainFrame()) title.setText(UiText.get("기기 연결 끊김 · 왼쪽에서 다시 검색", "Device disconnected · Use search on the left"));
        }
      });
    }
    if (web.getParent()!=null) ((ViewGroup)web.getParent()).removeView(web);
    web.stopLoading(); web.loadUrl("about:blank");
    TextView connecting = text(UiText.get("당근 기기 연결 중…\n기존 ChatGPT 연결을 확인하고 있어요.", "Connecting to Carrot…\nChecking your saved ChatGPT connection."),18);
    connecting.setPadding(dp(22),dp(28),dp(22),dp(16)); content.addView(connecting);
    if (autoVoiceEnabled()) welcome.prepare(device.address);
    recovering = true; final int token = generation;
    recovery = new ConnectionRecovery(this);
    recovery.start("http://"+device.address+":7000",previous,message -> {
      if (token != generation || isFinishing()) return;
      recovering = false;
      content.removeAllViews(); content.addView(web,new LinearLayout.LayoutParams(-1,-1));
      web.loadUrl("http://"+device.address+":7000/");
      if (!message.isEmpty() && !phoneMode()) {
        title.setText(UiText.get("ChatGPT 연결 확인 필요 · ", "Check ChatGPT connection · ")+device.address);
        if (foregroundActivity) notice(message);
      }
    });
  }
  private void openPhoneApp(Uri uri, WebView source) {
    if (!"phone".equals(uri.getHost()) || uri.getPathSegments().size() != 2 || !"naver_map".equals(uri.getPathSegments().get(0))) return;
    String id = uri.getPathSegments().get(1);
    if (!ConnectionPolicy.token(id)) return;
    long until;
    try { until = Long.parseLong(uri.getQueryParameter("expires")); } catch (RuntimeException e) { return; }
    long remaining = until - System.currentTimeMillis();
    if (remaining <= 0 || remaining > 30000 || !source.isShown() || source.getWindowVisibility() != View.VISIBLE) return;
    if (!id.equals(lastPhoneRequest)) {
      lastPhoneRequest = id; lastPhoneResult = "failed";
      Intent launch = getPackageManager().getLaunchIntentForPackage("com.nhn.android.nmap");
      if (launch == null) lastPhoneResult = "not_installed";
      else {
        try { startActivity(launch); lastPhoneResult = "requested"; }
        catch (android.content.ActivityNotFoundException | SecurityException ignored) {}
      }
    }
    source.evaluateJavascript("window.dispatchEvent(new CustomEvent('carrot-phone-result',{detail:{id:" + JSONObject.quote(id)
      + ",status:" + JSONObject.quote(lastPhoneResult) + "}}))", null);
  }
  private boolean isCurrent(Uri uri) { return currentAddress!=null && "http".equals(uri.getScheme()) && currentAddress.equals(uri.getHost()) && uri.getPort()==7000 && uri.getUserInfo()==null; }
  private void invokeAssistant() {
    if (!foregroundActivity) return;
    if (recovering) { notice(UiText.get("당근웹 연결 중이에요. 웹이 열리면 음성을 시작할 수 있어요.", "Connecting to Carrot Web. Voice is available when the page opens.")); return; }
    if (phoneSession) { stopPhoneVoice(); title.setText(currentAddress); return; }
    if (welcome.isGreeting()) { welcome.cancel(); launchVerifiedVoice(); return; }
    startVoice(false);
  }
  private void scheduleAutoVoice() {
    ui.removeCallbacks(tryAutoVoice);
    if (foregroundActivity && pageReady) ui.postDelayed(tryAutoVoice, 450);
  }
  @Override public void onWindowFocusChanged(boolean focus) {
    super.onWindowFocusChanged(focus);
    if (!focus && phoneSession) stopPhoneVoice();
    if (focus && ui != null) scheduleAutoVoice();
  }
  private void maybeStartAutoVoice() {
    if (!foregroundActivity || !hasWindowFocus() || !pageReady || recovering || waitingForPermission) return;
    if (resumePhoneVoice) {
      resumePhoneVoice = false;
      if (permissionDevice == generation && phoneMode()) startVoice(permissionGreeting);
      return;
    }
    if (autoVoiceGeneration != generation && autoVoiceEnabled()) startVoice(true);
  }
  private void startVoice(boolean automatic) {
    if (voiceStarting || !foregroundActivity) return;
    autoVoiceGeneration = generation;
    voiceVerifiedUntil = 0;
    if (currentAddress == null) {
      if (!automatic) {
        if (phoneMode()) notice(UiText.get("당근웹에 먼저 연결해 주세요.", "Connect to Carrot Web first."));
        else openAssistant();
      }
      return;
    }
    String page = web == null ? null : web.getUrl();
    if (page == null || !isCurrent(Uri.parse(page))) {
      if (!automatic) voiceProblem(UiText.get("당근웹이 아직 열리지 않았어요.", "Carrot Web has not opened yet."));
      return;
    }
    if (phoneMode()) { startPhoneVoice(automatic); return; }
    voiceStarting = true;
    voiceFromAuto = automatic;
    if (automatic) welcome.prepare(currentAddress);
    voiceAttempt = 0;
    titleBeforeVoiceCheck = title.getText();
    beginVoiceAttempt();
  }
  private boolean phoneMode() { return preferences.getBoolean("voice_local_first", true); }
  private boolean autoVoiceEnabled() { return preferences.getBoolean("voice_auto_start", phoneMode()); }

  private void stopPhoneVoice() {
    phoneGeneration++; directionCheck.clear(); phoneSearchId = null;
    if (phoneVoice != null) phoneVoice.stop();
    if (phoneSession && welcome != null) welcome.cancel();
    phoneSession = false;
  }

  private boolean localPage() {
    return foregroundActivity && !isFinishing() && hasWindowFocus() && pageReady && web != null && web.getUrl() != null
      && isCurrent(Uri.parse(web.getUrl())) && phoneMode();
  }
  private boolean currentSession(int token) { return phoneSession && token == phoneGeneration && localPage(); }
  private String onDevicePage(String expression) {
    return "(location.origin===" + JSONObject.quote("http://" + currentAddress + ":7000") +
      " && !document.hidden && (" + expression + "))";
  }
  private void startPhoneVoice(boolean automatic) {
    if (!localPage() || phoneSession || waitingForPermission) return;
    final int token = ++phoneGeneration;
    directionCheck.clear(); phoneSearchId = null; phoneSession = true;
    web.evaluateJavascript(onDevicePage("typeof window.CarrotLocalVoice?.start === 'function'"), value -> {
      if (!currentSession(token)) return;
      if (!"true".equals(value)) {
        stopPhoneVoice();
        notice(UiText.get("정차 후 당근 기기를 GitPull하고 재시작해 주세요. 휴대폰 직접 음성 명령에는 새 당근웹이 필요해요.",
                          "Update and restart the parked Carrot device. Direct voice commands need the updated Carrot Web."));
        return;
      }
      if (checkSelfPermission(android.Manifest.permission.RECORD_AUDIO) != android.content.pm.PackageManager.PERMISSION_GRANTED) {
        phoneSession = false;
        waitingForPermission = true; permissionDevice = generation; permissionGreeting = automatic;
        requestPermissions(new String[]{android.Manifest.permission.RECORD_AUDIO}, PHONE_MIC);
        return;
      }
      welcome.prepare(currentAddress);
      Runnable listen = () -> { if (currentSession(token)) phoneVoice.listen(); };
      if (automatic) welcome.greet(listen); else listen.run();
    });
  }

  @Override public void onRequestPermissionsResult(int request, String[] permissions, int[] grants) {
    super.onRequestPermissionsResult(request, permissions, grants);
    if (request != PHONE_MIC) return;
    waitingForPermission = false;
    if (grants.length > 0 && grants[0] == android.content.pm.PackageManager.PERMISSION_GRANTED && permissionDevice == generation) {
      resumePhoneVoice = true; scheduleAutoVoice();
    } else {
      resumePhoneVoice = false;
      title.setText(UiText.get("직접 음성 명령에는 마이크 권한이 필요해요.", "Microphone permission is required for direct voice commands."));
    }
  }

  private void handleSpokenSetting(SpokenSetting setting) {
    final int token = phoneGeneration;
    directionCheck.clear();
    if (!currentSession(token)) { stopPhoneVoice(); return; }
    if (setting == null) {
      replyAndListen(token, UiText.get("설정 이름과 숫자를 다시 말해 주세요.", "Please repeat the setting name and number."));
      return;
    }
    if (setting.position == 0) {
      startSettingAction("setValue", JSONObject.quote(setting.name) + "," + setting.value);
    } else if (phoneSearchId == null) {
      replyAndListen(token, UiText.get("먼저 설정을 찾아 달라고 말해 주세요.", "Search for settings first."));
    } else {
      startSettingAction("setResultValue", JSONObject.quote(phoneSearchId) + "," + setting.position + "," +
        setting.value + "," + JSONObject.quote(UiText.get("ko", "en")));
    }
  }
  private void startSettingAction(String method, String arguments) {
    final int token = phoneGeneration;
    directionCheck.clear();
    if (!currentSession(token)) { stopPhoneVoice(); return; }
    String id = ConnectionPolicy.nonce();
    long expires = System.currentTimeMillis() + 5000;
    if ("findSettings".equals(method)) phoneSearchId = null;
    String expression = "typeof window.CarrotLocalVoice?." + method + " !== 'function' ? 'update' : window.CarrotLocalVoice." + method + "(" +
      arguments + "," + JSONObject.quote(id) + "," + expires + ")";
    web.evaluateJavascript(onDevicePage(expression), raw -> {
      if (!currentSession(token)) return;
      if ("\"update\"".equals(raw)) {
        replyAndListen(token, UiText.get("정차 후 당근 기기를 깃풀하고 재시작해 주세요.", "Update and restart the parked Carrot device."));
      } else if ("true".equals(raw)) {
        checkPhoneResult(token, id, null, SystemClock.elapsedRealtime() + 12000);
      } else phoneResult(token, null, "unverified");
    });
  }
  private void handlePhoneCommand(LocalCommand command) {
    final int token = phoneGeneration;
    if (!currentSession(token)) { stopPhoneVoice(); return; }
    dispatchPhoneCommand(token, directionCheck.resolve(command, SystemClock.elapsedRealtime()));
  }
  private void dispatchPhoneCommand(int token, LocalCommand command) {
    switch (command) {
      case CLARIFY_LEFT: case CLARIFY_RIGHT:
        boolean left = command == LocalCommand.CLARIFY_LEFT;
        String prompt = left ? UiText.get("왼쪽이면 왼쪽 확인이라고 말해 주세요.", "Say confirm left if you mean left.")
                             : UiText.get("오른쪽이면 오른쪽 확인이라고 말해 주세요.", "Say confirm right if you mean right.");
        title.setText(prompt);
        welcome.say(prompt, () -> {
          if (currentSession(token)) { directionCheck.ask(command, SystemClock.elapsedRealtime()); phoneVoice.listen(); }
        });
        break;
      case LEFT: case RIGHT: case BLINKER_SETTINGS:
        String action = command == LocalCommand.LEFT ? "lane_left" : command == LocalCommand.RIGHT ? "lane_right" : "blinker_settings";
        String id = ConnectionPolicy.nonce();
        long expires = System.currentTimeMillis() + 3500;
        long deadline = SystemClock.elapsedRealtime() + 4300;
        web.evaluateJavascript(onDevicePage("window.CarrotLocalVoice.start(" + JSONObject.quote(action) + "," +
          JSONObject.quote(id) + "," + expires + ")"), value -> {
          if (!currentSession(token)) return;
          if ("true".equals(value)) checkPhoneResult(token, id, command, deadline);
          else phoneResult(token, command, "unverified");
        });
        break;
      case QUESTION:
        String handoff = UiText.get("음성비서를 열게요. 질문을 다시 말해 주세요.", "Opening the assistant. Please repeat your question there.");
        title.setText(handoff);
        welcome.say(handoff, () -> { if (currentSession(token)) { stopPhoneVoice(); openAssistant(); } });
        break;
      case STOP: stopPhoneVoice(); title.setText(currentAddress); break;
      default:
        replyAndListen(token, command == LocalCommand.UNCERTAIN
          ? UiText.get("방향을 확실히 알아듣지 못했어요. 요청을 보내지 않았어요.", "Direction uncertain. No request sent.")
          : UiText.get("명령을 알아듣지 못했어요.", "Command not recognized."));
    }
  }
  private void checkPhoneResult(int token, String id, LocalCommand command, long deadline) {
    if (!currentSession(token)) return;
    if (SystemClock.elapsedRealtime() >= deadline) { phoneResult(token, command, "unverified"); return; }
    web.evaluateJavascript(onDevicePage("window.CarrotLocalVoice.result(" + JSONObject.quote(id) + ")"), raw -> {
      if (!currentSession(token)) return;
      if ("\"\"".equals(raw)) { ui.postDelayed(() -> checkPhoneResult(token,id,command,deadline),120); return; }
      String status;
      try { Object v = new org.json.JSONTokener(raw).nextValue(); status = v instanceof String ? (String)v : "unverified"; }
      catch (Exception e) { status = "unverified"; }
      if (command == null) {
        try {
          Object value = new org.json.JSONTokener(raw).nextValue();
          if (value instanceof JSONObject) {
            JSONObject result = (JSONObject)value;
            String name = result.optString("label");
            switch (result.optString("status")) {
              case "found":
                int count = result.getInt("total");
                phoneSearchId = count > 0 ? id : null;
                replyAndListen(token, count == 0 ? UiText.get("관련 설정을 찾지 못했어요.", "No matching settings.")
                  : UiText.get(count + "개 찾았어요. 첫 번째는 " + result.optString("firstKo") + "예요.", "Found " + count + ". First is " + result.optString("firstEn") + "."));
                return;
              case "applied": replyAndListen(token, UiText.get(name + ", " + result.getInt("current") + "으로 설정했어요.", name + " set to " + result.getInt("current") + ".")); return;
              case "range": replyAndListen(token, UiText.get("허용 범위는 " + result.getInt("min") + "에서 " + result.getInt("max") + "까지예요.", "Allowed range is " + result.getInt("min") + " to " + result.getInt("max") + ".")); return;
              case "unchanged": replyAndListen(token, UiText.get(name + ", 이미 " + result.getInt("current") + "이에요.", name + " is already " + result.getInt("current") + ".")); return;
              case "permission": replyAndListen(token, UiText.get("정차 후 당근웹 도구에서 설정값 조회·변경을 허용해 주세요.", "While parked, allow setting reads and changes in Carrot Web Tools.")); return;
              case "no_result": replyAndListen(token, UiText.get("지금 검색 목록에 그 항목이 없어요.", "That item is not in the current search results.")); return;
              case "unknown": replyAndListen(token, UiText.get("설정 하나를 찾을 수 없어요. 설정 이름을 다시 말해 주세요.", "Please say the name of one setting.")); return;
            }
          }
        } catch (org.json.JSONException ignored) {}
      }
      phoneResult(token, command, status);
    });
  }
  private void phoneResult(int token, LocalCommand command, String status) {
    String output;
    switch (status) {
      case "opened": output = UiText.get("깜박이 설정을 열었어요.", "Opened the turn signal setting."); break;
      case "waiting": output = command == LocalCommand.LEFT
        ? UiText.get("왼쪽 요청을 접수했어요.", "Left request received.")
        : UiText.get("오른쪽 요청을 접수했어요.", "Right request received."); break;
      case "running": output = UiText.get("차로 변경 중이에요.", "Lane change in progress."); break;
      case "completed": output = UiText.get("차로 변경을 완료했어요.", "Lane change completed."); break;
      case "permission": output = UiText.get("정차 후 당근웹 도구에서 운전 명령을 허용해 주세요.", "While parked, allow driving commands in Carrot Web Tools."); break;
      case "duplicate": output = UiText.get("직전 요청이 있어 다시 보내지 않았어요.", "Recent request. No repeat sent."); break;
      case "rejected": case "expired": case "interrupted": case "cancelled":
        output = UiText.get("차로변경 요청이 종료됐어요.", "The lane change request ended."); break;
      case "unavailable": output = UiText.get("이 기기에서 설정을 찾지 못했어요.", "Setting unavailable on this device."); break;
      default: output = UiText.get("결과를 확인하지 못했어요.", "Result unavailable.");
    }
    replyAndListen(token, output);
  }
  private void replyAndListen(int token, String message) {
    if (!currentSession(token)) return;
    title.setText(message);
    welcome.say(message, () -> { if (currentSession(token)) phoneVoice.listen(); });
  }
  private void beginVoiceAttempt() {
    if (!voiceStarting || !foregroundActivity) return;
    voiceAttempt++;
    ConnectionRecovery check = new ConnectionRecovery(this);
    voiceCheck = check;
    title.setText(UiText.get("당근웹 연결 확인 중", "Checking Carrot Web") + " (" + voiceAttempt + "/3)");
    ui.postDelayed(voiceTimeout, 4000);
    check.checkForVoice(web, "http://" + currentAddress + ":7000", message -> {
      if (voiceCheck == check) finishVoiceCheck(message);
    });
  }
  private void stopVoiceAttempt() {
    ui.removeCallbacks(voiceTimeout);
    if (voiceCheck != null) { voiceCheck.cancel(); voiceCheck = null; }
  }
  private void cancelVoiceCheck() {
    ui.removeCallbacks(retryVoice);
    stopVoiceAttempt();
    if (voiceStarting) title.setText(titleBeforeVoiceCheck);
    voiceStarting = false;
    titleBeforeVoiceCheck = null;
  }
  private void finishVoiceCheck(String message) {
    if (!voiceStarting || voiceCheck == null) return;
    stopVoiceAttempt();
    if (!foregroundActivity || isFinishing()) { cancelVoiceCheck(); return; }
    if (ConnectionRecovery.temporaryVoiceFailure(message) && voiceAttempt < 3) {
      ui.postDelayed(retryVoice, voiceAttempt == 1 ? 600 : 1500);
      return;
    }
    boolean automatic = voiceFromAuto;
    cancelVoiceCheck();
    if (message.isEmpty()) {
      if (automatic) welcome.greet(this::launchVerifiedVoice);
      else launchVerifiedVoice();
    } else if (automatic) {
      android.widget.Toast.makeText(this,message,android.widget.Toast.LENGTH_SHORT).show();
    } else voiceProblem(message);
  }
  private void launchVerifiedVoice() {
    if (!foregroundActivity || isFinishing()) return;
    voiceVerifiedUntil = SystemClock.elapsedRealtime() + 5000;
    openAssistant();
  }
  @Override public void onProvideAssistContent(AssistContent content) {
    super.onProvideAssistContent(content);
    if (SystemClock.elapsedRealtime() >= voiceVerifiedUntil) return;
    // Optional Android assistant context. Share neither credentials nor vehicle or account identifiers.
    content.setStructuredData("{\"@context\":\"https://schema.org\",\"@type\":\"WebPage\","
      + "\"name\":\"CarrotAssist\",\"description\":\"Carrot Web was online and account-linked at voice startup.\"}");
  }
  private void voiceProblem(String message) {
    new AlertDialog.Builder(this).setMessage(message)
      .setNegativeButton(UiText.get("닫기", "Close"), null)
      .setNeutralButton(UiText.get("다시 확인", "Check again"), (d,w) -> invokeAssistant())
      .setPositiveButton(UiText.get("음성비서만 열기", "Open assistant only"), (d,w) -> openAssistant()).show();
  }
  private void openAssistant() {
    if (!foregroundActivity) return;
    try { if (showAssist(new Bundle())) return; } catch (RuntimeException ignored) {}
    new AlertDialog.Builder(this).setTitle(UiText.get("기본 음성비서 설정", "Default voice assistant")).setMessage(UiText.get("Android 기본 디지털 어시스턴트를 선택해 주세요. ChatGPT 또는 Gemini 등 선택한 앱을 호출합니다. 당근 설정 검색·화면 열기는 현재 ChatGPT 연결로 시험했으며, 음성창의 모양은 선택한 앱에 따라 달라집니다.", "Choose your default Android digital assistant. The microphone opens your selected app, such as ChatGPT or Gemini. Carrot setting search and screen navigation have been tested with ChatGPT. The voice window depends on the assistant app."))
      .setNegativeButton(UiText.get("닫기", "Close"),null).setPositiveButton(UiText.get("설정 열기", "Open settings"),(d,w) -> { try { startActivity(new Intent(Settings.ACTION_VOICE_INPUT_SETTINGS)); } catch (Exception e) { notice(UiText.get("휴대폰 설정에서 기본 디지털 어시스턴트를 선택해 주세요.", "Choose the default digital assistant in your phone settings.")); } }).show();
  }
  private void notice(String message) { new AlertDialog.Builder(this).setMessage(message).setPositiveButton(UiText.get("확인", "OK"),null).show(); }
  @Override protected void onResume() {
    super.onResume(); foregroundActivity = true; if (web!=null) web.onResume(); if (updater!=null) updater.resume();
    if (checkAfterBackground && currentAddress!=null && !recovering) {
      final String address=currentAddress; final int token=generation;
      final DeviceProbe probe=new DeviceProbe(url -> (HttpURLConnection)url.openConnection()); resumeProbe=probe;
      new Thread(() -> {
        boolean valid=probe.verify(address); probe.close();
        ui.post(() -> { boolean current=resumeProbe==probe; if (current) resumeProbe=null;
          if (current && !valid && foregroundActivity && token==generation && address.equals(currentAddress)) startSearch(); });
      },"carrot-resume-probe").start();
    }
    checkAfterBackground=false;
    scheduleAutoVoice();
  }
  @Override protected void onPause() { foregroundActivity = false; stopPhoneVoice(); welcome.cancel(); ui.removeCallbacks(tryAutoVoice); cancelVoiceCheck(); super.onPause(); }
  @Override protected void onStop() { checkAfterBackground=true; if (scanning) { stopSearch(); if (status!=null) status.setText(UiText.get("검색이 중단됐어요. 다시 검색을 눌러 주세요.", "Search paused. Tap Search again.")); } super.onStop(); }
  @Override public void onBackPressed() { if (web!=null && currentAddress!=null && web.canGoBack()) web.goBack(); else if (currentAddress!=null) startSearch(); else super.onBackPressed(); }
  @Override protected void onDestroy() { if (recovery!=null) recovery.cancel(); stopSearch(); ui.removeCallbacksAndMessages(null); if (updater!=null) updater.close(); if (web!=null) { web.destroy(); web=null; } super.onDestroy(); }
}
