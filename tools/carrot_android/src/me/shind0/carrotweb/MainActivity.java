package me.shind0.carrotweb;

import android.app.Activity;
import android.app.AlertDialog;
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
import android.widget.ImageButton;
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
    updater = new AppUpdater(this, preferences);
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
    Button search = button("⌕", v -> startSearch()); search.setContentDescription("당근 기기 찾기");
    bar.addView(search,new LinearLayout.LayoutParams(dp(52),dp(48)));
    title = text("CarrotAssist",18); title.setSingleLine(true); title.setEllipsize(android.text.TextUtils.TruncateAt.END);
    title.setOnClickListener(v -> updater.info()); title.setContentDescription("기기 이름 · 앱 정보와 업데이트 확인");
    bar.addView(title,new LinearLayout.LayoutParams(0,dp(48),1)); title.setGravity(Gravity.CENTER_VERTICAL);
    ImageButton assistant = new ImageButton(this);
    assistant.setImageResource(android.R.drawable.ic_btn_speak_now); assistant.setColorFilter(accent);
    assistant.setBackgroundColor(background); assistant.setOnClickListener(v -> invokeAssistant());
    assistant.setContentDescription("음성비서 호출");
    bar.addView(assistant,new LinearLayout.LayoutParams(dp(52),dp(48)));
    root.addView(bar); content = column(); root.addView(content,new LinearLayout.LayoutParams(-1,0,1)); setContentView(root);
    startSearch();
    updater.check(false);
  }
  private int dp(int value) { return Math.round(value * getResources().getDisplayMetrics().density); }
  private LinearLayout column() { LinearLayout v = new LinearLayout(this); v.setOrientation(LinearLayout.VERTICAL); return v; }
  private TextView text(String value,int size) { TextView v = new TextView(this); v.setText(value); v.setTextSize(size); v.setTextColor(foreground); return v; }
  private Button button(String label,View.OnClickListener listener) { Button b = new Button(this); b.setText(label); b.setAllCaps(false); b.setTextColor(accent); b.setOnClickListener(listener); return b; }
  private void stopSearch() {
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
    content.removeAllViews(); title.setText("당근 기기 찾기");
    LinearLayout panel = column(); panel.setPadding(dp(22),dp(28),dp(22),dp(16));
    panel.addView(text("당근을 찾고 있어요",26));
    TextView help = text("같은 Wi-Fi 또는 휴대폰 핫스팟에 연결해 주세요.\n5초 동안 한 대만 찾으면 자동으로 열립니다.",16); help.setPadding(0,dp(14),0,dp(16)); panel.addView(help);
    status = text("검색 준비 중…",16); panel.addView(status);
    progress = new ProgressBar(this,null,android.R.attr.progressBarStyleHorizontal); progress.setIndeterminate(true); panel.addView(progress);
    devices = column(); ScrollView list = new ScrollView(this); list.addView(devices); panel.addView(list,new LinearLayout.LayoutParams(-1,0,1));
    LinearLayout actions = new LinearLayout(this);
    actions.addView(button("다시 검색",v -> startSearch()),new LinearLayout.LayoutParams(0,dp(52),1));
    actions.addView(button("주소 직접 입력",v -> manualAddress()),new LinearLayout.LayoutParams(0,dp(52),1)); panel.addView(actions);
    panel.addView(button("검색 중단",v -> { stopSearch(); status.setText("검색을 중단했습니다. 발견된 기기를 선택할 수 있어요."); progress.setVisibility(View.GONE); }));
    content.addView(panel,new LinearLayout.LayoutParams(-1,-1));
    List<Target> targets = targets();
    if (targets.isEmpty()) { status.setText("검색할 Wi-Fi를 찾지 못했어요. 연결을 확인하거나 주소를 입력해 주세요."); progress.setVisibility(View.GONE); return; }
    started = SystemClock.elapsedRealtime(); scanning = true; final int token = generation;
    workers = Executors.newFixedThreadPool(32); AtomicInteger cursor = new AtomicInteger();
    for (int w=0;w<32;w++) workers.execute(() -> {
      while (token == generation && SystemClock.elapsedRealtime()-started < DiscoveryPolicy.LIMIT_MS) {
        int index = cursor.getAndIncrement();
        if (index >= targets.size()) break;
        Target target = targets.get(index);
        if (target.probe.verify(target.address)) {
          String name = "당근파일럿";
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
            if (target.probe.verify(target.address)) ui.post(() -> { if (token == generation && scanning) addDevice(new Device(target.address,"당근파일럿")); });
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
        stopSearch(); status.setText(found.isEmpty() ? "1분 동안 기기를 찾지 못했어요. 같은 Wi-Fi인지 확인한 뒤 다시 검색해 주세요." : "검색을 마쳤어요. 연결할 기기를 선택해 주세요."); return;
      }
      if (found.size() > 1) { choosing = true; status.setText(found.size()+"대를 찾았어요. 연결할 기기를 선택해 주세요. · "+Math.max(0,(60-elapsed/1000))+"초 남음"); }
      else if (elapsed < DiscoveryPolicy.QUICK_MS) status.setText("빠른 검색 · "+Math.max(1,(5-elapsed/1000))+"초 · 발견 "+found.size()+"대");
      else status.setText("계속 검색 중 · "+Math.max(0,(60-elapsed/1000))+"초 남음");
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
    new AlertDialog.Builder(this).setTitle("당근 기기 IP 주소").setView(input).setNegativeButton("취소",null).setPositiveButton("확인",(dialog,which) -> {
      String address = input.getText().toString().trim();
      if (!DiscoveryPolicy.local(address)) { notice("같은 네트워크의 IP 주소만 입력해 주세요. 포트는 7000번을 사용합니다."); return; }
      status.setText("당근웹인지 확인 중…"); final int token = generation;
      DeviceProbe probe = new DeviceProbe(url -> (HttpURLConnection)url.openConnection()); probes.add(probe);
      workers = Executors.newSingleThreadExecutor(); workers.execute(() -> {
        boolean valid = probe.verify(address);
        ui.post(() -> { if (token != generation) return; if (valid) openDevice(new Device(address,"당근파일럿")); else { stopSearch(); status.setText("해당 주소에서 당근웹을 확인하지 못했어요. IP와 Wi-Fi 연결을 확인해 주세요."); } });
      });
    }).show();
  }
  @android.annotation.SuppressLint("SetJavaScriptEnabled")
  private void openDevice(Device device) {
    String previous = "http://" + preferences.getString("last_address", "") + ":7000";
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
      settings.setUserAgentString(settings.getUserAgentString()+" CarrotWebAndroid/0.1");
      CookieManager.getInstance().setAcceptThirdPartyCookies(web,false);
      web.setWebChromeClient(new WebChromeClient());
      web.setWebViewClient(new WebViewClient() {
        @Override public boolean shouldOverrideUrlLoading(WebView view,WebResourceRequest request) {
          Uri uri = request.getUrl();
          if (isCurrent(uri)) return false;
          if (!request.isForMainFrame() || !request.hasGesture()) return true;
          if ("https".equals(uri.getScheme()) || "http".equals(uri.getScheme())) {
            try { startActivity(new Intent(Intent.ACTION_VIEW,uri).addCategory(Intent.CATEGORY_BROWSABLE)); } catch (Exception e) { notice("이 링크를 열 수 있는 브라우저가 필요합니다."); }
          }
          return true;
        }
        @Override public void onReceivedError(WebView view,WebResourceRequest request,android.webkit.WebResourceError error) {
          if (request.isForMainFrame()) title.setText("기기 연결 끊김 · 왼쪽에서 다시 검색");
        }
      });
    }
    if (web.getParent()!=null) ((ViewGroup)web.getParent()).removeView(web);
    web.stopLoading(); web.loadUrl("about:blank");
    TextView connecting = text("당근 기기 연결 중…\n기존 ChatGPT 연결을 확인하고 있어요.",18);
    connecting.setPadding(dp(22),dp(28),dp(22),dp(16)); content.addView(connecting);
    recovering = true; final int token = generation;
    recovery = new ConnectionRecovery(this);
    recovery.start("http://"+device.address+":7000",previous,message -> {
      if (token != generation || isFinishing()) return;
      recovering = false;
      content.removeAllViews(); content.addView(web,new LinearLayout.LayoutParams(-1,-1));
      web.loadUrl("http://"+device.address+":7000/");
      if (!message.isEmpty()) {
        title.setText("ChatGPT 연결 확인 필요 · "+device.address);
        if (foregroundActivity) notice(message);
      }
    });
  }
  private boolean isCurrent(Uri uri) { return currentAddress!=null && "http".equals(uri.getScheme()) && currentAddress.equals(uri.getHost()) && uri.getPort()==7000 && uri.getUserInfo()==null; }
  private void invokeAssistant() {
    if (!foregroundActivity) return;
    if (recovering) { notice("기존 ChatGPT 연결을 확인하고 있어요. 당근웹이 열린 뒤 마이크를 눌러 주세요."); return; }
    try { if (showAssist(new Bundle())) return; } catch (RuntimeException ignored) {}
    new AlertDialog.Builder(this).setTitle("기본 음성비서 설정").setMessage("Android 기본 디지털 어시스턴트를 선택해 주세요. ChatGPT 또는 Gemini 등 선택한 앱을 호출합니다. 당근 설정 검색·화면 열기는 현재 ChatGPT 연결로 시험했으며, 음성창의 모양은 선택한 앱에 따라 달라집니다.")
      .setNegativeButton("닫기",null).setPositiveButton("설정 열기",(d,w) -> { try { startActivity(new Intent(Settings.ACTION_VOICE_INPUT_SETTINGS)); } catch (Exception e) { notice("휴대폰 설정에서 기본 디지털 어시스턴트를 선택해 주세요."); } }).show();
  }
  private void notice(String message) { new AlertDialog.Builder(this).setMessage(message).setPositiveButton("확인",null).show(); }
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
  }
  @Override protected void onPause() { foregroundActivity = false; super.onPause(); }
  @Override protected void onStop() { checkAfterBackground=true; if (scanning) { stopSearch(); if (status!=null) status.setText("검색이 중단됐어요. 다시 검색을 눌러 주세요."); } super.onStop(); }
  @Override public void onBackPressed() { if (web!=null && currentAddress!=null && web.canGoBack()) web.goBack(); else if (currentAddress!=null) startSearch(); else super.onBackPressed(); }
  @Override protected void onDestroy() { if (recovery!=null) recovery.cancel(); stopSearch(); ui.removeCallbacksAndMessages(null); if (updater!=null) updater.close(); if (web!=null) { web.destroy(); web=null; } super.onDestroy(); }
}
