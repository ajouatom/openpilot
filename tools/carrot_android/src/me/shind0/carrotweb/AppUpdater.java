package me.shind0.carrotweb;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.content.pm.Signature;
import android.net.Uri;
import android.os.Build;
import android.provider.Settings;
import android.widget.Toast;
import org.json.JSONObject;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.security.MessageDigest;
import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public final class AppUpdater {
  private final Activity activity;
  private final SharedPreferences preferences;
  private final ExecutorService worker = Executors.newSingleThreadExecutor();
  private boolean busy, awaitingPermission, destroyed;
  private JSONObject available;
  private File pending;
  public AppUpdater(Activity activity, SharedPreferences preferences) { this.activity=activity; this.preferences=preferences; }
  private void toast(String message) { if (!destroyed) Toast.makeText(activity,message,Toast.LENGTH_LONG).show(); }
  private PackageInfo installed() throws Exception { return activity.getPackageManager().getPackageInfo(activity.getPackageName(), flags()); }
  private static int flags() { return Build.VERSION.SDK_INT >= 28 ? PackageManager.GET_SIGNING_CERTIFICATES : PackageManager.GET_SIGNATURES; }
  private static long version(PackageInfo info) { return Build.VERSION.SDK_INT >= 28 ? info.getLongVersionCode() : info.versionCode; }
  private static Signature[] signatures(PackageInfo info) {
    return Build.VERSION.SDK_INT >= 28 ? (info.signingInfo == null ? null : info.signingInfo.getApkContentsSigners()) : info.signatures;
  }
  public void info() {
    String label = "CarrotAssist";
    try { label += " "+installed().versionName; } catch (Exception ignored) {}
    android.widget.CheckBox autoVoice = new android.widget.CheckBox(activity);
    autoVoice.setText(UiText.get("당근웹 연결 시 음성조수 자동 실행", "Start the voice assistant when Carrot Web connects"));
    autoVoice.setChecked(preferences.getBoolean("voice_auto_start",preferences.getBoolean("voice_local_first", true)));
    autoVoice.setOnCheckedChangeListener((v,on) -> preferences.edit().putBoolean("voice_auto_start",on).apply());
    android.widget.CheckBox direct = new android.widget.CheckBox(activity);
    direct.setText(UiText.get("폰에서 음성 명령 먼저 듣기", "Listen for commands on this phone first"));
    direct.setChecked(preferences.getBoolean("voice_local_first", true));
    direct.setOnCheckedChangeListener((v,on) -> preferences.edit().putBoolean("voice_local_first", on)
      .putBoolean("voice_auto_start", autoVoice.isChecked()).apply());
    android.widget.LinearLayout options = new android.widget.LinearLayout(activity);
    options.setOrientation(android.widget.LinearLayout.VERTICAL);
    options.addView(autoVoice); options.addView(direct);
    new AlertDialog.Builder(activity).setTitle(label).setView(options).setMessage(UiText.get("폰 직접 듣기는 차로변경과 깜박이 설정 열기를 처리합니다. 질문은 기본 음성비서에서 다시 말해 주세요. 직접 듣기를 끄면 미니창으로 시작합니다.\n\n업데이트 설치는 주차 후 진행하세요.", "Direct listening handles lane requests and turn signal settings. Repeat questions in the default assistant. Turn off direct listening to start in the assistant overlay.\n\nInstall updates while parked."))
      .setPositiveButton(available == null ? UiText.get("업데이트 확인", "Check for updates") : UiText.get("새 버전 설치", "Install update"), (d,w)->{ if (available==null) check(true); else offer(); })
      .setNeutralButton(UiText.get("ChatGPT 맞춤 지침 복사", "Copy ChatGPT instructions"), (d,w)->copyChatGPTInstructions())
      .setNegativeButton(UiText.get("닫기", "Close"),null).show();
  }
  private void copyChatGPTInstructions() {
    String prompt = UiText.get(
      "운전이나 차량 설정에 관한 대화에서 내가 말하는 ‘당근’은 당근파일럿(CarrotAssist)을 뜻해. 당근파일럿 설정을 찾거나 바꿔 달라고 하거나, ‘왼쪽으로 차로 변경해’처럼 지금 실행할 운전 명령을 내리면 연결된 CarrotAssist 도구를 사용해 줘. 도구가 알려준 결과를 짧게 말해 줘.",
      "When we talk about driving or vehicle settings, Carrot means CarrotPilot (CarrotAssist). When I ask you to find or change a CarrotPilot setting, or give a driving command such as “change lanes left”, use the connected CarrotAssist tool. Briefly report the result returned by the tool.");
    boolean copied = false;
    try {
      ClipboardManager clipboard = activity.getSystemService(ClipboardManager.class);
      if (clipboard != null) {
        clipboard.setPrimaryClip(ClipData.newPlainText("CarrotAssist", prompt));
        copied = true;
      }
    } catch (SecurityException ignored) {
      // The selectable text below also works on phones that restrict clipboard access.
    }
    String steps = UiText.get(
      "1. ChatGPT 앱에서 메뉴를 열고 프로필을 누르세요.\n2. 설정 화면을 아래로 스크롤해 ‘ChatGPT 맞춤 설정’을 누른 뒤 ‘성격’을 여세요. 화면 아래에 ‘사용자 지정 지침’ 입력칸이 있어요.\n3. 기존 지침이 있으면 그 아래에 붙여넣고 오른쪽 위 ‘저장’을 누르세요.\n\n새 대화에서 “가속도 설정을 찾아줘”로 확인해 보세요.",
      "1. Open the menu in ChatGPT and tap your profile.\n2. In Settings, scroll to Customize ChatGPT and open Personality. The Custom instructions field is below the style controls.\n3. Paste below any existing instructions and tap Save at the top right.\n\nTry “Find acceleration settings” in a new chat.");
    AlertDialog guide = new AlertDialog.Builder(activity)
      .setTitle(copied ? UiText.get("맞춤 지침을 복사했어요", "Custom instructions copied")
                       : UiText.get("아래 문구를 길게 눌러 복사하세요", "Press and hold the text to copy"))
      .setMessage(steps + "\n\n“" + prompt + "”")
      .setPositiveButton(UiText.get("확인", "Done"), null).show();
    android.widget.TextView message = guide.findViewById(android.R.id.message);
    if (message != null) message.setTextIsSelectable(true);
  }
  public void check(boolean manual) {
    if (busy || destroyed) return;
    long elapsed = System.currentTimeMillis()-preferences.getLong("update_checked",0);
    if (!manual && elapsed>=0 && elapsed<6*3600000L) return;
    busy=true;
    worker.execute(()->{
      JSONObject next=null; String error=null;
      try {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream(); transfer(UpdatePolicy.MANIFEST,bytes,16384);
        JSONObject info = new JSONObject(bytes.toString("UTF-8"));
        if (!activity.getPackageName().equals(info.getString("package"))) throw new Exception("Wrong package");
        if (info.getLong("version_code") > version(installed())) {
          if (!UpdatePolicy.valid(version(installed()),info.getLong("version_code"),info.getString("version"),info.getString("apk_url"),info.getString("sha256"))) throw new Exception("Invalid update metadata");
          next=info;
        }
        preferences.edit().putLong("update_checked",System.currentTimeMillis()).apply();
      } catch (Exception failure) { error=UiText.get("업데이트 정보를 확인하지 못했어요. 인터넷 연결을 확인해 주세요.", "Could not check for updates. Check your Internet connection."); }
      final JSONObject result=next; final String message=error;
      activity.runOnUiThread(()->{
        busy=false; if (destroyed) return; available=result;
        if (message!=null) { if (manual) toast(message); }
        else if (available!=null) { if (manual) offer(); else toast(UiText.get("CarrotAssist 새 버전이 있어요. 상단 기기 이름을 눌러 업데이트하세요.", "A CarrotAssist update is available. Tap the device name at the top to update.")); }
        else if (manual) toast(UiText.get("현재 최신 버전입니다.", "You are up to date."));
      });
    });
  }
  private void offer() {
    if (available==null || busy || destroyed) return;
    new AlertDialog.Builder(activity).setTitle("CarrotAssist "+available.optString("version"))
      .setMessage(UiText.get("새 버전을 다운로드하고 안드로이드 설치 화면을 열까요? 설치 중 앱이 종료될 수 있으니 주차 후 진행하세요.", "Download the update and open the Android installer? The app may close during installation. Please park first."))
      .setPositiveButton(UiText.get("다운로드", "Download"),(d,w)->download()).setNegativeButton(UiText.get("나중에", "Later"),null).show();
  }
  private void download() {
    if (available==null || busy) return;
    busy=true; final JSONObject info=available; toast(UiText.get("업데이트를 다운로드하고 확인하고 있어요.", "Downloading and verifying the update."));
    worker.execute(()->{
      File part=new File(activity.getCacheDir(),"update.part"), apk=new File(activity.getCacheDir(),"update.apk");
      String error=null;
      try {
        try (FileOutputStream output=new FileOutputStream(part)) { transfer(info.getString("apk_url"),output,32*1024*1024); }
        verify(part,info);
        if (!part.renameTo(apk)) throw new Exception("Cannot prepare installer");
      } catch (Exception failure) { part.delete(); error=UiText.get("업데이트 검증에 실패했어요. 설치하지 않았습니다. 인터넷이나 배포 파일을 확인해 주세요.", "Update verification failed. Nothing was installed. Check your Internet connection or the release file."); }
      final String message=error;
      activity.runOnUiThread(()->{
        busy=false; if (destroyed) return;
        if (message!=null) toast(message); else { pending=apk; install(); }
      });
    });
  }
  private void verify(File apk, JSONObject info) throws Exception {
    MessageDigest digest=MessageDigest.getInstance("SHA-256");
    try (InputStream input=new FileInputStream(apk)) { byte[] bytes=new byte[16384]; int n; while ((n=input.read(bytes))!=-1) digest.update(bytes,0,n); }
    StringBuilder hash=new StringBuilder(); for(byte b:digest.digest()) hash.append(String.format("%02x",b & 255));
    if (!hash.toString().equalsIgnoreCase(info.getString("sha256"))) throw new Exception("Checksum mismatch");
    PackageInfo current=installed(), next=activity.getPackageManager().getPackageArchiveInfo(apk.getPath(),flags());
    if (next==null || !current.packageName.equals(next.packageName) || version(next)!=info.getLong("version_code") || version(next)<=version(current)) throw new Exception("Package/version mismatch");
    Signature[] a=signatures(current), b=signatures(next);
    if (a==null || b==null || a.length!=1 || b.length!=1 || !Arrays.equals(a[0].toByteArray(),b[0].toByteArray())) throw new Exception("Signing identity mismatch");
  }
  private void install() {
    if (pending==null || destroyed) return;
    try {
      if (!activity.getPackageManager().canRequestPackageInstalls()) {
        awaitingPermission=true;
        activity.startActivity(new Intent(Settings.ACTION_MANAGE_UNKNOWN_APP_SOURCES,Uri.parse("package:"+activity.getPackageName())));
        return;
      }
      verify(pending,available);
      Intent intent=new Intent(Intent.ACTION_VIEW).setDataAndType(Uri.parse("content://"+activity.getPackageName()+".updates/update.apk"),"application/vnd.android.package-archive")
        .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
      activity.startActivity(intent); pending=null;
    } catch (Exception error) { pending=null; toast(UiText.get("설치 화면을 열지 못했어요. 앱 정보에서 다시 확인해 주세요.", "Could not open the installer. Try again from app information.")); }
  }
  public void resume() {
    if (awaitingPermission) { awaitingPermission=false; if (activity.getPackageManager().canRequestPackageInstalls()) install(); else toast(UiText.get("설치 권한이 허용되지 않아 업데이트를 중단했어요.", "Update stopped because installation permission was not allowed.")); }
  }
  public void close() { destroyed=true; worker.shutdownNow(); }
  private static void transfer(String address, java.io.OutputStream output, int maximum) throws Exception {
    for(int redirects=0;redirects<5;redirects++) {
      if (!UpdatePolicy.downloadHost(address)) throw new Exception("Untrusted update host");
      HttpURLConnection connection=(HttpURLConnection)new URL(address).openConnection();
      connection.setInstanceFollowRedirects(false); connection.setConnectTimeout(8000); connection.setReadTimeout(15000);
      connection.setRequestProperty("User-Agent","CarrotAssist-Updater");
      try {
        int status=connection.getResponseCode();
        if (status==301 || status==302 || status==303 || status==307 || status==308) { address=new URL(new URL(address),connection.getHeaderField("Location")).toString(); continue; }
        if(status!=200 || connection.getContentLengthLong()>maximum) throw new Exception("Invalid download response");
        long deadline=System.nanoTime()+60000000000L;
        try(InputStream input=connection.getInputStream()) {
          byte[] bytes=new byte[16384]; int total=0,n;
          while((n=input.read(bytes))!=-1) {
            total+=n;
            if(total>maximum || System.nanoTime()>deadline || Thread.currentThread().isInterrupted()) throw new Exception("Download limit exceeded");
            output.write(bytes,0,n);
          }
        }
        return;
      } finally { connection.disconnect(); }
    }
    throw new Exception("Too many redirects");
  }
}
