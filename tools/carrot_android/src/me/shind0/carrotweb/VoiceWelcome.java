package me.shind0.carrotweb;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.speech.tts.TextToSpeech;
import android.speech.tts.UtteranceProgressListener;
import android.widget.Toast;
import java.net.HttpURLConnection;
import java.util.Locale;
import org.json.JSONObject;

/** The optional app-open greeting is separate from manual voice invocation and driving requests. */
final class VoiceWelcome {
  private final Context context;
  private final Handler ui = new Handler(Looper.getMainLooper());
  private TextToSpeech tts;
  private boolean ttsReady, updateAvailable;
  private int generation, utterance;
  private String address;
  private DeviceProbe probe;
  private Runnable pending;
  private String feedback;
  private final Runnable fallback = this::finish;
  private final Runnable speak = this::speakNow;

  VoiceWelcome(Context context) { this.context = context; }

  void prepare(String host) {
    if (host.equals(address)) return;
    reset(); address = host;
    final int token = generation;
    tts = new TextToSpeech(context, status -> ui.post(() -> {
      if (token != generation || tts == null || status != TextToSpeech.SUCCESS) return;
      ttsReady = tts.setLanguage(Locale.getDefault()) >= TextToSpeech.LANG_AVAILABLE;
      if (ttsReady && pending != null) { ui.removeCallbacks(speak); speakNow(); }
    }));
    DeviceProbe request = new DeviceProbe(url -> (HttpURLConnection)url.openConnection());
    probe = request;
    new Thread(() -> {
      boolean available = false;
      try {
        JSONObject result = new JSONObject(request.get(host, "/api/tools/git_status", 65536));
        available = verifiedUpdate(result);
      } catch (Exception ignored) {} finally { request.close(); }
      final boolean found = available;
      ui.post(() -> { if (token == generation) updateAvailable = found; });
    }, "carrot-startup-git-status").start();
  }

  static boolean verifiedUpdate(JSONObject result) {
    return result.optBoolean("ok") && result.optBoolean("available") && "ok".equals(result.optString("state"))
      && result.optInt("behind",0) > 0 && result.optInt("ahead",-1) == 0
      && result.optString("head").matches("[0-9a-fA-F]{40}|[0-9a-fA-F]{64}")
      && result.optString("target_head").matches("[0-9a-fA-F]{40}|[0-9a-fA-F]{64}")
      && !result.optString("target_head").equals(result.optString("head"));
  }

  void greet(Runnable done) {
    cancel(); pending = done;
    // Don't make an unresponsive speech engine delay access to the assistant.
    ui.postDelayed(speak, ttsReady ? 0 : 450);
  }

  void say(String text, Runnable done) {
    cancel(); feedback = text; pending = done;
    speakNow();
  }

  private void speakNow() {
    if (pending == null) return;
    ui.removeCallbacks(speak);
    String message = UiText.get("안녕하세요. 안전 운전하세요.", "Hello. Drive safely.");
    if (updateAvailable) message += UiText.get(" 당근 기기에 새 업데이트가 있어요.", " An update is available for your Carrot device.");
    if (feedback != null) message = feedback;
    if (!ttsReady || tts == null) { Toast.makeText(context,message,Toast.LENGTH_SHORT).show(); finish(); return; }
    final String id = "welcome-" + ++utterance;
    tts.setOnUtteranceProgressListener(new UtteranceProgressListener() {
      @Override public void onStart(String speaking) {}
      @Override public void onDone(String speaking) { complete(speaking); }
      @Override public void onError(String speaking) { complete(speaking); }
      private void complete(String speaking) {
        ui.post(() -> { if (id.equals(speaking) && id.equals("welcome-" + utterance)) finish(); });
      }
    });
    ui.postDelayed(fallback, 8000);
    if (tts.speak(message,TextToSpeech.QUEUE_FLUSH,null,id) == TextToSpeech.ERROR) finish();
  }

  boolean isGreeting() { return pending != null; }
  private void finish() {
    Runnable done = pending;
    cancel();
    if (done != null) done.run();
  }
  void cancel() {
    pending = null; feedback = null; utterance++;
    ui.removeCallbacks(fallback); ui.removeCallbacks(speak);
    if (tts != null && ttsReady) tts.stop();
  }
  void reset() {
    generation++; cancel(); address = null; updateAvailable = false; ttsReady = false;
    if (probe != null) { probe.close(); probe = null; }
    if (tts != null) { tts.shutdown(); tts = null; }
  }
}
