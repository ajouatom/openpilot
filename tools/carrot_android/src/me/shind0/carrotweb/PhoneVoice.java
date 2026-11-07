package me.shind0.carrotweb;

import android.content.Context;
import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import java.util.Locale;

/** One foreground utterance at a time. Partial and stale callbacks never dispatch commands. */
final class PhoneVoice {
  interface Listener {
    void status(String text);
    void heard(LocalCommand command);
  }
  private final Context context;
  private final Listener listener;
  private final Handler ui = new Handler(Looper.getMainLooper());
  private SpeechRecognizer engine;
  private int generation;
  private final Runnable timeout = () -> cancelWith(UiText.get("음성 듣기를 마쳤어요.", "Listening ended."));

  PhoneVoice(Context context, Listener listener) { this.context = context; this.listener = listener; }
  boolean listening() { return engine != null; }

  void listen() {
    stop();
    if (Build.VERSION.SDK_INT < 31 || !SpeechRecognizer.isOnDeviceRecognitionAvailable(context)) {
      listener.status(UiText.get("기기 내 음성 인식을 사용할 수 없어요. 휴대폰의 음성 언어 팩을 확인하세요.", "On-device speech recognition is unavailable. Check the phone’s speech language pack."));
      return;
    }
    final int token = generation;
    final long started = SystemClock.elapsedRealtime();
    try {
      engine = SpeechRecognizer.createOnDeviceSpeechRecognizer(context);
      engine.setRecognitionListener(new RecognitionListener() {
        private long ended;
        public void onReadyForSpeech(Bundle params) {
          if (token == generation) listener.status(UiText.get("듣고 있어요 · 마이크를 누르면 종료", "Listening · tap the mic to stop"));
        }
        public void onBeginningOfSpeech() {}
        public void onRmsChanged(float value) {}
        public void onBufferReceived(byte[] data) {}
        public void onEndOfSpeech() { ended = SystemClock.elapsedRealtime(); }
        public void onPartialResults(Bundle results) {}
        public void onEvent(int event, Bundle params) {}
        public void onResults(Bundle results) {
          if (token != generation) return;
          long now = SystemClock.elapsedRealtime();
          LocalCommand command = results == null ? LocalCommand.UNRECOGNIZED : LocalCommand.choose(
            results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION), results.getFloatArray(SpeechRecognizer.CONFIDENCE_SCORES));
          if ((command == LocalCommand.LEFT || command == LocalCommand.RIGHT || command == LocalCommand.CONFIRM_LEFT || command == LocalCommand.CONFIRM_RIGHT || command == LocalCommand.CLARIFY_LEFT || command == LocalCommand.CLARIFY_RIGHT) && (now-started > 10000 || ended != 0 && now-ended > 1500))
            command = LocalCommand.UNCERTAIN;
          stop();
          listener.heard(command);
        }
        public void onError(int error) {
          if (token != generation) return;
          if (error == SpeechRecognizer.ERROR_NO_MATCH || error == SpeechRecognizer.ERROR_SPEECH_TIMEOUT) {
            cancelWith(UiText.get("음성 대기 종료 · 마이크로 다시 시작", "Listening ended · tap the mic to start"));
          } else if (error == SpeechRecognizer.ERROR_LANGUAGE_UNAVAILABLE || error == SpeechRecognizer.ERROR_LANGUAGE_NOT_SUPPORTED) {
            cancelWith(UiText.get("휴대폰의 기기 내 음성 인식 언어 팩을 확인하세요.", "Check the phone’s on-device speech language pack."));
          } else cancelWith(UiText.get("음성 인식을 시작하지 못했어요. 마이크 권한과 사용 상태를 확인하세요.", "Speech unavailable. Check microphone permission and availability."));
        }
      });
      Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
      intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
      intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.getDefault().toLanguageTag());
      intent.putExtra(RecognizerIntent.EXTRA_PARTIAL_RESULTS, false);
      intent.putExtra(RecognizerIntent.EXTRA_MAX_RESULTS, 3);
      listener.status(UiText.get("기기 내 음성 인식 준비 중", "Preparing on-device speech"));
      ui.postDelayed(timeout, 15000);
      engine.startListening(intent);
    } catch (RuntimeException e) {
      cancelWith(UiText.get("휴대폰의 음성 인식을 사용할 수 없어요.", "Phone speech recognition is unavailable."));
    }
  }

  private void cancelWith(String message) { stop(); listener.status(message); }
  void stop() {
    generation++;
    ui.removeCallbacks(timeout);
    SpeechRecognizer previous = engine;
    engine = null;
    if (previous != null) {
      try { previous.cancel(); } catch (RuntimeException ignored) {}
      try { previous.destroy(); } catch (RuntimeException ignored) {}
    }
  }
}
