/**
 * Speech-to-Text helper using Web Speech API (webkitSpeechRecognition / SpeechRecognition)
 * Robust error handling, clean lifecycle, and non-destructive transcript appending.
 */

export class SpeechInputController {
  constructor(options = {}) {
    this.onTranscript = options.onTranscript || (() => {});
    this.onStatusChange = options.onStatusChange || (() => {});
    this.onError = options.onError || (() => {});
    this.recognition = null;
    this.isListening = false;
    this.initialText = "";
  }

  isSupported() {
    if (typeof window === "undefined") return false;
    return Boolean(window.SpeechRecognition || window.webkitSpeechRecognition);
  }

  _createRecognition() {
    const SpeechRecognitionClass = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!SpeechRecognitionClass) return null;

    let curLang = "ko-KR";
    try {
      const activeLang = typeof globalThis.getCurrentLanguage === "function"
        ? globalThis.getCurrentLanguage()
        : (typeof LANG !== "undefined" ? LANG : (localStorage.getItem("carrot_lang") || "ko"));
      if (activeLang === "en") curLang = "en-US";
      else if (activeLang === "zh") curLang = "zh-CN";
      else curLang = "ko-KR";
    } catch {}

    const rec = new SpeechRecognitionClass();
    rec.lang = curLang;
    rec.continuous = false;
    rec.interimResults = true;
    rec.maxAlternatives = 1;

    rec.onstart = () => {
      this.isListening = true;
      this.onStatusChange("listening");
    };

    rec.onresult = (event) => {
      let finalTranscript = "";
      let interimTranscript = "";
      for (let i = event.resultIndex; i < event.results.length; ++i) {
        const item = event.results[i];
        if (item.isFinal) {
          finalTranscript += item[0].transcript;
        } else {
          interimTranscript += item[0].transcript;
        }
      }

      const text = (finalTranscript || interimTranscript).trim();
      if (text) {
        const fullText = this.initialText ? `${this.initialText} ${text}` : text;
        this.onTranscript(fullText, Boolean(finalTranscript));
      }
    };

    rec.onerror = (event) => {
      this.isListening = false;
      this.onStatusChange("idle");

      const err = event.error;
      if (err === "no-speech") {
        // Timeout with no speech detected - graceful idle
        return;
      }
      if (err === "aborted") {
        return;
      }
      if (err === "not-allowed" || err === "service-not-allowed") {
        this.onError("마이크 사용 권한이 거부되었거나, HTTP 접속 환경에서는 브라우저 보안 정책상 음성 인식이 제한될 수 있습니다.");
        return;
      }
      if (err === "audio-capture") {
        this.onError("마이크 입력 장치를 찾을 수 없습니다.");
        return;
      }
      if (err === "network") {
        this.onError("음성 인식 네트워크 연결에 실패했습니다. 인터넷 연결을 확인해주세요.");
        return;
      }
      this.onError(`음성 인식 오류: ${err}`);
    };

    rec.onend = () => {
      this.isListening = false;
      this.onStatusChange("idle");
    };

    return rec;
  }

  toggle(existingText = "") {
    if (this.isListening) {
      this.stop();
    } else {
      this.start(existingText);
    }
  }

  start(existingText = "") {
    if (!this.isSupported()) {
      this.onError("현재 브라우저 환경에서는 음성 인식을 지원하지 않습니다. (Chrome/Safari/Edge 권장)");
      return;
    }

    this.initialText = (existingText || "").trim();

    try {
      if (this.recognition) {
        try { this.recognition.abort(); } catch {}
      }
      this.recognition = this._createRecognition();
      if (this.recognition) {
        this.recognition.start();
      }
    } catch (e) {
      this.isListening = false;
      this.onStatusChange("idle");
      this.onError(`음성 인식을 시작할 수 없습니다: ${e.message}`);
    }
  }

  stop() {
    this.isListening = false;
    this.onStatusChange("idle");
    if (this.recognition) {
      try {
        this.recognition.stop();
      } catch {}
    }
  }

  destroy() {
    this.stop();
    if (this.recognition) {
      try { this.recognition.abort(); } catch {}
      this.recognition = null;
    }
  }
}
