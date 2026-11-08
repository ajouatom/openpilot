package me.shind0.carrotweb;

import java.text.Normalizer;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;

/** Final utterances only. Match whole imperatives; never extract commands from larger sentences. */
public enum LocalCommand {
  LEFT, RIGHT, CLARIFY_LEFT, CLARIFY_RIGHT, CONFIRM_LEFT, CONFIRM_RIGHT, BLINKER_SETTINGS, QUESTION, STOP, UNRECOGNIZED, UNCERTAIN;

  private static final String CHANGE = "[,，]?(?:차로|차선)(?:를|을)?(?:(?:변경|바꿔)(?:해|해줘|해주세요|줘|주세요)?|변경(?:을)?(?:좀)?(?:해(?:줘|주세요)?|부탁해(?:요)?))";
  private static final Pattern LEFT_KO = Pattern.compile("(?:왼쪽|좌측|왼편)(?:으로|로)?" + CHANGE);
  private static final Pattern RIGHT_KO = Pattern.compile("(?:오른쪽|우측|오른편)(?:으로|로)?" + CHANGE);
  private static final Pattern SETTING = Pattern.compile("(?:깜박이|깜빡이|방향지시등)(?:자동제어|제어)?(?:관련)?설정(?:을)?(?:좀)?(?:열어|보여)(?:줘봐|줘|주세요)?");
  private static final float MIN_DRIVE_CONFIDENCE = .80f;

  static LocalCommand parse(String input) {
    if (input == null) return UNRECOGNIZED;
    String text = Normalizer.normalize(input, Normalizer.Form.NFKC).trim().toLowerCase(Locale.ROOT)
      .replaceAll("[\\s]+", " ").replaceFirst("[.!。！]+$", "").trim();
    String ko = text.replace(" ", "");
    if (LEFT_KO.matcher(ko).matches() || text.matches("(?:please )?change lanes? (?:to the )?left")) return LEFT;
    if (RIGHT_KO.matcher(ko).matches() || text.matches("(?:please )?change lanes? (?:to the )?right")) return RIGHT;
    if (ko.matches("왼쪽확인|좌측확인") || text.equals("confirm left")) return CONFIRM_LEFT;
    if (ko.matches("오른쪽확인|우측확인") || text.equals("confirm right")) return CONFIRM_RIGHT;
    if (SETTING.matcher(ko).matches() || text.matches("(?:please )?(?:open|show) (?:the )?(?:turn signal|blinker) settings")) return BLINKER_SETTINGS;
    if (ko.matches("그만|듣기종료|음성종료|stoplistening")) return STOP;
    if (ko.matches("(?:챗지피티|챗gpt|chatgpt)(?:열어줘|에물어볼게|로질문할게|open)?") ||
        ko.contains("왜") || ko.contains("어떻게") || text.endsWith("?") ||
        text.matches("(?:why|how|what|when|where) .+") || ko.endsWith("나요") || ko.endsWith("인가요") || ko.endsWith("설명해줘") || ko.endsWith("알려줘")) return QUESTION;
    return UNRECOGNIZED;
  }

  static LocalCommand choose(List<String> alternatives, float[] scores) {
    if (alternatives == null || alternatives.isEmpty()) return UNRECOGNIZED;
    LocalCommand first = parse(alternatives.get(0));
    if (first != LEFT && first != RIGHT && first != CONFIRM_LEFT && first != CONFIRM_RIGHT) return first;
    if (conflicting(alternatives)) return UNCERTAIN;
    if (scores == null || scores.length == 0 || scores[0] == -1f) {
      if (first == LEFT) return CLARIFY_LEFT;
      if (first == RIGHT) return CLARIFY_RIGHT;
      return first;  // A confirmation is usable only in an active, same-direction challenge.
    }
    if (!(scores[0] >= MIN_DRIVE_CONFIDENCE && scores[0] <= 1)) return UNCERTAIN;
    return first;
  }

  static boolean conflicting(List<String> alternatives) {
    LocalCommand first = parse(alternatives.get(0));
    for (int i = 1; i < alternatives.size(); i++)
      if (parse(alternatives.get(i)) != first) return true;
    return false;
  }

  static boolean stale(long start, long speechStart, long end, long now) {
    return now - (speechStart > 0 ? speechStart : start) > 10000 || end > 0 && now - end > 1500;
  }

  /** A bare yes cannot authorize movement. Challenges never survive pause, timeout or a different reply. */
  static final class DirectionCheck {
    private LocalCommand expected;
    private long deadline;
    void clear() { expected = null; }
    void ask(LocalCommand side, long now) {
      expected = side == CLARIFY_LEFT ? CONFIRM_LEFT : CONFIRM_RIGHT;
      deadline = now + 6000;
    }
    LocalCommand resolve(LocalCommand heard, long now) {
      boolean matches = heard == expected && now <= deadline;
      clear();
      if (heard == CONFIRM_LEFT || heard == CONFIRM_RIGHT) return matches ? (heard == CONFIRM_LEFT ? LEFT : RIGHT) : UNCERTAIN;
      return heard;
    }
  }

}
