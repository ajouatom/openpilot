package me.shind0.carrotweb;

import java.text.Normalizer;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Whole, absolute integer assignments from final Android recognition results. */
final class SpokenSetting {
  private static final String INTEGER = "(?:-?[0-9]{1,9}|(?:마이너스\\s*)?[영공일이삼사오육칠팔구십백천]+)";
  private static final Pattern KO = Pattern.compile("(.{1,100}?)(?:[을를]\\s*|\\s+)(" + INTEGER +
    ")\\s*(?:으로|로)\\s*(?:바꿔|변경해|설정해|맞춰)(?:\\s*(?:줘|주세요))?");
  private static final Pattern EN = Pattern.compile("(?:please )?(?:set|change) (.{1,100}?) to (-?[0-9]{1,9})");
  private static final Pattern FIND_KO = Pattern.compile("(.{1,80}?)(?:에\\s*(?:관련된|대한)|\\s*관련)?\\s*설정(?:들)?(?:을|좀)?\\s*(?:찾아|검색해|보여)(?:\\s*(?:줘|주세요|줘봐))?");
  private static final Pattern FIND_EN = Pattern.compile("(?:please )?(?:find|show|search for) (.{1,80}?) settings?");
  static String search(String input) {
    if (input == null || input.length() > 130) return null;
    String text = Normalizer.normalize(input, Normalizer.Form.NFKC).trim().toLowerCase(Locale.ROOT)
      .replaceAll("\\s+", " ").replaceFirst("[.!。！]+$", "").trim();
    Matcher match = FIND_KO.matcher(text);
    if (!match.matches()) {
      match = FIND_EN.matcher(text);
      if (!match.matches()) return null;
    }
    String query = match.group(1).trim();
    return query.isEmpty() ? null : query;
  }
  // The web displays at most twenty search results; spoken numbers are one-based.
  private static final String[] KOREAN_ORDINALS = {"첫", "두", "세", "네", "다섯", "여섯", "일곱", "여덟", "아홉", "열",
    "열한", "열두", "열세", "열네", "열다섯", "열여섯", "열일곱", "열여덟", "열아홉", "스무"};
  private static final String[] ENGLISH_ORDINALS = {"first", "second", "third", "fourth", "fifth", "sixth", "seventh", "eighth", "ninth", "tenth",
    "eleventh", "twelfth", "thirteenth", "fourteenth", "fifteenth", "sixteenth", "seventeenth", "eighteenth", "nineteenth", "twentieth"};
  private static final Pattern NUMBERED_KO = Pattern.compile("([1-9][0-9]?)(?:번째|번)");
  private static final Pattern NUMBERED_EN = Pattern.compile("([1-9][0-9]?)(?:st|nd|rd|th)");
  private static int position(String name) {
    String ko = name.replace(" ", "").replaceFirst("(?:항목|설정|거|것|걸)$", "");
    Matcher digits = NUMBERED_KO.matcher(ko);
    if (digits.matches()) return Integer.parseInt(digits.group(1));
    String en = name.replaceFirst("^the ", "").replaceFirst(" (?:one|result|setting|item)$", "");
    digits = NUMBERED_EN.matcher(en);
    if (digits.matches()) return Integer.parseInt(digits.group(1));
    for (int i = 0; i < KOREAN_ORDINALS.length; i++)
      if (ko.equals(KOREAN_ORDINALS[i] + "번째") || en.equals(ENGLISH_ORDINALS[i])) return i + 1;
    return 0;
  }

  final int position;
  final String name;
  final int value;

  SpokenSetting(String name, int value) { this.name = name; this.value = value; this.position = position(name); }

  private static Integer integer(String text) {
    if (text.matches("-?[0-9]+")) return Integer.valueOf(text);
    boolean negative = text.startsWith("마이너스");
    String digits = negative ? text.substring(4).trim() : text;
    if (digits.equals("영") || digits.equals("공")) return 0;
    if (digits.isEmpty()) return null;
    int total = 0, ones = 0, lastUnit = 10000;
    for (char c : digits.toCharArray()) {
      int digit = " 일이삼사오육칠팔구".indexOf(c);
      if (digit > 0) {
        if (ones != 0) return null;
        ones = digit;
      } else {
        int unit = c == '십' ? 10 : c == '백' ? 100 : c == '천' ? 1000 : 0;
        if (unit == 0 || unit >= lastUnit) return null;
        total += (ones == 0 ? 1 : ones) * unit;
        ones = 0;
        lastUnit = unit;
      }
    }
    return (negative ? -1 : 1) * (total + ones);
  }

  static SpokenSetting parse(String input) {
    if (input == null || input.length() > 180) return null;
    String text = Normalizer.normalize(input, Normalizer.Form.NFKC).trim().toLowerCase(Locale.ROOT)
      .replaceAll("\\s+", " ").replaceFirst("[.!。！]+$", "").trim();
    Matcher match = KO.matcher(text);
    if (!match.matches()) {
      match = EN.matcher(text);
      if (!match.matches()) return null;
    }
    Integer value = integer(match.group(2));
    return value == null ? null : new SpokenSetting(match.group(1).trim(), value);
  }

  boolean agreed(List<String> alternatives, float[] scores) {
    if (scores != null && scores.length > 0 && scores[0] != -1f && !(scores[0] >= .8f && scores[0] <= 1)) return false;
    for (String alternative : alternatives) {
      SpokenSetting other = parse(alternative);
      if (other == null || other.value != value || other.position != position) return false;
      if (position == 0 && !name.replace(" ", "").equals(other.name.replace(" ", ""))) return false;
    }
    return true;
  }
}
