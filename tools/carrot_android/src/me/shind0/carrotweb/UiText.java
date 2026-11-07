package me.shind0.carrotweb;

import java.util.Locale;

/** Native host follows the phone's language; Carrot Web keeps its own language preference. */
public final class UiText {
  public static String get(String korean, String english) {
    return "ko".equals(Locale.getDefault().getLanguage()) ? korean : english;
  }
  private UiText() {}
}
