package me.shind0.carrotweb;

import java.util.Arrays;
import java.util.Collections;
import static me.shind0.carrotweb.LocalCommand.*;

public class LocalCommandTest {
  private static int checks;
  private static void check(boolean ok) { if (!ok) throw new AssertionError("Voice check "+checks); checks++; }
  public static void main(String[] args) {
    for (String word : new String[]{"왼쪽으로 차로 변경해", "좌측 차선 변경해 주세요", "왼쪽 차로를 바꿔줘", "change lanes left"})
      check(LocalCommand.choose(Collections.singletonList(word),new float[]{.92f}) == LEFT);
    for (String word : new String[]{"오른쪽으로 차로 변경해줘", "우측 차선변경", "change lanes to the right"})
      check(LocalCommand.choose(Collections.singletonList(word),new float[]{.92f}) == RIGHT);
    for (String word : new String[]{"차로 변경해줘", "왼쪽으로 차로 변경하지 마", "왼쪽 차로 변경해 말고 오른쪽", "왼쪽 차로 변경해 라고 했어",
           "왼쪽 차로 변경해?", "왼쪽 차로 변경하면 어떻게 돼?", "Don't change lanes left", "change lanes left or right", "please change lanes left?",
           "어제 왼쪽으로 차로 변경해", "왼쪽으로 차로 변경해 그리고 속도를 올려", "가속이 너무 빨라 왜 그래?"}) {
      LocalCommand parsed = LocalCommand.choose(Collections.singletonList(word),new float[]{.99f});
      check(parsed != LEFT && parsed != RIGHT);
    }
    check(LocalCommand.choose(Arrays.asList("왼쪽으로 차로 변경해","오른쪽으로 차로 변경해"),new float[]{.95f,.89f}) == UNCERTAIN);
    check(LocalCommand.choose(Arrays.asList("왼쪽으로 차로 변경해","왼쪽으로 차로 변경하면?"),new float[]{.99f,.8f}) == UNCERTAIN);
    check(LocalCommand.choose(Arrays.asList("왼쪽으로 차로 변경해","왼쪽 차선 변경해줘"),new float[]{.95f,.9f}) == LEFT);
    for (float f : new float[]{0f,.4f,Float.NaN,1.5f}) check(LocalCommand.choose(Collections.singletonList("좌측차로변경"),new float[]{f}) == UNCERTAIN);
    check(LocalCommand.choose(Collections.singletonList("우측 차로 변경"),null) == CLARIFY_RIGHT);
    check(LocalCommand.choose(Collections.singletonList("왼쪽으로 차로 변경해"),new float[]{-1f}) == CLARIFY_LEFT);
    check(LocalCommand.choose(Collections.singletonList("왼쪽 확인"),null) == CONFIRM_LEFT);
    check(LocalCommand.choose(Arrays.asList("왼쪽 확인","오른쪽 확인"),null) == UNCERTAIN);
    check(LocalCommand.parse("깜박이 설정 열어줘") == BLINKER_SETTINGS);
    check(LocalCommand.parse("깜빡이 관련 설정 보여 주세요") == BLINKER_SETTINGS);
    check(LocalCommand.parse("Open turn signal settings") == BLINKER_SETTINGS);
    check(LocalCommand.parse("깜박이 설정을 바꿔줘") == UNRECOGNIZED);
    check(LocalCommand.parse("가속이 너무 빨라 왜 그래?") == QUESTION);
    check(LocalCommand.parse("음성 종료") == STOP);
    LocalCommand.DirectionCheck pending = new LocalCommand.DirectionCheck();
    check(pending.resolve(CONFIRM_LEFT, 1) == UNCERTAIN);
    pending.ask(CLARIFY_LEFT, 1); check(pending.resolve(CONFIRM_RIGHT, 2) == UNCERTAIN);
    pending.ask(CLARIFY_RIGHT, 1); check(pending.resolve(CONFIRM_RIGHT, 6002) == UNCERTAIN);
    pending.ask(CLARIFY_LEFT, 1); check(pending.resolve(CONFIRM_LEFT, 2) == LEFT);
    check(pending.resolve(CONFIRM_LEFT, 3) == UNCERTAIN);
    pending.ask(CLARIFY_RIGHT, 1); pending.clear(); check(pending.resolve(CONFIRM_RIGHT, 2) == UNCERTAIN);
    for (String phrase : Arrays.asList("가속도 설정 0을 240으로 바꿔줘", "가속도 설정 0을 이백사십으로 바꿔 줘", "가속도 설정 0 240으로 변경해 주세요")) {
      SpokenSetting value = SpokenSetting.parse(phrase);
      check(value != null && value.name.equals("가속도 설정 0") && value.value == 240);
    }
    SpokenSetting english = SpokenSetting.parse("Set acceleration setting zero to 240.");
    check(english != null && english.value == 240);
    check(SpokenSetting.parse("조향 지연을 마이너스 십으로 바꿔줘").value == -10);
    for (String phrase : Arrays.asList("가속도 설정 0을 240으로 바꾸지 마", "가속도 설정 0을 240으로 바꿔줘?", "가속도를 10%만 올려", "Set accel to 2.4", "밝기를 일일로 바꿔줘"))
      check(SpokenSetting.parse(phrase) == null);
    String phrase = "가속도 설정 0을 240으로 바꿔줘";
    SpokenSetting value = SpokenSetting.parse(phrase);
    check(value.agreed(Arrays.asList(phrase, "가속도설정 0을 이백사십으로 변경해줘"), null));
    check(!value.agreed(Arrays.asList(phrase, phrase.replace("240", "250")), null));
    check(!value.agreed(Arrays.asList(phrase, phrase.replace("0을", "1을")), null));
    check(!value.agreed(Arrays.asList(phrase, phrase + " 말고"), null));
    check(!value.agreed(Arrays.asList(phrase), new float[]{.4f}));
    check("가속도".equals(SpokenSetting.search("가속도 설정을 찾아줘")));
    check("가속도".equals(SpokenSetting.search("가속도에 관련된 설정을 찾아줘")));
    check("acceleration".equals(SpokenSetting.search("Find acceleration settings")));
    check(SpokenSetting.search(phrase) == null);
    String ordinal = "첫 번째 거 240으로 바꿔줘";
    for (String phraseByNumber : Arrays.asList(ordinal, "첫번째 것을 이백사십으로 변경해줘", "1번째 걸 240으로 바꿔줘", "set the first one to 240")) {
      SpokenSetting selected = SpokenSetting.parse(phraseByNumber);
      check(selected != null && selected.position == 1 && selected.value == 240);
    }
    check(SpokenSetting.parse("두 번째 설정을 245로 변경해줘").position == 2);
    check(SpokenSetting.parse("20번 항목을 10으로 바꿔줘").position == 20);
    check(SpokenSetting.parse("가속도 설정 0을 240으로 바꿔줘").position == 0);
    check(SpokenSetting.parse(ordinal).agreed(Arrays.asList(ordinal, "1번 항목을 240으로 바꿔줘"), null));
    check(!SpokenSetting.parse(ordinal).agreed(Arrays.asList(ordinal, "두 번째 거 240으로 바꿔줘"), null));
    check(!SpokenSetting.parse(ordinal).agreed(Arrays.asList(ordinal, "첫 번째 거 250으로 바꿔줘"), null));
    for (String text : Arrays.asList("왼쪽으로 차로변경", "왼쪽으로, 차로 변경", "왼쪽 차로 변경을 해줘"))
      check(LocalCommand.choose(Arrays.asList(text,"왼쪽으로 차로 변경"), new float[]{.95f}) == LEFT);
    check(LocalCommand.choose(Arrays.asList("오른쪽으로, 차로 변경", "우측 차선 변경 부탁해"), new float[]{.95f}) == RIGHT);
    check(LocalCommand.choose(Arrays.asList("왼쪽으로 차로변경", "왼쪽으로 차로변경 하지 마"), new float[]{.99f}) == UNCERTAIN);
    check(!LocalCommand.stale(1000,10000,11900,12000));
    check(LocalCommand.stale(1000,0,11900,12000));
    check(LocalCommand.stale(1000,9000,10400,12000));
    check(LocalCommand.stale(1000,2000,12900,13000));
    System.out.println("Local voice command checks passed: "+checks);
  }
}
