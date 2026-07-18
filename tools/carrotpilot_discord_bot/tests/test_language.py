from carrotbot.agent import LANGUAGE_INSTRUCTIONS, _answer_language


def test_answer_language_follows_the_user_question() -> None:
  assert "primarily English" in LANGUAGE_INSTRUCTIONS
  assert "primarily Korean" in LANGUAGE_INSTRUCTIONS
  assert "setting names, commands, and code identifiers" in LANGUAGE_INSTRUCTIONS


def test_detects_english_question_language() -> None:
  assert _answer_language("How do I use dashboard speed as the cruise target?", []) == "English"


def test_korean_prose_wins_over_setting_names() -> None:
  assert _answer_language("SpeedFromPCM 설정은 어디서 바꾸나요?", []) == "Korean"


def test_dashboard_speed_maps_to_speed_from_pcm_search() -> None:
  assert "dashboard" in LANGUAGE_INSTRUCTIONS
  assert "SpeedFromPCM" in LANGUAGE_INSTRUCTIONS
  assert "openpilot/selfdrive/car/cruise.py" in LANGUAGE_INSTRUCTIONS
