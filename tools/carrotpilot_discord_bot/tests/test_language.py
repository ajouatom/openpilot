from carrotbot.agent import LANGUAGE_INSTRUCTIONS


def test_answer_language_follows_the_user_question() -> None:
  assert "primarily English" in LANGUAGE_INSTRUCTIONS
  assert "primarily Korean" in LANGUAGE_INSTRUCTIONS
  assert "setting names, commands, and code identifiers" in LANGUAGE_INSTRUCTIONS
