from types import SimpleNamespace

from carrotbot.agent import SPEED_INSTRUCTIONS, SupportAgent, _looks_like_tool_arguments


class FakeResponses:
  def __init__(self) -> None:
    self.calls: list[dict[str, object]] = []

  def create(self, **kwargs: object) -> SimpleNamespace:
    self.calls.append(kwargs)
    if len(self.calls) == 1:
      tool_call = SimpleNamespace(
        type="function_call",
        name="repo_info",
        arguments="{}",
        call_id="call-1",
      )
      return SimpleNamespace(output=[tool_call], output_text="")
    return SimpleNamespace(
      output=[],
      output_text='{"answer":"확인된 코드 기준의 최종 답변"}',
      status="completed",
      incomplete_details=None,
    )


class FakeRepository:
  def repo_info(self) -> dict[str, str]:
    return {"branch": "carrot-wip", "commit": "abc123"}

  def call_tool(self, name: str, arguments: dict[str, object]) -> str:
    assert name == "repo_info"
    assert arguments == {}
    return '{"branch":"carrot-wip","commit":"abc123"}'


def test_tool_limit_forces_a_final_answer() -> None:
  responses = FakeResponses()
  agent = object.__new__(SupportAgent)
  agent.client = SimpleNamespace(responses=responses)
  agent.model = "test-model"
  agent.repository = FakeRepository()
  agent.max_tool_rounds = 1

  answer = agent.answer(
    "질문",
    "user-1",
    [],
    {"display_name": "CarrotMaster/Ioniq5PE", "roles": ["당근", "C4"]},
    ["https://cdn.discordapp.com/attachments/error.png"],
    ["Attachment: params.json\n{\"CruiseButtonMode\": 1}"],
    ["[priority member] CarrotMaster: SpeedFromPCM을 1로 설정하세요."],
  )

  assert answer == "확인된 코드 기준의 최종 답변"
  assert len(responses.calls) == 2
  assert "tools" in responses.calls[0]
  assert responses.calls[0]["parallel_tool_calls"] is True
  assert responses.calls[0]["reasoning"] == {"effort": "low"}
  assert responses.calls[0]["tool_choice"] == "auto"
  assert "tools" not in responses.calls[1]
  assert responses.calls[1]["max_output_tokens"] == 2000
  assert responses.calls[1]["reasoning"] == {"effort": "low"}
  assert "json_schema" in str(responses.calls[1]["text"])
  assert "Ioniq5PE" in str(responses.calls[0]["input"])
  assert "C4" in str(responses.calls[0]["input"])
  assert "input_image" in str(responses.calls[0]["input"])
  assert "error.png" in str(responses.calls[0]["input"])
  assert "params.json" in str(responses.calls[0]["input"])
  assert "untrusted diagnostic data" in str(responses.calls[0]["input"])
  assert "priority member" in str(responses.calls[0]["input"])
  assert "untrusted community context" in str(responses.calls[0]["input"])
  final_input = responses.calls[1]["input"]
  assert isinstance(final_input, list)
  assert len(final_input) == 1
  assert "도구: repo_info" in str(final_input[0])
  assert "input_image" in str(final_input[0])


def test_default_answer_style_is_user_first() -> None:
  assert "개발자가 아니라" in SPEED_INSTRUCTIONS
  assert "결론 → 사용자가 할 일 → 필요한 주의점" in SPEED_INSTRUCTIONS
  assert "함수명, 변수명" in SPEED_INSTRUCTIONS


def test_device_log_tool_and_privacy_instructions_are_present() -> None:
  from carrotbot.agent import INSTRUCTIONS, TOOLS

  assert any(tool["name"] == "inspect_device_logs" for tool in TOOLS)
  assert "동글 ID를 요청" in INSTRUCTIONS
  assert "식별번호를 답변에 옮기지" in INSTRUCTIONS


def test_tool_rounds_are_capped_for_legacy_container_config() -> None:
  agent = SupportAgent("sk-test", "test-model", FakeRepository(), max_tool_rounds=6)
  assert agent.max_tool_rounds == 4


def test_detects_leaked_tool_arguments() -> None:
  assert _looks_like_tool_arguments('{"path":"openpilot/a.py","start_line":1,"end_line":20}')
  assert not _looks_like_tool_arguments('{"결론":"설정을 확인하세요"}')
  assert not _looks_like_tool_arguments("자연스러운 한국어 답변")
