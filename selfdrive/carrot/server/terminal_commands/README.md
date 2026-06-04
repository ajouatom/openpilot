# Web Terminal Meta Commands

The Carrot web terminal sends normal input to its tmux shell unchanged. Input
starting with `:` is reserved for small web-terminal-only helpers.

```text
:help
:help vision_test
:vision_test status
```

## Adding A Command

1. Add `custom_commands/example.py`.
2. Register one handler with `register_command`.
3. Import the new module in `custom_commands/__init__.py`.
4. Print normal progress and result text from the handler. Output appears in
   the existing web terminal because the command runs through the tmux shell.

```python
from ..registry import register_command


@register_command(
  name="example",
  summary="Describe the helper in one line.",
  usage=":example [status]",
)
def run(args: list[str]) -> int:
  print("[example] ready")
  return 0
```

Use Python argument lists for subprocess calls. Do not pass user input to a
shell. Keep reusable process or state management in `server/services/`; the
command handler should remain a small text interface.

## Layout

```text
terminal_commands/
  bridge.py              # Converts : input into a fixed CLI call.
  cli.py                 # Parses one meta-command line.
  registry.py            # Stores command metadata and handlers.
  custom_commands/       # Actual user-editable custom command files.
    help.py
    vision_test.py
```

## Bridge Flow

1. `features/terminal.py` receives terminal input over the existing websocket.
2. `bridge.py` converts `:` input into a fixed Python CLI invocation.
3. `cli.py` parses the command and invokes a registered handler.
4. stdout and stderr are rendered by the existing tmux screen capture loop.

This keeps ordinary shell behavior intact and makes each helper independently
editable.
