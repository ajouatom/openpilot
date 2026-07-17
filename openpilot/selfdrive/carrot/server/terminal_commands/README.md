# Carrot Terminal Commands

The web terminal is a real interactive PTY. Custom commands therefore use the
same shell-native entry point as an SSH session:

```text
carrot help
carrot vision start
carrot vision status
carrot vision logs --lines 120
carrot vision stop
carrot web-intro
```

The login-shell bootstrap exports the `carrot` function before opening Bash.
Raw xterm input remains untouched, so interactive programs such as vim, btop,
less, and tmux keep their normal key handling.

`carrot web-intro` emits a private web-terminal action and opens the production
intro shell in dry-run mode. It uses the real steps and read-only data, while
device Params, web settings, restore, completion state, and browser car-label
cache writes are suppressed.

Legacy `::help`, `::vision_on`, and `::vision_off` input remains supported on
line-based clients, but new commands should document the `carrot` form.

## Adding a command

1. Add a module under `custom_commands/`.
2. Register one handler with `register_command`.
3. Import the module in `custom_commands/__init__.py`.
4. Keep reusable process and state management under `server/services/`.

```python
from ..registry import register_command


@register_command(
  name="example",
  summary="Describe the helper in one line.",
  usage="carrot example [status]",
)
def run(args: list[str]) -> int:
  print("[example] ready")
  return 0
```

Handlers receive an argument list and never need to parse a shell command.
Avoid `shell=True`; pass Python argument lists to subprocesses.

## Interfaces

- Shell: `carrot <command> [args...]`
- Catalog API: `GET /api/terminal_commands`
- Allowlisted run API: `POST /api/terminal_commands/run`
  with `{ "command": "vision", "args": ["status"] }`
- Legacy line bridge: `::command`, retained for compatibility

The run API starts only commands present in the registry. It launches the
fixed Python CLI without a shell, so request values cannot become arbitrary
shell commands.

## Layout

```text
terminal_commands/
  bridge.py              # Shell bootstrap and legacy line bridge.
  cli.py                 # Shell/API command dispatcher.
  registry.py            # Command metadata and handlers.
  custom_commands/       # Independently editable command modules.
    help.py
    vision_test.py
    web_intro.py
```
