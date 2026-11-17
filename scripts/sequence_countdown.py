"""Cancellable client countdown before a named sequence RUN is sent."""
from contextlib import closing
import math
import select
import sys
import time

from prompt_toolkit.input import create_input


def countdown(console, name, seconds=10, terminal_input=None):
    """Return False on Space/Q/Ctrl+C; always restore terminal input.

    Redirected input preserves immediate execution for scripts and automation.
    terminal_input allows tests to supply a real pipe instead of a terminal.
    """
    if terminal_input is None:
        if not sys.stdin.isatty():
            return True
        with closing(create_input()) as source:
            return countdown(console, name, seconds, source)
    console.print(f'Starting {name}: press Space or Q to cancel (no Enter needed).', markup=False)
    deadline = time.monotonic() + seconds
    previous = None
    try:
        with terminal_input.raw_mode():
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return True
                shown = math.ceil(remaining)
                if shown != previous:
                    console.print(f'Starting {name} in {shown}...', markup=False)
                    previous = shown
                if select.select([terminal_input.fileno()], [], [], min(remaining, 0.1))[0]:
                    keys = terminal_input.read_keys()
                    if terminal_input.closed or any(key.data in (' ', 'q', 'Q', '\x03') for key in keys):
                        break
    except (KeyboardInterrupt, EOFError):
        pass
    console.print('Cancelled. Sequence was not started; connection remains open.', markup=False)
    return False
