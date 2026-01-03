from __future__ import annotations

import importlib.util
import sys

# HACK: Zephyr imports runners one by one with an identical module name each time. As its module name is shadowed, other
# runners have no way to refer to this module. We bypass this by manually making this file available for import under
# another name when it's first executed. This only works if this file is specified first in `zephyr/module.yml`.
MODULE_NAME = 'clover_runners_common'

if __name__ != MODULE_NAME:
    spec = importlib.util.spec_from_file_location(MODULE_NAME, __file__)
    module = importlib.util.module_from_spec(spec)
    sys.modules[MODULE_NAME] = module
    spec.loader.exec_module(module)


class FlasherdCommand:
    """Wraps an argument array to properly set args for a call to flasherd"""

    def __init__(self, client: str):
        self.payload = [client]

    def __iadd__(self, other: list[str] | str):
        if isinstance(other, list):
            for arg in other:
                self.add_normal_arg(arg)
        elif isinstance(other, str):
            self.add_normal_arg(other)
        else:
            raise TypeError(f'other must be list[str] or str, got: {other}')
        return self

    def add_windows_command(self, command: str):
        self.payload += ['--command-windows', command]

    def add_macos_command(self, command: str):
        self.payload += ['--command-macos', command]

    def add_linux_command(self, command: str):
        self.payload += ['--command-linux', command]

    def add_normal_arg(self, arg: str):
        self.payload += ['--arg', arg]

    def add_path_arg(self, arg: str):
        self.payload += ['--arg-path', arg]

    def append(self, arg: str):
        self.add_normal_arg(arg)
