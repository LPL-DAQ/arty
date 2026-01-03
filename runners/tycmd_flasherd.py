"""Runner for teensy ."""

import subprocess
from pathlib import Path

from clover_runners_common import FlasherdCommand

from runners.core import ZephyrBinaryRunner


class TycmdFlasherdBinaryRunner(ZephyrBinaryRunner):
    """tycmd frontend integrated with flasherd"""

    def __init__(self, cfg):
        super().__init__(cfg)

    @classmethod
    def name(cls):
        return 'tycmd_flasherd'

    @classmethod
    def do_add_parser(cls, parser):
        pass

    @classmethod
    def do_create(cls, cfg, args) -> 'TycmdFlasherdBinaryRunner':
        return TycmdFlasherdBinaryRunner(cfg)

    def do_run(self, command):
        if command == 'flash':
            self.flash()

    def flash(self):
        # Prepare base command
        cmd = FlasherdCommand('/home/lpl/arty/bin/flasherd-client')
        cmd.add_windows_command(r'C:\Program Files (x86)\TyTools\tycmd.exe')
        cmd.add_macos_command('tycmd')
        cmd.add_linux_command('tycmd')

        if self.cfg.hex_file is not None and Path(self.cfg.hex_file).is_file():
            cmd += 'upload'
            cmd += '--nocheck'
            cmd.add_path_arg(self.cfg.hex_file)
        else:
            raise ValueError(f'Cannot flash; no hex ({self.cfg.hex_file}) file found. ')

        self.logger.info(f'Flashing file: {self.cfg.hex_file}')

        try:
            self.check_call(cmd.payload)
            self.logger.info('Success')
        except subprocess.CalledProcessError as grepexc:
            self.logger.error(f'Failure {grepexc.returncode}')
