"""Runner for teensy ."""

import subprocess
from pathlib import Path
import grpc
import warnings
import sys
import queue
import flasherd_pb2
import flasherd_pb2_grpc
import client
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
        if self.cfg.hex_file is None or not Path(self.cfg.hex_file).is_file():
            raise ValueError(f'Cannot flash; no hex ({self.cfg.hex_file}) file found. ')
        self.logger.info(f'Hex file: {self.cfg.hex_file}')

        binaries={
            "zephyr.hex": self.cfg.hex_file
        }
        command_req = flasherd_pb2.RunCommandRequest(
            command_windows=r'C:\Program Files (x86)\TyTools\tycmd.exe',
            command_macos='tycmd',
            command_linux='tycmd',
            args=[
                flasherd_pb2.Arg(regular='upload'),
                flasherd_pb2.Arg(regular='--nocheck'),
                flasherd_pb2.Arg(binary='zephyr.hex')
            ]
        )
        client.launch_flasherd_command(self.logger, command_req, binaries)
