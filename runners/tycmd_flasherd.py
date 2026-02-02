"""Runner for teensy ."""

import subprocess
from pathlib import Path
import grpc
import warnings
import sys
import queue
import flasherd_pb2
import flasherd_pb2_grpc

print(grpc.__version__)

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

        with grpc.insecure_channel('localhost:6767') as channel:
            stub = flasherd_pb2_grpc.FlasherdStub(channel)
            reqs = []

            with open(self.cfg.hex_file, 'rb') as binary_file:
                for block in iter(lambda: binary_file.read(4096), b''):
                    reqs.append(flasherd_pb2.RunCommandRequest(binary_name='zephyr_hex', binary_chunk=block))

            reqs.append(flasherd_pb2.RunCommandRequest(
                command_windows=r'C:\Program Files (x86)\TyTools\tycmd.exe',
                command_macos='tycmd',
                command_linux='tycmd',
                args=[
                    flasherd_pb2.Arg(regular='upload'),
                    flasherd_pb2.Arg(regular='--nocheck'),
                    flasherd_pb2.Arg(binary='zephyr_hex')
                ]
            ))

            self.logger.info(f'Launching tycmd via flasherd...')

            try:
                for resp in stub.RunCommand((req for req in reqs)):
                    if resp.HasField('stdout'):
                        print(resp.stdout, end='')
                    if resp.HasField('stderr'):
                        print(resp.stderr, file=sys.stderr, end='')
                    if resp.HasField('exit_code'):
                        self.logger.info('Process terminated with code {resp.exit_code}')
                        if resp.exit_code == 0:
                            self.logger.info('Flashed successfully.')
                        break
            except grpc.RpcError as e:
                self.logger.error('Failed to run command via flasherd.')
                print(e)
                if e.code() == grpc.StatusCode.UNAVAILABLE:
                    self.logger.error('Failed to connect to flasherd, it may not be running.')
