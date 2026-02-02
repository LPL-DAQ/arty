from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, override
import os
import flasherd_pb2
import flasherd_pb2_grpc
import client
from runners.stm32cubeprogrammer import STM32CubeProgrammerBinaryRunner

if TYPE_CHECKING:
    import argparse

    from runners.core import RunnerConfig

class STM32CubeProgrammerFlasherdBinaryRunner(STM32CubeProgrammerBinaryRunner):
    """STM32CubeProgrammer frontend integrated with flasherd"""

    def __init__(
        self,
        cfg: RunnerConfig, **kwargs
    ) -> None:
        super().__init__(
            cfg, **kwargs
        )

    @classmethod
    def name(cls):
        return 'stm32cubeprogrammer_flasherd'

    @classmethod
    def do_add_parser(cls, parser):
        super().do_add_parser(parser)

    @classmethod
    def do_create(
        cls, cfg: RunnerConfig, args: argparse.Namespace
    ) -> 'STM32CubeProgrammerFlasherdBinaryRunner':
        return STM32CubeProgrammerFlasherdBinaryRunner(
            cfg,
            port=args.port,
            frequency=args.frequency,
            reset_mode=args.reset_mode,
            download_address=args.download_address,
            download_modifiers=args.download_modifiers,
            start_address=args.start_address,
            start_modifiers=args.start_modifiers,
            conn_modifiers=args.conn_modifiers,
            cli=args.cli,
            use_elf=args.use_elf,
            erase=args.erase,
            extload=args.extload,
            tool_opt=args.tool_opt,
        )

    def do_run(self, command):
        if command == 'flash':
            self.flash()

    def flash(self, **kwargs) -> None:
        base_command = flasherd_pb2.RunCommandRequest(
            command_windows=r'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
            command_macos='/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI',
            command_linux='~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI',
        )

        # List devices
        command_req = flasherd_pb2.RunCommandRequest()
        command_req.CopyFrom(base_command)
        command_req.args.append(flasherd_pb2.Arg(regular='-l'))
        client.launch_flasherd_command(self.logger, command_req, {})

        connect_opts = f'port={self._port}'
        if self._frequency:
            connect_opts += f' freq={self._frequency}'
        if self._reset_mode:
            reset_mode = STM32CubeProgrammerFlasherdBinaryRunner._RESET_MODES[self._reset_mode]
            connect_opts += f' reset={reset_mode}'
        if self._conn_modifiers:
            connect_opts += f' {self._conn_modifiers}'

        args = ['--connect', connect_opts]
        args += self._tool_opt
        if self._extload:
            # external loader to come after the tool option in STM32CubeProgrammer
            args += self._extload

        # erase first if requested
        if self._erase:
            self.logger.info('Erasing first...')

            command_req = flasherd_pb2.RunCommandRequest()
            command_req.CopyFrom(base_command)
            command_req.args.extend([flasherd_pb2.Arg(regular=arg) for arg in args + ['--erase', 'all']])
            client.launch_flasherd_command(self.logger, command_req, {})

        # Define binary to be loaded
        dl_file = None

        if self._use_elf:
            # Use elf file if instructed to do so.
            dl_file = self.cfg.elf_file
        elif self.cfg.bin_file is not None and (
            self._download_address is not None
            or (str(self._port).startswith('usb') and self._download_modifiers is not None)
        ):
            # Use bin file if a binary is available and
            # --download-address provided
            # or flashing by dfu (port=usb and download-modifier used)
            dl_file = self.cfg.bin_file
        elif self.cfg.hex_file is not None:
            # Neither --use-elf nor --download-address are present:
            # default to flashing using hex file.
            dl_file = self.cfg.hex_file

        # Verify file configuration
        if dl_file is None:
            raise RuntimeError('cannot flash; no download file was specified')
        if not Path(dl_file).is_file():
            raise RuntimeError(f'download file {dl_file} does not exist')

        req_args = [flasherd_pb2.Arg(regular=arg) for arg in args + ['--download']]

        _, ext = os.path.splitext(dl_file)
        req_args.append(flasherd_pb2.Arg(binary=f'binary{ext}'))
        binaries = {f'binary{ext}': dl_file}

        if self._download_address is not None:
            req_args.append(flasherd_pb2.Arg(regular=f'0x{self._download_address:X}'))
        req_args += [flasherd_pb2.Arg(regular=arg) for arg in self._download_modifiers]

        # '--start' is needed to start execution after flash.
        # The default start address is the beggining of the flash,
        # but another value can be explicitly specified if desired.
        req_args.append(flasherd_pb2.Arg(regular='--start'))
        if self._start_address is not None:
            req_args.append(flasherd_pb2.Arg(regular=f'0x{self._start_address:X}'))
        req_args += [flasherd_pb2.Arg(regular=arg) for arg in self._start_modifiers]

        self.logger.info(f'Binary file: {dl_file}')

        command_req = flasherd_pb2.RunCommandRequest()
        command_req.CopyFrom(base_command)
        command_req.args.extend(req_args)
        client.launch_flasherd_command(self.logger, command_req, binaries)
