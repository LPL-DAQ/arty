from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, override

import flasherd_pb2
import flasherd_pb2_grpc

from runners.stm32cubeprogrammer import STM32CubeProgrammerBinaryRunner

if TYPE_CHECKING:
    import argparse

    from runners.core import RunnerConfig


class STM32CubeProgrammerFlasherdBinaryRunner(STM32CubeProgrammerBinaryRunner):
    """Extends the Zephyr base STM32CubeProgrammerBinaryRunner to use flasherd."""

    def __init__(
        self,
        cfg: RunnerConfig,
        port: str,
        frequency: int | None,
        reset_mode: str | None,
        download_address: int | None,
        download_modifiers: list[str],
        start_address: int | None,
        start_modifiers: list[str],
        conn_modifiers: str | None,
        use_elf: bool, # noqa: FBT001
        erase: bool, # noqa: FBT001
        extload: str | None,
        tool_opt: list[str],
    ) -> None:
        super().__init__(
            cfg,
            port,
            frequency,
            reset_mode,
            download_address,
            download_modifiers,
            start_address,
            start_modifiers,
            conn_modifiers,
            Path('flasherd-client'),
            use_elf,
            erase,
            extload,
            tool_opt,
        )

    @classmethod
    @override
    def name(cls):
        return 'stm32cubeprogrammer_flasherd'

    @classmethod
    @override
    def do_create(
        cls, cfg: RunnerConfig, args: argparse.Namespace
    ) -> STM32CubeProgrammerFlasherdBinaryRunner:
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
            use_elf=args.use_elf,
            erase=args.erase,
            extload=args.extload,
            tool_opt=args.tool_opt,
        )

    def flash(self, **kwargs) -> None:
        self.require(str(self._cli))

        # Prepare base command
        cmd = FlasherdCommand('flasherd-client')
        cmd.add_windows_command(
            r'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
        )
        cmd.add_macos_command(
            '/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI'
        )
        cmd.add_linux_command(
            '~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI'
        )

        connect_opts = f'port={self._port}'
        if self._frequency:
            connect_opts += f' freq={self._frequency}'
        if self._reset_mode:
            reset_mode = STM32CubeProgrammerBinaryRunner._RESET_MODES[self._reset_mode]
            connect_opts += f' reset={reset_mode}'
        if self._conn_modifiers:
            connect_opts += f' {self._conn_modifiers}'

        cmd += ['--connect', connect_opts]
        cmd += self._tool_opt
        if self._extload:
            # external loader to come after the tool option in STM32CubeProgrammer
            cmd += self._extload

        # erase first if requested
        if self._erase:
            erase_cmd = cmd + ['--erase', 'all']
            print(erase_cmd.payload)
            self.check_call(erase_cmd.payload)

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

        cmd += '--download'
        cmd.add_path_arg(dl_file)
        if self._download_address is not None:
            cmd += f'0x{self._download_address:X}'
        cmd += self._download_modifiers

        # '--start' is needed to start execution after flash.
        # The default start address is the beggining of the flash,
        # but another value can be explicitly specified if desired.
        cmd.append('--start')
        if self._start_address is not None:
            cmd.append(f'0x{self._start_address:X}')
        cmd += self._start_modifiers

        self.check_call(cmd.payload)
