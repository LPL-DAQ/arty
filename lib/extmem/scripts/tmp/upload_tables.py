import os
import socket
import time
import subprocess
import struct

ELF = "build_downloader/zephyr/zephyr.elf"
BIN = "tables.bin"
GDB = "/home/lpl/zephyr-sdk-0.17.2/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb"

RAM_ADDR = 0x20040000      # 可用 SRAM 區域
CHUNK_SIZE = 4096          # 必須和 MCU 一致

def run_gdb(commands):
    cmd_file = "tmp_upload.gdb"
    with open(cmd_file, "w") as f:
        for c in commands:
            f.write(c + "\n")

    subprocess.run([GDB, "-q", "-x", cmd_file, ELF])
    os.remove(cmd_file)

def upload():
    size = os.path.getsize(BIN)

    offset = 0
    while offset < size:
        chunk_size = min(CHUNK_SIZE, size - offset)

        with open(BIN, "rb") as f:
            f.seek(offset)
            chunk = f.read(chunk_size)

        tmp_chunk = "tmp_chunk.bin"
        with open(tmp_chunk, "wb") as f:
            f.write(chunk)

        cmds = [
            "target extended-remote :3333",
            "monitor reset halt",
            f"restore {tmp_chunk} binary 0x{RAM_ADDR:08X}",
            f"set var downloader_buf = (uint32_t)0x{RAM_ADDR:08X}",
            f"set var downloader_size = {chunk_size}",
            f"set var downloader_flash_offset = {offset}",
            "set var downloader_pending = 1",
            "detach",
            "quit"
        ]

        print(f"Uploading chunk @ offset {offset} size {chunk_size}")
        run_gdb(cmds)

        offset += chunk_size
        os.remove(tmp_chunk)

        time.sleep(0.2)

    print("Upload finished.")

if __name__ == "__main__":
    upload()
