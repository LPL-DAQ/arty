# flasherd

A host daemon that communicates with processes within the dev container to call MCU flashing toolchains.
Effectively just a grpc server that launches processes on behalf of requests and streams the output.
Integrates with zephyr runners for smooth integration with the rest of the development ecosystem.

## Build

We use cx_Freeze to build the python grpc server into standalone, installable apps. As it doesn't
support cross-compilation, we must unfortunately run specific build commands in each of our
supported OS's.

Builds are configured via the script at `flasherd/cxfreeze_setup.py`. Each time the flasherd server
is updated, ensure that the `VERSION` number in that script is bumped.

### Common setup

Ensure `uv` is installed on the host; instructions to do so are [here](https://docs.astral.sh/uv/getting-started/installation/).

Clone the `arty` repo somewhere on the host.

> **Run these build commands in a host terminal** (macOS Terminal/iTerm, Windows PowerShell/cmd),
> **not** the VSCode Dev Container terminal. The dev container is always Linux, and cx_Freeze only
> registers `bdist_dmg`/`bdist_msi` when it's actually running on macOS/Windows, so building from
> inside the container fails with `error: invalid command 'bdist_dmg'` (or `'bdist_msi'`).

### Windows

Run the following in a host terminal from the root of the repo.

```shell
uv --project flasherd run flasherd/cxfreeze_setup.py bdist_msi --dist-dir flasherd/dist
```

This will produce a Windows installer under `flasherd/dist`.

### MacOS

Run the following in a host terminal from the root of the repo.

```shell
uv --project flasherd run flasherd/cxfreeze_setup.py bdist_dmg && mv build/flasherd.dmg flasherd/dist/flasherd.dmg
```

This builds a disk image installer and moves it under `flasherd/dist`.

## Connection test

Run the following from within the dev container to check that flasherd is running:

```shell
if uv --project flasherd run flasherd/check_connection.py; then echo -e "\\e[32;1mflasherd is active\\e[0m"; else echo -e "\\e[31;1mflasherd is inactive\\e[0m"; fi
```

## Flashing Teensy boards (Ranger 1 / HIVE)

`west flash` on Teensy boards uses the `tycmd_flasherd` runner, which has flasherd run
`tycmd upload --nocheck zephyr.hex` **on the host**. `tycmd` is not installed with flasherd.

### Install tycmd (host, one time)

- **macOS:** there is no Homebrew package. Download `tytools_X.Y.Z_osx.dmg` from the
  [TyTools releases](https://github.com/Koromix/tytools/releases), open it, and copy the `tycmd`
  binary onto your `PATH` (e.g. `/opt/homebrew/bin`), then `chmod +x` it. Check with `which tycmd`.
- **Windows:** install TyTools with its installer. The runner expects
  `C:\Program Files (x86)\TyTools\tycmd.exe`.

Without it, `west flash` fails with `[Errno 2] No such file or directory: 'tycmd'`.

### Flash

1. **Press the PROGRAM button on the Teensy first.** Our Zephyr firmware doesn't respond to
   tycmd's software reboot request, so without the button tycmd fails with `Cannot reboot board`.
   `tycmd list` should now show the board as `Teensy 4.1 (HalfKay)` (bootloader).
2. Run `west flash` from the dev container.
3. tycmd often ends with `Board '...' has disappeared` and exit code 1 even though the flash worked.
   Confirm with `tycmd list`: the board should come back as `Ranger 1`. If it's still
   `(HalfKay)`, just run `west flash` again (no button needed).
