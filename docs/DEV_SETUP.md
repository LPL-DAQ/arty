# Development Environment

The arty dev relies on [Dev Containers](https://containers.dev/) to maintain a consistent programming environment
across all developer machines, regardless of operating system. This guide contains all setup necessary
to 1) build and flash firmware, 2) run Python scripts for data processing or other utilities, and 3)
contribute code.

This guide is intended to be comprehensible for any engineer, even if they have limited programming
background.

## Local setup

### Install Docker Desktop

Docker Desktop provides a lightweight virtual machine-esque Linux environment for any desktop platform,
which is the backing technology for Dev Containers. Install it
as [this link](https://www.docker.com/products/docker-desktop/).

Open Docker Desktop, and click through the setup. **Do not create an account if it asks you, instead click skip.**

![Alt text](docker-setup.png)

We require host networking for many internal utilities. Ensure that you are on the latest version of Docker Desktop,
then ensure host networking is enabled by checking `Settings > Resources > Network > Enable host networking`.

On MacOS, ensure that x86 emulation via Rosetta 2 is enabled by checking `Settings > General > Virtual Machine Options > Use Rosetta for x86_64/amd64 emulation on Apple Silicon`.

### Install an IDE

Dev Containers rely heavily on an IDE, or Integrated Development Environment, for a smooth experience.
It is the primary means through which you'll write code and run programs.

If you are doing work in C++ with Zephyr, I highly recommend installing CLion, which you may do
from [this link](https://www.jetbrains.com/clion/download). Unlike Visual Studio Code,
it includes first-party support for Zephyr and for embedded debugging toolchains. Although it is
a paid app, they offer a free [educational license](https://www.jetbrains.com/shop/eform/students) for university
students.

If you only need this environment for light programming and scripting, consider installing Visual Studio Code
at [this link](https://code.visualstudio.com/download).

### Install git

git is a tool used to manage software collaboration. Install it [here](https://git-scm.com/install/).

### Install flasherd

`flasherd` is a host daemon allowing you to flash microcontorllers from within the dev container.
Installers for Windows and MacOS are available under `flasherd/dist`, or from LPL drive.

For Windows, simply run the `.msi` installer.

For MacOS, open the disk image and add the app bundle to your Applications folder. Unfortunately,
the application doesn't have a UI so simply double-clicking it looks pretty janky, as it'll bounce
forever (it should still work though). As an alternative, you may open a terminal and simply run
the following whenever you want to start `flasherd`:

```shell
/Applications/flasherd.app/Contents/MacOS/flasherd
```

### TODO: Remove stuff below

The board should start flashing at a slow, 1-second period, indicating that it's awaiting serial connection.

Open TyCommander, select the board, and enable serial logging. Once the serial console connects, the LED should start
quickly flashing as the application runs.

To rebuild and reflash, simply press the reset button and run `west flash`. As long as TyCommander is open, it should
reconnect serial automatically.

To connect to the test command server, run:

```shell
nc 169.254.99.99 19690
```

And to ensure the output is saved to a file, run:

```shell
nc 169.254.99.99 19690 | tee -a ~/arty/out.log
```

To live-parse sequences, run the following in a separate terminal tab as well:

```shell
tail -n 0 -f ~/arty/out.log | uv --project scripts run scripts/seq_splitter.py
```

To run the status-watcher script, run the following, making sure that the script's IP is correctly set:

```shell
uv --project scripts run scripts/watch_status.py
```
