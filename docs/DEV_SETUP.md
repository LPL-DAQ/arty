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

Also ensure that the daemon is accessible over TCP by checking `Expose daemon on tcp://localhost:2375 without TLS`.

On MacOS, ensure that x86 emulation via Rosetta 2 is enabled by checking `Settings > General > Virtual Machine Options > Use Rosetta for x86_64/amd64 emulation on Apple Silicon`.

### Setup repo

Install Visual Studio Code. Then, install the Dev Containers extension in VSCode.

Open the command palette with `Ctrl-Shift-P` or `Cmd-Shift-P` and search for the action:
"Dev Containers: Clone Repository in Container Volume". Select this with `Enter`.

Choose the option to "Clone a repository from Github..." and authorize VSCode to use
your Github account credentials.

In the list that pops up, select the `arty` repo.

The dev container will now begin downloading. You may view the progress of this by select "Show container logs" in the
bottom-right popup.


