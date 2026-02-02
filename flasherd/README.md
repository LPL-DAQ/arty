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

### Windows

Run the following in a terminal from the root of the repo.

```shell
uv --project flasherd run flasherd/setup.py bdist_msi --dist-dir flasherd/dist
```

This will produce a Windows installer under `flasherd/dist`.
