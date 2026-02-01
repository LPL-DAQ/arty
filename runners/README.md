# `flasherd`-based west runners

Integrates with `flasherd` to flash microcontrollers across the container barrier. These effectively
copy a compiled binary to the host, then call a host flashing utility to flash the device.

The runners incorporate a `flasherd` client. It is regenerated with:

```shell
uv run python -m grpc_tools.protoc -I api --python_out=runners --pyi_out=runners --grpc_python_out=runners api/flasherd.proto
```
