from __future__ import annotations

import importlib.util
import sys

# HACK: Zephyr imports runners one by one with an identical module name each time. As its module name is shadowed, other
# runners have no way to refer to this module. We bypass this by manually making this file available for import under
# another name when it's first executed. This only works if this file is specified first in `zephyr/module.yml`.
MODULES = [
    ('flasherd_pb2', '/home/lpl/arty/flasherd/flasherd_pb2.py'),
    ('flasherd_pb2_grpc', '/home/lpl/arty/flasherd/flasherd_pb2_grpc.py'),
    ('client', '/home/lpl/arty/flasherd/client.py')
]

for name, file in MODULES:
    spec = importlib.util.spec_from_file_location(name, file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
