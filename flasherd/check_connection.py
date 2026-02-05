import flasherd_pb2
import flasherd_pb2_grpc
import grpc
import sys

with grpc.insecure_channel('localhost:6767') as channel:
    stub = flasherd_pb2_grpc.FlasherdStub(channel)

    reqs = []
    reqs.append(flasherd_pb2.RunCommandRequest(
        command_windows='cmd',
        command_macos='echo',
        command_linux='echo',
        args=[flasherd_pb2.Arg(regular=arg) for arg in ['/c', 'dir']]
    ))

    for resp in stub.RunCommand((req for req in reqs)):
        if resp.HasField('exit_code') and resp.exit_code == 0:
            sys.exit(0)
sys.exit(1)
