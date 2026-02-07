import flasherd_pb2
import flasherd_pb2_grpc
import grpc
import sys
import os

def launch_flasherd_command(logger, command_req: flasherd_pb2.RunCommandRequest, binaries: dict[str, str]):
    """
    command_req is the request specifying the main command and args to run.
    binaries should map binary_name to the dev container path to binaries to stream beforehand.
    """

    with grpc.insecure_channel('localhost:6767') as channel:
        stub = flasherd_pb2_grpc.FlasherdStub(channel)
        reqs = []

        for name, file_path in binaries.items():
            with open(file_path, 'rb') as f:
                for block in iter(lambda: f.read(4096), b''):
                    reqs.append(flasherd_pb2.RunCommandRequest(binary_name=name, binary_chunk=block))
        reqs.append(command_req)

        command_name = os.path.basename(command_req.command_linux)  # Usually this one is the most minimally formatted.
        logger.info(f'Launching command `{command_name}` with args: {[(arg.regular if arg.HasField("regular") else f"{arg.binary} (binary)") for arg in command_req.args]}')
        if len(binaries) != 0:
            logger.info(f'Binaries: {binaries}')
            
        try:
            for resp in stub.RunCommand((req for req in reqs)):
                if resp.HasField('stdout'):
                    sys.stdout.buffer.write(resp.stdout)
                if resp.HasField('stderr'):
                    sys.stdout.buffer.write(resp.stderr)
                if resp.HasField('exit_code'):
                    logger.info(f'Process terminated with code {resp.exit_code}')
                    break
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.UNAVAILABLE:
                logger.error('Failed to connect to flasherd, it might not be running.')
                
                # Bail out without printing a massive stacktrace; this keeps the advice that "it might not be running" more apparent to end user.
                sys.exit(1)
            else:
                logger.error('Failed to run command via flasherd: {e}')
                raise e


