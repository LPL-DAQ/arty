import flasherd_pb2
import flasherd_pb2_grpc
import grpc

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

        logger.debug(f'Windows command: {command_req.command_windows}')
        logger.debug(f'MacOS command: {command_req.command_macos}')
        logger.debug(f'Linux command: {command_req.command_linux}')
        logger.info(f'Launching command with args: {[(arg.regular if arg.HasField("regular") else f"{arg.binary} (binary)") for arg in command_req.args]}')
        logger.debug(f'Binaries: {binaries}')

        try:
            for resp in stub.RunCommand((req for req in reqs)):
                if resp.HasField('stdout'):
                    print(resp.stdout.decode(), end='')
                if resp.HasField('stderr'):
                    print(resp.stderr.decode(), file=sys.stderr, end='')
                if resp.HasField('exit_code'):
                    logger.info(f'Process terminated with code {resp.exit_code}')
                    break
        except grpc.RpcError as e:
            logger.error('Failed to run command via flasherd.')
            print(e)
            if e.code() == grpc.StatusCode.UNAVAILABLE:
                logger.error('Failed to connect to flasherd, it might not be running.')


