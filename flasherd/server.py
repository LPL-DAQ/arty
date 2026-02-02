import flasherd_pb2
import flasherd_pb2_grpc
import platformdirs
import threading
from pathlib import Path
import subprocess
import sys
import grpc
from concurrent import futures

INIT_REQUEST_ID = 67
BINARIES_PATH = Path(platformdirs.user_runtime_dir('flasherd', 'lpl'))


class Flasherd(flasherd_pb2_grpc.FlasherdServicer):

    def __init__(self):
        super().__init__()
        BINARIES_PATH.mkdir(parents=True, exist_ok=True)

        max_id_found = 0
        for child in BINARIES_PATH.iterdir():
            max_id_found = max(max_id_found, int(child.parts[-1].split('_')[0]))

        self.req_id_lock = threading.Lock()
        self.next_req_id = max(max_id_found + 1, INIT_REQUEST_ID)

    def RunCommand(self, req_iter, context):
        self.req_id_lock.acquire()
        req_id = self.next_req_id
        self.next_req_id += 1
        self.req_id_lock.release()
        
        print(f'Request ID: {req_id}')

        for req in req_iter:
            print(req)
            if req.HasField('binary_name'):
                with open(BINARIES_PATH.joinpath(f'{req_id}_{req.binary_name}'), 'ab') as f:
                    f.write(req.binary_chunk)
            else:
                break

        if sys.platform.startswith('linux'):
            command = req.command_linux
        elif sys.platform.startswith('win32') or sys.platform.startswith('cygwin'):
            command = req.command_windows
        elif sys.platform.startswith('darwin'):
            command = req.command_macos
        else:
            raise Exception(f'Unknown OS: {sys.platform}')

        if req.HasField('stdin'):
            raise Exception(f'stdin streaming is currently unsupported')

        args = [command]
        for arg in req.args:
            if arg.HasField('regular'):
                args.append(arg.regular)
            else:
                args.append(str(BINARIES_PATH.joinpath(f'{req_id}_{arg.binary}')))
        print(f'Launching command: {args}')

        proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        for line in proc.stdout.readlines():
            print(f'Got line: {line}')
            yield flasherd_pb2.RunCommandResponse(stdout=line)
        exit_code = proc.wait()
        print(f'Got exit code: {exit_code}')
        yield flasherd_pb2.RunCommandResponse(exit_code=exit_code)


if __name__ == "__main__":
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    flasherd_pb2_grpc.add_FlasherdServicer_to_server(Flasherd(), server)
    server.add_insecure_port("localhost:6767")
    server.start()
    print('Server started on port 6767')
    server.wait_for_termination()
