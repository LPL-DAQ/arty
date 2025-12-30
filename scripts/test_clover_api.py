import subprocess
import socket
import base64
import time
import threading
from google.protobuf.internal.encoder import _VarintBytes

try:
    import clover_pb2 as clover_api

except ImportError as e:
    print(f'Failed to import clover protobuf bindings: {e}')
    print(f'Attempting rebuild...')
    subprocess.run(['protoc', '--python_out=scripts', 'api/clover.proto', '--proto_path=api'], cwd='/home/lpl/arty')

    import clover_pb2 as clover_api

    print(f'Loaded API bindings.')

IP = '169.254.99.99'
COMMAND_PORT = 19690
DATA_PORT = 19691


def send_request(sock: socket.socket, req: clover_api.Request) -> clover_api.Response:
    """Send a command request. sock must be an already-connected socket."""

    payload = req.SerializeToString()
    payload = _VarintBytes(len(payload)) + payload
    sock.sendall(payload)
    print(f'Payload sent (base64: {base64.b64encode(payload)})')

    # Receive response.
    # First, parse varint length prefix.
    resp_len = 0
    prefix_bytes_seen = 0
    while True:
        varint_byte = sock.recv(1)
        if len(varint_byte) != 1:
            raise Exception(f'Expected 1 byte from socket but got 0')
        varint_byte = int.from_bytes(varint_byte)

        # After dropping continuation byte, bytes are sent in little-endian order
        resp_len |= ((varint_byte & 0x7f) << (7 * prefix_bytes_seen))

        if varint_byte & 0x80 == 0:
            break
        prefix_bytes_seen += 1
    print(f'Got response of length {resp_len}')

    # Then, decode message.
    resp_raw = sock.recv(resp_len)
    if len(resp_raw) != resp_len:
        raise Exception(f'Expected {resp_len} byte from socket but got {len(resp_raw)}')

    resp = clover_api.Response()
    resp.ParseFromString(resp_raw)
    return resp


def handle_data_stream():
    """Listens for data packets."""

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('localhost', DATA_PORT))
        print('Data stream connected.')

        while True:
            (raw_packet, _) = sock.recvfrom(9999)
            data = clover_api.DataPacket()
            data.ParseFromString(raw_packet)

            print(f'Got data packet:\n{data}')


def handle_command_stream():
    """Sends commands and receives responses from the server."""

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((IP, COMMAND_PORT))
        print('Command stream connected.')

        # Identify self
        req = clover_api.Request()
        req.identify_client.client = clover_api.ClientType.GNC
        resp = send_request(sock, req)
        print(f'Got resp: {resp}')

        # Subscribe data stream
        req = clover_api.Request()
        req.subscribe_data_stream.SetInParent()
        resp = send_request(sock, req)
        print(f'Got resp: {resp}')

        while True:
            time.sleep(1000)


t1 = threading.Thread(target=handle_data_stream)
t2 = threading.Thread(target=handle_command_stream)

t1.start()
t2.start()

t1.join()
t2.join()
