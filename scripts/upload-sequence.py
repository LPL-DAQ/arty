#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["protobuf"]
# ///

"""Upload or explicitly run an autonomous valve sequence on a Clover controller."""

import argparse
import pathlib
import socket
import sys

import clover_pb2
from google.protobuf.internal.encoder import _VarintBytes

DEFAULT_IP = '169.254.99.99'
COMMAND_PORT = 19690
MAX_FILE_SIZE = 8 * 1024
CHUNK_SIZE = 512
MAX_RUNS = 64


def receive_exact(sock: socket.socket, count: int) -> bytes:
    """Read exactly count bytes from sock.

    Parameters: sock is the connected controller socket; count is the byte count.
    Returns: the received bytes.
    """
    data = bytearray()
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError('controller closed the connection while sending a response')
        data.extend(chunk)
    return bytes(data)


def receive_varint(sock: socket.socket) -> int:
    """Read a protobuf varint length prefix from sock.

    Parameters: sock is the connected controller socket.
    Returns: the decoded message length.
    """
    result = 0
    shift = 0
    while shift < 35:
        byte = receive_exact(sock, 1)[0]
        result |= (byte & 0x7F) << shift
        if byte < 0x80:
            return result
        shift += 7
    raise ValueError('response length prefix is invalid')


def send_request(sock: socket.socket, request: clover_pb2.Request) -> None:
    """Send a length-delimited request and raise on a controller error.

    Parameters: sock is the connected controller socket; request is the protobuf request.
    Returns: None.
    """
    payload = request.SerializeToString()
    sock.sendall(_VarintBytes(len(payload)) + payload)
    response = clover_pb2.Response()
    response_size = receive_varint(sock)
    if response_size > 1024:
        raise ValueError('controller response is unexpectedly large')
    response.ParseFromString(receive_exact(sock, response_size))
    if response.HasField('err'):
        raise RuntimeError(response.err)


def validate_file_name(file_name: str) -> None:
    """Validate a sequence filename against the controller's basename rules.

    Parameters: file_name is the proposed .log basename.
    Returns: None; raises ValueError if the name is invalid.
    """
    if (
        not file_name
        or not file_name.isascii()
        or len(file_name) > 48
        or '..' in file_name
        or not file_name.endswith('.log')
        or any(not (char.isalnum() or char in '_.-') for char in file_name)
    ):
        raise ValueError('sequence filename must be a valid .log basename (letters, digits, _, -, . only)')


def upload(sock: socket.socket, path: pathlib.Path, run_index: int) -> None:
    """Upload and validate a sequence file without starting it.

    Parameters: sock is the connected controller socket; path is the local log file;
    run_index selects the zero-based ignition run.
    Returns: None.
    """
    file_name = path.name
    validate_file_name(file_name)
    contents = path.read_bytes()
    if not contents or len(contents) > MAX_FILE_SIZE:
        raise ValueError(f'sequence file must be between 1 and {MAX_FILE_SIZE} bytes')
    try:
        text = contents.decode('ascii')
    except UnicodeDecodeError as error:
        raise ValueError('sequence log must contain ASCII text') from error
    if '\0' in text:
        raise ValueError('sequence log must not contain null bytes')

    for offset in range(0, len(contents), CHUNK_SIZE):
        chunk = contents[offset : offset + CHUNK_SIZE].decode('ascii')
        request = clover_pb2.Request()
        request.upload_autonomous_valve_sequence_chunk.sequence_file = file_name
        request.upload_autonomous_valve_sequence_chunk.offset = offset
        request.upload_autonomous_valve_sequence_chunk.total_size = len(contents)
        request.upload_autonomous_valve_sequence_chunk.data = chunk
        request.upload_autonomous_valve_sequence_chunk.final_chunk = offset + len(chunk) == len(contents)
        request.upload_autonomous_valve_sequence_chunk.run_index = run_index
        send_request(sock, request)
        print(f'Uploaded {min(offset + len(chunk), len(contents))}/{len(contents)} bytes', end='\r')
    print(f'Uploaded and validated {file_name}, run {run_index}; it is in device RAM until reboot.')


def run(sock: socket.socket, file_name: str, run_index: int, continue_valve_states: bool) -> None:
    """Request execution of an uploaded or SD-card sequence.

    Parameters: sock is the connected controller socket; file_name is the log basename;
    run_index selects the run; continue_valve_states chooses successful-completion behavior.
    Returns: None.
    """
    validate_file_name(file_name)
    request = clover_pb2.Request()
    request.run_autonomous_valve_sequence.sequence_file = file_name
    request.run_autonomous_valve_sequence.run_index = run_index
    if continue_valve_states:
        request.run_autonomous_valve_sequence.continue_valve_states = True
    send_request(sock, request)
    print(f'Requested run {run_index} of {file_name}.')


def main() -> int:
    """Parse command-line arguments and perform the requested controller operation.

    Parameters: None; arguments are read from the process command line.
    Returns: zero on success and one on a controller or input error.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ip', default=DEFAULT_IP, help=f'controller IP address (default: {DEFAULT_IP})')
    subparsers = parser.add_subparsers(dest='command', required=True)
    upload_parser = subparsers.add_parser('upload', help='upload a .log file without running it')
    upload_parser.add_argument('file', type=pathlib.Path)
    upload_parser.add_argument('--run-index', type=int, default=0)
    run_parser = subparsers.add_parser('run', help='explicitly run an uploaded or SD-card sequence')
    run_parser.add_argument('file_name', help='sequence filename, for example ignition.log')
    run_parser.add_argument('--run-index', type=int, default=0)
    run_parser.add_argument(
        '--continue-valve-states',
        action='store_true',
        help='leave the valves at their final commanded states after normal completion (default: restore starting safe states)',
    )
    args = parser.parse_args()

    if not 0 <= args.run_index < MAX_RUNS:
        parser.error(f'--run-index must be between 0 and {MAX_RUNS - 1}')

    try:
        with socket.create_connection((args.ip, COMMAND_PORT), timeout=10) as sock:
            if args.command == 'upload':
                upload(sock, args.file, args.run_index)
            else:
                run(sock, args.file_name, args.run_index, args.continue_valve_states)
    except (OSError, RuntimeError, ValueError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
