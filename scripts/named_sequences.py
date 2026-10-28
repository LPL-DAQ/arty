"""Clover-framed named-sequence requests. Mutations are never automatically replayed."""
from google.protobuf.internal.encoder import _VarintBytes
import clover_pb2


class RequestRejected(ValueError):
    """The board decoded a request and returned a rejection without losing framing."""


def make_request(command):
    words = command.strip().split()
    if len(words) < 2 or words[0].lower() != 'sequence':
        raise ValueError('Use sequence list | run <name> | status | abort')
    operation = words[1].lower()
    if operation == 'abort' and len(words) == 2:
        request = clover_pb2.Request()
        request.abort.SetInParent()
        return request
    operations = {'list': clover_pb2.NamedSequenceRequest.LIST,
                  'run': clover_pb2.NamedSequenceRequest.RUN,
                  'status': clover_pb2.NamedSequenceRequest.STATUS}
    if operation not in operations or len(words) != (3 if operation == 'run' else 2):
        raise ValueError('Use sequence list | run <name> | status | abort')
    request = clover_pb2.Request()
    request.named_sequence.operation = operations[operation]
    if operation == 'run':
        request.named_sequence.name = words[2]  # Names are case sensitive.
    return request


def exchange(connection, request):
    """One transaction; caller must serialize access to connection. No retry on lost ACK."""
    raw = request.SerializeToString()
    connection.sendall(_VarintBytes(len(raw)) + raw)
    length = 0
    for shift in range(0, 35, 7):
        byte = connection.recv(1)
        if not byte:
            raise ConnectionError('Connection closed before acknowledgement; check status before retrying a run')
        length |= (byte[0] & 0x7f) << shift
        if not byte[0] & 0x80:
            break
    else:
        raise ValueError('Invalid response length prefix')
    if not 0 < length <= 4096:
        raise ValueError('Response exceeds protocol limit')
    body = bytearray()
    while len(body) < length:
        chunk = connection.recv(length - len(body))
        if not chunk:
            raise ConnectionError('Incomplete acknowledgement; check status before retrying a run')
        body.extend(chunk)
    response = clover_pb2.Response.FromString(bytes(body))
    if not response.IsInitialized():
        raise ValueError('Incomplete protocol response')
    if response.HasField('err'):
        raise RequestRejected(response.err)
    if not response.HasField('named_sequence'):
        raise ValueError('Firmware lacks named sequence support; flash the updated valve test firmware')
    return response.named_sequence
