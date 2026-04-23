import socket
import struct
import time
from google.protobuf.internal.encoder import _VarintBytes
import clover_pb2

ZEPHYR_IP = '169.254.99.99'
ZEPHYR_PORT = 19690
DATA_PORT = 19691

def recv_response(s):
    length, shift = 0, 0
    while True:
        b = s.recv(1)
        if not b: raise ConnectionError("Hardware dropped connection")
        byte = b[0]
        length |= (byte & 0x7F) << shift
        if not (byte & 0x80): break
        shift += 7
    data = b''
    while len(data) < length:
        chunk = s.recv(length - len(data))
        if not chunk: raise ConnectionError("Incomplete TCP frame")
        data += chunk
    resp = clover_pb2.Response()
    resp.ParseFromString(data)
    return resp

def run_test():
    print(f"--- HARDWARE TEST ---")
    
    # 1. TCP Connection
    print(f"Connecting to {ZEPHYR_IP}:{ZEPHYR_PORT}...")
    try:
        tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_sock.settimeout(2.0)
        tcp_sock.connect((ZEPHYR_IP, ZEPHYR_PORT))
        print("TCP Connected!")
    except Exception as e:
        print(f"TCP Failed: {e}")
        return

    # 2. Subscribe Request
    req = clover_pb2.Request()
    req.subscribe_data_stream.SetInParent()
    raw = req.SerializeToString()
    payload = _VarintBytes(len(raw)) + raw
    
    print("Sending Subscribe Request...")
    tcp_sock.sendall(payload)
    resp = recv_response(tcp_sock)
    if resp.HasField("err"):
        print(f"Subscription REJECTED: {resp.err}")
        return
    print("Subscription ACK received!")

    # 3. Listen for UDP
    print(f"Listening for telemetry on port {DATA_PORT}...")
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind(('0.0.0.0', DATA_PORT))
    udp_sock.settimeout(5.0)

    start_time = time.time()
    packet_count = 0
    while time.time() - start_time < 10:
        try:
            data, addr = udp_sock.recvfrom(8192)
            packet_count += 1
            if packet_count == 1:
                print(f"Success! Received first packet: {len(data)} bytes from {addr}")
                packet = clover_pb2.DataPacket()
                packet.ParseFromString(data)
                print(f"Telemetry Sample: PT001={packet.analog_sensors.pt001:.2f}, TC101={packet.analog_sensors.tc101:.2f}")
        except socket.timeout:
            print("Timed out waiting for UDP packets...")
            break
    
    print(f"Test finished. Total packets received: {packet_count}")
    tcp_sock.close()
    udp_sock.close()

if __name__ == "__main__":
    run_test()
