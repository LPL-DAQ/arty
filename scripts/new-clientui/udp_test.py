import socket
import time

DATA_IP = '0.0.0.0'
DATA_PORT = 19691

print(f"Listening for UDP on {DATA_IP}:{DATA_PORT}...")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((DATA_IP, DATA_PORT))
sock.settimeout(5.0)

try:
    while True:
        try:
            data, addr = sock.recvfrom(8192)
            print(f"Received {len(data)} bytes from {addr}")
        except socket.timeout:
            print("No packets received in 5 seconds...")
except KeyboardInterrupt:
    print("Stopped.")
finally:
    sock.close()
