import socket
import threading
import sys
import clover_pb2 # Your generated protobuf file

# Configuration from your prj.conf
ZEPHYR_IP = "169.254.99.99"
ZEPHYR_PORT = 5000  # <--- Verify this matches the port in your server.cpp!
LOCAL_PORT = 5001   # Port your python script listens on for incoming telemetry

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", LOCAL_PORT))

def listen_for_telemetry():
    """Background thread to print incoming data"""
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            packet = clover_pb2.DataPacket()
            packet.ParseFromString(data)

            # Map enum to string for easier reading
            state_name = clover_pb2.SystemState.Name(packet.state)

            # Use carriage return (\r) to print over the same line to avoid spamming the console
            sys.stdout.write(f"\r[LIVE] State: {state_name} | Time: {packet.time:.2f}s | Fuel Pos: {packet.fuel_valve.encoder_pos_deg:.1f}°   ")
            sys.stdout.flush()
        except Exception as e:
            pass

def send_command(command_type):
    """Packs and sends a simple UDP request"""
    req = clover_pb2.Request()

    if command_type == "start":
        req.start_sequence.SetInParent()
    elif command_type == "halt":
        req.halt_sequence.SetInParent()
    elif command_type == "closed_loop":
        req.start_closed_loop.SetInParent()

    sock.sendto(req.SerializeToString(), (ZEPHYR_IP, ZEPHYR_PORT))
    print(f"\n[!] Sent {command_type.upper()} command to {ZEPHYR_IP}:{ZEPHYR_PORT}\n")

if __name__ == "__main__":
    print(f"Connecting to Zephyr Board at {ZEPHYR_IP}...")

    # Start the listening thread
    listener = threading.Thread(target=listen_for_telemetry, daemon=True)
    listener.start()

    # Main CLI loop
    while True:
        cmd = input("\nCommands: [start] [halt] [closed_loop] [quit] \n> ").strip().lower()
        if cmd == "quit":
            print("Exiting...")
            break
        elif cmd in ["start", "halt", "closed_loop"]:
            send_command(cmd)
        else:
            print("Unknown command. Try 'start', 'halt', 'closed_loop', or 'quit'.")
