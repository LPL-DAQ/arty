import socket
import threading
import sys
from rich.console import Console
from rich.prompt import Prompt
from rich.text import Text
import clover_pb2 # Your generated protobuf file

# Configuration from your prj.conf
ZEPHYR_IP = "169.254.99.99"
ZEPHYR_PORT = 5000  # <--- Verify this matches the port in your server.cpp!
LOCAL_PORT = 5001   # Port your python script listens on for incoming telemetry

console = Console()

# Dedicated UDP socket for incoming telemetry data stream
data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data_sock.bind(("0.0.0.0", LOCAL_PORT))

def listen_for_telemetry():
    """Background thread to print incoming data"""
    while True:
        try:
            data, addr = data_sock.recvfrom(4096)
            packet = clover_pb2.DataPacket()
            packet.ParseFromString(data)

            # Map enum to string for easier reading
            state_name = clover_pb2.SystemState.Name(packet.state)

            # Format output with Rich
            status_text = Text()
            status_text.append(f"[LIVE] ", style="bold green")
            status_text.append(f"State: {state_name} | ")
            status_text.append(f"Time: {packet.time:.2f}s | ")
            status_text.append(f"Fuel Pos: {packet.fuel_valve.encoder_pos_deg:.1f}°   ")

            # Print over the same line
            console.print(status_text, end="\r")

        except Exception as e:
            pass

def send_command(command_type):
    """Packs and sends a TCP request for commands"""
    req = clover_pb2.Request()

    if command_type == "start":
        req.start_sequence.SetInParent()
    elif command_type == "halt":
        req.halt_sequence.SetInParent()
    elif command_type == "closed_loop":
        req.start_throttle_closed_loop.SetInParent()

    # Commands are over a TCP socket per PR review
    try:
        cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cmd_sock.settimeout(2.0)
        cmd_sock.connect((ZEPHYR_IP, ZEPHYR_PORT))
        cmd_sock.sendall(req.SerializeToString())
        cmd_sock.close()
        console.print(f"\n[bold cyan][!][/bold cyan] Sent {command_type.upper()} command to {ZEPHYR_IP}:{ZEPHYR_PORT} via TCP\n")
    except Exception as e:
        console.print(f"\n[bold red][!] Failed to send {command_type.upper()} command via TCP: {e}[/bold red]\n")

if __name__ == "__main__":
    console.print(f"[bold blue]Connecting to Zephyr Board at {ZEPHYR_IP}...[/bold blue]")

    # Start the listening thread
    listener = threading.Thread(target=listen_for_telemetry, daemon=True)
    listener.start()

    # Main CLI loop
    while True:
        cmd = Prompt.ask("\nCommands: [bold]\[start][/bold] [bold]\[halt][/bold] [bold]\[closed_loop][/bold] [bold]\[quit][/bold] \n> ").strip().lower()
        if cmd == "quit":
            console.print("[bold yellow]Exiting...[/bold yellow]")
            break
        elif cmd in ["start", "halt", "closed_loop"]:
            send_command(cmd)
        else:
            console.print("[bold red]Unknown command. Try 'start', 'halt', 'closed_loop', or 'quit'.[/bold red]")
