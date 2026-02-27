import socket
import threading
import sys
from rich.console import Console
from rich.prompt import Prompt
from rich.text import Text
from google.protobuf.internal.encoder import _VarintBytes

try:
    import clover_pb2 as clover_api
except ImportError as e:
    print(f"Failed to import clover protobuf bindings: {e}")
    print("Please ensure you have run the protoc compiler in your workspace.")
    sys.exit(1)

# Configuration from your lead's script
IP = '169.254.99.99'
COMMAND_PORT = 19690
DATA_PORT = 19691

console = Console()

def listen_for_telemetry():
    """Background thread to print incoming UDP data continuously on one line."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as data_sock:
        data_sock.bind(('0.0.0.0', DATA_PORT))
        while True:
            try:
                raw_packet, _ = data_sock.recvfrom(9999)
                packet = clover_api.DataPacket()
                packet.ParseFromString(raw_packet)

                state_name = clover_api.SystemState.Name(packet.state)

                # Format output with Rich
                status_text = Text()
                status_text.append(f"[LIVE] ", style="bold green")
                status_text.append(f"State: {state_name:15} | ")
                status_text.append(f"Time: {packet.time:8.2f}s | ")
                status_text.append(f"Fuel Pos: {packet.fuel_valve.encoder_pos_deg:6.1f}°   ")

                # Print over the same line without flooding the terminal
                console.print(status_text, end="\r")
            except Exception:
                pass

def send_request(sock: socket.socket, req: clover_api.Request) -> clover_api.Response:
    """Send a command request with varint length prefix and read response."""
    payload = req.SerializeToString()
    payload_with_header = _VarintBytes(len(payload)) + payload
    sock.sendall(payload_with_header)

    # Receive response (first parse varint length prefix)
    resp_len = 0
    prefix_bytes_seen = 0
    while True:
        varint_byte = sock.recv(1)
        if len(varint_byte) != 1:
            raise Exception('Expected 1 byte from socket but got 0')

        varint_byte = varint_byte[0] # Convert single byte to int safely
        resp_len |= (varint_byte & 0x7F) << (7 * prefix_bytes_seen)

        if varint_byte & 0x80 == 0:
            break
        prefix_bytes_seen += 1

    # Decode actual message response
    resp_raw = sock.recv(resp_len)
    if len(resp_raw) != resp_len:
        raise Exception(f'Expected {resp_len} byte from socket but got {len(resp_raw)}')

    resp = clover_api.Response()
    resp.ParseFromString(resp_raw)
    return resp

if __name__ == "__main__":
    console.print(f"[bold blue]Connecting to Zephyr Board at {IP}...[/bold blue]")

    # Start the UDP listener thread
    listener = threading.Thread(target=listen_for_telemetry, daemon=True)
    listener.start()

    # Establish the persistent TCP connection for Commands
    try:
        cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cmd_sock.connect((IP, COMMAND_PORT))
        console.print("[bold green]TCP Command stream connected.[/bold green]")

        # 1. Identify Client Handshake
        req = clover_api.Request()
        req.identify_client.client = clover_api.ClientType.GNC
        send_request(cmd_sock, req)

        # 2. Subscribe to Data Stream Handshake
        req = clover_api.Request()
        req.subscribe_data_stream.SetInParent()
        send_request(cmd_sock, req)

        console.print("[bold cyan]Handshake complete! Awaiting Telemetry...[/bold cyan]")

    except Exception as e:
        console.print(f"[bold red]Failed to connect to Teensy: {e}[/bold red]")
        sys.exit(1)

    # Main CLI Loop
    while True:
        prompt_str = r"\nCommands: [bold]\[load][/bold] [bold]\[start][/bold] [bold]\[halt][/bold] [bold]\[closed_loop][/bold] [bold]\[quit][/bold] \n> "

        try:
            cmd_input = Prompt.ask(prompt_str).strip().lower()
        except EOFError:
            break

        if cmd_input == "quit":
            console.print("[bold yellow]Exiting and closing connection...[/bold yellow]")
            break
        elif cmd_input in ["load", "start", "halt", "closed_loop"]:

            # Map input to the correct Protobuf Request Payload
            req = clover_api.Request()
            if cmd_input == "load":
                req.load_motor_sequence.SetInParent()
            elif cmd_input == "start":
                req.start_sequence.SetInParent()
            elif cmd_input == "halt":
                req.halt_sequence.SetInParent()
            elif cmd_input == "closed_loop":
                req.start_throttle_closed_loop.SetInParent()

            try:
                # Send the request over our open socket and wait for the Teensy's response
                resp = send_request(cmd_sock, req)

                # Check if the Teensy sent an error back in its response message
                if resp.HasField("err"):
                    console.print(f"\n[bold red][!] Server replied with error: {resp.err}[/bold red]\n")
                else:
                    console.print(f"\n[bold cyan][!][/bold cyan] Sent {cmd_input.upper()} successfully! Server confirmed.\n")

            except Exception as e:
                console.print(f"\n[bold red][!] Connection lost or error sending command: {e}[/bold red]\n")

        elif cmd_input == "":
            continue
        else:
            console.print("[bold red]Unknown command.[/bold red]")

    cmd_sock.close()
