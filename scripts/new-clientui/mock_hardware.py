import socket
import threading
import time
import math
import random

try:
    import clover_pb2
    from google.protobuf.internal.encoder import _VarintBytes
except ImportError:
    print("Need clover_pb2 to run simulator.")
    exit(1)

TCP_PORT = 19690
UDP_PORT = 19691
UDP_DEST = "127.0.0.1"

# Shared mock state
valves = {"fuel": False, "lox": False}

def run_tcp_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", TCP_PORT))
    server.listen(1)
    print(f"[Hardware] TCP Server listening on {TCP_PORT}")
    
    while True:
        conn, addr = server.accept()
        print(f"[Hardware] Connected to backend at {addr}")
        try:
            while True:
                # Basic Varint len reader (simplified for testing)
                b = conn.recv(1)
                if not b: break
                length = 0
                shift = 0
                while True:
                    byte = b[0]
                    length |= (byte & 0x7F) << shift
                    if not (byte & 0x80): break
                    shift += 7
                    b = conn.recv(1)
                
                data = conn.recv(length)
                req = clover_pb2.Request()
                req.ParseFromString(data)
                
                print(f"[Hardware] Got command: {req}")
                
                # Mock handling valve commands
                if req.HasField("reset_valve_position"):
                    is_open = req.reset_valve_position.new_pos_deg > 45.0
                    v = req.reset_valve_position.valve
                    if v == clover_pb2.FUEL: valves["fuel"] = is_open
                    elif v == clover_pb2.LOX: valves["lox"] = is_open

                # Return success response
                resp = clover_pb2.Response()
                raw = resp.SerializeToString()
                conn.sendall(_VarintBytes(len(raw)) + raw)
        except Exception as e:
            print(f"[Hardware] TCP connection ending: {e}")
        finally:
            conn.close()

def run_udp_telemetry():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[Hardware] Streaming UDP telemetry to {UDP_DEST}:{UDP_PORT}...")
    t = 0
    seq = 0
    while True:
        packet = clover_pb2.DataPacket()
        
        # Required primitives
        packet.time_ns = int(time.time() * 1e9)
        packet.state = clover_pb2.STATE_IDLE
        packet.data_queue_size = 0
        packet.sequence_number = seq
        packet.gnc_connected = True
        packet.gnc_last_pinged_ns = 0.0
        packet.daq_connected = True
        packet.daq_last_pinged_ns = 0.0
        
        # Required sub-messages
        packet.controller_timing.controller_tick_time_ns = 1e6
        packet.controller_timing.analog_sensors_sense_time_ns = 1e6
        packet.controller_timing.lidar_sense_time_ns = 1e6
        
        for v_name, v_status in [("fuel", packet.fuel_valve), ("lox", packet.lox_valve)]:
            pos = 90.0 if valves[v_name] else 0.0
            v_status.target_pos_deg = pos
            v_status.driver_setpoint_pos_deg = pos
            v_status.encoder_pos_deg = pos
            v_status.is_on = valves[v_name]

        # Mock sensors
        packet.analog_sensors.pt001 = 10.0 + math.sin(t)
        packet.analog_sensors.pt101 = 700.0 + random.uniform(-2, 2)
        
        sock.sendto(packet.SerializeToString(), (UDP_DEST, UDP_PORT))
        time.sleep(0.1) # 10 Hz
        t += 0.1
        seq += 1

if __name__ == "__main__":
    threading.Thread(target=run_tcp_server, daemon=True).start()
    run_udp_telemetry()
