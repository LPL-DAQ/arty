import asyncio
import json
import os
import re
import socket
import threading
import time
import traceback
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn
from google.protobuf.internal.encoder import _VarintBytes
import clover_pb2

def parse_sequences(file_path):
    sequences = {}
    current_seq = None
    if not os.path.exists(file_path):
        return sequences
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: continue
            if line.startswith('[') and line.endswith(']'):
                # Some files contain merged headers like [alm][ignition].
                # Use the final token as the active section and ensure every
                # parsed token exists in the dictionary for compatibility.
                tokens = re.findall(r'\[([^\]]+)\]', line)
                if tokens:
                    for tok in tokens:
                        if tok not in sequences:
                            sequences[tok] = []
                    current_seq = tokens[-1]
                else:
                    current_seq = line[1:-1]
                    sequences[current_seq] = []
            elif current_seq is not None:
                parts = line.split()
                if len(parts) >= 3:
                    comp = parts[0]
                    time_val = float(parts[-1])
                    action = " ".join(parts[1:-1])
                    sequences[current_seq].append({"time": time_val, "component": comp, "action": action})
    return sequences

SEQUENCES = parse_sequences(os.path.join(os.path.dirname(__file__), "sequences.txt"))

def sequence_summaries():
    summaries = []
    for name, events in SEQUENCES.items():
        if not events:
            start_time = 0.0
            t_min, t_max = 0.0, 0.0
        else:
            times = [e["time"] for e in events]
            t_min, t_max = min(times), max(times)
            neg_times = [t for t in times if t < 0]
            start_time = abs(min(neg_times)) if neg_times else 0.0
        summaries.append({
            "id": name,
            "name": name,
            "startTime": float(start_time),
            "tMin": float(t_min),
            "tMax": float(t_max),
        })
    summaries.sort(key=lambda s: s["name"].lower())
    return summaries

# --- HARDWARE CONFIG ---
# Target 1: Vehicle Board
VEHICLE_IP = os.environ.get('VEHICLE_IP', '169.254.99.99')
VEHICLE_PORT = int(os.environ.get('VEHICLE_PORT', 19690))

# Target 2: GSE Board
GSE_IP = os.environ.get('GSE_IP', '169.254.99.100')
GSE_PORT = int(os.environ.get('GSE_PORT', 19690))

# Your computer's identity (Listening on all interfaces)
DATA_IP = '0.0.0.0' 
DATA_PORT = int(os.environ.get('DATA_PORT', 19691))

try:
    import clover_pb2
    HAS_CLOVER = True
except ImportError:
    HAS_CLOVER = False
    print("WARNING: clover_pb2 not found. Running in UI-only mode.")

app = FastAPI()

def _make_tcp_socket(ip, port) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2.0)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    s.connect((ip, port))
    return s

def _build_hw_request(comp: str, action: str):
    """Build the correct protobuf Request for a valve command dynamically.
    Searches clover_pb2 for matching Enums based on the provided component string.
    Returns (Request, board_target) or (None, None) if no HW mapping exists.
    """
    if not HAS_CLOVER:
        return None, None
    
    # Normalize ID (Remove dashes, handle common prefixes)
    clean_id = comp.replace("-", "").upper()
    board = "GSE" if clean_id.startswith("SVG") or clean_id.startswith("BVG") else "VEHICLE"

    # 1. Check for Throttle Valve (PBV) match in ThrottleValveType enum
    # Maps PBV101 -> LOX, PBV201 -> FUEL based on common usage or direct ID
    throttle_enum = None
    if "PBV101" in clean_id: throttle_enum = getattr(clover_pb2, "LOX", None)
    elif "PBV201" in clean_id: throttle_enum = getattr(clover_pb2, "FUEL", None)
    
    if throttle_enum is not None:
        req = clover_pb2.Request()
        req.throttle_reset_valve_position.valve = throttle_enum
        req.throttle_reset_valve_position.new_pos_deg = 0.0 if action == "CLOSE" else 90.0
        return req, board

    # 2. Check for Standard Valve (SV/SVG) match in Valve enum
    # Look for the exact name in clover_pb2 (e.g., clover_pb2.SV001)
    valve_val = getattr(clover_pb2, clean_id, None)
    if valve_val is not None:
        req = clover_pb2.Request()
        req.actuate_valve_request.valve = valve_val
        req.actuate_valve_request.state = clover_pb2.OPEN if action == "OPEN" else clover_pb2.CLOSED
        return req, board
    
    return None, None

class ValveManager:
    def __init__(self):
        self.active_connections = []
        self.valves = {
            # Vehicle Valves
            "SV001": "CLOSE", "SV002": "CLOSE", "SV003": "CLOSE", "SV004": "CLOSE", 
            "SV005": "CLOSE", "SV006": "CLOSE", "PBV101": "CLOSE", "PBV201": "CLOSE",
            "CV001": "CLOSE", "CV002": "CLOSE", "CV004": "CLOSE", "CV005": "CLOSE",
            "BV001": "CLOSE", "BV002": "CLOSE", "BV201": "CLOSE",
            # Keep both IGN001 and IGN002 for compatibility with existing
            # sequence/config files and current UI labels.
            "IGN001": "CLOSE", "IGN002": "CLOSE", "ALM001": "CLOSE",
            # GSE Valves
            "SVG001": "CLOSE", "SVG002": "CLOSE", "SVG003": "CLOSE", "BVG101": "CLOSE"
        }
        self.armed = {k: False for k in self.valves.keys()}
        # After first ACTUATE on armed valve: True (PWR ON). Second ACTUATE clears (PWR OFF).
        self.powered = {k: False for k in self.valves.keys()}
        self.gauges = {}
        self.tcp_lock_vehicle = threading.Lock()  # FIX #4: Separate locks per board
        self.tcp_lock_gse = threading.Lock()
        self.tcp_sock_vehicle = None
        self.tcp_sock_gse = None
        self.sequence_task = None

    def _safe_print(self, msg: str):
        # Clear the terminal line before printing to avoid telemetry overlap
        print(f"\r{msg.ljust(100)}")
        
    def _connect_tcp(self, board="VEHICLE"):
        ip = VEHICLE_IP if board == "VEHICLE" else GSE_IP
        port = VEHICLE_PORT if board == "VEHICLE" else GSE_PORT
        
        try:
            if board == "VEHICLE":
                if self.tcp_sock_vehicle: self.tcp_sock_vehicle.close()
                self.tcp_sock_vehicle = _make_tcp_socket(ip, port)
                self._safe_print(f"CONNECTED: VEHICLE TCP established with {ip}:{port}")
            else:
                if self.tcp_sock_gse: self.tcp_sock_gse.close()
                self.tcp_sock_gse = _make_tcp_socket(ip, port)
                self._safe_print(f"CONNECTED: GSE TCP established with {ip}:{port}")
        except Exception as e:
            self._safe_print(f"CONN ERROR: Could not reach {board} hardware ({ip}): {e}")
            if board == "VEHICLE": self.tcp_sock_vehicle = None
            else: self.tcp_sock_gse = None

    def _recv_response(self, sock):
        length, shift = 0, 0
        while True:
            b = sock.recv(1)
            if not b: raise ConnectionError("Hardware dropped connection")
            byte = b[0]
            length |= (byte & 0x7F) << shift
            if not (byte & 0x80): break
            shift += 7
        data = b''
        while len(data) < length:
            chunk = sock.recv(length - len(data))
            if not chunk: raise ConnectionError("Incomplete TCP frame")
            data += chunk
        resp = clover_pb2.Response()
        resp.ParseFromString(data)
        return resp

    def send_request(self, req, label: str, board: str = "VEHICLE"):
        if not HAS_CLOVER: return False
        # FIX #4: Use per-board lock so VEHICLE and GSE don't block each other
        lock = self.tcp_lock_vehicle if board == "VEHICLE" else self.tcp_lock_gse
        with lock:
            # Pick the right socket
            if board == "VEHICLE" and not self.tcp_sock_vehicle: self._connect_tcp("VEHICLE")
            if board == "GSE" and not self.tcp_sock_gse: self._connect_tcp("GSE")
            
            active_sock = self.tcp_sock_vehicle if board == "VEHICLE" else self.tcp_sock_gse
            if not active_sock: return False

            try:
                raw = req.SerializeToString()
                payload = _VarintBytes(len(raw)) + raw
                
                start_t = time.perf_counter()
                self._safe_print(f"TX [{board}] >> [{label}]")
                active_sock.sendall(payload)
                
                resp = self._recv_response(active_sock)
                dt_ms = (time.perf_counter() - start_t) * 1000
                if resp.HasField("err"):
                    self._safe_print(f"RX [{board}] << [{label}] REJECTED ({dt_ms:.1f}ms): {resp.err}")
                    return False
                self._safe_print(f"RX [{board}] << [{label}] ACK ({dt_ms:.1f}ms)")
                return True
            except Exception as e:
                self._safe_print(f"TCP ERROR: [{board}] [{label}] failed: {e}")
                if board == "VEHICLE": self.tcp_sock_vehicle = None
                else: self.tcp_sock_gse = None
                return False

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        await websocket.send_json({
            "valves": self.valves,
            "armed": self.armed,
            "powered": self.powered,
            "gauges": self.gauges,
            "sequences": sequence_summaries(),
        })
        print(f"\r{' '*100}\rWS: Client Connected")

    async def broadcast(self, message: dict):
        for connection in list(self.active_connections):
            try:
                await connection.send_json(message)
            except:
                self.active_connections.remove(connection)

    async def run_sequence(self, sequence_name, loop):
        seq = SEQUENCES.get(sequence_name)
        if not seq:
            self._safe_print(f"WARN: Sequence '{sequence_name}' not found.")
            return

        sorted_events = sorted(seq, key=lambda x: x["time"])
        self._safe_print(f"START: Sequence '{sequence_name}' ({len(sorted_events)} steps) [HIGH PRECISION MODE]")
        mission_start_perf = time.perf_counter()
        t0_perf = mission_start_perf - sorted_events[0]["time"]
        
        try:
            for event in sorted_events:
                target_perf_time = t0_perf + event["time"]
                
                # High-Precision Busy Wait
                while time.perf_counter() < target_perf_time:
                    # Brief yield to main loop to keep UI responsive and allow cancellation
                    await asyncio.sleep(0) 
                
                comp = event["component"].replace("-", "")
                action = event["action"]
                self._safe_print(f"[T={event['time']:+.3f}s] {comp} -> {action}")
                
                # Internal state update
                if comp in self.valves:
                    self.valves[comp] = action
                    await self.broadcast({"valves": {comp: action}})
                    
                    # FIX #5: Use _build_hw_request for ALL valve types
                    hw_req, target_board = _build_hw_request(comp, action)
                    if hw_req is not None:
                        asyncio.create_task(asyncio.to_thread(self.send_request, hw_req, f"SEQ_{comp}_{action}", target_board))
                    else:
                        target_board = "GSE" if comp.startswith("SVG") or comp.startswith("BVG") else "VEHICLE"
                        self._safe_print(f"TX [{target_board}] >> [SEQ_{comp}_{action}] (NO HW MAPPING)")

        except asyncio.CancelledError:
            self._safe_print(f"ABORT: Sequence '{sequence_name}' CANCELLED")
            raise

manager = ValveManager()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            payload = json.loads(data)
            action, name = payload.get("action"), payload.get("name")
            
            if action == "TOGGLE_VALVE" and name in manager.valves:
                # FIX #1: Use _build_hw_request for ALL valve types
                is_open = manager.valves[name] == "OPEN"
                new_state = "CLOSE" if is_open else "OPEN"
                
                manager.valves[name] = new_state
                manager.powered[name] = False
                await manager.broadcast({"valves": {name: new_state}, "powered": dict(manager.powered)})
                
                hw_req, target_board = _build_hw_request(name, new_state)
                if hw_req is not None:
                    asyncio.create_task(asyncio.to_thread(manager.send_request, hw_req, f"SET_{name}_{new_state}", target_board))
                else:
                    target_board = "GSE" if name.startswith("SVG") or name.startswith("BVG") else "VEHICLE"
                    manager._safe_print(f"TX [{target_board}] >> [SET_{name}_{new_state}] (NO HW MAPPING)")

            # ACTUATE: exactly one armed valve. First press = PWR ON + toggle; second = PWR OFF + toggle.
            elif action == "ACTUATE":
                armed_valves = [k for k, v in manager.armed.items() if v and k in manager.valves]
                if not armed_valves:
                    manager._safe_print("ACTUATE: No valve armed — ignoring.")
                elif len(armed_valves) > 1:
                    manager._safe_print("ACTUATE: Multiple armed valves — use single select only.")
                else:
                    V = armed_valves[0]
                    is_open = manager.valves[V] == "OPEN"
                    new_state = "CLOSE" if is_open else "OPEN"
                    manager.valves[V] = new_state
                    if not manager.powered.get(V, False):
                        manager.powered[V] = True
                    else:
                        manager.powered[V] = False

                    hw_req, target_board = _build_hw_request(V, new_state)
                    if hw_req is not None:
                        asyncio.create_task(asyncio.to_thread(manager.send_request, hw_req, f"ACTUATE_{V}_{new_state}", target_board))
                    else:
                        target_board = "GSE" if V.startswith("SVG") or V.startswith("BVG") else "VEHICLE"
                        manager._safe_print(f"TX [{target_board}] >> [ACTUATE_{V}_{new_state}] (NO HW MAPPING)")

                    await manager.broadcast({
                        "valves": {V: new_state},
                        "powered": dict(manager.powered),
                    })

            elif action == "ABORT":
                # FIX #3: Reset ALL valve states in UI on ABORT
                for k in manager.valves:
                    manager.valves[k] = "CLOSE"
                for k in manager.armed:
                    manager.armed[k] = False
                for k in manager.powered:
                    manager.powered[k] = False
                await manager.broadcast({
                    "valves": dict(manager.valves),
                    "armed": dict(manager.armed),
                    "powered": dict(manager.powered),
                })
                
                if manager.sequence_task:
                    manager.sequence_task.cancel()
                    manager.sequence_task = None
                
                req = clover_pb2.Request()
                req.abort.SetInParent()
                asyncio.create_task(asyncio.to_thread(manager.send_request, req, "ABORT", "VEHICLE"))
                asyncio.create_task(asyncio.to_thread(manager.send_request, req, "ABORT", "GSE"))
                
            elif action == "STOP_SEQUENCE":
                if manager.sequence_task:
                    manager.sequence_task.cancel()
                    manager.sequence_task = None
                req = clover_pb2.Request()
                req.halt.SetInParent()
                asyncio.create_task(asyncio.to_thread(manager.send_request, req, "HALT", "VEHICLE"))
                asyncio.create_task(asyncio.to_thread(manager.send_request, req, "HALT", "GSE"))

            elif action == "INITIATE_SEQUENCE":
                if manager.sequence_task:
                    manager.sequence_task.cancel()
                seq_name = payload.get("name")
                manager.sequence_task = asyncio.create_task(manager.run_sequence(seq_name, asyncio.get_event_loop()))
            elif action == "GET_SEQUENCES":
                await websocket.send_json({"sequences": sequence_summaries()})

            elif action == "ARM_VALVE" and name in manager.armed:
                # Single-select: only one valve armed; clicking same valve again disarms it.
                was_armed = manager.armed.get(name, False)
                for k in manager.armed:
                    manager.armed[k] = False
                for k in manager.powered:
                    manager.powered[k] = False
                manager.armed[name] = not was_armed
                await manager.broadcast({"armed": dict(manager.armed), "powered": dict(manager.powered)})

            elif action == "RESET":
                if manager.sequence_task:
                    manager.sequence_task.cancel()
                    manager.sequence_task = None
                for k in manager.valves:
                    manager.valves[k] = "CLOSE"
                for k in manager.armed:
                    manager.armed[k] = False
                for k in manager.powered:
                    manager.powered[k] = False
                await manager.broadcast({
                    "valves": dict(manager.valves),
                    "armed": dict(manager.armed),
                    "powered": dict(manager.powered),
                })

    except WebSocketDisconnect:
        manager.active_connections.remove(websocket)

def listen_for_telemetry(loop):
    if not HAS_CLOVER: return
    
    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    data_sock.bind((DATA_IP, DATA_PORT))
    data_sock.settimeout(0.5)
    
    last_packet_time = 0
    print(f"UDP: Listening for telemetry on {DATA_PORT}...")

    while True:
        # 1. AUTO-RESUBSCRIBE: If no data for 3 seconds, poke the hardware
        if (time.time() - last_packet_time) > 3.0:
            print("WARN: Telemetry stream lost. Resubscribing...")
            req = clover_pb2.Request()
            req.subscribe_data_stream.SetInParent()
            manager.send_request(req, "RE-SUBSCRIBE", "VEHICLE")
            manager.send_request(req, "RE-SUBSCRIBE", "GSE")
            last_packet_time = time.time() # Reset timer to avoid spamming TCP
        
        try:
            data, addr = data_sock.recvfrom(8192)
            last_packet_time = time.time()
            packet = clover_pb2.DataPacket()
            packet.ParseFromString(data)
            s = packet.analog_sensors
            gauges = {f.name.upper(): getattr(s, f.name) for f in s.DESCRIPTOR.fields}
            manager.gauges.update(gauges)
            
            # 3. VERBOSE LOGGING: Keep a rolling line in the terminal + Valve states
            fuel_cmd = packet.fuel_valve_command
            lox_cmd = packet.lox_valve_command
            fuel_s = f"{fuel_cmd.target_deg:.0f}°" if fuel_cmd.HasField("target_deg") else "OFF"
            lox_s = f"{lox_cmd.target_deg:.0f}°" if lox_cmd.HasField("target_deg") else "OFF"
            print(f"UDP RX: {len(data)}B | PT001: {gauges.get('PT001', 0):.1f} | FUEL: {fuel_s} | LOX: {lox_s}".ljust(80), end='\r')

            asyncio.run_coroutine_threadsafe(
                manager.broadcast({"gauges": gauges}), 
                loop
            )
            
        except socket.timeout:
            continue
        except Exception as e:
            print(f"\r{' '*100}\rTELEMETRY ERROR: {e}")
            time.sleep(0.1)

@app.on_event("startup")
async def startup_event():
    loop = asyncio.get_running_loop()
    threading.Thread(target=listen_for_telemetry, args=(loop,), daemon=True).start()

if __name__ == "__main__":
    # Note: Binding to 0.0.0.0 so external UI devices can reach your laptop
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")