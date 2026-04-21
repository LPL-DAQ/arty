import asyncio
import json
import os
import socket
import threading
import time
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn
from google.protobuf.internal.encoder import _VarintBytes
import traceback
try:
    import clover_pb2
    HAS_CLOVER = True
except ImportError:
    HAS_CLOVER = False
    print("WARNING: clover_pb2 not found. The server will run but hardware integration will fail.")

ZEPHYR_IP = os.environ.get('ZEPHYR_IP', '169.254.99.99')
ZEPHYR_PORT = int(os.environ.get('ZEPHYR_PORT', 19690))
DATA_IP = os.environ.get('DATA_IP', '0.0.0.0')
DATA_PORT = int(os.environ.get('DATA_PORT', 19691))

app = FastAPI()

def _make_tcp_socket() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2.0)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
    s.connect((ZEPHYR_IP, ZEPHYR_PORT))
    return s

class ValveManager:
    def __init__(self):
        self.active_connections = []
        self.valves = {
            "PBV001": "CLOSE", "PBV002": "CLOSE", "PBV003": "CLOSE", 
            "PBV004": "CLOSE", "PBV005": "CLOSE", "PBV006": "CLOSE",
            "PBV101": "CLOSE", "PBV201": "CLOSE", "PBV301": "CLOSE", "PBV302": "CLOSE",
            "IGN002": "CLOSE", "ALM001": "CLOSE"
        }
        self.armed = {k: False for k in self.valves.keys()}
        self.gauges = {}
        self.tcp_lock = threading.Lock()
        self.tcp_sock = None
        if HAS_CLOVER:
            self._connect_tcp()
        
    def _connect_tcp(self):
        try:
            if self.tcp_sock:
                self.tcp_sock.close()
        except: pass
        try:
            self.tcp_sock = _make_tcp_socket()
            print("Connected to Zephyr TCP.")
        except Exception as e:
            print(f"Failed to connect to Zephyr TCP: {e}")
            self.tcp_sock = None

    def _recv_response(self):
        length = 0
        shift = 0
        while True:
            b = self.tcp_sock.recv(1)
            if not b:
                raise ConnectionError("Connection closed while reading response length")
            byte = b[0]
            length |= (byte & 0x7F) << shift
            if not (byte & 0x80):
                break
            shift += 7

        data = b''
        while len(data) < length:
            chunk = self.tcp_sock.recv(length - len(data))
            if not chunk:
                raise ConnectionError("Connection closed while reading response body")
            data += chunk

        resp = clover_pb2.Response()
        resp.ParseFromString(data)
        return resp

    def send_request(self, req, label: str):
        if not HAS_CLOVER:
            return False
            
        with self.tcp_lock:
            if not self.tcp_sock:
                self._connect_tcp()
            if not self.tcp_sock:
                print(f"[{label}] Failed to send: TCP sock not connected.")
                return False
            
            raw = req.SerializeToString()
            payload = _VarintBytes(len(raw)) + raw
            
            try:
                self.tcp_sock.sendall(payload)
                resp = self._recv_response()
                if resp.HasField("err"):
                    print(f"[{label}] Rejected: {resp.err}")
                    return False
                print(f"[{label}] Sent successfully.")
                return True
            except Exception as e:
                print(f"[{label}] Error sending: {e}")
                self.tcp_sock = None # force reconnect next time
                return False

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        await websocket.send_json({
            "valves": self.valves,
            "armed": self.armed,
            "gauges": self.gauges
        })
        print("Client Connected!")

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        print("Client Disconnected!")

    async def broadcast(self, message: dict):
        dead_connections = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception:
                dead_connections.append(connection)
        for dc in dead_connections:
            self.active_connections.remove(dc)

manager = ValveManager()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            payload = json.loads(data)
            action = payload.get("action")
            name = payload.get("name")
            
            print(f"UI Command Received -> {action} {name if name else ''}")
            
            if action == "TOGGLE_VALVE" and name in manager.valves:
                valve_enum = None
                valve_label = ""
                if HAS_CLOVER:
                    if name == "PBV201":
                        valve_enum = clover_pb2.FUEL
                        valve_label = "FUEL"
                    elif name == "PBV101":
                        valve_enum = clover_pb2.LOX
                        valve_label = "LOX"
                is_currently_open = (manager.valves[name] == "OPEN")
                new_state = "CLOSE" if is_currently_open else "OPEN"
                manager.valves[name] = new_state
                await manager.broadcast({"valves": {name: new_state}})
                if valve_enum is not None:
                    req = clover_pb2.Request()
                    if is_currently_open:
                        req.reset_valve_position.valve = valve_enum
                        req.reset_valve_position.new_pos_deg = 0.0
                        manager.send_request(req, f"CLOSE_VALVE ({valve_label})")
                    else:
                        req.reset_valve_position.valve = valve_enum
                        req.reset_valve_position.new_pos_deg = 90.0
                        manager.send_request(req, f"OPEN_VALVE ({valve_label})")
                
            elif action == "ARM_VALVE" and name in manager.armed:
                manager.armed = {k: (k == name and not manager.armed[k]) for k in manager.armed}
                await manager.broadcast({"armed": manager.armed})
            
            elif action == "ACTUATE":
                armed_valve = next((k for k, v in manager.armed.items() if v), None)
                if armed_valve:
                    is_currently_open = (manager.valves[armed_valve] == "OPEN")
                    new_state = "CLOSE" if is_currently_open else "OPEN"
                    manager.valves[armed_valve] = new_state
                    manager.armed[armed_valve] = False
                    await manager.broadcast({
                        "valves": {armed_valve: new_state},
                        "armed": manager.armed
                    })
                    
                    if HAS_CLOVER:
                        valve_enum = None
                        if armed_valve == "PBV201":
                            valve_enum, valve_label = clover_pb2.FUEL, "FUEL"
                        elif armed_valve == "PBV101":
                            valve_enum, valve_label = clover_pb2.LOX, "LOX"
                            
                        if valve_enum is not None:
                            req = clover_pb2.Request()
                            if is_currently_open:
                                req.reset_valve_position.valve = valve_enum
                                req.reset_valve_position.new_pos_deg = 0.0
                                manager.send_request(req, f"CLOSE_VALVE ({valve_label})")
                            else:
                                req.reset_valve_position.valve = valve_enum
                                req.reset_valve_position.new_pos_deg = 90.0
                                manager.send_request(req, f"OPEN_VALVE ({valve_label})")

            elif action == "INITIATE_SEQUENCE":
                seq_name = payload.get("name")
                if seq_name == "ignition" and HAS_CLOVER:
                    req = clover_pb2.Request()
                    req.start_thrust_sequence.SetInParent()
                    manager.send_request(req, "START_THRUST_SEQUENCE")
                
            elif action == "ABORT":
                if HAS_CLOVER:
                    req = clover_pb2.Request()
                    req.abort.SetInParent()
                    manager.send_request(req, "ABORT")
                    
                manager.valves = {k: "CLOSE" for k in manager.valves}
                manager.armed = {k: False for k in manager.armed}
                await manager.broadcast({"valves": manager.valves, "armed": manager.armed})

    except WebSocketDisconnect:
        manager.disconnect(websocket)


def listen_for_telemetry(loop):
    if not HAS_CLOVER:
        return
        
    data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    data_sock.bind((DATA_IP, DATA_PORT))
    data_sock.settimeout(1.0)
    print(f"Listening for UDP telemetry on {DATA_PORT}...")

    req = clover_pb2.Request()
    req.subscribe_data_stream.SetInParent()
    manager.send_request(req, "SUBSCRIBE_DATA_STREAM")
    
    while True:
        try:
            data, _ = data_sock.recvfrom(4096)
            packet = clover_pb2.DataPacket()
            packet.ParseFromString(data)
            sensors = packet.analog_sensors
            gauges = {}
            if hasattr(sensors, 'pt001'): gauges["PT001"] = sensors.pt001
            if hasattr(sensors, 'pt002'): gauges["PT002"] = sensors.pt002
            if hasattr(sensors, 'pt003'): gauges["PT003"] = sensors.pt003
            if hasattr(sensors, 'pt101'): gauges["PT101"] = sensors.pt101
            if hasattr(sensors, 'tc101'): gauges["TC101"] = sensors.tc101
            if hasattr(sensors, 'tc102'): gauges["TC102"] = sensors.tc102
            if hasattr(sensors, 'pt202'): gauges["PT202"] = sensors.pt202
            if hasattr(sensors, 'pt203'): gauges["PT203"] = sensors.pt203
            if hasattr(sensors, 'ptf401'): gauges["PTF401"] = sensors.ptf401
            if hasattr(sensors, 'pto401'): gauges["PTO401"] = sensors.pto401
            if hasattr(sensors, 'ptc401'): gauges["PTC401"] = sensors.ptc401
            if hasattr(sensors, 'ptc402'): gauges["PTC402"] = sensors.ptc402
            
            manager.gauges.update(gauges)
            
            valves_update = {
                "PBV201": "OPEN" if packet.fuel_valve.is_on else "CLOSE",
                "PBV101": "OPEN" if packet.lox_valve.is_on else "CLOSE"
            }
            manager.valves.update(valves_update)
            asyncio.run_coroutine_threadsafe(
                manager.broadcast({"gauges": gauges, "valves": valves_update}), 
                loop
            )
            
        except socket.timeout:
            continue
        except Exception as e:
            pass


@app.on_event("startup")
async def startup_event():
    loop = asyncio.get_running_loop()
    threading.Thread(target=listen_for_telemetry, args=(loop,), daemon=True).start()

if __name__ == "__main__":
    print("Starting Real Telemetry Backend on ws://0.0.0.0:8000/ws...")
    uvicorn.run(app, host=ZEPHYR_IP, port=ZEPHYR_PORT, log_level="warning")
