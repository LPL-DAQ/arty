import asyncio
import json
import random
import os
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn

def parse_sequences(file_path):
    sequences = {}
    current_seq = None
    if not os.path.exists(file_path):
        return sequences
    
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith('[') and line.endswith(']'):
                current_seq = line[1:-1]
                sequences[current_seq] = []
            elif current_seq is not None:
                parts = line.split()
                if len(parts) >= 3:
                    comp = parts[0]
                    time_val = float(parts[-1])
                    action = " ".join(parts[1:-1])
                    sequences[current_seq].append({
                        "time": time_val,
                        "component": comp,
                        "action": action
                    })
    return sequences

SEQUENCES = parse_sequences(os.path.join(os.path.dirname(__file__), "sequences.txt"))

app = FastAPI()

class ValveManager:
    def __init__(self):
        self.active_connections = []
        
        # Initial State identical to what React expects
        self.valves = {
            "PBV001": "CLOSE", "PBV002": "CLOSE", "PBV003": "CLOSE", 
            "PBV004": "CLOSE", "PBV005": "CLOSE", "PBV006": "CLOSE",
            "PBV101": "CLOSE", "PBV201": "CLOSE", "PBV301": "CLOSE", "PBV302": "CLOSE",
            "IGN002": "CLOSE", "ALM001": "CLOSE"
        }
        self.armed = {k: False for k in self.valves.keys()}
        self.sequence_task = None

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        # 1. Immediately send current state to any new client connecting
        await websocket.send_json({
            "valves": self.valves,
            "armed": self.armed
        })
        print("Client Connected!")

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        print("Client Disconnected!")

    async def broadcast(self, message: dict):
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception:
                pass

    async def run_sequence(self, sequence_name, sequences):
        seq = sequences.get(sequence_name)
        if not seq:
            print(f"Sequence '{sequence_name}' not found.")
            return

        sorted_events = sorted(seq, key=lambda x: x["time"])
        if not sorted_events:
            return

        print(f"Starting Sequence '{sequence_name}' with {len(sorted_events)} steps.")
        current_time = sorted_events[0]["time"]
        
        try:
            for event in sorted_events:
                delay = event["time"] - current_time
                if delay > 0:
                    await asyncio.sleep(delay)
                    current_time = event["time"]
                
                comp = event["component"].replace("-", "")
                action = event["action"]
                
                print(f"[T={current_time:+.2f}s] {comp} -> {action}")
                
                if comp in self.valves:
                    self.valves[comp] = action
                    await self.broadcast({"valves": {comp: action}})
        except asyncio.CancelledError:
            print(f"Sequence '{sequence_name}' was CANCELLED/ABORTED.")
            raise


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
                manager.valves[name] = "OPEN" if manager.valves[name] == "CLOSE" else "CLOSE"
                await manager.broadcast({"valves": {name: manager.valves[name]}})
                print(f"Valve {name} is now {manager.valves[name]}")
                
            elif action == "ARM_VALVE" and name in manager.armed:
                # Exclusive arming: set all others to False
                manager.armed = {k: (k == name and not manager.armed[k]) for k in manager.armed}
                await manager.broadcast({"armed": manager.armed})
            
            elif action == "ACTUATE":
                # Find the currently armed valve
                armed_valve = next((k for k, v in manager.armed.items() if v), None)
                if armed_valve:
                    # Toggle state
                    manager.valves[armed_valve] = "OPEN" if manager.valves[armed_valve] == "CLOSE" else "CLOSE"
                    # Unarm it
                    manager.armed[armed_valve] = False
                    # Broadcast both changes
                    await manager.broadcast({
                        "valves": {armed_valve: manager.valves[armed_valve]},
                        "armed": manager.armed
                    })
                    print(f"ACTUATE: Valve {armed_valve} is now {manager.valves[armed_valve]}")
                
            elif action == "INITIATE_SEQUENCE":
                seq_name = payload.get("name")
                if manager.sequence_task:
                    manager.sequence_task.cancel()
                
                fresh_sequences = parse_sequences(os.path.join(os.path.dirname(__file__), "sequences.txt"))
                manager.sequence_task = asyncio.create_task(manager.run_sequence(seq_name, fresh_sequences))
                
            elif action == "ABORT":
                if manager.sequence_task:
                    manager.sequence_task.cancel()
                    manager.sequence_task = None
                    
                manager.valves = {k: "CLOSE" for k in manager.valves}
                manager.armed = {k: False for k in manager.armed}
                await manager.broadcast({"valves": manager.valves, "armed": manager.armed})
                print("ABORT Triggered! All valves closed and unarmed. Sequence stopped.")

    except WebSocketDisconnect:
        manager.disconnect(websocket)

LOG_HEADERS = [
    "TIME", "PT001", "PT002", "PT003", "PT004", "PT005", "PT006", 
    "PT101", "PT102", "PT201", "PT202", "PT301", "PT401", "PTF401", 
    "PTF402", "PTF403", "PTF404", "PTF405", "PTC401", "PTC402", 
    "PTC403", "PTC404", "PTO401", "PTO402", "PTO501", "PTO502", 
    "PTO503", "PTF501", "PTF502", "PTF503", "PTP501", "PTP502", 
    "PTP503", "PTT501", "TC001", "TC002", "TC003", "TC101", "TC102", 
    "TC201", "TCF401", "TCF402", "TCF403", "TCS401", "TCS402", 
    "TCS403", "TCS404", "TCS405", "TCO401", "TCO501", "TCO503", 
    "TCF501", "TCF503", "TCP503", "TCS501", "FM201", "FM101", 
    "FMF501", "FMO501", "LC001", "LC002", "LC003"
]

async def simulate_telemetry():
    import time
    from datetime import datetime
    
    start_time = time.time()
    
    # Setup log file
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"telemetry_log_{timestamp}.txt"
    
    with open(log_filename, "w") as f:
        f.write(",".join(LOG_HEADERS) + "\n")
        
    print(f"Logging telemetry to {log_filename}")

    while True:
        await asyncio.sleep(0.1)  
        
        # Calculate elapsed time
        elapsed_time = time.time() - start_time
        
        # Calculate base simulated data
        gauges = {
            # Pressurant
            "PT001": 10.0,
            "PT002": 10.0,
            "PT003": round(510 + (random.random() - 0.5) * 5, 1),
            "PT301": 1.0, # As seen in sample
            # LOX
            "PT101": round(700 + (random.random() - 0.5) * 5, 1),
            "TC101": round(-180 + (random.random() - 0.5) * 5, 1),
            # Fuel
            "PT201": round(650 + (random.random() - 0.5) * 5, 1),
            "TC201": round(20 + (random.random() - 0.5) * 5, 1),
            # Load Cells (matching sample approx values)
            "LC001": round(8273.2 + (random.random() - 0.5) * 2, 1),
            "LC002": round(8285.9 + (random.random() - 0.5) * 2, 1),
            "LC003": round(4893.6 + (random.random() - 0.5) * 2, 1),
            # Flow Meters sample
            "FM201": -14.1000,
            "FM101": -13.8562,
            "FMF501": -13.8562,
            "FMO501": -13.8562,
        }
        
        # Construct log row mapped exactly to LOG_HEADERS
        log_row = []
        for header in LOG_HEADERS:
            if header == "TIME":
                log_row.append(f"{elapsed_time:.6f}")
            else:
                val = gauges.get(header, 0.0) # Default to 0.0 if not mocked
                log_row.append(f"{val:.4f}")
        
        # Write to log file
        with open(log_filename, "a") as f:
            f.write(",".join(log_row) + "\n")
            
        if manager.active_connections:
            await manager.broadcast({"gauges": gauges})

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(simulate_telemetry())

if __name__ == "__main__":
    print("Starting Mock Valve Telemetry WebSocket Server on ws://localhost:8000/ws...")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
