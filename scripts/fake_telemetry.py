"""
Fake Clover server — simulates the embedded controller for CLI testing.

- TCP port 5000: accepts Request protobufs, sends back Response protobufs
- UDP port 5001: streams DataPackets to subscribed clients

Commands handled:
  subscribe_data_stream       → register sender's IP as a subscriber
  identify_client             → acknowledged (logged)
  reset_valve_position        → directly set a valve's target position
  load_motor_sequence         → store fuel/lox ControlTraces
  start_sequence              → enter STATE_SEQUENCE and execute loaded traces
  halt_sequence               → enter STATE_ABORT, close valves
  start_throttle_closed_loop  → enter STATE_CLOSED_LOOP_THROTTLE, optionally run thrust trace
  set_controller_state        → force state directly

Usage:
    python fake_telemetry.py
"""

import math
import random
import socket
import threading
import time

import clover_pb2

# ── Config ────────────────────────────────────────────────────────────────────
TCP_PORT    = 5000
CLIENT_PORT = 5001   # port clients bind for incoming UDP
SEND_HZ     = 50     # telemetry packets per second

# ── Valve model ───────────────────────────────────────────────────────────────
class Valve:
    SLEW_RATE = 200.0  # max °/s the driver can move

    def __init__(self):
        self.target  = 0.0
        self.driver  = 0.0
        self.encoder = 0.0
        self.enabled = True

    def step(self, dt: float):
        delta    = self.target - self.driver
        max_step = self.SLEW_RATE * dt
        self.driver  += max(-max_step, min(max_step, delta))
        self.encoder  = self.driver + random.gauss(0, 0.03)   # small encoder noise


# ── Shared state (all access under `lock`) ────────────────────────────────────
lock = threading.Lock()

system_state = clover_pb2.STATE_IDLE
is_abort     = False
subscribers  = {"127.0.0.1"}  # pre-seeded so telemetry flows without a subscribe command

fuel = Valve()
lox  = Valve()

fuel_trace  = None         # clover_pb2.ControlTrace | None
lox_trace   = None
seq_start   = None         # monotonic timestamp when sequence began

start_mono  = time.monotonic()
seq_num     = 0


# ── Trace evaluation ──────────────────────────────────────────────────────────
def eval_trace(trace: clover_pb2.ControlTrace, t_ms: float) -> float | None:
    """
    Return the trace value at elapsed time t_ms, or None if the trace is done.
    Gaps between segments return None (no command → valve holds last position).
    """
    if t_ms >= trace.total_time_ms:
        return None
    for seg in trace.segments:
        if seg.start_ms <= t_ms < seg.start_ms + seg.length_ms:
            t_local = t_ms - seg.start_ms
            which   = seg.WhichOneof("type")
            if which == "linear":
                frac = t_local / seg.length_ms if seg.length_ms else 1.0
                return seg.linear.start_val + frac * (seg.linear.end_val - seg.linear.start_val)
            elif which == "sine":
                s = seg.sine
                return s.offset + s.amplitude * math.sin(
                    2 * math.pi * t_local / s.period + math.radians(s.phase_deg)
                )
    return None   # gap — no segment covers this instant


# ── Sensor model (pressure responds to valve openings) ────────────────────────
def compute_sensors(elapsed: float) -> dict:
    """
    Simulate pressure transducers.
    Valves fully open = 90°; normalised openness drives downstream pressure.
    """
    f_open = max(0.0, min(1.0, fuel.encoder / 90.0))
    l_open = max(0.0, min(1.0, lox.encoder  / 90.0))
    both   = f_open * l_open          # chamber only sees pressure when both open

    def noise(sigma): return random.gauss(0, sigma)

    return {
        "pt102":  800.0 + 20.0 * math.sin(elapsed * 0.2) + noise(1.0),   # fuel tank
        "pt103":  798.0 + 19.0 * math.sin(elapsed * 0.2 + 0.1) + noise(1.0),
        "pt202":  600.0 + 180.0 * f_open + noise(2.0),                    # fuel line
        "pt203":  598.0 + 178.0 * f_open + noise(2.0),
        "ptf401": 500.0 + 240.0 * f_open + noise(3.0),                    # fuel injector
        "pto401": 490.0 + 230.0 * l_open + noise(3.0),                    # lox injector
        "ptc401": 50.0  + 850.0 * both   + noise(5.0),                    # chamber
        "ptc402": 48.0  + 845.0 * both   + noise(5.0),
    }


# ── Simulation loop (runs in main thread) ─────────────────────────────────────
def simulation_loop():
    global system_state, is_abort, seq_num, seq_start

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / SEND_HZ
    last = time.monotonic()

    while True:
        now     = time.monotonic()
        dt      = now - last
        last    = now
        elapsed = now - start_mono

        with lock:
            # ── Execute loaded traces ─────────────────────────────────────
            if system_state in (clover_pb2.STATE_SEQUENCE,
                                clover_pb2.STATE_CLOSED_LOOP_THROTTLE) and seq_start is not None:
                t_ms = (now - seq_start) * 1000.0

                # "Running" = still within the trace's total duration.
                # Gaps between segments hold the valve at its last position —
                # they do NOT end the sequence early.
                fuel_running = fuel_trace is not None and t_ms < fuel_trace.total_time_ms
                lox_running  = lox_trace  is not None and t_ms < lox_trace.total_time_ms

                if fuel_running:
                    val = eval_trace(fuel_trace, t_ms)
                    if val is not None:
                        fuel.target = val   # hold position during gaps

                if lox_running:
                    val = eval_trace(lox_trace, t_ms)
                    if val is not None:
                        lox.target = val

                if not (fuel_running or lox_running):
                    system_state = clover_pb2.STATE_IDLE
                    seq_start    = None
                    print("[sim] Traces complete → STATE_IDLE")

            # ── Step valve physics ────────────────────────────────────────
            fuel.step(dt)
            lox.step(dt)

            # ── Sensors ───────────────────────────────────────────────────
            sv = compute_sensors(elapsed)

            # ── Build packet ──────────────────────────────────────────────
            pkt = clover_pb2.DataPacket()
            pkt.time             = elapsed
            pkt.sequence_number  = seq_num
            pkt.data_queue_size  = 0
            pkt.state            = system_state
            pkt.is_abort         = is_abort

            pkt.fuel_valve.enabled                 = fuel.enabled
            pkt.fuel_valve.target_pos_deg          = fuel.target
            pkt.fuel_valve.driver_setpoint_pos_deg = fuel.driver
            pkt.fuel_valve.encoder_pos_deg         = fuel.encoder

            pkt.lox_valve.enabled                  = lox.enabled
            pkt.lox_valve.target_pos_deg           = lox.target
            pkt.lox_valve.driver_setpoint_pos_deg  = lox.driver
            pkt.lox_valve.encoder_pos_deg          = lox.encoder

            pkt.sensors.pt102  = sv["pt102"]
            pkt.sensors.pt103  = sv["pt103"]
            pkt.sensors.pt202  = sv["pt202"]
            pkt.sensors.pt203  = sv["pt203"]
            pkt.sensors.ptf401 = sv["ptf401"]
            pkt.sensors.pto401 = sv["pto401"]
            pkt.sensors.ptc401 = sv["ptc401"]
            pkt.sensors.ptc402 = sv["ptc402"]

            seq_num += 1
            data = pkt.SerializeToString()
            subs = list(subscribers)

        for ip in subs:
            try:
                udp_sock.sendto(data, (ip, CLIENT_PORT))
            except Exception:
                pass

        time.sleep(max(0.0, interval - (time.monotonic() - now)))


# ── TCP command handler ───────────────────────────────────────────────────────
def handle_client(conn: socket.socket, addr):
    global system_state, is_abort, fuel_trace, lox_trace, seq_start

    try:
        data = b""
        conn.settimeout(2.0)
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            data += chunk

        req   = clover_pb2.Request()
        req.ParseFromString(data)
        resp  = clover_pb2.Response()
        which = req.WhichOneof("payload")

        with lock:
            if which == "subscribe_data_stream":
                subscribers.add(addr[0])
                print(f"[tcp] {addr[0]} subscribed to data stream")

            elif which == "identify_client":
                name = clover_pb2.ClientType.Name(req.identify_client.client)
                print(f"[tcp] Client identified as {name}")

            elif which == "reset_valve_position":
                r = req.reset_valve_position
                if r.valve == clover_pb2.FUEL:
                    fuel.target = r.new_pos_deg
                    print(f"[tcp] FUEL target → {r.new_pos_deg:.2f}°")
                else:
                    lox.target = r.new_pos_deg
                    print(f"[tcp] LOX  target → {r.new_pos_deg:.2f}°")

            elif which == "load_motor_sequence":
                r = req.load_motor_sequence
                if r.HasField("fuel_trace"):
                    fuel_trace = clover_pb2.ControlTrace()
                    fuel_trace.CopyFrom(r.fuel_trace)
                if r.HasField("lox_trace"):
                    lox_trace = clover_pb2.ControlTrace()
                    lox_trace.CopyFrom(r.lox_trace)
                print(f"[tcp] Traces loaded — fuel: {r.HasField('fuel_trace')} "
                      f"({fuel_trace.total_time_ms}ms, {len(fuel_trace.segments)} segs), "
                      f"lox: {r.HasField('lox_trace')} "
                      f"({lox_trace.total_time_ms if lox_trace else 0}ms)")

            elif which == "start_sequence":
                if fuel_trace is not None or lox_trace is not None:
                    system_state = clover_pb2.STATE_SEQUENCE
                    seq_start    = time.monotonic()
                    is_abort     = False
                    print("[tcp] START_SEQUENCE")
                else:
                    resp.err = "No traces loaded"
                    print("[tcp] START_SEQUENCE rejected — no traces")

            elif which == "halt_sequence":
                system_state = clover_pb2.STATE_ABORT
                is_abort     = True
                fuel.target  = 0.0
                lox.target   = 0.0
                seq_start    = None
                print("[tcp] HALT — valves closing, STATE_ABORT")

            elif which == "start_throttle_closed_loop":
                system_state = clover_pb2.STATE_CLOSED_LOOP_THROTTLE
                is_abort     = False
                r = req.start_throttle_closed_loop
                if r.HasField("thrust_trace"):
                    # Drive both valves from the thrust trace (treated as target °)
                    fuel_trace = clover_pb2.ControlTrace()
                    fuel_trace.CopyFrom(r.thrust_trace)
                    lox_trace  = clover_pb2.ControlTrace()
                    lox_trace.CopyFrom(r.thrust_trace)
                    seq_start  = time.monotonic()
                    print("[tcp] START_THROTTLE_CLOSED_LOOP with thrust trace")
                else:
                    seq_start = None
                    print("[tcp] START_THROTTLE_CLOSED_LOOP (manual)")

            elif which == "set_controller_state":
                system_state = req.set_controller_state.state
                is_abort     = (system_state == clover_pb2.STATE_ABORT)
                if is_abort:
                    fuel.target = 0.0
                    lox.target  = 0.0
                seq_start = None
                print(f"[tcp] State → {clover_pb2.SystemState.Name(system_state)}")

        conn.sendall(resp.SerializeToString())

    except Exception as e:
        print(f"[tcp] Error from {addr}: {e}")
    finally:
        conn.close()


def tcp_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", TCP_PORT))
    srv.listen(10)
    print(f"[tcp] Listening on port {TCP_PORT}")
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()


def main():
    print(f"Fake Clover server — {SEND_HZ} Hz telemetry, TCP:{TCP_PORT}, UDP→{CLIENT_PORT}")
    print("Ctrl+C to stop.\n")
    threading.Thread(target=tcp_server, daemon=True).start()
    try:
        simulation_loop()
    except KeyboardInterrupt:
        print("\n[sim] Stopped.")


if __name__ == "__main__":
    main()
