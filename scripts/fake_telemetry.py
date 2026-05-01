"""
Fake Clover server — simulates the embedded controller for CLI testing.

- TCP port 5000: accepts Request protobufs, sends back Response protobufs
- UDP port 5001: streams DataPackets to subscribed clients

Commands handled:
  subscribe_data_stream            → register sender's IP as a subscriber
  identify_client                  → acknowledged (logged)
  throttle_reset_valve_position    → directly set a valve's target position
  load_throttle_valve_sequence     → store fuel/lox ControlTraces, → THROTTLE_VALVE_PRIMED
  start_throttle_valve_sequence    → execute loaded traces, → THROTTLE_VALVE
  load_throttle_sequence           → store thrust trace, → THROTTLE_PRIMED
  start_throttle_sequence          → execute thrust trace, → THROTTLE
  load_static_fire_sequence        → → STATIC_FIRE_PRIMED
  start_static_fire_sequence       → → STATIC_FIRE
  load_flight_sequence             → → FLIGHT_PRIMED
  start_flight_sequence            → → FLIGHT
  load_tvc_sequence                → → TVC_PRIMED
  start_tvc_sequence               → → TVC
  load_rcs_valve_sequence          → → RCS_VALVE_PRIMED
  start_rcs_valve_sequence         → → RCS_VALVE
  load_rcs_sequence                → → RCS_PRIMED
  start_rcs_sequence               → → RCS
  abort                            → STATE_ABORT, close valves
  halt                             → STATE_ABORT, close valves
  unprime                          → STATE_IDLE

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
TCP_PORT    = 19690
CLIENT_PORT = 19691  # port clients bind for incoming UDP
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
        self.encoder  = self.driver + random.gauss(0, 0.03)


# ── Shared state (all access under `lock`) ────────────────────────────────────
lock = threading.Lock()

system_state = clover_pb2.STATE_IDLE
subscribers  = {"127.0.0.1"}  # pre-seeded so telemetry flows without a subscribe command

fuel = Valve()
lox  = Valve()

fuel_trace  = None         # clover_pb2.ControlTrace | None
lox_trace   = None
seq_start   = None         # monotonic timestamp when sequence began

start_mono  = time.monotonic()
seq_num     = 0

# States where traces are executing
_RUNNING_STATES = {
    clover_pb2.STATE_THROTTLE_VALVE,
    clover_pb2.STATE_THROTTLE,
    clover_pb2.STATE_TVC,
    clover_pb2.STATE_RCS_VALVE,
    clover_pb2.STATE_RCS,
    clover_pb2.STATE_STATIC_FIRE,
    clover_pb2.STATE_FLIGHT,
}


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
    both   = f_open * l_open

    def noise(sigma): return random.gauss(0, sigma)

    return {
        # Feed / prevalve pressures (slowly oscillating tank pressure)
        "pt001":  800.0 + 20.0 * math.sin(elapsed * 0.2) + noise(1.0),
        "pt002":  798.0 + 19.0 * math.sin(elapsed * 0.2 + 0.1) + noise(1.0),
        "pt003":  600.0 + 18.0 * math.sin(elapsed * 0.2 + 0.2) + noise(1.0),
        "pt004":  598.0 + 17.0 * math.sin(elapsed * 0.2 + 0.3) + noise(1.0),
        "pt005":  400.0 + 10.0 * math.sin(elapsed * 0.2 + 0.4) + noise(1.0),
        "pt006":  398.0 +  9.0 * math.sin(elapsed * 0.2 + 0.5) + noise(1.0),
        # Fuel / lox line pressures (rise with valve opening)
        "pt103":  800.0 + 20.0 * math.sin(elapsed * 0.2) + noise(1.0),
        "pt203":  600.0 + 180.0 * f_open + noise(2.0),
        "pt301":  300.0 + 50.0  * both   + noise(2.0),
        # Injector / chamber pressures
        "ptf401": 500.0 + 240.0 * f_open + noise(3.0),
        "pto401": 490.0 + 230.0 * l_open + noise(3.0),
        "ptc401":  50.0 + 850.0 * both   + noise(5.0),
        "ptc402":  48.0 + 845.0 * both   + noise(5.0),
        # Temperatures (slow drift + noise)
        "tc002":   20.0 +  5.0 * math.sin(elapsed * 0.05) + noise(0.2),
        "tc102":   18.0 +  4.0 * math.sin(elapsed * 0.05 + 0.5) + noise(0.2),
        "tc102_5": 19.0 +  4.5 * math.sin(elapsed * 0.05 + 1.0) + noise(0.2),
        "tcf401": 100.0 + 200.0 * f_open + noise(1.0),
        "tco401":  95.0 + 190.0 * l_open + noise(1.0),
        # Pressurant / gas side
        "ptg001": 3000.0 - 5.0 * elapsed + noise(2.0),
        "ptg002": 2998.0 - 5.0 * elapsed + noise(2.0),
        "ptg101": 1500.0 - 2.0 * elapsed + noise(1.5),
        # Battery
        "battery_voltage": 24.0 - 0.001 * elapsed + noise(0.01),
    }


# ── Simulation loop (runs in main thread) ─────────────────────────────────────
def simulation_loop():
    global system_state, seq_num, seq_start

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
            if system_state in _RUNNING_STATES and seq_start is not None:
                t_ms = (now - seq_start) * 1000.0

                fuel_running = fuel_trace is not None and t_ms < fuel_trace.total_time_ms
                lox_running  = lox_trace  is not None and t_ms < lox_trace.total_time_ms

                if fuel_running:
                    val = eval_trace(fuel_trace, t_ms)
                    if val is not None:
                        fuel.target = val

                if lox_running:
                    val = eval_trace(lox_trace, t_ms)
                    if val is not None:
                        lox.target = val

                if not fuel_running and not lox_running:
                    system_state = clover_pb2.STATE_IDLE
                    seq_start    = None
                    fuel.target  = 0.0
                    lox.target   = 0.0
                    print("[sim] Traces complete → STATE_IDLE")

            # ── Step valve physics ────────────────────────────────────────
            fuel.step(dt)
            lox.step(dt)

            # ── Sensors ───────────────────────────────────────────────────
            sv = compute_sensors(elapsed)

            # ── Build packet ──────────────────────────────────────────────
            pkt = clover_pb2.DataPacket()
            pkt.time_ns          = int(time.time() * 1e9)
            pkt.sequence_number  = seq_num
            pkt.data_queue_size  = 0
            pkt.state            = system_state
            pkt.gnc_connected    = True
            pkt.gnc_last_pinged_ns = 0.0
            pkt.daq_connected    = False
            pkt.daq_last_pinged_ns = 0.0

            pkt.controller_timing.controller_tick_time_ns       = 1_000_000
            pkt.controller_timing.analog_sensors_sense_time_ns  = 500_000
            pkt.controller_timing.state_estimator_update_time_ns = 200_000

            pkt.fuel_valve_status.encoder_pos_deg = fuel.encoder
            pkt.fuel_valve_status.is_on           = fuel.enabled
            pkt.lox_valve_status.encoder_pos_deg  = lox.encoder
            pkt.lox_valve_status.is_on            = lox.enabled

            pkt.fuel_valve_command.enable     = fuel.enabled
            pkt.fuel_valve_command.set_pos    = True
            pkt.fuel_valve_command.target_deg = fuel.target
            pkt.lox_valve_command.enable      = lox.enabled
            pkt.lox_valve_command.set_pos     = True
            pkt.lox_valve_command.target_deg  = lox.target

            pkt.valve_states.SetInParent()

            pkt.analog_sensors.pt001          = sv["pt001"]
            pkt.analog_sensors.pt002          = sv["pt002"]
            pkt.analog_sensors.pt003          = sv["pt003"]
            pkt.analog_sensors.pt004          = sv["pt004"]
            pkt.analog_sensors.pt005          = sv["pt005"]
            pkt.analog_sensors.pt006          = sv["pt006"]
            pkt.analog_sensors.pt103          = sv["pt103"]
            pkt.analog_sensors.pt203          = sv["pt203"]
            pkt.analog_sensors.pt301          = sv["pt301"]
            pkt.analog_sensors.ptf401         = sv["ptf401"]
            pkt.analog_sensors.pto401         = sv["pto401"]
            pkt.analog_sensors.ptc401         = sv["ptc401"]
            pkt.analog_sensors.ptc402         = sv["ptc402"]
            pkt.analog_sensors.tc002          = sv["tc002"]
            pkt.analog_sensors.tc102          = sv["tc102"]
            pkt.analog_sensors.tc102_5        = sv["tc102_5"]
            pkt.analog_sensors.tcf401         = sv["tcf401"]
            pkt.analog_sensors.tco401         = sv["tco401"]
            pkt.analog_sensors.ptg001         = sv["ptg001"]
            pkt.analog_sensors.ptg002         = sv["ptg002"]
            pkt.analog_sensors.ptg101         = sv["ptg101"]
            pkt.analog_sensors.battery_voltage = sv["battery_voltage"]

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
def _read_varint(conn: socket.socket) -> int | None:
    value, shift = 0, 0
    while True:
        b = conn.recv(1)
        if not b:
            return None
        byte = b[0]
        value |= (byte & 0x7F) << shift
        if not (byte & 0x80):
            return value
        shift += 7


def _encode_varint(value: int) -> bytes:
    bits = []
    while True:
        bits.append(value & 0x7F)
        value >>= 7
        if not value:
            break
    for i in range(len(bits) - 1):
        bits[i] |= 0x80
    return bytes(bits)


def handle_client(conn: socket.socket, addr):
    global system_state, fuel_trace, lox_trace, seq_start

    try:
        conn.settimeout(None)
        while True:
            length = _read_varint(conn)
            if length is None:
                break

            data = b""
            while len(data) < length:
                chunk = conn.recv(length - len(data))
                if not chunk:
                    return
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

                elif which == "throttle_reset_valve_position":
                    r = req.throttle_reset_valve_position
                    if r.valve == clover_pb2.FUEL:
                        fuel.target = r.new_pos_deg
                        print(f"[tcp] FUEL target → {r.new_pos_deg:.2f}°")
                    else:
                        lox.target = r.new_pos_deg
                        print(f"[tcp] LOX  target → {r.new_pos_deg:.2f}°")

                elif which == "load_throttle_valve_sequence":
                    r = req.load_throttle_valve_sequence
                    if r.HasField("fuel_trace_deg"):
                        fuel_trace = clover_pb2.ControlTrace()
                        fuel_trace.CopyFrom(r.fuel_trace_deg)
                    if r.HasField("lox_trace_deg"):
                        lox_trace = clover_pb2.ControlTrace()
                        lox_trace.CopyFrom(r.lox_trace_deg)
                    system_state = clover_pb2.STATE_THROTTLE_VALVE_PRIMED
                    print("[tcp] LOAD_THROTTLE_VALVE_SEQUENCE → STATE_THROTTLE_VALVE_PRIMED")

                elif which == "start_throttle_valve_sequence":
                    if fuel_trace is not None or lox_trace is not None:
                        system_state = clover_pb2.STATE_THROTTLE_VALVE
                        seq_start    = time.monotonic()
                        print("[tcp] START_THROTTLE_VALVE_SEQUENCE → STATE_THROTTLE_VALVE")
                    else:
                        resp.err = "No traces loaded"

                elif which == "load_throttle_sequence":
                    r = req.load_throttle_sequence
                    fuel_trace = clover_pb2.ControlTrace()
                    fuel_trace.CopyFrom(r.thrust_lbf)
                    lox_trace = clover_pb2.ControlTrace()
                    lox_trace.CopyFrom(r.thrust_lbf)
                    system_state = clover_pb2.STATE_THROTTLE_PRIMED
                    print("[tcp] LOAD_THROTTLE_SEQUENCE → STATE_THROTTLE_PRIMED")

                elif which == "start_throttle_sequence":
                    system_state = clover_pb2.STATE_THROTTLE
                    seq_start    = time.monotonic()
                    print("[tcp] START_THROTTLE_SEQUENCE → STATE_THROTTLE")

                elif which == "load_static_fire_sequence":
                    system_state = clover_pb2.STATE_STATIC_FIRE_PRIMED
                    print("[tcp] LOAD_STATIC_FIRE_SEQUENCE → STATE_STATIC_FIRE_PRIMED")

                elif which == "start_static_fire_sequence":
                    system_state = clover_pb2.STATE_STATIC_FIRE
                    seq_start    = time.monotonic()
                    print("[tcp] START_STATIC_FIRE_SEQUENCE → STATE_STATIC_FIRE")

                elif which == "load_flight_sequence":
                    system_state = clover_pb2.STATE_FLIGHT_PRIMED
                    print("[tcp] LOAD_FLIGHT_SEQUENCE → STATE_FLIGHT_PRIMED")

                elif which == "start_flight_sequence":
                    system_state = clover_pb2.STATE_FLIGHT
                    seq_start    = time.monotonic()
                    print("[tcp] START_FLIGHT_SEQUENCE → STATE_FLIGHT")

                elif which == "load_tvc_sequence":
                    system_state = clover_pb2.STATE_TVC_PRIMED
                    print("[tcp] LOAD_TVC_SEQUENCE → STATE_TVC_PRIMED")

                elif which == "start_tvc_sequence":
                    system_state = clover_pb2.STATE_TVC
                    seq_start    = time.monotonic()
                    print("[tcp] START_TVC_SEQUENCE → STATE_TVC")

                elif which == "load_rcs_valve_sequence":
                    system_state = clover_pb2.STATE_RCS_VALVE_PRIMED
                    print("[tcp] LOAD_RCS_VALVE_SEQUENCE → STATE_RCS_VALVE_PRIMED")

                elif which == "start_rcs_valve_sequence":
                    system_state = clover_pb2.STATE_RCS_VALVE
                    seq_start    = time.monotonic()
                    print("[tcp] START_RCS_VALVE_SEQUENCE → STATE_RCS_VALVE")

                elif which == "load_rcs_sequence":
                    system_state = clover_pb2.STATE_RCS_PRIMED
                    print("[tcp] LOAD_RCS_SEQUENCE → STATE_RCS_PRIMED")

                elif which == "start_rcs_sequence":
                    system_state = clover_pb2.STATE_RCS
                    seq_start    = time.monotonic()
                    print("[tcp] START_RCS_SEQUENCE → STATE_RCS")

                elif which in ("abort", "halt"):
                    system_state = clover_pb2.STATE_ABORT
                    fuel.target  = 0.0
                    lox.target   = 0.0
                    seq_start    = None
                    print(f"[tcp] {which.upper()} → STATE_ABORT")

                elif which == "unprime":
                    system_state = clover_pb2.STATE_IDLE
                    fuel_trace   = None
                    lox_trace    = None
                    seq_start    = None
                    print("[tcp] UNPRIME → STATE_IDLE")

                else:
                    if which:
                        print(f"[tcp] Unhandled command: {which}")

            raw = resp.SerializeToString()
            conn.sendall(_encode_varint(len(raw)) + raw)

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
