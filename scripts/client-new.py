# improved CLI

import argparse
import socket
import threading
import sys
import time
from rich.console import Console
from rich.prompt import Prompt, Confirm, IntPrompt, FloatPrompt
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.layout import Layout
from rich.live import Live
from rich.rule import Rule
from rich.columns import Columns
from rich import box
import clover_pb2
from rich.console import Group
from prompt_toolkit import PromptSession
from prompt_toolkit.formatted_text import HTML
import clickhouse_connect
import polars as pl


THEME = {
    "primary":      "bold gold1",
    "success":      "bold green",
    "warning":      "bold yellow",
    "danger":       "bold red",
    "info":         "bold gold1",
    "muted":        "dim white",
    "header":       "bold white on dark_red",
    "panel_border": "dark_red",
    "icon_fire":    "🔥",
    "icon_ok":      "✅",
    "icon_warn":    "⚠️ ",
    "icon_stop":    "🛑",
    "icon_live":    "📡",
    "icon_fuel":    "🟡",
    "icon_lox":     "🔴",
    "icon_valve":   "🔧",
    "icon_seq":     "▶️ ",
    "icon_loop":    "🔄",
    "icon_id":      "🪪",
    "icon_quit":    "❌",
}

# Network
# ZEPHYR_IP = "169.254.99.99"  # real board
ZEPHYR_IP   = "127.0.0.1"      # fake_telemetry.py
ZEPHYR_PORT = 5000
LOCAL_PORT  = 5001

# ── ClickHouse config ────────────────────────────────────────────────────────
CH_HOST     = "172.233.143.186"
CH_USER     = "writer"
CH_PASSWORD = "ce8XpzhRGhsvBxCPHDTcvh6DMWhb3jyxgmQMNLrsKaCqtZvKf2"
CH_DATABASE = "lpl"


console = Console()
latest_packet: clover_pb2.DataPacket | None = None
packet_lock = threading.Lock()

data_sock = None

_packet_buffer: list = []
_buffer_lock   = threading.Lock()

# Telemetry
def listen_for_telemetry():
    """Background thread — receives UDP DataPackets and stores latest."""
    global latest_packet
    while True:
        try:
            data, _ = data_sock.recvfrom(4096)
            packet = clover_pb2.DataPacket()
            packet.ParseFromString(data)
            recv_time = time.time()
            with packet_lock:
                latest_packet = packet
            with _buffer_lock:
                _packet_buffer.append((recv_time, packet))
        except Exception:
            pass


_STATE_NAMES = {float(i): clover_pb2.SystemState.Name(i) for i in range(5)}


def _packet_to_row(recv_time: float, pkt: clover_pb2.DataPacket) -> dict:
    """Flatten one DataPacket into a wide dict of numeric sensors."""
    s = pkt.sensors
    f = pkt.fuel_valve
    l = pkt.lox_valve
    return {
        'time':             int(recv_time * 1_000_000),
        'gnc_state':        float(pkt.state),
        'gnc_pt102':        s.pt102  if s.HasField('pt102')  else None,
        'gnc_pt103':        s.pt103  if s.HasField('pt103')  else None,
        'gnc_pt202':        s.pt202  if s.HasField('pt202')  else None,
        'gnc_pt203':        s.pt203  if s.HasField('pt203')  else None,
        'gnc_ptf401':       s.ptf401 if s.HasField('ptf401') else None,
        'gnc_pto401':       s.pto401 if s.HasField('pto401') else None,
        'gnc_ptc401':       s.ptc401 if s.HasField('ptc401') else None,
        'gnc_ptc402':       s.ptc402 if s.HasField('ptc402') else None,
        'gnc_fuel_target':  float(f.target_pos_deg),
        'gnc_fuel_driver':  float(f.driver_setpoint_pos_deg),
        'gnc_fuel_encoder': float(f.encoder_pos_deg),
        'gnc_lox_target':   float(l.target_pos_deg),
        'gnc_lox_driver':   float(l.driver_setpoint_pos_deg),
        'gnc_lox_encoder':  float(l.encoder_pos_deg),
    }


def _flush_loop():
    """Background thread — drains the packet buffer into ClickHouse every second."""
    ch = clickhouse_connect.get_client(
        host=CH_HOST,
        username=CH_USER,
        password=CH_PASSWORD,
        database=CH_DATABASE,
    )
    while True:
        time.sleep(1.0)
        with _buffer_lock:
            if not _packet_buffer:
                continue
            batch = list(_packet_buffer)
            _packet_buffer.clear()

        try:
            rows = [_packet_to_row(t, p) for t, p in batch]
            df = (
                pl.DataFrame(rows)
                .unpivot(index=['time'], variable_name='sensor', value_name='value')
                .drop_nulls('value')
                .with_columns(
                    pl.from_epoch('time', time_unit='us').alias('time'),
                    pl.col('value').cast(pl.Float64),
                    pl.when(pl.col('sensor') == 'gnc_state')
                      .then(pl.col('value').map_elements(
                          lambda v: _STATE_NAMES.get(v, ''), return_dtype=pl.String))
                      .otherwise(pl.lit(''))
                      .alias('event'),
                    pl.lit('atlas').alias('system'),
                    pl.lit('gnc').alias('source'),
                )
            )
            ch.insert_df_arrow('raw_sensors', df)
        except Exception as e:
            console.print(f"  [bold red]ClickHouse insert error:[/bold red] {e}")


STATE_COLORS = {
    "STATE_IDLE":                 "green",
    "STATE_SEQUENCE":             "gold1",
    "STATE_CLOSED_LOOP_THROTTLE": "yellow",
    "STATE_ABORT":                "bold red",
    "STATE_CALIBRATION":          "magenta",
}


def _build_status_renderable():
    """Build a rich renderable for the current telemetry snapshot."""
    with packet_lock:
        pkt = latest_packet

    t = THEME

    if pkt is None:
        return Panel(
            f"[{t['muted']}]Waiting for telemetry...[/{t['muted']}]",
            title=f"[{t['primary']}]{t['icon_live']} LIVE STATUS[/{t['primary']}]",
            border_style=t["panel_border"],
        )

    state_name  = clover_pb2.SystemState.Name(pkt.state)
    state_color = STATE_COLORS.get(state_name, "white")
    abort_str   = f"[bold red]{t['icon_stop']} ABORT[/bold red]" if pkt.is_abort else "[green]nominal[/green]"

    # ── Header ────────────────────────────────────────────────
    hdr = Table.grid(padding=(0, 3))
    for _ in range(5):
        hdr.add_column()
    hdr.add_row(
        Text(f"{t['icon_live']} LIVE",      style="bold green"),
        Text(f"State: {state_name}",         style=state_color),
        Text(f"t = {pkt.time:.3f} s",        style="white"),
        Text(f"seq #{pkt.sequence_number}",  style=t["muted"]),
        Text(f"queue: {pkt.data_queue_size}", style=t["muted"]),
    )
    header_panel = Panel(hdr, border_style=t["panel_border"], padding=(0, 1),
                         subtitle=abort_str)

    # ── Valve table ───────────────────────────────────────────
    vt = Table(box=box.SIMPLE_HEAD, show_header=True,
               header_style=t["primary"], border_style=t["panel_border"], padding=(0, 1))
    vt.add_column("Valve",     style="bold white", no_wrap=True)
    vt.add_column("Enabled",   no_wrap=True)
    vt.add_column("Target °",  style="white", no_wrap=True)
    vt.add_column("Driver °",  style="white", no_wrap=True)
    vt.add_column("Encoder °", style="white", no_wrap=True)

    for label, icon, color, v in [
        ("FUEL", t["icon_fuel"], "gold1",    pkt.fuel_valve),
        ("LOX",  t["icon_lox"],  "dark_red", pkt.lox_valve),
    ]:
        vt.add_row(
            f"{icon} [{color}]{label}[/{color}]",
            "[green]YES[/green]" if v.enabled else "[red]NO[/red]",
            f"{v.target_pos_deg:.2f}",
            f"{v.driver_setpoint_pos_deg:.2f}",
            f"{v.encoder_pos_deg:.2f}",
        )

    # ── Sensor table ──────────────────────────────────────────
    st = Table(box=box.SIMPLE_HEAD, show_header=True,
               header_style=t["primary"], border_style=t["panel_border"], padding=(0, 1))
    st.add_column("Sensor", style="bold white", no_wrap=True)
    st.add_column("Value",  style="white",      no_wrap=True)

    s = pkt.sensors
    for name, val in [
        ("PT-102",  s.pt102),  ("PT-103",  s.pt103),
        ("PT-202",  s.pt202),  ("PT-203",  s.pt203),
        ("PT-F401", s.ptf401), ("PT-O401", s.pto401),
        ("PT-C401", s.ptc401), ("PT-C402", s.ptc402),
    ]:
        val_str = f"{val:.2f}" if val != 0.0 else f"[{t['muted']}]—[/{t['muted']}]"
        st.add_row(name, val_str)

    bottom = Columns([
        Panel(vt, title=f"[{t['primary']}]Valves[/{t['primary']}]",   border_style=t["panel_border"]),
        Panel(st, title=f"[{t['primary']}]Sensors[/{t['primary']}]",  border_style=t["panel_border"]),
    ])

    return Group(header_panel, bottom)


def cmd_live_status():
    """Stream full telemetry panel at ~10 Hz. Press Enter to return to menu."""
    stop = threading.Event()

    def _wait_for_enter():
        try:
            sys.stdin.readline()
        except Exception:
            pass
        stop.set()

    console.print(f"  [{THEME['muted']}]Live view — press Enter to return to menu[/{THEME['muted']}]")
    threading.Thread(target=_wait_for_enter, daemon=True).start()

    try:
        with Live(_build_status_renderable(), refresh_per_second=10, screen=False) as live:
            while not stop.is_set():
                time.sleep(0.1)
                live.update(_build_status_renderable())
    except KeyboardInterrupt:
        pass


_TOOLBAR_STATE_TAGS = {
    "STATE_IDLE":                 ("ansigreen",  "STATE_IDLE"),
    "STATE_SEQUENCE":             ("ansiyellow", "STATE_SEQUENCE"),
    "STATE_CLOSED_LOOP_THROTTLE": ("ansiyellow", "STATE_CLT"),
    "STATE_ABORT":                ("ansired",    "STATE_ABORT"),
    "STATE_CALIBRATION":          ("ansired",    "STATE_CAL"),
}


def get_toolbar():
    """Compact live telemetry for the prompt_toolkit bottom toolbar."""
    with packet_lock:
        pkt = latest_packet

    if pkt is None:
        return HTML(" <b>📡</b> No telemetry yet...")

    state_name = clover_pb2.SystemState.Name(pkt.state)
    tag, short = _TOOLBAR_STATE_TAGS.get(state_name, ("ansiwhite", state_name))

    abort_html = "  <ansired><b>🛑 ABORT</b></ansired>" if pkt.is_abort else ""

    s = pkt.sensors
    sensor_parts = [
        f"{lbl}: {val:.1f}"
        for lbl, val in [
            ("PT-F401", s.ptf401), ("PT-O401", s.pto401),
            ("PT-C401", s.ptc401), ("PT-C402", s.ptc402),
        ]
        if val != 0.0
    ]
    sensor_html = ("  │  " + "  ".join(sensor_parts)) if sensor_parts else ""

    f = pkt.fuel_valve
    l = pkt.lox_valve
    return HTML(
        f" 📡 <{tag}><b>{short}</b></{tag}>"
        f"  │  t={pkt.time:.2f}s  seq#{pkt.sequence_number}"
        f"  │  🟡 drv={f.driver_setpoint_pos_deg:.1f}°  enc={f.encoder_pos_deg:.1f}°"
        f"  │  🔴 drv={l.driver_setpoint_pos_deg:.1f}°  enc={l.encoder_pos_deg:.1f}°"
        + sensor_html
        + abort_html
    )


# TCP

def send_request(req: clover_pb2.Request, label: str) -> bool:
    """Serialize and send a Request over TCP. Returns True on success."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect((ZEPHYR_IP, ZEPHYR_PORT))
        sock.sendall(req.SerializeToString())
        sock.close()
        console.print(
            f"\n  {THEME['icon_ok']} [{THEME['success']}]Sent {label} → {ZEPHYR_IP}:{ZEPHYR_PORT}[/{THEME['success']}]\n"
        )
        return True
    except Exception as e:
        console.print(
            f"\n  {THEME['icon_warn']} [{THEME['danger']}]Failed to send {label}: {e}[/{THEME['danger']}]\n"
        )
        return False

# comment out to test in terminal without real data
# def send_request(req, label):
#     console.print(f"\n  {THEME['icon_ok']} [bold green]Would send the following: {label}[/bold green]\n")
#     return True


# Commands

def cmd_subscribe_data_stream():
    """Request #1 — Subscribe to telemetry data stream."""
    req = clover_pb2.Request()
    req.subscribe_data_stream.SetInParent()
    send_request(req, "SUBSCRIBE_DATA_STREAM")


def cmd_identify_client():
    """Request #6 — Identify client as GNC"""
    t = THEME
    console.print(f"\n  {t['icon_id']} [{t['primary']}]Client identity: GNC[/{t['primary']}]")
    req = clover_pb2.Request()
    req.identify_client.client = clover_pb2.GNC
    send_request(req, "IDENTIFY_CLIENT (GNC)")


def cmd_reset_valve_position():
    """Request #2 — Reset a valve to a specified degree position."""
    t = THEME
    console.print(f"\n  {t['icon_valve']} [{t['primary']}]Reset Valve Position[/{t['primary']}]")
    console.print("    [1] FUEL")
    console.print("    [2] LOX")

    choice = Prompt.ask("  Select valve", choices=["1", "2"])
    valve = clover_pb2.FUEL if choice == "1" else clover_pb2.LOX
    valve_name = "FUEL" if choice == "1" else "LOX"

    pos = FloatPrompt.ask(f"  New position for {valve_name} valve (degrees)")

    req = clover_pb2.Request()
    req.reset_valve_position.valve = valve
    req.reset_valve_position.new_pos_deg = pos
    send_request(req, f"RESET_VALVE_POSITION ({valve_name} → {pos:.2f}°)")


def _build_control_trace() -> clover_pb2.ControlTrace:
    """
    Interactively build a ControlTrace with one or more segments.
    Returns a populated ControlTrace protobuf object.
    """
    t = THEME
    total_time = IntPrompt.ask("  Total trace duration (ms)")

    trace = clover_pb2.ControlTrace()
    trace.total_time_ms = total_time

    while True:
        console.print(f"\n  [{t['primary']}]─── Add a segment ───[/{t['primary']}]")
        start_ms  = IntPrompt.ask("    Segment start time (ms)")
        length_ms = IntPrompt.ask("    Segment length (ms)")

        console.print("    Segment type:")
        console.print("      [1] Linear  (ramp from value A → B)")
        console.print("      [2] Sine    (oscillation)")
        seg_type = Prompt.ask("    Choose", choices=["1", "2"], default="1")

        seg = trace.segments.add()
        seg.start_ms  = start_ms
        seg.length_ms = length_ms

        if seg_type == "1":
            start_val = FloatPrompt.ask("    Start value")
            end_val   = FloatPrompt.ask("    End value")
            seg.linear.start_val = start_val
            seg.linear.end_val   = end_val
            console.print(f"    [{t['success']}]Linear segment: {start_val} → {end_val}[/{t['success']}]")
        else:
            offset    = FloatPrompt.ask("    Offset (baseline)")
            amplitude = FloatPrompt.ask("    Amplitude")
            period    = FloatPrompt.ask("    Period (ms)")
            phase_deg = FloatPrompt.ask("    Phase (degrees)", default=0.0)
            seg.sine.offset    = offset
            seg.sine.amplitude = amplitude
            seg.sine.period    = period
            seg.sine.phase_deg = phase_deg
            console.print(f"    [{t['success']}]Sine segment: offset={offset}, amp={amplitude}, T={period}ms[/{t['success']}]")

        another = Confirm.ask("  Add another segment?", default=False)
        if not another:
            break

    return trace


def cmd_load_motor_sequence():
    """Request #3 — Load a motor sequence (fuel and/or LOX traces)."""
    t = THEME
    console.print(f"\n  {t['icon_seq']} [{t['primary']}]Load Motor Sequence[/{t['primary']}]")
    console.print(f"  [{t['muted']}]You can define a control trace for FUEL, LOX, or both.[/{t['muted']}]")

    req = clover_pb2.Request()

    do_fuel = Confirm.ask("  Configure FUEL trace?", default=True)
    if do_fuel:
        console.print(f"\n  {t['icon_fuel']} [{t['warning']}]FUEL trace setup:[/{t['warning']}]")
        fuel_trace = _build_control_trace()
        req.load_motor_sequence.fuel_trace.CopyFrom(fuel_trace)

    do_lox = Confirm.ask("\n  Configure LOX trace?", default=True)
    if do_lox:
        console.print(f"\n  {t['icon_lox']} [{t['info']}]LOX trace setup:[/{t['info']}]")
        lox_trace = _build_control_trace()
        req.load_motor_sequence.lox_trace.CopyFrom(lox_trace)

    if not do_fuel and not do_lox:
        console.print(f"  [{t['warning']}]No traces configured — nothing sent.[/{t['warning']}]")
        return

    send_request(req, "LOAD_MOTOR_SEQUENCE")


def cmd_start_sequence():
    """Request #4 — Start the loaded sequence."""
    t = THEME
    confirmed = Confirm.ask(
        f"\n  {t['icon_fire']} [{t['warning']}]START SEQUENCE — are you sure?[/{t['warning']}]",
        default=False,
    )
    if not confirmed:
        console.print(f"  [{t['muted']}]Aborted.[/{t['muted']}]")
        return
    req = clover_pb2.Request()
    req.start_sequence.SetInParent()
    send_request(req, "START_SEQUENCE")


def cmd_halt_sequence():
    """Request #5 — Halt immediately (abort command)"""
    t = THEME
    confirmed = Confirm.ask(
        f"\n  {t['icon_stop']} [{t['danger']}]HALT / ABORT — stop all sequences immediately?[/{t['danger']}]",
        default=True,
    )
    if not confirmed:
        console.print(f"  [{t['muted']}]Aborted.[/{t['muted']}]")
        return
    req = clover_pb2.Request()
    req.halt_sequence.SetInParent()
    send_request(req, "HALT_SEQUENCE")


def cmd_start_throttle_closed_loop():
    """Request #7 — Start closed-loop throttle control with an optional thrust trace."""
    t = THEME
    console.print(f"\n  {t['icon_loop']} [{t['primary']}]Start Throttle Closed Loop[/{t['primary']}]")

    do_trace = Confirm.ask("  Include a thrust trace?", default=False)
    req = clover_pb2.Request()

    if do_trace:
        console.print(f"\n  [{t['info']}]Thrust trace setup:[/{t['info']}]")
        thrust_trace = _build_control_trace()
        req.start_throttle_closed_loop.thrust_trace.CopyFrom(thrust_trace)
    else:
        req.start_throttle_closed_loop.SetInParent()

    send_request(req, "START_THROTTLE_CLOSED_LOOP")


def cmd_set_controller_state():
    """Request #8 — Manually set the controller system state."""
    t = THEME
    console.print(f"\n  [{t['primary']}]Set Controller State:[/{t['primary']}]")

    states = [
        ("0", "STATE_IDLE",                 clover_pb2.STATE_IDLE),
        ("1", "STATE_SEQUENCE",             clover_pb2.STATE_SEQUENCE),
        ("2", "STATE_CLOSED_LOOP_THROTTLE", clover_pb2.STATE_CLOSED_LOOP_THROTTLE),
        ("3", "STATE_ABORT",                clover_pb2.STATE_ABORT),
        ("4", "STATE_CALIBRATION",          clover_pb2.STATE_CALIBRATION),
    ]
    for num, name, _ in states:
        console.print(f"    [{num}] {name}")

    choice = Prompt.ask("  Select state", choices=[s[0] for s in states])
    _, state_name, state_val = next(s for s in states if s[0] == choice)

    req = clover_pb2.Request()
    req.set_controller_state.state = state_val
    send_request(req, f"SET_CONTROLLER_STATE ({state_name})")


# options

MENU_ITEMS = [
    ("sub",    "subscribe",       "Subscribe to data stream",              cmd_subscribe_data_stream),
    ("id",     "identify",        "Identify this client",                  cmd_identify_client),
    ("reset",  "reset",           "Reset valve position",                  cmd_reset_valve_position),
    ("load",   "load",            "Load motor sequence (fuel/LOX traces)", cmd_load_motor_sequence),
    ("start",  "start",           "Start sequence",                        cmd_start_sequence),
    ("halt",   "halt",            "HALT / Abort immediately",              cmd_halt_sequence),
    ("cl",     "closedloop",      "Start throttle closed loop",            cmd_start_throttle_closed_loop),
    ("state",  "state",           "Set controller state",                  cmd_set_controller_state),
    ("status", "status",          "Live telemetry dashboard (Ctrl+C to exit)", None),  # handled inline
    ("quit",   "quit",            "Exit",                                  None),  # handled inline
]


def print_menu():
    t = THEME
    console.print()
    console.rule(f"[{t['header']}] {t['icon_fire']} CLOVER CLI {t['icon_fire']} [/{t['header']}]")
    console.print()

    table = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t["primary"],
        border_style=t["panel_border"],
        padding=(0, 2),
    )
    table.add_column("CMD",         style="bold white",     no_wrap=True)
    table.add_column("Description", style="white")

    icons = {
        "sub":    THEME["icon_live"],
        "id":     THEME["icon_id"],
        "reset":  THEME["icon_valve"],
        "load":   THEME["icon_seq"],
        "start":  THEME["icon_fire"],
        "halt":   THEME["icon_stop"],
        "cl":     THEME["icon_loop"],
        "state":  "🎛️ ",
        "status": THEME["icon_live"],
        "quit":   THEME["icon_quit"],
    }

    for key, alias, desc, _ in MENU_ITEMS:
        table.add_row(f"{icons.get(key,'')} {key}", desc)

    console.print(table)


def route_command(cmd: str) -> bool:
    """Route a typed command. Returns False if user wants to quit."""
    cmd = cmd.strip().lower()

    if cmd in ("quit", "exit", "q"):
        console.print(f"\n  {THEME['icon_stop']} [{THEME['warning']}]Exiting.[/{THEME['warning']}]\n")
        return False

    if cmd in ("status", "s"):
        cmd_live_status()
        return True

    if cmd in ("help", "h", "menu", "?", ""):
        print_menu()
        return True

    for key, alias, desc, fn in MENU_ITEMS:
        if cmd in (key, alias) and fn is not None:
            fn()
            return True

    console.print(f"  [{THEME['warning']}]Unknown command '{cmd}'. Type 'help' to see all commands.[/{THEME['warning']}]")
    return True


def main():
    global data_sock
    t = THEME

    parser = argparse.ArgumentParser(description="Clover ground station CLI")
    parser.add_argument("--no-data", action="store_true",
                        help="Skip binding the UDP telemetry port (for second instances)")
    args = parser.parse_args()

    telemetry_info = f"Listen port: {LOCAL_PORT}" if not args.no_data else "Telemetry: disabled"
    console.print(Panel.fit(
        f"[{t['header']}] {t['icon_fire']}  CLOVER GROUND STATION  {t['icon_fire']} [/{t['header']}]\n"
        f"[{t['muted']}]Target: {ZEPHYR_IP}:{ZEPHYR_PORT}  |  {telemetry_info}[/{t['muted']}]",
        border_style=t["panel_border"],
        padding=(1, 4),
    ))

    if not args.no_data:
        data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        data_sock.bind(("0.0.0.0", LOCAL_PORT))
        # auto-subscribe on startup
        console.print(f"\n  [{t['info']}]Auto-subscribing to data stream...[/{t['info']}]")
        cmd_subscribe_data_stream()
        # start telemetry listener
        threading.Thread(target=listen_for_telemetry, daemon=True).start()
        threading.Thread(target=_flush_loop, daemon=True).start()

    print_menu()

    session = PromptSession(bottom_toolbar=get_toolbar, refresh_interval=0.1)

    while True:
        try:
            cmd = session.prompt("\n  CMD> ")
        except (KeyboardInterrupt, EOFError):
            console.print(f"\n  {t['icon_stop']} [{t['warning']}]Exiting.[/{t['warning']}]\n")
            break

        if not route_command(cmd):
            break


if __name__ == "__main__":
    main()
