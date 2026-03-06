# improved CLI
# changes made AFTER .proto file updates

import argparse
import math
import struct
import socket
import threading
import sys
import time
import csv
import pathlib
from google.protobuf import text_format
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
from google.protobuf.internal.encoder import _VarintBytes
from rich.console import Group
from prompt_toolkit import PromptSession
from prompt_toolkit.formatted_text import HTML
import clickhouse_connect
import polars as pl


THEME = {
    "primary": "bold gold1",
    "success": "bold green",
    "warning": "bold yellow",
    "danger": "bold red",
    "info": "bold gold1",
    "muted": "dim white",
    "header": "bold white on dark_red",
    "panel_border": "dark_red",
    "icon_fire": "🔥",
    "icon_ok": "✅",
    "icon_warn": "⚠️ ",
    "icon_stop": "🛑",
    "icon_live": "📡",
    "icon_fuel": "🟡",
    "icon_lox": "🔴",
    "icon_valve": "🔧",
    "icon_seq": "▶️ ",
    "icon_loop": "🔄",
    "icon_id": "🪪",
    "icon_quit": "❌",
}

VALVE_SEQ_DIR = pathlib.Path("sequences/valve")
THRUST_SEQ_DIR = pathlib.Path("sequences/thrust")

# Network
ZEPHYR_IP = "169.254.99.99"  # real board
# ZEPHYR_IP   = "127.0.0.1"      # fake_telemetry.py
ZEPHYR_PORT = 19690
DATA_IP = "0.0.0.0"  # Listen to UDP from anybody
DATA_PORT = 19691

# ── ClickHouse config ────────────────────────────────────────────────────────
CH_HOST = "172.233.143.186"
CH_USER = "writer"
CH_PASSWORD = "ce8XpzhRGhsvBxCPHDTcvh6DMWhb3jyxgmQMNLrsKaCqtZvKf2"
CH_DATABASE = "lpl"

# CSV columns mirror the unpivoted ClickHouse raw_sensors schema exactly:
#   time   — nanosecond-epoch Int64  (ClickHouse 'time')
#   sensor — field name string       (ClickHouse 'sensor')
#   value  — Float64 reading         (ClickHouse 'value')
#   event  — state label for gnc_state rows, else ''  (ClickHouse 'event')
#   system — always 'atlas'          (ClickHouse 'system')
#   source — always 'gnc'            (ClickHouse 'source')
# PROTO CHANGE: time is now time_ns (nanoseconds), updated comment above.
CSV_COLUMNS = ["time", "sensor", "value", "event", "system", "source"]

# CSV filename is generated at exit time so it reflects when the session ended.
# Format: raw_sensors_YYYYMMDD_HHMMSS.csv


console = Console()
latest_packet: clover_pb2.DataPacket | None = None
packet_lock = threading.Lock()

data_sock = None

_packet_buffer: list = []
_buffer_lock = threading.Lock()

_csv_store: list = []  # list of (recv_time: float, pkt: DataPacket); drained each second
_csv_store_lock = threading.Lock()
_csv_path: pathlib.Path | None = None  # set on first write
_csv_fh = None  # open file handle
_csv_writer = None  # csv.DictWriter bound to _csv_fh
_csv_rows_written: int = 0

_last_packet_time: float = 0.0
_last_packet_lock = threading.Lock()


def _make_tcp_socket() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2.0)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    s.setsockopt(
        socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5
    )  # start probes after 5s idle
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)  # probe every 2s
    s.setsockopt(
        socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3
    )  # drop after 3 missed probes
    s.connect((ZEPHYR_IP, ZEPHYR_PORT))
    return s


sock = _make_tcp_socket()
# ── Telemetry listener ───────────────────────────────────────────────────────


def listen_for_telemetry():
    """Background thread — receives UDP DataPackets and stores latest."""
    global latest_packet, _last_packet_time
    while True:
        try:
            data, _ = data_sock.recvfrom(4096)
            packet = clover_pb2.DataPacket()
            packet.ParseFromString(data)
            recv_time = time.time()
            for lock, name in (
                (packet_lock, "packet_lock"),
                (_last_packet_lock, "_last_packet_lock"),
                (_buffer_lock, "_buffer_lock"),
                (_csv_store_lock, "_csv_store_lock"),
            ):
                t0 = time.monotonic()
                lock.acquire()
                wait = time.monotonic() - t0
                if wait > 0.05:
                    console.print(
                        f"  [bold yellow]LOCK SLOW: {name} waited {wait:.3f}s[/bold yellow]"
                    )
                try:
                    if lock is packet_lock:
                        latest_packet = packet
                    elif lock is _last_packet_lock:
                        _last_packet_time = recv_time
                    elif lock is _buffer_lock:
                        _packet_buffer.append((recv_time, packet))
                    else:
                        _csv_store.append((recv_time, packet))
                finally:
                    lock.release()
        except socket.timeout:
            continue  # loop back and re-read data_sock global
        except Exception as e:
            console.print(
                f"  [bold red]listen_for_telemetry error:[/bold red] {type(e).__name__}: {e}"
            )
            continue


# update SystemState with (0–7); range 8
_STATE_NAMES = {float(i): clover_pb2.SystemState.Name(i) for i in range(8)}


# ── Data flattening ──────────────────────────────────────────────────────────


def _packet_to_row(recv_time: float, pkt: clover_pb2.DataPacket) -> dict:
    """
    Flatten one DataPacket into a wide dict of numeric sensors for ClickHouse.

    PROTO CHANGES applied here:
      - pkt.time   → pkt.time_ns  (field renamed; now nanoseconds uint64)
      - pkt.sensors (old Sensors message) → pkt.analog_sensors (new AnalogSensors)
      - AnalogSensors fields are now required (no HasField checks needed)
      - Two new TC sensors added: tc102, tc102_5
      - adc_read_time_ns added as a timing metric
      - ValveStatus lost 'enabled', gained 'is_on' (is_on stored as float 0/1)
      - pkt.is_abort removed — abort is now a SystemState (STATE_ABORT)
      - controller_tick_time_ns added as a timing metric
    """
    s = pkt.analog_sensors
    f = pkt.fuel_valve
    l = pkt.lox_valve
    return {
        "time": int(recv_time * 1e6),
        "gnc_state": float(pkt.state),
        "gnc_data_queue_size": float(pkt.data_queue_size),
        "gnc_sequence_number": float(pkt.sequence_number),
        "gnc_controller_tick_ns": float(pkt.controller_tick_time_ns),  # new field
        "gnc_pt102": float(s.pt102),
        "gnc_pt103": float(s.pt103),
        "gnc_pt202": float(s.pt202),
        "gnc_pt203": float(s.pt203),
        "gnc_ptf401": float(s.ptf401),
        "gnc_pto401": float(s.pto401),
        "gnc_ptc401": float(s.ptc401),
        "gnc_ptc402": float(s.ptc402),
        "gnc_tc102": float(s.tc102),  # new sensor
        "gnc_tc102_5": float(s.tc102_5),  # new sensor
        "gnc_adc_read_time_ns": float(s.adc_read_time_ns),  # new timing
        # Fuel valve
        "gnc_fuel_target": float(f.target_pos_deg),
        "gnc_fuel_driver": float(f.driver_setpoint_pos_deg),
        "gnc_fuel_encoder": float(f.encoder_pos_deg),
        "gnc_fuel_is_on": float(f.is_on),  # PROTO CHANGE: was 'enabled'
        # LOX valve — same changes as fuel
        "gnc_lox_target": float(l.target_pos_deg),
        "gnc_lox_driver": float(l.driver_setpoint_pos_deg),
        "gnc_lox_encoder": float(l.encoder_pos_deg),
        "gnc_lox_is_on": float(l.is_on),  # PROTO CHANGE: was 'enabled'
    }


def _packet_to_csv_rows(recv_time: float, pkt: clover_pb2.DataPacket) -> list[dict]:
    """
    Expand one DataPacket into multiple CSV rows (one per sensor/field),
    matching the unpivoted schema written to ClickHouse raw_sensors.
    """
    wide = _packet_to_row(recv_time, pkt)
    ts = wide["time"]
    rows = []

    for sensor, raw_value in wide.items():
        if sensor == "time" or raw_value is None:
            continue

        value = float(raw_value)
        # event column: state label for gnc_state rows, empty string otherwise
        event = _STATE_NAMES.get(raw_value, "") if sensor == "gnc_state" else ""

        rows.append(
            {
                "time": ts,
                "sensor": sensor,
                "value": value,
                "event": event,
                "system": "atlas",  # change if not atlas
                "source": "gnc",  # leave as all GNC for now
            }
        )

    return rows


def _write_csv_on_exit():
    """Flush any remaining buffered packets to the CSV file and close it."""
    global _csv_fh, _csv_writer, _csv_rows_written, _csv_path

    with _csv_store_lock:
        remaining = list(_csv_store)
        _csv_store.clear()

    if remaining and _csv_fh is None:
        # Client exited before the first flush_loop drain (ran very briefly)
        _csv_path = pathlib.Path(f"raw_sensors_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        _csv_fh = open(_csv_path, "w", newline="")
        _csv_writer = csv.DictWriter(_csv_fh, fieldnames=CSV_COLUMNS)
        _csv_writer.writeheader()

    if _csv_fh is None:
        console.print(
            f"  [{THEME['muted']}]No telemetry received — CSV not written.[/{THEME['muted']}]"
        )
        return

    for recv_time, pkt in remaining:
        try:
            for row in _packet_to_csv_rows(recv_time, pkt):
                _csv_writer.writerow(row)
                _csv_rows_written += 1
        except Exception as e:
            console.print(f"  [bold red]CSV row error:[/bold red] {e}")

    _csv_fh.close()
    _csv_fh = None
    console.print(
        f"\n  {THEME['icon_ok']} [bold green]CSV saved →[/bold green] "
        f"[dim]{_csv_path.resolve()}[/dim]\n"
        f"  [{THEME['muted']}]{_csv_rows_written:,} rows written[/{THEME['muted']}]"
    )


# ── ClickHouse flush ─────────────────────────────────────────────────────────

_STREAM_TIMEOUT = 5.0  # seconds without a packet before we re-subscribe


def _reconnect_and_resubscribe():
    """Reconnect the TCP socket and re-issue a subscribe request."""
    global sock
    try:
        sock.close()
    except Exception:
        pass
    try:
        sock = _make_tcp_socket()
        cmd_subscribe_data_stream()
        console.print(
            f"  [{THEME['warning']}]Reconnected and re-subscribed to data stream.[/{THEME['warning']}]"
        )
    except Exception as e:
        console.print(f"  [{THEME['danger']}]Reconnect failed: {e}[/{THEME['danger']}]")


def _flush_loop():
    """Background thread — drains the packet buffer into ClickHouse every second."""
    global _last_packet_time
    ch = None
    while True:
        time.sleep(1.0)

        # # Re-subscribe if no packets received recently
        # with _last_packet_lock:
        #     idle = time.time() - _last_packet_time
        # if _last_packet_time > 0 and idle > _STREAM_TIMEOUT:
        #     console.print(f"  [{THEME['warning']}]No telemetry for {idle:.1f}s — reconnecting...[/{THEME['warning']}]")
        #     with packet_lock:
        #         last_pkt = latest_packet
        #     if last_pkt:
        #         console.print(f"  [dim]Last packet: seq={last_pkt.sequence_number} queue={last_pkt.data_queue_size} time_ns={last_pkt.time_ns}[/dim]")
        #     _reconnect_and_resubscribe()
        #     with _last_packet_lock:
        #         _last_packet_time = time.time()

        if ch is None:
            try:
                ch = clickhouse_connect.get_client(
                    host=CH_HOST,
                    username=CH_USER,
                    password=CH_PASSWORD,
                    database=CH_DATABASE,
                )
            except Exception as e:
                console.print(f"  [bold red]ClickHouse connect error:[/bold red] {e}")
                continue

        with _buffer_lock:
            if not _packet_buffer:
                continue
            batch = list(_packet_buffer)
            _packet_buffer.clear()

        try:
            rows = [_packet_to_row(t, p) for t, p in batch]
            df = (
                pl.DataFrame(rows)
                .unpivot(index=["time"], variable_name="sensor", value_name="value")
                .drop_nulls("value")
                .with_columns(
                    pl.from_epoch("time", time_unit="us").alias("time"),
                    pl.col("value").cast(pl.Float64),
                    pl.when(pl.col("sensor") == "gnc_state")
                    .then(
                        pl.col("value").map_elements(
                            lambda v: _STATE_NAMES.get(v, ""), return_dtype=pl.String
                        )
                    )
                    .otherwise(pl.lit(""))
                    .alias("event"),
                    pl.lit("atlas").alias("system"),
                    pl.lit("gnc").alias("source"),
                )
            )
            ch.insert_df_arrow("raw_sensors", df)
        except Exception as e:
            console.print(
                f"  [bold red]ClickHouse insert error (will reconnect):[/bold red] {e}"
            )
            ch = None

        # ── CSV incremental flush ────────────────────────────────────────────
        with _csv_store_lock:
            if _csv_store:
                csv_batch = list(_csv_store)
                _csv_store.clear()
            else:
                csv_batch = []

        if csv_batch:
            global _csv_path, _csv_fh, _csv_writer, _csv_rows_written
            if _csv_fh is None:
                _csv_path = pathlib.Path(
                    f"raw_sensors_{time.strftime('%Y%m%d_%H%M%S')}.csv"
                )
                _csv_fh = open(_csv_path, "w", newline="")
                _csv_writer = csv.DictWriter(_csv_fh, fieldnames=CSV_COLUMNS)
                _csv_writer.writeheader()
            try:
                for recv_time, pkt in csv_batch:
                    for row in _packet_to_csv_rows(recv_time, pkt):
                        _csv_writer.writerow(row)
                        _csv_rows_written += 1
                _csv_fh.flush()
            except Exception as e:
                console.print(f"  [bold red]CSV write error:[/bold red] {e}")


# ── Live status display ──────────────────────────────────────────────────────

STATE_COLORS = {
    "STATE_UNKNOWN": "dim white",
    "STATE_IDLE": "green",
    "STATE_CALIBRATE_VALVE": "magenta",
    "STATE_VALVE_PRIMED": "cyan",
    "STATE_VALVE_SEQ": "gold1",
    "STATE_THRUST_PRIMED": "cyan",
    "STATE_THRUST_SEQ": "yellow",
    "STATE_ABORT": "bold red",
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

    state_name = clover_pb2.SystemState.Name(pkt.state)
    state_color = STATE_COLORS.get(state_name, "white")

    # abort is now STATE_ABORT in state enum
    is_abort = state_name == "STATE_ABORT"
    abort_str = (
        f"[bold red]{t['icon_stop']} ABORT[/bold red]"
        if is_abort
        else "[green]nominal[/green]"
    )

    # ── Header ────────────────────────────────────────────────
    hdr = Table.grid(padding=(0, 3))
    for _ in range(5):
        hdr.add_column()
    hdr.add_row(
        Text(f"{t['icon_live']} LIVE", style="bold green"),
        Text(f"State: {state_name}", style=state_color),
        # in nanoseconds
        Text(f"t = {pkt.time_ns / 1e9:.3f} s", style="white"),
        Text(f"seq #{pkt.sequence_number}", style=t["muted"]),
        Text(f"queue: {pkt.data_queue_size}", style=t["muted"]),
    )
    header_panel = Panel(
        hdr, border_style=t["panel_border"], padding=(0, 1), subtitle=abort_str
    )

    # ── Valve table ───────────────────────────────────────────
    vt = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t["primary"],
        border_style=t["panel_border"],
        padding=(0, 1),
    )
    vt.add_column("Valve", style="bold white", no_wrap=True)
    vt.add_column("On", no_wrap=True)
    vt.add_column("Target °", style="white", no_wrap=True)
    vt.add_column("Driver °", style="white", no_wrap=True)
    vt.add_column("Encoder °", style="white", no_wrap=True)

    for label, icon, color, v in [
        ("FUEL", t["icon_fuel"], "gold1", pkt.fuel_valve),
        ("LOX", t["icon_lox"], "dark_red", pkt.lox_valve),
    ]:
        vt.add_row(
            f"{icon} [{color}]{label}[/{color}]",
            "[green]YES[/green]" if v.is_on else "[red]NO[/red]",
            f"{v.target_pos_deg:.2f}",
            f"{v.driver_setpoint_pos_deg:.2f}",
            f"{v.encoder_pos_deg:.2f}",
        )

    # ── Sensor table ──────────────────────────────────────────
    #               Added TC-102, TC-102.5, ADC read time rows
    st = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t["primary"],
        border_style=t["panel_border"],
        padding=(0, 1),
    )
    st.add_column("Sensor", style="bold white", no_wrap=True)
    st.add_column("Value", style="white", no_wrap=True, justify="right")
    st.add_column("Unit", style=t["muted"], no_wrap=True)

    s = pkt.analog_sensors
    for name, val, unit in [
        ("PT-102", s.pt102, "psi"),
        ("PT-103", s.pt103, "psi"),
        ("PT-202", s.pt202, "psi"),
        ("PT-203", s.pt203, "psi"),
        ("PTF-401", s.ptf401, "psi"),
        ("PTO-401", s.pto401, "psi"),
        ("PTC-401", s.ptc401, "psi"),
        ("PTC-402", s.ptc402, "psi"),
        ("ADC t", s.adc_read_time_ns / 1000, "μs"),
    ]:
        val_str = f"{val:.2f}" if val != 0.0 else f"[{t['muted']}]—[/{t['muted']}]"
        st.add_row(name, val_str, unit)

    bottom = Columns(
        [
            Panel(
                vt,
                title=f"[{t['primary']}]Valves[/{t['primary']}]",
                border_style=t["panel_border"],
            ),
            Panel(
                st,
                title=f"[{t['primary']}]Sensors[/{t['primary']}]",
                border_style=t["panel_border"],
            ),
        ]
    )

    return Group(header_panel, bottom)


def cmd_live_status():
    """Stream full telemetry panel at 5 Hz (200 ms). Press Enter to return to menu."""
    stop = threading.Event()

    def _wait_for_enter():
        try:
            sys.stdin.readline()
        except Exception:
            pass
        stop.set()

    console.print(
        f"  [{THEME['muted']}]Live view — press Enter to return to menu[/{THEME['muted']}]"
    )
    threading.Thread(target=_wait_for_enter, daemon=True).start()

    try:
        with Live(
            _build_status_renderable(), refresh_per_second=5, screen=False
        ) as live:
            while not stop.is_set():
                time.sleep(0.2)
                live.update(_build_status_renderable())
    except KeyboardInterrupt:
        pass


_TOOLBAR_STATE_TAGS = {
    "STATE_UNKNOWN": ("ansiwhite", "UNKNOWN"),
    "STATE_IDLE": ("ansigreen", "IDLE"),
    "STATE_CALIBRATE_VALVE": ("ansimagenta", "CAL_VALVE"),
    "STATE_VALVE_PRIMED": ("ansicyan", "VLV_PRIMED"),
    "STATE_VALVE_SEQ": ("ansiyellow", "VLV_SEQ"),
    "STATE_THRUST_PRIMED": ("ansicyan", "THR_PRIMED"),
    "STATE_THRUST_SEQ": ("ansiyellow", "THR_SEQ"),
    "STATE_ABORT": ("ansired", "ABORT"),
}


def get_toolbar():
    """Compact live telemetry for the prompt_toolkit bottom toolbar."""
    with packet_lock:
        pkt = latest_packet

    if pkt is None:
        return HTML(" <b>📡</b> No telemetry yet...")

    state_name = clover_pb2.SystemState.Name(pkt.state)
    tag, short = _TOOLBAR_STATE_TAGS.get(state_name, ("ansiwhite", state_name))

    is_abort = state_name == "STATE_ABORT"
    abort_html = "  <ansired><b>🛑 ABORT</b></ansired>" if is_abort else ""

    s = pkt.analog_sensors
    sensor_parts = [
        f"{lbl}: {val:.1f}"
        for lbl, val in [
            ("PT-F401", s.ptf401),
            ("PT-O401", s.pto401),
            ("PT-C401", s.ptc401),
            ("PT-C402", s.ptc402),
            ("TC-102", s.tc102),
            ("TC-102.5", s.tc102_5),  # new sensors
        ]
        if val != 0.0
    ]
    sensor_html = ("  │  " + "  ".join(sensor_parts)) if sensor_parts else ""

    f = pkt.fuel_valve
    l = pkt.lox_valve
    return HTML(
        # 1e9 for seconds display
        f" 📡 <{tag}><b>{short}</b></{tag}>"
        f"  │  t={pkt.time_ns / 1e9:.2f}s  seq#{pkt.sequence_number}"
        f"  │  🟡 drv={f.driver_setpoint_pos_deg:.1f}°  enc={f.encoder_pos_deg:.1f}°"
        f"  │  🔴 drv={l.driver_setpoint_pos_deg:.1f}°  enc={l.encoder_pos_deg:.1f}°"
        + sensor_html
        + abort_html
    )


# ── TCP sender ───────────────────────────────────────────────────────────────


def _recv_response() -> clover_pb2.Response:
    """Read a varint-length-prefixed Response from the TCP socket."""
    length = 0
    shift = 0
    while True:
        b = sock.recv(1)
        if not b:
            raise ConnectionError("Connection closed while reading response length")
        byte = b[0]
        length |= (byte & 0x7F) << shift
        if not (byte & 0x80):
            break
        shift += 7

    data = b""
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk:
            raise ConnectionError("Connection closed while reading response body")
        data += chunk

    resp = clover_pb2.Response()
    resp.ParseFromString(data)
    return resp


def send_request(req: clover_pb2.Request, label: str) -> bool:
    global sock
    """Serialize and send a Request over TCP, then read and display the Response."""
    raw = req.SerializeToString()
    payload = _VarintBytes(len(raw)) + raw
    for attempt in range(2):
        try:
            sock.sendall(payload)
            break
        except Exception as e:
            if attempt == 0:
                console.print(
                    f"\n  {THEME['icon_warn']} [{THEME['warning']}]Connection lost, reconnecting...[/{THEME['warning']}]"
                )
                try:
                    sock = _make_tcp_socket()
                except Exception as re:
                    console.print(
                        f"\n  {THEME['icon_warn']} [{THEME['danger']}]Reconnect failed: {re}[/{THEME['danger']}]\n"
                    )
                    return False
            else:
                console.print(
                    f"\n  {THEME['icon_warn']} [{THEME['danger']}]Failed to send {label}: {e}[/{THEME['danger']}]\n"
                )
                return False

    try:
        resp = _recv_response()
        if resp.HasField("err"):
            console.print(
                f"\n  {THEME['icon_warn']} [{THEME['danger']}]{label} rejected: {resp.err}[/{THEME['danger']}]\n"
            )
            return False
        console.print(
            f"\n  {THEME['icon_ok']} [{THEME['success']}]Sent {label} → {ZEPHYR_IP}:{ZEPHYR_PORT}[/{THEME['success']}]\n"
        )
        return True
    except Exception as e:
        console.print(
            f"\n  {THEME['icon_warn']} [{THEME['warning']}]{label} sent but no response: {e}[/{THEME['warning']}]\n"
        )
        return True


# comment out to test in terminal without real data
# def send_request(req, label):
#     console.print(f"\n  {THEME['icon_ok']} [bold green]Would send the following: {label}[/bold green]\n")
#     return True


# ── Command implementations ──────────────────────────────────────────────────


def cmd_subscribe_data_stream():
    """Subscribe to telemetry data stream."""
    req = clover_pb2.Request()
    req.subscribe_data_stream.SetInParent()
    send_request(req, "SUBSCRIBE_DATA_STREAM")


def cmd_identify_client():
    """Identify client as GNC."""
    t = THEME
    console.print(
        f"\n  {t['icon_id']} [{t['primary']}]Client identity: GNC[/{t['primary']}]"
    )
    req = clover_pb2.Request()
    req.identify_client.client = clover_pb2.GNC
    send_request(req, "IDENTIFY_CLIENT (GNC)")


def cmd_is_not_aborted():
    """PROTO CHANGE: new request — check that system is not in ABORT state."""
    req = clover_pb2.Request()
    req.is_not_aborted_request.SetInParent()
    send_request(req, "IS_NOT_ABORTED")


def cmd_configure_sensor_bias():
    """PROTO CHANGE: new request — set analog bias for a PT or TC sensor."""
    t = THEME
    console.print(f"\n  [{t['primary']}]Configure Analog Sensor Bias[/{t['primary']}]")

    sensors = [
        ("1", "PT-102", clover_pb2.PT102),
        ("2", "PT-103", clover_pb2.PT103),
        ("3", "PT-202", clover_pb2.PT202),
        ("4", "PT-203", clover_pb2.PT203),
        ("5", "PT-F401", clover_pb2.PTF401),
        ("6", "PT-O401", clover_pb2.PTO401),
        ("7", "PT-C401", clover_pb2.PTC401),
        ("8", "PT-C402", clover_pb2.PTC402),
        ("9", "TC-102", clover_pb2.TC102),
        ("10", "TC-102.5", clover_pb2.TC102_5),
    ]
    for num, name, _ in sensors:
        console.print(f"    [{num:>2}] {name}")

    choice = Prompt.ask("  Select sensor", choices=[s[0] for s in sensors])
    _, sensor_name, sensor_val = next(s for s in sensors if s[0] == choice)

    bias = FloatPrompt.ask(f"  Bias value for {sensor_name}")

    req = clover_pb2.Request()
    req.configure_analog_sensors_bias.sensor = sensor_val
    req.configure_analog_sensors_bias.bias = bias
    send_request(req, f"CONFIGURE_SENSOR_BIAS ({sensor_name} bias={bias:.4f})")


def cmd_reset_valve_position():
    """Reset a valve to a specified degree position."""
    t = THEME
    console.print(
        f"\n  {t['icon_valve']} [{t['primary']}]Reset Valve Position[/{t['primary']}]"
    )
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


def cmd_power_on_valve():
    """PROTO CHANGE: new request — power on (enable stepper motor) for a valve."""
    t = THEME
    console.print(f"\n  [{t['primary']}]Power ON Valve[/{t['primary']}]")
    console.print("    [1] FUEL")
    console.print("    [2] LOX")

    choice = Prompt.ask("  Select valve", choices=["1", "2"])
    valve = clover_pb2.FUEL if choice == "1" else clover_pb2.LOX
    valve_name = "FUEL" if choice == "1" else "LOX"

    req = clover_pb2.Request()
    req.power_on_valve.valve = valve
    send_request(req, f"POWER_ON_VALVE ({valve_name})")


def cmd_power_off_valve():
    """Power off (disable stepper motor) for a valve."""
    t = THEME
    console.print(f"\n  [{t['primary']}]Power OFF Valve[/{t['primary']}]")
    console.print("    [1] FUEL")
    console.print("    [2] LOX")

    choice = Prompt.ask("  Select valve", choices=["1", "2"])
    valve = clover_pb2.FUEL if choice == "1" else clover_pb2.LOX
    valve_name = "FUEL" if choice == "1" else "LOX"

    req = clover_pb2.Request()
    req.power_off_valve.valve = valve
    send_request(req, f"POWER_OFF_VALVE ({valve_name})")


def cmd_abort():
    """Abort active sequence into safe state."""
    t = THEME
    confirmed = Confirm.ask(
        f"\n  {t['icon_stop']} [{t['danger']}]ABORT — stop active sequence immediately?[/{t['danger']}]",
        default=True,
    )
    if not confirmed:
        console.print(f"  [{t['muted']}]Cancelled.[/{t['muted']}]")
        return
    req = clover_pb2.Request()
    req.abort.SetInParent()
    send_request(req, "ABORT")


def cmd_halt():
    """Halt all actuators."""
    t = THEME
    confirmed = Confirm.ask(
        f"\n  {t['icon_stop']} [{t['danger']}]HALT — stop all actuators immediately?[/{t['danger']}]",
        default=True,
    )
    if not confirmed:
        console.print(f"  [{t['muted']}]Cancelled.[/{t['muted']}]")
        return
    req = clover_pb2.Request()
    req.halt.SetInParent()
    send_request(req, "HALT")


def cmd_unprime():
    """Unprime (VALVE_PRIMED/THRUST_PRIMED → IDLE)."""
    t = THEME
    confirmed = Confirm.ask(
        f"\n  [{t['warning']}]UNPRIME — cancel loaded sequence and return to IDLE?[/{t['warning']}]",
        default=False,
    )
    if not confirmed:
        console.print(f"  [{t['muted']}]Cancelled.[/{t['muted']}]")
        return
    req = clover_pb2.Request()
    req.unprime.SetInParent()
    send_request(req, "UNPRIME")


def cmd_calibrate_valve():
    """enter valve calibration mode (IDLE → CALIBRATE_VALVE)."""
    t = THEME
    console.print(
        f"\n  {t['icon_valve']} [{t['primary']}]Calibrate Valve[/{t['primary']}]"
    )
    console.print("    [1] FUEL")
    console.print("    [2] LOX")

    choice = Prompt.ask("  Select valve", choices=["1", "2"])
    valve = clover_pb2.FUEL if choice == "1" else clover_pb2.LOX
    valve_name = "FUEL" if choice == "1" else "LOX"

    req = clover_pb2.Request()
    req.calibrate_valve.valve = valve
    send_request(req, f"CALIBRATE_VALVE ({valve_name})")


def _list_saved_sequences(subdir: pathlib.Path) -> list[pathlib.Path]:
    if not subdir.exists():
        return []
    return sorted(subdir.glob("*.textproto"))


def _pick_and_load_sequence(subdir: pathlib.Path, msg_class):
    """Display saved sequences, prompt user to pick one, return parsed proto."""
    files = _list_saved_sequences(subdir)
    t = THEME
    table = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t["primary"],
        border_style=t["panel_border"],
        padding=(0, 2),
    )
    table.add_column("#", style="bold white", no_wrap=True)
    table.add_column("Name", style="white")
    for i, f in enumerate(files, 1):
        table.add_row(str(i), f.stem)
    console.print(table)

    choice = Prompt.ask("  Select", choices=[str(i) for i in range(1, len(files) + 1)])
    chosen = files[int(choice) - 1]
    msg = msg_class()
    text_format.Parse(chosen.read_text(), msg)
    console.print(f"  [{t['success']}]Loaded: {chosen.stem}[/{t['success']}]")
    return msg


def _save_sequence(subdir: pathlib.Path, prefix: str, msg):
    """Prompt for a name and save a proto message as a .textproto file."""
    t = THEME
    subdir.mkdir(parents=True, exist_ok=True)
    name = Prompt.ask("  Sequence name")
    filename = subdir / f"{prefix}_{name}.textproto"
    filename.write_text(text_format.MessageToString(msg))
    console.print(f"  [{t['success']}]Saved → {filename}[/{t['success']}]")


def _f32(v: float) -> float:
    """Round a Python float to float32 precision (matches firmware storage/arithmetic)."""
    return struct.unpack("f", struct.pack("f", v))[0]


def _sine_sample_f32(
    offset: float, amplitude: float, period: float, phase_deg: float, t_ms: int
) -> float:
    """
    Replicate firmware's float32 sine sample:
      offset + amplitude * sin(t / period * TAU + phase_deg / 360 * TAU)
    where TAU = 2.0f * pi_v<float> and every operation is float32.
    """
    TAU = _f32(2.0 * math.pi)
    o = _f32(offset)
    a = _f32(amplitude)
    per = _f32(period)
    ph = _f32(phase_deg)
    t = _f32(float(t_ms))
    arg = _f32(_f32(t / per) * TAU + _f32(ph / _f32(360.0)) * TAU)
    return _f32(o + _f32(a * _f32(math.sin(arg))))


def _build_control_trace() -> clover_pb2.ControlTrace:
    """Interactively build a ControlTrace with one or more segments."""
    t = THEME
    trace = clover_pb2.ControlTrace()

    cursor_ms = 0
    cursor_val: float | None = None  # unknown until first segment
    while True:
        pos_str = f"{cursor_val:.2f}" if cursor_val is not None else "?"
        console.print(
            f"\n  [{t['primary']}]─── Add a segment (t={cursor_ms} ms, pos={pos_str}) ───[/{t['primary']}]"
        )
        length_ms = IntPrompt.ask("    Segment length (ms)")

        console.print("    Segment type:")
        console.print("      [1] Linear  (ramp from value A → B)")
        console.print("      [2] Sine    (oscillation)")
        seg_type = Prompt.ask("    Choose", choices=["1", "2"], default="1")

        seg = trace.segments.add()
        seg.start_ms = cursor_ms
        seg.length_ms = length_ms
        cursor_ms += length_ms

        if seg_type == "1":
            if cursor_val is not None:
                console.print(
                    f"    [{t['muted']}]Start value locked to {cursor_val:.2f}[/{t['muted']}]"
                )
                start_val = cursor_val
            else:
                start_val = FloatPrompt.ask("    Start value")
            end_val = FloatPrompt.ask("    End value")
            seg.linear.start_val = start_val
            seg.linear.end_val = end_val
            cursor_val = end_val
            console.print(
                f"    [{t['success']}]Linear segment: {start_val} → {end_val}[/{t['success']}]"
            )
        else:
            amplitude = FloatPrompt.ask("    Amplitude")
            period = FloatPrompt.ask("    Period (ms)")
            phase_deg = FloatPrompt.ask("    Phase (degrees)", default=0.0)
            phase_rad = math.radians(phase_deg)
            if cursor_val is not None:
                offset = cursor_val - amplitude * math.sin(phase_rad)
                console.print(
                    f"    [{t['muted']}]Offset auto-set to {offset:.4f} for continuity[/{t['muted']}]"
                )
            else:
                offset = FloatPrompt.ask("    Offset (baseline)")
            end_val = _sine_sample_f32(offset, amplitude, period, phase_deg, length_ms)
            seg.sine.offset = offset
            seg.sine.amplitude = amplitude
            seg.sine.period = period
            seg.sine.phase_deg = phase_deg
            cursor_val = end_val
            console.print(
                f"    [{t['success']}]Sine segment: offset={offset:.4f}, amp={amplitude}, T={period}ms → ends at {end_val:.2f}[/{t['success']}]"
            )

        another = Confirm.ask("  Add another segment?", default=False)
        if not another:
            break

    trace.total_time_ms = cursor_ms
    console.print(
        f"  [{THEME['muted']}]Total trace duration: {cursor_ms} ms[/{THEME['muted']}]"
    )
    return trace


def cmd_load_valve_sequence():
    """
    Load a valve sequence (IDLE → VALVE_PRIMED). At least one trace required.
    New field names: fuel_trace_deg, lox_trace_deg
    """
    t = THEME
    console.print(
        f"\n  {t['icon_seq']} [{t['primary']}]Load Valve Sequence[/{t['primary']}]"
    )

    saved = _list_saved_sequences(VALVE_SEQ_DIR)
    if saved and Confirm.ask("  Load a saved sequence?", default=True):
        loaded = _pick_and_load_sequence(
            VALVE_SEQ_DIR, clover_pb2.LoadValveSequenceRequest
        )
        req = clover_pb2.Request()
        req.load_valve_sequence.CopyFrom(loaded)
        send_request(req, "LOAD_VALVE_SEQUENCE")
        return

    console.print(
        f"  [{t['muted']}]Define a control trace for FUEL, LOX, or both. At least one required.[/{t['muted']}]"
    )

    req = clover_pb2.Request()

    do_fuel = Confirm.ask("  Configure FUEL trace (degrees)?", default=True)
    if do_fuel:
        console.print(
            f"\n  {t['icon_fuel']} [{t['warning']}]FUEL trace setup:[/{t['warning']}]"
        )
        fuel_trace = _build_control_trace()
        req.load_valve_sequence.fuel_trace_deg.CopyFrom(fuel_trace)

    do_lox = Confirm.ask("\n  Configure LOX trace (degrees)?", default=True)
    if do_lox:
        console.print(
            f"\n  {t['icon_lox']} [{t['info']}]LOX trace setup:[/{t['info']}]"
        )
        lox_trace = _build_control_trace()
        req.load_valve_sequence.lox_trace_deg.CopyFrom(lox_trace)

    if not do_fuel and not do_lox:
        console.print(
            f"  [{t['danger']}]At least one trace is required — nothing sent.[/{t['danger']}]"
        )
        return

    if Confirm.ask("  Save this sequence for later?", default=False):
        _save_sequence(VALVE_SEQ_DIR, "v", req.load_valve_sequence)

    send_request(req, "LOAD_VALVE_SEQUENCE")


def cmd_start_valve_sequence():
    """Replacement of start_sequence"""
    t = THEME
    confirmed = Confirm.ask(
        f"\n  {t['icon_fire']} [{t['warning']}]START VALVE SEQUENCE — are you sure?[/{t['warning']}]",
        default=False,
    )
    if not confirmed:
        console.print(f"  [{t['muted']}]Cancelled.[/{t['muted']}]")
        return
    req = clover_pb2.Request()
    req.start_valve_sequence.SetInParent()
    send_request(req, "START_VALVE_SEQUENCE")


def cmd_load_thrust_sequence():
    """
    Load a thrust sequence (IDLE → THRUST_PRIMED).
    Thrust trace is in lbf
    """
    t = THEME
    console.print(
        f"\n  {t['icon_loop']} [{t['primary']}]Load Thrust Sequence[/{t['primary']}]"
    )

    saved = _list_saved_sequences(THRUST_SEQ_DIR)
    if saved and Confirm.ask("  Load a saved sequence?", default=True):
        loaded = _pick_and_load_sequence(
            THRUST_SEQ_DIR, clover_pb2.LoadThrustSequenceRequest
        )
        req = clover_pb2.Request()
        req.load_thrust_sequence.CopyFrom(loaded)
        send_request(req, "LOAD_THRUST_SEQUENCE")
        return

    console.print(f"  [{t['muted']}]Thrust trace values are in lbf.[/{t['muted']}]")

    thrust_trace = _build_control_trace()

    req = clover_pb2.Request()
    req.load_thrust_sequence.thrust_trace_lbf.CopyFrom(thrust_trace)

    if Confirm.ask("  Save this sequence for later?", default=False):
        _save_sequence(THRUST_SEQ_DIR, "t", req.load_thrust_sequence)

    send_request(req, "LOAD_THRUST_SEQUENCE")


def cmd_start_thrust_sequence():
    """Start thrust sequence."""
    t = THEME
    confirmed = Confirm.ask(
        f"\n  {t['icon_fire']} [{t['warning']}]START THRUST SEQUENCE — are you sure?[/{t['warning']}]",
        default=False,
    )
    if not confirmed:
        console.print(f"  [{t['muted']}]Cancelled.[/{t['muted']}]")
        return
    req = clover_pb2.Request()
    req.start_thrust_sequence.SetInParent()
    send_request(req, "START_THRUST_SEQUENCE")


# ── Menu ─────────────────────────────────────────────────────────────────────

# changed options based on updated .proto file
MENU_ITEMS = [
    ("sub", "subscribe", "Subscribe to data stream", cmd_subscribe_data_stream),
    ("id", "identify", "Identify this client (GNC)", cmd_identify_client),
    ("check", "check", "Check system is not aborted", cmd_is_not_aborted),
    ("bias", "bias", "Configure analog sensor bias", cmd_configure_sensor_bias),
    ("reset", "reset", "Reset valve position", cmd_reset_valve_position),
    ("pon", "poweron", "Power ON valve (enable stepper)", cmd_power_on_valve),
    ("poff", "poweroff", "Power OFF valve (disable stepper)", cmd_power_off_valve),
    (
        "cal",
        "calibrate",
        "Calibrate valve  (IDLE → CALIBRATE_VALVE)",
        cmd_calibrate_valve,
    ),
    (
        "lvseq",
        "loadvalve",
        "Load valve sequence  (IDLE → VALVE_PRIMED)",
        cmd_load_valve_sequence,
    ),
    (
        "svseq",
        "startvalve",
        "Start valve sequence (VALVE_PRIMED → VALVE_SEQ)",
        cmd_start_valve_sequence,
    ),
    (
        "ltseq",
        "loadthrust",
        "Load thrust sequence (IDLE → THRUST_PRIMED)",
        cmd_load_thrust_sequence,
    ),
    (
        "stseq",
        "startthrust",
        "Start thrust sequence (THRUST_PRIMED → THRUST_SEQ)",
        cmd_start_thrust_sequence,
    ),
    ("unprime", "unprime", "Unprime  (VALVE/THRUST_PRIMED → IDLE)", cmd_unprime),
    ("abort", "abort", "ABORT active sequence → safe state", cmd_abort),
    ("halt", "halt", "HALT all actuators", cmd_halt),
    ("status", "status", "Live telemetry dashboard (Enter to exit)", None),
    ("quit", "quit", "Exit", None),
]


def print_menu():
    t = THEME
    console.print()
    console.rule(
        f"[{t['header']}] {t['icon_fire']} CLOVER CLI {t['icon_fire']} [/{t['header']}]"
    )
    console.print()

    table = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t["primary"],
        border_style=t["panel_border"],
        padding=(0, 2),
    )
    table.add_column("CMD", style="bold white", no_wrap=True)
    table.add_column("Description", style="white")

    icons = {
        "sub": THEME["icon_live"],
        "id": THEME["icon_id"],
        "check": "🔍",
        "bias": "⚙️ ",
        "reset": THEME["icon_valve"],
        "pon": "🟢",
        "poff": "⚫",
        "cal": "📐",
        "lvseq": THEME["icon_seq"],
        "svseq": THEME["icon_fire"],
        "ltseq": THEME["icon_loop"],
        "stseq": THEME["icon_fire"],
        "unprime": "↩️ ",
        "abort": THEME["icon_stop"],
        "halt": THEME["icon_stop"],
        "status": THEME["icon_live"],
        "quit": THEME["icon_quit"],
    }

    for key, alias, desc, _ in MENU_ITEMS:
        table.add_row(f"{icons.get(key, '')} {key}", desc)

    console.print(table)


def route_command(cmd: str) -> bool:
    """Route a typed command. Returns False if user wants to quit."""
    cmd = cmd.strip().lower()

    if cmd in ("quit", "exit", "q"):
        console.print(
            f"\n  {THEME['icon_stop']} [{THEME['warning']}]Exiting.[/{THEME['warning']}]\n"
        )
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

    console.print(
        f"  [{THEME['warning']}]Unknown command '{cmd}'. Type 'help' to see all commands.[/{THEME['warning']}]"
    )
    return True


# ── Entry point ───────────────────────────────────────────────────────────────


def main():
    global data_sock
    t = THEME

    parser = argparse.ArgumentParser(description="Clover ground station CLI")
    parser.add_argument(
        "--no-data",
        action="store_true",
        help="Skip binding the UDP telemetry port (for second instances)",
    )
    args = parser.parse_args()

    telemetry_info = (
        f"Listen port: {DATA_PORT}" if not args.no_data else "Telemetry: disabled"
    )
    console.print(
        Panel.fit(
            f"[{t['header']}] {t['icon_fire']}  CLOVER CLI INFO  {t['icon_fire']} [/{t['header']}]\n"
            f"[{t['muted']}]Target: {ZEPHYR_IP}:{ZEPHYR_PORT}  |  {telemetry_info}[/{t['muted']}]",
            border_style=t["panel_border"],
            padding=(1, 4),
        )
    )

    if not args.no_data:
        data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        data_sock.bind((DATA_IP, DATA_PORT))
        data_sock.settimeout(1.0)
        console.print(
            f"\n  [{t['info']}]Auto-subscribing to data stream...[/{t['info']}]"
        )
        cmd_subscribe_data_stream()
        threading.Thread(target=listen_for_telemetry, daemon=True).start()
        threading.Thread(target=_flush_loop, daemon=True).start()

    print_menu()

    session = PromptSession(bottom_toolbar=get_toolbar, refresh_interval=0.1)

    while True:
        try:
            cmd = session.prompt("\n  CMD> ")
        except (KeyboardInterrupt, EOFError):
            console.print(
                f"\n  {t['icon_stop']} [{t['warning']}]Exiting.[/{t['warning']}]\n"
            )
            break

        if not route_command(cmd):
            break

    # Write CSV on exit — filename uses timestamp at exit: raw_sensors_YYYYMMDD_HHMMSS.csv
    if not args.no_data:
        _write_csv_on_exit()


if __name__ == "__main__":
    main()
