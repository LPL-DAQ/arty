# /// script
# requires-python = ">=3.8"
# dependencies = [
#     "protobuf",
#     "rich",
#     "prompt-toolkit",
#     "plotext",
#     "plotly",
# ]
# ///

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
import webbrowser
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
from collections import deque
import plotext as plt


THEME = {
    'primary': 'bold gold1',
    'success': 'bold green',
    'warning': 'bold yellow',
    'danger': 'bold red',
    'info': 'bold gold1',
    'muted': 'dim white',
    'header': 'bold white on dark_red',
    'panel_border': 'dark_red',
    'icon_fire': '🔥',
    'icon_ok': '✅',
    'icon_warn': '⚠️ ',
    'icon_stop': '🛑',
    'icon_live': '📡',
    'icon_fuel': '🟡',
    'icon_lox': '🔴',
    'icon_valve': '🔧',
    'icon_seq': '▶️ ',
    'icon_loop': '🔄',
    'icon_id': '🪪',
    'icon_quit': '❌',
}

VALVE_SEQ_DIR = pathlib.Path('sequences/valve')
THRUST_SEQ_DIR = pathlib.Path('sequences/thrust')
TVC_SEQ_DIR = pathlib.Path('sequences/tvc')
RCS_VALVE_SEQ_DIR = pathlib.Path('sequences/rcs_valve')
RCS_SEQ_DIR = pathlib.Path('sequences/rcs')
STATIC_FIRE_SEQ_DIR = pathlib.Path('sequences/static_fire')
FLIGHT_SEQ_DIR = pathlib.Path('sequences/flight')

# Network
# ZEPHYR_IP = '169.254.99.99'  # real board
ZEPHYR_IP = '192.168.0.150'  # daq box router
# ZEPHYR_IP = '127.0.0.1'  # fake_telemetry.py
ZEPHYR_PORT = 19690
DATA_IP = '0.0.0.0'  # Listen to UDP from anybody
DATA_PORT = 19691

# CSV columns mirror the unpivoted ClickHouse raw_sensors schema exactly:
#   time   — nanosecond-epoch Int64  (ClickHouse 'time')
#   sensor — field name string       (ClickHouse 'sensor')
#   value  — Float64 reading         (ClickHouse 'value')
#   event  — state label for gnc_state rows, else ''  (ClickHouse 'event')
#   system — always 'atlas'          (ClickHouse 'system')
#   source — always 'gnc'            (ClickHouse 'source')
# PROTO CHANGE: time is now time_ns (nanoseconds), updated comment above.
CSV_COLUMNS = ['time', 'sensor', 'value', 'event', 'system', 'source']

# CSV filename is generated at exit time so it reflects when the session ended.
# Format: raw_sensors_YYYYMMDD_HHMMSS.csv


console = Console()
latest_packet: clover_pb2.DataPacket | None = None
packet_lock = threading.Lock()

data_sock = None

_csv_store: list = []  # list of (recv_time: float, pkt: DataPacket); drained each second
_csv_store_lock = threading.Lock()
_csv_path: pathlib.Path | None = None  # set on first write
_csv_fh = None  # open file handle
_csv_writer = None  # csv.DictWriter bound to _csv_fh
_csv_rows_written: int = 0
_seq_recording: bool = False

_last_packet_time: float = 0.0
_last_packet_lock = threading.Lock()

_GRAPH_MAXLEN = 300  # ~60 s at 5 Hz
_graph_history: deque = deque(maxlen=_GRAPH_MAXLEN)
_graph_lock = threading.Lock()


def _make_tcp_socket() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)  # start probes after 5s idle
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)  # probe every 2s
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)  # drop after 3 missed probes
    s.connect((ZEPHYR_IP, ZEPHYR_PORT))
    return s


sock = _make_tcp_socket()
# ── Telemetry listener ───────────────────────────────────────────────────────


def listen_for_telemetry():
    """Background thread — receives UDP DataPackets and stores latest."""
    global latest_packet, _last_packet_time, _seq_recording
    while True:
        try:
            data, _ = data_sock.recvfrom(4096)
            packet = clover_pb2.DataPacket()
            packet.ParseFromString(data)
            recv_time = time.time()
            for lock, name in (
                (packet_lock, 'packet_lock'),
                (_last_packet_lock, '_last_packet_lock'),
                (_csv_store_lock, '_csv_store_lock'),
                (_graph_lock, '_graph_lock'),
            ):
                t0 = time.monotonic()
                lock.acquire()
                wait = time.monotonic() - t0
                if wait > 0.05:
                    console.print(
                        f'  [bold yellow]LOCK SLOW: {name} waited {wait:.3f}s[/bold yellow]'
                    )
                try:
                    if lock is packet_lock:
                        latest_packet = packet
                    elif lock is _last_packet_lock:
                        _last_packet_time = recv_time
                    elif lock is _csv_store_lock:
                        if clover_pb2.SystemState.Name(packet.state) != 'STATE_IDLE' and 'PRIMED' not in clover_pb2.SystemState.Name(packet.state):
                            _seq_recording = True
                        if _seq_recording:
                            _csv_store.append((recv_time, packet))
                    else:
                        _graph_history.append(packet)
                finally:
                    lock.release()
        except socket.timeout:
            continue  # loop back and re-read data_sock global
        except Exception as e:
            console.print(
                f'  [bold red]listen_for_telemetry error:[/bold red] {type(e).__name__}: {e}'
            )
            continue


# update SystemState with (18); range 19
_STATE_NAMES = {float(i): clover_pb2.SystemState.Name(i) for i in range(19)}


# Shared protobuf descriptor helpers.
def _has_msg_field(msg, field: str) -> bool:
    return field in msg.DESCRIPTOR.fields_by_name


def _fd_is_required(fd) -> bool:
    marker = getattr(fd, 'is_required', None)
    if callable(marker):
        return marker()
    if marker is not None:
        return bool(marker)
    return fd.label == fd.LABEL_REQUIRED


def _has(msg, field: str) -> bool:
    if not _has_msg_field(msg, field):
        return False
    try:
        return msg.HasField(field)
    except ValueError:
        return True


def _opt(msg, field: str):
    """Return float(msg.field) if present, else None."""
    return float(getattr(msg, field)) if _has(msg, field) else None


def _recv_time_ns(recv_time: float) -> int:
    """Return the host receive timestamp in epoch nanoseconds."""
    return int(recv_time * 1_000_000_000)


def _packet_to_row(recv_time: float, pkt: clover_pb2.DataPacket) -> dict:
    """
    Flatten one DataPacket into a wide dict of numeric sensors for ClickHouse.
    only emits values for fields present.
    """
    row = {
        # 'time': _recv_time_ns(recv_time),
        'time': pkt.time_ns,
        #'state': float(pkt.state),
        #'data_queue_size': float(pkt.data_queue_size),
        'sequence_number': float(pkt.sequence_number),
        #'controller_tick_ns': float(pkt.controller_timing.controller_tick_time_ns),
        #'analog_sense_ns': _opt(pkt.controller_timing, 'analog_sensors_sense_time_ns'),
        #'state_estimator_ns': _opt(pkt.controller_timing, 'state_estimator_update_time_ns'),
        #'lidar_sense_ns': _opt(pkt.controller_timing, 'lidar_sense_time_ns'),
        #'imu_sense_ns': _opt(pkt.controller_timing, 'imu_sense_time_ns'),
        #'gnc_connected': float(pkt.gnc_connected),
        #'gnc_last_pinged_ns': float(pkt.gnc_last_pinged_ns),
        #'daq_connected': float(pkt.daq_connected),
        #'daq_last_pinged_ns': float(pkt.daq_last_pinged_ns),
    }

    # Common analog/estimate data.
    row.update(
        {
            #'battery_voltage': _opt(pkt.analog_sensors, 'battery_voltage'),
            #'pt001': _opt(pkt.analog_sensors, 'pt001'),
            #'pt002': _opt(pkt.analog_sensors, 'pt002'),
            #'pt003': _opt(pkt.analog_sensors, 'pt003'),
            'pt004': _opt(pkt.analog_sensors, 'pt004'),
            'pt005': _opt(pkt.analog_sensors, 'pt005'),
            'pt006': _opt(pkt.analog_sensors, 'pt006'),
            'pt103': _opt(pkt.analog_sensors, 'pt103'),
            'pt203': _opt(pkt.analog_sensors, 'pt203'),
            #'pt301': _opt(pkt.analog_sensors, 'pt301'),
            'ptf401': _opt(pkt.analog_sensors, 'ptf401'),
            'pto401': _opt(pkt.analog_sensors, 'pto401'),
            'ptc401': _opt(pkt.analog_sensors, 'ptc401'),
            'ptc402': _opt(pkt.analog_sensors, 'ptc402'),
            #'tc002': _opt(pkt.analog_sensors, 'tc002'),
            #'tc102': _opt(pkt.analog_sensors, 'tc102'),
            #'tc102_5': _opt(pkt.analog_sensors, 'tc102_5'),
            #'tcf401': _opt(pkt.analog_sensors, 'tcf401'),
            #'tco401': _opt(pkt.analog_sensors, 'tco401'),
            #'ptg001': _opt(pkt.analog_sensors, 'ptg001'),
            #'ptg002': _opt(pkt.analog_sensors, 'ptg002'),
            #'ptg101': _opt(pkt.analog_sensors, 'ptg101'),
            #'est_pos_x_m': float(pkt.estimated_state.position.x),
            #'est_pos_y_m': float(pkt.estimated_state.position.y),
            #'est_pos_z_m': float(pkt.estimated_state.position.z),
            #'est_vel_x_m_s': float(pkt.estimated_state.velocity.x),
            #'est_vel_y_m_s': float(pkt.estimated_state.velocity.y),
            #'est_vel_z_m_s': float(pkt.estimated_state.velocity.z),
            #'est_yaw_deg': float(pkt.estimated_state.euler.x),
            #'est_pitch_deg': float(pkt.estimated_state.euler.y),
            #'est_roll_deg': float(pkt.estimated_state.euler.z),
            #'trace_time_msec': _opt(pkt, 'trace_time_msec'),
            #'throttle_thrust_command_lbf': _opt(pkt, 'throttle_thrust_command_lbf'),
            #'tvc_pitch_command_deg': _opt(pkt, 'tvc_pitch_command_deg'),
            #'tvc_yaw_command_deg': _opt(pkt, 'tvc_yaw_command_deg'),
            #'rcs_roll_command_deg': _opt(pkt, 'rcs_roll_command_deg'),
            #'flight_x_command_m': _opt(pkt, 'flight_x_command_m'),
            #'flight_y_command_m': _opt(pkt, 'flight_y_command_m'),
            #'flight_z_command_m': _opt(pkt, 'flight_z_command_m'),
            #'flight_pitch_accel_rad_s2': _opt(pkt, 'flight_pitch_accel_rad_s2'),
            #'flight_yaw_accel_rad_s2': _opt(pkt, 'flight_yaw_accel_rad_s2'),
            #'flight_z_accel_m_s2': _opt(pkt, 'flight_z_accel_m_s2'),
            #'main_propeller_command_us': _opt(pkt, 'main_propeller_command'),
            #'pitch_servo_command_us': _opt(pkt, 'pitch_servo_command'),
            #'yaw_servo_command_us': _opt(pkt, 'yaw_servo_command'),
            #'rcs_propeller_cw_command_us': _opt(pkt, 'rcs_propeller_cw_command'),
            #'rcs_propeller_ccw_command_us': _opt(pkt, 'rcs_propeller_ccw_command'),
        }
    )

    # # Optional GNSS data.
    # if _has(pkt, 'gnss'):
    #     row.update(
    #         {
    #             'gnss_north_m': float(pkt.gnss.north_m),
    #             'gnss_east_m': float(pkt.gnss.east_m),
    #             'gnss_up_m': float(pkt.gnss.up_m),
    #             'gnss_vx_ms': float(pkt.gnss.vx_ms),
    #             'gnss_vy_ms': float(pkt.gnss.vy_ms),
    #             'gnss_vz_ms': float(pkt.gnss.vz_ms),
    #             'gnss_sol_type': float(pkt.gnss.sol_type),
    #         }
    #     )

    # # Optional lidar data.
    # if _has(pkt, 'lidar_1'):
    #     row['lidar_1_distance_m'] = float(pkt.lidar_1.distance_m)
    # if _has(pkt, 'lidar_2'):
    #     row['lidar_2_distance_m'] = float(pkt.lidar_2.distance_m)

    # # Optional Hornet metrics.
    # if _has(pkt, 'hornet_throttle_metrics'):
    #     row['hornet_thrust_N'] = _opt(pkt.hornet_throttle_metrics, 'thrust_N')

    # if _has(pkt, 'flight_controller_metrics'):
    #     fcm = pkt.flight_controller_metrics
    #     row.update(
    #         {
    #             'fcm_des_world_tilt_x_rad': float(fcm.desired_world_tilt_x_rad),
    #             'fcm_des_world_tilt_y_rad': float(fcm.desired_world_tilt_y_rad),
    #             'fcm_act_world_tilt_x_rad': float(fcm.actual_world_tilt_x_rad),
    #             'fcm_act_world_tilt_y_rad': float(fcm.actual_world_tilt_y_rad),
    #             'fcm_des_vz_m_s': float(fcm.desired_vertical_velocity_m_s),
    #             'fcm_cmd_vacc_m_s2': float(fcm.commanded_vertical_acceleration_m_s2),
    #             'fcm_cmd_pitch_accel_rad_s2': float(fcm.commanded_pitch_acceleration_rad_s2),
    #             'fcm_cmd_yaw_accel_rad_s2': float(fcm.commanded_yaw_acceleration_rad_s2),
    #         }
    #     )

    # Throttle valve commands and motor feedback.
    if _has(pkt, 'fuel_valve_command'):
        row['gnc_fuel_enable'] = _opt(pkt.fuel_valve_command, 'enable')
        row['gnc_fuel_set_pos'] = _opt(pkt.fuel_valve_command, 'set_pos')
        row['gnc_fuel_target_deg'] = _opt(pkt.fuel_valve_command, 'target_deg')
    if _has(pkt, 'lox_valve_command'):
        row['gnc_lox_enable'] = _opt(pkt.lox_valve_command, 'enable')
        row['gnc_lox_set_pos'] = _opt(pkt.lox_valve_command, 'set_pos')
        row['gnc_lox_target_deg'] = _opt(pkt.lox_valve_command, 'target_deg')

    # Motor position feedback (ValveStatus: encoder position, on/off).
    if _has(pkt, 'fuel_valve_status'):
        row.update(
            {
                'fuel_encoder_pos_deg': float(pkt.fuel_valve_status.encoder_pos_deg),
                'fuel_is_on': float(pkt.fuel_valve_status.is_on),
            }
        )
    if _has(pkt, 'lox_valve_status'):
        row.update(
            {
                'lox_encoder_pos_deg': float(pkt.lox_valve_status.encoder_pos_deg),
                'lox_is_on': float(pkt.lox_valve_status.is_on),
            }
        )

    # if _has(pkt, 'ranger_throttle_metrics'):
    #     rtm = pkt.ranger_throttle_metrics
    #     row.update(
    #         {
    #             'predicted_thrust': _opt(rtm, 'predicted_thrust_lbf'),
    #             'predicted_of': _opt(rtm, 'predicted_of'),
    #             'mdot_fuel': _opt(rtm, 'mdot_fuel'),
    #             'mdot_lox': _opt(rtm, 'mdot_lox'),
    #             'change_alpha_cmd': _opt(rtm, 'change_alpha_cmd'),
    #             'clamped_change_alpha_cmd': _opt(rtm, 'clamped_change_alpha_cmd'),
    #             'alpha': _opt(rtm, 'alpha'),
    #             'thrust_from_alpha': _opt(rtm, 'thrust_from_alpha_lbf'),
    #         }
    #     )
    if _has(pkt, 'ranger_throttle_metrics'):
        rtm = pkt.ranger_throttle_metrics
        row['throttle_thrust_command_lbf'] = _opt(pkt, 'throttle_thrust_command_lbf')
        row['predicted_thrust'] = _opt(rtm, 'predicted_thrust_lbf')
        row['thrust_from_alpha'] = _opt(rtm, 'thrust_from_alpha_lbf')
        row['predicted_of'] = _opt(rtm, 'predicted_of')
        row['mdot_fuel'] = _opt(rtm, 'mdot_fuel')
        row['mdot_lox'] = _opt(rtm, 'mdot_lox')
        row['change_alpha_cmd'] = _opt(rtm, 'change_alpha_cmd')
        row['clamped_change_alpha_cmd'] = _opt(rtm, 'clamped_change_alpha_cmd')
        row['alpha'] = _opt(rtm, 'alpha')

    return row


def _packet_to_csv_rows(recv_time: float, pkt: clover_pb2.DataPacket) -> list[dict]:
    """
    Expand one DataPacket into multiple CSV rows (one per sensor/field),
    matching the unpivoted schema written to ClickHouse raw_sensors.
    """
    wide = _packet_to_row(recv_time, pkt)
    ts = wide['time']
    rows = []

    for sensor, raw_value in wide.items():
        if sensor == 'time' or raw_value is None:
            continue

        value = float(raw_value)
        # event column: state label for gnc_state rows, empty string otherwise
        event = _STATE_NAMES.get(raw_value, '') if sensor == 'gnc_state' else ''

        rows.append(
            {
                'time': ts,
                'sensor': sensor,
                'value': value,
                'event': event,
                'system': 'atlas',  # change if not atlas
                'source': 'gnc',  # leave as all GNC for now
            }
        )

    return rows


def _write_csv_on_exit():
    """Flush any remaining buffered packets to the CSV file and close it."""
    global _csv_fh, _csv_writer, _csv_path, _csv_rows_written

    with _csv_store_lock:
        remaining = list(_csv_store)
        _csv_store.clear()

    # Sequence started but flush_loop never ran — create the file now
    if remaining and _csv_fh is None:
        _csv_path = pathlib.Path('data') / f'raw_sensors_{time.strftime("%Y%m%d_%H%M%S")}.csv'
        _csv_path.parent.mkdir(exist_ok=True)
        _csv_fh = open(_csv_path, 'w', newline='')
        _csv_writer = csv.DictWriter(_csv_fh, fieldnames=CSV_COLUMNS)
        _csv_writer.writeheader()

    if _csv_fh is None:
        return

    for recv_time, pkt in remaining:
        try:
            for row in _packet_to_csv_rows(recv_time, pkt):
                _csv_writer.writerow(row)
                _csv_rows_written += 1
        except Exception as e:
            console.print(f'  [bold red]CSV row error:[/bold red] {e}')

    _close_csv()


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
            f'  [{THEME["warning"]}]Reconnected and re-subscribed to data stream.[/{THEME["warning"]}]'
        )
    except Exception as e:
        console.print(f'  [{THEME["danger"]}]Reconnect failed: {e}[/{THEME["danger"]}]')


_USEFUL_SENSORS: dict[str, str] = {
    'pt004': 'PT-004',
    'pt005': 'PT-005',
    'pt006': 'PT-006',
    'pt103': 'PT-103',
    'pt203': 'PT-203',
    'ptf401': 'PTF-401',
    'pto401': 'PTO-401',
    'ptc401': 'PTC-401',
    'ptc402': 'PTC-402',
}

_MOTOR_SENSORS: dict[str, str] = {
    'gnc_fuel_target_deg': 'Fuel Desired',
    'fuel_encoder_pos_deg': 'Fuel Actual',
    'gnc_lox_target_deg': 'LOX Desired',
    'lox_encoder_pos_deg': 'LOX Actual',
}


_THROTTLE_METRIC_PLOTS: list[tuple[str, list[tuple[str, str]]]] = [
    ('Thrust (lbf)',         [('throttle_thrust_command_lbf', 'Desired thrust (lbf)'), ('predicted_thrust', 'Predicted Thrust'), ('thrust_from_alpha', 'Thrust from Alpha')]),
    ('Change Alpha Command',        [('change_alpha_cmd', 'Change Alpha'), ('clamped_change_alpha_cmd', 'Clamped Change Alpha')]),
    ('Alpha',                [('alpha', 'Alpha')]),
    ('Mass Flow (kg/s)',     [('mdot_fuel', 'M dot fuel'), ('mdot_lox', 'M dot lox')]),
    ('Predicted O/F',       [('predicted_of', 'Predicted OF')]),
]


def _parse_plot_time(raw_time: str):
    try:
        return int(raw_time)
    except ValueError:
        return float(raw_time)


def _read_plot_series(csv_path: pathlib.Path) -> tuple[dict[str, list[tuple[float, float]]], list]:
    series = {}
    times = []

    with csv_path.open(newline='') as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            try:
                timestamp = _parse_plot_time(row['time'])
                value = float(row['value'])
            except (KeyError, TypeError, ValueError):
                continue

            sensor = row.get('sensor', '')
            if not sensor:
                continue

            times.append(timestamp)
            series.setdefault(sensor, []).append((timestamp, value))

    return series, times


def _time_units_per_second(time_values: list) -> float:
    """Support old seconds CSVs, epoch ns CSVs, and real-board boot ns CSVs."""
    if not time_values:
        return 1.0

    time_min = min(time_values)
    time_max = max(time_values)
    max_abs_time = max(abs(t) for t in time_values)
    span = time_max - time_min
    if max_abs_time >= 1_000_000_000_000 or span >= 1_000_000:
        return 1_000_000_000.0
    return 1.0


def _add_traces(fig, series, sensor_label_pairs, t0, tups, row, go):
    for sensor, label in sensor_label_pairs:
        points = series.get(sensor, [])
        if not points:
            continue
        points.sort(key=lambda p: p[0])
        fig.add_trace(
            go.Scatter(
                x=[(ts - t0) / tups for ts, _ in points],
                y=[v for _, v in points],
                mode='lines',
                name=label,
            ),
            row=row,
            col=1,
        )


def _generate_plots(csv_path: pathlib.Path) -> None:
    try:
        from plotly.subplots import make_subplots
        import plotly.graph_objects as go

        series, times = _read_plot_series(csv_path)
        if not series:
            return

        t0 = min(times)
        tups = _time_units_per_second(times)

        has_throttle_metrics = any(
            series.get(sensor) for sensor, _ in _THROTTLE_METRIC_PLOTS[0][1]
        )

        if has_throttle_metrics:
            n_rows = 2 + len(_THROTTLE_METRIC_PLOTS)
            subplot_titles = (
                'All Sensors',
                'Motor Traces — Desired vs Actual',
                *(title for title, _ in _THROTTLE_METRIC_PLOTS),
            )
        else:
            n_rows = 2
            subplot_titles = ('All Sensors', 'Motor Traces — Desired vs Actual')

        fig = make_subplots(
            rows=n_rows,
            cols=1,
            subplot_titles=subplot_titles,
            shared_xaxes=True,
            vertical_spacing=0.06,
        )

        _add_traces(fig, series, _USEFUL_SENSORS.items(), t0, tups, row=1, go=go)
        _add_traces(fig, series, _MOTOR_SENSORS.items(), t0, tups, row=2, go=go)

        if has_throttle_metrics:
            for i, (_, sensor_label_pairs) in enumerate(_THROTTLE_METRIC_PLOTS):
                _add_traces(fig, series, sensor_label_pairs, t0, tups, row=3 + i, go=go)

        fig.update_xaxes(title_text='Time (s)', row=n_rows, col=1)
        fig.update_yaxes(title_text='Value', row=1, col=1)
        fig.update_yaxes(title_text='Position (°)', row=2, col=1)
        fig.update_layout(hovermode='x unified', height=400 * n_rows)

        plots_dir = csv_path.parent / 'plots'
        plots_dir.mkdir(exist_ok=True)
        stem = csv_path.stem
        out_html = plots_dir / (stem + '.html')
        fig.write_html(str(out_html))
        fig.show(renderer='browser')
        console.print(
            f'  {THEME["icon_ok"]} [bold green]Plot saved →[/bold green]\n'
            f'  [dim]{out_html.resolve()}[/dim]'
        )
    except Exception as e:
        console.print(f'  [bold red]Plot generation failed:[/bold red] {e}')


def _close_csv():
    global _csv_fh, _csv_path, _csv_writer, _csv_rows_written
    if _csv_fh is None:
        return
    _csv_fh.close()
    _csv_fh = None
    path = _csv_path
    console.print(
        f'\n  {THEME["icon_ok"]} [bold green]CSV saved →[/bold green] '
        f'[dim]{path.resolve()}[/dim]\n'
        f'  [{THEME["muted"]}]{_csv_rows_written:,} rows written[/{THEME["muted"]}]'
    )
    _csv_path = None
    _csv_writer = None
    _csv_rows_written = 0
    _generate_plots(path)


def _flush_loop():
    global _seq_recording
    while True:
        time.sleep(1)

        # ── CSV incremental flush ────────────────────────────────────────────
        with _csv_store_lock:
            if _csv_store:
                csv_batch = list(_csv_store)
                _csv_store.clear()
            else:
                csv_batch = []

        if csv_batch:
            global _csv_path, _csv_fh, _csv_writer, _csv_rows_written
            if _csv_fh is None and _seq_recording:
                _csv_path = (
                    pathlib.Path('data') / f'raw_sensors_{time.strftime("%Y%m%d_%H%M%S")}.csv'
                )
                _csv_path.parent.mkdir(exist_ok=True)
                _csv_fh = open(_csv_path, 'w', newline='')
                _csv_writer = csv.DictWriter(_csv_fh, fieldnames=CSV_COLUMNS)
                _csv_writer.writeheader()
            if _csv_fh is not None:
                try:
                    for recv_time, pkt in csv_batch:
                        for row in _packet_to_csv_rows(recv_time, pkt):
                            _csv_writer.writerow(row)
                            _csv_rows_written += 1
                    _csv_fh.flush()
                except Exception as e:
                    console.print(f'  [bold red]CSV write error:[/bold red] {e}')

        # ── Detect sequence end (after flush so last data lands in current file)
        pkt = latest_packet
        if _seq_recording and pkt is not None:
            if clover_pb2.SystemState.Name(pkt.state) == 'STATE_IDLE':
                _seq_recording = False
                _close_csv()


# ── Live status display ──────────────────────────────────────────────────────


def _plotext_panel(title: str) -> Panel:
    """Wrap the current plotext figure in a rich Panel."""
    t = THEME
    return Panel(
        Text.from_ansi(plt.build()),
        title=f'[{t["primary"]}]{title}[/{t["primary"]}]',
        border_style=t['panel_border'],
        padding=(0, 0),
    )


def _waiting_panel(title: str) -> Panel:
    t = THEME
    return Panel(
        f'[{t["muted"]}]Waiting for data...[/{t["muted"]}]',
        title=f'[{t["primary"]}]{title}[/{t["primary"]}]',
        border_style=t['panel_border'],
    )


def _half_width() -> int:
    return (console.width // 2) - 6


def _build_fuel_valve_graph() -> Panel:
    """Fuel valve driver and encoder positions over time."""
    with _graph_lock:
        history = list(_graph_history)
    if len(history) < 2:
        return _waiting_panel('🟡 Fuel Valve')

    t0 = history[0].time_ns / 1e9
    times = [p.time_ns / 1e9 - t0 for p in history]

    plt.clf()
    plt.plotsize(_half_width(), 15)
    plt.theme('dark')
    plt.plot(
        times,
        [p.fuel_valve_command.target_deg for p in history],
        label='Target',
        color=(50, 100, 220),
        marker='braille',
    )
    plt.plot(
        times,
        [p.fuel_valve_status.encoder_pos_deg for p in history],
        label='Encoder',
        color=(255, 220, 0),
        marker='braille',
    )
    plt.ylim(-5, 95)
    plt.xlabel('t (s)')
    plt.ylabel('deg')
    return _plotext_panel('🟡 Fuel Valve')


def _build_lox_valve_graph() -> Panel:
    """LOX valve driver and encoder positions over time."""
    with _graph_lock:
        history = list(_graph_history)
    if len(history) < 2:
        return _waiting_panel('🔴 LOX Valve')

    t0 = history[0].time_ns / 1e9
    times = [p.time_ns / 1e9 - t0 for p in history]

    plt.clf()
    plt.plotsize(_half_width(), 15)
    plt.theme('dark')
    plt.plot(
        times,
        [p.lox_valve_command.target_deg for p in history],
        label='Target',
        color=(0, 200, 220),
        marker='braille',
    )
    plt.plot(
        times,
        [p.lox_valve_status.encoder_pos_deg for p in history],
        label='Encoder',
        color=(220, 50, 50),
        marker='braille',
    )
    plt.ylim(-5, 95)
    plt.xlabel('t (s)')
    plt.ylabel('deg')
    return _plotext_panel('🔴 LOX Valve')


def _build_fuel_graph() -> Panel:
    """Fuel-side pressure sensors (200-series + PTC) over time."""
    with _graph_lock:
        history = list(_graph_history)
    if len(history) < 2:
        return _waiting_panel('🟡 Fuel Sensors')

    t0 = history[0].time_ns / 1e9
    times = [p.time_ns / 1e9 - t0 for p in history]

    plt.clf()
    plt.plotsize(_half_width(), 15)
    plt.theme('dark')
    plt.plot(
        times,
        [p.analog_sensors.pt006 for p in history],
        label='PT-006',
        color=(255, 220, 0),
        marker='braille',
    )
    plt.plot(
        times,
        [p.analog_sensors.pt203 for p in history],
        label='PT-203',
        color=(50, 200, 50),
        marker='braille',
    )
    plt.plot(
        times,
        [p.analog_sensors.ptf401 for p in history],
        label='PTF-401',
        color=(255, 140, 0),
        marker='braille',
    )
    plt.plot(
        times,
        [p.analog_sensors.ptc401 for p in history],
        label='PTC-401',
        color=(0, 200, 220),
        marker='braille',
    )
    plt.plot(
        times,
        [p.analog_sensors.ptc402 for p in history],
        label='PTC-402',
        color=(200, 50, 200),
        marker='braille',
    )
    fuel_vals = (
        [p.analog_sensors.pt006 for p in history]
        + [p.analog_sensors.pt203 for p in history]
        + [p.analog_sensors.ptf401 for p in history]
        + [p.analog_sensors.ptc401 for p in history]
        + [p.analog_sensors.ptc402 for p in history]
    )
    plt.ylim(30, max(fuel_vals) * 1.05 if max(fuel_vals) > 30 else 60)
    plt.xlabel('t (s)')
    plt.ylabel('psi')
    return _plotext_panel('🟡 Fuel Sensors (200s + PTC)')


def _build_lox_graph() -> Panel:
    """LOX-side pressure sensors (100-series + PTC) over time."""
    with _graph_lock:
        history = list(_graph_history)
    if len(history) < 2:
        return _waiting_panel('🔴 LOX Sensors')

    t0 = history[0].time_ns / 1e9
    times = [p.time_ns / 1e9 - t0 for p in history]

    plt.clf()
    plt.plotsize(_half_width(), 15)
    plt.theme('dark')
    plt.plot(
        times,
        [p.analog_sensors.pt006 for p in history],
        label='PT-006',
        color=(255, 220, 0),
        marker='braille',
    )
    plt.plot(
        times,
        [p.analog_sensors.pt103 for p in history],
        label='PT-103',
        color=(50, 200, 50),
        marker='braille',
    )
    plt.plot(
        times,
        [p.analog_sensors.pto401 for p in history],
        label='PTO-401',
        color=(255, 140, 0),
        marker='braille',
    )
    plt.plot(
        times,
        [p.analog_sensors.ptc401 for p in history],
        label='PTC-401',
        color=(0, 200, 220),
        marker='braille',
    )
    plt.plot(
        times,
        [p.analog_sensors.ptc402 for p in history],
        label='PTC-402',
        color=(200, 50, 200),
        marker='braille',
    )
    lox_vals = (
        [p.analog_sensors.pt006 for p in history]
        + [p.analog_sensors.pt103 for p in history]
        + [p.analog_sensors.pto401 for p in history]
        + [p.analog_sensors.ptc401 for p in history]
        + [p.analog_sensors.ptc402 for p in history]
    )
    plt.ylim(30, max(lox_vals) * 1.05 if max(lox_vals) > 30 else 60)
    plt.xlabel('t (s)')
    plt.ylabel('psi')
    return _plotext_panel('🔴 LOX Sensors (100s + PTC)')


STATE_COLORS = {
    'STATE_UNKNOWN': 'dim white',
    'STATE_IDLE': 'green',
    'STATE_CALIBRATE_THROTTLE_VALVE': 'magenta',
    'STATE_THROTTLE_VALVE_PRIMED': 'cyan',
    'STATE_THROTTLE_VALVE': 'yellow',
    'STATE_THROTTLE_PRIMED': 'cyan',
    'STATE_THROTTLE': 'yellow',
    'STATE_CALIBRATE_TVC': 'magenta',
    'STATE_TVC_PRIMED': 'cyan',
    'STATE_TVC': 'yellow',
    'STATE_RCS_VALVE_PRIMED': 'cyan',
    'STATE_RCS_VALVE': 'yellow',
    'STATE_RCS_PRIMED': 'cyan',
    'STATE_RCS': 'yellow',
    'STATE_STATIC_FIRE_PRIMED': 'cyan',
    'STATE_STATIC_FIRE': 'yellow',
    'STATE_FLIGHT_PRIMED': 'cyan',
    'STATE_FLIGHT': 'bold green',
    'STATE_ABORT': 'bold red',
}


def _build_status_renderable():
    """Build a rich renderable for the current telemetry snapshot."""
    with packet_lock:
        pkt = latest_packet

    t = THEME

    if pkt is None:
        return Panel(
            f'[{t["muted"]}]Waiting for telemetry...[/{t["muted"]}]',
            title=f'[{t["primary"]}]{t["icon_live"]} LIVE STATUS[/{t["primary"]}]',
            border_style=t['panel_border'],
        )

    state_name = clover_pb2.SystemState.Name(pkt.state)
    state_color = STATE_COLORS.get(state_name, 'white')
    is_abort = state_name == 'STATE_ABORT'
    abort_str = (
        f'[bold red]{t["icon_stop"]} ABORT[/bold red]' if is_abort else '[green]nominal[/green]'
    )

    hdr = Table.grid(padding=(0, 3))
    for _ in range(5):
        hdr.add_column()
    hdr.add_row(
        Text(f'{t["icon_live"]} LIVE', style='bold green'),
        Text(f'State: {state_name}', style=state_color),
        Text(f't = {pkt.time_ns / 1e9:.3f} s', style='white'),
        Text(f'seq #{pkt.sequence_number}', style=t['muted']),
        Text(f'queue: {pkt.data_queue_size}', style=t['muted']),
    )
    header_panel = Panel(hdr, border_style=t['panel_border'], padding=(0, 1), subtitle=abort_str)

    # Pose table
    pose = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t['primary'],
        border_style=t['panel_border'],
        padding=(0, 1),
    )
    pose.add_column('Vehicle', style='bold white', no_wrap=True)
    pose.add_column('x', style='white', justify='right', no_wrap=True)
    pose.add_column('y', style='white', justify='right', no_wrap=True)
    pose.add_column('z', style='white', justify='right', no_wrap=True)
    pose.add_column('Unit', style=t['muted'], no_wrap=True)

    pos = pkt.estimated_state.position
    vel = pkt.estimated_state.velocity
    euler = pkt.estimated_state.euler
    pose.add_row('Position', f'{pos.x:.2f}', f'{pos.y:.2f}', f'{pos.z:.2f}', 'm')
    pose.add_row('Velocity', f'{vel.x:.2f}', f'{vel.y:.2f}', f'{vel.z:.2f}', 'm/s')
    pose.add_row('Y/P/R', f'{euler.x:.1f}', f'{euler.y:.1f}', f'{euler.z:.1f}', 'deg')

    # Command table
    cmd = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t['primary'],
        border_style=t['panel_border'],
        padding=(0, 1),
    )
    cmd.add_column('Command', style='bold white', no_wrap=True)
    cmd.add_column('Value', style='white', justify='right', no_wrap=True)
    cmd.add_column('Unit', style=t['muted'], no_wrap=True)

    if _has(pkt, 'main_propeller_command'):
        pwm = max(1000, min(2000, pkt.main_propeller_command))
        throttle_pct = (pwm - 1000) / 10.0
        cmd.add_row('Main throttle', f'{throttle_pct:.2f}', '%')
        cmd.add_row('Main PWM', str(pkt.main_propeller_command), 'us')
    else:
        cmd.add_row('Main throttle', f'[{t["muted"]}]—[/{t["muted"]}]', '%')
        cmd.add_row('Main PWM', f'[{t["muted"]}]—[/{t["muted"]}]', 'us')

    for field, label in [
        ('pitch_servo_command', 'Pitch servo'),
        ('yaw_servo_command', 'Yaw servo'),
        ('rcs_propeller_cw_command', 'RCS CW PWM'),
        ('rcs_propeller_ccw_command', 'RCS CCW PWM'),
    ]:
        if _has(pkt, field):
            cmd.add_row(label, str(getattr(pkt, field)), 'us')
        else:
            cmd.add_row(label, f'[{t["muted"]}]—[/{t["muted"]}]', 'us')

    for field, label, unit in [
        ('trace_time_msec', 'Trace time', 'ms'),
        ('throttle_thrust_command_lbf', 'Thrust cmd', 'lbf'),
        ('tvc_pitch_command_deg', 'TVC pitch cmd', 'deg'),
        ('tvc_yaw_command_deg', 'TVC yaw cmd', 'deg'),
        ('rcs_roll_command_deg', 'RCS roll cmd', 'deg'),
    ]:
        if _has(pkt, field):
            cmd.add_row(label, f'{getattr(pkt, field):.2f}', unit)
        else:
            cmd.add_row(label, f'[{t["muted"]}]—[/{t["muted"]}]', unit)

    # Analog Table
    sensors = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t['primary'],
        border_style=t['panel_border'],
        padding=(0, 1),
    )
    sensors.add_column('Sensor', style='bold white', no_wrap=True)
    sensors.add_column('Value', style='white', justify='right', no_wrap=True)
    sensors.add_column('Unit', style=t['muted'], no_wrap=True)

    def _add_analog_row(label: str, field: str, unit: str):
        if _has(pkt.analog_sensors, field):
            sensors.add_row(label, f'{getattr(pkt.analog_sensors, field):.2f}', unit)
        elif _has_msg_field(pkt.analog_sensors, field):
            sensors.add_row(label, f'[{t["muted"]}]—[/{t["muted"]}]', unit)

    # Analog sensors (shown when available in schema/config).
    _add_analog_row('PT-001', 'pt001', 'psi')
    _add_analog_row('PT-002', 'pt002', 'psi')
    _add_analog_row('PT-003', 'pt003', 'psi')
    _add_analog_row('PT-004', 'pt004', 'psi')
    _add_analog_row('PT-005', 'pt005', 'psi')
    _add_analog_row('PT-005', 'pt006', 'psi')
    _add_analog_row('PT-103', 'pt103', 'psi')
    _add_analog_row('PT-203', 'pt203', 'psi')
    _add_analog_row('PTF-401', 'ptf401', 'psi')
    _add_analog_row('PT0-401', 'pto401', 'psi')
    _add_analog_row('PTC-401', 'ptc401', 'psi')
    _add_analog_row('PTC-402', 'ptc402', 'psi')
    _add_analog_row('TC-001', 'tc001', 'C')
    _add_analog_row('TC-101', 'tc101', 'C')
    _add_analog_row('TC-102', 'tc102', 'C')
    _add_analog_row('TC-F1', 'tcf1', 'C')
    _add_analog_row('TC-O1', 'tco1', 'C')
    _add_analog_row('PT-G001', 'ptg001', 'psi')
    _add_analog_row('PT-G002', 'ptg002', 'psi')
    _add_analog_row('PT-G101', 'ptg101', 'psi')
    _add_analog_row('Battery', 'battery_voltage', 'V')

    sensors.add_row(
        'Lidar 1',
        f'{pkt.lidar_1.distance_m:.2f}'
        if _has(pkt, 'lidar_1')
        else f'[{t["muted"]}]—[/{t["muted"]}]',
        'm',
    )
    sensors.add_row(
        'Lidar 2',
        f'{pkt.lidar_2.distance_m:.2f}'
        if _has(pkt, 'lidar_2')
        else f'[{t["muted"]}]—[/{t["muted"]}]',
        'm',
    )

    thrust_value = None
    if _has(pkt, 'hornet_throttle_metrics') and _has(pkt.hornet_throttle_metrics, 'thrust_N'):
        thrust_value = f'{pkt.hornet_throttle_metrics.thrust_N:.2f}'
    sensors.add_row(
        'Thrust',
        thrust_value if thrust_value is not None else f'[{t["muted"]}]—[/{t["muted"]}]',
        'N',
    )

    # Shared valve state telemetry.
    if _has(pkt, 'valve_states'):
        vs = pkt.valve_states

        def _valve_state_to_str(v: int) -> str:
            try:
                return clover_pb2.ValveState.Name(v)
            except ValueError:
                return str(v)

        for field, label in [
            ('sv001', 'SV001'),
            ('sv002', 'SV002'),
            ('sv003', 'SV003'),
            ('sv004', 'SV004'),
            ('pbv002', 'PBV002'),
            ('sv005', 'SV005'),
            ('svg001', 'SVG001'),
            ('svg002', 'SVG002'),
            ('svg003', 'SVG003'),
        ]:
            if _has(vs, field):
                sensors.add_row(label, _valve_state_to_str(getattr(vs, field)), '')

    # Controller table
    timing = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t['primary'],
        border_style=t['panel_border'],
        padding=(0, 1),
    )
    timing.add_column('Timing', style='bold white', no_wrap=True)
    timing.add_column('Value', style='white', justify='right', no_wrap=True)
    timing.add_column('Unit', style=t['muted'], no_wrap=True)
    timing.add_row(
        'Controller tick', f'{pkt.controller_timing.controller_tick_time_ns / 1000:.2f}', 'us'
    )
    if _has(pkt.controller_timing, 'analog_sensors_sense_time_ns'):
        timing.add_row(
            'Analog read', f'{pkt.controller_timing.analog_sensors_sense_time_ns / 1000:.2f}', 'us'
        )
    if _has(pkt.controller_timing, 'lidar_sense_time_ns'):
        timing.add_row(
            'Lidar read', f'{pkt.controller_timing.lidar_sense_time_ns / 1000:.2f}', 'us'
        )
    if _has(pkt.controller_timing, 'imu_sense_time_ns'):
        timing.add_row('IMU read', f'{pkt.controller_timing.imu_sense_time_ns / 1000:.2f}', 'us')
    if _has(pkt.controller_timing, 'state_estimator_update_time_ns'):
        timing.add_row(
            'State estimator',
            f'{pkt.controller_timing.state_estimator_update_time_ns / 1000:.2f}',
            'us',
        )
    timing.add_row('DAQ connected?', 'YES' if pkt.daq_connected else 'NO', '')

    top = Columns(
        [
            Panel(
                pose,
                title=f'[{t["primary"]}]Vehicle State[/{t["primary"]}]',
                border_style=t['panel_border'],
            ),
            Panel(
                cmd,
                title=f'[{t["primary"]}]Commands[/{t["primary"]}]',
                border_style=t['panel_border'],
            ),
        ]
    )
    bottom_columns = [
        Panel(
            sensors,
            title=f'[{t["primary"]}]Sensors[/{t["primary"]}]',
            border_style=t['panel_border'],
        ),
        Panel(
            timing,
            title=f'[{t["primary"]}]Timing[/{t["primary"]}]',
            border_style=t['panel_border'],
        ),
    ]

    if _has(pkt, 'fuel_valve_command'):
        table = Table(
            box=box.SIMPLE_HEAD,
            show_header=True,
            header_style=t['primary'],
            border_style=t['panel_border'],
            padding=(0, 1),
        )
        table.add_column('Reading', style='bold white', no_wrap=True)
        table.add_column('Value', style='white', justify='right', no_wrap=True)
        table.add_column('Unit', style=t['muted'], no_wrap=True)

        table.add_row('Commanded pos', f'{pkt.fuel_valve_command.target_deg:.3f}', '°')
        table.add_row('Encoder pos', f'{pkt.fuel_valve_status.encoder_pos_deg:.3f}', '°')
        table.add_row('Power', 'ON' if pkt.fuel_valve_status.is_on else 'OFF', '')
        table.add_row('Enable?', 'YES' if pkt.fuel_valve_command.enable else 'NO', '')
        table.add_row('Set pos?', 'YES' if pkt.fuel_valve_command.set_pos else 'NO', '')

        bottom_columns.append(
            Panel(
                table,
                title=f'[{t["primary"]}]Fuel Valve[/{t["primary"]}]',
                border_style=t['panel_border'],
            ),
        )

    if _has(pkt, 'lox_valve_command'):
        table = Table(
            box=box.SIMPLE_HEAD,
            show_header=True,
            header_style=t['primary'],
            border_style=t['panel_border'],
            padding=(0, 1),
        )
        table.add_column('Reading', style='bold white', no_wrap=True)
        table.add_column('Value', style='white', justify='right', no_wrap=True)
        table.add_column('Unit', style=t['muted'], no_wrap=True)

        table.add_row('Commanded pos', f'{pkt.lox_valve_command.target_deg:.3f}', '°')
        table.add_row('Encoder pos', f'{pkt.lox_valve_status.encoder_pos_deg:.3f}', '°')
        table.add_row('Power', 'ON' if pkt.lox_valve_status.is_on else 'OFF', '')
        table.add_row('Enable?', 'YES' if pkt.lox_valve_command.enable else 'NO', '')
        table.add_row('Set pos?', 'YES' if pkt.lox_valve_command.set_pos else 'NO', '')

        bottom_columns.append(
            Panel(
                table,
                title=f'[{t["primary"]}]LOx Valve[/{t["primary"]}]',
                border_style=t['panel_border'],
            ),
        )
    # controller metrics table
    fcm = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t['primary'],
        border_style=t['panel_border'],
        padding=(0, 1),
    )
    fcm.add_column('Flight Controller Metric', style='bold white', no_wrap=True)
    fcm.add_column('Value', style='white', justify='right', no_wrap=True)
    fcm.add_column('Unit', style=t['muted'], no_wrap=True)

    if _has(pkt, 'flight_controller_metrics'):
        m = pkt.flight_controller_metrics
        fcm.add_row('Desired tilt X', f'{m.desired_world_tilt_x_rad:.3f}', 'rad')
        fcm.add_row('Desired tilt Y', f'{m.desired_world_tilt_y_rad:.3f}', 'rad')
        fcm.add_row('Actual tilt X', f'{m.actual_world_tilt_x_rad:.3f}', 'rad')
        fcm.add_row('Actual tilt Y', f'{m.actual_world_tilt_y_rad:.3f}', 'rad')
        fcm.add_row('Desired vz', f'{m.desired_vertical_velocity_m_s:.3f}', 'm/s')
        fcm.add_row('Cmd vz accel', f'{m.commanded_vertical_acceleration_m_s2:.3f}', 'm/s^2')
        fcm.add_row('Cmd pitch accel', f'{m.commanded_pitch_acceleration_rad_s2:.3f}', 'rad/s^2')
        fcm.add_row('Cmd yaw accel', f'{m.commanded_yaw_acceleration_rad_s2:.3f}', 'rad/s^2')

    # Ranger throttle sequence metrics (equivalent replacement for old thrust_sequence_data panel values).
    if _has(pkt, 'ranger_throttle_metrics'):
        fcm.add_row('Desired thrust', f'{pkt.throttle_thrust_command_lbf:.3f}', 'lbf')
        rtm = pkt.ranger_throttle_metrics
        if _has(rtm, 'predicted_thrust_lbf'):
            fcm.add_row('Pred thrust', f'{rtm.predicted_thrust_lbf:.3f}', 'lbf')
        if _has(rtm, 'predicted_of'):
            fcm.add_row('Pred O/F', f'{rtm.predicted_of:.3f}', '')
        if _has(rtm, 'mdot_fuel'):
            fcm.add_row('m_dot fuel', f'{rtm.mdot_fuel:.3f}', '')
        if _has(rtm, 'mdot_lox'):
            fcm.add_row('m_dot lox', f'{rtm.mdot_lox:.3f}', '')
        if _has(rtm, 'change_alpha_cmd'):
            fcm.add_row('d_alpha cmd', f'{rtm.change_alpha_cmd:.3f}', '')
        if _has(rtm, 'clamped_change_alpha_cmd'):
            fcm.add_row('d_alpha clamped', f'{rtm.clamped_change_alpha_cmd:.3f}', '')
        if _has(rtm, 'alpha'):
            fcm.add_row('alpha', f'{rtm.alpha:.3f}', '')
        if _has(rtm, 'thrust_from_alpha_lbf'):
            fcm.add_row('Thrust(alpha)', f'{rtm.thrust_from_alpha_lbf:.3f}', 'lbf')

    if fcm.row_count != 0:
        bottom_columns.append(Panel(
            fcm,
            title=f'[{t["primary"]}]Controller[/{t["primary"]}]',
            border_style=t['panel_border'],
        ),)

    bottom = Columns(bottom_columns)

    return Group(header_panel, top, bottom)


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
        f'  [{THEME["muted"]}]Live view — press Enter to return to menu[/{THEME["muted"]}]'
    )
    threading.Thread(target=_wait_for_enter, daemon=True).start()

    try:
        with Live(_build_status_renderable(), refresh_per_second=5, screen=False) as live:
            while not stop.is_set():
                time.sleep(0.2)
                live.update(_build_status_renderable())
    except KeyboardInterrupt:
        pass


_TOOLBAR_STATE_TAGS = {
    'STATE_UNKNOWN': ('ansiwhite', 'UNKNOWN'),
    'STATE_IDLE': ('ansigreen', 'IDLE'),
    'STATE_CALIBRATE_VALVE': ('ansimagenta', 'CAL_VALVE'),
    'STATE_VALVE_PRIMED': ('ansicyan', 'VLV_PRIMED'),
    'STATE_VALVE_SEQ': ('ansiyellow', 'VLV_SEQ'),
    'STATE_THRUST_PRIMED': ('ansicyan', 'THR_PRIMED'),
    'STATE_THRUST_SEQ': ('ansiyellow', 'THR_SEQ'),
    'STATE_ABORT': ('ansired', 'ABORT'),
}


def get_toolbar():
    """Compact live telemetry for the prompt_toolkit bottom toolbar."""
    with packet_lock:
        pkt = latest_packet

    if pkt is None:
        return HTML(' <b>📡</b> No telemetry yet...')

    state_name = clover_pb2.SystemState.Name(pkt.state)
    tag, short = _TOOLBAR_STATE_TAGS.get(state_name, ('ansiwhite', state_name))

    is_abort = state_name == 'STATE_ABORT'
    abort_html = '  │  <ansired><b>🛑 ABORT</b></ansired>' if is_abort else ''

    sensors = pkt.analog_sensors
    sensor_parts = []
    for field, label in [('tc101', 'TC-101'), ('tc102', 'TC-102'), ('pt101', 'PT-101')]:
        if _has_msg_field(sensors, field):
            val = getattr(sensors, field)
            if abs(val) > 1e-6:
                sensor_parts.append(f'{label}: {val:.1f}')
    sensor_html = ('  │  ' + '  '.join(sensor_parts)) if sensor_parts else ''

    cmd_html = ''
    if _has(pkt, 'main_propeller_command'):
        pwm = max(1000, min(2000, pkt.main_propeller_command))
        throttle_pct = (pwm - 1000) / 10.0
        cmd_html = f'  │  throttle={throttle_pct:.2f}%'
    else:
        ranger_parts = []
        if _has(pkt, 'fuel_valve_command'):
            ranger_parts.append(f'🟡 tgt={pkt.fuel_valve_command.target_deg:.1f}°')
        if _has(pkt, 'lox_valve_command'):
            ranger_parts.append(f'🔴 tgt={pkt.lox_valve_command.target_deg:.1f}°')
        if ranger_parts:
            cmd_html = '  │  ' + '  '.join(ranger_parts)

    position_html = ''
    attitude_html = ''
    if _has_msg_field(pkt, 'estimated_state'):
        pos = pkt.estimated_state.position
        position_html = f'  │  pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})m'

        euler = pkt.estimated_state.euler
        attitude_html = f'  │  ypr=({euler.x:.1f}, {euler.y:.1f}, {euler.z:.1f})°'

    return HTML(
        f' 📡 <{tag}><b>{short}</b></{tag}>'
        f'  │  t={pkt.time_ns / 1e9:.2f}s  seq#{pkt.sequence_number}'
        + cmd_html
        + position_html
        + attitude_html
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
            raise ConnectionError('Connection closed while reading response length')
        byte = b[0]
        length |= (byte & 0x7F) << shift
        if not (byte & 0x80):
            break
        shift += 7

    data = b''
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk:
            raise ConnectionError('Connection closed while reading response body')
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
                    f'\n  {THEME["icon_warn"]} [{THEME["warning"]}]Connection lost, reconnecting...[/{THEME["warning"]}]'
                )
                try:
                    sock = _make_tcp_socket()
                except Exception as re:
                    console.print(
                        f'\n  {THEME["icon_warn"]} [{THEME["danger"]}]Reconnect failed: {re}[/{THEME["danger"]}]\n'
                    )
                    return False
            else:
                console.print(
                    f'\n  {THEME["icon_warn"]} [{THEME["danger"]}]Failed to send {label}: {e}[/{THEME["danger"]}]\n'
                )
                return False

    try:
        resp = _recv_response()
        if resp.HasField('err'):
            console.print(
                f'\n  {THEME["icon_warn"]} [{THEME["danger"]}]{label} rejected: {resp.err}[/{THEME["danger"]}]\n'
            )
            return False
        console.print(
            f'\n  {THEME["icon_ok"]} [{THEME["success"]}]Sent {label} → {ZEPHYR_IP}:{ZEPHYR_PORT}[/{THEME["success"]}]\n'
        )
        return True
    except Exception as e:
        console.print(
            f'\n  {THEME["icon_warn"]} [{THEME["warning"]}]{label} sent but no response: {e}[/{THEME["warning"]}]\n'
        )
        return True


# comment out to test in terminal without real data
# def send_request(req, label):
#     console.print(f"\n  {THEME['icon_ok']} [bold green]Would send the following: {label}[/bold green]\n")
#     return True


# ── Command implementations ──────────────────────────────────────────────────


def cmd_configure_analog_sensors():
    """Configure analog sensors."""
    # cfg1 = clover_pb2.AnalogSensorConfig()
    # cfg1.channel = 0
    # cfg1.assignment = clover_pb2.TC102
    # cfg1.tc_type = clover_pb2.K_TYPE
    cfg1 = clover_pb2.AnalogSensorConfig()
    cfg1.channel = 3
    cfg1.assignment = clover_pb2.PT006
    cfg1.pt_range_psig = 2000
    cfg1.pt_bias_psig = -35

    cfg2 = clover_pb2.AnalogSensorConfig()
    cfg2.channel = 4
    cfg2.assignment = clover_pb2.PT103
    cfg2.pt_range_psig = 1000
    cfg2.pt_bias_psig = -20

    cfg3 = clover_pb2.AnalogSensorConfig()
    cfg3.channel = 2
    cfg3.assignment = clover_pb2.PT004
    cfg3.pt_range_psig = 2000
    cfg3.pt_bias_psig = -40

    cfg4 = clover_pb2.AnalogSensorConfig()
    cfg4.channel = 6
    cfg4.assignment = clover_pb2.PT203
    cfg4.pt_range_psig = 3000
    cfg4.pt_bias_psig = -43

    cfg5 = clover_pb2.AnalogSensorConfig()
    cfg5.channel = 5
    cfg5.assignment = clover_pb2.PTF401
    cfg5.pt_range_psig = 1000
    cfg5.pt_bias_psig = -18

    cfg6 = clover_pb2.AnalogSensorConfig()
    cfg6.channel = 1
    cfg6.assignment = clover_pb2.PTO401
    cfg6.pt_range_psig = 1000
    cfg6.pt_bias_psig = -15

    cfg7 = clover_pb2.AnalogSensorConfig()
    cfg7.channel = 0
    cfg7.assignment = clover_pb2.PTC401
    cfg7.pt_range_psig = 1000
    cfg7.pt_bias_psig = -15

    cfg8 = clover_pb2.AnalogSensorConfig()
    cfg8.channel = 7
    cfg8.assignment = clover_pb2.PTC402
    cfg8.pt_range_psig = 1000
    cfg8.pt_bias_psig = -13.8

    req = clover_pb2.Request()
    req.configure_analog_sensors.configs.extend([cfg1, cfg2, cfg3, cfg4, cfg5, cfg6, cfg7, cfg8])
    send_request(req, 'CONFIGURE_ANALOG_SENSORS')


def cmd_subscribe_data_stream():
    """Subscribe to telemetry data stream."""
    req = clover_pb2.Request()
    req.subscribe_data_stream.SetInParent()
    send_request(req, 'SUBSCRIBE_DATA_STREAM')


def cmd_identify_client():
    """Identify client as GNC."""
    t = THEME
    console.print(f'\n  {t["icon_id"]} [{t["primary"]}]Client identity: GNC[/{t["primary"]}]')
    req = clover_pb2.Request()
    req.identify_client.client = clover_pb2.GNC
    send_request(req, 'IDENTIFY_CLIENT (GNC)')


def cmd_is_not_aborted():
    """PROTO CHANGE: new request — check that system is not in ABORT state."""
    req = clover_pb2.Request()
    req.is_not_aborted_request.SetInParent()
    send_request(req, 'IS_NOT_ABORTED')


def cmd_reset_valve_position():
    """Reset a valve to a specified degree position."""
    t = THEME
    console.print(f'\n  {t["icon_valve"]} [{t["primary"]}]Reset Valve Position[/{t["primary"]}]')
    console.print('    [1] FUEL')
    console.print('    [2] LOX')

    choice = Prompt.ask('  Select valve', choices=['1', '2'])
    valve = clover_pb2.FUEL if choice == '1' else clover_pb2.LOX
    valve_name = 'FUEL' if choice == '1' else 'LOX'
    pos = FloatPrompt.ask(f'  New position for {valve_name} valve (degrees)')

    req = clover_pb2.Request()
    req.throttle_reset_valve_position.valve = valve
    req.throttle_reset_valve_position.new_pos_deg = pos
    send_request(req, f'RESET_VALVE_POSITION ({valve_name} → {pos:.2f}°)')


def cmd_power_on_valve():
    """PROTO CHANGE: new request — power on (enable stepper motor) for a valve."""
    t = THEME
    console.print(f'\n  [{t["primary"]}]Power ON Valve[/{t["primary"]}]')
    console.print('    [1] FUEL')
    console.print('    [2] LOX')

    choice = Prompt.ask('  Select valve', choices=['1', '2'])
    valve = clover_pb2.FUEL if choice == '1' else clover_pb2.LOX
    valve_name = 'FUEL' if choice == '1' else 'LOX'

    req = clover_pb2.Request()
    req.throttle_power_on.valve = valve
    send_request(req, f'POWER_ON_VALVE ({valve_name})')


def cmd_power_off_valve():
    """Power off (disable stepper motor) for a valve."""
    t = THEME
    console.print(f'\n  [{t["primary"]}]Power OFF Valve[/{t["primary"]}]')
    console.print('    [1] FUEL')
    console.print('    [2] LOX')

    choice = Prompt.ask('  Select valve', choices=['1', '2'])
    valve = clover_pb2.FUEL if choice == '1' else clover_pb2.LOX
    valve_name = 'FUEL' if choice == '1' else 'LOX'

    req = clover_pb2.Request()
    req.throttle_power_off.valve = valve
    send_request(req, f'POWER_OFF_VALVE ({valve_name})')


def cmd_abort():
    """Abort active sequence into safe state."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  {t["icon_stop"]} [{t["danger"]}]ABORT — stop active sequence immediately?[/{t["danger"]}]',
        default=True,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.abort.SetInParent()
    send_request(req, 'ABORT')


def cmd_halt():
    """Halt all actuators."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  {t["icon_stop"]} [{t["danger"]}]HALT — stop all actuators immediately?[/{t["danger"]}]',
        default=True,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.halt.SetInParent()
    send_request(req, 'HALT')


def cmd_unprime():
    """Unprime (VALVE_PRIMED/THRUST_PRIMED → IDLE)."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  [{t["warning"]}]UNPRIME — cancel loaded sequence and return to IDLE?[/{t["warning"]}]',
        default=False,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.unprime.SetInParent()
    send_request(req, 'UNPRIME')


def cmd_calibrate_throttle_valve():
    """Enter throttle valve calibration mode (IDLE → CALIBRATE_THROTTLE_VALVE)."""
    t = THEME
    console.print(
        f'\n  {t["icon_valve"]} [{t["primary"]}]Calibrate Throttle Valve[/{t["primary"]}]'
    )
    console.print('    [1] FUEL')
    console.print('    [2] LOX')

    choice = Prompt.ask('  Select valve', choices=['1', '2'])
    valve = clover_pb2.FUEL if choice == '1' else clover_pb2.LOX
    valve_name = 'FUEL' if choice == '1' else 'LOX'

    req = clover_pb2.Request()
    req.calibrate_throttle_valve.valve = valve
    send_request(req, f'CALIBRATE_THROTTLE_VALVE ({valve_name})')


def _list_saved_sequences(subdir: pathlib.Path) -> list[pathlib.Path]:
    if not subdir.exists():
        return []
    return sorted(subdir.glob('*.textproto'))


def _pick_and_load_sequence(subdir: pathlib.Path, msg_class):
    """Display saved sequences, prompt user to pick one, return parsed proto."""
    files = _list_saved_sequences(subdir)
    t = THEME
    table = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t['primary'],
        border_style=t['panel_border'],
        padding=(0, 2),
    )
    table.add_column('#', style='bold white', no_wrap=True)
    table.add_column('Name', style='white')
    for i, f in enumerate(files, 1):
        table.add_row(str(i), f.stem)
    console.print(table)

    choice = Prompt.ask('  Select', choices=[str(i) for i in range(1, len(files) + 1)])
    chosen = files[int(choice) - 1]
    msg = msg_class()
    text_format.Parse(chosen.read_text(), msg)
    console.print(f'  [{t["success"]}]Loaded: {chosen.stem}[/{t["success"]}]')
    return msg


def _save_sequence(subdir: pathlib.Path, prefix: str, msg):
    """Prompt for a name and save a proto message as a .textproto file."""
    t = THEME
    subdir.mkdir(parents=True, exist_ok=True)
    name = Prompt.ask('  Sequence name')
    filename = subdir / f'{prefix}_{name}.textproto'
    filename.write_text(text_format.MessageToString(msg))
    console.print(f'  [{t["success"]}]Saved → {filename}[/{t["success"]}]')


def _f32(v: float) -> float:
    """Round a Python float to float32 precision (matches firmware storage/arithmetic)."""
    return struct.unpack('f', struct.pack('f', v))[0]


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
        pos_str = f'{cursor_val:.2f}' if cursor_val is not None else '?'
        console.print(
            f'\n  [{t["primary"]}]─── Add a segment (t={cursor_ms} ms, pos={pos_str}) ───[/{t["primary"]}]'
        )
        length_ms = IntPrompt.ask('    Segment length (ms)')

        console.print('    Segment type:')
        console.print('      [1] Linear  (ramp from value A → B)')
        console.print('      [2] Sine    (oscillation)')
        seg_type = Prompt.ask('    Choose', choices=['1', '2'], default='1')

        seg = trace.segments.add()
        seg.start_ms = cursor_ms
        seg.length_ms = length_ms
        cursor_ms += length_ms

        if seg_type == '1':
            if cursor_val is not None:
                console.print(
                    f'    [{t["muted"]}]Start value locked to {cursor_val:.2f}[/{t["muted"]}]'
                )
                start_val = cursor_val
            else:
                start_val = FloatPrompt.ask('    Start value')
            end_val = FloatPrompt.ask('    End value')
            seg.linear.start_val = start_val
            seg.linear.end_val = end_val
            cursor_val = end_val
            console.print(
                f'    [{t["success"]}]Linear segment: {start_val} → {end_val}[/{t["success"]}]'
            )
        else:
            amplitude = FloatPrompt.ask('    Amplitude')
            period = FloatPrompt.ask('    Period (ms)')
            phase_deg = FloatPrompt.ask('    Phase (degrees)', default=0.0)
            phase_rad = math.radians(phase_deg)
            if cursor_val is not None:
                offset = cursor_val - amplitude * math.sin(phase_rad)
                console.print(
                    f'    [{t["muted"]}]Offset auto-set to {offset:.4f} for continuity[/{t["muted"]}]'
                )
            else:
                offset = FloatPrompt.ask('    Offset (baseline)')
            end_val = _sine_sample_f32(offset, amplitude, period, phase_deg, length_ms)
            seg.sine.offset = offset
            seg.sine.amplitude = amplitude
            seg.sine.period = period
            seg.sine.phase_deg = phase_deg
            cursor_val = end_val
            console.print(
                f'    [{t["success"]}]Sine segment: offset={offset:.4f}, amp={amplitude}, T={period}ms → ends at {end_val:.2f}[/{t["success"]}]'
            )

        another = Confirm.ask('  Add another segment?', default=False)
        if not another:
            break

    trace.total_time_ms = cursor_ms
    console.print(f'  [{THEME["muted"]}]Total trace duration: {cursor_ms} ms[/{THEME["muted"]}]')
    return trace


def cmd_load_throttle_valve_sequence():
    """Load a throttle valve sequence (IDLE → THROTTLE_VALVE_PRIMED). At least one trace required."""
    t = THEME
    console.print(
        f'\n  {t["icon_seq"]} [{t["primary"]}]Load Throttle Valve Sequence[/{t["primary"]}]'
    )

    saved = _list_saved_sequences(VALVE_SEQ_DIR)
    if saved and Confirm.ask('  Load a saved sequence?', default=True):
        loaded = _pick_and_load_sequence(VALVE_SEQ_DIR, clover_pb2.LoadThrottleValveSequenceRequest)
        req = clover_pb2.Request()
        req.load_throttle_valve_sequence.CopyFrom(loaded)
        send_request(req, 'LOAD_THROTTLE_VALVE_SEQUENCE')
        return

    console.print(
        f'  [{t["muted"]}]Define a control trace for FUEL, LOX, or both. At least one required.[/{t["muted"]}]'
    )

    req = clover_pb2.Request()

    do_fuel = Confirm.ask('  Configure FUEL trace (degrees)?', default=True)
    if do_fuel:
        console.print(f'\n  {t["icon_fuel"]} [{t["warning"]}]FUEL trace setup:[/{t["warning"]}]')
        fuel_trace = _build_control_trace()
        req.load_throttle_valve_sequence.fuel_trace_deg.CopyFrom(fuel_trace)

    do_lox = Confirm.ask('\n  Configure LOX trace (degrees)?', default=True)
    if do_lox:
        console.print(f'\n  {t["icon_lox"]} [{t["info"]}]LOX trace setup:[/{t["info"]}]')
        lox_trace = _build_control_trace()
        req.load_throttle_valve_sequence.lox_trace_deg.CopyFrom(lox_trace)

    if not do_fuel and not do_lox:
        console.print(
            f'  [{t["danger"]}]At least one trace is required — nothing sent.[/{t["danger"]}]'
        )
        return

    if Confirm.ask('  Save this sequence for later?', default=False):
        _save_sequence(VALVE_SEQ_DIR, 'v', req.load_throttle_valve_sequence)

    send_request(req, 'LOAD_THROTTLE_VALVE_SEQUENCE')


def cmd_start_throttle_valve_sequence():
    """Start throttle valve sequence (THROTTLE_VALVE_PRIMED → THROTTLE_VALVE)."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  {t["icon_fire"]} [{t["warning"]}]START THROTTLE VALVE SEQUENCE — are you sure?[/{t["warning"]}]',
        default=False,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.start_throttle_valve_sequence.SetInParent()
    send_request(req, 'START_THROTTLE_VALVE_SEQUENCE')


def cmd_load_throttle_sequence():
    """Load a throttle sequence (IDLE → THROTTLE_PRIMED). Thrust trace is in lbf."""
    t = THEME
    console.print(f'\n  {t["icon_loop"]} [{t["primary"]}]Load Throttle Sequence[/{t["primary"]}]')

    saved = _list_saved_sequences(THRUST_SEQ_DIR)
    if saved and Confirm.ask('  Load a saved sequence?', default=True):
        loaded = _pick_and_load_sequence(THRUST_SEQ_DIR, clover_pb2.LoadThrottleSequenceRequest)
        req = clover_pb2.Request()
        req.load_throttle_sequence.CopyFrom(loaded)
        send_request(req, 'LOAD_THROTTLE_SEQUENCE')
        return
    console.print(f'  [{t["muted"]}]Thrust trace values are in lbf.[/{t["muted"]}]')

    thrust_trace = _build_control_trace()

    req = clover_pb2.Request()
    req.load_throttle_sequence.thrust_lbf.CopyFrom(thrust_trace)

    if Confirm.ask('  Save this sequence for later?', default=False):
        _save_sequence(THRUST_SEQ_DIR, 't', req.load_throttle_sequence)

    send_request(req, 'LOAD_THROTTLE_SEQUENCE')


def cmd_start_throttle_sequence():
    """Start throttle sequence (THROTTLE_PRIMED → THROTTLE)."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  {t["icon_fire"]} [{t["warning"]}]START THROTTLE SEQUENCE — are you sure?[/{t["warning"]}]',
        default=False,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.start_throttle_sequence.SetInParent()
    send_request(req, 'START_THROTTLE_SEQUENCE')


def cmd_configure_flight_controller_gains():
    """Configure flight controller PID gains."""
    t = THEME
    console.print(f'\n  [{t["primary"]}]Configure Flight Controller Gains[/{t["primary"]}]')
    console.print(
        f'  [{t["muted"]}]Leave blank to keep current value for any field.[/{t["muted"]}]'
    )

    # Mapping of user-friendly names to prefixes
    pid_map = {
        'xtilt': 'pidXTilt',
        'ytilt': 'pidYTilt',
        'x': 'pidX',
        'y': 'pidY',
        'z': 'pidZ',
        'zvel': 'pidZVelocity',
    }

    req = clover_pb2.Request()
    gains = req.configure_flight_controller_gains
    configured_pids = set()

    while True:
        console.print(f'\n  [{t["primary"]}]Available PIDs:[/{t["primary"]}]')
        for alias in sorted(pid_map.keys()):
            prefix = pid_map[alias]
            status = f'[green]✓ configured[/green]' if prefix in configured_pids else ''
            console.print(f'    {alias:<12} → {prefix:<20} {status}')
        console.print(f'    {"done":<12} → finish configuration')

        pid_choice = Prompt.ask('  Select PID to configure').strip().lower()

        if pid_choice == 'done':
            break

        if pid_choice not in pid_map:
            console.print(
                f'  [{t["warning"]}]Unknown PID "{pid_choice}". Try again.[/{t["warning"]}]'
            )
            continue

        prefix = pid_map[pid_choice]
        console.print(f'\n  [{t["primary"]}]Configuring {prefix}:[/{t["primary"]}]')

        kp = Prompt.ask(f'    {prefix}_kp', default='')
        ki = Prompt.ask(f'    {prefix}_ki', default='')
        kd = Prompt.ask(f'    {prefix}_kd', default='')

        if kp:
            setattr(gains, f'{prefix}_kp', float(kp))
        if ki:
            setattr(gains, f'{prefix}_ki', float(ki))
        if kd:
            setattr(gains, f'{prefix}_kd', float(kd))

        configured_pids.add(prefix)

        # Offer optional limits/integral/deriv per PID
        if Confirm.ask(f'  Set advanced options for {prefix}?', default=False):
            if Confirm.ask(f'    Set output limits?', default=False):
                mn = Prompt.ask(f'      {prefix}_min_out', default='')
                mx = Prompt.ask(f'      {prefix}_max_out', default='')
                if mn:
                    setattr(gains, f'{prefix}_min_out', float(mn))
                if mx:
                    setattr(gains, f'{prefix}_max_out', float(mx))

            if Confirm.ask(f'    Set integral limits?', default=False):
                mn = Prompt.ask(f'      {prefix}_min_integral', default='')
                mx = Prompt.ask(f'      {prefix}_max_integral', default='')
                if mn:
                    setattr(gains, f'{prefix}_min_integral', float(mn))
                if mx:
                    setattr(gains, f'{prefix}_max_integral', float(mx))

            if Confirm.ask(f'    Set integral zone?', default=False):
                val = Prompt.ask(f'      {prefix}_integral_zone', default='')
                if val:
                    setattr(gains, f'{prefix}_integral_zone', float(val))

            if Confirm.ask(f'    Set derivative low-pass filter (Hz)?', default=False):
                val = Prompt.ask(f'      {prefix}_deriv_lp_hz', default='')
                if val:
                    setattr(gains, f'{prefix}_deriv_lp_hz', float(val))

        if not Confirm.ask('  Configure another PID?', default=False):
            break

    if configured_pids:
        console.print(
            f'\n  [{t["success"]}]Configured: {", ".join(sorted(configured_pids))}[/{t["success"]}]'
        )
        send_request(req, 'CONFIGURE_FLIGHT_CONTROLLER_GAINS')
    else:
        console.print(f'  [{t["muted"]}]No gains configured.[/{t["muted"]}]')


# TODO: is this all that's needed?
def cmd_calibrate_tvc():
    """Enter TVC calibration mode (IDLE → CALIBRATE_TVC)."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  [{t["warning"]}]CALIBRATE TVC — enter calibration mode?[/{t["warning"]}]',
        default=False,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.calibrate_tvc.SetInParent()
    send_request(req, 'CALIBRATE_TVC')


# TODO: test if this works
def cmd_load_tvc_sequence():
    """Load a TVC sequence (IDLE → TVC_PRIMED)."""
    t = THEME
    console.print(f'\n  {t["icon_seq"]} [{t["primary"]}]Load TVC Sequence[/{t["primary"]}]')

    saved = _list_saved_sequences(TVC_SEQ_DIR)
    if saved and Confirm.ask('  Load a saved sequence?', default=True):
        loaded = _pick_and_load_sequence(TVC_SEQ_DIR, clover_pb2.LoadTvcSequenceRequest)
        req = clover_pb2.Request()
        req.load_tvc_sequence.CopyFrom(loaded)
        send_request(req, 'LOAD_TVC_SEQUENCE')
        return

    console.print(f'  [{t["muted"]}]Define pitch and yaw traces (degrees).[/{t["muted"]}]')
    req = clover_pb2.Request()

    console.print(f'\n  [{t["primary"]}]Pitch trace:[/{t["primary"]}]')
    req.load_tvc_sequence.pitch_trace_deg.CopyFrom(_build_control_trace())

    console.print(f'\n  [{t["primary"]}]Yaw trace:[/{t["primary"]}]')
    req.load_tvc_sequence.yaw_trace_deg.CopyFrom(_build_control_trace())

    if Confirm.ask('  Save this sequence for later?', default=False):
        _save_sequence(TVC_SEQ_DIR, 'tvc', req.load_tvc_sequence)

    send_request(req, 'LOAD_TVC_SEQUENCE')


def cmd_start_tvc_sequence():
    """Start TVC sequence (TVC_PRIMED → TVC)."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  {t["icon_fire"]} [{t["warning"]}]START TVC SEQUENCE — are you sure?[/{t["warning"]}]',
        default=False,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.start_tvc_sequence.SetInParent()
    send_request(req, 'START_TVC_SEQUENCE')


# TODO: This is not a linear or sin trace
def cmd_load_rcs_valve_sequence():
    """Load an RCS valve sequence (IDLE → RCmenuS_VALVE_PRIMED)."""
    t = THEME
    console.print(f'\n  {t["icon_seq"]} [{t["primary"]}]Load RCS Valve Sequence[/{t["primary"]}]')

    saved = _list_saved_sequences(RCS_VALVE_SEQ_DIR)
    if saved and Confirm.ask('  Load a saved sequence?', default=True):
        loaded = _pick_and_load_sequence(RCS_VALVE_SEQ_DIR, clover_pb2.LoadRcsValveSequenceRequest)
        req = clover_pb2.Request()
        req.load_rcs_valve_sequence.CopyFrom(loaded)
        send_request(req, 'LOAD_RCS_VALVE_SEQUENCE')
        return

    console.print(f'  [{t["muted"]}]Define CW and CCW valve traces (0=off, 1=on).[/{t["muted"]}]')
    req = clover_pb2.Request()

    console.print(f'\n  [{t["primary"]}]CW valve trace:[/{t["primary"]}]')
    req.load_rcs_valve_sequence.rcs_cw_valve_trace.CopyFrom(_build_control_trace())

    console.print(f'\n  [{t["primary"]}]CCW valve trace:[/{t["primary"]}]')
    req.load_rcs_valve_sequence.rcs_ccw_valve_trace.CopyFrom(_build_control_trace())

    if Confirm.ask('  Save this sequence for later?', default=False):
        _save_sequence(RCS_VALVE_SEQ_DIR, 'rcsv', req.load_rcs_valve_sequence)

    send_request(req, 'LOAD_RCS_VALVE_SEQUENCE')


def cmd_start_rcs_valve_sequence():
    """Start RCS valve sequence (RCS_VALVE_PRIMED → RCS_VALVE)."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  {t["icon_fire"]} [{t["warning"]}]START RCS VALVE SEQUENCE — are you sure?[/{t["warning"]}]',
        default=False,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.start_rcs_valve_sequence.SetInParent()
    send_request(req, 'START_RCS_VALVE_SEQUENCE')


def cmd_load_rcs_sequence():
    """Load an RCS roll sequence (IDLE → RCS_PRIMED)."""
    t = THEME
    console.print(f'\n  {t["icon_seq"]} [{t["primary"]}]Load RCS Sequence[/{t["primary"]}]')

    saved = _list_saved_sequences(RCS_SEQ_DIR)
    if saved and Confirm.ask('  Load a saved sequence?', default=True):
        loaded = _pick_and_load_sequence(RCS_SEQ_DIR, clover_pb2.LoadRcsSequenceRequest)
        req = clover_pb2.Request()
        req.load_rcs_sequence.CopyFrom(loaded)
        send_request(req, 'LOAD_RCS_SEQUENCE')
        return

    console.print(f'  [{t["muted"]}]Define roll trace (degrees).[/{t["muted"]}]')
    req = clover_pb2.Request()
    req.load_rcs_sequence.trace_deg.CopyFrom(_build_control_trace())

    if Confirm.ask('  Save this sequence for later?', default=False):
        _save_sequence(RCS_SEQ_DIR, 'rcs', req.load_rcs_sequence)

    send_request(req, 'LOAD_RCS_SEQUENCE')


def cmd_start_rcs_sequence():
    """Start RCS sequence (RCS_PRIMED → RCS)."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  {t["icon_fire"]} [{t["warning"]}]START RCS SEQUENCE — are you sure?[/{t["warning"]}]',
        default=False,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.start_rcs_sequence.SetInParent()
    send_request(req, 'START_RCS_SEQUENCE')


# TODO: test if this works
def cmd_load_static_fire_sequence():
    """Load a static fire sequence (IDLE → STATIC_FIRE_PRIMED)."""
    t = THEME
    console.print(f'\n  {t["icon_seq"]} [{t["primary"]}]Load Static Fire Sequence[/{t["primary"]}]')

    saved = _list_saved_sequences(STATIC_FIRE_SEQ_DIR)
    if saved and Confirm.ask('  Load a saved sequence?', default=True):
        loaded = _pick_and_load_sequence(
            STATIC_FIRE_SEQ_DIR, clover_pb2.LoadStaticFireSequenceRequest
        )
        req = clover_pb2.Request()
        req.load_static_fire_sequence.CopyFrom(loaded)
        send_request(req, 'LOAD_STATIC_FIRE_SEQUENCE')
        return

    console.print(
        f'  [{t["muted"]}]Define thrust (lbf), pitch (deg), and yaw (deg) traces.[/{t["muted"]}]'
    )
    req = clover_pb2.Request()

    console.print(f'\n  [{t["primary"]}]Thrust trace (lbf):[/{t["primary"]}]')
    req.load_static_fire_sequence.thrust_lbf.CopyFrom(_build_control_trace())

    console.print(f'\n  [{t["primary"]}]Pitch trace (deg):[/{t["primary"]}]')
    req.load_static_fire_sequence.pitch_trace_deg.CopyFrom(_build_control_trace())

    console.print(f'\n  [{t["primary"]}]Yaw trace (deg):[/{t["primary"]}]')
    req.load_static_fire_sequence.yaw_trace_deg.CopyFrom(_build_control_trace())

    if Confirm.ask('  Save this sequence for later?', default=False):
        _save_sequence(STATIC_FIRE_SEQ_DIR, 'sf', req.load_static_fire_sequence)

    send_request(req, 'LOAD_STATIC_FIRE_SEQUENCE')


def cmd_start_static_fire_sequence():
    """Start static fire sequence (STATIC_FIRE_PRIMED → STATIC_FIRE)."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  {t["icon_fire"]} [{t["danger"]}]START STATIC FIRE SEQUENCE — are you sure?[/{t["danger"]}]',
        default=False,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.start_static_fire_sequence.SetInParent()
    send_request(req, 'START_STATIC_FIRE_SEQUENCE')


# TODO: test if this works
def cmd_load_flight_sequence():
    """Load a flight sequence (IDLE → FLIGHT_PRIMED)."""
    t = THEME
    console.print(f'\n  {t["icon_seq"]} [{t["primary"]}]Load Flight Sequence[/{t["primary"]}]')

    saved = _list_saved_sequences(FLIGHT_SEQ_DIR)
    if saved and Confirm.ask('  Load a saved sequence?', default=True):
        loaded = _pick_and_load_sequence(FLIGHT_SEQ_DIR, clover_pb2.LoadFlightSequenceRequest)
        req = clover_pb2.Request()
        req.load_flight_sequence.CopyFrom(loaded)
        send_request(req, 'LOAD_FLIGHT_SEQUENCE')
        return

    console.print(
        f'  [{t["muted"]}]Define X (m), Y (m), Z (m), and roll (deg) traces.[/{t["muted"]}]'
    )
    req = clover_pb2.Request()

    console.print(f'\n  [{t["primary"]}]X position trace (m):[/{t["primary"]}]')
    req.load_flight_sequence.x_position_trace_m.CopyFrom(_build_control_trace())

    console.print(f'\n  [{t["primary"]}]Y position trace (m):[/{t["primary"]}]')
    req.load_flight_sequence.y_position_trace_m.CopyFrom(_build_control_trace())

    console.print(f'\n  [{t["primary"]}]Z position trace (m):[/{t["primary"]}]')
    req.load_flight_sequence.z_position_trace_m.CopyFrom(_build_control_trace())

    console.print(f'\n  [{t["primary"]}]Roll angle trace (deg):[/{t["primary"]}]')
    req.load_flight_sequence.roll_angle_trace_deg.CopyFrom(_build_control_trace())

    if Confirm.ask('  Save this sequence for later?', default=False):
        _save_sequence(FLIGHT_SEQ_DIR, 'flt', req.load_flight_sequence)

    send_request(req, 'LOAD_FLIGHT_SEQUENCE')


def cmd_start_flight_sequence():
    """Start flight sequence (FLIGHT_PRIMED → FLIGHT)."""
    t = THEME
    confirmed = Confirm.ask(
        f'\n  {t["icon_fire"]} [{t["danger"]}]START FLIGHT SEQUENCE — are you sure?[/{t["danger"]}]',
        default=False,
    )
    if not confirmed:
        console.print(f'  [{t["muted"]}]Cancelled.[/{t["muted"]}]')
        return
    req = clover_pb2.Request()
    req.start_flight_sequence.SetInParent()
    send_request(req, 'START_FLIGHT_SEQUENCE')


# ── Menu ─────────────────────────────────────────────────────────────────────

MENU_ITEMS = [
    ('sub', 'subscribe', 'Subscribe to data stream', cmd_subscribe_data_stream),
    ('id', 'identify', 'Identify this client (GNC)', cmd_identify_client),
    ('check', 'check', 'Check system is not aborted', cmd_is_not_aborted),
    ('cfgana', 'cfgana', 'Configure analog sensors', cmd_configure_analog_sensors),
    ('reset', 'reset', 'Reset throttle valve position', cmd_reset_valve_position),
    ('pon', 'poweron', 'Power ON stepper motor', cmd_power_on_valve),
    ('poff', 'poweroff', 'Power OFF stepper motor', cmd_power_off_valve),
    ('gains', 'gains', 'Configure flight controller gains', cmd_configure_flight_controller_gains),
    (
        'cal',
        'calibrate',
        'Calibrate throttle valve  (IDLE → CALIBRATE_THROTTLE_VALVE)',
        cmd_calibrate_throttle_valve,
    ),
    (
        'lvseq',
        'loadvalve',
        'Load throttle valve sequence  (IDLE → THROTTLE_VALVE_PRIMED)',
        cmd_load_throttle_valve_sequence,
    ),
    (
        'svseq',
        'startvalve',
        'Start throttle valve sequence (THROTTLE_VALVE_PRIMED → THROTTLE_VALVE)',
        cmd_start_throttle_valve_sequence,
    ),
    (
        'ltseq',
        'loadthrust',
        'Load throttle sequence  (IDLE → THROTTLE_PRIMED)',
        cmd_load_throttle_sequence,
    ),
    (
        'stseq',
        'startthrust',
        'Start throttle sequence (THROTTLE_PRIMED → THROTTLE)',
        cmd_start_throttle_sequence,
    ),
    ('caltvc', 'caltvc', 'Calibrate TVC  (IDLE → CALIBRATE_TVC)', cmd_calibrate_tvc),
    ('ltvcseq', 'loadtvc', 'Load TVC sequence  (IDLE → TVC_PRIMED)', cmd_load_tvc_sequence),
    ('stvcseq', 'starttvc', 'Start TVC sequence (TVC_PRIMED → TVC)', cmd_start_tvc_sequence),
    (
        'lrvseq',
        'loadrcsvalve',
        'Load RCS valve sequence  (IDLE → RCS_VALVE_PRIMED)',
        cmd_load_rcs_valve_sequence,
    ),
    (
        'srvseq',
        'startrcsvalve',
        'Start RCS valve sequence (RCS_VALVE_PRIMED → RCS_VALVE)',
        cmd_start_rcs_valve_sequence,
    ),
    ('lrseq', 'loadrcs', 'Load RCS sequence  (IDLE → RCS_PRIMED)', cmd_load_rcs_sequence),
    ('srseq', 'startrcs', 'Start RCS sequence (RCS_PRIMED → RCS)', cmd_start_rcs_sequence),
    (
        'lsfseq',
        'loadstaticfire',
        'Load static fire sequence  (IDLE → STATIC_FIRE_PRIMED)',
        cmd_load_static_fire_sequence,
    ),
    (
        'ssfseq',
        'startstaticfire',
        'Start static fire sequence (STATIC_FIRE_PRIMED → STATIC_FIRE)',
        cmd_start_static_fire_sequence,
    ),
    (
        'lfseq',
        'loadflight',
        'Load flight sequence  (IDLE → FLIGHT_PRIMED)',
        cmd_load_flight_sequence,
    ),
    (
        'sfseq',
        'startflight',
        'Start flight sequence (FLIGHT_PRIMED → FLIGHT)',
        cmd_start_flight_sequence,
    ),
    ('unprime', 'unprime', 'Unprime  (any PRIMED → IDLE)', cmd_unprime),
    ('halt', 'halt', 'HALT active sequence → IDLE', cmd_halt),
    ('abort', 'abort', 'ABORT → safe state', cmd_abort),
    ('status', 'status', 'Live telemetry dashboard (Enter to exit)', None),
    ('quit', 'quit', 'Exit', None),
]


def print_menu():
    t = THEME
    console.print()
    console.rule(f'[{t["header"]}] {t["icon_fire"]} CLOVER CLI {t["icon_fire"]} [/{t["header"]}]')
    console.print()

    table = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=t['primary'],
        border_style=t['panel_border'],
        padding=(0, 2),
    )
    table.add_column('CMD', style='bold white', no_wrap=True)
    table.add_column('Description', style='white')

    icons = {
        'sub': THEME['icon_live'],
        'id': THEME['icon_id'],
        'check': '🔍',
        'bias': '⚙️ ',
        'reset': THEME['icon_valve'],
        'pon': '🟢',
        'poff': '⚫',
        'gains': '📊',
        'cal': '📐',
        'lvseq': THEME['icon_seq'],
        'svseq': THEME['icon_fire'],
        'ltseq': THEME['icon_loop'],
        'stseq': THEME['icon_fire'],
        'caltvc': '📐',
        'ltvcseq': THEME['icon_seq'],
        'stvcseq': THEME['icon_fire'],
        'lrvseq': THEME['icon_seq'],
        'srvseq': THEME['icon_fire'],
        'lrseq': THEME['icon_seq'],
        'srseq': THEME['icon_fire'],
        'lsfseq': THEME['icon_seq'],
        'ssfseq': THEME['icon_fire'],
        'lfseq': THEME['icon_seq'],
        'sfseq': THEME['icon_fire'],
        'unprime': '↩️ ',
        'halt': THEME['icon_stop'],
        'abort': THEME['icon_stop'],
        'status': THEME['icon_live'],
        'quit': THEME['icon_quit'],
    }

    for key, alias, desc, _ in MENU_ITEMS:
        table.add_row(f'{icons.get(key, "")} {key}', desc)

    console.print(table)


def route_command(cmd: str) -> bool:
    """Route a typed command. Returns False if user wants to quit."""
    cmd = cmd.strip().lower()

    if cmd in ('quit', 'exit', 'q'):
        console.print(
            f'\n  {THEME["icon_stop"]} [{THEME["warning"]}]Exiting.[/{THEME["warning"]}]\n'
        )
        return False

    if cmd in ('status', 's'):
        cmd_live_status()
        return True

    if cmd in ('help', 'h', 'menu', '?', ''):
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

    parser = argparse.ArgumentParser(description='Clover ground station CLI')
    parser.add_argument(
        '--no-data',
        action='store_true',
        help='Skip binding the UDP telemetry port (for second instances)',
    )
    args = parser.parse_args()

    telemetry_info = f'Listen port: {DATA_PORT}' if not args.no_data else 'Telemetry: disabled'
    console.print(
        Panel.fit(
            f'[{t["header"]}] {t["icon_fire"]}  CLOVER CLI INFO  {t["icon_fire"]} [/{t["header"]}]\n'
            f'[{t["muted"]}]Target: {ZEPHYR_IP}:{ZEPHYR_PORT}  |  {telemetry_info}[/{t["muted"]}]',
            border_style=t['panel_border'],
            padding=(1, 4),
        )
    )

    if not args.no_data:
        data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        data_sock.bind((DATA_IP, DATA_PORT))
        console.print(f'\n  [{t["info"]}]Auto-subscribing to data stream...[/{t["info"]}]')
        cmd_subscribe_data_stream()
        threading.Thread(target=listen_for_telemetry, daemon=True).start()
        threading.Thread(target=_flush_loop, daemon=True).start()

    print_menu()

    session = PromptSession(bottom_toolbar=get_toolbar, refresh_interval=0.1)

    while True:
        try:
            cmd = session.prompt('\n  CMD> ')
        except (KeyboardInterrupt, EOFError):
            console.print(f'\n  {t["icon_stop"]} [{t["warning"]}]Exiting.[/{t["warning"]}]\n')
            break

        if not route_command(cmd):
            break

    # Write CSV on exit — filename uses timestamp at exit: raw_sensors_YYYYMMDD_HHMMSS.csv
    if not args.no_data:
        _write_csv_on_exit()


if __name__ == '__main__':
    main()
