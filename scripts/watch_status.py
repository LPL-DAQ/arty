import socket
import datetime
import time
from rich.live import Live
from rich.text import Text
from rich.table import Table
from rich.console import Group
from rich.panel import Panel
from rich import box

IP = '192.168.0.150'
PORT = 19690

print(f'Connecting to {IP}:{PORT}')


def generate_output(refresh_in, data):
    connections_table = Table.grid(padding=(0, 3), pad_edge=True)
    connections_table.add_column('Connection')
    connections_table.add_column('Status')
    connections_table.add_row('Refresh in', f'{refresh_in} sec')
    connections_table.add_row('Command', 'DISCONNECTED' if data[
                                                               'command_ping_elapsed'] == '-1' else f'Last pinged {float(data["command_ping_elapsed"]) / 1000} sec ago')
    connections_table.add_row('DAQ', 'DISCONNECTED' if data[
                                                           'daq_ping_elapsed'] == '-1' else f'Last pinged {float(data["daq_ping_elapsed"]) / 1000} sec ago')
    connections_table.add_row('Data recipient', 'PRIMED' if data['data_recipient_primed'] == '1' else 'NOT READY')

    fuel_valve_table = Table.grid(padding=(0, 3), pad_edge=True)
    fuel_valve_table.add_column('Value')
    fuel_valve_table.add_row("Internal position", f'{data["fuel_internal"]} deg')
    fuel_valve_table.add_row("Encoder position", f'{data["fuel_encoder"]} deg')

    lox_valve_table = Table.grid(padding=(0, 3), pad_edge=True)
    lox_valve_table.add_column('Sensor')
    lox_valve_table.add_column('Value')
    lox_valve_table.add_row("Internal position", f'{data["lox_internal"]} deg')
    lox_valve_table.add_row("Encoder position", f'{data["lox_encoder"]} deg')

    pts_table = Table.grid(padding=(0, 3), pad_edge=True)
    pts_table.add_column('Sensor')
    pts_table.add_column('Value')
    for key in data:
        if key.startswith('pt'):
            pts_table.add_row(key.upper(), f'{data[key]} psig')

    table = Table.grid(padding=1, pad_edge=True)
    table.add_column('Category', no_wrap=True, justify='center', style='bold red')
    table.add_column('What')

    table.add_row('Connections', connections_table)
    table.add_row('Fuel valve', fuel_valve_table)
    table.add_row('LOx valve', lox_valve_table)
    table.add_row('PTs', pts_table)

    return table


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.connect((IP, PORT))
    print('Socket OK, identifying as status client...')
    sock.sendall('name;status#'.encode())

    buf = ''
    while True:
        buf += sock.recv(1).decode()
        if buf[-1] == '\n':
            print(f'Message: {buf[:-1]}')
            buf = ''
            break

    print('Connected.')
    sock.sendall('status#'.encode())
    buf = ''
    while True:
        buf += sock.recv(1).decode()
        if buf[-16:] == '>>STATUS DONE<<\n':
            payload = buf[:-16]
            break

    with Live(Text('Fetching status'), refresh_per_second=10) as live:

        while True:
            sock.sendall('status#'.encode())
            buf = ''
            while True:
                buf += sock.recv(1).decode()
                if buf[-16:] == '>>STATUS DONE<<\n':
                    payload = buf[:-16]
                    break

            datapoints = {}
            for line in payload.splitlines():
                [key, val] = line.split('=')
                datapoints[key] = val

            for i in range(5):
                live.update(generate_output(round(0.5 - i / 10, 1), datapoints))
                time.sleep(0.1)
