"""Shared valve readback display for the dashboard and named-sequence views."""
import json
from pathlib import Path
from rich.table import Table

MAP_PATH = Path(__file__).resolve().parents[1] / 'tests/valve_gpio/valve_map.json'


class ClientValveCommands:
    """Acknowledged manual commands for one connection, independent of readback."""

    def __init__(self):
        self.connection = None
        self.commands = {}

    def for_connection(self, connection):
        if connection is not self.connection:
            self.connection = connection
            self.commands = {}
        return self.commands

    def record(self, connection, channel, powered):
        self.for_connection(connection)[channel] = powered


def valve_rows(info, mapping=None):
    if mapping is None:
        mapping = json.loads(MAP_PATH.read_text())
    readback = (info is not None and info.HasField('powered_channels')
                and info.HasField('known_channels'))
    for name, channel in mapping.items():
        state = 'UNKNOWN'
        if channel is None:
            state = 'UNASSIGNED'
        elif readback and info.known_channels & (1 << channel):
            state = 'ON' if info.powered_channels & (1 << channel) else 'OFF'
        yield name, str(channel) if channel is not None else '—', state


def valve_table(info, client_commands=None):
    table = Table(title='Valve GPIO readback (not mechanical position)')
    table.add_column('Valve')
    table.add_column('Channel')
    table.add_column('Power')
    table.add_column('Sequence')
    table.add_column('Override')
    table.add_column('This client')
    table.caption = ('Override takes priority over Sequence. This client shows last accepted commands. '
                     'GPIO readback is not mechanical position.')
    try:
        for name, channel, state in valve_rows(info):
            command = (client_commands or {}).get(int(channel)) if channel != '—' else None
            indicator = 'Requested ON' if command is True else 'Requested OFF' if command is False else '—'
            bit = 1 << int(channel) if channel != '—' else 0
            sequence = override = '—'
            if bit and info is not None:
                if info.HasField('sequence_requested_channels'):
                    sequence = 'ON' if info.sequence_requested_channels & bit else 'OFF'
                if (info.HasField('manual_override_channels')
                        and info.manual_override_channels & bit):
                    override = 'ON' if info.manual_powered_channels & bit else 'OFF'
            table.add_row(name, channel, state, sequence, override, indicator, style={'ON': 'green', 'OFF': 'dim'}.get(state, 'yellow'))
    except (OSError, ValueError):
        table.add_row('Valve map unavailable', '—', 'UNKNOWN', '—', '—', '—')
    return table
