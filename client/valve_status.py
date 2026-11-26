"""Shared valve readback display for the dashboard and named-sequence views."""
import json
from rich.table import Table
from rich import box

from app_paths import VALVE_MAP_PATH as MAP_PATH


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


def valve_table(info, client_commands=None, theme=None):
    theme = theme or {'primary': 'bold gold1', 'panel_border': 'dark_red', 'muted': 'dim white'}
    table = Table(
        box=box.SIMPLE_HEAD,
        show_header=True,
        header_style=theme['primary'],
        border_style=theme['panel_border'],
        padding=(0, 1),
        pad_edge=False,
    )
    table.add_column('Valve', style='bold white', no_wrap=True)
    table.add_column('Channel', style=theme['muted'], justify='right', no_wrap=True)
    table.add_column('Power', style='white', justify='right', no_wrap=True)
    table.add_column('Control', style='white', no_wrap=True)
    try:
        for name, channel, state in valve_rows(info):
            bit = 1 << int(channel) if channel != '—' else 0
            indicator = 'UNKNOWN' if bit else '—'
            if bit and info is not None:
                if info.HasField('manual_override_channels') and info.manual_override_channels & bit:
                    indicator = 'MANUAL'
                elif info.HasField('sequence_requested_channels') and info.sequence_requested_channels & bit:
                    indicator = 'SEQUENCE'
                elif info.HasField('manual_override_channels') and info.HasField('sequence_requested_channels'):
                    indicator = '—'
            table.add_row(name, channel, state, indicator, style={'ON': 'green', 'OFF': 'dim'}.get(state, 'yellow'))
    except (OSError, ValueError):
        table.add_row('Valve map unavailable', '—', 'UNKNOWN', '—')
    return table
